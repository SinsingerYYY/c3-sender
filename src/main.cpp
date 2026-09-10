#include <Arduino.h>
#include <WiFi.h>
#include "ESP32_NOW_Serial.h"   // 官方 IDF5/core3 原生 ESP-NOW Serial

// ============================================================================
// ESP32-C3  RM 协议帧感知桥 (UART0 GPIO20/21 <-> ESP-NOW, 官方 ESP_NOW_Serial)
//
// 规则：
//   - 按 RM 帧头 SOF(0xA5)+data_length 切帧，整帧长 = 9 + data_length
//   - 不校验 CRC（有效性由应用端识别）
//   - 所有长度都转发（实际不会超过 MAX_FRAME）
//   - 发送侧：整帧一次 NowSerial.write；发不出去(缓冲满)则丢弃该帧，不缓存、无 ACK
//   - 接收侧：同样按 RM 帧切分，收满整帧后一次写 UART 输出
//
// 负载预期：300B 数据帧@50Hz、75B 数据帧@35Hz（非同时）
//
// 两块板同一份代码，只改 PEER_MAC 为对方：
//   无柱端(0F:40) -> PEER = 有柱 1F:E4
//   有柱端(1F:E4) -> PEER = 无柱 0F:40
// ============================================================================

// // === 本份固件用于【无柱板（本机 0F:40）】，对端=有柱 1F:E4 ===
// static const uint8_t PEER_MAC[6] = {
//     0x70, 0xAF, 0x09, 0x3B, 0x1F, 0xE4
// };
//烧有柱板(receiver 1F:E4)时改用：
static const uint8_t PEER_MAC[6] = {
    0x70, 0xAF, 0x09, 0x3B, 0x0F, 0x40
};

static const int8_t CHANNEL = 1;
static const uint32_t UART_BAUD = 921600;      // 业务串口波特率
static const uint8_t UART_RX_PIN = 20;         // ESP32 RX <- 主板 TX
static const uint8_t UART_TX_PIN = 21;         // ESP32 TX -> 主板 RX
static const uint8_t LED_PIN = 8;

static const size_t RX_BUF_SIZE = 16384;       // NowSerial 接收缓冲(字节)
static const size_t TX_BUF_SIZE = 16384;       // NowSerial 发送缓冲(字节)

// ---- RM 帧参数 ----
static const uint8_t RM_SOF = 0xA5;
static const size_t RM_HEADER_LEN = 5;         // SOF + data_length(2) + seq + CRC8
static const size_t RM_OVERHEAD = 9;           // header5 + cmd_id2 + tail2
static const size_t MAX_FRAME = 1470;          // 不超过 ESP-NOW v2 单消息上限

// 调试输出总开关：0=全部关闭（编译期裁剪），1=打开启动日志/统计/逐帧 hex
// 921600 下必须为 0，否则 USB 跟不上会溢出
#define ENABLE_DEBUG_PRINTS 0

// 保留上电 5 秒后输出本机 MAC 的操作
#define SEND_MAC_AFTER_BOOT 1
static const uint32_t SEND_MAC_DELAY_MS = 5000;

// LED: 收到任一方向帧后 5 秒内 1Hz；空闲每 5 秒短闪一次
static const uint32_t LED_ACTIVE_MS = 5000;
static const uint32_t LED_HALF_1HZ = 500;
static const uint32_t IDLE_PERIOD_MS = 5000;
static const uint32_t IDLE_PULSE_MS = 200;

static uint32_t lastActivityMs = 0;
static bool hasActivity = false;
static uint8_t selfMac[6] = {};

// 初始化失败不退出：标记状态后在 loop 中重试
static bool nowReady = false;
static uint32_t lastNowRetryMs = 0;
static const uint32_t NOW_RETRY_INTERVAL_MS = 2000;

// 官方单 peer Serial over ESP-NOW（STA 接口，固定对端 MAC）
ESP_NOW_Serial_Class NowSerial(PEER_MAC, CHANNEL, WIFI_IF_STA);

// ============================ RM 帧解析器 ===================================

struct RmFrameParser
{
    uint8_t buf[MAX_FRAME];
    size_t idx = 0;
    size_t need = 0;
    uint8_t state = 0;          // 0=找SOF 1=收头 2=收body

    void reset()
    {
        idx = 0;
        need = 0;
        state = 0;
    }

    // 返回完整帧长度(>0)表示 buf 中已有一整帧；否则 0
    size_t feed(uint8_t b)
    {
        if (state == 0)
        {
            if (b == RM_SOF)
            {
                buf[0] = b;
                idx = 1;
                state = 1;
            }
            return 0;
        }

        if (state == 1)
        {
            buf[idx++] = b;
            if (idx == RM_HEADER_LEN)
            {
                const uint16_t dataLength = static_cast<uint16_t>(buf[1]) |
                                            (static_cast<uint16_t>(buf[2]) << 8);
                const size_t total = RM_OVERHEAD + dataLength;
                if (total < RM_OVERHEAD || total > MAX_FRAME)
                {
                    reset();                 // 长度非法：丢弃并重同步
                    return 0;
                }
                need = total;
                state = 2;
            }
            return 0;
        }

        // state == 2
        buf[idx++] = b;
        if (idx == need)
        {
            return need;                     // 完整帧，调用方消费后需 reset()
        }
        return 0;
    }
};

static RmFrameParser txParser;   // UART -> 无线
static RmFrameParser rxParser;   // 无线 -> UART

// ============================ 统计 ==========================================
static uint32_t statTxFrames = 0;
static uint32_t statTxDropped = 0;
static uint32_t statRxFrames = 0;
static uint32_t statTxBytes = 0;
static uint32_t statRxBytes = 0;

// ============================ 辅助 ==========================================

static void setLed(bool on)
{
    digitalWrite(LED_PIN, on ? LOW : HIGH);   // C3 SuperMini 低电平点亮
}

static void markActivity()
{
    lastActivityMs = millis();
    hasActivity = true;
}

static void printMac(const uint8_t mac[6])
{
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

#if ENABLE_DEBUG_PRINTS
static void printHex(const char *tag, const uint8_t *data, size_t len)
{
    Serial.printf("%s (%u B): ", tag, static_cast<unsigned>(len));
    for (size_t i = 0; i < len; ++i)
    {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
}

static void printStartup()
{
    WiFi.macAddress(selfMac);
    Serial.println();
    Serial.println("===== RM frame-aware bridge (ESP_NOW_Serial) =====");
    Serial.print("Local MAC : ");
    printMac(selfMac);
    Serial.println();
    Serial.print("Peer MAC  : ");
    printMac(PEER_MAC);
    Serial.printf("\nChannel   : %d\n", CHANNEL);
    Serial.printf("UART      : TX=GPIO%u RX=GPIO%u @%lu 8N1\n",
                  UART_TX_PIN, UART_RX_PIN, static_cast<unsigned long>(UART_BAUD));
    Serial.printf("ESP-NOW v%d, maxDataLen=%d\n",
                  ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());
    Serial.printf("RM frame  : SOF=0x%02X, total=9+data_length, max=%u\n",
                  RM_SOF, static_cast<unsigned>(MAX_FRAME));
    Serial.println("=================================================");
}
#endif

static void sendMacAfterBoot()
{
#if SEND_MAC_AFTER_BOOT
    if (millis() < SEND_MAC_DELAY_MS)
    {
        return;
    }

    static bool sent = false;
    if (sent)
    {
        return;
    }
    sent = true;
    Serial.print("MAC: ");
    printMac(selfMac);
    Serial.println();
#endif
}

static void updateLed()
{
    const uint32_t now = millis();
    const bool active = hasActivity && (now - lastActivityMs < LED_ACTIVE_MS);
    static uint32_t timer = 0;
    static bool ledOn = false;
    if (active)
    {
        if (now - timer >= LED_HALF_1HZ)
        {
            timer = now;
            ledOn = !ledOn;
            setLed(ledOn);
        }
        return;
    }
    if (!ledOn && now - timer >= IDLE_PERIOD_MS)
    {
        timer = now;
        ledOn = true;
        setLed(true);
    }
    else if (ledOn && now - timer >= IDLE_PULSE_MS)
    {
        ledOn = false;
        setLed(false);
    }
}

// ============================ setup / loop ==================================

// 尝试初始化 ESP-NOW Serial；失败只提示并返回 false，由 loop 继续重试
static bool initNowSerial()
{
    NowSerial.setRxBufferSize(RX_BUF_SIZE);
    NowSerial.setTxBufferSize(TX_BUF_SIZE);

    if (!NowSerial.begin())
    {
        Serial.println("[err] NowSerial.begin failed, retry later");
        nowReady = false;
        return false;
    }

    nowReady = true;
#if ENABLE_DEBUG_PRINTS
    printStartup();
#endif
    return true;
}

void setup()
{
    Serial.begin(115200);                      // USB 日志
    Serial0.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    pinMode(LED_PIN, OUTPUT);
    setLed(false);

    // WiFi 到固定信道（STA，不连接 AP，仅供 ESP-NOW）
    WiFi.mode(WIFI_STA);
    uint32_t wifiWaitStart = millis();
    while (!WiFi.STA.started())
    {
        if (millis() - wifiWaitStart >= 3000)
        {
            Serial.println("[err] WiFi STA not started, continue anyway");
            break;
        }
        delay(10);
    }
    WiFi.setChannel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(100);

    WiFi.macAddress(selfMac);

    initNowSerial();
}

void loop()
{
    updateLed();
    sendMacAfterBoot();

    // ESP-NOW 未就绪：定时重试，不阻塞其他逻辑
    if (!nowReady)
    {
        const uint32_t now = millis();
        if (now - lastNowRetryMs >= NOW_RETRY_INTERVAL_MS)
        {
            lastNowRetryMs = now;
            initNowSerial();
        }
        delay(1);
        return;
    }

    bool hadWork = false;

    // ---- UART(GPIO20/21) -> 无线：解析整帧，整帧一次发送；发不出则丢 ----
    while (Serial0.available() > 0)
    {
        const uint8_t b = static_cast<uint8_t>(Serial0.read());
        const size_t frameLen = txParser.feed(b);
        if (frameLen == 0)
        {
            continue;
        }

        // 得到一整帧
        hadWork = true;
        markActivity();
#if ENABLE_DEBUG_PRINTS
        printHex("[TX] UART frame", txParser.buf, frameLen);
#endif

        const int space = NowSerial.availableForWrite();
        size_t written = 0;
        if (space >= static_cast<int>(frameLen))
        {
            written = NowSerial.write(txParser.buf, frameLen);
        }

        if (written == frameLen)
        {
            ++statTxFrames;
            statTxBytes += frameLen;
        }
        else
        {
            ++statTxDropped;          // 缓冲不足：丢弃整帧，不缓存、无 ACK
        }

        txParser.reset();
    }

    // ---- 无线 -> UART：解析整帧，整帧一次输出 ----
    while (NowSerial.available() > 0)
    {
        const uint8_t b = static_cast<uint8_t>(NowSerial.read());
        const size_t frameLen = rxParser.feed(b);
        if (frameLen == 0)
        {
            continue;
        }

        hadWork = true;
        markActivity();
#if ENABLE_DEBUG_PRINTS
        printHex("[RX] wire frame", rxParser.buf, frameLen);
#endif

        Serial0.write(rxParser.buf, frameLen);
        ++statRxFrames;
        statRxBytes += frameLen;
        rxParser.reset();
    }

    // ---- 每秒统计（低频，避免干扰） ----
#if ENABLE_DEBUG_PRINTS
    static uint32_t lastStat = 0;
    if (millis() - lastStat >= 1000)
    {
        lastStat = millis();
        Serial.printf("[stat] TX frames=%lu drop=%lu bytes=%lu | RX frames=%lu bytes=%lu\n",
                      static_cast<unsigned long>(statTxFrames),
                      static_cast<unsigned long>(statTxDropped),
                      static_cast<unsigned long>(statTxBytes),
                      static_cast<unsigned long>(statRxFrames),
                      static_cast<unsigned long>(statRxBytes));
    }
#endif

    if (!hadWork)
    {
        delay(1);
    }
}

# C3-sender-IDF5

ESP32-C3 上的 **UART ↔ ESP-NOW 双向桥**，按 RoboMaster 通信协议整帧转发。

- 平台：PlatformIO + Arduino framework（Arduino core3 / ESP-IDF 5）
- 无线：ESP-NOW v2，点对点，固定对端 MAC
- 业务串口：UART0，GPIO20/21，`921600` 8N1
- 协议：RoboMaster 2026 V2.0.0，帧长 = `9 + data_length`，最大 `1470 B`

## 功能

- UART 收到一整帧后，通过 ESP-NOW 一次性发送。
- ESP-NOW 收到一整帧后，通过 UART 一次性输出。
- 以 `SOF = 0xA5` 切帧；不校验 CRC8 / CRC16（有效性由应用端识别）。
- 非法字节或非法长度丢弃并重同步，不输出残帧。
- 不缓存、不重传：发送缓冲不足时丢弃整帧并计数。

## 硬件与接线

| ESP32-C3 | 方向 | 主板 |
| --- | --- | --- |
| GPIO20 | RX ← | 主板 TX |
| GPIO21 | TX → | 主板 RX |
| GPIO8 | — | 板载 LED（低电平点亮） |
| USB | — | 仅日志，不承载业务数据 |

## 配置

两块板烧录**同一份代码**，只需把 `PEER_MAC` 改为对方，切勿指向自己。

| 板 | 本机 MAC | `PEER_MAC` |
| --- | --- | --- |
| 无柱（发送端） | `70:AF:09:3B:0F:40` | `70:AF:09:3B:1F:E4` |
| 有柱（接收端） | `70:AF:09:3B:1F:E4` | `70:AF:09:3B:0F:40` |

主要常量位于 `src/main.cpp` 头部：

```cpp
PEER_MAC          // 对端 MAC
CHANNEL = 1       // ESP-NOW 信道（两端必须一致）
UART_BAUD = 921600
UART_RX_PIN = 20
UART_TX_PIN = 21
MAX_FRAME = 1470
```

## 调试与日志

| 开关 | 默认 | 说明 |
| --- | --- | --- |
| `ENABLE_DEBUG_PRINTS` | `0` | 总开关。关闭时启动日志、每秒统计、逐帧 hex 全部编译期裁剪 |
| `SEND_MAC_AFTER_BOOT` | `1` | 上电 5 秒后输出一次本机 MAC（USB，115200） |

- 错误提示 `[err] ...` 始终输出，不会直接退出；初始化失败会在 `loop()` 中每 2 秒重试。
- LED 指示：
  - 收到任一方向帧后 5 秒内 → 1Hz 闪烁；
  - 空闲（5 秒无数据）→ 每 5 秒短闪一次。

## 构建与烧录

```bash
cd /home/singer/esp/C3-sender-IDF5
~/.platformio/penv/bin/platformio run -e esp32-c3-devkitm-1
~/.platformio/penv/bin/platformio run -e esp32-c3-devkitm-1 -t upload
```

## 目录结构

```text
.
├── platformio.ini        # PlatformIO 平台/环境配置
├── src/main.cpp          # 全部业务代码
├── lib/EspNowBus/        # 供应商化的 ESP-NOW 总线库（当前未使用）
├── 项目需求说明.md        # 详细需求
└── RoboMaster ...md      # 协议原文
```

> 当前实现使用官方 `ESP32_NOW_Serial`，`lib/EspNowBus` 为历史保留，不参与编译主流程。

## 常见问题

- **`RE-SEND_MAX[5]` / `drop` 持续增长、`RX frames=0`**
  多为对端未上电、MAC 配置错误或信道不一致。请确认两端 MAC 互指、`CHANNEL` 相同。
- **`TX frames` 增长不代表无线发送成功**
  它只表示数据写入了 ESP-NOW Serial 缓冲；链路失败会表现为重发与丢帧计数。

# UWB 系统接线与上板验收

本文档对应以下实现：

- 固定端：LinkPG 主基站 A + ESP32-CAM 网关；
- 车载端：LinkPG 标签 1 + FPGA；USB 直读确认该模块的协议 ID 为 `1`；
- 标签 0/2/3：与车载标签 1 一起用于四角定位；
- 基站 B/C/D：只参与 UWB 定位，不接 FPGA 或 ESP32。

所有 UART 都使用 `115200 bit/s, 8N1`。UART 必须交叉连接：发送端
`TXD` 接接收端 `RXD`，并且必须共地。

## 1. 接线前电平检查

LinkPG 模块本体使用 5 V 供电，但资料没有明确保证 TTL 串口一定是
3.3 V 电平。FPGA 和 ESP32 GPIO 都只能按 3.3 V 电平使用。

1. 先只给 LinkPG 上电，不接 ESP32/FPGA UART。
2. 万用表测 LinkPG `TXD` 对 `GND` 的空闲电压。
3. 空闲电压约 3.3 V时可以直连。
4. 如果超过 3.6 V，LinkPG `TXD -> ESP32/FPGA RX` 必须经过 5 V 到
   3.3 V 电平转换器；不要直接接入 FPGA 或 ESP32。

## 2. 主基站 A 与 ESP32-CAM

LinkPG 排针定义：1=`5V`，2=`GND`，3=`DIO`，4=`RXD`，5=`CLK`，
6=`TXD`，7=`GND`，8=`BOOT0`。

| LinkPG 主基站 A | ESP32-CAM | 方向 | 用途 |
| --- | --- | --- | --- |
| 1 `5V` | 稳定 5 V 电源 | 电源 | 给 LinkPG 供电 |
| 2 或 7 `GND` | `GND` | 电源 | LinkPG、ESP32 和电源共地 |
| 6 `TXD` | `GPIO13 / U2RXD` | A -> ESP32 | 定位帧和透传帧 |
| 4 `RXD` | `GPIO14 / U2TXD` | ESP32 -> A | 启停定位和下发命令 |
| 3 `DIO` | 不接 | - | SWD 调试口 |
| 5 `CLK` | 不接 | - | SWD 调试口 |
| 8 `BOOT0` | 不接 | - | 保持正常启动模式 |

ESP32-CAM 可以继续由当前 USB 下载板供电。若 LinkPG 由另一套 5 V 电源
供电，只连接两边的 `GND`，不要把两个独立电源的 5 V 输出互相短接。
当前固件占用 GPIO13/14，因此不能同时使用 ESP32-CAM 的 microSD 卡。

最终运行时不要让电脑上位机和 ESP32 同时向同一 LinkPG 串口发送命令。
如果保留 LinkPG USB 仅作供电，也应先确认该硬件批次的 USB 串口不会
自动发送配置命令；首次验收建议拔掉 LinkPG USB，使用独立 5 V 供电。

## 3. 车载标签 1 与 FPGA

FPGA 新增接口位于 H4：

| LinkPG 标签 1 | FPGA H4 | FPGA 管脚 | 方向 | 用途 |
| --- | --- | --- | --- | --- |
| 1 `5V` | 外部稳定 5 V | - | 电源 | 不从 FPGA 3.3 V GPIO 取电 |
| 2 或 7 `GND` | H4 的 `GND` | - | 电源 | 与小车、FPGA 共地 |
| 6 `TXD` | H4 pin 1 | `N14` | 标签 -> FPGA | `uwb_uart_rx` |
| 4 `RXD` | H4 pin 2 | `N13` | FPGA -> 标签 | `uwb_uart_tx` |
| 3 `DIO` | 不接 | - | - | SWD 调试口 |
| 5 `CLK` | 不接 | - | - | SWD 调试口 |
| 8 `BOOT0` | 不接 | - | - | 保持正常启动模式 |

`N14` 已设置 `LVCMOS33 + PULLUP`；`N13` 已设置
`LVCMOS33 + DRIVE 8 + SLEW SLOW`。这两个脚没有占用现有 HDMI、蓝牙、
C5 底盘串口、RPLIDAR 或超声波接口。

## 4. 其余 UWB 模块

| 模块 | 必需连接 | 说明 |
| --- | --- | --- |
| 基站 B/C/D | 各自 5 V、GND | 不接 ESP32/FPGA；配置正确 ID、坐标、信道和速率 |
| 标签 0/2/3 | 各自 5 V、GND | 分别安装在另外三个车角；不接 UART |

四块标签已通过 USB 确认为协议 ID 0/1/2/3。当前为随机摆放，不具备
角点含义；最终安装后必须记录四角对应 ID，并同步修改服务器 `.env`
中的角色映射。四个基站必须使用
同一坐标系，所有模块必须使用相同信道和兼容速率。

## 5. 首次验收顺序

1. 只接主基站 A 与 ESP32 的两根 UART 和共地线，不接车载 FPGA。
2. ESP32 上电后访问 `https://relay.example.invalid/health`，确认
   `gateway_connected` 为 `true`。
3. 给基站 B/C/D 和四个标签供电。网关状态中的 `good_frames` 应持续
   增加，`crc_errors` 不应持续快速增加。
4. 关闭整车电源，再连接标签 1 到 FPGA 的 N14/N13 和共地线。
5. 在 Robei 中重新生成、综合、布局布线并烧录含
   `linkpg_uwb_bridge25` 的当前工程。
6. 从服务器或手机端向协议 ID 1（物理标签 1）发送 `?`。应先收到 UWB 命令 ACK，随后
   收到 FPGA 的最新 `@S` 状态帧。
7. 再测试 `P`，应收到重组后的 `@PONG`；最后才架空车轮测试
   `W/A/S/R/C`，松开按键必须发送 `C`。

若第 3 步有定位帧而第 6 步没有 ACK，优先检查车载标签 1 的 UART
交叉方向、共地和 FPGA 是否烧录了当前版本。若有 ACK 但没有 `@S`，
检查 FPGA 标签侧回传链路；若 `good_frames` 为零，先检查主基站是否已
进入连续定位模式以及基站/标签信道配置。

# UWB 集成验证记录（2026-08-04）

## FPGA / Robei

1. `tools/integrate_linkpg_uwb.py` 连续执行后结构计数保持不变：
   - `linkpg_uwb_bridge25` 实例：1；
   - 顶层 UWB 端口：2；
   - N14/N13 管脚约束：2；
   - `.proj` 模块注册：1。
2. 以下四个文件均通过 XML 解析：
   - `little_car/model/linkpg_uwb_bridge.model`；
   - `little_car/model/sys_route_gui_fsm_top.model`；
   - `little_car/const/top.const`；
   - `little_car/little_car.proj`。
3. 从最终 `.model` 反向生成 Verilog，再运行
   `tb_linkpg_uwb_bridge.v`，结果为：

```text
RESULT TEST_PASS linkpg_uwb_bridge
```

测试覆盖有效命令注入、命令 ACK、`@KQ` 遥测分片、重复序号抑制和
错误 Modbus CRC 丢弃。

4. 使用 Robei 随附 Yosys 对完整顶层执行：

```text
synth_xilinx -flatten -arch xc7 -abc9 -nodsp
```

结果：

```text
CHECK: 0 problems
Estimated LCs: 12841
CARRY4: 3019
FDCE/FDPE/FDRE: 3287 / 947 / 1434
LUT2/LUT3/LUT4/LUT5/LUT6: 5251 / 4132 / 2394 / 3852 / 2463
Elapsed: 334.3 s
```

完整顶层没有出现 ABC9 拓扑排序失败。上述统计是 Yosys 映射结果，不能
替代 Robei `Roplace` 的最终 `SLICE_LUTX`、布局布线时间和时序报告。
新增桥接模块单独综合时估算约 1,988 LCs；由于单独作为顶层时调试输出
不会被裁剪，这一数值是偏保守的上限。

5. 使用 Vivado 2025.2 对同一份 `filelist.vf` 和 `top.xdc` 执行 RTL
   综合检查，命令退出码为 0，`Missing instances` 为空。输出只有原工程
   已存在的属性重复、非完整 `case` 等告警，没有模块缺失或约束错误。

## ESP32-CAM

Arduino CLI 使用 `esp32:esp32:esp32cam` 重新编译成功：

```text
Flash: 1094924 / 3145728 bytes (34%)
Global RAM: 48044 / 327680 bytes (14%)
```

固件已烧录到当前连接的 ESP32-CAM，并已通过 Wi-Fi 连接服务器。本段
记录的是 TTL 接线前的阶段性结果；后文已经补充主基站 TTL 与四标签
二维定位的端到端实测。

## 服务器

本地协议、姿态融合和运行时测试：

```text
11 passed
```

服务已部署为 Raspberry Pi Docker 容器，并通过公网 TLS 反向代理暴露。
在 ESP32 在线时，健康接口能够显示网关已连接；拔掉或重启网关时该值
会随实际连接状态变化。

## 当时尚待实物确认

- 主基站 A TTL 接入 ESP32 后，`good_frames` 是否持续递增；
- 车载标签 1 TTL 接入 FPGA 后，命令 ACK 和 `@S/@L` 是否双向返回；
- Robei 实际布局布线和最终时序；
- 四个基站完成坐标标定后的四标签定位精度；
- 测量车体标签中心距后，服务器矩形刚性检查与航向输出。

## 首次 TTL 实测补充

主基站 A 接入 ESP32-CAM 后，连续采样得到：

```text
gateway_connected: true
good_frames: 1426 -> 1486 -> 1540
crc_errors: 0
queue_drops: 0
protocol_errors: 0
```

首次自动帧曾报告协议 ID 0，但当时无法证明它来自哪一块物理标签。随后
逐块通过 USB 直读，确认四块物理标签的协议 ID 为 0/1/2/3；服务器默认
命令目标保持为 1。标签改由 USB 稳定供电后，主基站恢复约 29 帧/秒
连续输出，四基站距离样本约为 139/360/349/301 cm，说明此前停帧位于
标签外部供电或启动状态一侧。

## 二维定位端到端实测

按 A 左上、B 右上、C 右下、D 左下布置，写入主基站的坐标为：

```text
A=(0,0,0) cm
B=(150,0,0) cm
C=(150,170,0) cm
D=(0,170,0) cm
```

配置脚本在写入前确认设备角色为主基站，写入后回读确认 HDS-TWR、二维
模式、A-D 使能、硬件解算、标签列表 0/1/2/3 和持续自动输出均正确。
直接从主基站 USB 采样 5 秒得到 219 个 CRC 正确的定位帧，四个标签均
为 `position_valid=1`，四基站测距成功掩码均为 `0x000F`。

恢复主基站 A 与 ESP32-CAM 的 TTL 接线后，公网服务器间隔 3 秒采样：

```text
position_frames: 3358 -> 3500 (delta 142)
crc_errors: 0
framing_errors: 0
rx_idle_ms: 12
tag_ids: 0,1,2,3
```

树莓派运行配置中遗留的右后标签 ID 4 已修正为 0。容器重建后四个标签
均进入融合输入，`pose.quality=unconfigured`；这是预期结果，因为标签
仍为随机摆放，尚未分配实际车体四角，也没有录入车长和车宽。此阶段
输出的临时航向角不能用于导航。

## 车载标签 UART 方向交换实验

为验证接口丝印是否表示“应连接的对端信号”，曾仅交换 UWB 两个 FPGA
逻辑端口的管脚约束，保持其他网表和现有杜邦线不变：

```text
实验固件：uwb_uart_rx=N13, uwb_uart_tx=N14
Roplace：LUT 60%, FF 8%, CARRY4 42%，路由冲突收敛到 0
时序：三组时钟均 PASS
服务器：P 已入队，uwb_tx 3 -> 4
12 秒观察：pass_frames 仍为 0，未收到 ACK 或 @PONG
```

该实验未打通透传。PG3.9 官方端口表明确排针 4 `RXD` 为模块 USART1
输入、排针 6 `TXD` 为模块 USART1 输出；继续保留交换约束可能形成
输出对输出连接。因此板上和源码都已恢复安全交叉映射：

```text
标签 pin 6 TXD -> H4 pin 1 / N14 / FPGA uwb_uart_rx
标签 pin 4 RXD <- H4 pin 2 / N13 / FPGA uwb_uart_tx
```

恢复固件 SHA-256：
`542D49D29537D60EA5CF770BA8D2BE8939131E258EFC2172FCDEEF427CB8D2A2`，
Vivado Hardware Manager 返回 `program_complete=1` 且 startup status 为 HIGH。
下一步应通过标签 1 USB 只读寄存器 112/113，确认标签串口输出使能和
Modbus 输出协议，而不是继续交换管脚。

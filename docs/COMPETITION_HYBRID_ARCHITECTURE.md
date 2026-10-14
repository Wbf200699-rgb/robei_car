# FPGA + Android 混合自主小车架构

## 1. 设计目标

当前版本以稳定性和可验证性优先，不再把大容量栅格地图、全局搜索和地图文件系统同时塞进 FPGA。系统分成硬实时车载层和高算力手机层，两层通过 CH9141K BLE 透明串口交换经过限流的状态和传感器摘要。

竞赛展示必须如实区分两层贡献：FPGA 展示并行传感器接口、确定性安全链、电机闭环修正和掉线保护；Android 展示高分辨率建图、全局定位融合、路径规划、地图保存和人机界面。

## 2. FPGA 硬实时层

FPGA 保留以下功能，任何手机卡顿或蓝牙断开都不能绕过这些安全条件：

- RPLIDAR C1、UWB Tag1、双 BMI088、PMW3901、VL53L1X、超声波采集。
- 雷达与超声波避障、矩形车体前角和侧面安全判断。
- C5 四轮 UART 帧生成、启动等待、停车强化、周期刷新和运动看门狗。
- 双 IMU 零偏/极性学习、短时航向积分、光流局部位移和 UWB 低频绝对位置修正。
- 蓝牙命令确认、心跳、状态帧原子发送和掉线停车。

FPGA 不再保存完整占用栅格，也不再执行射线投影、全局路径搜索或 SD 地图文件操作。`onboard_occupancy_map` 仅保留轻量位姿估计接口，旧地图和 SD 模块保留相同端口并输出安全空闲值，使现有 Robei 图形连线无需大改。

## 3. Android 高算力层

Android 负责：

- 将 72 点雷达圈投影到 241 x 241、25 mm/格的占用栅格。
- 使用 FPGA 的 UWB/IMU 位姿作为绝对约束，结合雷达扫描匹配做局部修正。
- 自动探索决策、全局路径规划、目标点交互和地图可视化。
- 将地图快照持久化到手机，并在下次连接后恢复。
- 自动标定流程的组织与结果评估；实际电机闭环和安全停止仍由 FPGA 执行。

## 4. 低带宽 BLE 协议

继续使用现有 ASCII 帧和单一 FPGA 发送调度器，避免临近比赛重写协议：

| 帧 | 内容 | 用途 |
|---|---|---|
| `@S` | 模式、路线、四轮参数、阻塞来源 | 控制状态与参数闭环 |
| `@L` | 8 扇区距离和避障状态 | 安全状态与快速 UI |
| `@R1` | 一圈分 6 块，每块 12 点 | 手机高分辨率建图 |
| `@M2` | 双 IMU、光流和本地校准状态 | 姿态诊断与标定 |
| `@G0` | UWB/IMU 融合位姿、标志和计数 | 手机绝对位姿约束 |
| `@V` | 5 路 VL53L1X | 近场辅助感知 |
| `@PONG` | 心跳回复 | 链路存活检测 |

`@G1` 地图块在混合架构中禁用。所有帧必须由同一个 UART 调度器完整发送，控制确认和停车优先于传感器上报。手机不得把 GATT 写成功等同于车辆动作成功，必须等待 `@K` 或新 `@S`。

## 5. 稳定性策略

- CH9141K 使用持久化透明从机配置，FPGA 复位后不再主动发送 AT+RESET，避免反复打断广播和连接。
- FPGA 在蓝牙失联、手动命令超时、传感器危险或复位时独立停车。
- 手机地图的暂停、保存和读取是本地操作；连接恢复后通过 `?` 请求最新状态和 `@G0` 位姿。
- 雷达圈带序号、分块号和 CRC；不完整圈丢弃，不能写入地图。
- 先通过模块仿真，再做 Vivado 综合/实现，最后用 Robei 生成和布线；硬件测试分别确认蓝牙、雷达、位姿和电机安全链。

## 6. 迭代顺序

1. 验证 BLE 心跳、控制确认和掉线停车。
2. 验证 RPLIDAR 完整圈与 `@G0` 位姿连续性。
3. 手机静止建图，确认墙体轮廓不随时间漂移。
4. 低速直行/原地转向校准，固化四轮 trim 与航向参数。
5. 开启自动探索，但 FPGA 始终保留最终避障和停车权。
6. 最后再加入目标点规划和地图复用，不在基础链路未稳定时叠加功能。
# Goal navigation and dynamic avoidance (2026-08-11)

The Android map now treats destination navigation as a closed-loop process:

1. A map tap is accepted only inside the convex hull of anchors A/B/C/D.
2. Occupied lidar points are merged by dilation, then inflated by the tag-rectangle
   half diagonal plus a 75 mm margin. Unknown cells and cells outside the anchor
   boundary are non-traversable.
3. Cardinal A* produces a chassis-executable path. The phone sends one short turn
   or forward action, stops, consumes the newest pose/map data, and plans again.
4. A lidar/ultrasonic safety trigger cancels a forward action immediately. The
   triggering sector is inserted into the map before replanning. If a detour exists,
   navigation resumes along it; if no detour exists, the vehicle remains stopped.
5. VL53L1X channels remain telemetry-only in this revision and do not affect the
   obstacle map or motor interlock.

The FPGA remains the final real-time motor safety authority. The current Android
executor uses the manual command channel, so BLE loss invokes the FPGA manual
watchdog and stops the vehicle. Continuing a complete autonomous route through a
BLE outage requires a later bulk-route preload interface into the FPGA route FSM;
it must not be implemented by disabling the manual watchdog.

Each phone map profile stores the 241x241 occupancy snapshot and the four anchor
coordinates together. Loading a profile restores the anchor coordinate frame before
the map. Anchor edits therefore remain local to the selected environment.

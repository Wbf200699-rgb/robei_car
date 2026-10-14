# 简历与面试表达参考

## 项目名称

**基于 FPGA 与 RobeiEDA 的智能小车控制及感知系统**

技术栈：Verilog / Artix-7 / RobeiEDA / UART / BLE / Android Java/XML / ESP32-S3 / FastAPI / WebSocket / Python。

项目荣誉：**集创赛全国二等奖**（项目作者提供）。求职目标：嵌入式软件工程师，关注 Linux、AI 与机器人。

## 面向求职的简历简介

> **FPGA 智能小车与感知控制系统｜集创赛全国二等奖**：基于 Artix-7、RobeiEDA 与 Verilog 搭建小车控制链，配套 Android BLE 控制端、ESP32-S3 UWB 网关及 Python/FastAPI 服务。项目覆盖 UART/BLE 协议解析、状态机与看门狗、雷达数据分段传输、位姿/地图参考算法和可复现验证。源码：https://github.com/Wbf200699-rgb/robei_car 。

这段介绍描述项目成果；个人负责部分请按实际分工补充。面向 Linux/机器人岗位，优先讲通信、时效性、故障恢复、数据处理与自动化测试，再以 FPGA 和 Android 说明完整系统的联调能力。

下面是按模块准备的候选表述。只选本人实际参与、能够展示代码并回答追问的部分；“设计/实现/负责/参与”的动词由真实分工决定，不能直接全选。

| 方向 | 可按实际贡献改写的表述 | 可展示材料 |
| --- | --- | --- |
| FPGA | 参与基于状态机的小车路线控制，实现/调试 C5 四轮串口控制、手动命令看门狗与避障停车链路 | `c5_route_controller.model`、`c5_motor_frame_driver.model`、对应 testbench |
| 感知通信 | 实现/调试 RPLIDAR UART 数据解析、八扇区距离与 72 角度栅格六段 CRC 传输 | `rplidar_c1_uart.model`、`lidar_map_sampler.model`、VCD 与日志 |
| 移动端 | 参与 Java/XML Android 控制端，完成 BLE 状态同步、路线/雷达视图与运动标定交互 | `android/car_app`、单元测试、本人设备录屏 |
| 网络定位 | 参与 ESP32-S3 UWB 网关与 WebSocket 服务，处理 CRC、消息重组、四标签位姿和数据过期判断 | `esp32/uwb_gateway`、`server/tests` |
| 验证工程 | 建立 Robei 模型与独立 Verilog 测试镜像，用紧凑 VCD 和 SHA-256 清单留存可复现证据 | `tools/run_hardware_parameter_tests.py`、波形清单、CI |

## 一段式介绍模板

> 基于 Artix-7 FPGA 与 RobeiEDA 搭建智能小车控制系统，配套 Android BLE 控制端及 UWB 网关/服务器。我主要负责【本人模块】，针对【具体问题】完成【可展示的实现】，通过【测试/波形/实物记录】验证【边界明确的结果】。项目源码和复现步骤见 https://github.com/Wbf200699-rgb/robei_car 。

## 面试讲解建议

1. 先画 BLE → FPGA → C5 控制链，明确每层职责。
2. 选一个自己解决的问题，说明触发条件、原因、改法和测试证据。
3. 展示一小段代码及其对应波形；解释时序、状态和数据单位。
4. 主动区分 RTL 仿真、算法回放和实际车辆测试。
5. 说明当前限制，例如实际外参、通信干扰、累计漂移和时钟约束。

## 避免的表述

- 未实测就写“厘米级定位”“零丢包”“安全可靠”“自动驾驶已落地”。
- 将九组 RTL 测试通过解释成九项实车功能全部交付。
- 将 VL53L1X 五路写成当前启用传感器。
- 把共享/团队源码、第三方库或 AI 辅助生成部分描述成独立原创。
- 填写未经确认的比赛、奖项、角色和个人投入时间。

作者已确认“集创赛全国二等奖”，因此可写项目荣誉；比赛完整名称、届次、赛道、个人分工和演示视频仍未单独提供，不作推断。

# 系统架构与源码阅读入口

## 控制与安全主链

Android 经 CH9141K 透明串口服务访问 FPGA，BLE 使用 FFF0 服务、FFF1 通知和 FFF2 写入。FPGA 负责命令解析、路线运行、避障判断与 C5 串口输出。手机显示状态不代表底盘已经执行动作，应结合新鲜的状态回传判断。

建议依次阅读：

| 入口 | 关注点 |
| --- | --- |
| `little_car/model/sys_route_gui_fsm_top.model` | 顶层接口、子模块与连接 |
| `little_car/model/bt_route_uart.model` | 单字节命令、遥测帧和发送调度 |
| `little_car/model/c5_route_controller.model` | 手动/自动动作、停车与轮速 |
| `little_car/model/c5_motor_frame_driver.model` | C5 电机串口帧、启动与刷新 |
| `little_car/model/rplidar_c1_uart.model` | 雷达启动、节点和扇区解析 |
| `little_car/model/lidar_map_sampler.model` | 72 个角度栅格、六段输出和 CRC |
| `little_car/model/ultrasonic_obstacle_guard.model` | 距离转换、超时和停车信号 |
| `little_car/model/gui_route_renderer.model` | HDMI 上的图形与状态输出 |

Robei `.model` 是设计源；`little_car/source_review/` 是导出的阅读副本，可能对应较早快照。重新生成源码请运行 `python tools/prepare_fpga_sources.py`，不要只修改生成的 `build/*.v`。

## Android 层

`android/car_app/app/src/main/java/car/car_app/`：

- `MainActivity.java`：页面与控制器协调，是当前主要维护热点。
- `bluetooth/`：BLE 生命周期、发送、通知和帧解析。
- `map/`、`mapping/`：路线、占用栅格与扫描匹配。
- `calibration/`：运动标定与估计公式。
- `geometry/`、`localization/`：几何配置、四标签刚体拟合与静止窗口过滤。
- `uwb/`：服务器连接、定位状态与服务地址设置。

实际估值须由标定参数、参考控制值和当前原始控制值共同计算。原始控制单位与毫米/秒不能直接互换；缺少标定时不得用实验示例数值代替。

## UWB 网络链

`esp32/uwb_gateway` 接入主基站 UART，处理 LinkPG 帧与恢复状态。`server/app` 负责消息重组、定位质量、数据时效和 WebSocket 广播。四标签位姿依赖实际安装几何、新鲜度和时间同步，不能把任意摆放的四标签坐标解释成车辆位姿。

UWB 相关远程命令接口保留用于工程调试，但 App 的车辆主控制链使用 BLE。协议细节见 [UWB_GATEWAY_PROTOCOL](UWB_GATEWAY_PROTOCOL.md) 和 [mobile_app_handoff](mobile_app_handoff.md)。

## 配置与派生产物

`config/vehicle_geometry.json` 生成 Android `VehicleGeometry.java` 和服务器 `vehicle_geometry_generated.py`。运行 `python tools/generate_vehicle_geometry.py --check` 检查它们是否一致。Robei 原生精简入口见 `robei_model_project`，开发模型和测试来源见 `little_car`。这些镜像用于不同工具，不是几套独立实现。

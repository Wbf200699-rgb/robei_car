# 阅读顺序与定位方法

## 第一遍：只看主链，建立全局图

1. 看 `verilog/sys_route_gui_fsm_top.v` 的端口、内部 `wire` 和 23 个实例，不要先读子模块代码。
2. 看 `verilog/route_contol_core.v`，理解路线编码、四状态 FSM、坐标如何变化。
3. 看 `verilog/c5_route_controller.v`，理解自动/手动如何生成四轮有符号速度。
4. 看 `verilog/c5_motor_frame_driver.v`，理解轮速如何变成 C5 ASCII 串口帧。
5. 看 `verilog/bt_route_uart.v`，先看端口和 `status_char`，再看 RX 命令处理和 TX 调度。
6. 看 `verilog/linkpg_uwb_bridge.v`，理解 UWB 命令如何复用蓝牙命令入口以及状态行如何分片回传。
7. 看 `verilog/rplidar_c1_uart.v` 与 `verilog/ultrasonic_obstacle_guard.v`，理解停车信号来源。
8. 单独看 `verilog/motion_sensor_subsystem.v` 和 `motion_sensor_bringup_top.v`；它们目前是独立标定固件，不在主车顶层中。
9. 最后看 `verilog/hdmi_top.v` 和 `verilog/gui_route_renderer.v`。渲染器最长，放在最后。

第一遍可以跳过所有 `temp_*`、旧 `motor_*`、演示顶层和字符点阵函数。

## 第二遍：沿一个实际动作追信号

### 手机按前进

```text
bt_uart_rx
 -> bt_route_uart.rx_data
 -> manual_mode/manual_cmd
 -> c5_route_controller.lf/rf/lr/rr_speed
 -> c5_motor_frame_driver.frame_*_pulse
 -> c5_uart_tx
```

同时反向追踪 `obstacle_stop -> motor_blocked -> @S/@L`，可以理解为什么命令收到但
电机可能不执行。

### 板上按键编辑路线

```text
key_pin*
 -> gpio_key_adapter
 -> key_low_merge
 -> route_key_lock_gate
 -> key_pulse_filter
 -> route_contol_core.route_info/target_x/target_y
 -> gui_route_renderer + bt_route_uart
```

### 雷达点触发停车

```text
lidar_uart_rx
 -> 5 字节节点解析
 -> node_angle_q6 / distance_q2
 -> 八扇区本圈最小值
 -> 三样本中值
 -> 长方形前角/侧面阈值
 -> lidar obstacle_stop
 -> obstacle_stop_merge
 -> c5_route_controller
```

### 服务器经 UWB 下发前进

```text
服务器 -> ESP32-CAM -> 主基站 A -> 标签 1
 -> linkpg_uwb_bridge.Modbus/CRC16/命令 CRC8
 -> bt_uart_rx_merged
 -> bt_route_uart
 -> c5_route_controller
```

回程沿 `bt_uart_tx_tap -> report_buffer -> UWB 分片 -> ESP32-CAM -> 服务器`
返回。命令队列 ACK 只证明到达某一传输阶段，实际动作仍以新的 `@S` 状态为准。

### 运动传感器标定

```text
PMW3901 + 两块 BMI088
 -> motion_sensor_subsystem 共享 SPI 初始化/采样/融合
 -> @M 标定帧
 -> 板载 USB-UART
 -> read_motion_sensor_uart.ps1
 -> analyze_motion_calibration.py
```

该链当前不进入 `c5_route_controller`，避免未标定轴向和比例影响电机控制。

## 第三遍：对照仿真看时序

从 `testbenches/` 开始：

- `c5_motor_frame_driver_robei_test.v`：上电等待、停车序列、刷新周期。
- `c5_motor_mapping_robei_test.v`：前后左右及四轮修正到脉宽的映射。
- `car_param_controller_robei_test.v`：默认值、增减步进和上下限。
- `c5_route_controller_robei_test.v`：手动、自动、前进阻挡及倒车/转向脱困。

对应已归档波形证据位于仓库 `docs/experiment_evidence/2026-07-12_*`。这些 testbench
覆盖电机和参数主链，但尚未完整覆盖蓝牙调度、雷达节点解析、路线数组边界和 HDMI
复位时序。

## 搜索技巧

```powershell
# 找一个信号在哪里定义和使用
rg -n "motor_blocked" little_car\source_review\verilog

# 找全部状态和协议常量
rg -n "localparam|parameter" little_car\source_review\verilog

# 找所有模块实例关系
Import-Csv little_car\source_review\module_manifest.csv |
  Select-Object module,child_classes

# 比较导出源码和 Robei 当前 build 产物
git diff --no-index little_car\build\c5_route_controller.v `
  little_car\source_review\verilog\c5_route_controller.v
```

调试时优先找“产生信号的模块”，再找“消费信号的模块”。不要从顶层长线网名猜业务
含义；顶层网名通常只是 `实例名_端口名`，真正算法都在叶子模块中。

## 修改时的边界

`source_review` 是阅读副本，不是工程真源。后续修改应先改 `little_car/build/*.v`，
再同步到相应 `.model`，最后重新运行本目录导出脚本。若只修改这里，Robei 重新生成
工程时不会采用这些变化。

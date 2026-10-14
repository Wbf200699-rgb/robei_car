# 项目架构与算法

完整模块和信号流图见 `ARCHITECTURE_DIAGRAM.md`。

## 1. 总体数据流

```text
板载按键 -> gpio_key_adapter -> key_low_merge -> route_key_lock_gate
                                              |
CH9141K -> linkpg_uwb_bridge -> bt_route_uart-+-> sys_route_control_fsm
        |                                         -> route_info/坐标/状态
        +-> 手动命令/参数脉冲
UWB标签1 -> LinkPG命令/状态桥 ----------------+
                  |                    超声波 ----+
                  v                    雷达 ------+-> obstacle_stop_merge
          car_param_controller                      |
                  |                                 v
                  +------------------------> c5_route_controller
                                                    |
                                                    v
                                           c5_motor_frame_driver -> C5 UART

route_info/状态 -> gui_route_renderer -> hdmi_top -> TMDS 差分输出
雷达/状态/C5诊断 -> bt_route_uart -> CH9141K手机 + LinkPG标签1 -> 服务器

PMW3901 + 双BMI088 -> motion_sensor_subsystem -> @M标定UART
                         （当前为独立固件，不进入主车控制）
```

系统顶层是 `sys_route_gui_fsm_top`。主逻辑使用 50 MHz 系统时钟，
`power_on_reset` 在外部复位释放后再保持内部复位 1,000,000 周期，即约 20 ms。

## 2. 路线与按键链

`gpio_key_adapter` 处理物理按键极性，`key_low_merge` 合并物理按键和蓝牙模拟按键，
`route_key_lock_gate` 在手机参数设置期间阻断路线按键。`sys_route_control_fsm`
用 5 个 `key_pulse_filter` 做消抖和单脉冲，再调用实际核心 `route_contol_core`。

路线使用 `route_info[63:0]`，每条命令占 2 bit：停止 `00`、右转 `01`、左转
`10`、前进 `11`。路线状态机有菜单、编辑、运行和障碍停止四种状态。编辑时左转或
右转会写入“转向 + 前进”两个内部命令；执行时 `run_step_pulse` 令队首命令出队，
并更新 `now_x/now_y/now_dir`。

注意：当前实际核心内部数组 `ROUTE_WORDS=16`，虽然总线宽度可表示 32 条命令，
部分写入判断仍用了 32。这是源码现状，不应把总线理论容量直接当作安全容量。

## 3. 参数和运动控制

`car_param_controller` 保存手机可调参数：整体速度、单步长度、四轮默认速度和雷达
安全距离。速度范围 100..900、步长 5..80、雷达阈值 150..1500 mm。

`c5_route_controller` 同时支持自动路线和手机手动模式。每轮实际速度采用：

```text
adjusted_speed = clamp(common_speed + wheel_default - 700, 100, 900)
```

转向速度在大于 150 时再减 50。自动前进时间约为：

```text
step_to_ms = step_length * 125
```

因为 RTL 用 `(steps << 7) - (steps << 1) - steps` 实现 125 倍。手动前进遇到
障碍会被禁止，但倒车和原地转向仍可用于脱困；自动模式遇到合并障碍信号则停止。

## 4. C5 电机帧与停车保护

`c5_motor_frame_driver` 把四路 11 bit 有符号速度转成 C5 脉宽命令：

```text
左轮脉宽 = 1500 + signed_speed
右轮脉宽 = 1500 - signed_speed
```

四轮 ID 为 006、007、008、009，运动帧是 62 字节 ASCII 串，115200 8N1。
运动期间每 100 ms 触发一次刷新；上电先等待 500 ms，再发送 3 个
`#255PDST!` 停车帧。停止转换、强制停止和待发送停车序列优先于普通运动帧。
模块还输出帧计数、最后四轮脉宽、帧忙和停车序列状态供蓝牙诊断。

## 5. 超声波与雷达避障

`ultrasonic_obstacle_guard` 每 60 ms 测量一次，发 10 us 触发脉冲，等待/测量超时
均为 30 ms。距离换算用 `echo_cycles / 2900` 得到厘米；超声阈值仍由路线单步长度
控制，超时值统一为 `FFFF`，超时本身不触发停车。

`rplidar_c1_uart` 使用 460800 8N1。状态机先等待 0.5 s，发停止命令 `A5 25`，
等待 0.1 s，再发普通扫描命令 `A5 20`；2 s 无有效字节则离线，1 s 后重试。
扫描节点按 5 字节格式解析，角度为 Q6、距离为 Q2。只接受质量非零且距离在
50..12000 mm 的点。

360 度被划成 8 个 45 度扇区。每圈先取扇区最小值，再结合前两圈历史值做三样本
中值滤波；无有效点输出 `FFFF`。避障不是只看正前方一个点：正前方使用可调安全
距离，前角增加 150 mm 裕量，侧面使用约 220 mm 保护宽度，从而近似小车的长方形
轮廓。`obstacle_stop_merge` 对雷达和超声停车信号做逻辑或。

## 6. 蓝牙闭环

`bt_route_uart` 是 CH9141K 透明串口侧协议，115200 8N1。手机发 `P` 维持链路，
3 s 无心跳认为离线；手动运动命令 300 ms 未刷新会自动回到停止。

该模块内只有一个 UART 发送调度器，按“短回复、路线状态、雷达状态”的顺序发送
完整帧，避免 `@PONG` 插入 `@L` 中间。状态每 200 ms 周期触发，并在路线、坐标、
参数、手动状态或电机阻塞变化时提前触发。`?` 会请求路线状态。

主要上报：`@Kx\n` 命令确认、`@PONG\n` 心跳、`@S...\n` 路线/参数/C5 状态、
`@L...\n` 双传感器和八扇区数据。`@S` 和 `@L` 尾部都有 8 bit 十六进制序号。

## 7. UWB 与服务器桥

`linkpg_uwb_bridge` 连接车载标签 1 的 115200 8N1 TTL UART。服务器经
ESP32-CAM 和主基站 A 下发带序号、命令 CRC8 和 Modbus CRC16 的单字符命令；
桥接器校验目标、白名单、格式与重复序号后，只在 CH9141K RX 连续空闲至少
12 bit 时把字符注入现有 `bt_route_uart`。因此 UWB 不绕过手动 300 ms 看门狗、
参数限幅、避障或 C5 停车保护。

回程旁听 `bt_uart_tx`，从 `@` 捕获到换行，最多缓存 96 字节；超过 64 字节时
拆成两个 UWB 透传分片。下行截断 Modbus 帧连续空闲 10 ms 后清理解析状态，
防止一条坏帧吞掉后续有效命令。`rx_packet_count`、`rx_crc_error_count`、
`tx_packet_count` 和 `telemetry_drop_count` 用于区分无线链路、解析和队列故障。

## 8. PMW3901 与双 BMI088 独立标定链

`motion_sensor_subsystem` 通过一条约 1.923 MHz SPI mode-3 总线和五路独立片选
连接 PMW3901、两块 BMI088 的加速度与陀螺器件。PMW3901 完成 49/B6 ID 检查、
初始化和约 100 Hz DX/DY/SQUAL 采样；BMI088 完成 1E/0F ID 检查、量程配置、
512 样本启动零偏和双传感器一致性判断。

当前 `motion_sensor_bringup_top` 是独立 bit：每 100 ms 输出一条 99 字节 `@M`
标定帧，只用于测轴向、比例、噪声和退化状态。它没有实例化在
`sys_route_gui_fsm_top` 中，也没有 C5 输出。完成
`docs/MOTION_SENSOR_CALIBRATION_PROCEDURE.md` 的实物实验后，才会生成只读融合
模块并评估是否接入主工程。

## 9. HDMI 显示链

`clk_wiz_0` 直接实例化 7 系列 MMCM：50 MHz 输入，经 20 倍 VCO 后分别除以 20
和 4，得到 50 MHz 像素时钟与 250 MHz 串化时钟；MMCM 锁定后再等待 50,000 个
像素时钟才释放显示逻辑。

`hdmi_timing_gen` 产生 1024x600 时序，总计 1324x645。`gui_route_renderer` 根据
像素坐标组合生成 RGB，绘制状态栏、路线网格、当前位置和目标；它是资源占用最大、
组合逻辑最重的模块。三个 `hdmi_tmds_encoder` 完成 8b/10b 风格 TMDS 编码，四个
`hdmi_serializer` 用 `ODDR` 和 `OBUFDS` 输出三路数据与一路时钟差分信号。

## 10. 旧链路和辅助模块

- `motor_encoder_interface`、`motor_route_controller`、`motor_tb6612_driver`：旧 TB6612 底盘链路。
- `gui_route_test_top`、`gui_route_demo_info_source`：显示演示链，不在当前顶层中。
- `temp_*`：把固定配置值接入图形工程，部分仍被当前顶层使用。
- `c5_motor_test_pattern`：测试模式模块，当前顶层端口空接，不参与实际控制。

# 演示与截图

建议准备一次 2–3 分钟的真实演示：架构介绍 → BLE 连接与状态 → 手动控制及停止 → 雷达 → 参数标定 → 一组波形。当前仓库没有伪造实物照片或 App 截图。

## RobeiEDA 波形

打开 `robei_model_project/robei_model_project.proj`，选择对应 `_parameter.test`，运行至 `RESULT TEST_PASS`。版本不同按钮名称可能不同，以软件中的仿真/波形入口为准。

| 测试 | 建议信号 | 截图要证明什么 |
| --- | --- | --- |
| `rplidar_protocol_parameter.test` | `scan_enable`、`cmd_state`、`scan_active`、`sample_valid`、`lidar_online`、`front_distance_mm` | 接收有效节点后才上线，不能只凭描述符宣称健康 |
| `lidar_map_sampler_parameter.test` | `new_scan`、`quality`、`angle_q6`、`distance_q2`、`map_chunk_valid`、`map_part`、`map_chunk_crc`、`map_drop_count` | 分段编号 0..5、有效脉冲、CRC 与丢段计数 |
| `onboard_exploration_parameter.test` | `lidar_new_scan`、`obstacle_stop`、`manual_mode`、`auto_mode`、`auto_cmd`、`stop_reason` | 障碍与手动接管时控制输出变化 |
| `c5_motion_stabilizer_parameter.test` | `fused_gyro_z`、`yaw_correction`、`lf_speed`、`rf_speed`、`force_stop` | 修正限幅和停止响应 |

每组保存“整体过程”和“关键跳变放大”两张图。保留信号名称、时间轴、文件/测试名和通过信息；距离/速度用十进制，有符号量用有符号十进制，CRC 用十六进制。用 Windows `Win + Shift + S` 截图，文件名说明现象，例如 `lidar_map_parts_0_to_5.png`。

也可以打开已归档的 `little_car/test/results/hardware_parameters/*.vcd`。这些是加速 RTL 参数波形，不能截图后标注成真实 UART 频率或实车响应时间。

## App 与实车

优先使用同一版 APK、同一辆车、同一套配置，录制连接、状态同步、手动按下/松开、急停、雷达数据和实际估值随标定更新的过程。页面没有设备数据时保留“未连接/待标定”等真实状态。

截图前隐藏 MAC、IP、Token、Wi-Fi 信息和通知栏个人消息。录像说明日期、版本、测试场地与操作；未经测量不标注精度或性能数字。

可在 `docs/media/` 放本人确认可公开的 PNG/JPG/短 GIF，在首页通过相对链接引用。大视频放本人视频平台或 GitHub Release 后链接，避免塞入 Git 历史。

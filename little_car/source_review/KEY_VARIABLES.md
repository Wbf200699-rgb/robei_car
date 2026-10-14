# 关键变量与编码

## 路线状态

| 名称 | 位宽/编码 | 含义 |
|---|---:|---|
| `route_info` | 64 bit | 低位命令先执行，每命令 2 bit |
| `ROUTE_STOP/RIGHT/LEFT/FORWARD` | `00/01/10/11` | 停止、右转、左转、前进 |
| `state_info` | 5 bit | `00000` 菜单，`00001` 编辑，`00011` 运行，`00100` 障碍停止 |
| `active_route_steps` | 6 bit | 尚未执行的内部命令数 |
| `completed_route_steps` | 6 bit | 已执行命令数 |
| `now_x/now_y` | 8 bit | 当前逻辑栅格位置，复位值 128/128 |
| `target_x/target_y` | 8 bit | 编辑后路线终点 |
| `run_step_pulse` | 1 bit | 一条电机动作执行完成，推动路线出队 |

## 手机可调参数

| 参数 | 默认值 | 范围 | 步进 | 关键输出 |
|---|---:|---:|---:|---|
| 整体速度 | 700 | 100..900 | 25 | `overall_speed` |
| 单步长度 | 20 | 5..80 | 5 | `step_length_set` |
| 左前默认速度 | 700 | 100..900 | 25 | `lf_default_speed` |
| 右前默认速度 | 700 | 100..900 | 25 | `rf_default_speed` |
| 左后默认速度 | 700 | 100..900 | 25 | `lr_default_speed` |
| 右后默认速度 | 700 | 100..900 | 25 | `rr_default_speed` |
| 雷达安全距离 | 450 mm | 150..1500 mm | 50 mm | `lidar_safe_mm` |

参数选择 `selected_param`：0 整体、1 步长、2 左前、3 右前、4 左后、5 右后、6 雷达。

## 手动控制和安全状态

| 名称 | 编码/含义 |
|---|---|
| `manual_mode` | 1 表示手机手动控制覆盖自动路线 |
| `manual_cmd` | 0 停止、1 前进、2 后退、3 左转、4 右转 |
| `manual_watchdog_expired` | 300 ms 未续发运动命令后置 1 |
| `ultrasonic_obstacle_stop` | 超声波停车请求 |
| `lidar_obstacle_stop` | 长方形轮廓雷达停车请求 |
| `merged_obstacle_stop` | 两种停车请求的逻辑或 |
| `motor_blocked` | 当前命令确实被避障逻辑拦截 |

## 雷达变量

| 名称 | 单位 | 含义 |
|---|---|---|
| `angle_q6` | 1/64 度 | 当前扫描点角度 |
| `distance_q2` | 1/4 mm | 当前扫描点距离 |
| `sector_min_0_q2..7` | Q2 | 当前圈八扇区最小值 |
| `sector_hist_1_q2/sector_hist_2_q2` | Q2 | 前两圈历史值，用于三样本中值 |
| `sector_front_mm..sector_front_left_mm` | mm | 手机上报的八方向滤波值 |
| `lidar_online` | 布尔 | 已收到有效扫描数据且未超时 |
| `FFFF` | mm | 扇区无有效点或距离无效 |

扇区顺序为：前、右前、右、右后、后、左后、左、左前。

## C5 执行链变量

| 名称 | 含义 |
|---|---|
| `lf/rf/lr/rr_speed` | 11 bit 二补码有符号轮速 |
| `frame_*_pulse` | 当前运动帧锁存的四轮脉宽 |
| `last_*_speed` | 判断命令变化和停止转换 |
| `refresh_count` | 运动帧 100 ms 刷新计数 |
| `stop_frames_remaining` | 强化停车剩余帧数，启动值 3 |
| `frame_busy` | 正在发送一条完整 C5 帧 |
| `frame_count` | 已完成帧计数，供手机诊断 |
| `force_stop` | 蓝牙停止命令的高优先级停车脉冲 |

## 蓝牙定时与帧

| 常量/变量 | 值 | 含义 |
|---|---:|---|
| `BAUD` | 115200 | CH9141K 透明串口波特率 |
| `STATUS_CYCLES` | 10,000,000 | 200 ms 周期上报 |
| `LINK_TIMEOUT_CYCLES` | 150,000,000 | 3 s 心跳离线 |
| `MANUAL_TIMEOUT_CYCLES` | 15,000,000 | 300 ms 手动动作看门狗 |
| `route_status_seq` | 8 bit | `@S` 序号 |
| `lidar_status_seq` | 8 bit | `@L` 序号 |
| `sending_reply/sending_status` | 布尔 | 串口帧原子发送互斥状态 |

`@S` 的负载长度为 84 个十六进制/类型字符，外加前导 `@` 和结尾换行；
`@L` 的负载长度为 54 个字符。准确字段位置直接看 `bt_route_uart.v` 的
`status_char` 函数，不要根据日志肉眼猜字段。

## UWB 桥关键量

| 名称 | 值/含义 |
|---|---|
| `uwb_uart_rx/tx` | 标签 1 与 FPGA 的 115200 8N1 交叉串口 |
| `bt_uart_rx_merged` | CH9141K RX 与 UWB 注入线按线与后的现有命令入口 |
| `FRAME_IDLE_TIMEOUT_CLKS` | 500,000 周期，即截断 Modbus 帧 10 ms 后恢复找帧 |
| `DUPLICATE_WINDOW_CLKS` | 两秒内相同命令序号只执行一次 |
| `COMMAND_WAIT_CLKS` | 注入前等待蓝牙 RX 空闲的 100 ms 上限 |
| `report_buffer[0:95]` | 捕获一条完整 `@S/@L/@K/@PONG` 的最大 96 字节缓存 |
| `report_part` | 超过 64 字节的状态行分成第 0/1 段回传 |
| `rx_packet_count/rx_crc_error_count` | LinkPG 接收链路好帧与 CRC16 错帧计数 |
| `telemetry_drop_count` | 捕获过长或上行忙导致的状态行丢弃计数 |

UWB 下行不是第二套运动协议，而是把白名单单字符命令安全注入原
`bt_route_uart`。因此蓝牙和 UWB 最终共享同一手动看门狗、参数寄存器、避障和
C5 执行链。

## PMW3901 与双 BMI088 关键量

| 名称 | 值/含义 |
|---|---|
| `main_state` | PMW 初始化、BMI 配置、双 IMU/光流轮询总状态机 |
| `spi_cs_n_raw[4:0]` | PMW、BMI0 ACC/GYRO、BMI1 ACC/GYRO 五路低有效片选 |
| `calibration_count` | 启动静止零偏样本数，硬件目标 512 |
| `bmi*_gyro_bias_*` | 各 BMI088 三轴启动平均零偏 |
| `imu_source` | 0 无源、1 BMI0、2 BMI1、3 两块平均 |
| `imu_degraded` | 一块 IMU 失效后的单块降级状态 |
| `imu_disagreement` | 双 IMU 加速度或陀螺差超过门限 |
| `flow_squal` | PMW3901 表面质量，标定默认要求至少 90% 帧不低于 20 |
| `flow_delta_x/y` | 当前光流增量，轴向、符号和比例尚待实物标定 |
| `flow_accum_x/y` | 自复位累计的 32 bit 有符号光流计数 |
| `sample_sequence` | 融合采样序号，用于 UART 采集缺口检查 |
| `snap_*` | 一条 99 字节 `@M` 发送前锁存的一致数据快照 |

这些变量当前只存在于独立 `motion_sensor_bringup_top` 固件。完成规范静止、
前移、右移和顺逆旋转标定前，不接入主车 C5 控制。

## HDMI 关键量

| 名称 | 值/含义 |
|---|---|
| `clk_out1` | 50 MHz 像素时钟 |
| `clk_out2` | 250 MHz TMDS 串化时钟 |
| `stable_locked` | MMCM 锁定后延迟 50,000 像素周期的稳定标志 |
| `H_ACTIVE/V_ACTIVE` | 1024 / 600 |
| `H_TOTAL/V_TOTAL` | 1324 / 645 |
| `pixel_x/pixel_y` | 当前有效显示区坐标 |
| `rgb_data` | 渲染器输出的 24 bit RGB888 |
| `running_disparity` | 每个 TMDS 编码器的直流平衡累计量 |

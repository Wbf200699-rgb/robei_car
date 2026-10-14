"""Human-oriented annotations used by the source-review Verilog exporter."""

from __future__ import annotations

import re


# Every exported module gets a role description.  The long implementation notes
# are intentionally concentrated on the modules that form the active C5 design.
MODULE_DOCS = {
    "sys_route_gui_fsm_top": (
        "当前小车工程的系统顶层，连接所有物理接口和业务子系统。",
        [
            "主时钟为 50 MHz；外部复位先经过 power_on_reset 形成内部 system_rst_n。",
            "按键和蓝牙既可进入自动路线 FSM，也可由 manual_mode 直接控制 C5 四轮。",
            "超声波与 RPLIDAR 的停车请求经 obstacle_stop_merge 合并后送入运动控制器。",
            "路线、传感器、参数和 C5 诊断状态同时送往 HDMI 渲染器与蓝牙上报器。",
            "LinkPG 桥把 UWB 下行命令注入原蓝牙 RX，并把 FPGA 状态帧分片回传到 UWB。",
        ],
    ),
    "power_on_reset": (
        "上电复位延时器，在外部复位释放后继续保持内部复位约 20 ms。",
        ["POR_CYCLES=1,000,000；50 MHz 下每周期 20 ns，因此延时为 20 ms。"],
    ),
    "gpio_key_adapter": (
        "物理按键极性适配器，把板级按键转换为统一的低有效方向/确认按键。",
        ["key_active_low 决定输入是否反相，后级始终按低有效 key_*_n 解释。"],
    ),
    "key_low_merge": (
        "低有效按键合并器，把实体按键和蓝牙模拟按键接到同一条路线输入链。",
        ["低有效信号使用逻辑与合并：任意来源拉低都会形成一次有效按键。"],
    ),
    "route_key_lock_gate": (
        "路线按键门控器，手机处于参数设置模式时阻断路线编辑按键。",
        ["锁定时所有输出保持 1，即低有效按键的未按下状态。"],
    ),
    "key_pulse_filter": (
        "按键同步、消抖和单脉冲生成器。",
        ["只有输入持续稳定达到计数阈值后才改变状态；每次按下只产生一个时钟脉冲。"],
    ),
    "sys_route_control_fsm": (
        "路线控制层次封装：五路按键消抖后进入实际路线核心。",
        [
            "该模块自身没有业务状态，只负责 key_*_n 到 key_*_pulse 的转换与层次连接。",
            "当前实例类型拼写为 route_contol_core，这是工程真实依赖，不是注释笔误。",
        ],
    ),
    "route_contol_core": (
        "当前实际使用的路线编辑、执行、坐标更新状态机。",
        [
            "route_info 每 2 bit 保存一条内部命令：00 停止、01 右转、10 左转、11 前进。",
            "编辑时一次左/右操作会写入转向和前进两个内部命令，因此 UI 步和内部命令数不同。",
            "运行时 c5_route_controller 的 run_step_pulse 使队首命令出队并更新当前位置/朝向。",
            "重要风险：数组 ROUTE_WORDS=16，但前进分支的部分边界判断允许计数小于 32。",
        ],
    ),
    "sys_route_control_core": (
        "未被当前顶层采用的另一版路线核心，保留在 Robei 项目清单中。",
        ["它直接在 64 bit route_info 上移位，并使用 step_start/step_words 保存撤销边界。"],
    ),
    "car_param_controller": (
        "保存并限幅所有手机可调运行参数。",
        [
            "参数包括整体速度、单步长度、四轮默认速度和雷达安全距离。",
            "速度范围 100..900、步长 5..80、雷达阈值 150..1500 mm，所有修改均饱和限幅。",
            "selected_param 编码 0..6；配置脉冲来自 bt_route_uart。",
        ],
    ),
    "c5_route_controller": (
        "把自动路线或手机手动命令转换为四轮 11 bit 二补码有符号速度。",
        [
            "每轮速度 = clamp(整体速度 + 该轮默认速度 - 700, 100, 900)。",
            "原地转向时左、右侧符号相反，并在速度大于 150 时将转向幅值减 50。",
            "手动模式只阻断受障碍影响的前进，倒车和左右原地转向仍可用于脱困。",
            "自动模式每条命令计时完成后输出一个 run_step_pulse，推动路线 FSM 出队。",
        ],
    ),
    "c5_motor_frame_driver": (
        "将四轮有符号速度编码为 C5 总线 ASCII 帧，并实现启动/停车保护。",
        [
            "电机 ID 006/007/008/009；运动帧 62 字节，串口 115200 8N1。",
            "左轮脉宽=1500+速度，右轮脉宽=1500-速度，用于补偿左右安装方向。",
            "运动期间每 100 ms 刷新一次；上电等待 500 ms 后发送三次 #255PDST!。",
            "force_stop、运动到停止的边沿和剩余停车序列具有高于普通运动帧的优先级。",
        ],
    ),
    "c5_motor_test_pattern": (
        "C5 四轮固定测试序列发生器。",
        ["当前系统顶层保留了实例但输出未接入执行链，因此不会控制实际轮子。"],
    ),
    "temp_c5_motor_config_source": (
        "当前 C5 执行链使用的固定使能、默认时间和后备速度配置源。",
        ["手机参数有效时整体/单轮速度会覆盖这里的后备速度。"],
    ),
    "bt_route_uart": (
        "CH9141K 透明串口协议、手动控制、参数命令和原子状态上报调度器。",
        [
            "UART 为 115200 8N1；P/p 心跳维持连接，3 s 未收到心跳则 bt_connected 清零。",
            "手动运动命令有 300 ms 看门狗，超时后强制 manual_cmd=STOP 并产生停车脉冲。",
            "短回复、@S 路线状态和 @L 雷达状态共享唯一发送状态机，保证帧不会互相穿插。",
            "@S 在数据变化、问号查询或 200 ms 周期到达时发送；@L 携带八扇区和障碍来源。",
        ],
    ),
    "linkpg_uwb_bridge": (
        "LinkPG 标签侧双向桥，把 UWB 数据透传帧与现有蓝牙单字符协议无侵入连接。",
        [
            "UWB 与蓝牙侧均为 115200 bit/s 8N1；蓝牙物理 RX 和 UWB 注入线按线与合并。",
            "下行只接受带序号和 CRC8 的白名单单字符命令，并用两秒窗口抑制重复执行。",
            "仅在蓝牙 RX 连续空闲至少 12 bit 后注入命令，100 ms 等待仍冲突则返回失败 ACK。",
            "上行捕获完整 @S/@L/@K/@PONG 行，最多 96 字节并拆成两段 LinkPG 透传载荷。",
            "所有 LinkPG Modbus 帧校验 CRC16；截断帧空闲 10 ms 后自动复位解析器。",
        ],
    ),
    "motion_sensor_subsystem": (
        "共享 SPI 总线驱动一块 PMW3901 与两块 BMI088，并输出原始光流、双 IMU 融合和标定遥测。",
        [
            "SPI mode 3 约 1.923 MHz，五个独立片选共享 SCLK/MOSI/MISO，不使用厂商 IP 核。",
            "PMW3901 完成复位、49/B6 ID 检查、寄存器初始化和约 100 Hz DX/DY/SQUAL 轮询。",
            "两块 BMI088 分别检查 1E/0F ID，配置加速度 +/-6 g 与陀螺 +/-250 dps。",
            "启动静止 512 组样本估计两块陀螺偏置；一致时平均，单块异常时降级。",
            "每 100 ms 输出固定 99 字节 @M ASCII 帧，供轴向、比例、噪声和一致性标定。",
        ],
    ),
    "motion_sensor_bringup_top": (
        "PMW3901 与双 BMI088 独立上板验证顶层，不连接 HDMI、UWB、蓝牙或 C5 电机。",
        [
            "复用 power_on_reset 的 20 ms 内部复位，并把三块器件在线状态直接映射到 LED。",
            "当前独立 bit 只用于采集标定数据；未完成轴向和比例标定前不能进入电机闭环。",
        ],
    ),
    "uart_tx_8n1": (
        "通用单字节 UART 8N1 发送器。",
        ["start 接收一个周期脉冲后锁存 data，依次发送起始位、8 个数据位和停止位。"],
    ),
    "rplidar_c1_uart": (
        "RPLIDAR C1 启停、串口接收、扫描节点解析、八扇区滤波和轮廓避障。",
        [
            "UART 为 460800 8N1；启动序列为等待 0.5 s、A5 25 停止、等待 0.1 s、A5 20 扫描。",
            "描述符 A5 5A 05 00 00 40 81 匹配后进入连续 5 字节扫描节点解析。",
            "角度采用 Q6（1/64 度），距离采用 Q2（1/4 mm）；质量为零或 50..12000 mm 外的点丢弃。",
            "每圈八扇区先取最小值，再与前两圈值做 median3；无效扇区统一输出 FFFF。",
            "正前、前角和侧面采用不同阈值，近似长方形车体的前角及侧边保护区。",
        ],
    ),
    "ultrasonic_obstacle_guard": (
        "前向超声波触发、回波计时、距离换算和独立停车判断。",
        [
            "每 60 ms 测量一次，触发高电平 10 us，等待及测量超时均为 30 ms。",
            "距离厘米数约等于 echo_cycles/2900；超时输出 FFFF，但不会误触发停车。",
        ],
    ),
    "obstacle_stop_merge": (
        "双源避障合并器。",
        ["最终 obstacle_stop = ultrasonic_stop OR lidar_stop；来源仍分别回传给手机。"],
    ),
    "clk_wiz_0": (
        "使用 7 系列 MMCM/BUFG 原语产生 HDMI 像素与 5 倍串化时钟。",
        [
            "50 MHz 输入经 VCO 后产生 50 MHz clk_out1 和 250 MHz clk_out2。",
            "MMCM LOCKED 后额外等待 50,000 个像素周期，降低复位后 HDMI 偶发失锁风险。",
            "这是手写原语封装，不是 Vivado IP 核。",
        ],
    ),
    "hdmi_timing_gen": (
        "生成 1024x600 视频时序、同步信号、数据有效和像素坐标。",
        ["水平总周期 1324、垂直总行数 645；有效区外 pixel_x/pixel_y 输出 0。"],
    ),
    "hdmi_tmds_encoder": (
        "单颜色通道 TMDS 8 bit 到 10 bit 编码器。",
        ["消隐期输出控制码；有效期根据输入 1 的数量和 running disparity 选择 XOR/XNOR 与反相。"],
    ),
    "hdmi_serializer": (
        "把一个像素周期的 10 bit TMDS 字并串转换为差分输出。",
        ["250 MHz 双沿 ODDR 实现等效 500 Mbit/s；OBUFDS 使用 TMDS_33 标准。"],
    ),
    "hdmi_top": (
        "完整 HDMI 发送流水线的层次封装。",
        [
            "时钟 -> 1024x600 时序 -> RGB 拆分 -> 三路 TMDS 编码 -> 四路串化差分输出。",
            "蓝色通道承载 hsync/vsync 控制码，红绿通道消隐期控制输入固定为 0。",
        ],
    ),
    "hdmi_const": ("HDMI 常量源。", ["提供 TMDS 时钟通道 10 bit 模式和逻辑零。"]),
    "hdmi_rgb_splitter": ("RGB888 通道拆分与消隐门控。", ["pixel_de=0 时三色数据强制为零。"]),
    "hdmi_pixel_shader": ("早期像素测试图案发生器。", ["当前主显示使用 gui_route_renderer，本模块不在系统顶层。"]),
    "gui_route_renderer": (
        "按像素坐标组合绘制小车状态、菜单、路线网格、当前位置和目标。",
        [
            "该模块不保存帧缓存，每个像素直接由坐标、路线和状态组合计算 RGB。",
            "路线命令被展开为水平/垂直线段；字符使用 5x7 点阵放大两倍绘制。",
            "这是工程组合逻辑和 LUT 占用最大的模块，改动字符/图形时需重点关注 ABC9 与布线。",
        ],
    ),
    "gui_route_test_top": ("独立 HDMI GUI 演示顶层。", ["使用临时状态源验证显示，不属于当前小车系统顶层。"]),
    "gui_route_demo_info_source": ("GUI 演示路线与状态常量源。", ["只用于显示测试。"]),
    "gui_menu_to_state_info": ("把简化菜单编码映射成 GUI 状态编码。", ["只用于演示顶层。"]),
    "temp_gui_status_source": ("GUI 临时状态源。", ["为早期显示验证提供固定运行状态。"]),
    "temp_run_status_source": ("临时运行状态源。", ["为旧模块测试提供固定 state_info。"]),
    "temp_ultrasonic_echo_source": ("超声波回波仿真/临时源。", ["硬件顶层实际使用外部 ultrasonic_echo。"]),
    "temp_ultrasonic_enable_source": ("超声波固定使能源。", ["当前顶层用它保持超声测距开启。"]),
    "motor_encoder_interface": ("旧底盘双路编码器计数与方向接口。", ["C5 串口底盘当前不使用此模块。"]),
    "motor_route_controller": ("旧 TB6612 底盘路线到左右轮控制器。", ["当前 C5 四驱链由 c5_route_controller 替代。"]),
    "motor_tb6612_driver": ("旧 TB6612 双 H 桥 PWM 驱动器。", ["当前 C5 串口电机不使用此模块。"]),
    "temp_motor_config_source": ("旧 TB6612 链固定参数源。", ["不在当前 C5 主执行链。"]),
    "temp_key_gpio_config_source": ("板级按键极性常量源。", ["当前输出决定 gpio_key_adapter 是否反相输入。"]),
    "buzzer_event_gen": ("从复位、路线编辑和运行状态边沿产生蜂鸣事件脉冲。", ["输出事件进入 buzzer_ctrl 的优先级音效状态机。"]),
    "buzzer_ctrl": ("蜂鸣器音调/节奏发生器。", ["根据事件选择提示音序列，通过计数分频生成 buzzer_out。"]),
}


PORT_DOCS = {
    "clk": "50 MHz 系统时钟输入。",
    "clk_pix": "HDMI 像素时钟域时钟。",
    "clk_5x": "像素时钟 5 倍频，用于 TMDS 双沿串化。",
    "rst_n": "低有效复位；0 清零模块内部状态。",
    "ext_rst_n": "板级外部低有效复位输入。",
    "system_rst_n": "延时释放后的系统低有效复位。",
    "bt_uart_rx": "CH9141K TXD 到 FPGA 的透明串口接收线。",
    "bt_uart_tx": "FPGA 到 CH9141K RXD 的透明串口发送线。",
    "bt_uart_rx_in": "CH9141K 物理 RX 电平在进入 UWB 合并器前的原始输入。",
    "bt_uart_tx_tap": "旁听 FPGA 蓝牙 TX 的 UART 电平，用于捕获完整状态/确认行并经 UWB 回传。",
    "bt_uart_rx_merged": "CH9141K RX 与 UWB 命令注入线按线与后的 UART 输入，送入 bt_route_uart。",
    "bt_connected": "仅在持续收到 P/p 心跳时为 1。",
    "bt_at": "CH9141K AT 模式控制脚；当前顶层固定拉高进入透传。",
    "uwb_uart_rx": "LinkPG 标签 TXD 到 FPGA 的 115200 bit/s 数据透传输入。",
    "uwb_uart_tx": "FPGA 到 LinkPG 标签 RXD 的 Modbus 数据透传/ACK 输出。",
    "uwb_link_online": "最近三秒内收到过 CRC 正确 LinkPG 帧时保持为 1。",
    "rx_packet_count": "已接收且通过 LinkPG Modbus CRC16 校验的帧累计数。",
    "rx_crc_error_count": "长度完整但 CRC16 错误的 LinkPG 接收帧累计数。",
    "tx_packet_count": "已经完整发往 LinkPG UART 的 Modbus 帧累计数。",
    "telemetry_drop_count": "上行状态行过长、捕获冲突或队列占满造成的遥测丢弃累计数。",
    "c5_uart_rx": "C5 总线回传输入；当前控制链未解析该信号。",
    "c5_uart_tx": "发送到四轮 C5 控制总线的 UART 线。",
    "lidar_uart_rx": "RPLIDAR C1 TXD 到 FPGA 的 460800 波特输入。",
    "lidar_uart_tx": "FPGA 到 RPLIDAR C1 RXD 的命令发送线。",
    "sensor_spi_miso": "PMW3901 与两块 BMI088 共享 SPI MISO；只有当前片选器件允许驱动。",
    "sensor_spi_sclk": "约 1.923 MHz 的共享 SPI mode-3 时钟输出，空闲电平为高。",
    "sensor_spi_mosi": "发送给 PMW3901/BMI088 的共享 SPI MOSI 数据。",
    "pmw3901_cs_n": "PMW3901 低有效片选；初始化特殊阶段可被强制保持为低。",
    "bmi0_acc_cs_n": "第一块 BMI088 加速度计低有效 SPI 片选。",
    "bmi0_gyro_cs_n": "第一块 BMI088 陀螺仪低有效 SPI 片选。",
    "bmi1_acc_cs_n": "第二块 BMI088 加速度计低有效 SPI 片选。",
    "bmi1_gyro_cs_n": "第二块 BMI088 陀螺仪低有效 SPI 片选。",
    "sensor_debug_uart_tx": "115200 bit/s @M 标定遥测输出，独立固件连接板载 USB-UART RX。",
    "pmw3901_online": "PMW3901 产品 ID=49、反 ID=B6 且初始化完成后的在线锁存标志。",
    "bmi0_online": "第一块 BMI088 加速度/陀螺 ID 与连续样本均有效时的在线标志。",
    "bmi1_online": "第二块 BMI088 加速度/陀螺 ID 与连续样本均有效时的在线标志。",
    "calibration_done": "两块 BMI088 已累计 512 个静止样本并锁存陀螺零偏。",
    "fused_sample_valid": "当前周期产生一组双 IMU 平均或单 IMU 降级融合结果。",
    "imu_degraded": "只有一块 BMI088 可用时置 1；融合输出退化到在线的单块传感器。",
    "imu_disagreement": "两块有效 BMI088 的加速度或角速度差持续超过门限。",
    "imu_source": "融合来源编码：0 无有效源、1 BMI0、2 BMI1、3 两块平均。",
    "pmw3901_product_id": "PMW3901 寄存器 0x00 产品 ID，正常值为 0x49。",
    "pmw3901_inverse_id": "PMW3901 寄存器 0x5F 反产品 ID，正常值为 0xB6。",
    "bmi0_accel_id": "第一块 BMI088 加速度芯片 ID，正常值为 0x1E。",
    "bmi0_gyro_id": "第一块 BMI088 陀螺芯片 ID，正常值为 0x0F。",
    "bmi1_accel_id": "第二块 BMI088 加速度芯片 ID，正常值为 0x1E。",
    "bmi1_gyro_id": "第二块 BMI088 陀螺芯片 ID，正常值为 0x0F。",
    "flow_sample_valid": "PMW3901 一组 DX/DY/SQUAL 读取完成的单周期脉冲。",
    "flow_squal": "PMW3901 当前表面质量；标定工具默认要求至少 90% 帧不低于 20。",
    "flow_delta_x": "PMW3901 当前采样间隔的有符号 X 增量，轴向和比例尚待实物标定。",
    "flow_delta_y": "PMW3901 当前采样间隔的有符号 Y 增量，轴向和比例尚待实物标定。",
    "flow_accum_x": "自复位以来累计的有符号光流 X 计数，32 bit 回绕。",
    "flow_accum_y": "自复位以来累计的有符号光流 Y 计数，32 bit 回绕。",
    "fused_accel_x": "双 BMI088 一致性检查后的 X 轴加速度原始计数。",
    "fused_accel_y": "双 BMI088 一致性检查后的 Y 轴加速度原始计数。",
    "fused_accel_z": "双 BMI088 一致性检查后的 Z 轴加速度原始计数。",
    "fused_gyro_x": "扣除启动零偏后的融合 X 轴陀螺原始计数。",
    "fused_gyro_y": "扣除启动零偏后的融合 Y 轴陀螺原始计数。",
    "fused_gyro_z": "扣除启动零偏后的融合 Z 轴陀螺原始计数。",
    "sample_sequence": "每次形成融合采样时递增的 16 bit 序号，用于检测采集缺口。",
    "pmw_status_led": "独立验证顶层的 PMW3901 在线 LED。",
    "bmi0_status_led": "独立验证顶层的第一块 BMI088 在线 LED。",
    "bmi1_status_led": "独立验证顶层的第二块 BMI088 在线 LED。",
    "ultrasonic_echo": "超声波模块 ECHO 回波脉冲输入。",
    "ultrasonic_trig": "超声波模块 TRIG 触发脉冲输出。",
    "obstacle_stop": "合并或当前模块产生的停车请求。",
    "ultrasonic_stop": "超声波产生的停车请求。",
    "lidar_stop": "雷达产生的停车请求。",
    "route_info": "64 bit 路线命令队列；最低 2 bit 是下一条命令。",
    "old_route_info": "已完成路线历史，用于 GUI 和手机显示。",
    "state_info": "路线 FSM 状态编码。",
    "menu_info": "菜单焦点/页面状态编码。",
    "run_step_pulse": "一条运动动作完成的单周期脉冲。",
    "key_up_pulse": "消抖后的上键单周期按下事件；编辑态用于添加前进，菜单态用于切换焦点。",
    "key_down_pulse": "消抖后的下键单周期按下事件；编辑态用于撤销最后一个 UI 动作。",
    "key_left_pulse": "消抖后的左键单周期按下事件；编辑态写入左转加前进复合动作。",
    "key_right_pulse": "消抖后的右键单周期按下事件；编辑态写入右转加前进复合动作。",
    "key_mid_pulse": "消抖后的中键单周期按下事件；用于确认菜单、开始运行或从停车返回。",
    "active_route_steps": "当前待执行内部命令数量。",
    "completed_route_steps": "已完成内部命令数量。",
    "step_length_set": "手机设置的单步长度参数。",
    "speed_set": "路线控制采用的整体速度参数。",
    "step_length": "当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。",
    "speed": "当前生效的路线整体速度参数，向 GUI 和执行控制链同步。",
    "total_route_length": "已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。",
    "complete_route_length": "completed_route_steps 乘以 step_length 得到的已完成路线长度。",
    "remaining_route_length": "active_route_steps 乘以 step_length 得到的剩余路线长度。",
    "now_x": "小车当前逻辑栅格 X 坐标；自动动作完成后由路线核心更新。",
    "now_y": "小车当前逻辑栅格 Y 坐标；自动动作完成后由路线核心更新。",
    "target_x": "编辑路线末端预测 X 坐标；添加或撤销动作时同步变化。",
    "target_y": "编辑路线末端预测 Y 坐标；添加或撤销动作时同步变化。",
    "overall_speed": "手机可调的公共速度基准。",
    "lf_default_speed": "左前轮校准默认速度。",
    "rf_default_speed": "右前轮校准默认速度。",
    "lr_default_speed": "左后轮校准默认速度。",
    "rr_default_speed": "右后轮校准默认速度。",
    "lidar_safe_mm": "手机可调雷达正前方安全距离，单位 mm。",
    "manual_mode": "1 表示手机手动模式覆盖自动路线。",
    "manual_cmd": "手动动作：0停、1前、2后、3左转、4右转。",
    "setting_mode": "手机参数设置模式；为 1 时参数按键优先并锁住路线编辑按键。",
    "selected_param": "当前手机增减操作选中的参数编号 0..6。",
    "route_key_lock": "参数设置期间的路线按键锁；为 1 时实体/蓝牙方向键不会修改路线。",
    "motor_blocked": "当前运动命令被避障逻辑拦截。",
    "motor_stop_pulse": "高优先级强制停车单周期脉冲。",
    "motor_enable": "C5 运动链总使能。",
    "lf_speed": "左前轮 11 bit 二补码有符号目标速度。",
    "rf_speed": "右前轮 11 bit 二补码有符号目标速度。",
    "lr_speed": "左后轮 11 bit 二补码有符号目标速度。",
    "rr_speed": "右后轮 11 bit 二补码有符号目标速度。",
    "frame_count": "已经完整发送的 C5 帧数量。",
    "frame_busy": "1 表示 C5 UART 正在发送完整帧。",
    "stop_frames_complete": "强化停车序列三帧已经完成。",
    "stop_frames_remaining": "尚未发送完成的强化停车帧数量。",
    "force_stop": "高优先级停车请求；到达后等待当前字节/帧边界并优先装载强化停车序列。",
    "c5_bus_tx": "C5 四轮控制总线的 115200 bit/s、8N1 串行输出。",
    "last_lf_pulse": "最近一帧发送给左前轮的 C5 脉宽诊断值，单位 us。",
    "last_rf_pulse": "最近一帧发送给右前轮的 C5 脉宽诊断值，单位 us。",
    "last_lr_pulse": "最近一帧发送给左后轮的 C5 脉宽诊断值，单位 us。",
    "last_rr_pulse": "最近一帧发送给右后轮的 C5 脉宽诊断值，单位 us。",
    "forward_ms": "自动前进动作的后备持续时间，单位 ms；步长参数有效时由换算值覆盖。",
    "turn_ms": "自动原地转向动作的持续时间，单位 ms。",
    "motion_busy": "自动路线动作正在计时执行；为 1 时不会重复启动同一队首命令。",
    "current_cmd": "当前执行或最近锁存的 2 bit 路线命令编码。",
    "lidar_online": "收到有效雷达扫描样本且未超时。",
    "scan_enable": "允许雷达旋转和连续扫描。",
    "sample_valid": "当前周期解析出一个有效扫描节点。",
    "new_scan": "扫描节点的整圈起始标志。",
    "angle_q6": "扫描角度 Q6，单位 1/64 度。",
    "distance_q2": "扫描距离 Q2，单位 1/4 mm。",
    "front_distance_mm": "滤波后的正前方最近距离，FFFF 表示无效。",
    "safe_distance_mm": "正前方可调避障距离，单位 mm。",
    "distance_cm": "超声波距离，单位 cm；FFFF 表示超时无效。",
    "distance_valid": "当前周期更新了超声波测距结果。",
    "echo_timeout": "等待或测量回波超过 30 ms。",
    "ultrasonic_distance_cm": "最近超声波距离，单位 cm；FFFF 表示当前没有有效回波。",
    "ultrasonic_distance_valid": "当前周期出现新的超声测距结果，可用于锁存或上报。",
    "ultrasonic_echo_timeout": "最近一次超声等待/测量超过 30 ms 的超时状态。",
    "lidar_front_distance_mm": "三圈滤波后的正前方最近雷达距离，单位 mm；FFFF 表示无效。",
    "lidar_obstacle_stop": "雷达按车体前方、前角和侧边轮廓判定产生的停车请求。",
    "ultrasonic_obstacle_stop": "前向超声距离低于原有阈值时产生的停车请求。",
    "merged_obstacle_stop": "雷达和超声停车请求按 OR 合并后的最终避障输入。",
    "c5_frame_count": "C5 驱动已经完整发送的运动帧/停车帧累计数量。",
    "c5_stop_frames_complete": "最近一次三帧强化停车序列是否已经全部发送完成。",
    "c5_stop_frames_remaining": "最近一次强化停车序列仍待发送的帧数，范围 0..3。",
    "c5_frame_busy": "C5 驱动当前是否占用 UART 发送一条完整控制帧。",
    "bt_key_up_n": "蓝牙方向字符模拟的低有效上键电平；字符到达后保持约 30 ms。",
    "bt_key_down_n": "蓝牙方向字符模拟的低有效下键电平；字符到达后保持约 30 ms。",
    "bt_key_left_n": "蓝牙方向字符模拟的低有效左键电平；字符到达后保持约 30 ms。",
    "bt_key_right_n": "蓝牙方向字符模拟的低有效右键电平；字符到达后保持约 30 ms。",
    "bt_key_mid_n": "蓝牙确认/停车字符模拟的低有效中键电平；字符到达后保持约 30 ms。",
    "bt_cfg_mode_pulse": "手机切换参数设置模式的单周期事件脉冲。",
    "bt_cfg_next_pulse": "手机选择下一项可调参数的单周期事件脉冲。",
    "bt_cfg_prev_pulse": "手机选择上一项可调参数的单周期事件脉冲。",
    "bt_cfg_inc_pulse": "手机把当前所选参数增加一个固定步长的单周期事件脉冲。",
    "bt_cfg_dec_pulse": "手机把当前所选参数减少一个固定步长的单周期事件脉冲。",
    "bt_cfg_select": "手机直接指定的参数编号 0..6；仅在 bt_cfg_select_valid 脉冲周期被参数控制器采纳。",
    "bt_cfg_select_valid": "bt_cfg_select 携带新参数编号的单周期有效脉冲。",
    "pixel_x": "当前有效显示区水平像素坐标。",
    "pixel_y": "当前有效显示区垂直像素坐标。",
    "pixel_de": "视频数据有效；1 表示位于可见区域。",
    "rgb_data": "GUI 输出或 HDMI 输入的 RGB888 像素。",
    "buzzer_out": "送往有源/无源蜂鸣器的方波输出。",
}


PROCESS_DOCS = {
    "route_contol_core": [
        "组合映射 menu_info：把内部 FSM 状态和菜单焦点转换成 GUI 菜单编码。",
        "组合打包路线数组：数组下标 i 映射到 route_info[i*2 +: 2]。",
        "路线主状态机：处理编辑、撤销、运行出队、坐标更新和障碍停止。",
    ],
    "c5_route_controller": [
        "运动控制主时序：手动模式优先，其次避障，再执行自动路线命令和动作计时。",
    ],
    "c5_motor_frame_driver": [
        "C5 UART 与帧调度联合状态机：先完成当前字节/帧，再处理停车或新运动命令。",
    ],
    "bt_route_uart": [
        "蓝牙 UART 接收器：同步 RX、半位确认起始位、LSB-first 采集 8 bit。",
        "手动/自动模式和雷达开关命令处理，并实现 300 ms 手动运动看门狗。",
        "蓝牙心跳在线判定：仅 P/p 刷新 3 s 链路超时计数。",
        "锁存最近一次有效超声结果，确保状态上报不会读取单周期脉冲的瞬时值。",
        "生成待发送短回复：方向/配置命令回 @Kx，心跳回 @PONG。",
        "把配置字符转换为选择、增减和设置模式的单周期脉冲。",
        "自动模式下把蓝牙方向字符模拟成约 30 ms 的低有效实体按键。",
        "唯一 TX 调度器：原子发送短回复、@S、@L，并在发送前锁存整帧快照。",
    ],
    "linkpg_uwb_bridge": [
        "LinkPG UART 接收器：同步异步 RX，按 115200 8N1 半位确认并产生单字节有效脉冲。",
        "Modbus 帧解析器：识别长度、数据透传标记、目标 ID、CRC16 和 CxxccH 命令载荷。",
        "UWB 在线计时器：CRC 正确帧刷新三秒超时，超时后仅清在线标志而不影响本地安全链。",
        "命令注入器：等待蓝牙 RX 空闲，在位边界完整发送单字节并生成执行/冲突/超时 ACK。",
        "蓝牙 TX 旁路 UART 接收器：从物理发送电平恢复 FPGA 输出的 ASCII 字节。",
        "状态行捕获器：从 @ 开始缓存到换行，限制 96 字节并在完整行到达时产生事件。",
        "ACK 挂起队列：把解析和注入阶段结果锁存到统一 LinkPG 上行发送路径。",
        "遥测分片调度器：将最多 96 字节状态行拆为两段，并保证上一段发送完成后再推进。",
        "上行载荷构造器：生成 A/Ack 或 R/Report ASCII 数据及 CRC8 字段。",
        "LinkPG UART 字节发送器：按 115200 8N1 输出构造好的 Modbus 帧字节。",
        "Modbus 流发送器：原子发送前缀、载荷、CRC16 和帧间隔，防止不同消息交叉。",
    ],
    "motion_sensor_subsystem": [
        "组合命令生成器：根据主状态选择 SPI 器件、寄存器序列、时序间隔和传输长度。",
        "共享 SPI mode-3 引擎：锁存一次命令后驱动五片选、SCLK/MOSI，并收集最多 9 字节 MISO。",
        "传感器主状态机：依次初始化 PMW3901/BMI088、采样双 IMU 和光流、校准偏置并融合。",
        "@M 遥测调度器：每 100 ms 锁存一次一致快照，再按固定字符索引启动 UART 字节发送。",
        "调试 UART 位发送器：按 115200 8N1 LSB-first 输出一个 ASCII 字节。",
    ],
    "rplidar_c1_uart": [
        "雷达 UART 接收器：以 460800 波特采样并输出 rx_data/rx_valid。",
        "雷达 UART 发送器：发送 A5 25、A5 20 等两字节命令。",
        "雷达启停状态机：控制启动延时、描述符匹配、在线超时和重新启动。",
        "扫描数据面：组装 5 字节节点、按圈统计八扇区、三样本中值并判断轮廓障碍。",
    ],
    "ultrasonic_obstacle_guard": [
        "双触发器同步异步 ECHO 输入，降低亚稳态传播风险。",
        "超声测距状态机：TRIG、等待上升沿、测量高电平和测量间隔。",
    ],
    "hdmi_timing_gen": [
        "水平像素计数器，在每行 H_TOTAL-1 后回零。",
        "垂直行计数器，仅在一行结束时递增。",
        "寄存输出同步、数据有效和坐标，避免长组合路径直接进入 TMDS 编码器。",
    ],
    "hdmi_tmds_encoder": [
        "TMDS 编码时序：消隐期输出控制码；有效期更新 10 bit 码和 running disparity。",
    ],
    "hdmi_serializer": [
        "像素域锁存新的 10 bit TMDS 字，并通过 toggle 通知 5 倍频时钟域。",
        "跨时钟域同步 toggle，形成一次 load_pulse。",
        "5 倍频域每周期输出两位给 ODDR，五周期发送完整 10 bit。",
    ],
    "car_param_controller": [
        "参数寄存器：处理模式切换、参数选择、增减脉冲和饱和上下限。",
    ],
    "power_on_reset": ["上电/复位计数器：外部复位立即拉低，计满后只释放一次。"],
    "sys_route_control_core": [
        "组合映射 menu_info：根据 state_info 和 focus_run 选择菜单焦点/进入状态编码。",
        "旧版路线主状态机：直接移位 64 bit route_info，维护撤销边界、历史路线、坐标和运行出队。",
    ],
    "clk_wiz_0": [
        "综合硬件锁定延时：MMCM 锁定后再累计 50,000 个 clk_out1 周期才置 stable_locked。",
        "非综合仿真锁定模型：resetn 释放后延时 100 ns 置 is_locked。",
        "非综合仿真 50 MHz 时钟模型：每 10 ns 翻转 clk_50m。",
        "非综合仿真 250 MHz 时钟模型：每 2 ns 翻转 clk_250m。",
    ],
}


INITIAL_DOCS = {
    "power_on_reset": "为支持 FPGA 上电即进入确定复位状态，初始化 por_count=0、system_rst_n=0；外部复位时序过程随后接管。",
}


FUNCTION_DOCS = {
    ("c5_route_controller", "adjusted_speed"): "叠加公共速度和单轮校准偏置，并限制到 C5 可用范围。",
    ("c5_route_controller", "turn_mag"): "转向降速函数，降低原地旋转的突发冲击。",
    ("c5_route_controller", "pos_speed"): "把无符号幅值扩展成正的 11 bit 二补码。",
    ("c5_route_controller", "neg_speed"): "把无符号幅值转换为负的 11 bit 二补码。",
    ("c5_route_controller", "ms_to_cycles"): "把毫秒动作时间换算为 50 MHz 周期终值。",
    ("c5_route_controller", "step_to_ms"): "用移位加减实现 step_length*125，避免综合乘法器。",
    ("c5_motor_frame_driver", "left_motor_pulse"): "左侧安装方向映射：1500 + signed_speed。",
    ("c5_motor_frame_driver", "right_motor_pulse"): "右侧安装方向映射：1500 - signed_speed。",
    ("c5_motor_frame_driver", "digit4"): "把 0..2400 左右的脉宽拆成四个 ASCII 十进制数字。",
    ("c5_motor_frame_driver", "frame_char"): "按发送索引即时生成 62 字节运动帧或 9 字节停车帧字符。",
    ("c5_motor_frame_driver", "speed_abs"): "读取 11 bit 二补码轮速的符号位，返回对应无符号幅值供左右轮脉宽映射使用。",
    ("bt_route_uart", "status_char"): "按字符索引编码 @S 或 @L；这是手机协议字段定义的唯一权威位置。",
    ("bt_route_uart", "hex_char"): "把 4 bit 半字节转换为大写 ASCII 十六进制字符 0..9/A..F。",
    ("bt_route_uart", "normalized_key"): "把 W/U、S/D、A/L、R、C/M 同义字符归一化为 U/D/L/R/M。",
    ("bt_route_uart", "cfg_command"): "判断接收字符是否属于参数选择、增减、模式切换或雷达开关命令集合。",
    ("bt_route_uart", "reply_char"): "生成不会与长状态帧交叉的 @Kx 或 @PONG 短回复。",
    ("linkpg_uwb_bridge", "crc16_modbus_next"): "逐字节更新 LinkPG 帧使用的 Modbus CRC16（反射多项式 A001）。",
    ("linkpg_uwb_bridge", "crc8_next"): "逐字节更新命令/ACK/遥测载荷内部使用的 CRC8（多项式 07）。",
    ("linkpg_uwb_bridge", "is_hex_ascii"): "判断载荷字符是否为十六进制 0-9/A-F/a-f。",
    ("linkpg_uwb_bridge", "hex_value"): "把十六进制 ASCII 字符转换为 4 bit 数值。",
    ("linkpg_uwb_bridge", "hex_ascii"): "把 4 bit 数值转换为大写十六进制 ASCII。",
    ("linkpg_uwb_bridge", "allowed_command"): "UWB 下行白名单，只允许现有蓝牙协议已经定义的单字符命令。",
    ("linkpg_uwb_bridge", "command_crc8"): "计算 C+序号+命令三个字节的下行命令 CRC8。",
    ("linkpg_uwb_bridge", "ack_crc8"): "计算 A+序号+命令+结果四字节 ACK 的 CRC8。",
    ("linkpg_uwb_bridge", "prefix_byte"): "按字节索引生成 LinkPG Modbus 数据透传请求头。",
    ("linkpg_uwb_bridge", "suffix_byte"): "按当前载荷和 CRC 累计值生成 Modbus 帧尾 CRC16。",
    ("motion_sensor_subsystem", "abs16"): "返回 16 bit 二补码输入的无符号绝对值，用于静止和异常门限。",
    ("motion_sensor_subsystem", "abs_diff16"): "计算两个 16 bit 有符号样本差的 17 bit 绝对值。",
    ("motion_sensor_subsystem", "abs_diff17"): "计算两个 17 bit 校正陀螺值差的 18 bit 绝对值。",
    ("motion_sensor_subsystem", "saturate16"): "把 17 bit 融合中间值饱和到 16 bit 有符号输出范围。",
    ("motion_sensor_subsystem", "pmw_init_word"): "按初始化索引返回 PMW3901 寄存器地址和值。",
    ("motion_sensor_subsystem", "hex_ascii"): "把 4 bit 半字节转换为 @M 使用的大写十六进制 ASCII。",
    ("motion_sensor_subsystem", "hex8_char"): "按字符位置输出 8 bit 数值的两个十六进制字符。",
    ("motion_sensor_subsystem", "hex16_char"): "按字符位置输出 16 bit 数值的四个十六进制字符。",
    ("motion_sensor_subsystem", "hex32_char"): "按字符位置输出 32 bit 数值的八个十六进制字符。",
    ("motion_sensor_subsystem", "debug_frame_char"): "按固定索引生成 99 字节 @M 标定遥测帧中的当前 ASCII 字符。",
    ("rplidar_c1_uart", "q2_to_mm"): "Q2 距离除以 4，并对无效 FFFF 保持无效标记。",
    ("rplidar_c1_uart", "median3_q2"): "三输入无乘除中值滤波，抑制单圈跳点。",
    ("car_param_controller", "next_param"): "在整体速度、步长、四轮默认速度和雷达阈值七项之间循环选择下一项。",
    ("car_param_controller", "prev_param"): "在七项手机可调参数之间反向循环选择，PARAM_OVERALL 的前一项回绕到 PARAM_LIDAR。",
    ("car_param_controller", "inc_speed"): "把速度增加 25，并在 900 上限处饱和，避免参数溢出。",
    ("car_param_controller", "dec_speed"): "把速度减少 25，并在 100 下限处饱和，避免参数下溢。",
    ("car_param_controller", "inc_step"): "把路线单步长度增加 5，并在 80 上限处饱和。",
    ("car_param_controller", "dec_step"): "把路线单步长度减少 5，并在 5 下限处饱和。",
    ("car_param_controller", "inc_lidar_safe"): "把雷达正前方安全距离增加 50 mm，并在 1500 mm 上限处饱和。",
    ("car_param_controller", "dec_lidar_safe"): "把雷达正前方安全距离减少 50 mm，并在 150 mm 下限处饱和。",
    ("gui_route_renderer", "calc_step_pix"): "根据路线步数分段选择 58/44/34/26/20/16 像素网格间距，使完整路线尽量落入屏幕地图区。",
    ("gui_route_renderer", "clamp_bar_w"): "把信息条宽度限制在 0..320 像素，防止数值超出右侧信息栏边界。",
    ("gui_route_renderer", "scaled_offset"): "把以 128 为原点的逻辑坐标差按当前网格间距换算为有符号像素偏移；乘法用移位相加实现。",
    ("gui_route_renderer", "segment_hit"): "判断当前 pixel_x/pixel_y 是否落在给定水平或垂直路线线段及其线宽范围内。",
    ("gui_route_renderer", "glyph5x7_2x_hit"): "查询内置 5x7 字模并放大到 10x14 像素，返回当前像素是否应点亮。",
    ("gui_route_renderer", "dec5_char"): "用重复减法把 16 bit 数值拆成五位十进制，并按 idx 返回指定 ASCII 数字。",
    ("gui_route_renderer", "dec3_char"): "把 8 bit 数值拆成三位十进制，并按 idx 返回百/十/个位 ASCII 数字。",
    ("gui_route_renderer", "dec2_char"): "把 8 bit 数值拆成两位十进制，并按 idx 返回十位或个位 ASCII 数字。",
    ("gui_route_renderer", "slot12"): "把水平像素偏移按每 12 像素一格量化到 0..15 字符槽位。",
    ("gui_route_renderer", "hex16_char"): "从 64 bit 路线字段中按高位优先选择一个半字节并输出大写十六进制字符。",
    ("gui_route_renderer", "text_word_char"): "根据 WORD_* 文本编号和字符下标查表输出菜单/状态英文字符。",
    ("gui_route_renderer", "cmd_word_char"): "把 2 bit 路线命令转换为 LEFT/RIGHT/FWD/STOP 四字符显示文本。",
}


TASK_DOCS = {
    ("route_contol_core", "move_target_forward"): "按 target_dir 将编辑终点前移一个逻辑栅格。",
    ("route_contol_core", "move_target_backward"): "撤销前进时将编辑终点后退一个逻辑栅格。",
    ("route_contol_core", "move_now_forward"): "动作完成时按 now_dir 更新小车当前位置。",
    ("route_contol_core", "clear_active_route"): "清空待执行路线，并把目标位置重新对齐当前位置。",
    ("c5_route_controller", "stop_motors"): "原子地把四路目标速度全部写为零。",
    ("sys_route_control_core", "apply_route_cmd_to_target"): "把一条 2 bit 路线命令应用到预测终点和 target_dir；转向只改朝向，前进只改坐标。",
    ("sys_route_control_core", "apply_route_cmd_to_now"): "把一条已完成命令应用到当前坐标和 now_dir，重建小车实际逻辑位置。",
    ("sys_route_control_core", "clear_active_route"): "清空旧版待执行队列及 32 项撤销元数据，并把预测终点重新对齐当前位置。",
    ("sys_route_control_core", "rebuild_target_from_active"): "从当前位置起重放 route_info 前 limit_count 条命令，重新计算撤销后的预测终点和朝向。",
}


INTERNAL_SIGNAL_DOCS = {
    "route_contol_core": {
        "i": "路线数组循环下标；组合打包 route_info/old_route_info、复位清零队列以及运行出队左移时遍历 0..ROUTE_WORDS-1。",
        "focus_run": "菜单焦点：0 选择编辑路线，1 选择开始运行。",
        "now_dir": "小车当前朝向，采用 DIR_UP/RIGHT/DOWN/LEFT 编码。",
        "target_dir": "编辑路线末端的预测朝向。",
        "step_count": "用户编辑动作数；转向+前进虽然占两个命令，但只算一个 UI 动作。",
        "step_is_turn": "每个 UI 动作是否为转向复合动作，用于一次撤销两个命令。",
        "route_cmd_mem": "待执行命令数组；索引 0 是自动运行的队首。",
        "old_route_cmd_mem": "已经执行完成的命令历史，用于屏幕显示走过的路径。",
    },
    "c5_route_controller": {
        "elapsed_cycles": "当前自动动作已经持续的 50 MHz 时钟周期数。",
        "target_cycles": "当前动作计划持续的总周期数。",
        "restart_pending": "动作结束后的单周期隔离标志，避免同一命令立即重复启动。",
        "in_run": "state_info 是否等于 STATE_RUN。",
        "motion_done": "动作计时达到 target_cycles 的组合完成条件。",
        "effective_forward_speed": "整体速度为零时回退到固定 forward_speed，否则使用手机参数。",
        "effective_turn_speed": "整体速度为零时回退到固定 turn_speed，否则使用手机参数。",
    },
    "c5_motor_frame_driver": {
        "effective_lf_speed": "motor_enable=1 时等于左前目标速度，否则强制为零；后续变化检测和组帧只使用该值。",
        "effective_rf_speed": "motor_enable=1 时等于右前目标速度，否则强制为零；后续变化检测和组帧只使用该值。",
        "effective_lr_speed": "motor_enable=1 时等于左后目标速度，否则强制为零；后续变化检测和组帧只使用该值。",
        "effective_rr_speed": "motor_enable=1 时等于右后目标速度，否则强制为零；后续变化检测和组帧只使用该值。",
        "command_changed": "任一当前有效轮速与最近一次已锁存发送轮速不同时为 1，用于立即请求新运动帧。",
        "refresh_due": "小车正在运动且 100 ms 刷新计数到期；即使命令未变化也补发当前四轮帧。",
        "any_motion": "四个当前有效轮速中至少一个非零的组合判定。",
        "last_any_motion": "最近一次锁存的四轮速度中至少一个非零，用于识别从运动切换到零速的边沿。",
        "stop_transition": "当前四轮已全为零而上一组锁存速度仍有运动时为 1，触发强化停车序列。",
        "send_stop_now": "强制停车、运动到停止、仍有停车帧待发或当前零速时提出的停车帧优先请求。",
        "active_frame_last": "当前帧最后字符下标：停车帧选择 STOP_FRAME_LAST，四轮运动帧选择 FRAME_LAST。",
        "last_lf_speed": "最近一次已锁存发送的左前目标速度，用于检测命令变化。",
        "last_rf_speed": "最近一次已锁存发送的右前目标速度，用于检测命令变化。",
        "last_lr_speed": "最近一次已锁存发送的左后目标速度，用于检测命令变化。",
        "last_rr_speed": "最近一次已锁存发送的右后目标速度，用于检测命令变化。",
        "frame_lf_pulse": "当前帧锁存的左前 C5 脉宽，帧发送中途不会随输入改变。",
        "frame_rf_pulse": "当前帧锁存的右前 C5 脉宽。",
        "frame_lr_pulse": "当前帧锁存的左后 C5 脉宽。",
        "frame_rr_pulse": "当前帧锁存的右后 C5 脉宽。",
        "refresh_count": "运动期间 100 ms 周期刷新计数器。",
        "send_index": "当前完整帧正在发送的字符下标。",
        "sending": "完整 C5 帧互斥标志；为 1 时不得切换到另一帧。",
        "pending_send": "发送期间检测到新命令时暂存请求，当前帧结束后处理。",
        "frame_is_stop": "当前锁存帧类型：1 为 #255PDST!，0 为四轮 62 字节运动帧。",
        "startup_count": "上电 500 ms 安全等待计数器。",
        "startup_ready": "启动等待完成后锁存为 1。",
        "uart_state": "底层单字节 UART 的 IDLE/START/DATA/STOP 状态。",
    },
    "bt_route_uart": {
        "rx_sync": "两级同步后的异步蓝牙 RX，降低亚稳态风险。",
        "rx_data": "最近完整接收的 ASCII 命令字节。",
        "rx_valid": "仅在一个 UART 字节通过停止位检查后脉冲一个周期。",
        "status_count": "200 ms 周期状态上报计数器。",
        "status_index": "@S/@L 当前发送字符位置。",
        "sending_status": "长状态帧发送互斥锁；防止 @PONG 或 @K 插入其中。",
        "sending_reply": "短回复发送互斥锁。",
        "pending_reply": "已经收到命令但尚未开始发送 @K/@PONG。",
        "link_idle_count": "距离最近 P/p 心跳经过的周期数。",
        "link_active": "心跳在线锁存状态，对外成为 bt_connected。",
        "manual_timeout_count": "手动运动命令 300 ms 看门狗计数。",
        "manual_watchdog_expired": "看门狗曾把运动命令强制改为停止。",
        "route_status_pending": "要求尽快发送一帧最新 @S 的挂起标志。",
        "route_status_seq": "每次锁存 @S 时自增的 8 bit 序号。",
        "lidar_status_seq": "每次锁存 @L 时自增的 8 bit 序号。",
        "active_status_seq": "当前正在发送帧的序号快照。",
        "active_status_lidar": "当前长帧类型锁存：0 发送 @S 路线/参数状态，1 发送 @L 八扇区雷达状态。",
        "status_packet_lidar": "周期上报轮换选择：0 表示下一周期优先 @S，1 表示下一周期优先 @L。",
        "status_left": "当前 @S/@L 长帧尚待调度的字符数量；递减到零后释放 sending_status 互斥锁。",
        "status_due": "200 ms 周期计数已经到期的组合条件；仅提出上报请求，不直接抢占正在发送的帧。",
        "status_route_info": "开始发送 @S 时锁存的路线快照，保证整帧字段一致。",
        "status_lidar_sector_mm": "开始发送 @L 时锁存的八方向距离快照。",
        "reply_kind": "最新接收命令产生的短回复类型：按键确认 @Kx、心跳 @PONG 或无回复。",
        "active_reply_kind": "短回复开始发送时锁存的回复类型，保证发送途中不被后续命令覆盖。",
        "reply_key": "待回复的归一化按键字符；用于形成 @K<key> 换行帧。",
        "active_reply_key": "当前 @K 帧锁存的按键字符，直到整帧发完才允许更新。",
        "reply_index": "当前 @K/@PONG 短回复正在发送的字符下标。",
        "reply_done": "当前短回复最后一个字符已经发送完成的组合条件。",
        "tx_done": "底层 UART busy 由 1 变 0 的单周期完成脉冲；TX 调度器据此推进字符索引。",
        "norm_key": "把同义方向字符归一化后的 U/D/L/R/M 控制字符；无效输入归一化为零。",
        "is_cfg_command": "当前接收字节是否属于参数选择、增减、模式或雷达开关命令。",
        "route_status_changed": "路线、位置、目标、状态、模式或可调参数相对上一快照发生变化的组合标志。",
        "ultrasonic_last_distance_cm": "最近一次有效超声测距结果，单位 cm；用于跨越 distance_valid 单周期脉冲持续上报。",
        "ultrasonic_last_valid": "已经至少锁存过一次有效超声测量的标志；复位前保持有效。",
        "ultrasonic_last_timeout": "最近一次超声测量是否超时；与 ultrasonic_last_distance_cm 一起进入 @S 状态帧。",
        "ultrasonic_last_stop": "最近一次超声测量对应的停车判定锁存值；用于 @S 上报障碍来源。",
        "up_hold_count": "把蓝牙前进字符模拟为实体上键低电平的剩余保持周期数，约 30 ms。",
        "down_hold_count": "把蓝牙后退/撤销字符模拟为实体下键低电平的剩余保持周期数，约 30 ms。",
        "left_hold_count": "把蓝牙左转字符模拟为实体左键低电平的剩余保持周期数，约 30 ms。",
        "right_hold_count": "把蓝牙右转字符模拟为实体右键低电平的剩余保持周期数，约 30 ms。",
        "mid_hold_count": "把蓝牙确认/停车字符模拟为实体中键低电平的剩余保持周期数，约 30 ms。",
        "tx_state": "唯一蓝牙 UART 发送器的 IDLE/START/DATA/STOP 状态。",
    },
    "linkpg_uwb_bridge": {
        "uwb_frame_index": "当前 LinkPG Modbus 帧已经接收的字节下标。",
        "uwb_frame_expected": "从功能码和长度字段推导出的完整 Modbus 帧字节数。",
        "uwb_frame_byte_count": "当前帧数据区声明的载荷字节数。",
        "uwb_frame_crc": "从帧首开始逐字节累计的 Modbus CRC16。",
        "uwb_payload": "缓存下行 CxxccH 命令载荷的 8 字节数组。",
        "uwb_frame_idle_count": "不完整帧最后一个字节后的空闲周期数；10 ms 后丢弃截断帧并重新找帧首。",
        "new_command_valid": "解析出 CRC 正确且通过白名单的新命令时产生的单周期脉冲。",
        "new_command_sequence": "下行命令携带的 8 bit 序号，用于 ACK 和重复抑制。",
        "new_command_byte": "准备注入现有 bt_route_uart 的单个 ASCII 命令。",
        "last_command_sequence": "最近一次已接受命令的序号；两秒窗口内相同序号不会再次执行。",
        "duplicate_age": "最近命令距离当前的周期数，用于界定重复抑制有效窗口。",
        "uwb_link_idle_count": "距离最近 CRC 正确 LinkPG 帧的周期数，达到三秒后清除在线标志。",
        "command_pending": "已有一条 UWB 命令等待蓝牙 RX 空闲或正在注入。",
        "pending_command_sequence": "注入过程锁存的命令序号，直到 ACK 完成前保持不变。",
        "pending_command_byte": "注入过程锁存的 ASCII 字节。",
        "command_wait_count": "等待蓝牙物理 RX 空闲的 100 ms 超时计数。",
        "bt_idle_high_count": "蓝牙 RX 已连续保持高电平的周期数；至少 12 bit 才允许开始注入。",
        "inject_collision": "注入期间蓝牙物理 RX 被外部拉低，表示手机与 UWB 同时发送产生冲突。",
        "report_buffer": "缓存 FPGA 蓝牙 TX 完整 ASCII 行的 96 字节寄存器数组。",
        "report_capture_active": "已看到 @ 行首且正在等待换行的捕获状态。",
        "report_capture_length": "当前 @S/@L/@K/@PONG 行已缓存的字节数。",
        "report_pending": "已有完整状态行等待构造一个或两个 UWB 上行分片。",
        "report_message_sequence": "每条完整上行状态行自增的 8 bit 序号，两个分片共享该值。",
        "report_part": "当前状态行分片编号：0 为前 64 字节，1 为剩余字节。",
        "tx_payload_buffer": "准备封装到 LinkPG Modbus 帧的数据透传 ASCII 载荷。",
        "tx_payload_ready": "载荷和内部 CRC8 已构造完成，等待 Modbus 流发送器取走。",
        "stream_state": "上行 Modbus 流状态：空闲、前缀、载荷、后缀和帧间等待。",
        "tx_gap_count": "相邻 LinkPG Modbus 帧之间的 25 ms 保护间隔计数。",
    },
    "motion_sensor_subsystem": {
        "spi_cs_n_raw": "五个 SPI 从设备的低有效片选寄存器，bit0..4 依次对应 PMW、BMI0 ACC/GYRO、BMI1 ACC/GYRO。",
        "pmw_force_cs_low": "PMW3901 上电初始化要求片选保持低的特殊阶段控制。",
        "spi_state": "共享 SPI mode-3 引擎的空闲、建立、驱动、采样、字节间隔、保持和结束状态。",
        "spi_length_latched": "一次 SPI 命令锁存的总字节数，传输开始后不再受主状态机修改。",
        "spi_tx_shift": "最多 9 字节的 SPI 发送移位寄存器，最高有效字节先发送。",
        "spi_rx_shift": "SPI 时钟采样期间逐位移入的临时 MISO 数据。",
        "spi_rx_data": "一条 SPI 命令完成时锁存的最多 9 字节返回数据。",
        "main_state": "传感器初始化与轮询总状态，覆盖 PMW 初始化、BMI 配置、双 IMU 和光流采样。",
        "command_inflight": "主状态机已经提交一条 SPI 命令，等待 spi_done 后才允许推进。",
        "pmw_init_index": "PMW3901 参考初始化表当前寄存器项下标。",
        "pmw_retry_count": "PMW3901 ID 或初始化失败后的有限重试次数。",
        "bmi_init_step": "两块 BMI088 加速度计和陀螺仪配置序列的当前步骤。",
        "pair_sample_strobe": "两块 BMI088 一轮采样均完成后产生的融合触发脉冲。",
        "flow_data_seen": "至少接收过一次 PMW3901 光流采样；进入 @M 状态 bit7。",
        "bmi0_bad_sample_count": "第一块 BMI088 连续全零/全一坏样本计数，达到门限后判离线。",
        "bmi1_bad_sample_count": "第二块 BMI088 连续全零/全一坏样本计数，达到门限后判离线。",
        "calibration_count": "启动静止陀螺偏置已经累计的样本数，目标为 512。",
        "bmi0_bias_sum_x": "第一块 BMI088 X 轴启动偏置的 512 样本有符号累加和。",
        "bmi0_gyro_bias_x": "第一块 BMI088 X 轴由启动样本均值得到的陀螺零偏。",
        "bmi1_gyro_bias_x": "第二块 BMI088 X 轴由启动样本均值得到的陀螺零偏。",
        "calibration_stationary": "两块陀螺三轴原始值均未超过启动静止门限时为 1。",
        "gyro_diff_x": "两块已扣偏置 X 轴陀螺的绝对差，用于一致性判定。",
        "pmw_init_entry": "pmw_init_word 当前返回的寄存器地址/数值组合。",
        "debug_period_count": "@M 遥测 100 ms 周期计数器。",
        "debug_sending": "固定 99 字节 @M 帧正在原子发送，期间快照不会被新采样覆盖。",
        "debug_char_index": "当前 @M 帧正在发送的字符下标。",
        "snap_status": "开始一帧 @M 时锁存的在线、校准、融合、降级、分歧和光流状态位。",
        "snap_flow_accum_x": "开始一帧 @M 时锁存的 32 bit 累计光流 X，保证整行字段一致。",
        "snap_gyro_z": "开始一帧 @M 时锁存的融合 Z 轴陀螺值。",
    },
    "rplidar_c1_uart": {
        "cmd_state": "雷达启动/停止/描述符等待/扫描/重试状态。",
        "desc_index": "已经连续匹配正确的 7 字节扫描描述符位置。",
        "scan_active": "描述符通过后允许把接收字节解释为扫描节点。",
        "byte_timeout_count": "扫描链路无字节超时计数，达到 2 s 后判离线。",
        "wait_count": "雷达命令状态机通用等待计数器；用于 500 ms 上电稳定、100 ms 停止间隔和重试等待。",
        "node_index": "当前 5 字节标准扫描节点的字节位置。",
        "node0": "扫描节点第 0 字节：起始/反起始位和 6 bit 质量。",
        "node1": "扫描节点第 1 字节：角度低位及固定校验位。",
        "node2": "扫描节点第 2 字节：角度高 8 bit。",
        "node3": "扫描节点第 3 字节：距离低 8 bit。",
        "front_min_q2": "当前一圈正前方有效点的最小 Q2 距离。",
        "sector_min_0_q2": "当前一圈前方 45 度扇区最小 Q2 距离。",
        "sector_min_1_q2": "当前一圈右前 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。",
        "sector_min_2_q2": "当前一圈右侧 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。",
        "sector_min_3_q2": "当前一圈右后 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。",
        "sector_min_4_q2": "当前一圈后方 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。",
        "sector_min_5_q2": "当前一圈左后 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。",
        "sector_min_6_q2": "当前一圈左侧 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。",
        "sector_min_7_q2": "当前一圈左前 45 度扇区最小 Q2 距离；FFFF 表示本圈没有有效点。",
        "sector_hist_1_q2": "八扇区上一圈结果。",
        "sector_hist_2_q2": "八扇区上上圈结果，与当前圈共同做 median3。",
        "front_hist_1_q2": "正前方上一圈最小值。",
        "front_hist_2_q2": "正前方上上圈最小值。",
        "obstacle_next": "使用本圈/历史滤波距离组合计算的下一拍停车判定。",
        "node_start": "扫描节点 byte0 的整圈起始位 S。",
        "node_inverse_start": "扫描节点 byte0 的反起始位 not-S；必须与 node_start 互斥。",
        "node_good": "起始位互斥且 byte1 固定位为 1 的扫描节点结构校验结果。",
        "node_quality": "扫描节点 byte0[7:2] 的 6 bit 回波质量；零质量测点会被丢弃。",
        "node_angle_q6": "由 node2 和 node1[7:1] 拼出的 15 bit Q6 扫描角度，单位 1/64 度。",
        "rx_node_distance_q2": "由当前 rx_data 高字节和 node3 低字节拼出的 16 bit Q2 扫描距离。",
        "node_in_front": "节点角度落入跨 0 度的正前方窄扇区时为 1，用于独立 front_min_q2。",
        "rx_node_distance_good": "质量非零且距离位于 50..12000 mm 有效范围内的测点判定。",
        "tx_done": "雷达命令 UART 的 tx_busy 下降沿脉冲，状态机据此发送命令第二字节或进入等待。",
        "safe_distance_q2": "手机设置的正前方安全距离从 mm 左移两位得到的 Q2 阈值。",
        "corner_distance_q2": "正前安全距离加前角余量后的 Q2 阈值；加法溢出时饱和到 FFFF。",
        "node_sector": "按 Q6 角度映射得到的八方向扇区编号：0前、1右前、2右、3右后、4后、5左后、6左、7左前。",
    },
    "hdmi_timing_gen": {
        "h_cnt": "同一声明包含两个扫描计数器：h_cnt 是当前行内像素周期位置 0..H_TOTAL-1，v_cnt 是当前帧内行位置 0..V_TOTAL-1。",
        "v_cnt": "当前帧内行位置，范围 0..V_TOTAL-1。",
    },
    "car_param_controller": {
        "selected_param": "当前手机增减命令作用的参数索引 0..6。",
        "setting_mode": "参数设置模式；为 1 时路线按键被锁定。",
    },
    "sys_route_control_core": {
        "i": "旧版路线数组循环下标；用于清零 32 项撤销元数据、重放有效命令重建目标位置，以及撤销时定位需要清除的命令槽。",
        "focus_run": "旧版菜单焦点：0 选择路线编辑，1 选择开始运行。",
        "now_dir": "旧版路线核心记录的小车当前朝向，采用 DIR_UP/RIGHT/DOWN/LEFT 模 4 编码。",
        "target_dir": "旧版路线队列全部执行后预测的小车朝向；编辑和撤销时重新计算。",
        "step_count": "旧版用户动作数量；用于索引 step_start/step_words 撤销元数据。",
        "step_start": "32 项 UI 动作各自在 64 bit route_info 中的起始命令槽下标。",
        "step_words": "32 项 UI 动作各占用的内部命令数；前进为 1，转向加前进为 2。",
        "undo_start": "组合读取的最后一个 UI 动作起始命令槽，用于一次撤销整组命令。",
        "undo_words": "组合读取的最后一个 UI 动作命令数量，决定撤销时清除一个还是两个槽。",
        "run_cmd": "旧版自动运行时从 route_info[1:0] 读取的当前队首命令。",
    },
    "clk_wiz_0": {
        "clkfb_mmcm": "MMCME2_ADV 的未缓冲反馈输出，送入反馈 BUFG。",
        "clkfb_bufg": "反馈 BUFG 输出，回接 MMCME2_ADV CLKFBIN 以闭合补偿环路。",
        "clk_pix_mmcm": "MMCME2_ADV CLKOUT0 产生的未缓冲 50 MHz 像素时钟。",
        "clk_5x_mmcm": "MMCME2_ADV CLKOUT1 产生的未缓冲 250 MHz 串化时钟。",
        "mmcm_locked": "MMCME2_ADV 原始 LOCKED 输出；稳定延时计数从它拉高时开始。",
        "lock_count": "MMCM 原始 locked 拉高后继续等待 50,000 个 50 MHz 周期的稳定计数器。",
        "stable_locked": "综合硬件最终时钟就绪标志；只有 MMCM 锁定并完成额外等待后才置 1。",
        "clk_50m": "仅在 `ifndef SYNTHESIS 分支使用的 50 MHz 仿真时钟变量，每 10 ns 翻转。",
        "clk_250m": "仅在 `ifndef SYNTHESIS 分支使用的 250 MHz 仿真时钟变量，每 2 ns 翻转。",
        "is_locked": "仅在 `ifndef SYNTHESIS 分支使用的仿真锁定标志，复位释放 100 ns 后置 1。",
    },
    "gpio_key_adapter": {
        "sw1_n": "按 key_active_low 归一化后的物理 key_pin1 低有效电平，后续映射为上键。",
        "sw2_n": "按 key_active_low 归一化后的物理 key_pin2 低有效电平，后续映射为左键。",
        "sw3_n": "按 key_active_low 归一化后的物理 key_pin3 低有效电平，后续映射为右键。",
        "sw4_n": "按 key_active_low 归一化后的物理 key_pin4 低有效电平，后续映射为下键。",
        "sw5_n": "按 key_active_low 归一化后的物理 key_pin5 低有效电平，后续映射为中键。",
    },
    "hdmi_serializer": {
        "pix_word": "像素时钟域锁存的完整 10 bit TMDS 码字；锁存后由 pix_toggle 通知 250 MHz 域取走。",
        "pix_toggle": "像素域每接收一个新 TMDS 码字翻转一次的事件位；以电平翻转方式跨时钟域传递“新字到达”。",
        "pix_toggle_sync0": "pix_toggle 进入 250 MHz 域后的第一级同步采样，用于降低跨时钟域亚稳态传播风险。",
        "pix_toggle_sync1": "pix_toggle 的第二级同步采样；与 sync2 不同时表示检测到一次新的像素码字。",
        "pix_toggle_sync2": "pix_toggle_sync1 的上一周期值；和 sync1 异或形成单周期 load_word_5x。",
        "shift_word": "250 MHz 域正在串化的 10 bit TMDS 移位寄存器；每周期右移两位供 ODDR 双沿输出。",
        "pair_idx": "当前 10 bit TMDS 字已输出的双 bit 组编号 0..4；五个 clk_5x 周期发送完一个字。",
        "ser_d1": "当前 clk_5x 上升沿由 ODDR 输出的 TMDS 串行 bit。",
        "ser_d2": "当前 clk_5x 下降沿由 ODDR 输出的 TMDS 串行 bit。",
        "reset_sync_5x": "把异步低有效复位同步释放到 250 MHz 域的两级移位寄存器；bit1 形成 rst_5x_n。",
        "ser_data_out": "ODDR 产生、尚未经过差分缓冲的单端 TMDS 串行码流；随后由 OBUFDS 转成 ser_p/ser_n。",
        "data_latched_pix": "像素域锁存的完整 10 bit TMDS 字。",
        "data_toggle_pix": "每锁存一个新像素翻转一次的跨域事件标志。",
        "toggle_sync_5x": "进入 5 倍频域的双级同步 toggle。",
        "shift_reg": "正在按每周期两位移出的 10 bit TMDS 移位寄存器。",
        "pair_index": "当前输出第 0..4 组双 bit。",
        "load_word_5x": "像素 toggle 两级同步值不相等时产生的新 10 bit 字装载脉冲。",
        "rst_5x_n": "进入 250 MHz 域并同步释放后的低有效复位。",
    },
    "hdmi_tmds_encoder": {
        "running_disparity": "已发送码流的直流不平衡累计值，用于选择当前码字极性。",
        "q_m": "第一阶段 8 bit XOR/XNOR 转换结果。",
        "n1d": "原始输入字节中逻辑 1 的数量。",
        "n1q_m": "q_m[7:0] 中逻辑 1 的数量。",
        "use_xnor": "输入 1 的数量较多时选择 XNOR 链，减少第一阶段码字翻转次数。",
        "n0q_m": "q_m[7:0] 中逻辑 0 的数量，由 8-n1q_m 得到。",
        "cnt": "TMDS 当前码流的 4 bit 有符号 running disparity 累计值；消隐时清零，有效像素时决定是否反相。",
    },
    "key_pulse_filter": {
        "key_pressed": "两级同步后的低有效物理按键即时按下判定；key_sync[1]=0 时为 1，尚未经过 20 ms 消抖。",
        "key_level": "经过连续 1,000,000 个 50 MHz 周期确认后的稳定按下状态；1 表示按下。",
        "key_level_d": "key_level 的上一周期值，用于只在稳定状态 0->1 时输出一次 key_pulse。",
        "stable_cnt": "原始同步按键与当前 key_level 不一致时的消抖持续计数；达到约 20 ms 才接受变化。",
    },
    "motor_route_controller": {
        "lf_count": "旧底盘当前动作期间累计的左前轮编码器脉冲数；启动新动作时清零，达到 target_ticks 后用于 motion_done 判定。",
        "rf_count": "旧底盘当前动作期间累计的右前轮编码器脉冲数；与其余三轮计数共同决定动作是否完成。",
        "lr_count": "旧底盘当前动作期间累计的左后轮编码器脉冲数；只在 motion_busy 且 lr_tick 到达时递增。",
        "rr_count": "旧底盘当前动作期间累计的右后轮编码器脉冲数；只在 motion_busy 且 rr_tick 到达时递增。",
        "in_run": "旧底盘路线状态是否处于 STATE_RUN。",
        "motion_done": "四轮编码器累计值均达到 target_ticks 的旧底盘动作完成判定。",
        "target_ticks": "当前旧底盘动作要求每个轮子累计达到的编码器脉冲目标。",
        "start_cmd": "当前已启动并保持到动作结束的 2 bit 路线命令。",
        "restart_pending": "旧底盘动作结束后的单周期隔离标志，避免同一队首命令立即重复启动。",
    },
    "motor_tb6612_driver": {
        "pwm_cnt": "8 bit 自由运行 PWM 载波计数器；与四轮 duty 比较生成对应 PWM 输出。",
    },
    "temp_ultrasonic_echo_source": {
        "trig_rise": "临时模型对 trig 同步值检测出的上升沿单周期脉冲，用于开始伪回波。",
        "state": "临时超声模型的空闲、触发延时、ECHO 高电平和结束状态。",
        "cnt": "临时超声模型当前状态已经经过的系统时钟周期数。",
        "echo_target_cycles": "fake_distance_cm 换算得到的目标 ECHO 高电平总周期数。",
        "trig_count": "用于观察触发次数的 4 bit 诊断计数器。",
        "trig_d0": "trig 输入的第一级同步采样。",
        "trig_d1": "trig 输入的第二级同步采样，与 trig_d0 比较检测上升沿。",
    },
    "ultrasonic_obstacle_guard": {
        "echo_sync": "两级同步后的 ECHO 输入。",
        "state": "TRIG、WAIT_ECHO、MEASURE、GAP 四状态测距流程。",
        "cnt": "超声状态机当前阶段的 50 MHz 周期计数：在 TRIG 阶段形成 10 us 脉冲，在等待/测量阶段判断 30 ms 超时，在 GAP 阶段补足 60 ms 周期。",
        "echo_cycles": "ECHO 高电平持续周期数，与往返声波时间成正比。",
        "measured_cm": "echo_cycles/2900 的组合距离结果。",
        "threshold_cm": "停车阈值；步长为零时采用默认 20 cm。",
        "cm_cycle_cnt": "ECHO 高电平内 0..2899 的厘米换算分频计数；每累计 2900 周期 measured_cm 加一。",
        "echo_d0": "异步 ECHO 输入的第一级同步采样。",
        "echo_d1": "ECHO 的第二级同步采样，对外作为 echo_sync。",
        "echo_d2": "echo_d1 的上一周期值，与 echo_d1 比较检测上升沿和下降沿。",
        "compare_length": "当前实际超声停车阈值，单位 cm；step_length 为零时使用 DEFAULT_STEP_CM。",
        "echo_rise": "同步后 ECHO 从 0 到 1 的单周期边沿脉冲，结束等待并开始计时。",
        "echo_fall": "同步后 ECHO 从 1 到 0 的单周期边沿脉冲，结束计时并换算距离。",
    },
    "c5_motor_test_pattern": {
        "counter": "当前测试阶段已经持续的 50 MHz 周期数；驱动阶段计到 2 s，停车阶段计到 1 s 后切换阶段。",
        "drive_phase": "固定测试序列阶段标志：1 时四轮输出 TEST_SPEED，0 时四轮输出零速。",
    },
    "power_on_reset": {
        "por_count": "外部 rst_n 释放后累计的 50 MHz 周期数；计满 POR_CYCLES=1,000,000 后才把 system_rst_n 置高。",
    },
    "uart_tx_8n1": {
        "state": "UART 字节发送状态：IDLE 等待 start，START 保持起始低位，DATA 依次发送 bit0..bit7，STOP 保持停止高位。",
        "clk_count": "当前 UART bit 已保持的 50 MHz 周期数；达到 CLKS_PER_BIT-1 时进入下一数据位或状态。",
        "bit_index": "当前正在发送的数据位编号 0..7；从 shifter 的最低位开始按 LSB-first 递增。",
        "shifter": "start 被接受时锁存的 8 bit data；整个字节发送期间保持不变，避免上游改写破坏帧。",
    },
    "gui_route_renderer": {
        "i": "路线重建与逐像素命中循环下标；遍历 8 个命令槽清零线段数组、生成历史/待执行线段并测试当前像素。",
    },
}


LINE_MARKER_DOCS = {
    "route_contol_core": [
        (r"^\s*STATE_CHANGE: begin", "编辑态：添加前进/转向复合动作，或按下键撤销最后一个 UI 动作。"),
        (r"^\s*STATE_RUN: begin", "运行态：障碍优先停止；否则每个 run_step_pulse 消费队首命令。"),
        (r"^\s*STATE_STOP: begin", "障碍停止态：等待中键返回编辑，并清除尚未执行的路线。"),
    ],
    "c5_route_controller": [
        (r"^\s*end else if \(manual_mode\) begin", "手动模式优先于自动路线；动作持续时间由手机重复命令和蓝牙看门狗决定。"),
        (r"^\s*end else if \(obstacle_stop\) begin", "自动模式遇到任一障碍源立即把四轮目标速度清零。"),
        (r"^\s*ROUTE_FORWARD: begin", "自动前进：四轮同为正速度，持续 step_length*125 ms。"),
        (r"^\s*ROUTE_LEFT: begin", "自动左转：左侧反转、右侧正转，形成原地左旋。"),
        (r"^\s*ROUTE_RIGHT: begin", "自动右转：左侧正转、右侧反转，形成原地右旋。"),
    ],
    "c5_motor_frame_driver": [
        (r"^\s*if \(!startup_ready\) begin", "启动保护阶段：任何运动输入都不会发出，先等待 500 ms。"),
        (r"^\s*if \(startup_ready && \(force_stop", "检测强制停止或运动到零速边沿，重新装载三帧停车序列。"),
        (r"^\s*if \(send_stop_now\) begin", "停车帧具有优先级：锁存四轮 1500 中位值并选择 #255PDST!。"),
    ],
    "bt_route_uart": [
        (r'^\s*"H", "h": begin', "H/h：进入手机手动模式，同时先发强制停车防止继承旧速度。"),
        (r'^\s*"O", "o": begin', "O/o：退出手动回到自动路线模式，并先停车。"),
        (r"^\s*if \(!sending_status && !sending_reply", "发送调度入口：只有当前无完整帧时才从挂起队列选择下一帧。"),
    ],
    "rplidar_c1_uart": [
        (r"^\s*CMD_BOOT: begin", "等待雷达电源和电机稳定 0.5 s。"),
        (r"^\s*CMD_STOP_WAIT: begin", "A5 25 停止命令后等待 0.1 s，清除雷达旧扫描状态。"),
        (r"^\s*CMD_WAIT_DESC: begin", "逐字节匹配标准扫描响应描述符 A5 5A 05 00 00 40 81。"),
        (r"^\s*CMD_SCAN: begin", "连续扫描态：接收字节维持在线计时，有效节点令 lidar_online 置位。"),
    ],
    "ultrasonic_obstacle_guard": [
        (r"^\s*ST_TRIG: begin", "输出 10 us 高电平触发超声波发射。"),
        (r"^\s*ST_WAIT_ECHO: begin", "等待 ECHO 上升沿；30 ms 未出现则输出 FFFF 超时。"),
        (r"^\s*ST_MEASURE: begin", "累计 ECHO 高电平周期，下降沿到达时换算厘米并比较阈值。"),
        (r"^\s*ST_GAP: begin", "补足到 60 ms 测量周期，避免触发回波互相干扰。"),
    ],
}


TESTBENCH_DOCS = {
    "c5_motor_frame_driver_robei_test.v": "验证 C5 上电等待、三帧停车保护、运动刷新周期和脉宽。",
    "c5_motor_mapping_robei_test.v": "验证前进、后退、原地左右转及四轮校准到脉宽的映射。",
    "car_param_controller_robei_test.v": "验证七类参数默认值、步进和饱和上下限。",
    "c5_route_controller_robei_test.v": "验证手动/自动运动、前进避障阻断以及倒车和转向脱困。",
}


COMMON_SIGNAL_DOCS = {
    "rx_state": "UART 接收状态，依次经历空闲、起始位确认、8 位数据采样和停止位检查。",
    "rx_clk_count": "UART 接收当前 bit 内的系统时钟计数；计满 CLKS_PER_BIT 后采样下一位。",
    "rx_bit_index": "UART 正在接收的数据位编号 0..7，数据按 LSB-first 顺序到达。",
    "rx_shift": "UART 接收移位缓存；8 个数据位收齐后整体复制到 rx_data。",
    "rx_data": "最近一个通过停止位检查的完整 UART 字节。",
    "rx_valid": "UART 字节接收完成脉冲；仅保持一个系统时钟周期。",
    "rx_sync": "异步 RX 输入的两级同步寄存器，降低亚稳态进入 UART 状态机的概率。",
    "tx_state": "UART 发送状态，依次输出空闲高电平、起始位、8 位数据和停止位。",
    "tx_clk_count": "UART 当前发送 bit 已保持的系统时钟周期数。",
    "tx_bit_index": "UART 正在发送的数据位编号 0..7，发送顺序为 LSB-first。",
    "tx_shift": "UART 开始发送时锁存的数据字节，避免发送中途被新输入改变。",
    "tx_data": "等待交给 UART 位发送状态机的下一个字节。",
    "tx_start": "启动一个 UART 字节发送的单周期脉冲。",
    "tx_line": "UART 发送引脚的内部逻辑电平；空闲和停止位为 1。",
    "tx_busy": "UART 正在发送一个字节；为 1 时上层不得覆盖 tx_data。",
    "tx_busy_d": "tx_busy 的上一周期值，与当前值组合检测字节发送完成下降沿。",
    "uart_state": "UART 字节发送状态，依次为 IDLE、START、DATA 和 STOP。",
    "uart_clk_count": "UART 当前 bit 的系统时钟分频计数器。",
    "uart_bit_index": "UART 当前发送的数据位编号 0..7。",
    "uart_shifter": "UART 发送开始时锁存的 8 bit 字节。",
    "uart_data": "帧调度器准备发送的当前 ASCII 字符。",
    "uart_start": "请求底层 UART 发送 uart_data 的单周期脉冲。",
    "uart_tx": "底层 UART 状态机产生的串行输出电平。",
    "uart_busy": "底层 UART 正在发送一个字节。",
    "uart_busy_d": "uart_busy 的一拍延迟，用于生成 uart_done。",
    "uart_done": "检测到 UART busy 从 1 变 0，表示一个完整字节已经发送。",
    "enable_sync": "异步使能输入的同步结果，防止直接跨时钟域使用。",
    "key_sync": "异步按键输入的同步结果。",
    "stable_state": "消抖后确认的稳定按键电平。",
    "debounce_count": "输入保持同一候选电平的连续周期数；计满后才接受状态变化。",
    "raw_buzzer": "蜂鸣器节奏门控后的内部方波，尚未根据 ACTIVE_LOW 调整极性。",
    "tone_bit": "按 TONE_HZ 分频翻转的音调方波状态。",
    "beep_active": "当前处于发声时间窗。",
    "gap_active": "多声提示之间的静音间隔时间窗。",
    "tone_cnt": "半个音调周期计数器，计满时翻转 tone_bit。",
    "beep_cnt": "单次蜂鸣持续时间计数器。",
    "gap_cnt": "两次蜂鸣之间静音时间计数器。",
    "pending_beeps": "新事件请求但尚未开始执行的蜂鸣次数。",
    "current_beeps": "当前提示序列仍需完成的蜂鸣次数。",
    "h_cnt": "当前视频行内的像素时钟位置。",
    "v_cnt": "当前视频帧内的行位置。",
    "running_disparity": "TMDS 已发送数据的直流不平衡累计值，用于选择下一码字极性。",
    "data_count": "数据窗口或串行传输过程中已经处理的元素数量。",
    "bit_index": "当前正在处理的位编号。",
    "state": "该模块主状态机的当前状态寄存器；复位后回到安全初始状态。",
}


TOKEN_LABELS = {
    "lf": "左前轮", "rf": "右前轮", "lr": "左后轮", "rr": "右后轮",
    "left": "左", "right": "右", "front": "前方", "rear": "后方",
    "prev": "上一周期", "last": "最近一次", "active": "当前有效",
    "current": "当前", "next": "下一值", "old": "历史", "target": "目标",
    "now": "当前位置", "route": "路线", "step": "步骤", "completed": "已完成",
    "remaining": "剩余", "total": "总计", "manual": "手动控制",
    "motor": "电机", "speed": "速度", "pulse": "脉冲", "frame": "帧",
    "count": "计数", "cnt": "计数", "index": "索引", "state": "状态",
    "busy": "忙标志", "valid": "有效标志", "enable": "使能",
    "data": "数据", "shift": "移位缓存", "sync": "同步值",
    "distance": "距离", "angle": "角度", "sector": "扇区", "quality": "质量",
    "pixel": "像素", "rgb": "RGB 像素", "menu": "菜单", "focus": "焦点",
    "cmd": "命令", "command": "命令", "status": "状态帧快照",
    "timeout": "超时", "watchdog": "看门狗", "safe": "安全阈值",
    "x": "X 坐标", "y": "Y 坐标", "dir": "朝向", "length": "长度",
    "forward": "前进", "turn": "转向", "reverse": "后退", "stop": "停车",
    "uart": "UART", "tx": "发送", "rx": "接收", "clk": "时钟",
    "hist": "历史值", "min": "最小值", "max": "最大值", "raw": "原始值",
    "line": "输出线", "lock": "锁定", "setting": "参数设置",
    "up": "上键/前进", "down": "下键/后退", "mid": "中键/确认",
    "hold": "保持时间", "due": "到期条件", "done": "完成标志",
    "reply": "短回复", "kind": "类型", "packet": "数据包",
    "lidar": "激光雷达", "ultrasonic": "超声波", "obstacle": "障碍",
    "merged": "合并结果", "echo": "回波", "online": "在线状态",
    "mode": "模式", "selected": "已选择", "param": "参数",
    "default": "默认值", "overall": "整体", "steps": "步数",
    "info": "信息", "set": "设定值", "expired": "已超时",
    "blocked": "受阻状态", "complete": "完成状态", "frames": "帧数",
    "cfg": "配置", "norm": "归一化", "changed": "变化标志",
    "start": "启动", "idle": "空闲", "scan": "扫描", "boot": "上电启动",
    "wait": "等待", "descriptor": "描述符", "source": "来源",
    "seq": "序号", "char": "字符", "word": "字表项", "sample": "采样",
    "pending": "待处理", "ready": "就绪", "is": "判定",
    "remain": "剩余量", "car": "小车", "body": "车体", "arrow": "方向箭头",
    "marker": "标记", "bar": "信息条", "row": "字模行", "col": "字模列",
    "bits": "字模位图", "value": "输入数值", "idx": "字符位置",
    "mag": "绝对幅值", "digit": "十进制位", "remainder": "除法余数",
    "rem": "剩余值", "offset": "偏移量", "scaled": "缩放结果",
    "diff": "有符号差值", "width": "线宽", "base": "基准坐标",
    "select": "选择值", "force": "强制请求", "motion": "运动动作",
    "bus": "总线", "resetn": "低有效复位", "locked": "时钟锁定",
    "de": "数据有效", "hsync": "水平同步", "vsync": "垂直同步",
    "sx": "线段起点 X", "sy": "线段起点 Y", "ex": "线段终点 X",
    "ey": "线段终点 Y", "px": "当前像素 X", "py": "当前像素 Y",
    "nibble": "十六进制半字节", "ch": "待显示字符",
}


def identifier_label(name: str) -> str:
    """Turn a snake_case RTL name into a short Chinese semantic label."""
    tokens = [token for token in name.strip("_").split("_") if token]
    labels = [TOKEN_LABELS.get(token, token.upper() if len(token) <= 3 else token) for token in tokens]
    return " / ".join(labels) if labels else name


def inferred_signal_doc(module_name: str, name: str, kind: str = "wire", size: int | None = None) -> str:
    explicit = INTERNAL_SIGNAL_DOCS.get(module_name, {}).get(name)
    if explicit:
        return explicit
    if name in COMMON_SIGNAL_DOCS:
        return COMMON_SIGNAL_DOCS[name]

    label = identifier_label(name)
    width = f"{size} bit " if size and size > 1 else ""

    if module_name == "motor_encoder_interface":
        sync_match = re.match(r"(lf|rf|lr|rr)_([ab])_d([01])$", name)
        if sync_match:
            wheel, phase, stage = sync_match.groups()
            stage_text = "第一级同步采样" if stage == "0" else "第二级/上一周期采样"
            return f"旧底盘 {TOKEN_LABELS[wheel]}编码器 {phase.upper()} 相的{stage_text}。"
        rise_match = re.match(r"(lf|rf|lr|rr)_a_rise$", name)
        if rise_match:
            return f"旧底盘 {TOKEN_LABELS[rise_match.group(1)]}编码器 A 相同步值的单周期上升沿脉冲。"
        total_match = re.match(r"(lf|rf|lr|rr)_total_count$", name)
        if total_match:
            return f"旧底盘 {TOKEN_LABELS[total_match.group(1)]}累计编码器脉冲数；到 FFFF 后饱和保持。"

    if module_name == "gui_route_renderer":
        gui_exact = {
            "cmd": "路线重建循环当前读取的 2 bit 命令临时量；分别遍历历史队列和待执行队列。",
            "dir": "路线重建循环的临时朝向；遇到左/右命令旋转，遇到前进命令决定线段方向。",
            "cur_x": "路线重建游标的当前屏幕 X 坐标；每生成一条前进线段后移动到线段终点。",
            "cur_y": "路线重建游标的当前屏幕 Y 坐标；每生成一条前进线段后移动到线段终点。",
            "next_x": "根据当前朝向和 step_pix 计算的下一条路线线段终点 X 坐标。",
            "next_y": "根据当前朝向和 step_pix 计算的下一条路线线段终点 Y 坐标。",
            "step_pix": "当前每个逻辑路线步骤对应的屏幕像素间距；随 active_route_steps 分段缩放。",
            "total_bar_w": "计划总路线长度换算并限制到 0..320 后的信息条宽度，单位像素。",
            "done_bar_w": "已完成路线长度换算并限制到 0..320 后的信息条宽度，单位像素。",
            "remain_bar_w": "剩余路线长度换算并限制到 0..320 后的信息条宽度，单位像素。",
            "speed_bar_w": "当前速度除以 2 并限制到 0..320 后的信息条宽度，单位像素。",
            "car_x": "根据已完成路线重建得到的小车屏幕中心 X 坐标。",
            "car_y": "根据已完成路线重建得到的小车屏幕中心 Y 坐标。",
            "car_dir": "根据已完成路线重建得到的小车当前显示朝向。",
            "target_draw_x": "逻辑目标 X 坐标相对 128 原点缩放后得到的屏幕标记 X 坐标。",
            "target_draw_y": "逻辑目标 Y 坐标相对 128 原点缩放后得到的屏幕标记 Y 坐标。",
            "stop_seg_valid": "停车状态前方警戒短线是否存在；为 1 时 stop_seg_* 四个端点有效。",
            "old_route_pixel": "当前像素是否命中已完成路线中的任一虚线线段。",
            "active_route_pixel": "当前像素是否命中待执行路线中的任一实线线段。",
            "stop_line_pixel": "当前像素是否命中停车状态下绘制的前方警戒短线。",
            "rgb_next": "当前像素完成背景、面板、路线、小车和文字优先级叠加后的 RGB888 组合颜色。",
            "grid_x_line": "当前 pixel_x 是否等于任一预设垂直网格线坐标。",
            "grid_y_line": "当前 pixel_y 是否等于任一预设水平网格线坐标。",
            "text_idx": "当前像素所在的 12 像素文本字符槽编号。",
            "text_base_x": "当前字符槽左边界的屏幕 X 坐标，传给 5x7 字模命中函数。",
            "text_char": "当前文本槽查表得到的 ASCII 字符，随后交给字模函数绘制。",
            "state_stop": "state_info 是否等于 STATE_STOP 的组合判定；控制停车警戒线和 STOP 状态配色。",
            "state_menu_sel": "state_info 是否等于 STATE_MENU 的组合判定；选择菜单状态栏和文字。",
            "state_change_sel": "state_info 是否等于 STATE_CHANGE 的组合判定；选择路线编辑状态栏和文字。",
            "state_run_sel": "state_info 是否等于 STATE_RUN 的组合判定；选择运行状态栏和文字。",
            "state_stop_sel": "state_info 是否等于 STATE_STOP 的组合判定；选择停车状态栏和文字。",
            "change_select": "menu_info 当前是否选中/进入路线编辑项；决定 EDIT 菜单框高亮。",
            "run_select": "menu_info 当前是否选中/进入运行项；决定 RUN 菜单框高亮。",
        }
        if name in gui_exact:
            return gui_exact[name]
        segment_match = re.match(r"(old|active)_seg_(s|e)(x|y)$", name)
        if segment_match:
            route_kind, endpoint, axis = segment_match.groups()
            route_text = "已完成路线" if route_kind == "old" else "待执行路线"
            endpoint_text = "起点" if endpoint == "s" else "终点"
            return f"{route_text}每个槽位对应线段的{endpoint_text} {axis.upper()} 屏幕坐标数组。"
        if name in {"old_seg_valid", "active_seg_valid"}:
            route_text = "已完成路线" if name.startswith("old") else "待执行路线"
            return f"{route_text}各槽位是否包含一条可绘制前进线段的位图；bit i 对应第 i 个内部命令。"
        if name.startswith("stop_seg_"):
            return f"停车警戒短线的“{identifier_label(name[9:])}”屏幕坐标；方向由 car_dir 决定。"
        if name.startswith("txt_"):
            return f"当前像素是否命中“{identifier_label(name[4:])}”对应文字字模；为 1 时文字颜色覆盖底图。"
        if name in {
                "status_panel", "menu_panel", "route_panel", "change_box",
                "run_box", "map_area", "border_line", "grid_line",
                "state_bar_0", "state_bar_1", "state_bar_2", "state_bar_3",
                "bt_status_box", "info_bar_total", "info_bar_done",
                "info_bar_remain", "info_bar_speed", "car_body", "car_arrow",
                "target_marker", "now_marker",
        }:
            return f"当前 pixel_x/pixel_y 对“{identifier_label(name)}”图形区域的命中判定；为 1 时进入对应着色分支。"

    if name.startswith("prev_"):
        return f"上一周期锁存的“{identifier_label(name[5:])}”；与当前输入比较以触发状态变化上报。"
    if name.startswith("status_") and name not in {"status_due", "status_left"}:
        return f"开始发送状态帧时锁存的“{identifier_label(name[7:])}”快照；保证长帧发送期间字段前后一致。"
    if name.endswith("_d"):
        return f"“{identifier_label(name[:-2])}”的一拍延迟值；用于检测上升沿、下降沿或状态变化。"
    if name.endswith("_q2"):
        return f"{width}{label}，采用 Q2 定点格式，最低两位表示 1/4 mm。"
    if name.endswith("_q6"):
        return f"{width}{label}，采用 Q6 定点格式，最低六位表示 1/64 度。"
    if name.endswith("_mm"):
        return f"{width}{label}，单位为毫米；FFFF 通常表示当前没有有效测量。"
    if name.endswith("_cm"):
        return f"{width}{label}，单位为厘米；用于超声距离显示或阈值比较。"
    if name.endswith(("_count", "_cnt")):
        return f"{width}用于“{label}”的计数寄存器；由时序过程递增、清零或饱和保持，以形成对应时间/数量条件。"
    if name.endswith("_index"):
        return f"{width}用于“{label}”的位置索引；指示数组、帧字符或串行 bit 当前处理到的位置。"
    if name.endswith("_state"):
        return f"{width}用于“{identifier_label(name[:-6])}”的状态寄存器；决定状态机当前执行的分支和下一状态。"
    if name.endswith("_pulse"):
        return f"“{identifier_label(name[:-6])}”事件脉冲；条件成立时只保持一个 clk 周期，供下游状态机触发一次动作。"
    if name.endswith("_valid"):
        return f"“{label}”信号；为 1 的周期内，与它配套的数据可以被下游锁存。"
    if name.endswith("_busy"):
        return f"“{label}”信号；为 1 时当前事务尚未结束，上层必须等待而不能启动冲突操作。"
    if name.endswith("_enable"):
        return f"“{identifier_label(name[:-7])}”功能使能；为 0 时模块回到停止/空闲安全状态。"
    if name.endswith("_speed"):
        return f"{width}{label}；在运动控制链中表示轮速幅值或 11 bit 二补码有符号轮速。"
    if name.startswith("is_"):
        return f"{label}组合判定；为 1 表示名称描述的条件当前成立，为 0 表示不成立。"
    if name.endswith(("_hit", "_inside")):
        return f"{label}像素命中判定；为 1 表示当前 pixel_x/pixel_y 落入对应图形区域。"
    if name.endswith("_sel"):
        return f"“{identifier_label(name[:-4])}”选择判定；通常由状态编码比较产生，决定当前组合逻辑分支。"
    if name.endswith("_stop"):
        return f"“{identifier_label(name[:-5])}”高有效停车请求；为 1 时运动控制链应输出零速或进入停车保护。"
    if name.endswith(("_active", "_ready", "_pending", "_complete", "_lock", "_timeout", "_online")):
        return f"“{label}”控制标志；为 1 表示名称描述的状态当前成立。"
    if name.endswith("_char"):
        return f"{width}“{identifier_label(name[:-5])}”字符，保存或组合生成一个 8 bit ASCII 值供显示/串口组帧。"
    if name.endswith("_info"):
        return f"{width}{label}打包字段；各 bit 按模块协议承载路线、菜单或运行状态。"
    if name.endswith(("_x", "_y")):
        return f"{width}“{label}”数值；表示路线栅格或显示空间中的对应坐标。"
    if name.endswith("_length"):
        return f"{width}“{identifier_label(name[:-7])}”长度结果；由步数和单步长度组合换算。"
    if name.endswith("_ms"):
        return f"{width}“{identifier_label(name[:-3])}”时间量，单位毫秒；用于动作持续时间配置或换算。"
    if kind in {"input", "output", "inout"}:
        direction = {"input": "输入", "output": "输出", "inout": "双向"}[kind]
        return f"{width}函数或任务的“{label}”{direction}参数；只在本次调用求值期间参与组合计算。"
    if kind == "comb_reg":
        return f"{width}组合过程变量“{label}”；always @(*) 每次重新计算，本身不保存跨像素或跨时钟周期状态。"
    if kind == "clocked_temp_reg":
        return f"{width}时钟过程内部的阻塞赋值临时量“{label}”；只参与本次边沿计算，不作为需跨周期保存的状态输出。"
    if kind == "clocked_reg":
        return f"{width}寄存器，保存“{label}”的跨周期状态；仅在时钟过程更新，并在复位分支初始化。"
    if kind == "mixed_reg":
        return f"{width}过程变量“{label}”；代码在多个过程上下文赋值，通读时必须同时检查组合默认值和时钟更新路径。"
    if kind == "reg":
        return f"{width}过程变量“{label}”；具体硬件含义由其赋值所在的 always/函数决定，相关过程块注释给出更新时机。"
    if kind == "integer":
        return f"整数临时量“{label}”；用于循环展开、坐标运算或组合中间计算，不代表独立硬件接口。"
    return f"{width}组合网络，传递“{label}”；由连续赋值或上游模块驱动，本身不保存跨周期状态。"


CONSTANT_DOCS = {
    "CLK_HZ": "模块假定的系统时钟频率，单位 Hz；所有毫秒和波特率计数均由它换算。",
    "BAUD": "UART 目标波特率，单位 bit/s。",
    "CLKS_PER_BIT": "每个 UART bit 占用的系统时钟周期数；整数除法会形成实际波特率误差。",
    "CYCLES_PER_MS": "1 ms 对应的系统时钟周期数。",
    "POR_CYCLES": "外部复位释放后继续保持内部复位的周期数。",
    "CAL_BASE_SPEED": "四轮默认校准的零偏基准；默认轮速等于该值时不改变整体速度。",
    "FRAME_LAST": "62 字节 C5 运动帧最后一个字符的零基下标。",
    "STOP_FRAME_LAST": "9 字节 #255PDST! 停车帧最后一个字符的零基下标。",
    "FRONT_MIN_EMPTY": "当前扫描圈尚未收到有效前方点时使用的无效哨兵 FFFF。",
    "ROUTE_STATUS_LEFT": "@S 路线/参数状态帧的字符计数初值；发送器据此覆盖协议规定的完整帧长度。",
    "LIDAR_STATUS_LEFT": "@L 八扇区雷达状态帧的字符计数初值；发送器据此覆盖协议规定的完整帧长度。",
    "MANUAL_TIMEOUT_CYCLES": "300 ms 手动运动看门狗对应的系统时钟周期数；超时后强制停车。",
    "STATUS_CYCLES": "200 ms 主动状态上报周期对应的系统时钟周期数。",
    "LINK_TIMEOUT_CYCLES": "3 s 蓝牙心跳离线判定对应的系统时钟周期数。",
    "KEY_HOLD_CYCLES": "蓝牙字符模拟实体按键低电平约 30 ms 所需的系统时钟周期数。",
    "MOVING_REFRESH_CYCLES": "C5 运动状态两次主动刷新之间的 100 ms 等待周期数。",
    "IDLE_REFRESH_CYCLES": "C5 零速/空闲状态的刷新等待周期数；当前策略可用它控制是否周期补发。",
    "STARTUP_WAIT_CYCLES": "C5 上电后禁止运动并等待外设稳定的 500 ms 周期数。",
    "ACTIVE_LOW": "蜂鸣器输出极性配置：1 对 raw_buzzer 取反，0 保持原极性。",
    "HALF_PERIOD": "蜂鸣器音调半周期对应的系统时钟数；计满一次翻转 tone_bit。",
    "TONE_HZ": "蜂鸣器提示音基频 2000 Hz；HALF_PERIOD 根据它计算方波翻转间隔。",
    "BEEP_MS": "每一声蜂鸣保持有效的时间，单位 ms；当前值 80 ms。",
    "GAP_MS": "多声提示中相邻两声之间的静音间隔，单位 ms；当前值 80 ms。",
    "BEEP_CYCLES": "单声蜂鸣 80 ms 对应的 50 MHz 周期数；beep_cnt 达到该值后进入静音或结束提示。",
    "GAP_CYCLES": "蜂鸣间隔 80 ms 对应的 50 MHz 周期数；gap_cnt 达到该值后开始下一声。",
    "MOVING_REFRESH_MS": "C5 四轮处于运动状态时主动重发当前控制帧的等待时间，单位 ms；当前为 100 ms。",
    "STARTUP_DELAY_MS": "FPGA 复位释放后 C5 驱动保持零速并等待底盘稳定的时间，单位 ms；当前为 500 ms。",
    "STARTUP_CYCLES": "C5 上电保护 500 ms 对应的 50 MHz 周期数；计满前不会接受普通运动帧。",
    "DRIVE_CYCLES": "未接入主链的固定测试图案连续输出 TEST_SPEED 的时间；50 MHz 下等于 2 s。",
    "STOP_CYCLES": "固定测试图案四轮保持零速的时间；50 MHz 下等于 1 s，之后再次进入驱动阶段。",
    "BOOT_DELAY_CYCLES": "RPLIDAR 上电后等待电源和转子稳定的 0.5 s 周期数；计满后发送 A5 25 停止命令。",
    "STOP_WAIT_CYCLES": "向 RPLIDAR 发送 A5 25 后等待旧数据流停止的 0.1 s 周期数。",
    "RETRY_WAIT_CYCLES": "雷达描述符或数据超时后再次执行启动序列前等待的 1 s 周期数。",
    "BYTE_TIMEOUT_CYCLES": "连续 2 s 没有收到雷达串口字节时判定链路离线并进入重试。",
    "TEST_SPEED": "未接入主控制链的 C5 测试图案固定轮速幅值。",
    "ROUTE_WORDS": "本模块实际展开/显示的 2 bit 路线命令槽数量；数组和循环边界均以此为准。",
    "DEFAULT_STEP_CM": "超声避障阈值输入为零时采用的后备安全距离 20 cm。",
}


def constant_doc(module_name: str, name: str, rhs: str) -> str:
    if name in CONSTANT_DOCS:
        return CONSTANT_DOCS[name]
    label = identifier_label(name)
    if module_name == "uart_tx_8n1" and name.startswith("STATE_"):
        uart_state_docs = {
            "STATE_IDLE": "UART 空闲状态编码；tx 保持高电平，busy=0，并等待一个 start 脉冲锁存 data。",
            "STATE_START": "UART 起始位状态编码；tx 连续保持低电平一个 CLKS_PER_BIT 时间。",
            "STATE_DATA": "UART 数据位状态编码；按 bit_index=0..7 以 LSB-first 顺序输出 shifter。",
            "STATE_STOP": "UART 停止位状态编码；tx 保持高电平一个 bit 后清除 busy 并回到 IDLE。",
        }
        return uart_state_docs[name]
    if name.startswith("STATE_"):
        return f"路线/业务状态“{label}”的固定编码；case 分支和状态上报必须保持一致。"
    if name.startswith("MENU_"):
        return f"菜单显示状态“{label}”的固定编码。"
    if name in {"ROUTE_STOP", "ROUTE_RIGHT", "ROUTE_LEFT", "ROUTE_FORWARD"}:
        return f"2 bit 路线命令“{label}”的编码，存入 route_info 对应槽位。"
    if name in {"MANUAL_STOP", "MANUAL_FORWARD", "MANUAL_REVERSE", "MANUAL_LEFT", "MANUAL_RIGHT"}:
        return f"手机手动动作“{label}”的 3 bit 协议/内部统一编码。"
    if name in {"MOTOR_STOP", "MOTOR_FORWARD", "MOTOR_REVERSE", "MOTOR_BRAKE"}:
        return f"旧 TB6612 底盘电机方向“{label}”的 2 bit 编码。"
    if name.startswith("DIR_"):
        return f"小车朝向“{label}”的模 4 编码；加减 1 分别完成 90 度右/左转。"
    if name.startswith("PARAM_"):
        return f"可调参数“{label}”的选择索引，供手机直接选中后执行增减。"
    if name.startswith("WORD_"):
        return f"GUI 字符表中“{label}”文本的索引。"
    if name.startswith("CMD_"):
        return f"雷达命令状态机“{label}”的状态编码。"
    if name.startswith("ST_"):
        return f"超声/控制状态机“{label}”的状态编码。"
    if name.startswith("RX_"):
        return f"UART 接收状态“{label}”的编码。"
    if name.startswith(("TX_", "UART_")):
        return f"UART 发送状态“{label}”的编码。"
    if name.startswith("REPLY_"):
        return f"蓝牙短回复类型或结束位置“{label}”的编码。"
    if name.startswith("H_"):
        return f"HDMI 水平时序参数“{label}”，单位为像素时钟周期。"
    if name.startswith("V_"):
        return f"HDMI 垂直时序参数“{label}”，单位为行。"
    if name.endswith("_CYCLES"):
        return f"“{label}”对应的系统时钟周期数；达到该值后触发状态转换或超时。"
    if name.endswith("_MS"):
        return f"“{label}”的时间配置，单位毫秒。"
    if name.endswith("_HZ"):
        return f"“{label}”的频率配置，单位 Hz。"
    if name.endswith("_MIN"):
        return f"“{label}”允许的饱和下限；继续减小时保持此值。"
    if name.endswith("_MAX"):
        return f"“{label}”允许的饱和上限；继续增加时保持此值。"
    if name.endswith("_STEP"):
        return f"每次手机增减命令对“{label}”施加的固定步进。"
    if name.endswith("_LAST"):
        return f"“{label}”数组或帧的最后一个有效零基下标。"
    if name.endswith("_Q2"):
        return f"“{label}”使用 Q2 定点格式，最低两位表示 1/4 mm。"
    if name.endswith("_Q6"):
        return f"“{label}”使用 Q6 定点格式，最低六位表示 1/64 度。"
    return f"本模块固定参数“{label}”，RTL 取值为 {rhs.strip()}；用于统一相关比较和计算。"


def inferred_function_doc(name: str) -> str:
    """Describe an unlisted helper from its conventional RTL name."""
    label = identifier_label(name)
    if name.startswith(("clamp", "limit", "saturate")):
        return f"把输入限制到“{label}”规定的上下界，防止算术结果越过协议或硬件允许范围。"
    if name.startswith(("hex", "digit", "ascii")):
        return f"把数值字段转换成“{label}”所需的单个 ASCII/十六进制字符，供串口逐字节组帧。"
    if name.startswith(("glyph", "font", "text", "word", "segment")):
        return f"根据字符、字表或像素位置组合查询“{label}”，直接供 GUI 当前像素着色使用。"
    if name.startswith(("calc", "scaled", "adjust", "map", "convert")):
        return f"根据函数输入组合计算“{label}”；结果在调用处立即使用，不引入寄存器或额外时序状态。"
    if name.startswith(("min", "max", "median")):
        return f"对输入样本执行“{label}”比较/选择，用于滤波或边界判断，函数本身不保存历史。"
    return f"根据调用参数组合求得“{label}”；函数内没有时钟和跨周期状态，输出只由本次输入决定。"


def inferred_task_doc(name: str) -> str:
    """Describe an unlisted procedural task without pretending it is hardware."""
    return (
        f"封装“{identifier_label(name)}”所需的一组过程赋值；"
        "调用时在当前 always 分支内展开执行，不产生独立模块或并行线程。"
    )


SCOPED_SIGNAL_DOCS = {
    ("c5_motor_frame_driver", "left_motor_pulse", "mag"): "输入有符号轮速的绝对幅值；用于计算左侧脉宽 1500+speed 或 1500-mag。",
    ("c5_motor_frame_driver", "right_motor_pulse", "mag"): "输入有符号轮速的绝对幅值；用于计算右侧反向安装对应的 1500-speed 或 1500+mag。",
    ("c5_motor_frame_driver", "digit4", "digit"): "index 指定位置最终得到的 0..9 十进制数字；函数末尾加 ASCII 字符 '0' 后返回。",
    ("c5_motor_frame_driver", "digit4", "remainder"): "逐级减去千、百、十位权值后的剩余数；最终剩余值直接作为个位。",
    ("c5_route_controller", "adjusted_speed", "value"): "common_speed + wheel_default - 700 的有符号中间结果；随后饱和限制到 100..900。",
    ("c5_route_controller", "step_to_ms", "ms_calc"): "step_length*125 的 32 bit 中间毫秒数；超过 60000 时输出饱和到 60 s。",
    ("gui_route_renderer", "clamp_bar_w", "bar_w"): "把输入路线/速度量复制成整数后限制到 0..320，所得值直接作为信息条像素宽度。",
    ("gui_route_renderer", "scaled_offset", "diff"): "相对逻辑原点 128 的最终有符号像素偏移；value<128 时取负，反之取正。",
    ("gui_route_renderer", "scaled_offset", "mag"): "逻辑坐标 value 与原点 128 之间的无符号格数差。",
    ("gui_route_renderer", "scaled_offset", "scaled"): "mag 按当前 pix 网格间距用移位加法放大后的无符号像素距离。",
    ("gui_route_renderer", "segment_hit", "lx"): "线段两个 X 端点中的较小值，用于建立带 width 余量的水平包围盒。",
    ("gui_route_renderer", "segment_hit", "hx"): "线段两个 X 端点中的较大值，用于建立带 width 余量的水平包围盒。",
    ("gui_route_renderer", "segment_hit", "ly"): "线段两个 Y 端点中的较小值，用于建立带 width 余量的垂直包围盒。",
    ("gui_route_renderer", "segment_hit", "hy"): "线段两个 Y 端点中的较大值，用于建立带 width 余量的垂直包围盒。",
    ("gui_route_renderer", "glyph5x7_2x_hit", "row"): "当前 py 在 2 倍放大 5x7 字模中的行号 0..6；每两个屏幕像素对应一行。",
    ("gui_route_renderer", "glyph5x7_2x_hit", "col"): "当前 px 在 2 倍放大字模中的反向列号 4..0，用于索引 bits。",
    ("gui_route_renderer", "glyph5x7_2x_hit", "bits"): "按字符 ch 和 row 查出的当前 5 像素字模行；bits[col] 决定当前像素是否点亮。",
    ("gui_route_renderer", "hex16_char", "nibble"): "从 64 bit value 中按 idx 选出的 4 bit 半字节；小于 10 映射 '0'..'9'，否则映射 'A'..'F'。",
}


def scoped_declaration_doc(
        module_name: str,
        scope_kind: str,
        scope_name: str,
        name: str,
        kind: str,
        size: int | None) -> str:
    """Explain declarations inside a function/task without calling them flip-flops."""
    width = f"{size} bit " if size and size > 1 else ""
    label = identifier_label(name)
    owner = "函数" if scope_kind == "function" else "任务"
    explicit = SCOPED_SIGNAL_DOCS.get((module_name, scope_name, name))
    if explicit:
        return f"{width}{explicit} 该变量只在本次{owner}求值期间存在，不保存跨周期状态。"

    decimal_places = {
        "dec5_char": {"d0": "万位", "d1": "千位", "d2": "百位", "d3": "十位", "d4": "个位"},
        "dec3_char": {"d0": "百位", "d1": "十位", "d2": "个位"},
        "dec2_char": {"d0": "十位", "d1": "个位"},
    }
    if scope_name in decimal_places:
        if name == "digit":
            return f"{owner} `{scope_name}` 根据 idx 选中的最终十进制数字 0..9；末尾加 '0' 转成 ASCII。"
        if name == "rem":
            return f"{owner} `{scope_name}` 逐级减去高位十进制权值后保留的余数；最终用于计算较低位。"
        if name == "n":
            return f"{owner} `{scope_name}` 重复减法循环的展开下标；只控制本次十进制拆位次数，不是运行时状态计数器。"
        if name in decimal_places[scope_name]:
            place = decimal_places[scope_name][name]
            return f"{owner} `{scope_name}` 拆分得到的{place}数字 0..9；由重复减法累计得到。"

    if kind in {"input", "output", "inout"}:
        direction = {"input": "输入", "output": "输出", "inout": "双向"}[kind]
        return (
            f"{width}{owner} `{scope_name}` 的“{label}”{direction}参数；"
            "只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。"
        )
    if kind == "integer":
        return (
            f"{owner} `{scope_name}` 内部的整数临时量“{label}”；"
            "用阻塞赋值完成循环、除法替代或坐标中间运算，不会综合成独立状态寄存器。"
        )
    return (
        f"{width}{owner} `{scope_name}` 内部的组合临时变量“{label}”；"
        "仅保存本次函数/任务求值的中间结果，不是由 clk 驱动的跨周期寄存器。"
    )


def assign_dependency_doc(rhs: str) -> str:
    """Summarize direct operands of a continuous assignment."""
    ignored = {
        "assign", "begin", "end", "if", "else", "case", "default",
        "wire", "reg", "signed", "unsigned",
    }
    dependencies: list[str] = []
    for match in re.finditer(r"[A-Za-z_]\w*", rhs):
        name = match.group(0)
        if name in ignored:
            continue
        if match.start() > 0 and rhs[match.start() - 1] == "'":
            continue
        if name not in dependencies:
            dependencies.append(name)
    if not dependencies:
        return "右侧是固定常量，因此该网络不会随运行状态变化。"
    shown = "、".join(
        f"`{name}`（{identifier_label(name)}）" for name in dependencies[:6]
    )
    suffix = "等当前值" if len(dependencies) > 6 else "的当前值"
    return f"该组合结果直接由 {shown}{suffix}决定，不引入时钟周期延迟。"


def continuous_assignment_rhs_by_line(body: str) -> dict[int, tuple[str, str]]:
    """Map each continuous assignment's first line to its full right-hand side."""
    result: dict[int, tuple[str, str]] = {}
    pattern = re.compile(
        r"(?ms)^[ \t]*assign\s+([A-Za-z_]\w*)(?:\s*\[[^]]+\])?\s*=\s*(.*?);"
    )
    for match in pattern.finditer(body):
        line_index = body.count("\n", 0, match.start())
        result[line_index] = (match.group(1), match.group(2))
    return result


def process_assignment_targets(body: str) -> list[list[str]]:
    """Collect the main left-hand-side names for each always block."""
    starts = list(re.finditer(r"(?m)^\s*always(?:\s+@|\s+#)", body))
    result: list[list[str]] = []
    for index, start in enumerate(starts):
        end = starts[index + 1].start() if index + 1 < len(starts) else len(body)
        block = body[start.start():end]
        targets: list[str] = []
        for pattern in (
            r"(?m)^\s*(?:[^:\n]+:\s*)?([A-Za-z_]\w*)(?:\s*\[[^]]+\])?\s*<=",
            r"(?m)^\s*(?:[^:\n]+:\s*)?([A-Za-z_]\w*)(?:\s*\[[^]]+\])?\s*=(?!=)",
        ):
            for match in re.finditer(pattern, block):
                name = match.group(1)
                if name not in targets:
                    targets.append(name)
        result.append(targets)
    return result


def process_assignment_kinds(body: str) -> dict[str, set[str]]:
    """Classify each procedural assignment as state, combinational, or edge-local."""
    starts = list(re.finditer(r"(?m)^\s*always(?:\s+@|\s+#)", body))
    result: dict[str, set[str]] = {}
    for index, start in enumerate(starts):
        end = starts[index + 1].start() if index + 1 < len(starts) else len(body)
        block = body[start.start():end]
        header = block.splitlines()[0] if block else ""
        sequential = "posedge" in header or "negedge" in header
        operator_patterns = (
            (r"(?m)^\s*(?:[^:\n]+:\s*)?([A-Za-z_]\w*)(?:\s*\[[^]]+\])?\s*<=", "clocked"),
            (r"(?m)^\s*(?:[^:\n]+:\s*)?([A-Za-z_]\w*)(?:\s*\[[^]]+\])?\s*=(?!=)",
             "clocked_temp" if sequential else "comb"),
        )
        for pattern, assignment_kind in operator_patterns:
            for match in re.finditer(pattern, block):
                result.setdefault(match.group(1), set()).add(assignment_kind)
    return result


def inferred_process_doc(module_name: str, targets: list[str], sequential: bool) -> str:
    """Build a useful fallback for an always block missing a hand-written note."""
    if targets:
        shown = "、".join(f"{name}（{identifier_label(name)}）" for name in targets[:6])
        remainder = "等信号" if len(targets) > 6 else ""
    else:
        shown = "本模块的输出和内部状态"
        remainder = ""
    if sequential:
        return (
            f"时序过程：在有效时钟边沿更新 {shown}{remainder}；"
            "复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。"
        )
    return (
        f"组合过程：根据当前输入和寄存状态计算 {shown}{remainder}；"
        "过程内应为所有输出路径赋值，避免推断非预期锁存器。"
    )


def module_doc(module_name: str) -> tuple[str, list[str]]:
    return MODULE_DOCS.get(
        module_name,
        ("Robei 工程导出的辅助模块。", ["请结合端口连接和上层实例理解其具体用途。"]),
    )


def port_doc(module_name: str, name: str, direction: str, size: int) -> str:
    if name in PORT_DOCS:
        return PORT_DOCS[name]
    module_exact = {
        ("clk_wiz_0", "clk_in1"): "板载 50 MHz 时钟，经 IBUF 后送入 MMCME2_ADV。",
        ("clk_wiz_0", "resetn"): "MMCM 封装低有效复位；0 立即复位锁定与输出时钟。",
        ("clk_wiz_0", "clk_out1"): "MMCM/BUFG 产生的 50 MHz HDMI 像素时钟。",
        ("clk_wiz_0", "clk_out2"): "MMCM/BUFG 产生的 250 MHz TMDS 5 倍串化时钟。",
        ("clk_wiz_0", "locked"): "MMCM 原始锁定后再稳定等待 50,000 个像素周期的可靠时钟就绪标志。",
        ("gpio_key_adapter", "key_active_low"): "板级按键极性配置；1 表示物理引脚按下时为低电平。",
        ("gui_route_demo_info_source", "system_info"): "GUI 演示使用的固定 2 bit 系统状态，不进入实际小车主链。",
        ("hdmi_const", "tmds_clk_pattern"): "TMDS 时钟通道固定 10 bit 码型 1111100000。",
        ("hdmi_const", "gnd_bit"): "HDMI 辅助连接使用的固定逻辑 0。",
        ("hdmi_pixel_shader", "v_de"): "旧测试图案发生器的数据有效输入；0 时输出消隐颜色。",
        ("hdmi_pixel_shader", "r"): "旧测试图案当前像素的 8 bit 红色分量。",
        ("hdmi_pixel_shader", "g"): "旧测试图案当前像素的 8 bit 绿色分量。",
        ("hdmi_pixel_shader", "b"): "旧测试图案当前像素的 8 bit 蓝色分量。",
        ("hdmi_rgb_splitter", "data_r"): "pixel_de 门控后的 RGB888 红色字节；消隐区强制为 0。",
        ("hdmi_rgb_splitter", "data_g"): "pixel_de 门控后的 RGB888 绿色字节；消隐区强制为 0。",
        ("hdmi_rgb_splitter", "data_b"): "pixel_de 门控后的 RGB888 蓝色字节；消隐区强制为 0。",
        ("hdmi_serializer", "data_10b"): "像素域待串化的单通道 10 bit TMDS 码字。",
        ("hdmi_serializer", "ser_p"): "OBUFDS 输出的 TMDS 正端串行信号。",
        ("hdmi_serializer", "ser_n"): "OBUFDS 输出的 TMDS 负端串行信号。",
        ("hdmi_timing_gen", "hsync"): "1024x600 模式水平同步信号；极性和区间由 H_SYNC_START/END 定义。",
        ("hdmi_timing_gen", "vsync"): "1024x600 模式垂直同步信号；极性和区间由 V_SYNC_START/END 定义。",
        ("hdmi_timing_gen", "v_de"): "当前像素位于 1024x600 可见区的有效标志。",
        ("hdmi_tmds_encoder", "data_in"): "当前颜色通道待编码的 8 bit 像素字节。",
        ("hdmi_tmds_encoder", "c0"): "消隐期 TMDS 控制位 0；蓝色通道连接 hsync。",
        ("hdmi_tmds_encoder", "c1"): "消隐期 TMDS 控制位 1；蓝色通道连接 vsync。",
        ("hdmi_tmds_encoder", "v_de"): "数据有效选择；1 编码像素数据，0 输出四种控制码之一。",
        ("hdmi_tmds_encoder", "tmds_10b"): "按 running disparity 平衡后的 10 bit TMDS 码字。",
        ("hdmi_top", "pixel_clk"): "导出给 GUI 渲染器使用的 50 MHz HDMI 像素时钟。",
        ("rplidar_c1_uart", "quality"): "标准扫描节点的 6 bit 回波质量；为 0 的测点直接丢弃。",
        ("temp_key_gpio_config_source", "key_active_low"): "旧/测试按键链固定输出的低有效极性配置。",
        ("temp_ultrasonic_echo_source", "trig"): "临时超声模型接收的触发输入，用于启动伪回波计时。",
        ("temp_ultrasonic_echo_source", "echo"): "临时超声模型产生的伪 ECHO 高电平。",
        ("temp_ultrasonic_echo_source", "fake_distance_cm"): "临时超声模型用于决定伪回波宽度的目标距离，单位 cm。",
        ("ultrasonic_obstacle_guard", "enable"): "超声测距总使能；为 0 时终止测量并清除停车请求。",
        ("ultrasonic_obstacle_guard", "echo"): "经板级引脚进入的异步 ECHO 脉冲，宽度表示声波往返时间。",
        ("ultrasonic_obstacle_guard", "trig"): "每 60 ms 输出一次约 10 us 高电平以触发超声模块。",
        ("uart_tx_8n1", "data"): "start 有效时锁存并按 LSB-first 发送的 8 bit 字节。",
        ("uart_tx_8n1", "start"): "启动发送 data 的单周期请求；busy=1 时请求不会开启第二字节。",
        ("uart_tx_8n1", "tx"): "8N1 串行输出；空闲/停止位为高，起始位为低。",
        ("uart_tx_8n1", "busy"): "从锁存 start 到停止位结束期间保持为 1 的发送占用标志。",
    }
    if (module_name, name) in module_exact:
        return module_exact[(module_name, name)]
    if "sector_" in name and name.endswith("_mm"):
        sector_name = name[name.index("sector_") + 7:-3]
        return f"雷达 {identifier_label(sector_name)} 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。"
    wheel_match = re.match(r"(lf|rf|lr|rr)_(enc_[ab]|tick|dir|duty|in[12]|pwm)$", name)
    if wheel_match:
        wheel, signal = wheel_match.groups()
        wheel_name = TOKEN_LABELS[wheel]
        if signal.startswith("enc_"):
            return f"旧底盘 {wheel_name}编码器 {signal[-1].upper()} 相输入，用于边沿计数和方向判断。"
        if signal == "tick":
            action = "接收的编码器计数脉冲" if direction == "input" else "检测到编码器边沿时产生的单周期计数脉冲"
            return f"旧底盘 {wheel_name}{action}。"
        if signal == "dir":
            return f"旧底盘 {wheel_name}方向编码；motor_route_controller 与 TB6612 方向译码之间传递。"
        if signal == "duty":
            return f"旧底盘 {wheel_name}8 bit PWM 占空比命令，0 停止、255 接近全速。"
        if signal in {"in1", "in2"}:
            return f"旧底盘 TB6612 {wheel_name}方向控制 {signal.upper()} 物理输出。"
        return f"旧底盘 TB6612 {wheel_name}PWM 物理输出，由 8 bit 占空比比较产生。"
    if module_name == "motor_route_controller" and name in {
            "forward_ticks", "turn_ticks", "forward_duty", "turn_duty"}:
        motion = "前进" if name.startswith("forward") else "转向"
        if name.endswith("ticks"):
            return f"旧编码器底盘一次{motion}动作需要累计的轮脉冲目标数。"
        return f"旧编码器底盘执行{motion}动作时采用的 8 bit PWM 占空比。"
    if module_name == "motor_tb6612_driver" and name in {"stby_a", "stby_b"}:
        return "旧 TB6612 驱动器待机控制输出；motor_enable=1 时拉高使能对应双通道。"
    if module_name == "key_low_merge" and name.startswith("merged_key_"):
        return f"实体按键与蓝牙模拟按键按位 AND 后的“{identifier_label(name[11:-2])}”低有效电平。"
    if module_name == "route_key_lock_gate" and name.startswith("key_"):
        key_name = name[4:-5] if name.endswith("_n_in") else name[4:-6]
        if name.endswith("_n_in"):
            return f"门控前“{identifier_label(key_name)}”低有效按键电平。"
        return f"参数设置锁生效时被强制为 1 的“{identifier_label(key_name)}”低有效按键输出。"
    if module_name in {"temp_c5_motor_config_source", "temp_motor_config_source"}:
        if name.endswith("_ms"):
            return f"测试/后备配置输出的“{identifier_label(name[:-3])}”持续时间，单位 ms。"
        if name.endswith("_ticks"):
            return f"旧底盘测试配置输出的“{identifier_label(name[:-6])}”编码器目标脉冲数。"
        if name.endswith("_duty"):
            return f"旧底盘测试配置输出的“{identifier_label(name[:-5])}”8 bit PWM 占空比。"
    if name.startswith("key_") and name.endswith("_n"):
        return "低有效按键输入/输出；0 表示按下。"
    if name.startswith("hdmi_") and name.endswith(("_p", "_n")):
        return "HDMI TMDS 差分物理输出端。"
    if name.startswith("sector_") and name.endswith("_mm"):
        return "对应 45 度雷达扇区的滤波最近距离，FFFF 表示无效。"
    if (name.startswith("last_") or "_last_" in name) and name.endswith("_pulse"):
        return "最近一帧发送给该轮的 C5 脉宽诊断值。"
    if name.endswith("_pulse"):
        return f"“{identifier_label(name[:-6])}”事件脉冲；条件成立时仅保持一个 clk 周期。"
    if name.endswith("_speed"):
        return f"“{identifier_label(name)}”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。"
    if name.endswith("_valid"):
        return f"“{identifier_label(name[:-6])}”有效脉冲/标志；为 1 时下游可锁存配套数据。"
    if name.endswith("_enable"):
        return f"“{identifier_label(name[:-7])}”功能使能；为 0 时该功能保持停止或安全空闲状态。"
    if name.startswith("key_pin"):
        return "板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。"
    if name.endswith("_count"):
        return inferred_signal_doc(module_name, name, "wire", size)
    exact = inferred_signal_doc(module_name, name, "wire", size)
    if name in COMMON_SIGNAL_DOCS or name in INTERNAL_SIGNAL_DOCS.get(module_name, {}):
        return exact
    label = identifier_label(name)
    width = f"{size} bit " if size > 1 else ""
    if direction == "input":
        return f"{width}“{label}”输入；由物理接口或上游模块驱动，本模块读取后参与当前控制流程。"
    if direction == "output":
        return f"{width}“{label}”输出；把本模块计算或锁存的结果提供给顶层和下游模块。"
    return f"{width}“{label}”双向接口；方向和驱动时机由模块内部逻辑控制。"


def inline_comment_block(indent: str, title: str, details: list[str]) -> list[str]:
    result = [f"{indent}// -----------------------------------------------------------------------------"]
    result.append(f"{indent}// {title}")
    result.extend(f"{indent}// {detail}" for detail in details)
    result.append(f"{indent}// -----------------------------------------------------------------------------")
    return result


def annotate_body(module_name: str, body: str) -> str:
    """Add stable review comments without changing executable Verilog tokens."""
    if not body.strip():
        return body

    output: list[str] = []
    constants_seen = False
    declarations_seen = False
    assigns_seen = False
    process_index = 0
    initial_index = 0
    process_docs = PROCESS_DOCS.get(module_name, [])
    process_targets = process_assignment_targets(body)
    assignment_kinds = process_assignment_kinds(body)
    continuous_assignments = continuous_assignment_rhs_by_line(body)
    marker_docs = LINE_MARKER_DOCS.get(module_name, [])
    current_scope_kind: str | None = None
    current_scope_name: str | None = None

    for line_index, line in enumerate(body.splitlines()):
        stripped = line.strip()
        indent = line[: len(line) - len(line.lstrip())]

        if not constants_seen and re.match(r"(?:localparam|parameter)\b", stripped):
            output.extend(inline_comment_block(indent, "常量、状态编码与时序参数", [
                "这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。"
            ]))
            constants_seen = True

        if (not declarations_seen and
                re.match(r"(?:reg|wire|integer)\b", stripped)):
            output.extend(inline_comment_block(indent, "内部网络、寄存器和计数器", [
                "wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。",
                "每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。",
            ]))
            declarations_seen = True

        if not assigns_seen and stripped.startswith("assign "):
            output.extend(inline_comment_block(indent, "组合派生信号", [
                "以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。"
            ]))
            assigns_seen = True

        assign_match = re.match(r"assign\s+([A-Za-z_]\w*)", stripped)
        if assign_match:
            assign_name = assign_match.group(1)
            mapped_name, assign_rhs = continuous_assignments.get(
                line_index, (assign_name, "")
            )
            if mapped_name != assign_name:
                assign_rhs = ""
            assign_semantic = inferred_signal_doc(
                module_name, assign_name, "wire"
            )
            output_semantic = port_doc(
                module_name, assign_name, "output", 1
            )
            if not any(placeholder in output_semantic for placeholder in (
                    "把本模块计算或锁存的结果",
                    "由物理接口或上游模块驱动",
            )):
                assign_semantic = output_semantic
            output.append(
                f"{indent}// [ASSIGN] 组合定义“{assign_name}”："
                f"{assign_semantic} "
                f"{assign_dependency_doc(assign_rhs)}"
            )

        function_match = re.match(
            r"function\s+(?:automatic\s+)?(?:\[[^]]+\]\s+|integer\s+|signed\s+)?(\w+)",
            stripped,
        )
        if function_match:
            name = function_match.group(1)
            current_scope_kind = "function"
            current_scope_name = name
            detail = FUNCTION_DOCS.get(
                (module_name, name),
                inferred_function_doc(name),
            )
            output.extend(inline_comment_block(indent, f"[FUNCTION] 辅助函数: {name}", [detail]))

        task_match = re.match(r"task\s+(\w+)", stripped)
        if task_match:
            name = task_match.group(1)
            current_scope_kind = "task"
            current_scope_name = name
            detail = TASK_DOCS.get(
                (module_name, name),
                inferred_task_doc(name),
            )
            output.extend(inline_comment_block(indent, f"[TASK] 过程任务: {name}", [detail]))

        if re.match(r"always(?:\s+@|\s+#)", stripped):
            if process_index < len(process_docs):
                detail = process_docs[process_index]
            else:
                targets = (
                    process_targets[process_index]
                    if process_index < len(process_targets) else []
                )
                detail = inferred_process_doc(
                    module_name,
                    targets,
                    "posedge" in stripped or "negedge" in stripped,
                )
            process_index += 1
            output.extend(inline_comment_block(
                indent,
                f"[PROCESS] 过程块 {process_index}",
                [detail, "非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。"]
                if "posedge" in stripped else [detail],
            ))

        if re.match(r"initial\b", stripped):
            initial_index += 1
            detail = INITIAL_DOCS.get(
                module_name,
                "初始化过程：在仿真开始或 FPGA 配置完成时建立声明变量的确定初值。",
            )
            output.extend(inline_comment_block(
                indent,
                f"[INITIAL] 初始化块 {initial_index}",
                [detail, "该块不重复执行；后续状态变化由 always 过程负责。"],
            ))

        for pattern, detail in marker_docs:
            if re.match(pattern, line):
                output.extend(inline_comment_block(indent, "关键分支", [detail]))
                break

        constant_match = re.match(
            r"(?:localparam|parameter)(?:\s+\w+)?(?:\s+\[[^]]+\])?\s+"
            r"([A-Za-z_]\w*)\s*=\s*(.*?);",
            stripped,
        )
        if constant_match and "//" not in line:
            const_name, rhs = constant_match.groups()
            line = f"{line}  // {constant_doc(module_name, const_name, rhs)}"

        declaration_match = re.match(
            r"(?:\(\*.*?\*\)\s*)?(reg|wire|integer|input|output|inout)\b\s*"
            r"(?:(?:reg|wire|integer|signed)\s+)*(?:\[[^]]+\]\s*)?"
            r"([A-Za-z_]\w*)",
            stripped,
        )
        if declaration_match and "//" not in line:
            decl_kind, decl_name = declaration_match.groups()
            width_match = re.search(r"\[\s*(\d+)\s*:\s*(\d+)\s*\]", stripped)
            decl_size = None
            if width_match:
                decl_size = abs(int(width_match.group(1)) - int(width_match.group(2))) + 1
            if current_scope_kind and current_scope_name:
                declaration_doc = scoped_declaration_doc(
                    module_name,
                    current_scope_kind,
                    current_scope_name,
                    decl_name,
                    decl_kind,
                    decl_size,
                )
            else:
                effective_kind = decl_kind
                if decl_kind == "reg":
                    kinds = assignment_kinds.get(decl_name, set())
                    if kinds == {"comb"}:
                        effective_kind = "comb_reg"
                    elif kinds == {"clocked_temp"}:
                        effective_kind = "clocked_temp_reg"
                    elif kinds == {"clocked"}:
                        effective_kind = "clocked_reg"
                    elif kinds and kinds <= {"comb", "clocked_temp"}:
                        effective_kind = "comb_reg"
                    elif len(kinds) > 1:
                        effective_kind = "mixed_reg"
                declaration_doc = inferred_signal_doc(
                    module_name, decl_name, effective_kind, decl_size
                )
            line = f"{line}  // {declaration_doc}"

        output.append(line)

        if stripped.startswith("endfunction") or stripped.startswith("endtask"):
            current_scope_kind = None
            current_scope_name = None

    return "\n".join(output)

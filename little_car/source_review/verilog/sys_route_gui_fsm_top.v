// =============================================================================
// 模块: sys_route_gui_fsm_top
// 作用: 当前小车工程的系统顶层，连接所有物理接口和业务子系统。
// 说明: 主时钟为 50 MHz；外部复位先经过 power_on_reset 形成内部 system_rst_n。
// 说明: 按键和蓝牙既可进入自动路线 FSM，也可由 manual_mode 直接控制 C5 四轮。
// 说明: 超声波与 RPLIDAR 的停车请求经 obstacle_stop_merge 合并后送入运动控制器。
// 说明: 路线、传感器、参数和 C5 诊断状态同时送往 HDMI 渲染器与蓝牙上报器。
// 说明: LinkPG 桥把 UWB 下行命令注入原蓝牙 RX，并把 FPGA 状态帧分片回传到 UWB。
//
// Robei 源: little_car/model/sys_route_gui_fsm_top.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module sys_route_gui_fsm_top (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  output wire hdmi_clk_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_clk_n,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d0_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d0_n,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d1_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d1_n,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d2_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d2_n,  // HDMI TMDS 差分物理输出端。
  input wire key_pin1,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire key_pin2,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire key_pin3,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire key_pin4,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire key_pin5,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire ultrasonic_echo,  // 超声波模块 ECHO 回波脉冲输入。
  output wire ultrasonic_trig,  // 超声波模块 TRIG 触发脉冲输出。
  output wire buzzer_out,  // 送往有源/无源蜂鸣器的方波输出。
  input wire bt_uart_rx,  // CH9141K TXD 到 FPGA 的透明串口接收线。
  output wire bt_uart_tx,  // FPGA 到 CH9141K RXD 的透明串口发送线。
  output wire bt_at,  // CH9141K AT 模式控制脚；当前顶层固定拉高进入透传。
  input wire c5_uart_rx,  // C5 总线回传输入；当前控制链未解析该信号。
  output wire c5_uart_tx,  // 发送到四轮 C5 控制总线的 UART 线。
  input wire lidar_uart_rx,  // RPLIDAR C1 TXD 到 FPGA 的 460800 波特输入。
  output wire lidar_uart_tx,  // FPGA 到 RPLIDAR C1 RXD 的命令发送线。
  input wire uwb_uart_rx,  // LinkPG 标签 TXD 到 FPGA 的 115200 bit/s 数据透传输入。
  output wire uwb_uart_tx  // FPGA 到 LinkPG 标签 RXD 的 Modbus 数据透传/ACK 输出。
);

  // ===========================================================================
  // 层次内部网络
  // 每条注释给出驱动端以及所有接收端，便于从顶层沿数据流追踪。
  // ===========================================================================
  // power_on_reset24.system_rst_n -> sys_route_control_fsm1.rst_n, gui_route_renderer2.rst_n, hdmi_top3.rst_n, temp_ultrasonic_echo_source5.rst_n, ultrasonic_obstacle_guard6.rst_n, buzzer_ctrl14.rst_n, buzzer_event_gen15.rst_n, c5_route_controller9.rst_n, c5_motor_frame_driver10.rst_n, bt_route_uart16.rst_n, car_param_controller22.rst_n, rplidar_c1_uart19.rst_n, linkpg_uwb_bridge25.rst_n
  wire sys_route_gui_fsm_top_rst_n;  // 延时释放后的系统低有效复位。 具体驱动端和全部接收端见上一行。
  // gui_route_renderer2.rgb_data -> hdmi_top3.rgb_data
  wire [23:0] gui_route_renderer2_rgb_data;  // GUI 输出或 HDMI 输入的 RGB888 像素。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.route_info -> gui_route_renderer2.route_info, c5_route_controller9.route_info, bt_route_uart16.route_info
  wire [63:0] sys_route_control_fsm1_route_info;  // 64 bit 路线命令队列；最低 2 bit 是下一条命令。 具体驱动端和全部接收端见上一行。
  // hdmi_top3.pixel_clk -> gui_route_renderer2.clk_pix
  wire hdmi_top3_pixel_clk;  // 导出给 GUI 渲染器使用的 50 MHz HDMI 像素时钟。 具体驱动端和全部接收端见上一行。
  // hdmi_top3.pixel_de -> gui_route_renderer2.pixel_de
  wire hdmi_top3_pixel_de;  // 视频数据有效；1 表示位于可见区域。 具体驱动端和全部接收端见上一行。
  // hdmi_top3.pixel_x -> gui_route_renderer2.pixel_x
  wire [11:0] hdmi_top3_pixel_x;  // 当前有效显示区水平像素坐标。 具体驱动端和全部接收端见上一行。
  // hdmi_top3.pixel_y -> gui_route_renderer2.pixel_y
  wire [11:0] hdmi_top3_pixel_y;  // 当前有效显示区垂直像素坐标。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.menu_info -> gui_route_renderer2.menu_info, bt_route_uart16.menu_info
  wire [1:0] sys_route_control_fsm1_menu_info;  // 菜单焦点/页面状态编码。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.state_info -> gui_route_renderer2.state_info, buzzer_event_gen15.state_info, c5_route_controller9.state_info, bt_route_uart16.state_info
  wire [4:0] sys_route_control_fsm1_state_info;  // 路线 FSM 状态编码。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.old_route_info -> gui_route_renderer2.old_route_info
  wire [63:0] sys_route_control_fsm1_old_route_info;  // 已完成路线历史，用于 GUI 和手机显示。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.active_route_steps -> gui_route_renderer2.active_route_steps, buzzer_event_gen15.active_route_steps, c5_route_controller9.active_route_steps, bt_route_uart16.active_route_steps
  wire [5:0] sys_route_control_fsm1_active_route_steps;  // 当前待执行内部命令数量。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.completed_route_steps -> gui_route_renderer2.completed_route_steps, bt_route_uart16.completed_route_steps
  wire [5:0] sys_route_control_fsm1_completed_route_steps;  // 已完成内部命令数量。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.step_length -> gui_route_renderer2.step_length, ultrasonic_obstacle_guard6.step_length
  wire [15:0] sys_route_control_fsm1_step_length;  // 当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.speed -> gui_route_renderer2.speed
  wire [15:0] sys_route_control_fsm1_speed;  // 当前生效的路线整体速度参数，向 GUI 和执行控制链同步。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.total_route_length -> gui_route_renderer2.total_route_length
  wire [15:0] sys_route_control_fsm1_total_route_length;  // 已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.complete_route_length -> gui_route_renderer2.complete_route_length
  wire [15:0] sys_route_control_fsm1_complete_route_length;  // completed_route_steps 乘以 step_length 得到的已完成路线长度。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.remaining_route_length -> gui_route_renderer2.remaining_route_length
  wire [15:0] sys_route_control_fsm1_remaining_route_length;  // active_route_steps 乘以 step_length 得到的剩余路线长度。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.now_x -> gui_route_renderer2.now_x, bt_route_uart16.now_x
  wire [7:0] sys_route_control_fsm1_now_x;  // 小车当前逻辑栅格 X 坐标；自动动作完成后由路线核心更新。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.now_y -> gui_route_renderer2.now_y, bt_route_uart16.now_y
  wire [7:0] sys_route_control_fsm1_now_y;  // 小车当前逻辑栅格 Y 坐标；自动动作完成后由路线核心更新。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.target_x -> gui_route_renderer2.target_x, bt_route_uart16.target_x
  wire [7:0] sys_route_control_fsm1_target_x;  // 编辑路线末端预测 X 坐标；添加或撤销动作时同步变化。 具体驱动端和全部接收端见上一行。
  // sys_route_control_fsm1.target_y -> gui_route_renderer2.target_y, bt_route_uart16.target_y
  wire [7:0] sys_route_control_fsm1_target_y;  // 编辑路线末端预测 Y 坐标；添加或撤销动作时同步变化。 具体驱动端和全部接收端见上一行。
  // temp_ultrasonic_enable_source7.ultrasonic_enable -> ultrasonic_obstacle_guard6.enable
  wire temp_ultrasonic_enable_source7_ultrasonic_enable;  // “超声波”功能使能；为 0 时该功能保持停止或安全空闲状态。 具体驱动端和全部接收端见上一行。
  // temp_key_gpio_config_source13.key_active_low -> gpio_key_adapter12.key_active_low
  wire temp_key_gpio_config_source13_key_active_low;  // 旧/测试按键链固定输出的低有效极性配置。 具体驱动端和全部接收端见上一行。
  // buzzer_event_gen15.power_on_pulse -> buzzer_ctrl14.power_on_pulse
  wire buzzer_event_gen15_power_on_pulse;  // “power / ON”事件脉冲；条件成立时仅保持一个 clk 周期。 具体驱动端和全部接收端见上一行。
  // buzzer_event_gen15.route_add_pulse -> buzzer_ctrl14.route_add_pulse
  wire buzzer_event_gen15_route_add_pulse;  // “路线 / ADD”事件脉冲；条件成立时仅保持一个 clk 周期。 具体驱动端和全部接收端见上一行。
  // buzzer_event_gen15.run_enter_pulse -> buzzer_ctrl14.run_enter_pulse
  wire buzzer_event_gen15_run_enter_pulse;  // “RUN / enter”事件脉冲；条件成立时仅保持一个 clk 周期。 具体驱动端和全部接收端见上一行。
  // buzzer_event_gen15.stop_enter_pulse -> buzzer_ctrl14.stop_enter_pulse
  wire buzzer_event_gen15_stop_enter_pulse;  // “停车 / enter”事件脉冲；条件成立时仅保持一个 clk 周期。 具体驱动端和全部接收端见上一行。
  // c5_route_controller9.run_step_pulse -> sys_route_control_fsm1.run_step_pulse
  wire c5_route_controller9_run_step_pulse;  // 一条运动动作完成的单周期脉冲。 具体驱动端和全部接收端见上一行。
  // temp_c5_motor_config_source11.motor_enable -> c5_route_controller9.motor_enable, c5_motor_frame_driver10.motor_enable
  wire temp_c5_motor_config_source11_motor_enable;  // C5 运动链总使能。 具体驱动端和全部接收端见上一行。
  // temp_c5_motor_config_source11.motor_forward_ms -> c5_route_controller9.forward_ms
  wire [15:0] temp_c5_motor_config_source11_motor_forward_ms;  // 测试/后备配置输出的“电机 / 前进”持续时间，单位 ms。 具体驱动端和全部接收端见上一行。
  // temp_c5_motor_config_source11.motor_turn_ms -> c5_route_controller9.turn_ms
  wire [15:0] temp_c5_motor_config_source11_motor_turn_ms;  // 测试/后备配置输出的“电机 / 转向”持续时间，单位 ms。 具体驱动端和全部接收端见上一行。
  // temp_c5_motor_config_source11.motor_forward_speed -> c5_route_controller9.forward_speed
  wire [9:0] temp_c5_motor_config_source11_motor_forward_speed;  // “电机 / 前进 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。 具体驱动端和全部接收端见上一行。
  // temp_c5_motor_config_source11.motor_turn_speed -> c5_route_controller9.turn_speed
  wire [9:0] temp_c5_motor_config_source11_motor_turn_speed;  // “电机 / 转向 / 速度”数值；在 C5 链中为轮速幅值或 11 bit 二补码有符号速度。 具体驱动端和全部接收端见上一行。
  // c5_route_controller9.lf_speed -> c5_motor_frame_driver10.lf_speed
  wire [10:0] c5_route_controller9_lf_speed;  // 左前轮 11 bit 二补码有符号目标速度。 具体驱动端和全部接收端见上一行。
  // c5_route_controller9.rf_speed -> c5_motor_frame_driver10.rf_speed
  wire [10:0] c5_route_controller9_rf_speed;  // 右前轮 11 bit 二补码有符号目标速度。 具体驱动端和全部接收端见上一行。
  // c5_route_controller9.lr_speed -> c5_motor_frame_driver10.lr_speed
  wire [10:0] c5_route_controller9_lr_speed;  // 左后轮 11 bit 二补码有符号目标速度。 具体驱动端和全部接收端见上一行。
  // c5_route_controller9.rr_speed -> c5_motor_frame_driver10.rr_speed
  wire [10:0] c5_route_controller9_rr_speed;  // 右后轮 11 bit 二补码有符号目标速度。 具体驱动端和全部接收端见上一行。
  // gpio_key_adapter12.key_up_n -> key_low_merge17.key_up_n
  wire gpio_key_adapter12_key_up_n_merge;  // 低有效按键输入/输出；0 表示按下。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_key_up_n -> key_low_merge17.bt_key_up_n
  wire bt_route_uart16_key_up_n_merge;  // 蓝牙方向字符模拟的低有效上键电平；字符到达后保持约 30 ms。 具体驱动端和全部接收端见上一行。
  // gpio_key_adapter12.key_down_n -> key_low_merge17.key_down_n
  wire gpio_key_adapter12_key_down_n_merge;  // 低有效按键输入/输出；0 表示按下。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_key_down_n -> key_low_merge17.bt_key_down_n
  wire bt_route_uart16_key_down_n_merge;  // 蓝牙方向字符模拟的低有效下键电平；字符到达后保持约 30 ms。 具体驱动端和全部接收端见上一行。
  // gpio_key_adapter12.key_left_n -> key_low_merge17.key_left_n
  wire gpio_key_adapter12_key_left_n_merge;  // 低有效按键输入/输出；0 表示按下。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_key_left_n -> key_low_merge17.bt_key_left_n
  wire bt_route_uart16_key_left_n_merge;  // 蓝牙方向字符模拟的低有效左键电平；字符到达后保持约 30 ms。 具体驱动端和全部接收端见上一行。
  // gpio_key_adapter12.key_right_n -> key_low_merge17.key_right_n
  wire gpio_key_adapter12_key_right_n_merge;  // 低有效按键输入/输出；0 表示按下。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_key_right_n -> key_low_merge17.bt_key_right_n
  wire bt_route_uart16_key_right_n_merge;  // 蓝牙方向字符模拟的低有效右键电平；字符到达后保持约 30 ms。 具体驱动端和全部接收端见上一行。
  // gpio_key_adapter12.key_mid_n -> key_low_merge17.key_mid_n
  wire gpio_key_adapter12_key_mid_n_merge;  // 低有效按键输入/输出；0 表示按下。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_key_mid_n -> key_low_merge17.bt_key_mid_n
  wire bt_route_uart16_key_mid_n_merge;  // 蓝牙确认/停车字符模拟的低有效中键电平；字符到达后保持约 30 ms。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_connected -> gui_route_renderer2.bt_connected
  wire bt_route_uart16_bt_connected;  // 仅在持续收到 P/p 心跳时为 1。 具体驱动端和全部接收端见上一行。
  // key_low_merge17.merged_key_up_n -> car_param_controller22.key_up_n, route_key_lock_gate23.key_up_n_in
  wire key_low_merge17_key_up_n;  // 实体按键与蓝牙模拟按键按位 AND 后的“上键/前进”低有效电平。 具体驱动端和全部接收端见上一行。
  // route_key_lock_gate23.key_up_n_out -> sys_route_control_fsm1.key_up_n
  wire route_key_lock_gate23_key_up_n_out;  // 参数设置锁生效时被强制为 1 的“上键/前进”低有效按键输出。 具体驱动端和全部接收端见上一行。
  // key_low_merge17.merged_key_down_n -> car_param_controller22.key_down_n, route_key_lock_gate23.key_down_n_in
  wire key_low_merge17_key_down_n;  // 实体按键与蓝牙模拟按键按位 AND 后的“下键/后退”低有效电平。 具体驱动端和全部接收端见上一行。
  // route_key_lock_gate23.key_down_n_out -> sys_route_control_fsm1.key_down_n
  wire route_key_lock_gate23_key_down_n_out;  // 参数设置锁生效时被强制为 1 的“下键/后退”低有效按键输出。 具体驱动端和全部接收端见上一行。
  // key_low_merge17.merged_key_left_n -> car_param_controller22.key_left_n, route_key_lock_gate23.key_left_n_in
  wire key_low_merge17_key_left_n;  // 实体按键与蓝牙模拟按键按位 AND 后的“左”低有效电平。 具体驱动端和全部接收端见上一行。
  // route_key_lock_gate23.key_left_n_out -> sys_route_control_fsm1.key_left_n
  wire route_key_lock_gate23_key_left_n_out;  // 参数设置锁生效时被强制为 1 的“左”低有效按键输出。 具体驱动端和全部接收端见上一行。
  // key_low_merge17.merged_key_right_n -> car_param_controller22.key_right_n, route_key_lock_gate23.key_right_n_in
  wire key_low_merge17_key_right_n;  // 实体按键与蓝牙模拟按键按位 AND 后的“右”低有效电平。 具体驱动端和全部接收端见上一行。
  // route_key_lock_gate23.key_right_n_out -> sys_route_control_fsm1.key_right_n
  wire route_key_lock_gate23_key_right_n_out;  // 参数设置锁生效时被强制为 1 的“右”低有效按键输出。 具体驱动端和全部接收端见上一行。
  // key_low_merge17.merged_key_mid_n -> car_param_controller22.key_mid_n, route_key_lock_gate23.key_mid_n_in
  wire key_low_merge17_key_mid_n;  // 实体按键与蓝牙模拟按键按位 AND 后的“中键/确认”低有效电平。 具体驱动端和全部接收端见上一行。
  // route_key_lock_gate23.key_mid_n_out -> sys_route_control_fsm1.key_mid_n
  wire route_key_lock_gate23_key_mid_n_out;  // 参数设置锁生效时被强制为 1 的“中键/确认”低有效按键输出。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.route_key_lock -> route_key_lock_gate23.route_key_lock
  wire car_param_controller22_route_key_lock;  // 参数设置期间的路线按键锁；为 1 时实体/蓝牙方向键不会修改路线。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_cfg_mode_pulse -> car_param_controller22.bt_cfg_mode_pulse
  wire bt_route_uart16_bt_cfg_mode_pulse;  // 手机切换参数设置模式的单周期事件脉冲。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_cfg_next_pulse -> car_param_controller22.bt_cfg_next_pulse
  wire bt_route_uart16_bt_cfg_next_pulse;  // 手机选择下一项可调参数的单周期事件脉冲。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_cfg_prev_pulse -> car_param_controller22.bt_cfg_prev_pulse
  wire bt_route_uart16_bt_cfg_prev_pulse;  // 手机选择上一项可调参数的单周期事件脉冲。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_cfg_inc_pulse -> car_param_controller22.bt_cfg_inc_pulse
  wire bt_route_uart16_bt_cfg_inc_pulse;  // 手机把当前所选参数增加一个固定步长的单周期事件脉冲。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_cfg_dec_pulse -> car_param_controller22.bt_cfg_dec_pulse
  wire bt_route_uart16_bt_cfg_dec_pulse;  // 手机把当前所选参数减少一个固定步长的单周期事件脉冲。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_cfg_select -> car_param_controller22.bt_cfg_select
  wire [2:0] bt_route_uart16_bt_cfg_select;  // 手机直接指定的参数编号 0..6；仅在 bt_cfg_select_valid 脉冲周期被参数控制器采纳。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.bt_cfg_select_valid -> car_param_controller22.bt_cfg_select_valid
  wire bt_route_uart16_bt_cfg_select_valid;  // bt_cfg_select 携带新参数编号的单周期有效脉冲。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.setting_mode -> bt_route_uart16.setting_mode
  wire car_param_controller22_setting_mode;  // 手机参数设置模式；为 1 时参数按键优先并锁住路线编辑按键。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.selected_param -> bt_route_uart16.selected_param
  wire [2:0] car_param_controller22_selected_param;  // 当前手机增减操作选中的参数编号 0..6。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.step_length_set -> sys_route_control_fsm1.step_length_set, bt_route_uart16.step_length_set, c5_route_controller9.step_length
  wire [15:0] car_param_controller22_step_length_set;  // 手机设置的单步长度参数。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.speed_set -> sys_route_control_fsm1.speed_set
  wire [15:0] car_param_controller22_speed_set;  // 路线控制采用的整体速度参数。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.overall_speed -> bt_route_uart16.overall_speed, c5_route_controller9.overall_speed
  wire [9:0] car_param_controller22_overall_speed;  // 手机可调的公共速度基准。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.lf_default_speed -> bt_route_uart16.lf_default_speed, c5_route_controller9.lf_default_speed
  wire [9:0] car_param_controller22_lf_default_speed;  // 左前轮校准默认速度。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.rf_default_speed -> bt_route_uart16.rf_default_speed, c5_route_controller9.rf_default_speed
  wire [9:0] car_param_controller22_rf_default_speed;  // 右前轮校准默认速度。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.lr_default_speed -> bt_route_uart16.lr_default_speed, c5_route_controller9.lr_default_speed
  wire [9:0] car_param_controller22_lr_default_speed;  // 左后轮校准默认速度。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.rr_default_speed -> bt_route_uart16.rr_default_speed, c5_route_controller9.rr_default_speed
  wire [9:0] car_param_controller22_rr_default_speed;  // 右后轮校准默认速度。 具体驱动端和全部接收端见上一行。
  // car_param_controller22.lidar_safe_mm -> bt_route_uart16.lidar_safe_mm, rplidar_c1_uart19.safe_distance_mm
  wire [15:0] car_param_controller22_lidar_safe_mm;  // 手机可调雷达正前方安全距离，单位 mm。 具体驱动端和全部接收端见上一行。
  // ultrasonic_obstacle_guard6.obstacle_stop -> obstacle_stop_merge20.ultrasonic_stop, bt_route_uart16.ultrasonic_obstacle_stop
  wire ultrasonic_obstacle_guard6_obstacle_stop;  // 合并或当前模块产生的停车请求。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.obstacle_stop -> obstacle_stop_merge20.lidar_stop, bt_route_uart16.lidar_obstacle_stop
  wire rplidar_c1_uart19_obstacle_stop;  // 合并或当前模块产生的停车请求。 具体驱动端和全部接收端见上一行。
  // obstacle_stop_merge20.obstacle_stop -> sys_route_control_fsm1.obstacle_stop, c5_route_controller9.obstacle_stop, bt_route_uart16.merged_obstacle_stop
  wire obstacle_stop_merge20_obstacle_stop;  // 合并或当前模块产生的停车请求。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.lidar_online -> bt_route_uart16.lidar_online
  wire rplidar_c1_uart19_lidar_online;  // 收到有效雷达扫描样本且未超时。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.front_distance_mm -> bt_route_uart16.lidar_front_distance_mm
  wire [15:0] rplidar_c1_uart19_front_distance_mm_bt;  // 滤波后的正前方最近距离，FFFF 表示无效。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.sector_front_mm -> bt_route_uart16.lidar_sector_front_mm
  wire [15:0] rplidar_c1_uart19_sector_front_mm;  // 雷达 前方 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.sector_front_right_mm -> bt_route_uart16.lidar_sector_front_right_mm
  wire [15:0] rplidar_c1_uart19_sector_front_right_mm;  // 雷达 前方 / 右 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.sector_right_mm -> bt_route_uart16.lidar_sector_right_mm
  wire [15:0] rplidar_c1_uart19_sector_right_mm;  // 雷达 右 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.sector_rear_right_mm -> bt_route_uart16.lidar_sector_rear_right_mm
  wire [15:0] rplidar_c1_uart19_sector_rear_right_mm;  // 雷达 后方 / 右 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.sector_rear_mm -> bt_route_uart16.lidar_sector_rear_mm
  wire [15:0] rplidar_c1_uart19_sector_rear_mm;  // 雷达 后方 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.sector_rear_left_mm -> bt_route_uart16.lidar_sector_rear_left_mm
  wire [15:0] rplidar_c1_uart19_sector_rear_left_mm;  // 雷达 后方 / 左 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.sector_left_mm -> bt_route_uart16.lidar_sector_left_mm
  wire [15:0] rplidar_c1_uart19_sector_left_mm;  // 雷达 左 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。 具体驱动端和全部接收端见上一行。
  // rplidar_c1_uart19.sector_front_left_mm -> bt_route_uart16.lidar_sector_front_left_mm
  wire [15:0] rplidar_c1_uart19_sector_front_left_mm;  // 雷达 前方 / 左 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.manual_mode -> c5_route_controller9.manual_mode
  wire bt_route_uart16_manual_mode;  // 1 表示手机手动模式覆盖自动路线。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.manual_cmd -> c5_route_controller9.manual_cmd
  wire [2:0] bt_route_uart16_manual_cmd;  // 手动动作：0停、1前、2后、3左转、4右转。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.lidar_enable -> rplidar_c1_uart19.scan_enable
  wire bt_route_uart16_lidar_enable;  // “激光雷达”功能使能；为 0 时该功能保持停止或安全空闲状态。 具体驱动端和全部接收端见上一行。
  // c5_route_controller9.motor_blocked -> bt_route_uart16.motor_blocked
  wire c5_route_controller9_motor_blocked;  // 当前运动命令被避障逻辑拦截。 具体驱动端和全部接收端见上一行。
  // ultrasonic_obstacle_guard6.distance_cm -> bt_route_uart16.ultrasonic_distance_cm
  wire [15:0] ultrasonic_obstacle_guard6_distance_cm;  // 超声波距离，单位 cm；FFFF 表示超时无效。 具体驱动端和全部接收端见上一行。
  // ultrasonic_obstacle_guard6.distance_valid -> bt_route_uart16.ultrasonic_distance_valid
  wire ultrasonic_obstacle_guard6_distance_valid;  // 当前周期更新了超声波测距结果。 具体驱动端和全部接收端见上一行。
  // ultrasonic_obstacle_guard6.echo_timeout -> bt_route_uart16.ultrasonic_echo_timeout
  wire ultrasonic_obstacle_guard6_echo_timeout;  // 等待或测量回波超过 30 ms。 具体驱动端和全部接收端见上一行。
  // c5_motor_frame_driver10.frame_count -> bt_route_uart16.c5_frame_count
  wire [15:0] c5_motor_frame_driver10_frame_count;  // 已经完整发送的 C5 帧数量。 具体驱动端和全部接收端见上一行。
  // c5_motor_frame_driver10.last_lf_pulse -> bt_route_uart16.c5_last_lf_pulse
  wire [11:0] c5_motor_frame_driver10_last_lf_pulse;  // 最近一帧发送给左前轮的 C5 脉宽诊断值，单位 us。 具体驱动端和全部接收端见上一行。
  // c5_motor_frame_driver10.last_rf_pulse -> bt_route_uart16.c5_last_rf_pulse
  wire [11:0] c5_motor_frame_driver10_last_rf_pulse;  // 最近一帧发送给右前轮的 C5 脉宽诊断值，单位 us。 具体驱动端和全部接收端见上一行。
  // c5_motor_frame_driver10.last_lr_pulse -> bt_route_uart16.c5_last_lr_pulse
  wire [11:0] c5_motor_frame_driver10_last_lr_pulse;  // 最近一帧发送给左后轮的 C5 脉宽诊断值，单位 us。 具体驱动端和全部接收端见上一行。
  // c5_motor_frame_driver10.last_rr_pulse -> bt_route_uart16.c5_last_rr_pulse
  wire [11:0] c5_motor_frame_driver10_last_rr_pulse;  // 最近一帧发送给右后轮的 C5 脉宽诊断值，单位 us。 具体驱动端和全部接收端见上一行。
  // c5_motor_frame_driver10.stop_frames_complete -> bt_route_uart16.c5_stop_frames_complete
  wire c5_motor_frame_driver10_stop_frames_complete;  // 强化停车序列三帧已经完成。 具体驱动端和全部接收端见上一行。
  // c5_motor_frame_driver10.stop_frames_remaining -> bt_route_uart16.c5_stop_frames_remaining
  wire [1:0] c5_motor_frame_driver10_stop_frames_remaining;  // 尚未发送完成的强化停车帧数量。 具体驱动端和全部接收端见上一行。
  // c5_motor_frame_driver10.frame_busy -> bt_route_uart16.c5_frame_busy
  wire c5_motor_frame_driver10_frame_busy;  // 1 表示 C5 UART 正在发送完整帧。 具体驱动端和全部接收端见上一行。
  // bt_route_uart16.motor_stop_pulse -> c5_motor_frame_driver10.force_stop
  wire bt_route_uart16_motor_stop_pulse;  // 高优先级强制停车单周期脉冲。 具体驱动端和全部接收端见上一行。
  // linkpg_uwb_bridge25.bt_uart_rx_merged -> bt_route_uart16.bt_uart_rx
  wire linkpg_uwb_bridge25_bt_uart_rx_merged;  // CH9141K RX 与 UWB 命令注入线按线与后的 UART 输入，送入 bt_route_uart。 具体驱动端和全部接收端见上一行。

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“bt_at”：CH9141K AT 模式控制脚；当前顶层固定拉高进入透传。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign bt_at = 1'b1;

  // ---------------------------------------------------------------------------
  // 子模块实例: power_on_reset24
  // 类型: power_on_reset
  // 作用: 上电复位延时器，在外部复位释放后继续保持内部复位约 20 ms。
  // 连接说明: POR_CYCLES=1,000,000；50 MHz 下每周期 20 ns，因此延时为 20 ms。
  // ---------------------------------------------------------------------------
  power_on_reset
  power_on_reset24 (
    .clk(clk),
    .ext_rst_n(rst_n),
    .system_rst_n(sys_route_gui_fsm_top_rst_n)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: sys_route_control_fsm1
  // 类型: sys_route_control_fsm
  // 作用: 路线控制层次封装：五路按键消抖后进入实际路线核心。
  // 连接说明: 该模块自身没有业务状态，只负责 key_*_n 到 key_*_pulse 的转换与层次连接。
  // 连接说明: 当前实例类型拼写为 route_contol_core，这是工程真实依赖，不是注释笔误。
  // ---------------------------------------------------------------------------
  sys_route_control_fsm
  sys_route_control_fsm1 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .key_up_n(route_key_lock_gate23_key_up_n_out),
    .key_down_n(route_key_lock_gate23_key_down_n_out),
    .key_left_n(route_key_lock_gate23_key_left_n_out),
    .key_right_n(route_key_lock_gate23_key_right_n_out),
    .key_mid_n(route_key_lock_gate23_key_mid_n_out),
    .route_info(sys_route_control_fsm1_route_info),
    .menu_info(sys_route_control_fsm1_menu_info),
    .run_step_pulse(c5_route_controller9_run_step_pulse),
    .obstacle_stop(obstacle_stop_merge20_obstacle_stop),
    .step_length_set(car_param_controller22_step_length_set),
    .speed_set(car_param_controller22_speed_set),
    .state_info(sys_route_control_fsm1_state_info),
    .old_route_info(sys_route_control_fsm1_old_route_info),
    .active_route_steps(sys_route_control_fsm1_active_route_steps),
    .completed_route_steps(sys_route_control_fsm1_completed_route_steps),
    .step_length(sys_route_control_fsm1_step_length),
    .speed(sys_route_control_fsm1_speed),
    .total_route_length(sys_route_control_fsm1_total_route_length),
    .complete_route_length(sys_route_control_fsm1_complete_route_length),
    .remaining_route_length(sys_route_control_fsm1_remaining_route_length),
    .now_x(sys_route_control_fsm1_now_x),
    .now_y(sys_route_control_fsm1_now_y),
    .target_x(sys_route_control_fsm1_target_x),
    .target_y(sys_route_control_fsm1_target_y)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: gui_route_renderer2
  // 类型: gui_route_renderer
  // 作用: 按像素坐标组合绘制小车状态、菜单、路线网格、当前位置和目标。
  // 连接说明: 该模块不保存帧缓存，每个像素直接由坐标、路线和状态组合计算 RGB。
  // 连接说明: 路线命令被展开为水平/垂直线段；字符使用 5x7 点阵放大两倍绘制。
  // ---------------------------------------------------------------------------
  gui_route_renderer
  gui_route_renderer2 (
    .pixel_x(hdmi_top3_pixel_x),
    .pixel_y(hdmi_top3_pixel_y),
    .pixel_de(hdmi_top3_pixel_de),
    .route_info(sys_route_control_fsm1_route_info),
    .rgb_data(gui_route_renderer2_rgb_data),
    .clk_pix(hdmi_top3_pixel_clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .menu_info(sys_route_control_fsm1_menu_info),
    .state_info(sys_route_control_fsm1_state_info),
    .old_route_info(sys_route_control_fsm1_old_route_info),
    .active_route_steps(sys_route_control_fsm1_active_route_steps),
    .completed_route_steps(sys_route_control_fsm1_completed_route_steps),
    .step_length(sys_route_control_fsm1_step_length),
    .speed(sys_route_control_fsm1_speed),
    .total_route_length(sys_route_control_fsm1_total_route_length),
    .complete_route_length(sys_route_control_fsm1_complete_route_length),
    .remaining_route_length(sys_route_control_fsm1_remaining_route_length),
    .now_x(sys_route_control_fsm1_now_x),
    .now_y(sys_route_control_fsm1_now_y),
    .target_x(sys_route_control_fsm1_target_x),
    .target_y(sys_route_control_fsm1_target_y),
    .bt_connected(bt_route_uart16_bt_connected)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_top3
  // 类型: hdmi_top
  // 作用: 完整 HDMI 发送流水线的层次封装。
  // 连接说明: 时钟 -> 1024x600 时序 -> RGB 拆分 -> 三路 TMDS 编码 -> 四路串化差分输出。
  // 连接说明: 蓝色通道承载 hsync/vsync 控制码，红绿通道消隐期控制输入固定为 0。
  // ---------------------------------------------------------------------------
  hdmi_top
  hdmi_top3 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .hdmi_clk_p(hdmi_clk_p),
    .hdmi_d0_p(hdmi_d0_p),
    .hdmi_d1_p(hdmi_d1_p),
    .hdmi_d2_p(hdmi_d2_p),
    .hdmi_clk_n(hdmi_clk_n),
    .hdmi_d0_n(hdmi_d0_n),
    .hdmi_d1_n(hdmi_d1_n),
    .hdmi_d2_n(hdmi_d2_n),
    .rgb_data(gui_route_renderer2_rgb_data),
    .pixel_x(hdmi_top3_pixel_x),
    .pixel_y(hdmi_top3_pixel_y),
    .pixel_de(hdmi_top3_pixel_de),
    .pixel_clk(hdmi_top3_pixel_clk)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: temp_run_status_source4
  // 类型: temp_run_status_source
  // 作用: 临时运行状态源。
  // 连接说明: 为旧模块测试提供固定 state_info。
  // ---------------------------------------------------------------------------
  temp_run_status_source
  temp_run_status_source4 (
    .run_step_pulse(),
    .obstacle_stop(),
    .step_length_set(),
    .speed_set()
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: temp_ultrasonic_echo_source5
  // 类型: temp_ultrasonic_echo_source
  // 作用: 超声波回波仿真/临时源。
  // 连接说明: 硬件顶层实际使用外部 ultrasonic_echo。
  // ---------------------------------------------------------------------------
  temp_ultrasonic_echo_source
  temp_ultrasonic_echo_source5 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .trig(),
    .echo(),
    .fake_distance_cm()
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: ultrasonic_obstacle_guard6
  // 类型: ultrasonic_obstacle_guard
  // 作用: 前向超声波触发、回波计时、距离换算和独立停车判断。
  // 连接说明: 每 60 ms 测量一次，触发高电平 10 us，等待及测量超时均为 30 ms。
  // 连接说明: 距离厘米数约等于 echo_cycles/2900；超时输出 FFFF，但不会误触发停车。
  // ---------------------------------------------------------------------------
  ultrasonic_obstacle_guard
  ultrasonic_obstacle_guard6 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .echo(ultrasonic_echo),
    .step_length(sys_route_control_fsm1_step_length),
    .trig(ultrasonic_trig),
    .distance_cm(ultrasonic_obstacle_guard6_distance_cm),
    .distance_valid(ultrasonic_obstacle_guard6_distance_valid),
    .obstacle_stop(ultrasonic_obstacle_guard6_obstacle_stop),
    .echo_timeout(ultrasonic_obstacle_guard6_echo_timeout),
    .enable(temp_ultrasonic_enable_source7_ultrasonic_enable)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: temp_ultrasonic_enable_source7
  // 类型: temp_ultrasonic_enable_source
  // 作用: 超声波固定使能源。
  // 连接说明: 当前顶层用它保持超声测距开启。
  // ---------------------------------------------------------------------------
  temp_ultrasonic_enable_source
  temp_ultrasonic_enable_source7 (
    .ultrasonic_enable(temp_ultrasonic_enable_source7_ultrasonic_enable)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: gpio_key_adapter12
  // 类型: gpio_key_adapter
  // 作用: 物理按键极性适配器，把板级按键转换为统一的低有效方向/确认按键。
  // 连接说明: key_active_low 决定输入是否反相，后级始终按低有效 key_*_n 解释。
  // ---------------------------------------------------------------------------
  gpio_key_adapter
  gpio_key_adapter12 (
    .key_pin1(key_pin1),
    .key_pin2(key_pin2),
    .key_pin3(key_pin3),
    .key_pin4(key_pin4),
    .key_pin5(key_pin5),
    .key_active_low(temp_key_gpio_config_source13_key_active_low),
    .key_up_n(gpio_key_adapter12_key_up_n_merge),
    .key_down_n(gpio_key_adapter12_key_down_n_merge),
    .key_left_n(gpio_key_adapter12_key_left_n_merge),
    .key_right_n(gpio_key_adapter12_key_right_n_merge),
    .key_mid_n(gpio_key_adapter12_key_mid_n_merge)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: temp_key_gpio_config_source13
  // 类型: temp_key_gpio_config_source
  // 作用: 板级按键极性常量源。
  // 连接说明: 当前输出决定 gpio_key_adapter 是否反相输入。
  // ---------------------------------------------------------------------------
  temp_key_gpio_config_source
  temp_key_gpio_config_source13 (
    .key_active_low(temp_key_gpio_config_source13_key_active_low)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: buzzer_ctrl14
  // 类型: buzzer_ctrl
  // 作用: 蜂鸣器音调/节奏发生器。
  // 连接说明: 根据事件选择提示音序列，通过计数分频生成 buzzer_out。
  // ---------------------------------------------------------------------------
  buzzer_ctrl
  buzzer_ctrl14 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .power_on_pulse(buzzer_event_gen15_power_on_pulse),
    .route_add_pulse(buzzer_event_gen15_route_add_pulse),
    .run_enter_pulse(buzzer_event_gen15_run_enter_pulse),
    .stop_enter_pulse(buzzer_event_gen15_stop_enter_pulse),
    .buzzer_out(buzzer_out)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: buzzer_event_gen15
  // 类型: buzzer_event_gen
  // 作用: 从复位、路线编辑和运行状态边沿产生蜂鸣事件脉冲。
  // 连接说明: 输出事件进入 buzzer_ctrl 的优先级音效状态机。
  // ---------------------------------------------------------------------------
  buzzer_event_gen
  buzzer_event_gen15 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .state_info(sys_route_control_fsm1_state_info),
    .active_route_steps(sys_route_control_fsm1_active_route_steps),
    .power_on_pulse(buzzer_event_gen15_power_on_pulse),
    .route_add_pulse(buzzer_event_gen15_route_add_pulse),
    .run_enter_pulse(buzzer_event_gen15_run_enter_pulse),
    .stop_enter_pulse(buzzer_event_gen15_stop_enter_pulse)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: bt_route_uart16
  // 类型: bt_route_uart
  // 作用: CH9141K 透明串口协议、手动控制、参数命令和原子状态上报调度器。
  // 连接说明: UART 为 115200 8N1；P/p 心跳维持连接，3 s 未收到心跳则 bt_connected 清零。
  // 连接说明: 手动运动命令有 300 ms 看门狗，超时后强制 manual_cmd=STOP 并产生停车脉冲。
  // ---------------------------------------------------------------------------
  bt_route_uart
  bt_route_uart16 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .bt_uart_rx(linkpg_uwb_bridge25_bt_uart_rx_merged),
    .menu_info(sys_route_control_fsm1_menu_info),
    .state_info(sys_route_control_fsm1_state_info),
    .route_info(sys_route_control_fsm1_route_info),
    .active_route_steps(sys_route_control_fsm1_active_route_steps),
    .completed_route_steps(sys_route_control_fsm1_completed_route_steps),
    .now_x(sys_route_control_fsm1_now_x),
    .now_y(sys_route_control_fsm1_now_y),
    .target_x(sys_route_control_fsm1_target_x),
    .target_y(sys_route_control_fsm1_target_y),
    .setting_mode(car_param_controller22_setting_mode),
    .selected_param(car_param_controller22_selected_param),
    .step_length_set(car_param_controller22_step_length_set),
    .overall_speed(car_param_controller22_overall_speed),
    .lf_default_speed(car_param_controller22_lf_default_speed),
    .rf_default_speed(car_param_controller22_rf_default_speed),
    .lr_default_speed(car_param_controller22_lr_default_speed),
    .rr_default_speed(car_param_controller22_rr_default_speed),
    .lidar_online(rplidar_c1_uart19_lidar_online),
    .lidar_obstacle_stop(rplidar_c1_uart19_obstacle_stop),
    .lidar_front_distance_mm(rplidar_c1_uart19_front_distance_mm_bt),
    .lidar_safe_mm(car_param_controller22_lidar_safe_mm),
    .lidar_sector_front_mm(rplidar_c1_uart19_sector_front_mm),
    .lidar_sector_front_right_mm(rplidar_c1_uart19_sector_front_right_mm),
    .lidar_sector_right_mm(rplidar_c1_uart19_sector_right_mm),
    .lidar_sector_rear_right_mm(rplidar_c1_uart19_sector_rear_right_mm),
    .lidar_sector_rear_mm(rplidar_c1_uart19_sector_rear_mm),
    .lidar_sector_rear_left_mm(rplidar_c1_uart19_sector_rear_left_mm),
    .lidar_sector_left_mm(rplidar_c1_uart19_sector_left_mm),
    .lidar_sector_front_left_mm(rplidar_c1_uart19_sector_front_left_mm),
    .bt_uart_tx(bt_uart_tx),
    .bt_connected(bt_route_uart16_bt_connected),
    .bt_key_up_n(bt_route_uart16_key_up_n_merge),
    .bt_key_down_n(bt_route_uart16_key_down_n_merge),
    .bt_key_left_n(bt_route_uart16_key_left_n_merge),
    .bt_key_right_n(bt_route_uart16_key_right_n_merge),
    .bt_key_mid_n(bt_route_uart16_key_mid_n_merge),
    .bt_cfg_mode_pulse(bt_route_uart16_bt_cfg_mode_pulse),
    .bt_cfg_next_pulse(bt_route_uart16_bt_cfg_next_pulse),
    .bt_cfg_prev_pulse(bt_route_uart16_bt_cfg_prev_pulse),
    .bt_cfg_inc_pulse(bt_route_uart16_bt_cfg_inc_pulse),
    .bt_cfg_dec_pulse(bt_route_uart16_bt_cfg_dec_pulse),
    .bt_cfg_select(bt_route_uart16_bt_cfg_select),
    .bt_cfg_select_valid(bt_route_uart16_bt_cfg_select_valid),
    .manual_mode(bt_route_uart16_manual_mode),
    .manual_cmd(bt_route_uart16_manual_cmd),
    .lidar_enable(bt_route_uart16_lidar_enable),
    .ultrasonic_obstacle_stop(ultrasonic_obstacle_guard6_obstacle_stop),
    .merged_obstacle_stop(obstacle_stop_merge20_obstacle_stop),
    .motor_blocked(c5_route_controller9_motor_blocked),
    .ultrasonic_distance_cm(ultrasonic_obstacle_guard6_distance_cm),
    .ultrasonic_distance_valid(ultrasonic_obstacle_guard6_distance_valid),
    .ultrasonic_echo_timeout(ultrasonic_obstacle_guard6_echo_timeout),
    .c5_frame_count(c5_motor_frame_driver10_frame_count),
    .c5_last_lf_pulse(c5_motor_frame_driver10_last_lf_pulse),
    .c5_last_rf_pulse(c5_motor_frame_driver10_last_rf_pulse),
    .c5_last_lr_pulse(c5_motor_frame_driver10_last_lr_pulse),
    .c5_last_rr_pulse(c5_motor_frame_driver10_last_rr_pulse),
    .c5_stop_frames_complete(c5_motor_frame_driver10_stop_frames_complete),
    .c5_stop_frames_remaining(c5_motor_frame_driver10_stop_frames_remaining),
    .c5_frame_busy(c5_motor_frame_driver10_frame_busy),
    .motor_stop_pulse(bt_route_uart16_motor_stop_pulse)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: key_low_merge17
  // 类型: key_low_merge
  // 作用: 低有效按键合并器，把实体按键和蓝牙模拟按键接到同一条路线输入链。
  // 连接说明: 低有效信号使用逻辑与合并：任意来源拉低都会形成一次有效按键。
  // ---------------------------------------------------------------------------
  key_low_merge
  key_low_merge17 (
    .key_up_n(gpio_key_adapter12_key_up_n_merge),
    .key_down_n(gpio_key_adapter12_key_down_n_merge),
    .key_left_n(gpio_key_adapter12_key_left_n_merge),
    .key_right_n(gpio_key_adapter12_key_right_n_merge),
    .key_mid_n(gpio_key_adapter12_key_mid_n_merge),
    .bt_key_up_n(bt_route_uart16_key_up_n_merge),
    .bt_key_down_n(bt_route_uart16_key_down_n_merge),
    .bt_key_left_n(bt_route_uart16_key_left_n_merge),
    .bt_key_right_n(bt_route_uart16_key_right_n_merge),
    .bt_key_mid_n(bt_route_uart16_key_mid_n_merge),
    .merged_key_up_n(key_low_merge17_key_up_n),
    .merged_key_down_n(key_low_merge17_key_down_n),
    .merged_key_left_n(key_low_merge17_key_left_n),
    .merged_key_right_n(key_low_merge17_key_right_n),
    .merged_key_mid_n(key_low_merge17_key_mid_n)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: temp_c5_motor_config_source11
  // 类型: temp_c5_motor_config_source
  // 作用: 当前 C5 执行链使用的固定使能、默认时间和后备速度配置源。
  // 连接说明: 手机参数有效时整体/单轮速度会覆盖这里的后备速度。
  // ---------------------------------------------------------------------------
  temp_c5_motor_config_source
  temp_c5_motor_config_source11 (
    .motor_enable(temp_c5_motor_config_source11_motor_enable),
    .motor_forward_ms(temp_c5_motor_config_source11_motor_forward_ms),
    .motor_turn_ms(temp_c5_motor_config_source11_motor_turn_ms),
    .motor_forward_speed(temp_c5_motor_config_source11_motor_forward_speed),
    .motor_turn_speed(temp_c5_motor_config_source11_motor_turn_speed)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: c5_route_controller9
  // 类型: c5_route_controller
  // 作用: 把自动路线或手机手动命令转换为四轮 11 bit 二补码有符号速度。
  // 连接说明: 每轮速度 = clamp(整体速度 + 该轮默认速度 - 700, 100, 900)。
  // 连接说明: 原地转向时左、右侧符号相反，并在速度大于 150 时将转向幅值减 50。
  // ---------------------------------------------------------------------------
  c5_route_controller
  c5_route_controller9 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .motor_enable(temp_c5_motor_config_source11_motor_enable),
    .state_info(sys_route_control_fsm1_state_info),
    .route_info(sys_route_control_fsm1_route_info),
    .active_route_steps(sys_route_control_fsm1_active_route_steps),
    .forward_ms(temp_c5_motor_config_source11_motor_forward_ms),
    .turn_ms(temp_c5_motor_config_source11_motor_turn_ms),
    .forward_speed(temp_c5_motor_config_source11_motor_forward_speed),
    .turn_speed(temp_c5_motor_config_source11_motor_turn_speed),
    .step_length(car_param_controller22_step_length_set),
    .overall_speed(car_param_controller22_overall_speed),
    .lf_default_speed(car_param_controller22_lf_default_speed),
    .rf_default_speed(car_param_controller22_rf_default_speed),
    .lr_default_speed(car_param_controller22_lr_default_speed),
    .rr_default_speed(car_param_controller22_rr_default_speed),
    .run_step_pulse(c5_route_controller9_run_step_pulse),
    .lf_speed(c5_route_controller9_lf_speed),
    .rf_speed(c5_route_controller9_rf_speed),
    .lr_speed(c5_route_controller9_lr_speed),
    .rr_speed(c5_route_controller9_rr_speed),
    .motion_busy(),
    .current_cmd(),
    .manual_mode(bt_route_uart16_manual_mode),
    .manual_cmd(bt_route_uart16_manual_cmd),
    .obstacle_stop(obstacle_stop_merge20_obstacle_stop),
    .motor_blocked(c5_route_controller9_motor_blocked)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: c5_motor_test_pattern18
  // 类型: c5_motor_test_pattern
  // 作用: C5 四轮固定测试序列发生器。
  // 连接说明: 当前系统顶层保留了实例但输出未接入执行链，因此不会控制实际轮子。
  // ---------------------------------------------------------------------------
  c5_motor_test_pattern
  c5_motor_test_pattern18 (
    .clk(),
    .rst_n(),
    .lf_speed(),
    .rf_speed(),
    .lr_speed(),
    .rr_speed()
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: c5_motor_frame_driver10
  // 类型: c5_motor_frame_driver
  // 作用: 将四轮有符号速度编码为 C5 总线 ASCII 帧，并实现启动/停车保护。
  // 连接说明: 电机 ID 006/007/008/009；运动帧 62 字节，串口 115200 8N1。
  // 连接说明: 左轮脉宽=1500+速度，右轮脉宽=1500-速度，用于补偿左右安装方向。
  // ---------------------------------------------------------------------------
  c5_motor_frame_driver
  c5_motor_frame_driver10 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .motor_enable(temp_c5_motor_config_source11_motor_enable),
    .lf_speed(c5_route_controller9_lf_speed),
    .rf_speed(c5_route_controller9_rf_speed),
    .lr_speed(c5_route_controller9_lr_speed),
    .rr_speed(c5_route_controller9_rr_speed),
    .c5_bus_tx(c5_uart_tx),
    .frame_busy(c5_motor_frame_driver10_frame_busy),
    .force_stop(bt_route_uart16_motor_stop_pulse),
    .frame_count(c5_motor_frame_driver10_frame_count),
    .last_lf_pulse(c5_motor_frame_driver10_last_lf_pulse),
    .last_rf_pulse(c5_motor_frame_driver10_last_rf_pulse),
    .last_lr_pulse(c5_motor_frame_driver10_last_lr_pulse),
    .last_rr_pulse(c5_motor_frame_driver10_last_rr_pulse),
    .stop_frames_complete(c5_motor_frame_driver10_stop_frames_complete),
    .stop_frames_remaining(c5_motor_frame_driver10_stop_frames_remaining)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: car_param_controller22
  // 类型: car_param_controller
  // 作用: 保存并限幅所有手机可调运行参数。
  // 连接说明: 参数包括整体速度、单步长度、四轮默认速度和雷达安全距离。
  // 连接说明: 速度范围 100..900、步长 5..80、雷达阈值 150..1500 mm，所有修改均饱和限幅。
  // ---------------------------------------------------------------------------
  car_param_controller
  car_param_controller22 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .key_up_n(key_low_merge17_key_up_n),
    .key_down_n(key_low_merge17_key_down_n),
    .key_left_n(key_low_merge17_key_left_n),
    .key_right_n(key_low_merge17_key_right_n),
    .key_mid_n(key_low_merge17_key_mid_n),
    .bt_cfg_mode_pulse(bt_route_uart16_bt_cfg_mode_pulse),
    .bt_cfg_next_pulse(bt_route_uart16_bt_cfg_next_pulse),
    .bt_cfg_prev_pulse(bt_route_uart16_bt_cfg_prev_pulse),
    .bt_cfg_inc_pulse(bt_route_uart16_bt_cfg_inc_pulse),
    .bt_cfg_dec_pulse(bt_route_uart16_bt_cfg_dec_pulse),
    .bt_cfg_select(bt_route_uart16_bt_cfg_select),
    .bt_cfg_select_valid(bt_route_uart16_bt_cfg_select_valid),
    .route_key_lock(car_param_controller22_route_key_lock),
    .setting_mode(car_param_controller22_setting_mode),
    .selected_param(car_param_controller22_selected_param),
    .step_length_set(car_param_controller22_step_length_set),
    .speed_set(car_param_controller22_speed_set),
    .overall_speed(car_param_controller22_overall_speed),
    .lf_default_speed(car_param_controller22_lf_default_speed),
    .rf_default_speed(car_param_controller22_rf_default_speed),
    .lr_default_speed(car_param_controller22_lr_default_speed),
    .rr_default_speed(car_param_controller22_rr_default_speed),
    .lidar_safe_mm(car_param_controller22_lidar_safe_mm)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: route_key_lock_gate23
  // 类型: route_key_lock_gate
  // 作用: 路线按键门控器，手机处于参数设置模式时阻断路线编辑按键。
  // 连接说明: 锁定时所有输出保持 1，即低有效按键的未按下状态。
  // ---------------------------------------------------------------------------
  route_key_lock_gate
  route_key_lock_gate23 (
    .route_key_lock(car_param_controller22_route_key_lock),
    .key_up_n_in(key_low_merge17_key_up_n),
    .key_down_n_in(key_low_merge17_key_down_n),
    .key_left_n_in(key_low_merge17_key_left_n),
    .key_right_n_in(key_low_merge17_key_right_n),
    .key_mid_n_in(key_low_merge17_key_mid_n),
    .key_up_n_out(route_key_lock_gate23_key_up_n_out),
    .key_down_n_out(route_key_lock_gate23_key_down_n_out),
    .key_left_n_out(route_key_lock_gate23_key_left_n_out),
    .key_right_n_out(route_key_lock_gate23_key_right_n_out),
    .key_mid_n_out(route_key_lock_gate23_key_mid_n_out)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: rplidar_c1_uart19
  // 类型: rplidar_c1_uart
  // 作用: RPLIDAR C1 启停、串口接收、扫描节点解析、八扇区滤波和轮廓避障。
  // 连接说明: UART 为 460800 8N1；启动序列为等待 0.5 s、A5 25 停止、等待 0.1 s、A5 20 扫描。
  // 连接说明: 描述符 A5 5A 05 00 00 40 81 匹配后进入连续 5 字节扫描节点解析。
  // ---------------------------------------------------------------------------
  rplidar_c1_uart
  rplidar_c1_uart19 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .lidar_uart_rx(lidar_uart_rx),
    .safe_distance_mm(car_param_controller22_lidar_safe_mm),
    .lidar_uart_tx(lidar_uart_tx),
    .lidar_online(rplidar_c1_uart19_lidar_online),
    .scan_active(),
    .sample_valid(),
    .new_scan(),
    .quality(),
    .angle_q6(),
    .distance_q2(),
    .front_distance_mm(rplidar_c1_uart19_front_distance_mm_bt),
    .front_valid(),
    .obstacle_stop(rplidar_c1_uart19_obstacle_stop),
    .sector_front_mm(rplidar_c1_uart19_sector_front_mm),
    .sector_front_right_mm(rplidar_c1_uart19_sector_front_right_mm),
    .sector_right_mm(rplidar_c1_uart19_sector_right_mm),
    .sector_rear_right_mm(rplidar_c1_uart19_sector_rear_right_mm),
    .sector_rear_mm(rplidar_c1_uart19_sector_rear_mm),
    .sector_rear_left_mm(rplidar_c1_uart19_sector_rear_left_mm),
    .sector_left_mm(rplidar_c1_uart19_sector_left_mm),
    .sector_front_left_mm(rplidar_c1_uart19_sector_front_left_mm),
    .scan_enable(bt_route_uart16_lidar_enable)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: obstacle_stop_merge20
  // 类型: obstacle_stop_merge
  // 作用: 双源避障合并器。
  // 连接说明: 最终 obstacle_stop = ultrasonic_stop OR lidar_stop；来源仍分别回传给手机。
  // ---------------------------------------------------------------------------
  obstacle_stop_merge
  obstacle_stop_merge20 (
    .ultrasonic_stop(ultrasonic_obstacle_guard6_obstacle_stop),
    .lidar_stop(rplidar_c1_uart19_obstacle_stop),
    .obstacle_stop(obstacle_stop_merge20_obstacle_stop)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: linkpg_uwb_bridge25
  // 类型: linkpg_uwb_bridge
  // 作用: LinkPG 标签侧双向桥，把 UWB 数据透传帧与现有蓝牙单字符协议无侵入连接。
  // 连接说明: UWB 与蓝牙侧均为 115200 bit/s 8N1；蓝牙物理 RX 和 UWB 注入线按线与合并。
  // 连接说明: 下行只接受带序号和 CRC8 的白名单单字符命令，并用两秒窗口抑制重复执行。
  // ---------------------------------------------------------------------------
  linkpg_uwb_bridge
  linkpg_uwb_bridge25 (
    .clk(clk),
    .rst_n(sys_route_gui_fsm_top_rst_n),
    .uwb_uart_rx(uwb_uart_rx),
    .bt_uart_rx_in(bt_uart_rx),
    .bt_uart_tx_tap(bt_uart_tx),
    .uwb_uart_tx(uwb_uart_tx),
    .bt_uart_rx_merged(linkpg_uwb_bridge25_bt_uart_rx_merged),
    .uwb_link_online(),
    .rx_packet_count(),
    .rx_crc_error_count(),
    .tx_packet_count(),
    .telemetry_drop_count()
  );

endmodule

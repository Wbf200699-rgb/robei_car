// =============================================================================
// 模块: bt_route_uart
// 作用: CH9141K 透明串口协议、手动控制、参数命令和原子状态上报调度器。
// 说明: UART 为 115200 8N1；P/p 心跳维持连接，3 s 未收到心跳则 bt_connected 清零。
// 说明: 手动运动命令有 300 ms 看门狗，超时后强制 manual_cmd=STOP 并产生停车脉冲。
// 说明: 短回复、@S 路线状态和 @L 雷达状态共享唯一发送状态机，保证帧不会互相穿插。
// 说明: @S 在数据变化、问号查询或 200 ms 周期到达时发送；@L 携带八扇区和障碍来源。
//
// Robei 源: little_car/model/bt_route_uart.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module bt_route_uart (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire bt_uart_rx,  // CH9141K TXD 到 FPGA 的透明串口接收线。
  input wire [1:0] menu_info,  // 菜单焦点/页面状态编码。
  input wire [4:0] state_info,  // 路线 FSM 状态编码。
  input wire [63:0] route_info,  // 64 bit 路线命令队列；最低 2 bit 是下一条命令。
  input wire [5:0] active_route_steps,  // 当前待执行内部命令数量。
  input wire [5:0] completed_route_steps,  // 已完成内部命令数量。
  input wire [7:0] now_x,  // 小车当前逻辑栅格 X 坐标；自动动作完成后由路线核心更新。
  input wire [7:0] now_y,  // 小车当前逻辑栅格 Y 坐标；自动动作完成后由路线核心更新。
  input wire [7:0] target_x,  // 编辑路线末端预测 X 坐标；添加或撤销动作时同步变化。
  input wire [7:0] target_y,  // 编辑路线末端预测 Y 坐标；添加或撤销动作时同步变化。
  input wire setting_mode,  // 手机参数设置模式；为 1 时参数按键优先并锁住路线编辑按键。
  input wire [2:0] selected_param,  // 当前手机增减操作选中的参数编号 0..6。
  input wire [15:0] step_length_set,  // 手机设置的单步长度参数。
  input wire [9:0] overall_speed,  // 手机可调的公共速度基准。
  input wire [9:0] lf_default_speed,  // 左前轮校准默认速度。
  input wire [9:0] rf_default_speed,  // 右前轮校准默认速度。
  input wire [9:0] lr_default_speed,  // 左后轮校准默认速度。
  input wire [9:0] rr_default_speed,  // 右后轮校准默认速度。
  input wire lidar_online,  // 收到有效雷达扫描样本且未超时。
  input wire lidar_obstacle_stop,  // 雷达按车体前方、前角和侧边轮廓判定产生的停车请求。
  input wire ultrasonic_obstacle_stop,  // 前向超声距离低于原有阈值时产生的停车请求。
  input wire merged_obstacle_stop,  // 雷达和超声停车请求按 OR 合并后的最终避障输入。
  input wire motor_blocked,  // 当前运动命令被避障逻辑拦截。
  input wire [15:0] ultrasonic_distance_cm,  // 最近超声波距离，单位 cm；FFFF 表示当前没有有效回波。
  input wire ultrasonic_distance_valid,  // 当前周期出现新的超声测距结果，可用于锁存或上报。
  input wire ultrasonic_echo_timeout,  // 最近一次超声等待/测量超过 30 ms 的超时状态。
  input wire [15:0] lidar_front_distance_mm,  // 三圈滤波后的正前方最近雷达距离，单位 mm；FFFF 表示无效。
  input wire [15:0] lidar_safe_mm,  // 手机可调雷达正前方安全距离，单位 mm。
  input wire [15:0] lidar_sector_front_mm,  // 雷达 前方 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  input wire [15:0] lidar_sector_front_right_mm,  // 雷达 前方 / 右 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  input wire [15:0] lidar_sector_right_mm,  // 雷达 右 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  input wire [15:0] lidar_sector_rear_right_mm,  // 雷达 后方 / 右 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  input wire [15:0] lidar_sector_rear_mm,  // 雷达 后方 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  input wire [15:0] lidar_sector_rear_left_mm,  // 雷达 后方 / 左 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  input wire [15:0] lidar_sector_left_mm,  // 雷达 左 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  input wire [15:0] lidar_sector_front_left_mm,  // 雷达 前方 / 左 45 度扇区的三圈中值滤波最近距离，单位 mm；FFFF 表示无效。
  input wire [15:0] c5_frame_count,  // C5 驱动已经完整发送的运动帧/停车帧累计数量。
  input wire [11:0] c5_last_lf_pulse,  // 最近一帧发送给该轮的 C5 脉宽诊断值。
  input wire [11:0] c5_last_rf_pulse,  // 最近一帧发送给该轮的 C5 脉宽诊断值。
  input wire [11:0] c5_last_lr_pulse,  // 最近一帧发送给该轮的 C5 脉宽诊断值。
  input wire [11:0] c5_last_rr_pulse,  // 最近一帧发送给该轮的 C5 脉宽诊断值。
  input wire c5_stop_frames_complete,  // 最近一次三帧强化停车序列是否已经全部发送完成。
  input wire [1:0] c5_stop_frames_remaining,  // 最近一次强化停车序列仍待发送的帧数，范围 0..3。
  input wire c5_frame_busy,  // C5 驱动当前是否占用 UART 发送一条完整控制帧。
  output wire bt_uart_tx,  // FPGA 到 CH9141K RXD 的透明串口发送线。
  output wire bt_connected,  // 仅在持续收到 P/p 心跳时为 1。
  output wire bt_key_up_n,  // 蓝牙方向字符模拟的低有效上键电平；字符到达后保持约 30 ms。
  output wire bt_key_down_n,  // 蓝牙方向字符模拟的低有效下键电平；字符到达后保持约 30 ms。
  output wire bt_key_left_n,  // 蓝牙方向字符模拟的低有效左键电平；字符到达后保持约 30 ms。
  output wire bt_key_right_n,  // 蓝牙方向字符模拟的低有效右键电平；字符到达后保持约 30 ms。
  output wire bt_key_mid_n,  // 蓝牙确认/停车字符模拟的低有效中键电平；字符到达后保持约 30 ms。
  output reg bt_cfg_mode_pulse,  // 手机切换参数设置模式的单周期事件脉冲。
  output reg bt_cfg_next_pulse,  // 手机选择下一项可调参数的单周期事件脉冲。
  output reg bt_cfg_prev_pulse,  // 手机选择上一项可调参数的单周期事件脉冲。
  output reg bt_cfg_inc_pulse,  // 手机把当前所选参数增加一个固定步长的单周期事件脉冲。
  output reg bt_cfg_dec_pulse,  // 手机把当前所选参数减少一个固定步长的单周期事件脉冲。
  output reg [2:0] bt_cfg_select,  // 手机直接指定的参数编号 0..6；仅在 bt_cfg_select_valid 脉冲周期被参数控制器采纳。
  output reg bt_cfg_select_valid,  // bt_cfg_select 携带新参数编号的单周期有效脉冲。
  output reg manual_mode,  // 1 表示手机手动模式覆盖自动路线。
  output reg [2:0] manual_cmd,  // 手动动作：0停、1前、2后、3左转、4右转。
  output reg lidar_enable,  // “激光雷达”功能使能；为 0 时该功能保持停止或安全空闲状态。
  output reg motor_stop_pulse  // 高优先级强制停车单周期脉冲。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
              // -----------------------------------------------------------------------------
              // 常量、状态编码与时序参数
              // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
              // -----------------------------------------------------------------------------
              localparam integer CLK_HZ = 50000000;  // 模块假定的系统时钟频率，单位 Hz；所有毫秒和波特率计数均由它换算。
              localparam integer BAUD = 115200;  // UART 目标波特率，单位 bit/s。
              localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;  // 每个 UART bit 占用的系统时钟周期数；整数除法会形成实际波特率误差。
              localparam integer KEY_HOLD_CYCLES = (CLK_HZ / 1000) * 30;  // 蓝牙字符模拟实体按键低电平约 30 ms 所需的系统时钟周期数。
              localparam integer STATUS_CYCLES = CLK_HZ / 5;  // 200 ms 主动状态上报周期对应的系统时钟周期数。
            localparam integer LINK_TIMEOUT_CYCLES = CLK_HZ * 3;  // 3 s 蓝牙心跳离线判定对应的系统时钟周期数。
            localparam integer MANUAL_TIMEOUT_CYCLES = (CLK_HZ / 10) * 3;  // 300 ms 手动运动看门狗对应的系统时钟周期数；超时后强制停车。
          localparam [6:0] ROUTE_STATUS_LEFT = 7'd85;  // @S 路线/参数状态帧的字符计数初值；发送器据此覆盖协议规定的完整帧长度。
            localparam [6:0] LIDAR_STATUS_LEFT = 7'd55;  // @L 八扇区雷达状态帧的字符计数初值；发送器据此覆盖协议规定的完整帧长度。
              localparam [2:0] REPLY_ACK_LAST = 3'd3;  // 蓝牙短回复类型或结束位置“REPLY / ACK / LAST”的编码。
              localparam [2:0] REPLY_PONG_LAST = 3'd5;  // 蓝牙短回复类型或结束位置“REPLY / PONG / LAST”的编码。
              localparam [1:0] REPLY_NONE = 2'd0;  // 蓝牙短回复类型或结束位置“REPLY / NONE”的编码。
              localparam [1:0] REPLY_KEY  = 2'd1;  // 蓝牙短回复类型或结束位置“REPLY / KEY”的编码。
              localparam [1:0] REPLY_PONG = 2'd2;  // 蓝牙短回复类型或结束位置“REPLY / PONG”的编码。

              localparam [1:0] RX_IDLE  = 2'd0;  // UART 接收状态“RX / IDLE”的编码。
              localparam [1:0] RX_START = 2'd1;  // UART 接收状态“RX / START”的编码。
              localparam [1:0] RX_DATA  = 2'd2;  // UART 接收状态“RX / DATA”的编码。
              localparam [1:0] RX_STOP  = 2'd3;  // UART 接收状态“RX / STOP”的编码。
              localparam [1:0] TX_IDLE  = 2'd0;  // UART 发送状态“TX / IDLE”的编码。
              localparam [1:0] TX_START = 2'd1;  // UART 发送状态“TX / START”的编码。
              localparam [1:0] TX_DATA  = 2'd2;  // UART 发送状态“TX / DATA”的编码。
            localparam [1:0] TX_STOP  = 2'd3;  // UART 发送状态“TX / STOP”的编码。
            localparam [2:0] MANUAL_STOP    = 3'd0;  // 手机手动动作“MANUAL / STOP”的 3 bit 协议/内部统一编码。
            localparam [2:0] MANUAL_FORWARD = 3'd1;  // 手机手动动作“MANUAL / FORWARD”的 3 bit 协议/内部统一编码。
            localparam [2:0] MANUAL_REVERSE = 3'd2;  // 手机手动动作“MANUAL / REVERSE”的 3 bit 协议/内部统一编码。
            localparam [2:0] MANUAL_LEFT    = 3'd3;  // 手机手动动作“MANUAL / LEFT”的 3 bit 协议/内部统一编码。
            localparam [2:0] MANUAL_RIGHT   = 3'd4;  // 手机手动动作“MANUAL / RIGHT”的 3 bit 协议/内部统一编码。

              // -----------------------------------------------------------------------------
              // 内部网络、寄存器和计数器
              // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
              // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
              // -----------------------------------------------------------------------------
              reg [1:0] rx_sync;  // 两级同步后的异步蓝牙 RX，降低亚稳态风险。
              reg [1:0] rx_state;  // UART 接收状态，依次经历空闲、起始位确认、8 位数据采样和停止位检查。
              reg [15:0] rx_clk_count;  // UART 接收当前 bit 内的系统时钟计数；计满 CLKS_PER_BIT 后采样下一位。
              reg [2:0] rx_bit_index;  // UART 正在接收的数据位编号 0..7，数据按 LSB-first 顺序到达。
              reg [7:0] rx_shift;  // UART 接收移位缓存；8 个数据位收齐后整体复制到 rx_data。
              reg [7:0] rx_data;  // 最近完整接收的 ASCII 命令字节。
              reg rx_valid;  // 仅在一个 UART 字节通过停止位检查后脉冲一个周期。

              reg [21:0] up_hold_count;  // 把蓝牙前进字符模拟为实体上键低电平的剩余保持周期数，约 30 ms。
              reg [21:0] down_hold_count;  // 把蓝牙后退/撤销字符模拟为实体下键低电平的剩余保持周期数，约 30 ms。
              reg [21:0] left_hold_count;  // 把蓝牙左转字符模拟为实体左键低电平的剩余保持周期数，约 30 ms。
              reg [21:0] right_hold_count;  // 把蓝牙右转字符模拟为实体右键低电平的剩余保持周期数，约 30 ms。
              reg [21:0] mid_hold_count;  // 把蓝牙确认/停车字符模拟为实体中键低电平的剩余保持周期数，约 30 ms。

              reg [31:0] status_count;  // 200 ms 周期状态上报计数器。
            reg [6:0] status_left;  // 当前 @S/@L 长帧尚待调度的字符数量；递减到零后释放 sending_status 互斥锁。
            reg [6:0] status_index;  // @S/@L 当前发送字符位置。
              reg active_status_lidar;  // 当前长帧类型锁存：0 发送 @S 路线/参数状态，1 发送 @L 八扇区雷达状态。
              reg [7:0] tx_data;  // 等待交给 UART 位发送状态机的下一个字节。
              reg tx_start;  // 启动一个 UART 字节发送的单周期脉冲。
              reg tx_line;  // UART 发送引脚的内部逻辑电平；空闲和停止位为 1。
              reg tx_busy;  // UART 正在发送一个字节；为 1 时上层不得覆盖 tx_data。
              reg tx_busy_d;  // tx_busy 的上一周期值，与当前值组合检测字节发送完成下降沿。
              reg [1:0] tx_state;  // 唯一蓝牙 UART 发送器的 IDLE/START/DATA/STOP 状态。
              reg [15:0] tx_clk_count;  // UART 当前发送 bit 已保持的系统时钟周期数。
              reg [2:0] tx_bit_index;  // UART 正在发送的数据位编号 0..7，发送顺序为 LSB-first。
              reg [7:0] tx_shift;  // UART 开始发送时锁存的数据字节，避免发送中途被新输入改变。
              reg sending_status;  // 长状态帧发送互斥锁；防止 @PONG 或 @K 插入其中。
              reg sending_reply;  // 短回复发送互斥锁。
              reg status_packet_lidar;  // 周期上报轮换选择：0 表示下一周期优先 @S，1 表示下一周期优先 @L。
              reg pending_reply;  // 已经收到命令但尚未开始发送 @K/@PONG。
              reg [1:0] reply_kind;  // 最新接收命令产生的短回复类型：按键确认 @Kx、心跳 @PONG 或无回复。
              reg [1:0] active_reply_kind;  // 短回复开始发送时锁存的回复类型，保证发送途中不被后续命令覆盖。
              reg [2:0] reply_index;  // 当前 @K/@PONG 短回复正在发送的字符下标。
              reg [7:0] reply_key;  // 待回复的归一化按键字符；用于形成 @K<key> 换行帧。
              reg [7:0] active_reply_key;  // 当前 @K 帧锁存的按键字符，直到整帧发完才允许更新。
            reg [31:0] link_idle_count;  // 距离最近 P/p 心跳经过的周期数。
            reg link_active;  // 心跳在线锁存状态，对外成为 bt_connected。
          reg [23:0] manual_timeout_count;  // 手动运动命令 300 ms 看门狗计数。
          reg manual_watchdog_expired;  // 看门狗曾把运动命令强制改为停止。
            reg route_status_pending;  // 要求尽快发送一帧最新 @S 的挂起标志。
            reg [7:0] route_status_seq;  // 每次锁存 @S 时自增的 8 bit 序号。
            reg [7:0] lidar_status_seq;  // 每次锁存 @L 时自增的 8 bit 序号。
            reg [7:0] active_status_seq;  // 当前正在发送帧的序号快照。
            reg [1:0] prev_menu_info;  // 上一周期锁存的“菜单 / 信息”；与当前输入比较以触发状态变化上报。
            reg [4:0] prev_state_info;  // 上一周期锁存的“状态 / 信息”；与当前输入比较以触发状态变化上报。
            reg [63:0] prev_route_info;  // 上一周期锁存的“路线 / 信息”；与当前输入比较以触发状态变化上报。
            reg [5:0] prev_active_route_steps;  // 上一周期锁存的“当前有效 / 路线 / 步数”；与当前输入比较以触发状态变化上报。
            reg [5:0] prev_completed_route_steps;  // 上一周期锁存的“已完成 / 路线 / 步数”；与当前输入比较以触发状态变化上报。
            reg [7:0] prev_now_x;  // 上一周期锁存的“当前位置 / X 坐标”；与当前输入比较以触发状态变化上报。
            reg [7:0] prev_now_y;  // 上一周期锁存的“当前位置 / Y 坐标”；与当前输入比较以触发状态变化上报。
          reg [7:0] prev_target_x;  // 上一周期锁存的“目标 / X 坐标”；与当前输入比较以触发状态变化上报。
          reg [7:0] prev_target_y;  // 上一周期锁存的“目标 / Y 坐标”；与当前输入比较以触发状态变化上报。
          reg [15:0] prev_step_length_set;  // 上一周期锁存的“步骤 / 长度 / 设定值”；与当前输入比较以触发状态变化上报。
          reg [9:0] prev_overall_speed;  // 上一周期锁存的“整体 / 速度”；与当前输入比较以触发状态变化上报。
          reg [9:0] prev_lf_default_speed;  // 上一周期锁存的“左前轮 / 默认值 / 速度”；与当前输入比较以触发状态变化上报。
          reg [9:0] prev_rf_default_speed;  // 上一周期锁存的“右前轮 / 默认值 / 速度”；与当前输入比较以触发状态变化上报。
          reg [9:0] prev_lr_default_speed;  // 上一周期锁存的“左后轮 / 默认值 / 速度”；与当前输入比较以触发状态变化上报。
          reg [9:0] prev_rr_default_speed;  // 上一周期锁存的“右后轮 / 默认值 / 速度”；与当前输入比较以触发状态变化上报。
          reg [15:0] prev_lidar_safe_mm;  // 上一周期锁存的“激光雷达 / 安全阈值 / MM”；与当前输入比较以触发状态变化上报。
          reg prev_setting_mode;  // 上一周期锁存的“参数设置 / 模式”；与当前输入比较以触发状态变化上报。
          reg [2:0] prev_selected_param;  // 上一周期锁存的“已选择 / 参数”；与当前输入比较以触发状态变化上报。
          reg prev_manual_mode;  // 上一周期锁存的“手动控制 / 模式”；与当前输入比较以触发状态变化上报。
          reg [2:0] prev_manual_cmd;  // 上一周期锁存的“手动控制 / 命令”；与当前输入比较以触发状态变化上报。
          reg prev_manual_watchdog_expired;  // 上一周期锁存的“手动控制 / 看门狗 / 已超时”；与当前输入比较以触发状态变化上报。
          reg prev_lidar_enable;  // 上一周期锁存的“激光雷达 / 使能”；与当前输入比较以触发状态变化上报。
          reg prev_motor_blocked;  // 上一周期锁存的“电机 / 受阻状态”；与当前输入比较以触发状态变化上报。
            reg [1:0] status_menu_info;  // 开始发送状态帧时锁存的“菜单 / 信息”快照；保证长帧发送期间字段前后一致。
            reg [4:0] status_state_info;  // 开始发送状态帧时锁存的“状态 / 信息”快照；保证长帧发送期间字段前后一致。
            reg [63:0] status_route_info;  // 开始发送 @S 时锁存的路线快照，保证整帧字段一致。
            reg [5:0] status_active_route_steps;  // 开始发送状态帧时锁存的“当前有效 / 路线 / 步数”快照；保证长帧发送期间字段前后一致。
            reg [5:0] status_completed_route_steps;  // 开始发送状态帧时锁存的“已完成 / 路线 / 步数”快照；保证长帧发送期间字段前后一致。
            reg [7:0] status_now_x;  // 开始发送状态帧时锁存的“当前位置 / X 坐标”快照；保证长帧发送期间字段前后一致。
            reg [7:0] status_now_y;  // 开始发送状态帧时锁存的“当前位置 / Y 坐标”快照；保证长帧发送期间字段前后一致。
            reg [7:0] status_target_x;  // 开始发送状态帧时锁存的“目标 / X 坐标”快照；保证长帧发送期间字段前后一致。
            reg [7:0] status_target_y;  // 开始发送状态帧时锁存的“目标 / Y 坐标”快照；保证长帧发送期间字段前后一致。
            reg [15:0] status_step_length_set;  // 开始发送状态帧时锁存的“步骤 / 长度 / 设定值”快照；保证长帧发送期间字段前后一致。
          reg [9:0] status_overall_speed;  // 开始发送状态帧时锁存的“整体 / 速度”快照；保证长帧发送期间字段前后一致。
          reg [9:0] status_lf_default_speed;  // 开始发送状态帧时锁存的“左前轮 / 默认值 / 速度”快照；保证长帧发送期间字段前后一致。
          reg [9:0] status_rf_default_speed;  // 开始发送状态帧时锁存的“右前轮 / 默认值 / 速度”快照；保证长帧发送期间字段前后一致。
          reg [9:0] status_lr_default_speed;  // 开始发送状态帧时锁存的“左后轮 / 默认值 / 速度”快照；保证长帧发送期间字段前后一致。
          reg [9:0] status_rr_default_speed;  // 开始发送状态帧时锁存的“右后轮 / 默认值 / 速度”快照；保证长帧发送期间字段前后一致。
          reg [15:0] status_lidar_safe_mm_route;  // 开始发送状态帧时锁存的“激光雷达 / 安全阈值 / MM / 路线”快照；保证长帧发送期间字段前后一致。
          reg status_setting_mode;  // 开始发送状态帧时锁存的“参数设置 / 模式”快照；保证长帧发送期间字段前后一致。
            reg [2:0] status_selected_param;  // 开始发送状态帧时锁存的“已选择 / 参数”快照；保证长帧发送期间字段前后一致。
            reg status_lidar_online;  // 开始发送状态帧时锁存的“激光雷达 / 在线状态”快照；保证长帧发送期间字段前后一致。
            reg status_manual_mode;  // 开始发送状态帧时锁存的“手动控制 / 模式”快照；保证长帧发送期间字段前后一致。
            reg status_lidar_enable;  // 开始发送状态帧时锁存的“激光雷达 / 使能”快照；保证长帧发送期间字段前后一致。
          reg status_lidar_obstacle_stop;  // 开始发送状态帧时锁存的“激光雷达 / 障碍 / 停车”快照；保证长帧发送期间字段前后一致。
          reg status_ultrasonic_obstacle_stop;  // 开始发送状态帧时锁存的“超声波 / 障碍 / 停车”快照；保证长帧发送期间字段前后一致。
          reg status_merged_obstacle_stop;  // 开始发送状态帧时锁存的“合并结果 / 障碍 / 停车”快照；保证长帧发送期间字段前后一致。
          reg status_motor_blocked;  // 开始发送状态帧时锁存的“电机 / 受阻状态”快照；保证长帧发送期间字段前后一致。
          reg status_ultrasonic_distance_valid;  // 开始发送状态帧时锁存的“超声波 / 距离 / 有效标志”快照；保证长帧发送期间字段前后一致。
          reg status_ultrasonic_echo_timeout;  // 开始发送状态帧时锁存的“超声波 / 回波 / 超时”快照；保证长帧发送期间字段前后一致。
          reg [15:0] status_ultrasonic_distance_cm;  // 开始发送状态帧时锁存的“超声波 / 距离 / CM”快照；保证长帧发送期间字段前后一致。
          reg [2:0] status_manual_cmd;  // 开始发送状态帧时锁存的“手动控制 / 命令”快照；保证长帧发送期间字段前后一致。
          reg status_manual_watchdog_expired;  // 开始发送状态帧时锁存的“手动控制 / 看门狗 / 已超时”快照；保证长帧发送期间字段前后一致。
          reg [15:0] status_c5_frame_count;  // 开始发送状态帧时锁存的“C5 / 帧 / 计数”快照；保证长帧发送期间字段前后一致。
          reg [11:0] status_c5_last_lf_pulse;  // 开始发送状态帧时锁存的“C5 / 最近一次 / 左前轮 / 脉冲”快照；保证长帧发送期间字段前后一致。
          reg [11:0] status_c5_last_rf_pulse;  // 开始发送状态帧时锁存的“C5 / 最近一次 / 右前轮 / 脉冲”快照；保证长帧发送期间字段前后一致。
          reg [11:0] status_c5_last_lr_pulse;  // 开始发送状态帧时锁存的“C5 / 最近一次 / 左后轮 / 脉冲”快照；保证长帧发送期间字段前后一致。
          reg [11:0] status_c5_last_rr_pulse;  // 开始发送状态帧时锁存的“C5 / 最近一次 / 右后轮 / 脉冲”快照；保证长帧发送期间字段前后一致。
          reg status_c5_stop_frames_complete;  // 开始发送状态帧时锁存的“C5 / 停车 / 帧数 / 完成状态”快照；保证长帧发送期间字段前后一致。
          reg [1:0] status_c5_stop_frames_remaining;  // 开始发送状态帧时锁存的“C5 / 停车 / 帧数 / 剩余”快照；保证长帧发送期间字段前后一致。
          reg status_c5_frame_busy;  // 开始发送状态帧时锁存的“C5 / 帧 / 忙标志”快照；保证长帧发送期间字段前后一致。
          reg [15:0] ultrasonic_last_distance_cm;  // 最近一次有效超声测距结果，单位 cm；用于跨越 distance_valid 单周期脉冲持续上报。
          reg ultrasonic_last_valid;  // 已经至少锁存过一次有效超声测量的标志；复位前保持有效。
          reg ultrasonic_last_timeout;  // 最近一次超声测量是否超时；与 ultrasonic_last_distance_cm 一起进入 @S 状态帧。
          reg ultrasonic_last_stop;  // 最近一次超声测量对应的停车判定锁存值；用于 @S 上报障碍来源。
            reg [15:0] status_lidar_safe_mm;  // 开始发送状态帧时锁存的“激光雷达 / 安全阈值 / MM”快照；保证长帧发送期间字段前后一致。
            reg [15:0] status_lidar_front_distance_mm;  // 开始发送状态帧时锁存的“激光雷达 / 前方 / 距离 / MM”快照；保证长帧发送期间字段前后一致。
            reg [15:0] status_lidar_sector_mm [0:7];  // 开始发送 @L 时锁存的八方向距离快照。

              wire status_due;  // 200 ms 周期计数已经到期的组合条件；仅提出上报请求，不直接抢占正在发送的帧。
              wire tx_done;  // 底层 UART busy 由 1 变 0 的单周期完成脉冲；TX 调度器据此推进字符索引。
              wire reply_done;  // 当前短回复最后一个字符已经发送完成的组合条件。
              wire [7:0] norm_key;  // 把同义方向字符归一化后的 U/D/L/R/M 控制字符；无效输入归一化为零。
              wire is_cfg_command;  // 当前接收字节是否属于参数选择、增减、模式或雷达开关命令。
              wire route_status_changed;  // 路线、位置、目标、状态、模式或可调参数相对上一快照发生变化的组合标志。

              // -----------------------------------------------------------------------------
              // 组合派生信号
              // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
              // -----------------------------------------------------------------------------
              // [ASSIGN] 组合定义“bt_uart_tx”：FPGA 到 CH9141K RXD 的透明串口发送线。 该组合结果直接由 `tx_line`（发送 / 输出线）的当前值决定，不引入时钟周期延迟。
              assign bt_uart_tx = tx_line;
              // [ASSIGN] 组合定义“bt_connected”：仅在持续收到 P/p 心跳时为 1。 该组合结果直接由 `link_active`（link / 当前有效）的当前值决定，不引入时钟周期延迟。
              assign bt_connected = link_active;
              // [ASSIGN] 组合定义“bt_key_up_n”：蓝牙方向字符模拟的低有效上键电平；字符到达后保持约 30 ms。 该组合结果直接由 `up_hold_count`（上键/前进 / 保持时间 / 计数）的当前值决定，不引入时钟周期延迟。
              assign bt_key_up_n = (up_hold_count == 22'd0);
              // [ASSIGN] 组合定义“bt_key_down_n”：蓝牙方向字符模拟的低有效下键电平；字符到达后保持约 30 ms。 该组合结果直接由 `down_hold_count`（下键/后退 / 保持时间 / 计数）的当前值决定，不引入时钟周期延迟。
              assign bt_key_down_n = (down_hold_count == 22'd0);
              // [ASSIGN] 组合定义“bt_key_left_n”：蓝牙方向字符模拟的低有效左键电平；字符到达后保持约 30 ms。 该组合结果直接由 `left_hold_count`（左 / 保持时间 / 计数）的当前值决定，不引入时钟周期延迟。
              assign bt_key_left_n = (left_hold_count == 22'd0);
              // [ASSIGN] 组合定义“bt_key_right_n”：蓝牙方向字符模拟的低有效右键电平；字符到达后保持约 30 ms。 该组合结果直接由 `right_hold_count`（右 / 保持时间 / 计数）的当前值决定，不引入时钟周期延迟。
              assign bt_key_right_n = (right_hold_count == 22'd0);
              // [ASSIGN] 组合定义“bt_key_mid_n”：蓝牙确认/停车字符模拟的低有效中键电平；字符到达后保持约 30 ms。 该组合结果直接由 `mid_hold_count`（中键/确认 / 保持时间 / 计数）的当前值决定，不引入时钟周期延迟。
              assign bt_key_mid_n = (mid_hold_count == 22'd0);
              // [ASSIGN] 组合定义“status_due”：200 ms 周期计数已经到期的组合条件；仅提出上报请求，不直接抢占正在发送的帧。 该组合结果直接由 `status_count`（状态帧快照 / 计数）、`STATUS_CYCLES`（STATUS / CYCLES）的当前值决定，不引入时钟周期延迟。
              assign status_due = (status_count >= STATUS_CYCLES - 1);
              // [ASSIGN] 组合定义“tx_done”：底层 UART busy 由 1 变 0 的单周期完成脉冲；TX 调度器据此推进字符索引。 该组合结果直接由 `tx_busy_d`（发送 / 忙标志 / D）、`tx_busy`（发送 / 忙标志）的当前值决定，不引入时钟周期延迟。
              assign tx_done = tx_busy_d && !tx_busy;
              // [ASSIGN] 组合定义“reply_done”：当前短回复最后一个字符已经发送完成的组合条件。 该组合结果直接由 `active_reply_kind`（当前有效 / 短回复 / 类型）、`REPLY_KEY`（REPLY / KEY）、`reply_index`（短回复 / 索引）、`REPLY_ACK_LAST`（REPLY / ACK / LAST）、`REPLY_PONG_LAST`（REPLY / PONG / LAST）的当前值决定，不引入时钟周期延迟。
              assign reply_done = (active_reply_kind == REPLY_KEY) ?
                                  (reply_index == REPLY_ACK_LAST) :
                                  (reply_index == REPLY_PONG_LAST);
              // [ASSIGN] 组合定义“norm_key”：把同义方向字符归一化后的 U/D/L/R/M 控制字符；无效输入归一化为零。 该组合结果直接由 `normalized_key`（normalized / KEY）、`rx_data`（接收 / 数据）的当前值决定，不引入时钟周期延迟。
              assign norm_key = normalized_key(rx_data);
              // [ASSIGN] 组合定义“is_cfg_command”：当前接收字节是否属于参数选择、增减、模式或雷达开关命令。 该组合结果直接由 `cfg_command`（配置 / 命令）、`rx_data`（接收 / 数据）的当前值决定，不引入时钟周期延迟。
              assign is_cfg_command = cfg_command(rx_data);
              // [ASSIGN] 组合定义“route_status_changed”：路线、位置、目标、状态、模式或可调参数相对上一快照发生变化的组合标志。 该组合结果直接由 `menu_info`（菜单 / 信息）、`prev_menu_info`（上一周期 / 菜单 / 信息）、`state_info`（状态 / 信息）、`prev_state_info`（上一周期 / 状态 / 信息）、`route_info`（路线 / 信息）、`prev_route_info`（上一周期 / 路线 / 信息）等当前值决定，不引入时钟周期延迟。
              assign route_status_changed =
                  (menu_info != prev_menu_info) ||
                  (state_info != prev_state_info) ||
                  (route_info != prev_route_info) ||
                  (active_route_steps != prev_active_route_steps) ||
                (completed_route_steps != prev_completed_route_steps) ||
                (now_x != prev_now_x) || (now_y != prev_now_y) ||
                (target_x != prev_target_x) || (target_y != prev_target_y) ||
                (step_length_set != prev_step_length_set) ||
                (overall_speed != prev_overall_speed) ||
                (lf_default_speed != prev_lf_default_speed) ||
                (rf_default_speed != prev_rf_default_speed) ||
                (lr_default_speed != prev_lr_default_speed) ||
                (rr_default_speed != prev_rr_default_speed) ||
                (lidar_safe_mm != prev_lidar_safe_mm) ||
                (setting_mode != prev_setting_mode) ||
                (selected_param != prev_selected_param) ||
                (manual_mode != prev_manual_mode) ||
                (manual_cmd != prev_manual_cmd) ||
                (manual_watchdog_expired != prev_manual_watchdog_expired) ||
                (lidar_enable != prev_lidar_enable) ||
                (motor_blocked != prev_motor_blocked);

              // -----------------------------------------------------------------------------
              // [FUNCTION] 辅助函数: hex_char
              // 把 4 bit 半字节转换为大写 ASCII 十六进制字符 0..9/A..F。
              // -----------------------------------------------------------------------------
              function [7:0] hex_char;
                input [3:0] value;  // 4 bit 函数 `hex_char` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
                begin
                  hex_char = value + ((value < 4'd10) ? 8'd48 : 8'd55);
                end
              endfunction

              // -----------------------------------------------------------------------------
              // [FUNCTION] 辅助函数: normalized_key
              // 把 W/U、S/D、A/L、R、C/M 同义字符归一化为 U/D/L/R/M。
              // -----------------------------------------------------------------------------
              function [7:0] normalized_key;
                input [7:0] value;  // 8 bit 函数 `normalized_key` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
                begin
                  case (value)
                    "U", "u", "W", "w": normalized_key = "U";
                    "D", "d", "S", "s": normalized_key = "D";
                    "L", "l", "A", "a": normalized_key = "L";
                    "R", "r":           normalized_key = "R";
                    "M", "m", "C", "c": normalized_key = "M";
                    default:             normalized_key = 8'd0;
                  endcase
                end
              endfunction

            // -----------------------------------------------------------------------------
            // [FUNCTION] 辅助函数: status_char
            // 按字符索引编码 @S 或 @L；这是手机协议字段定义的唯一权威位置。
            // -----------------------------------------------------------------------------
            function [7:0] status_char;
              input lidar_packet;  // 函数 `status_char` 的“激光雷达 / 数据包”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
              input [6:0] char_index;  // 7 bit 函数 `status_char` 的“字符 / 索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
                begin
                  if (lidar_packet) begin
                    case (char_index)
                      7'd0: status_char = "L";
                      7'd1: status_char = hex_char({3'd0, status_lidar_online});
                      7'd2: status_char = hex_char({3'd0, status_lidar_obstacle_stop});
                      7'd3: status_char = hex_char({3'd0, status_ultrasonic_obstacle_stop});
                      7'd4: status_char = hex_char({3'd0, status_merged_obstacle_stop});
                      7'd5: status_char = hex_char({3'd0, status_motor_blocked});
                      7'd6: status_char = hex_char({3'd0, status_ultrasonic_distance_valid});
                      7'd7: status_char = hex_char({3'd0, status_ultrasonic_echo_timeout});
                      7'd8: status_char = hex_char(status_lidar_safe_mm[15:12]);
                      7'd9: status_char = hex_char(status_lidar_safe_mm[11:8]);
                      7'd10: status_char = hex_char(status_lidar_safe_mm[7:4]);
                      7'd11: status_char = hex_char(status_lidar_safe_mm[3:0]);
                      7'd12: status_char = hex_char(status_ultrasonic_distance_cm[15:12]);
                      7'd13: status_char = hex_char(status_ultrasonic_distance_cm[11:8]);
                      7'd14: status_char = hex_char(status_ultrasonic_distance_cm[7:4]);
                      7'd15: status_char = hex_char(status_ultrasonic_distance_cm[3:0]);
                      7'd16: status_char = hex_char(status_lidar_front_distance_mm[15:12]);
                      7'd17: status_char = hex_char(status_lidar_front_distance_mm[11:8]);
                      7'd18: status_char = hex_char(status_lidar_front_distance_mm[7:4]);
                      7'd19: status_char = hex_char(status_lidar_front_distance_mm[3:0]);
                      7'd20: status_char = hex_char(status_lidar_sector_mm[0][15:12]);
                      7'd21: status_char = hex_char(status_lidar_sector_mm[0][11:8]);
                      7'd22: status_char = hex_char(status_lidar_sector_mm[0][7:4]);
                      7'd23: status_char = hex_char(status_lidar_sector_mm[0][3:0]);
                      7'd24: status_char = hex_char(status_lidar_sector_mm[1][15:12]);
                      7'd25: status_char = hex_char(status_lidar_sector_mm[1][11:8]);
                      7'd26: status_char = hex_char(status_lidar_sector_mm[1][7:4]);
                      7'd27: status_char = hex_char(status_lidar_sector_mm[1][3:0]);
                      7'd28: status_char = hex_char(status_lidar_sector_mm[2][15:12]);
                      7'd29: status_char = hex_char(status_lidar_sector_mm[2][11:8]);
                      7'd30: status_char = hex_char(status_lidar_sector_mm[2][7:4]);
                      7'd31: status_char = hex_char(status_lidar_sector_mm[2][3:0]);
                      7'd32: status_char = hex_char(status_lidar_sector_mm[3][15:12]);
                      7'd33: status_char = hex_char(status_lidar_sector_mm[3][11:8]);
                      7'd34: status_char = hex_char(status_lidar_sector_mm[3][7:4]);
                      7'd35: status_char = hex_char(status_lidar_sector_mm[3][3:0]);
                      7'd36: status_char = hex_char(status_lidar_sector_mm[4][15:12]);
                      7'd37: status_char = hex_char(status_lidar_sector_mm[4][11:8]);
                      7'd38: status_char = hex_char(status_lidar_sector_mm[4][7:4]);
                      7'd39: status_char = hex_char(status_lidar_sector_mm[4][3:0]);
                      7'd40: status_char = hex_char(status_lidar_sector_mm[5][15:12]);
                      7'd41: status_char = hex_char(status_lidar_sector_mm[5][11:8]);
                      7'd42: status_char = hex_char(status_lidar_sector_mm[5][7:4]);
                      7'd43: status_char = hex_char(status_lidar_sector_mm[5][3:0]);
                      7'd44: status_char = hex_char(status_lidar_sector_mm[6][15:12]);
                      7'd45: status_char = hex_char(status_lidar_sector_mm[6][11:8]);
                      7'd46: status_char = hex_char(status_lidar_sector_mm[6][7:4]);
                      7'd47: status_char = hex_char(status_lidar_sector_mm[6][3:0]);
                      7'd48: status_char = hex_char(status_lidar_sector_mm[7][15:12]);
                      7'd49: status_char = hex_char(status_lidar_sector_mm[7][11:8]);
                      7'd50: status_char = hex_char(status_lidar_sector_mm[7][7:4]);
                      7'd51: status_char = hex_char(status_lidar_sector_mm[7][3:0]);
                      7'd52: status_char = hex_char(active_status_seq[7:4]);
                      7'd53: status_char = hex_char(active_status_seq[3:0]);
                      default: status_char = 8'd10;
                    endcase
                  end else begin
                    case (char_index)
                      6'd0: status_char = "S";
                      6'd1: status_char = hex_char({3'd0, status_state_info[4]});
                      6'd2: status_char = hex_char(status_state_info[3:0]);
                      6'd3: status_char = hex_char({2'd0, status_menu_info});
                      6'd4: status_char = hex_char({2'd0, status_active_route_steps[5:4]});
                      6'd5: status_char = hex_char(status_active_route_steps[3:0]);
                      6'd6: status_char = hex_char({2'd0, status_completed_route_steps[5:4]});
                      6'd7: status_char = hex_char(status_completed_route_steps[3:0]);
                      6'd8: status_char = hex_char(status_now_x[7:4]);
                      6'd9: status_char = hex_char(status_now_x[3:0]);
                      6'd10: status_char = hex_char(status_now_y[7:4]);
                      6'd11: status_char = hex_char(status_now_y[3:0]);
                      6'd12: status_char = hex_char(status_target_x[7:4]);
                      6'd13: status_char = hex_char(status_target_x[3:0]);
                      6'd14: status_char = hex_char(status_target_y[7:4]);
                      6'd15: status_char = hex_char(status_target_y[3:0]);
                      6'd16: status_char = hex_char(status_route_info[63:60]);
                      6'd17: status_char = hex_char(status_route_info[59:56]);
                      6'd18: status_char = hex_char(status_route_info[55:52]);
                      6'd19: status_char = hex_char(status_route_info[51:48]);
                      6'd20: status_char = hex_char(status_route_info[47:44]);
                      6'd21: status_char = hex_char(status_route_info[43:40]);
                      6'd22: status_char = hex_char(status_route_info[39:36]);
                      6'd23: status_char = hex_char(status_route_info[35:32]);
                      6'd24: status_char = hex_char(status_route_info[31:28]);
                      6'd25: status_char = hex_char(status_route_info[27:24]);
                      6'd26: status_char = hex_char(status_route_info[23:20]);
                      6'd27: status_char = hex_char(status_route_info[19:16]);
                      6'd28: status_char = hex_char(status_route_info[15:12]);
                      6'd29: status_char = hex_char(status_route_info[11:8]);
                      6'd30: status_char = hex_char(status_route_info[7:4]);
                      6'd31: status_char = hex_char(status_route_info[3:0]);
                      6'd32: status_char = hex_char(status_step_length_set[15:12]);
                      6'd33: status_char = hex_char(status_step_length_set[11:8]);
                      6'd34: status_char = hex_char(status_step_length_set[7:4]);
                      6'd35: status_char = hex_char(status_step_length_set[3:0]);
                      6'd36: status_char = hex_char({2'd0, status_overall_speed[9:8]});
                      6'd37: status_char = hex_char(status_overall_speed[7:4]);
                      6'd38: status_char = hex_char(status_overall_speed[3:0]);
                      7'd39: status_char = hex_char({2'd0, status_lf_default_speed[9:8]});
                      7'd40: status_char = hex_char(status_lf_default_speed[7:4]);
                      7'd41: status_char = hex_char(status_lf_default_speed[3:0]);
                      7'd42: status_char = hex_char({2'd0, status_rf_default_speed[9:8]});
                      7'd43: status_char = hex_char(status_rf_default_speed[7:4]);
                      7'd44: status_char = hex_char(status_rf_default_speed[3:0]);
                      7'd45: status_char = hex_char({2'd0, status_lr_default_speed[9:8]});
                      7'd46: status_char = hex_char(status_lr_default_speed[7:4]);
                      7'd47: status_char = hex_char(status_lr_default_speed[3:0]);
                      7'd48: status_char = hex_char({2'd0, status_rr_default_speed[9:8]});
                      7'd49: status_char = hex_char(status_rr_default_speed[7:4]);
                      7'd50: status_char = hex_char(status_rr_default_speed[3:0]);
                      7'd51: status_char = hex_char(status_lidar_safe_mm_route[15:12]);
                      7'd52: status_char = hex_char(status_lidar_safe_mm_route[11:8]);
                      7'd53: status_char = hex_char(status_lidar_safe_mm_route[7:4]);
                      7'd54: status_char = hex_char(status_lidar_safe_mm_route[3:0]);
                      7'd55: status_char = hex_char({3'd0, status_setting_mode});
                      7'd56: status_char = hex_char({1'd0, status_selected_param});
                      7'd57: status_char = hex_char({3'd0, status_lidar_online});
                      7'd58: status_char = hex_char({3'd0, status_manual_mode});
                      7'd59: status_char = hex_char({3'd0, status_lidar_enable});
                      7'd60: status_char = hex_char({1'd0, status_manual_cmd});
                      7'd61: status_char = hex_char({3'd0, status_manual_watchdog_expired});
                      7'd62: status_char = hex_char({3'd0, status_motor_blocked});
                      7'd63: status_char = hex_char(status_c5_frame_count[15:12]);
                      7'd64: status_char = hex_char(status_c5_frame_count[11:8]);
                      7'd65: status_char = hex_char(status_c5_frame_count[7:4]);
                      7'd66: status_char = hex_char(status_c5_frame_count[3:0]);
                      7'd67: status_char = hex_char(status_c5_last_lf_pulse[11:8]);
                      7'd68: status_char = hex_char(status_c5_last_lf_pulse[7:4]);
                      7'd69: status_char = hex_char(status_c5_last_lf_pulse[3:0]);
                      7'd70: status_char = hex_char(status_c5_last_rf_pulse[11:8]);
                      7'd71: status_char = hex_char(status_c5_last_rf_pulse[7:4]);
                      7'd72: status_char = hex_char(status_c5_last_rf_pulse[3:0]);
                      7'd73: status_char = hex_char(status_c5_last_lr_pulse[11:8]);
                      7'd74: status_char = hex_char(status_c5_last_lr_pulse[7:4]);
                      7'd75: status_char = hex_char(status_c5_last_lr_pulse[3:0]);
                      7'd76: status_char = hex_char(status_c5_last_rr_pulse[11:8]);
                      7'd77: status_char = hex_char(status_c5_last_rr_pulse[7:4]);
                      7'd78: status_char = hex_char(status_c5_last_rr_pulse[3:0]);
                      7'd79: status_char = hex_char({3'd0, status_c5_stop_frames_complete});
                      7'd80: status_char = hex_char({2'd0, status_c5_stop_frames_remaining});
                      7'd81: status_char = hex_char({3'd0, status_c5_frame_busy});
                      7'd82: status_char = hex_char(active_status_seq[7:4]);
                      7'd83: status_char = hex_char(active_status_seq[3:0]);
                    default: status_char = 8'd10;
                    endcase
                  end
                end
              endfunction

              // -----------------------------------------------------------------------------
              // [FUNCTION] 辅助函数: cfg_command
              // 判断接收字符是否属于参数选择、增减、模式切换或雷达开关命令集合。
              // -----------------------------------------------------------------------------
              function cfg_command;
                input [7:0] value;  // 8 bit 函数 `cfg_command` 的“输入数值”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
                begin
                  case (value)
                    "G", "g", "N", "n", "B", "b", "+", "=", "-", "_",
                    "V", "v", "T", "t", "1", "2", "3", "4", "5", "Z", "z",
                    "H", "h", "O", "o", "E", "e", "Q", "q": cfg_command = 1'b1;
                    default: cfg_command = 1'b0;
                  endcase
                end
              endfunction

              // -----------------------------------------------------------------------------
              // [FUNCTION] 辅助函数: reply_char
              // 生成不会与长状态帧交叉的 @Kx 或 @PONG 短回复。
              // -----------------------------------------------------------------------------
              function [7:0] reply_char;
                input [1:0] kind;  // 2 bit 函数 `reply_char` 的“类型”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
                input [2:0] index;  // 3 bit 函数 `reply_char` 的“索引”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
                input [7:0] key_char;  // 8 bit 函数 `reply_char` 的“KEY / 字符”输入参数；只在本次调用展开期间参与求值，不会独立保存到下一时钟周期。
                begin
                  case (kind)
                    REPLY_KEY: begin
                      case (index)
                        3'd0: reply_char = "@";
                        3'd1: reply_char = "K";
                        3'd2: reply_char = key_char;
                        default: reply_char = 8'd10;
                      endcase
                    end
                    REPLY_PONG: begin
                      case (index)
                        3'd0: reply_char = "@";
                        3'd1: reply_char = "P";
                        3'd2: reply_char = "O";
                        3'd3: reply_char = "N";
                        3'd4: reply_char = "G";
                        default: reply_char = 8'd10;
                      endcase
                    end
                    default: reply_char = 8'd10;
                  endcase
                end
              endfunction

            // -----------------------------------------------------------------------------
            // [PROCESS] 过程块 1
            // 蓝牙 UART 接收器：同步 RX、半位确认起始位、LSB-first 采集 8 bit。
            // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
            // -----------------------------------------------------------------------------
            always @(posedge clk or negedge rst_n) begin
              if (!rst_n) begin
                  rx_sync <= 2'b11;
                  rx_state <= RX_IDLE;
                  rx_clk_count <= 16'd0;
                  rx_bit_index <= 3'd0;
                  rx_shift <= 8'd0;
                  rx_data <= 8'd0;
                  rx_valid <= 1'b0;
                end else begin
                  rx_sync <= {rx_sync[0], bt_uart_rx};
                  rx_valid <= 1'b0;
                  case (rx_state)
                    RX_IDLE: begin
                      rx_clk_count <= 16'd0;
                      rx_bit_index <= 3'd0;
                      if (!rx_sync[1]) rx_state <= RX_START;
                    end
                    RX_START: begin
                      if (rx_clk_count == (CLKS_PER_BIT / 2)) begin
                        rx_clk_count <= 16'd0;
                        rx_state <= !rx_sync[1] ? RX_DATA : RX_IDLE;
                      end else begin
                        rx_clk_count <= rx_clk_count + 16'd1;
              end
            end

                    RX_DATA: begin
                      if (rx_clk_count == CLKS_PER_BIT - 1) begin
                        rx_clk_count <= 16'd0;
                        rx_shift[rx_bit_index] <= rx_sync[1];
                        if (rx_bit_index == 3'd7) begin
                          rx_bit_index <= 3'd0;
                          rx_state <= RX_STOP;
                        end else begin
                          rx_bit_index <= rx_bit_index + 3'd1;
                        end
                      end else begin
                        rx_clk_count <= rx_clk_count + 16'd1;
                      end
                    end
                    default: begin
                      if (rx_clk_count == CLKS_PER_BIT - 1) begin
                        rx_clk_count <= 16'd0;
                        rx_data <= rx_shift;
                        rx_valid <= rx_sync[1];
                        rx_state <= RX_IDLE;
                      end else begin
                        rx_clk_count <= rx_clk_count + 16'd1;
                      end
                    end
                  endcase
                end
              end

              // -----------------------------------------------------------------------------
              // [PROCESS] 过程块 2
              // 手动/自动模式和雷达开关命令处理，并实现 300 ms 手动运动看门狗。
              // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
              // -----------------------------------------------------------------------------
              always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                  manual_mode <= 1'b0;
                  manual_cmd <= MANUAL_STOP;
                  lidar_enable <= 1'b1;
                  manual_timeout_count <= 24'd0;
                  manual_watchdog_expired <= 1'b0;
                  motor_stop_pulse <= 1'b0;
                end else begin
                  motor_stop_pulse <= 1'b0;
                  if (manual_mode && (manual_cmd != MANUAL_STOP)) begin
                    if (manual_timeout_count >= MANUAL_TIMEOUT_CYCLES - 1) begin
                      manual_cmd <= MANUAL_STOP;
                      motor_stop_pulse <= 1'b1;
                      manual_timeout_count <= 24'd0;
                      manual_watchdog_expired <= 1'b1;
                    end else begin
                      manual_timeout_count <= manual_timeout_count + 24'd1;
                    end
                  end else begin
                    manual_timeout_count <= 24'd0;
                  end

                  if (rx_valid) begin
                    case (rx_data)
                      // -----------------------------------------------------------------------------
                      // 关键分支
                      // H/h：进入手机手动模式，同时先发强制停车防止继承旧速度。
                      // -----------------------------------------------------------------------------
                      "H", "h": begin
                        manual_mode <= 1'b1;
                        manual_cmd <= MANUAL_STOP;
                        motor_stop_pulse <= 1'b1;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      // -----------------------------------------------------------------------------
                      // 关键分支
                      // O/o：退出手动回到自动路线模式，并先停车。
                      // -----------------------------------------------------------------------------
                      "O", "o": begin
                        manual_mode <= 1'b0;
                        manual_cmd <= MANUAL_STOP;
                        motor_stop_pulse <= 1'b1;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "E", "e": lidar_enable <= 1'b1;
                      "Q", "q": lidar_enable <= 1'b0;
                      "W", "w", "U", "u": if (manual_mode) begin
                        manual_cmd <= MANUAL_FORWARD;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "S", "s": if (manual_mode) begin
                        manual_cmd <= MANUAL_REVERSE;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "A", "a", "L", "l": if (manual_mode) begin
                        manual_cmd <= MANUAL_LEFT;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "D", "d", "R", "r": if (manual_mode) begin
                        manual_cmd <= MANUAL_RIGHT;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      "C", "c", "M", "m": if (manual_mode) begin
                        manual_cmd <= MANUAL_STOP;
                        motor_stop_pulse <= 1'b1;
                        manual_timeout_count <= 24'd0;
                        manual_watchdog_expired <= 1'b0;
                      end
                      default: begin end
                    endcase
                  end
                end
              end

              // -----------------------------------------------------------------------------
              // [PROCESS] 过程块 3
              // 蓝牙心跳在线判定：仅 P/p 刷新 3 s 链路超时计数。
              // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
              // -----------------------------------------------------------------------------
              always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                  link_idle_count <= 32'd0;
                  link_active <= 1'b0;
                end else if (rx_valid && (rx_data == "P" || rx_data == "p")) begin
                  link_idle_count <= 32'd0;
                  link_active <= 1'b1;
                end else if (link_active) begin
                  if (link_idle_count >= LINK_TIMEOUT_CYCLES - 1) begin
                    link_active <= 1'b0;
                    link_idle_count <= LINK_TIMEOUT_CYCLES - 1;
                  end else begin
                    link_idle_count <= link_idle_count + 32'd1;
                  end
                end else begin
                  link_idle_count <= 32'd0;
                end
              end

              // -----------------------------------------------------------------------------
              // [PROCESS] 过程块 4
              // 锁存最近一次有效超声结果，确保状态上报不会读取单周期脉冲的瞬时值。
              // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
              // -----------------------------------------------------------------------------
              always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                  ultrasonic_last_distance_cm <= 16'hffff;
                  ultrasonic_last_valid <= 1'b0;
                  ultrasonic_last_timeout <= 1'b0;
                  ultrasonic_last_stop <= 1'b0;
                end else if (ultrasonic_distance_valid) begin
                  ultrasonic_last_distance_cm <= ultrasonic_distance_cm;
                  ultrasonic_last_valid <= !ultrasonic_echo_timeout &&
                                           (ultrasonic_distance_cm != 16'hffff);
                  ultrasonic_last_timeout <= ultrasonic_echo_timeout;
                  ultrasonic_last_stop <= ultrasonic_obstacle_stop;
                end
              end

              // -----------------------------------------------------------------------------
              // [PROCESS] 过程块 5
              // 生成待发送短回复：方向/配置命令回 @Kx，心跳回 @PONG。
              // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
              // -----------------------------------------------------------------------------
              always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                  pending_reply <= 1'b0;
                  reply_kind <= REPLY_NONE;
                  reply_key <= 8'd0;
                end else if (rx_valid) begin
                  if (norm_key != 8'd0) begin
                    pending_reply <= 1'b1;
                    reply_kind <= REPLY_KEY;
                    reply_key <= norm_key;
                  end else if (rx_data == "P" || rx_data == "p") begin
                    pending_reply <= 1'b1;
                    reply_kind <= REPLY_PONG;
                    reply_key <= 8'd0;
                  end else if (is_cfg_command) begin
                    pending_reply <= 1'b1;
                    reply_kind <= REPLY_KEY;
                    reply_key <= rx_data;
                  end
                end else if (!sending_status && !sending_reply && pending_reply && !tx_busy) begin
                  pending_reply <= 1'b0;
                end
              end

              // -----------------------------------------------------------------------------
              // [PROCESS] 过程块 6
              // 把配置字符转换为选择、增减和设置模式的单周期脉冲。
              // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
              // -----------------------------------------------------------------------------
              always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                  bt_cfg_mode_pulse <= 1'b0;
                  bt_cfg_next_pulse <= 1'b0;
                  bt_cfg_prev_pulse <= 1'b0;
                  bt_cfg_inc_pulse <= 1'b0;
                  bt_cfg_dec_pulse <= 1'b0;
                  bt_cfg_select <= 3'd0;
                  bt_cfg_select_valid <= 1'b0;
                end else begin
                  bt_cfg_mode_pulse <= 1'b0;
                  bt_cfg_next_pulse <= 1'b0;
                  bt_cfg_prev_pulse <= 1'b0;
                  bt_cfg_inc_pulse <= 1'b0;
                  bt_cfg_dec_pulse <= 1'b0;
                  bt_cfg_select_valid <= 1'b0;
                  if (rx_valid) begin
                    case (rx_data)
                      "G", "g": bt_cfg_mode_pulse <= 1'b1;
                      "N", "n": bt_cfg_next_pulse <= 1'b1;
                      "B", "b": bt_cfg_prev_pulse <= 1'b1;
                      "+", "=": bt_cfg_inc_pulse <= 1'b1;
                      "-", "_": bt_cfg_dec_pulse <= 1'b1;
                      "V", "v": begin bt_cfg_select <= 3'd0; bt_cfg_select_valid <= 1'b1; end
                      "T", "t": begin bt_cfg_select <= 3'd1; bt_cfg_select_valid <= 1'b1; end
                      "1": begin bt_cfg_select <= 3'd2; bt_cfg_select_valid <= 1'b1; end
                      "2": begin bt_cfg_select <= 3'd3; bt_cfg_select_valid <= 1'b1; end
                      "3": begin bt_cfg_select <= 3'd4; bt_cfg_select_valid <= 1'b1; end
                      "4": begin bt_cfg_select <= 3'd5; bt_cfg_select_valid <= 1'b1; end
                      "5", "Z", "z": begin bt_cfg_select <= 3'd6; bt_cfg_select_valid <= 1'b1; end
                      default: begin end
                    endcase
                  end
                end
              end

              // -----------------------------------------------------------------------------
              // [PROCESS] 过程块 7
              // 自动模式下把蓝牙方向字符模拟成约 30 ms 的低有效实体按键。
              // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
              // -----------------------------------------------------------------------------
              always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                  up_hold_count <= 22'd0;
                  down_hold_count <= 22'd0;
                  left_hold_count <= 22'd0;
                  right_hold_count <= 22'd0;
                  mid_hold_count <= 22'd0;
                end else begin
                  if (up_hold_count != 22'd0) up_hold_count <= up_hold_count - 22'd1;
                  if (down_hold_count != 22'd0) down_hold_count <= down_hold_count - 22'd1;
                  if (left_hold_count != 22'd0) left_hold_count <= left_hold_count - 22'd1;
                  if (right_hold_count != 22'd0) right_hold_count <= right_hold_count - 22'd1;
                  if (mid_hold_count != 22'd0) mid_hold_count <= mid_hold_count - 22'd1;
                  if (rx_valid && !manual_mode) begin
                    case (rx_data)
                      "U", "u", "W", "w": up_hold_count <= KEY_HOLD_CYCLES[21:0];
                      "D", "d", "S", "s": down_hold_count <= KEY_HOLD_CYCLES[21:0];
                      "L", "l", "A", "a": left_hold_count <= KEY_HOLD_CYCLES[21:0];
                      "R", "r":           right_hold_count <= KEY_HOLD_CYCLES[21:0];
                      "M", "m", "C", "c": mid_hold_count <= KEY_HOLD_CYCLES[21:0];
                      default: begin end
                    endcase
                  end
                end
              end

              // -----------------------------------------------------------------------------
              // [PROCESS] 过程块 8
              // 唯一 TX 调度器：原子发送短回复、@S、@L，并在发送前锁存整帧快照。
              // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
              // -----------------------------------------------------------------------------
              always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                  tx_line <= 1'b1;
                  tx_busy <= 1'b0;
                  tx_busy_d <= 1'b0;
                  tx_state <= TX_IDLE;
                  tx_clk_count <= 16'd0;
                  tx_bit_index <= 3'd0;
                  tx_shift <= 8'd0;
                  tx_data <= 8'd0;
                  tx_start <= 1'b0;
                  status_count <= 32'd0;
                  status_left <= 7'd0;
                  status_index <= 7'd0;
                  active_status_lidar <= 1'b0;
                  sending_status <= 1'b0;
                  status_packet_lidar <= 1'b0;
                  sending_reply <= 1'b0;
                  active_reply_kind <= REPLY_NONE;
                  active_reply_key <= 8'd0;
                  reply_index <= 3'd0;
                  route_status_pending <= 1'b1;
                  route_status_seq <= 8'd0;
                  lidar_status_seq <= 8'd0;
                  active_status_seq <= 8'd0;
                  prev_menu_info <= 2'd0;
                  prev_state_info <= 5'd0;
                  prev_route_info <= 64'd0;
                  prev_active_route_steps <= 6'd0;
                  prev_completed_route_steps <= 6'd0;
                  prev_now_x <= 8'd0;
                  prev_now_y <= 8'd0;
                  prev_target_x <= 8'd0;
                  prev_target_y <= 8'd0;
                  prev_step_length_set <= 16'd0;
                  prev_overall_speed <= 10'd0;
                  prev_lf_default_speed <= 10'd0;
                  prev_rf_default_speed <= 10'd0;
                  prev_lr_default_speed <= 10'd0;
                  prev_rr_default_speed <= 10'd0;
                  prev_lidar_safe_mm <= 16'd0;
                  prev_setting_mode <= 1'b0;
                  prev_selected_param <= 3'd0;
                  prev_manual_mode <= 1'b0;
                  prev_manual_cmd <= MANUAL_STOP;
                  prev_manual_watchdog_expired <= 1'b0;
                  prev_lidar_enable <= 1'b0;
                  prev_motor_blocked <= 1'b0;
                  status_menu_info <= 2'd0;
                  status_state_info <= 5'd0;
                  status_route_info <= 64'd0;
                  status_active_route_steps <= 6'd0;
                  status_completed_route_steps <= 6'd0;
                  status_now_x <= 8'd0;
                  status_now_y <= 8'd0;
                  status_target_x <= 8'd0;
                  status_target_y <= 8'd0;
                  status_step_length_set <= 16'd0;
                  status_overall_speed <= 10'd0;
                  status_lf_default_speed <= 10'd0;
                  status_rf_default_speed <= 10'd0;
                  status_lr_default_speed <= 10'd0;
                  status_rr_default_speed <= 10'd0;
                  status_lidar_safe_mm_route <= 16'd0;
                  status_setting_mode <= 1'b0;
                  status_selected_param <= 3'd0;
                  status_lidar_online <= 1'b0;
                  status_manual_mode <= 1'b0;
                  status_lidar_enable <= 1'b0;
                  status_lidar_obstacle_stop <= 1'b0;
                  status_ultrasonic_obstacle_stop <= 1'b0;
                  status_merged_obstacle_stop <= 1'b0;
                  status_motor_blocked <= 1'b0;
                  status_ultrasonic_distance_valid <= 1'b0;
                  status_ultrasonic_echo_timeout <= 1'b0;
                  status_ultrasonic_distance_cm <= 16'hffff;
                  status_manual_cmd <= MANUAL_STOP;
                  status_manual_watchdog_expired <= 1'b0;
                  status_c5_frame_count <= 16'd0;
                  status_c5_last_lf_pulse <= 12'd1500;
                  status_c5_last_rf_pulse <= 12'd1500;
                  status_c5_last_lr_pulse <= 12'd1500;
                  status_c5_last_rr_pulse <= 12'd1500;
                  status_c5_stop_frames_complete <= 1'b0;
                  status_c5_stop_frames_remaining <= 2'd0;
                  status_c5_frame_busy <= 1'b0;
                  status_lidar_safe_mm <= 16'd0;
                  status_lidar_front_distance_mm <= 16'hffff;
                  status_lidar_sector_mm[0] <= 16'hffff;
                  status_lidar_sector_mm[1] <= 16'hffff;
                  status_lidar_sector_mm[2] <= 16'hffff;
                  status_lidar_sector_mm[3] <= 16'hffff;
                  status_lidar_sector_mm[4] <= 16'hffff;
                  status_lidar_sector_mm[5] <= 16'hffff;
                  status_lidar_sector_mm[6] <= 16'hffff;
                  status_lidar_sector_mm[7] <= 16'hffff;
                end else begin
                  tx_start <= 1'b0;
                  tx_busy_d <= tx_busy;

                  if (route_status_changed) begin
                    prev_menu_info <= menu_info;
                    prev_state_info <= state_info;
                    prev_route_info <= route_info;
                    prev_active_route_steps <= active_route_steps;
                    prev_completed_route_steps <= completed_route_steps;
                    prev_now_x <= now_x;
                    prev_now_y <= now_y;
                    prev_target_x <= target_x;
                    prev_target_y <= target_y;
                    prev_step_length_set <= step_length_set;
                    prev_overall_speed <= overall_speed;
                    prev_lf_default_speed <= lf_default_speed;
                    prev_rf_default_speed <= rf_default_speed;
                    prev_lr_default_speed <= lr_default_speed;
                    prev_rr_default_speed <= rr_default_speed;
                    prev_lidar_safe_mm <= lidar_safe_mm;
                    prev_setting_mode <= setting_mode;
                    prev_selected_param <= selected_param;
                    prev_manual_mode <= manual_mode;
                    prev_manual_cmd <= manual_cmd;
                    prev_manual_watchdog_expired <= manual_watchdog_expired;
                    prev_lidar_enable <= lidar_enable;
                    prev_motor_blocked <= motor_blocked;
                    route_status_pending <= 1'b1;
                  end

                  case (tx_state)
                    TX_IDLE: begin
                      tx_line <= 1'b1;
                      tx_busy <= 1'b0;
                      tx_clk_count <= 16'd0;
                      tx_bit_index <= 3'd0;
                      if (tx_start) begin
                        tx_busy <= 1'b1;
                        tx_shift <= tx_data;
                        tx_state <= TX_START;
                        tx_line <= 1'b0;
                      end
                    end
                    TX_START: begin
                      tx_busy <= 1'b1;
                      tx_line <= 1'b0;
                      if (tx_clk_count == CLKS_PER_BIT - 1) begin
                        tx_clk_count <= 16'd0;
                        tx_state <= TX_DATA;
                        tx_line <= tx_shift[0];
                      end else begin
                        tx_clk_count <= tx_clk_count + 16'd1;
                      end
                    end
                    TX_DATA: begin
                      tx_busy <= 1'b1;
                      tx_line <= tx_shift[tx_bit_index];
                      if (tx_clk_count == CLKS_PER_BIT - 1) begin
                        tx_clk_count <= 16'd0;
                        if (tx_bit_index == 3'd7) begin
                          tx_bit_index <= 3'd0;
                          tx_state <= TX_STOP;
                          tx_line <= 1'b1;
                        end else begin
                          tx_bit_index <= tx_bit_index + 3'd1;
                          tx_line <= tx_shift[tx_bit_index + 3'd1];
                        end
                      end else begin
                        tx_clk_count <= tx_clk_count + 16'd1;
                      end
                    end
                    default: begin
                      tx_busy <= 1'b1;
                      tx_line <= 1'b1;
                      if (tx_clk_count == CLKS_PER_BIT - 1) begin
                        tx_clk_count <= 16'd0;
                        tx_busy <= 1'b0;
                        tx_state <= TX_IDLE;
                      end else begin
                        tx_clk_count <= tx_clk_count + 16'd1;
                      end
                    end
                  endcase

                  if (rx_valid && ((rx_data == "?") ||
                                   (norm_key != 8'd0) ||
                                   is_cfg_command))
                    route_status_pending <= 1'b1;
                  // -----------------------------------------------------------------------------
                  // 关键分支
                  // 发送调度入口：只有当前无完整帧时才从挂起队列选择下一帧。
                  // -----------------------------------------------------------------------------
                  if (!sending_status && !sending_reply && !status_due)
                    status_count <= status_count + 32'd1;

                  // -----------------------------------------------------------------------------
                  // 关键分支
                  // 发送调度入口：只有当前无完整帧时才从挂起队列选择下一帧。
                  // -----------------------------------------------------------------------------
                  if (!sending_status && !sending_reply && pending_reply && !tx_busy) begin
                    active_reply_kind <= reply_kind;
                    active_reply_key <= reply_key;
                    reply_index <= 3'd0;
                    sending_reply <= 1'b1;
                    tx_data <= reply_char(reply_kind, 3'd0, reply_key);
                    tx_start <= 1'b1;
                  end else if (sending_reply) begin
                    if (tx_done) begin
                      if (reply_done) begin
                        sending_reply <= 1'b0;
                        active_reply_kind <= REPLY_NONE;
                      end else begin
                        reply_index <= reply_index + 3'd1;
                        tx_data <= reply_char(active_reply_kind, reply_index + 3'd1, active_reply_key);
                        tx_start <= 1'b1;
                      end
                    end
                  end else if (!sending_status) begin
                    if ((route_status_pending || status_due) && !tx_busy) begin
                      status_count <= 32'd0;
                      sending_status <= 1'b1;
                      status_index <= 7'd0;
                      tx_data <= "@";
                      tx_start <= 1'b1;
                      if (route_status_pending) begin
                        route_status_pending <= 1'b0;
                        active_status_lidar <= 1'b0;
                        active_status_seq <= route_status_seq;
                        route_status_seq <= route_status_seq + 8'd1;
                        status_left <= ROUTE_STATUS_LEFT;
                        status_menu_info <= menu_info;
                        status_state_info <= state_info;
                        status_route_info <= route_info;
                        status_active_route_steps <= active_route_steps;
                        status_completed_route_steps <= completed_route_steps;
                        status_now_x <= now_x;
                        status_now_y <= now_y;
                        status_target_x <= target_x;
                        status_target_y <= target_y;
                        status_step_length_set <= step_length_set;
                        status_overall_speed <= overall_speed;
                        status_lf_default_speed <= lf_default_speed;
                        status_rf_default_speed <= rf_default_speed;
                        status_lr_default_speed <= lr_default_speed;
                        status_rr_default_speed <= rr_default_speed;
                        status_lidar_safe_mm_route <= lidar_safe_mm;
                        status_setting_mode <= setting_mode;
                        status_selected_param <= selected_param;
                        status_lidar_online <= lidar_online;
                        status_manual_mode <= manual_mode;
                        status_lidar_enable <= lidar_enable;
                        status_manual_cmd <= manual_cmd;
                        status_manual_watchdog_expired <= manual_watchdog_expired;
                        status_motor_blocked <= motor_blocked;
                        status_c5_frame_count <= c5_frame_count;
                        status_c5_last_lf_pulse <= c5_last_lf_pulse;
                        status_c5_last_rf_pulse <= c5_last_rf_pulse;
                        status_c5_last_lr_pulse <= c5_last_lr_pulse;
                        status_c5_last_rr_pulse <= c5_last_rr_pulse;
                        status_c5_stop_frames_complete <= c5_stop_frames_complete;
                        status_c5_stop_frames_remaining <= c5_stop_frames_remaining;
                        status_c5_frame_busy <= c5_frame_busy;
                      end else if (status_packet_lidar) begin
                        status_packet_lidar <= 1'b0;
                        active_status_lidar <= 1'b1;
                        active_status_seq <= lidar_status_seq;
                        lidar_status_seq <= lidar_status_seq + 8'd1;
                        status_left <= LIDAR_STATUS_LEFT;
                        status_lidar_online <= lidar_online;
                        status_lidar_obstacle_stop <= lidar_obstacle_stop;
                        status_ultrasonic_obstacle_stop <= ultrasonic_last_stop;
                        status_merged_obstacle_stop <= merged_obstacle_stop | ultrasonic_last_stop;
                        status_motor_blocked <= motor_blocked;
                        status_c5_frame_count <= c5_frame_count;
                        status_c5_last_lf_pulse <= c5_last_lf_pulse;
                        status_c5_last_rf_pulse <= c5_last_rf_pulse;
                        status_c5_last_lr_pulse <= c5_last_lr_pulse;
                        status_c5_last_rr_pulse <= c5_last_rr_pulse;
                        status_c5_stop_frames_complete <= c5_stop_frames_complete;
                        status_c5_stop_frames_remaining <= c5_stop_frames_remaining;
                        status_c5_frame_busy <= c5_frame_busy;
                        status_ultrasonic_distance_valid <= ultrasonic_last_valid;
                        status_ultrasonic_echo_timeout <= ultrasonic_last_timeout;
                        status_ultrasonic_distance_cm <= ultrasonic_last_distance_cm;
                        status_lidar_safe_mm <= lidar_safe_mm;
                        status_lidar_front_distance_mm <= lidar_front_distance_mm;
                        status_lidar_sector_mm[0] <= lidar_sector_front_mm;
                        status_lidar_sector_mm[1] <= lidar_sector_front_right_mm;
                        status_lidar_sector_mm[2] <= lidar_sector_right_mm;
                        status_lidar_sector_mm[3] <= lidar_sector_rear_right_mm;
                        status_lidar_sector_mm[4] <= lidar_sector_rear_mm;
                        status_lidar_sector_mm[5] <= lidar_sector_rear_left_mm;
                        status_lidar_sector_mm[6] <= lidar_sector_left_mm;
                        status_lidar_sector_mm[7] <= lidar_sector_front_left_mm;
                      end else begin
                        status_packet_lidar <= 1'b1;
                        active_status_lidar <= 1'b0;
                        active_status_seq <= route_status_seq;
                        route_status_seq <= route_status_seq + 8'd1;
                        status_left <= ROUTE_STATUS_LEFT;
                        status_menu_info <= menu_info;
                        status_state_info <= state_info;
                        status_route_info <= route_info;
                        status_active_route_steps <= active_route_steps;
                        status_completed_route_steps <= completed_route_steps;
                        status_now_x <= now_x;
                        status_now_y <= now_y;
                        status_target_x <= target_x;
                        status_target_y <= target_y;
                        status_step_length_set <= step_length_set;
                        status_overall_speed <= overall_speed;
                        status_lf_default_speed <= lf_default_speed;
                        status_rf_default_speed <= rf_default_speed;
                        status_lr_default_speed <= lr_default_speed;
                        status_rr_default_speed <= rr_default_speed;
                        status_lidar_safe_mm_route <= lidar_safe_mm;
                        status_setting_mode <= setting_mode;
                        status_selected_param <= selected_param;
                        status_lidar_online <= lidar_online;
                        status_manual_mode <= manual_mode;
                        status_lidar_enable <= lidar_enable;
                        status_manual_cmd <= manual_cmd;
                        status_manual_watchdog_expired <= manual_watchdog_expired;
                        status_motor_blocked <= motor_blocked;
                      end
                    end
                  end else if (tx_done) begin
                    if (status_left == 7'd0) begin
                      sending_status <= 1'b0;
                    end else begin
                      tx_data <= status_char(active_status_lidar, status_index);
                      status_index <= status_index + 7'd1;
                      status_left <= status_left - 7'd1;
                      tx_start <= 1'b1;
                    end
                  end
                end
              end

endmodule

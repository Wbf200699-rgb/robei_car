// =============================================================================
// 模块: sys_route_control_fsm
// 作用: 路线控制层次封装：五路按键消抖后进入实际路线核心。
// 说明: 该模块自身没有业务状态，只负责 key_*_n 到 key_*_pulse 的转换与层次连接。
// 说明: 当前实例类型拼写为 route_contol_core，这是工程真实依赖，不是注释笔误。
//
// Robei 源: little_car/model/sys_route_control_fsm.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module sys_route_control_fsm (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire key_up_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_down_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_left_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_right_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_mid_n,  // 低有效按键输入/输出；0 表示按下。
  output wire [1:0] menu_info,  // 菜单焦点/页面状态编码。
  output wire [63:0] route_info,  // 64 bit 路线命令队列；最低 2 bit 是下一条命令。
  input wire run_step_pulse,  // 一条运动动作完成的单周期脉冲。
  input wire obstacle_stop,  // 合并或当前模块产生的停车请求。
  input wire [15:0] step_length_set,  // 手机设置的单步长度参数。
  input wire [15:0] speed_set,  // 路线控制采用的整体速度参数。
  output wire [4:0] state_info,  // 路线 FSM 状态编码。
  output wire [63:0] old_route_info,  // 已完成路线历史，用于 GUI 和手机显示。
  output wire [5:0] active_route_steps,  // 当前待执行内部命令数量。
  output wire [5:0] completed_route_steps,  // 已完成内部命令数量。
  output wire [15:0] step_length,  // 当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。
  output wire [15:0] speed,  // 当前生效的路线整体速度参数，向 GUI 和执行控制链同步。
  output wire [15:0] total_route_length,  // 已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。
  output wire [15:0] complete_route_length,  // completed_route_steps 乘以 step_length 得到的已完成路线长度。
  output wire [15:0] remaining_route_length,  // active_route_steps 乘以 step_length 得到的剩余路线长度。
  output wire [7:0] now_x,  // 小车当前逻辑栅格 X 坐标；自动动作完成后由路线核心更新。
  output wire [7:0] now_y,  // 小车当前逻辑栅格 Y 坐标；自动动作完成后由路线核心更新。
  output wire [7:0] target_x,  // 编辑路线末端预测 X 坐标；添加或撤销动作时同步变化。
  output wire [7:0] target_y  // 编辑路线末端预测 Y 坐标；添加或撤销动作时同步变化。
);

  // ===========================================================================
  // 层次内部网络
  // 每条注释给出驱动端以及所有接收端，便于从顶层沿数据流追踪。
  // ===========================================================================
  // key_up_filter.key_pulse -> route_control_core6.key_up_pulse
  wire key_up_filter_key_pulse;  // “KEY”事件脉冲；条件成立时仅保持一个 clk 周期。 具体驱动端和全部接收端见上一行。
  // key_down_filter.key_pulse -> route_control_core6.key_down_pulse
  wire key_down_filter_key_pulse;  // “KEY”事件脉冲；条件成立时仅保持一个 clk 周期。 具体驱动端和全部接收端见上一行。
  // key_left_filter.key_pulse -> route_control_core6.key_left_pulse
  wire key_left_filter_key_pulse;  // “KEY”事件脉冲；条件成立时仅保持一个 clk 周期。 具体驱动端和全部接收端见上一行。
  // key_right_filter.key_pulse -> route_control_core6.key_right_pulse
  wire key_right_filter_key_pulse;  // “KEY”事件脉冲；条件成立时仅保持一个 clk 周期。 具体驱动端和全部接收端见上一行。
  // key_mid_filter.key_pulse -> route_control_core6.key_mid_pulse
  wire key_mid_filter_key_pulse;  // “KEY”事件脉冲；条件成立时仅保持一个 clk 周期。 具体驱动端和全部接收端见上一行。

  // ---------------------------------------------------------------------------
  // 子模块实例: key_up_filter
  // 类型: key_pulse_filter
  // 作用: 按键同步、消抖和单脉冲生成器。
  // 连接说明: 只有输入持续稳定达到计数阈值后才改变状态；每次按下只产生一个时钟脉冲。
  // ---------------------------------------------------------------------------
  key_pulse_filter
  key_up_filter (
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_up_n),
    .key_pulse(key_up_filter_key_pulse)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: key_down_filter
  // 类型: key_pulse_filter
  // 作用: 按键同步、消抖和单脉冲生成器。
  // 连接说明: 只有输入持续稳定达到计数阈值后才改变状态；每次按下只产生一个时钟脉冲。
  // ---------------------------------------------------------------------------
  key_pulse_filter
  key_down_filter (
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_down_n),
    .key_pulse(key_down_filter_key_pulse)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: key_left_filter
  // 类型: key_pulse_filter
  // 作用: 按键同步、消抖和单脉冲生成器。
  // 连接说明: 只有输入持续稳定达到计数阈值后才改变状态；每次按下只产生一个时钟脉冲。
  // ---------------------------------------------------------------------------
  key_pulse_filter
  key_left_filter (
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_left_n),
    .key_pulse(key_left_filter_key_pulse)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: key_right_filter
  // 类型: key_pulse_filter
  // 作用: 按键同步、消抖和单脉冲生成器。
  // 连接说明: 只有输入持续稳定达到计数阈值后才改变状态；每次按下只产生一个时钟脉冲。
  // ---------------------------------------------------------------------------
  key_pulse_filter
  key_right_filter (
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_right_n),
    .key_pulse(key_right_filter_key_pulse)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: key_mid_filter
  // 类型: key_pulse_filter
  // 作用: 按键同步、消抖和单脉冲生成器。
  // 连接说明: 只有输入持续稳定达到计数阈值后才改变状态；每次按下只产生一个时钟脉冲。
  // ---------------------------------------------------------------------------
  key_pulse_filter
  key_mid_filter (
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_mid_n),
    .key_pulse(key_mid_filter_key_pulse)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: route_control_core6
  // 类型: route_contol_core
  // 作用: 当前实际使用的路线编辑、执行、坐标更新状态机。
  // 连接说明: route_info 每 2 bit 保存一条内部命令：00 停止、01 右转、10 左转、11 前进。
  // 连接说明: 编辑时一次左/右操作会写入转向和前进两个内部命令，因此 UI 步和内部命令数不同。
  // ---------------------------------------------------------------------------
  route_contol_core
  route_control_core6 (
    .clk(clk),
    .rst_n(rst_n),
    .key_up_pulse(key_up_filter_key_pulse),
    .key_down_pulse(key_down_filter_key_pulse),
    .key_left_pulse(key_left_filter_key_pulse),
    .key_right_pulse(key_right_filter_key_pulse),
    .key_mid_pulse(key_mid_filter_key_pulse),
    .menu_info(menu_info),
    .route_info(route_info),
    .run_step_pulse(run_step_pulse),
    .obstacle_stop(obstacle_stop),
    .step_length_set(step_length_set),
    .speed_set(speed_set),
    .state_info(state_info),
    .old_route_info(old_route_info),
    .active_route_steps(active_route_steps),
    .completed_route_steps(completed_route_steps),
    .step_length(step_length),
    .speed(speed),
    .total_route_length(total_route_length),
    .complete_route_length(complete_route_length),
    .remaining_route_length(remaining_route_length),
    .now_x(now_x),
    .now_y(now_y),
    .target_x(target_x),
    .target_y(target_y)
  );

endmodule

// =============================================================================
// 模块: temp_gui_status_source
// 作用: GUI 临时状态源。
// 说明: 为早期显示验证提供固定运行状态。
//
// Robei 源: little_car/model/temp_gui_status_source.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module temp_gui_status_source (
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
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“old_route_info”：已完成路线历史，用于 GUI 和手机显示。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign old_route_info = 64'd0;
  // [ASSIGN] 组合定义“active_route_steps”：当前待执行内部命令数量。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign active_route_steps = 6'd12;
  // [ASSIGN] 组合定义“completed_route_steps”：已完成内部命令数量。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign completed_route_steps = 6'd0;
  // [ASSIGN] 组合定义“step_length”：当前生效的路线单步长度；设置为零时路线核心回退到默认值 20。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign step_length = 16'd20;
  // [ASSIGN] 组合定义“speed”：当前生效的路线整体速度参数，向 GUI 和执行控制链同步。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign speed = 16'd25;
  // [ASSIGN] 组合定义“total_route_length”：已完成步数与待执行步数之和乘以 step_length 得到的计划总长度。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign total_route_length = 16'd240;
  // [ASSIGN] 组合定义“complete_route_length”：completed_route_steps 乘以 step_length 得到的已完成路线长度。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign complete_route_length = 16'd0;
  // [ASSIGN] 组合定义“remaining_route_length”：active_route_steps 乘以 step_length 得到的剩余路线长度。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign remaining_route_length = 16'd240;
  // [ASSIGN] 组合定义“now_x”：小车当前逻辑栅格 X 坐标；自动动作完成后由路线核心更新。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign now_x = 8'd128;
  // [ASSIGN] 组合定义“now_y”：小车当前逻辑栅格 Y 坐标；自动动作完成后由路线核心更新。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign now_y = 8'd128;
  // [ASSIGN] 组合定义“target_x”：编辑路线末端预测 X 坐标；添加或撤销动作时同步变化。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign target_x = 8'd128;
  // [ASSIGN] 组合定义“target_y”：编辑路线末端预测 Y 坐标；添加或撤销动作时同步变化。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign target_y = 8'd124;

endmodule

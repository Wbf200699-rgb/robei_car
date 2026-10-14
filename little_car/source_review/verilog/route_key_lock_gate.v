// =============================================================================
// 模块: route_key_lock_gate
// 作用: 路线按键门控器，手机处于参数设置模式时阻断路线编辑按键。
// 说明: 锁定时所有输出保持 1，即低有效按键的未按下状态。
//
// Robei 源: little_car/model/route_key_lock_gate.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module route_key_lock_gate (
  input wire route_key_lock,  // 参数设置期间的路线按键锁；为 1 时实体/蓝牙方向键不会修改路线。
  input wire key_up_n_in,  // 门控前“上键/前进”低有效按键电平。
  input wire key_down_n_in,  // 门控前“下键/后退”低有效按键电平。
  input wire key_left_n_in,  // 门控前“左”低有效按键电平。
  input wire key_right_n_in,  // 门控前“右”低有效按键电平。
  input wire key_mid_n_in,  // 门控前“中键/确认”低有效按键电平。
  output wire key_up_n_out,  // 参数设置锁生效时被强制为 1 的“上键/前进”低有效按键输出。
  output wire key_down_n_out,  // 参数设置锁生效时被强制为 1 的“下键/后退”低有效按键输出。
  output wire key_left_n_out,  // 参数设置锁生效时被强制为 1 的“左”低有效按键输出。
  output wire key_right_n_out,  // 参数设置锁生效时被强制为 1 的“右”低有效按键输出。
  output wire key_mid_n_out  // 参数设置锁生效时被强制为 1 的“中键/确认”低有效按键输出。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“key_up_n_out”：参数设置锁生效时被强制为 1 的“上键/前进”低有效按键输出。 该组合结果直接由 `route_key_lock`（路线 / KEY / 锁定）、`key_up_n_in`（KEY / 上键/前进 / N / IN）的当前值决定，不引入时钟周期延迟。
  assign key_up_n_out = route_key_lock ? 1'b1 : key_up_n_in;
  // [ASSIGN] 组合定义“key_down_n_out”：参数设置锁生效时被强制为 1 的“下键/后退”低有效按键输出。 该组合结果直接由 `route_key_lock`（路线 / KEY / 锁定）、`key_down_n_in`（KEY / 下键/后退 / N / IN）的当前值决定，不引入时钟周期延迟。
  assign key_down_n_out = route_key_lock ? 1'b1 : key_down_n_in;
  // [ASSIGN] 组合定义“key_left_n_out”：参数设置锁生效时被强制为 1 的“左”低有效按键输出。 该组合结果直接由 `route_key_lock`（路线 / KEY / 锁定）、`key_left_n_in`（KEY / 左 / N / IN）的当前值决定，不引入时钟周期延迟。
  assign key_left_n_out = route_key_lock ? 1'b1 : key_left_n_in;
  // [ASSIGN] 组合定义“key_right_n_out”：参数设置锁生效时被强制为 1 的“右”低有效按键输出。 该组合结果直接由 `route_key_lock`（路线 / KEY / 锁定）、`key_right_n_in`（KEY / 右 / N / IN）的当前值决定，不引入时钟周期延迟。
  assign key_right_n_out = route_key_lock ? 1'b1 : key_right_n_in;
  // [ASSIGN] 组合定义“key_mid_n_out”：参数设置锁生效时被强制为 1 的“中键/确认”低有效按键输出。 该组合结果直接由 `route_key_lock`（路线 / KEY / 锁定）、`key_mid_n_in`（KEY / 中键/确认 / N / IN）的当前值决定，不引入时钟周期延迟。
  assign key_mid_n_out = route_key_lock ? 1'b1 : key_mid_n_in;

endmodule

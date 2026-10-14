// =============================================================================
// 模块: key_low_merge
// 作用: 低有效按键合并器，把实体按键和蓝牙模拟按键接到同一条路线输入链。
// 说明: 低有效信号使用逻辑与合并：任意来源拉低都会形成一次有效按键。
//
// Robei 源: little_car/model/key_low_merge.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module key_low_merge (
  input wire key_up_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_down_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_left_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_right_n,  // 低有效按键输入/输出；0 表示按下。
  input wire key_mid_n,  // 低有效按键输入/输出；0 表示按下。
  input wire bt_key_up_n,  // 蓝牙方向字符模拟的低有效上键电平；字符到达后保持约 30 ms。
  input wire bt_key_down_n,  // 蓝牙方向字符模拟的低有效下键电平；字符到达后保持约 30 ms。
  input wire bt_key_left_n,  // 蓝牙方向字符模拟的低有效左键电平；字符到达后保持约 30 ms。
  input wire bt_key_right_n,  // 蓝牙方向字符模拟的低有效右键电平；字符到达后保持约 30 ms。
  input wire bt_key_mid_n,  // 蓝牙确认/停车字符模拟的低有效中键电平；字符到达后保持约 30 ms。
  output wire merged_key_up_n,  // 实体按键与蓝牙模拟按键按位 AND 后的“上键/前进”低有效电平。
  output wire merged_key_down_n,  // 实体按键与蓝牙模拟按键按位 AND 后的“下键/后退”低有效电平。
  output wire merged_key_left_n,  // 实体按键与蓝牙模拟按键按位 AND 后的“左”低有效电平。
  output wire merged_key_right_n,  // 实体按键与蓝牙模拟按键按位 AND 后的“右”低有效电平。
  output wire merged_key_mid_n  // 实体按键与蓝牙模拟按键按位 AND 后的“中键/确认”低有效电平。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“merged_key_up_n”：实体按键与蓝牙模拟按键按位 AND 后的“上键/前进”低有效电平。 该组合结果直接由 `key_up_n`（KEY / 上键/前进 / N）、`bt_key_up_n`（BT / KEY / 上键/前进 / N）的当前值决定，不引入时钟周期延迟。
  assign merged_key_up_n = key_up_n & bt_key_up_n;
  // [ASSIGN] 组合定义“merged_key_down_n”：实体按键与蓝牙模拟按键按位 AND 后的“下键/后退”低有效电平。 该组合结果直接由 `key_down_n`（KEY / 下键/后退 / N）、`bt_key_down_n`（BT / KEY / 下键/后退 / N）的当前值决定，不引入时钟周期延迟。
  assign merged_key_down_n = key_down_n & bt_key_down_n;
  // [ASSIGN] 组合定义“merged_key_left_n”：实体按键与蓝牙模拟按键按位 AND 后的“左”低有效电平。 该组合结果直接由 `key_left_n`（KEY / 左 / N）、`bt_key_left_n`（BT / KEY / 左 / N）的当前值决定，不引入时钟周期延迟。
  assign merged_key_left_n = key_left_n & bt_key_left_n;
  // [ASSIGN] 组合定义“merged_key_right_n”：实体按键与蓝牙模拟按键按位 AND 后的“右”低有效电平。 该组合结果直接由 `key_right_n`（KEY / 右 / N）、`bt_key_right_n`（BT / KEY / 右 / N）的当前值决定，不引入时钟周期延迟。
  assign merged_key_right_n = key_right_n & bt_key_right_n;
  // [ASSIGN] 组合定义“merged_key_mid_n”：实体按键与蓝牙模拟按键按位 AND 后的“中键/确认”低有效电平。 该组合结果直接由 `key_mid_n`（KEY / 中键/确认 / N）、`bt_key_mid_n`（BT / KEY / 中键/确认 / N）的当前值决定，不引入时钟周期延迟。
  assign merged_key_mid_n = key_mid_n & bt_key_mid_n;

endmodule

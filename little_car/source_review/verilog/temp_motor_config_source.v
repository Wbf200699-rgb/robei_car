// =============================================================================
// 模块: temp_motor_config_source
// 作用: 旧 TB6612 链固定参数源。
// 说明: 不在当前 C5 主执行链。
//
// Robei 源: little_car/model/temp_motor_config_source.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module temp_motor_config_source (
  output wire motor_enable,  // C5 运动链总使能。
  output wire [15:0] motor_forward_ticks,  // 旧底盘测试配置输出的“电机 / 前进”编码器目标脉冲数。
  output wire [15:0] motor_turn_ticks,  // 旧底盘测试配置输出的“电机 / 转向”编码器目标脉冲数。
  output wire [7:0] motor_forward_duty,  // 旧底盘测试配置输出的“电机 / 前进”8 bit PWM 占空比。
  output wire [7:0] motor_turn_duty  // 旧底盘测试配置输出的“电机 / 转向”8 bit PWM 占空比。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“motor_enable”：C5 运动链总使能。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign motor_enable = 1'b1;
  // [ASSIGN] 组合定义“motor_forward_ticks”：旧底盘测试配置输出的“电机 / 前进”编码器目标脉冲数。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign motor_forward_ticks = 16'd2496;
  // [ASSIGN] 组合定义“motor_turn_ticks”：旧底盘测试配置输出的“电机 / 转向”编码器目标脉冲数。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign motor_turn_ticks = 16'd360;
  // [ASSIGN] 组合定义“motor_forward_duty”：旧底盘测试配置输出的“电机 / 前进”8 bit PWM 占空比。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign motor_forward_duty = 8'd120;
  // [ASSIGN] 组合定义“motor_turn_duty”：旧底盘测试配置输出的“电机 / 转向”8 bit PWM 占空比。 右侧是固定常量，因此该网络不会随运行状态变化。
  assign motor_turn_duty = 8'd100;

endmodule

// =============================================================================
// 模块: gpio_key_adapter
// 作用: 物理按键极性适配器，把板级按键转换为统一的低有效方向/确认按键。
// 说明: key_active_low 决定输入是否反相，后级始终按低有效 key_*_n 解释。
//
// Robei 源: little_car/model/gpio_key_adapter.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module gpio_key_adapter (
  input wire key_pin1,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire key_pin2,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire key_pin3,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire key_pin4,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire key_pin5,  // 板级五向按键物理引脚；具体方向由 gpio_key_adapter 的端口连接决定。
  input wire key_active_low,  // 板级按键极性配置；1 表示物理引脚按下时为低电平。
  output wire key_up_n,  // 低有效按键输入/输出；0 表示按下。
  output wire key_down_n,  // 低有效按键输入/输出；0 表示按下。
  output wire key_left_n,  // 低有效按键输入/输出；0 表示按下。
  output wire key_right_n,  // 低有效按键输入/输出；0 表示按下。
  output wire key_mid_n  // 低有效按键输入/输出；0 表示按下。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  wire sw1_n;  // 按 key_active_low 归一化后的物理 key_pin1 低有效电平，后续映射为上键。
  wire sw2_n;  // 按 key_active_low 归一化后的物理 key_pin2 低有效电平，后续映射为左键。
  wire sw3_n;  // 按 key_active_low 归一化后的物理 key_pin3 低有效电平，后续映射为右键。
  wire sw4_n;  // 按 key_active_low 归一化后的物理 key_pin4 低有效电平，后续映射为下键。
  wire sw5_n;  // 按 key_active_low 归一化后的物理 key_pin5 低有效电平，后续映射为中键。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“sw1_n”：按 key_active_low 归一化后的物理 key_pin1 低有效电平，后续映射为上键。 该组合结果直接由 `key_active_low`（KEY / 当前有效 / LOW）、`key_pin1`（KEY / pin1）的当前值决定，不引入时钟周期延迟。
  assign sw1_n = key_active_low ? key_pin1 : ~key_pin1;
  // [ASSIGN] 组合定义“sw2_n”：按 key_active_low 归一化后的物理 key_pin2 低有效电平，后续映射为左键。 该组合结果直接由 `key_active_low`（KEY / 当前有效 / LOW）、`key_pin2`（KEY / pin2）的当前值决定，不引入时钟周期延迟。
  assign sw2_n = key_active_low ? key_pin2 : ~key_pin2;
  // [ASSIGN] 组合定义“sw3_n”：按 key_active_low 归一化后的物理 key_pin3 低有效电平，后续映射为右键。 该组合结果直接由 `key_active_low`（KEY / 当前有效 / LOW）、`key_pin3`（KEY / pin3）的当前值决定，不引入时钟周期延迟。
  assign sw3_n = key_active_low ? key_pin3 : ~key_pin3;
  // [ASSIGN] 组合定义“sw4_n”：按 key_active_low 归一化后的物理 key_pin4 低有效电平，后续映射为下键。 该组合结果直接由 `key_active_low`（KEY / 当前有效 / LOW）、`key_pin4`（KEY / pin4）的当前值决定，不引入时钟周期延迟。
  assign sw4_n = key_active_low ? key_pin4 : ~key_pin4;
  // [ASSIGN] 组合定义“sw5_n”：按 key_active_low 归一化后的物理 key_pin5 低有效电平，后续映射为中键。 该组合结果直接由 `key_active_low`（KEY / 当前有效 / LOW）、`key_pin5`（KEY / pin5）的当前值决定，不引入时钟周期延迟。
  assign sw5_n = key_active_low ? key_pin5 : ~key_pin5;

  // [ASSIGN] 组合定义“key_up_n”：低有效按键输入/输出；0 表示按下。 该组合结果直接由 `sw1_n`（SW1 / N）的当前值决定，不引入时钟周期延迟。
  assign key_up_n    = sw1_n;
  // [ASSIGN] 组合定义“key_left_n”：低有效按键输入/输出；0 表示按下。 该组合结果直接由 `sw2_n`（SW2 / N）的当前值决定，不引入时钟周期延迟。
  assign key_left_n  = sw2_n;
  // [ASSIGN] 组合定义“key_right_n”：低有效按键输入/输出；0 表示按下。 该组合结果直接由 `sw3_n`（SW3 / N）的当前值决定，不引入时钟周期延迟。
  assign key_right_n = sw3_n;
  // [ASSIGN] 组合定义“key_down_n”：低有效按键输入/输出；0 表示按下。 该组合结果直接由 `sw4_n`（SW4 / N）的当前值决定，不引入时钟周期延迟。
  assign key_down_n  = sw4_n;
  // [ASSIGN] 组合定义“key_mid_n”：低有效按键输入/输出；0 表示按下。 该组合结果直接由 `sw5_n`（SW5 / N）的当前值决定，不引入时钟周期延迟。
  assign key_mid_n   = sw5_n;

endmodule

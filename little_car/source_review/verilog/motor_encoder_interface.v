// =============================================================================
// 模块: motor_encoder_interface
// 作用: 旧底盘双路编码器计数与方向接口。
// 说明: C5 串口底盘当前不使用此模块。
//
// Robei 源: little_car/model/motor_encoder_interface.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module motor_encoder_interface (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire lf_enc_a,  // 旧底盘 左前轮编码器 A 相输入，用于边沿计数和方向判断。
  input wire lf_enc_b,  // 旧底盘 左前轮编码器 B 相输入，用于边沿计数和方向判断。
  input wire rf_enc_a,  // 旧底盘 右前轮编码器 A 相输入，用于边沿计数和方向判断。
  input wire rf_enc_b,  // 旧底盘 右前轮编码器 B 相输入，用于边沿计数和方向判断。
  output reg lf_tick,  // 旧底盘 左前轮检测到编码器边沿时产生的单周期计数脉冲。
  output reg rf_tick,  // 旧底盘 右前轮检测到编码器边沿时产生的单周期计数脉冲。
  output reg lr_tick,  // 旧底盘 左后轮检测到编码器边沿时产生的单周期计数脉冲。
  output reg rr_tick,  // 旧底盘 右后轮检测到编码器边沿时产生的单周期计数脉冲。
  output reg lf_dir,  // 旧底盘 左前轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  output reg rf_dir,  // 旧底盘 右前轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  input wire lr_enc_a,  // 旧底盘 左后轮编码器 A 相输入，用于边沿计数和方向判断。
  input wire lr_enc_b,  // 旧底盘 左后轮编码器 B 相输入，用于边沿计数和方向判断。
  input wire rr_enc_a,  // 旧底盘 右后轮编码器 A 相输入，用于边沿计数和方向判断。
  input wire rr_enc_b,  // 旧底盘 右后轮编码器 B 相输入，用于边沿计数和方向判断。
  output reg lr_dir,  // 旧底盘 左后轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  output reg rr_dir,  // 旧底盘 右后轮方向编码；motor_route_controller 与 TB6612 方向译码之间传递。
  output reg [15:0] lf_total_count,  // 旧底盘 左前轮累计编码器脉冲数；到 FFFF 后饱和保持。
  output reg [15:0] rf_total_count,  // 旧底盘 右前轮累计编码器脉冲数；到 FFFF 后饱和保持。
  output reg [15:0] lr_total_count,  // 旧底盘 左后轮累计编码器脉冲数；到 FFFF 后饱和保持。
  output reg [15:0] rr_total_count  // 旧底盘 右后轮累计编码器脉冲数；到 FFFF 后饱和保持。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg lf_a_d0;  // 旧底盘 左前轮编码器 A 相的第一级同步采样。
  reg lf_a_d1;  // 旧底盘 左前轮编码器 A 相的第二级/上一周期采样。
  reg lf_b_d0;  // 旧底盘 左前轮编码器 B 相的第一级同步采样。
  reg lf_b_d1;  // 旧底盘 左前轮编码器 B 相的第二级/上一周期采样。
  reg rf_a_d0;  // 旧底盘 右前轮编码器 A 相的第一级同步采样。
  reg rf_a_d1;  // 旧底盘 右前轮编码器 A 相的第二级/上一周期采样。
  reg rf_b_d0;  // 旧底盘 右前轮编码器 B 相的第一级同步采样。
  reg rf_b_d1;  // 旧底盘 右前轮编码器 B 相的第二级/上一周期采样。
  reg lr_a_d0;  // 旧底盘 左后轮编码器 A 相的第一级同步采样。
  reg lr_a_d1;  // 旧底盘 左后轮编码器 A 相的第二级/上一周期采样。
  reg lr_b_d0;  // 旧底盘 左后轮编码器 B 相的第一级同步采样。
  reg lr_b_d1;  // 旧底盘 左后轮编码器 B 相的第二级/上一周期采样。
  reg rr_a_d0;  // 旧底盘 右后轮编码器 A 相的第一级同步采样。
  reg rr_a_d1;  // 旧底盘 右后轮编码器 A 相的第二级/上一周期采样。
  reg rr_b_d0;  // 旧底盘 右后轮编码器 B 相的第一级同步采样。
  reg rr_b_d1;  // 旧底盘 右后轮编码器 B 相的第二级/上一周期采样。

  wire lf_a_rise;  // 旧底盘 左前轮编码器 A 相同步值的单周期上升沿脉冲。
  wire rf_a_rise;  // 旧底盘 右前轮编码器 A 相同步值的单周期上升沿脉冲。
  wire lr_a_rise;  // 旧底盘 左后轮编码器 A 相同步值的单周期上升沿脉冲。
  wire rr_a_rise;  // 旧底盘 右后轮编码器 A 相同步值的单周期上升沿脉冲。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“lf_a_rise”：旧底盘 左前轮编码器 A 相同步值的单周期上升沿脉冲。 该组合结果直接由 `lf_a_d0`（左前轮 / A / D0）、`lf_a_d1`（左前轮 / A / D1）的当前值决定，不引入时钟周期延迟。
  assign lf_a_rise = lf_a_d0 & ~lf_a_d1;
  // [ASSIGN] 组合定义“rf_a_rise”：旧底盘 右前轮编码器 A 相同步值的单周期上升沿脉冲。 该组合结果直接由 `rf_a_d0`（右前轮 / A / D0）、`rf_a_d1`（右前轮 / A / D1）的当前值决定，不引入时钟周期延迟。
  assign rf_a_rise = rf_a_d0 & ~rf_a_d1;
  // [ASSIGN] 组合定义“lr_a_rise”：旧底盘 左后轮编码器 A 相同步值的单周期上升沿脉冲。 该组合结果直接由 `lr_a_d0`（左后轮 / A / D0）、`lr_a_d1`（左后轮 / A / D1）的当前值决定，不引入时钟周期延迟。
  assign lr_a_rise = lr_a_d0 & ~lr_a_d1;
  // [ASSIGN] 组合定义“rr_a_rise”：旧底盘 右后轮编码器 A 相同步值的单周期上升沿脉冲。 该组合结果直接由 `rr_a_d0`（右后轮 / A / D0）、`rr_a_d1`（右后轮 / A / D1）的当前值决定，不引入时钟周期延迟。
  assign rr_a_rise = rr_a_d0 & ~rr_a_d1;

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 时序过程：在有效时钟边沿更新 lf_a_d0（左前轮 / A / D0）、lf_a_d1（左前轮 / A / D1）、lf_b_d0（左前轮 / B / D0）、lf_b_d1（左前轮 / B / D1）、rf_a_d0（右前轮 / A / D0）、rf_a_d1（右前轮 / A / D1）等信号；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lf_a_d0 <= 1'b0;
      lf_a_d1 <= 1'b0;
      lf_b_d0 <= 1'b0;
      lf_b_d1 <= 1'b0;
      rf_a_d0 <= 1'b0;
      rf_a_d1 <= 1'b0;
      rf_b_d0 <= 1'b0;
      rf_b_d1 <= 1'b0;
      lr_a_d0 <= 1'b0;
      lr_a_d1 <= 1'b0;
      lr_b_d0 <= 1'b0;
      lr_b_d1 <= 1'b0;
      rr_a_d0 <= 1'b0;
      rr_a_d1 <= 1'b0;
      rr_b_d0 <= 1'b0;
      rr_b_d1 <= 1'b0;
      lf_tick <= 1'b0;
      rf_tick <= 1'b0;
      lr_tick <= 1'b0;
      rr_tick <= 1'b0;
      lf_dir <= 1'b0;
      rf_dir <= 1'b0;
      lr_dir <= 1'b0;
      rr_dir <= 1'b0;
      lf_total_count <= 16'd0;
      rf_total_count <= 16'd0;
      lr_total_count <= 16'd0;
      rr_total_count <= 16'd0;
    end else begin
      lf_a_d0 <= lf_enc_a;
      lf_a_d1 <= lf_a_d0;
      lf_b_d0 <= lf_enc_b;
      lf_b_d1 <= lf_b_d0;
      rf_a_d0 <= rf_enc_a;
      rf_a_d1 <= rf_a_d0;
      rf_b_d0 <= rf_enc_b;
      rf_b_d1 <= rf_b_d0;
      lr_a_d0 <= lr_enc_a;
      lr_a_d1 <= lr_a_d0;
      lr_b_d0 <= lr_enc_b;
      lr_b_d1 <= lr_b_d0;
      rr_a_d0 <= rr_enc_a;
      rr_a_d1 <= rr_a_d0;
      rr_b_d0 <= rr_enc_b;
      rr_b_d1 <= rr_b_d0;

      lf_tick <= lf_a_rise;
      rf_tick <= rf_a_rise;
      lr_tick <= lr_a_rise;
      rr_tick <= rr_a_rise;

      if (lf_a_rise) begin
        lf_dir <= lf_b_d1;
        if (lf_total_count != 16'hFFFF)
          lf_total_count <= lf_total_count + 16'd1;
      end

      if (rf_a_rise) begin
        rf_dir <= rf_b_d1;
        if (rf_total_count != 16'hFFFF)
          rf_total_count <= rf_total_count + 16'd1;
      end

      if (lr_a_rise) begin
        lr_dir <= lr_b_d1;
        if (lr_total_count != 16'hFFFF)
          lr_total_count <= lr_total_count + 16'd1;
      end

      if (rr_a_rise) begin
        rr_dir <= rr_b_d1;
        if (rr_total_count != 16'hFFFF)
          rr_total_count <= rr_total_count + 16'd1;
      end
    end
  end

endmodule

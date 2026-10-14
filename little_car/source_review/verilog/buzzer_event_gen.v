// =============================================================================
// 模块: buzzer_event_gen
// 作用: 从复位、路线编辑和运行状态边沿产生蜂鸣事件脉冲。
// 说明: 输出事件进入 buzzer_ctrl 的优先级音效状态机。
//
// Robei 源: little_car/model/buzzer_event_gen.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module buzzer_event_gen (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire [4:0] state_info,  // 路线 FSM 状态编码。
  input wire [5:0] active_route_steps,  // 当前待执行内部命令数量。
  output wire power_on_pulse,  // “power / ON”事件脉冲；条件成立时仅保持一个 clk 周期。
  output wire route_add_pulse,  // “路线 / ADD”事件脉冲；条件成立时仅保持一个 clk 周期。
  output wire run_enter_pulse,  // “RUN / enter”事件脉冲；条件成立时仅保持一个 clk 周期。
  output wire stop_enter_pulse  // “停车 / enter”事件脉冲；条件成立时仅保持一个 clk 周期。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 常量、状态编码与时序参数
  // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
  // -----------------------------------------------------------------------------
  localparam [4:0] STATE_RUN  = 5'b00011;  // 路线/业务状态“STATE / RUN”的固定编码；case 分支和状态上报必须保持一致。
  localparam [4:0] STATE_STOP = 5'b00100;  // 路线/业务状态“STATE / STOP”的固定编码；case 分支和状态上报必须保持一致。

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg rst_n_d;  // “RST / N”的一拍延迟值；用于检测上升沿、下降沿或状态变化。
  reg [4:0] state_info_d;  // “状态 / 信息”的一拍延迟值；用于检测上升沿、下降沿或状态变化。
  reg [5:0] active_route_steps_d;  // “当前有效 / 路线 / 步数”的一拍延迟值；用于检测上升沿、下降沿或状态变化。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“power_on_pulse”：“power / ON”事件脉冲；条件成立时仅保持一个 clk 周期。 该组合结果直接由 `rst_n`（RST / N）、`rst_n_d`（RST / N / D）的当前值决定，不引入时钟周期延迟。
  assign power_on_pulse = rst_n & ~rst_n_d;
  // [ASSIGN] 组合定义“route_add_pulse”：“路线 / ADD”事件脉冲；条件成立时仅保持一个 clk 周期。 该组合结果直接由 `active_route_steps`（当前有效 / 路线 / 步数）、`active_route_steps_d`（当前有效 / 路线 / 步数 / D）的当前值决定，不引入时钟周期延迟。
  assign route_add_pulse = (active_route_steps > active_route_steps_d);
  // [ASSIGN] 组合定义“run_enter_pulse”：“RUN / enter”事件脉冲；条件成立时仅保持一个 clk 周期。 该组合结果直接由 `state_info`（状态 / 信息）、`STATE_RUN`（STATE / RUN）、`state_info_d`（状态 / 信息 / D）的当前值决定，不引入时钟周期延迟。
  assign run_enter_pulse = (state_info == STATE_RUN) && (state_info_d != STATE_RUN);
  // [ASSIGN] 组合定义“stop_enter_pulse”：“停车 / enter”事件脉冲；条件成立时仅保持一个 clk 周期。 该组合结果直接由 `state_info`（状态 / 信息）、`STATE_STOP`（STATE / STOP）、`state_info_d`（状态 / 信息 / D）的当前值决定，不引入时钟周期延迟。
  assign stop_enter_pulse = (state_info == STATE_STOP) && (state_info_d != STATE_STOP);

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 时序过程：在有效时钟边沿更新 rst_n_d（RST / N / D）、state_info_d（状态 / 信息 / D）、active_route_steps_d（当前有效 / 路线 / 步数 / D）；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rst_n_d <= 1'b0;
      state_info_d <= 5'd0;
      active_route_steps_d <= 6'd0;
    end else begin
      rst_n_d <= 1'b1;
      state_info_d <= state_info;
      active_route_steps_d <= active_route_steps;
    end
  end

endmodule

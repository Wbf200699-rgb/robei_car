// =============================================================================
// 模块: buzzer_ctrl
// 作用: 蜂鸣器音调/节奏发生器。
// 说明: 根据事件选择提示音序列，通过计数分频生成 buzzer_out。
//
// Robei 源: little_car/model/buzzer_ctrl.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module buzzer_ctrl (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire power_on_pulse,  // “power / ON”事件脉冲；条件成立时仅保持一个 clk 周期。
  input wire route_add_pulse,  // “路线 / ADD”事件脉冲；条件成立时仅保持一个 clk 周期。
  input wire run_enter_pulse,  // “RUN / enter”事件脉冲；条件成立时仅保持一个 clk 周期。
  input wire stop_enter_pulse,  // “停车 / enter”事件脉冲；条件成立时仅保持一个 clk 周期。
  output wire buzzer_out  // 送往有源/无源蜂鸣器的方波输出。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 常量、状态编码与时序参数
  // 这些值决定协议编码、计数周期和状态机含义；修改后必须重新仿真。
  // -----------------------------------------------------------------------------
  localparam integer CLK_HZ = 50000000;  // 模块假定的系统时钟频率，单位 Hz；所有毫秒和波特率计数均由它换算。
  localparam integer TONE_HZ = 2000;  // 蜂鸣器提示音基频 2000 Hz；HALF_PERIOD 根据它计算方波翻转间隔。
  localparam integer BEEP_MS = 80;  // 每一声蜂鸣保持有效的时间，单位 ms；当前值 80 ms。
  localparam integer GAP_MS = 80;  // 多声提示中相邻两声之间的静音间隔，单位 ms；当前值 80 ms。
  localparam ACTIVE_LOW = 1'b0;  // 蜂鸣器输出极性配置：1 对 raw_buzzer 取反，0 保持原极性。

  localparam integer HALF_PERIOD = CLK_HZ / (TONE_HZ * 2);  // 蜂鸣器音调半周期对应的系统时钟数；计满一次翻转 tone_bit。
  localparam integer BEEP_CYCLES = (CLK_HZ / 1000) * BEEP_MS;  // 单声蜂鸣 80 ms 对应的 50 MHz 周期数；beep_cnt 达到该值后进入静音或结束提示。
  localparam integer GAP_CYCLES  = (CLK_HZ / 1000) * GAP_MS;  // 蜂鸣间隔 80 ms 对应的 50 MHz 周期数；gap_cnt 达到该值后开始下一声。

  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg tone_bit;  // 按 TONE_HZ 分频翻转的音调方波状态。
  reg beep_active;  // 当前处于发声时间窗。
  reg gap_active;  // 多声提示之间的静音间隔时间窗。
  reg [31:0] tone_cnt;  // 半个音调周期计数器，计满时翻转 tone_bit。
  reg [31:0] beep_cnt;  // 单次蜂鸣持续时间计数器。
  reg [31:0] gap_cnt;  // 两次蜂鸣之间静音时间计数器。
  reg [1:0] pending_beeps;  // 新事件请求但尚未开始执行的蜂鸣次数。
  reg [1:0] current_beeps;  // 当前提示序列仍需完成的蜂鸣次数。
  wire raw_buzzer;  // 蜂鸣器节奏门控后的内部方波，尚未根据 ACTIVE_LOW 调整极性。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“raw_buzzer”：蜂鸣器节奏门控后的内部方波，尚未根据 ACTIVE_LOW 调整极性。 该组合结果直接由 `beep_active`（beep / 当前有效）、`tone_bit`（tone / BIT）的当前值决定，不引入时钟周期延迟。
  assign raw_buzzer = beep_active ? tone_bit : 1'b0;
  // [ASSIGN] 组合定义“buzzer_out”：送往有源/无源蜂鸣器的方波输出。 该组合结果直接由 `ACTIVE_LOW`（ACTIVE / LOW）、`raw_buzzer`（原始值 / buzzer）的当前值决定，不引入时钟周期延迟。
  assign buzzer_out = ACTIVE_LOW ? ~raw_buzzer : raw_buzzer;

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 时序过程：在有效时钟边沿更新 tone_bit（tone / BIT）、beep_active（beep / 当前有效）、gap_active（GAP / 当前有效）、tone_cnt（tone / 计数）、beep_cnt（beep / 计数）、gap_cnt（GAP / 计数）等信号；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tone_bit <= 1'b0;
      beep_active <= 1'b0;
      gap_active <= 1'b0;
      tone_cnt <= 32'd0;
      beep_cnt <= 32'd0;
      gap_cnt <= 32'd0;
      pending_beeps <= 2'd0;
      current_beeps <= 2'd0;
    end else begin
      if (stop_enter_pulse)
        pending_beeps <= 2'd2;
      else if (run_enter_pulse)
        pending_beeps <= 2'd1;
      else if (route_add_pulse)
        pending_beeps <= 2'd1;
      else if (power_on_pulse)
        pending_beeps <= 2'd1;

      if (beep_active) begin
        if (tone_cnt == HALF_PERIOD - 1) begin
          tone_cnt <= 32'd0;
          tone_bit <= ~tone_bit;
        end else begin
          tone_cnt <= tone_cnt + 32'd1;
        end

        if (beep_cnt == BEEP_CYCLES - 1) begin
          beep_active <= 1'b0;
          beep_cnt <= 32'd0;
          tone_cnt <= 32'd0;
          tone_bit <= 1'b0;
          if (current_beeps > 2'd1) begin
            current_beeps <= current_beeps - 2'd1;
            gap_active <= 1'b1;
            gap_cnt <= 32'd0;
          end else begin
            current_beeps <= 2'd0;
          end
        end else begin
          beep_cnt <= beep_cnt + 32'd1;
        end
      end else if (gap_active) begin
        if (gap_cnt == GAP_CYCLES - 1) begin
          gap_active <= 1'b0;
          gap_cnt <= 32'd0;
          beep_active <= 1'b1;
          beep_cnt <= 32'd0;
          tone_cnt <= 32'd0;
          tone_bit <= 1'b0;
        end else begin
          gap_cnt <= gap_cnt + 32'd1;
        end
      end else if (current_beeps != 2'd0) begin
        beep_active <= 1'b1;
        beep_cnt <= 32'd0;
        tone_cnt <= 32'd0;
        tone_bit <= 1'b0;
      end else if (pending_beeps != 2'd0) begin
        current_beeps <= pending_beeps;
        pending_beeps <= 2'd0;
      end
    end
  end

endmodule

// =============================================================================
// 模块: key_pulse_filter
// 作用: 按键同步、消抖和单脉冲生成器。
// 说明: 只有输入持续稳定达到计数阈值后才改变状态；每次按下只产生一个时钟脉冲。
//
// Robei 源: little_car/model/key_pulse_filter.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module key_pulse_filter (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  input wire key_n,  // 低有效按键输入/输出；0 表示按下。
  output reg key_pulse  // “KEY”事件脉冲；条件成立时仅保持一个 clk 周期。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
  // -----------------------------------------------------------------------------
  // 内部网络、寄存器和计数器
  // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
  // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
  // -----------------------------------------------------------------------------
  reg [1:0] key_sync;  // 异步按键输入的同步结果。
  reg key_level;  // 经过连续 1,000,000 个 50 MHz 周期确认后的稳定按下状态；1 表示按下。
  reg key_level_d;  // key_level 的上一周期值，用于只在稳定状态 0->1 时输出一次 key_pulse。
  reg [19:0] stable_cnt;  // 原始同步按键与当前 key_level 不一致时的消抖持续计数；达到约 20 ms 才接受变化。

  wire key_pressed;  // 两级同步后的低有效物理按键即时按下判定；key_sync[1]=0 时为 1，尚未经过 20 ms 消抖。

  // -----------------------------------------------------------------------------
  // 组合派生信号
  // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
  // -----------------------------------------------------------------------------
  // [ASSIGN] 组合定义“key_pressed”：两级同步后的低有效物理按键即时按下判定；key_sync[1]=0 时为 1，尚未经过 20 ms 消抖。 该组合结果直接由 `key_sync`（KEY / 同步值）的当前值决定，不引入时钟周期延迟。
  assign key_pressed = ~key_sync[1];

  // -----------------------------------------------------------------------------
  // [PROCESS] 过程块 1
  // 时序过程：在有效时钟边沿更新 key_sync（KEY / 同步值）、key_level（KEY / level）、key_level_d（KEY / level / D）、stable_cnt（stable / 计数）、key_pulse（KEY / 脉冲）；复位分支先建立确定的安全初值，其余分支按当前输入和旧状态推进。
  // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
  // -----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      key_sync    <= 2'b11;
      key_level   <= 1'b0;
      key_level_d <= 1'b0;
      stable_cnt  <= 20'd0;
      key_pulse   <= 1'b0;
    end else begin
      key_sync <= {key_sync[0], key_n};

      if (key_pressed == key_level) begin
        stable_cnt <= 20'd0;
      end else if (stable_cnt >= 20'd999999) begin
        key_level  <= key_pressed;
        stable_cnt <= 20'd0;
      end else begin
        stable_cnt <= stable_cnt + 20'd1;
      end

      key_level_d <= key_level;
      key_pulse   <= key_level & ~key_level_d;
    end
  end

endmodule

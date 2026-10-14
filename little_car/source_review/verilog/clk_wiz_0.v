// =============================================================================
// 模块: clk_wiz_0
// 作用: 使用 7 系列 MMCM/BUFG 原语产生 HDMI 像素与 5 倍串化时钟。
// 说明: 50 MHz 输入经 VCO 后产生 50 MHz clk_out1 和 250 MHz clk_out2。
// 说明: MMCM LOCKED 后额外等待 50,000 个像素周期，降低复位后 HDMI 偶发失锁风险。
// 说明: 这是手写原语封装，不是 Vivado IP 核。
//
// Robei 源: little_car/model/clk_wiz_0.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module clk_wiz_0 (
  input wire clk_in1,  // 板载 50 MHz 时钟，经 IBUF 后送入 MMCME2_ADV。
  input wire resetn,  // MMCM 封装低有效复位；0 立即复位锁定与输出时钟。
  output wire clk_out1,  // MMCM/BUFG 产生的 50 MHz HDMI 像素时钟。
  output wire clk_out2,  // MMCM/BUFG 产生的 250 MHz TMDS 5 倍串化时钟。
  output wire locked  // MMCM 原始锁定后再稳定等待 50,000 个像素周期的可靠时钟就绪标志。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
    `ifdef SYNTHESIS
      // -----------------------------------------------------------------------------
      // 内部网络、寄存器和计数器
      // wire 只承载组合连接；reg 是过程赋值变量，是否保存跨周期状态由其所在时钟/组合过程决定。
      // 每条声明后的注释继续说明具体数据含义、单位、编码和更新用途。
      // -----------------------------------------------------------------------------
      wire clkfb_mmcm;  // MMCME2_ADV 的未缓冲反馈输出，送入反馈 BUFG。
      wire clkfb_bufg;  // 反馈 BUFG 输出，回接 MMCME2_ADV CLKFBIN 以闭合补偿环路。
      wire clk_pix_mmcm;  // MMCME2_ADV CLKOUT0 产生的未缓冲 50 MHz 像素时钟。
      wire clk_5x_mmcm;  // MMCME2_ADV CLKOUT1 产生的未缓冲 250 MHz 串化时钟。
      wire mmcm_locked;  // MMCME2_ADV 原始 LOCKED 输出；稳定延时计数从它拉高时开始。
      reg [15:0] lock_count = 16'd0;  // MMCM 原始 locked 拉高后继续等待 50,000 个 50 MHz 周期的稳定计数器。
      reg stable_locked = 1'b0;  // 综合硬件最终时钟就绪标志；只有 MMCM 锁定并完成额外等待后才置 1。

      MMCME2_ADV #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKIN1_PERIOD(20.000),
        .CLKIN2_PERIOD(0.000),
        .CLKFBOUT_MULT_F(20.000),
        .CLKFBOUT_PHASE(0.000),
        .DIVCLK_DIVIDE(1),
        .CLKOUT0_DIVIDE_F(20.000),
        .CLKOUT0_PHASE(0.000),
        .CLKOUT0_DUTY_CYCLE(0.500),
        .CLKOUT1_DIVIDE(4),
        .CLKOUT1_PHASE(0.000),
        .CLKOUT1_DUTY_CYCLE(0.500),
        .CLKOUT2_DIVIDE(16),
        .CLKOUT2_PHASE(0.000),
        .CLKOUT2_DUTY_CYCLE(0.500),
        .CLKOUT3_DIVIDE(16),
        .CLKOUT3_PHASE(0.000),
        .CLKOUT3_DUTY_CYCLE(0.500),
        .CLKOUT4_DIVIDE(16),
        .CLKOUT4_PHASE(0.000),
        .CLKOUT4_DUTY_CYCLE(0.500),
        .CLKOUT5_DIVIDE(16),
        .CLKOUT5_PHASE(0.000),
        .CLKOUT5_DUTY_CYCLE(0.500),
        .CLKOUT6_DIVIDE(16),
        .CLKOUT6_PHASE(0.000),
        .CLKOUT6_DUTY_CYCLE(0.500),
        .COMPENSATION("ZHOLD"),
        .REF_JITTER1(0.010),
        .STARTUP_WAIT("FALSE")
      ) mmcm_inst (
        .CLKIN1(clk_in1),
        .CLKIN2(1'b0),
        .CLKINSEL(1'b1),
        .RST(1'b0),
        .PWRDWN(1'b0),
        .CLKFBIN(clkfb_bufg),
        .CLKFBOUT(clkfb_mmcm),
        .CLKFBOUTB(),
          .CLKOUT0(clk_pix_mmcm),
          .CLKOUT0B(),
          .CLKOUT1(clk_5x_mmcm),
          .CLKOUT1B(),
        .CLKOUT2(),
        .CLKOUT2B(),
        .CLKOUT3(),
        .CLKOUT3B(),
        .CLKOUT4(),
        .CLKOUT5(),
        .CLKOUT6(),
        .DO(),
        .DRDY(),
        .DADDR(7'd0),
        .DCLK(1'b0),
        .DEN(1'b0),
        .DI(16'd0),
        .DWE(1'b0),
        .PSCLK(1'b0),
        .PSEN(1'b0),
        .PSINCDEC(1'b0),
        .PSDONE(),
        .LOCKED(mmcm_locked),
        .CLKINSTOPPED(),
        .CLKFBSTOPPED()
      );

      BUFG clkfb_bufg_inst (
        .I(clkfb_mmcm),
        .O(clkfb_bufg)
      );

      BUFG clk_pix_bufg_inst (
        .I(clk_pix_mmcm),
        .O(clk_out1)
      );

      BUFG clk_5x_bufg_inst (
        .I(clk_5x_mmcm),
        .O(clk_out2)
      );

      // -----------------------------------------------------------------------------
      // [PROCESS] 过程块 1
      // 综合硬件锁定延时：MMCM 锁定后再累计 50,000 个 clk_out1 周期才置 stable_locked。
      // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
      // -----------------------------------------------------------------------------
      always @(posedge clk_out1) begin
        if (!mmcm_locked) begin
          lock_count <= 16'd0;
          stable_locked <= 1'b0;
        end else if (!stable_locked) begin
          if (lock_count == 16'd49999)
            stable_locked <= 1'b1;
          else
            lock_count <= lock_count + 16'd1;
        end
      end

      // -----------------------------------------------------------------------------
      // 组合派生信号
      // 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
      // -----------------------------------------------------------------------------
      // [ASSIGN] 组合定义“locked”：MMCM 原始锁定后再稳定等待 50,000 个像素周期的可靠时钟就绪标志。 该组合结果直接由 `stable_locked`（stable / 时钟锁定）的当前值决定，不引入时钟周期延迟。
      assign locked = stable_locked;
    `else
      reg clk_50m = 1'b0;  // 仅在 `ifndef SYNTHESIS 分支使用的 50 MHz 仿真时钟变量，每 10 ns 翻转。
      reg clk_250m = 1'b0;  // 仅在 `ifndef SYNTHESIS 分支使用的 250 MHz 仿真时钟变量，每 2 ns 翻转。
      reg is_locked = 1'b0;  // 仅在 `ifndef SYNTHESIS 分支使用的仿真锁定标志，复位释放 100 ns 后置 1。

      // -----------------------------------------------------------------------------
      // [PROCESS] 过程块 2
      // 非综合仿真锁定模型：resetn 释放后延时 100 ns 置 is_locked。
      // 非阻塞赋值 <= 表示寄存器在该时钟边沿结束后统一更新。
      // -----------------------------------------------------------------------------
      always @(posedge clk_in1 or negedge resetn) begin
        if (!resetn)
          is_locked <= 1'b0;
        else
          #100 is_locked <= 1'b1;
      end

      // -----------------------------------------------------------------------------
      // [PROCESS] 过程块 3
      // 非综合仿真 50 MHz 时钟模型：每 10 ns 翻转 clk_50m。
      // -----------------------------------------------------------------------------
      always #10.000 clk_50m = ~clk_50m;
      // -----------------------------------------------------------------------------
      // [PROCESS] 过程块 4
      // 非综合仿真 250 MHz 时钟模型：每 2 ns 翻转 clk_250m。
      // -----------------------------------------------------------------------------
      always #2.000 clk_250m = ~clk_250m;

      // [ASSIGN] 组合定义“clk_out1”：MMCM/BUFG 产生的 50 MHz HDMI 像素时钟。 该组合结果直接由 `clk_50m`（时钟 / 50M）的当前值决定，不引入时钟周期延迟。
      assign clk_out1 = clk_50m;
      // [ASSIGN] 组合定义“clk_out2”：MMCM/BUFG 产生的 250 MHz TMDS 5 倍串化时钟。 该组合结果直接由 `clk_250m`（时钟 / 250m）的当前值决定，不引入时钟周期延迟。
      assign clk_out2 = clk_250m;
      // [ASSIGN] 组合定义“locked”：MMCM 原始锁定后再稳定等待 50,000 个像素周期的可靠时钟就绪标志。 该组合结果直接由 `is_locked`（判定 / 时钟锁定）的当前值决定，不引入时钟周期延迟。
      assign locked = is_locked;
    `endif

endmodule

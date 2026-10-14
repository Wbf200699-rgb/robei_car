module clk_wiz_0(
input wire clk_in1,
input wire resetn,
output wire clk_out1,
output wire clk_out2,
output wire locked
);

`ifdef SYNTHESIS
  wire clkfb_mmcm;
  wire clkfb_bufg;
  wire clk_pix_mmcm;
  wire clk_5x_mmcm;
  wire mmcm_locked;

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
    .RST(~resetn),
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

  assign locked = mmcm_locked;
`else
  reg clk_50m = 1'b0;
  reg clk_250m = 1'b0;
  reg is_locked = 1'b0;

  always @(posedge clk_in1 or negedge resetn) begin
    if (!resetn)
      is_locked <= 1'b0;
    else
      #100 is_locked <= 1'b1;
  end

  always #10.000 clk_50m = ~clk_50m;
  always #2.000 clk_250m = ~clk_250m;

  assign clk_out1 = clk_50m;
  assign clk_out2 = clk_250m;
  assign locked = is_locked;
`endif

endmodule

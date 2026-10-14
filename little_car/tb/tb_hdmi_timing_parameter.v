`timescale 1ns/1ps

module tb_hdmi_timing_parameter;
  reg clk_pix = 1'b0;
  reg rst_n = 1'b0;
  wire hsync, vsync, v_de;
  wire [11:0] pixel_x, pixel_y;
  integer errors;

  always #5 clk_pix = ~clk_pix;

  hdmi_timing_gen dut (
    .clk_pix(clk_pix), .rst_n(rst_n), .hsync(hsync), .vsync(vsync),
    .v_de(v_de), .pixel_x(pixel_x), .pixel_y(pixel_y)
  );

  task probe;
    input [11:0] h;
    input [11:0] v;
    input expected_de;
    input expected_hsync;
    input expected_vsync;
    begin
      @(negedge clk_pix);
      dut.h_cnt = h;
      dut.v_cnt = v;
      @(posedge clk_pix);
      #1;
      $display("RESULT probe h=%0d v=%0d de=%0d hs=%0d vs=%0d", h, v, v_de, hsync, vsync);
      if (v_de !== expected_de || hsync !== expected_hsync ||
          vsync !== expected_vsync)
        errors = errors + 1;
    end
  endtask

  initial begin
    errors = 0;
    repeat (3) @(posedge clk_pix);
    rst_n = 1'b1;
    probe(12'd0,    12'd0,   1, 0, 0);
    probe(12'd1023, 12'd599, 1, 0, 0);
    probe(12'd1024, 12'd0,   0, 0, 0);
    probe(12'd1063, 12'd0,   0, 0, 0);
    probe(12'd1064, 12'd0,   0, 1, 0);
    probe(12'd1163, 12'd0,   0, 1, 0);
    probe(12'd1164, 12'd0,   0, 0, 0);
    probe(12'd0,    12'd611, 0, 0, 0);
    probe(12'd0,    12'd612, 0, 0, 1);
    probe(12'd0,    12'd621, 0, 0, 1);
    probe(12'd0,    12'd622, 0, 0, 0);
    probe(12'd1323, 12'd644, 0, 0, 0);
    @(posedge clk_pix);
    #1;
    if (!v_de || dut.h_cnt != 12'd1 || dut.v_cnt != 12'd0)
      errors = errors + 1;

    $display("RESULT mode=1024x600 h_active=1024 h_fp=40 h_sync=100 h_bp=160 h_total=1324");
    $display("RESULT v_active=600 v_fp=12 v_sync=10 v_bp=23 v_total=645");
    $display("RESULT pixels_per_frame=853980 active_pixels=614400 blank_pixels=239580");
    if (errors == 0)
      $display("RESULT TEST_PASS hdmi_timing_parameter");
    else
      $display("RESULT TEST_FAIL hdmi_timing_parameter errors=%0d", errors);
    $finish;
  end

  initial begin
    #20000;
    $display("RESULT TEST_FAIL hdmi_timing_parameter timeout=1");
    $finish;
  end
endmodule

`timescale 1ns/1ps
module hdmi_timing_gen(
  input wire clk_pix,
  input wire rst_n,
  output reg hsync,
  output reg vsync,
  output reg v_de,
  output wire [11:0] pixel_x,
  output wire [11:0] pixel_y
);
  parameter H_ACTIVE = 12'd1024, H_FP = 12'd40, H_SYNC = 12'd100, H_BP = 12'd160, H_TOTAL = 12'd1324;
  parameter V_ACTIVE = 12'd600,  V_FP = 12'd12, V_SYNC = 12'd10,  V_BP = 12'd23,  V_TOTAL = 12'd645;

  reg [11:0] h_cnt, v_cnt;

  always @(posedge clk_pix or negedge rst_n) begin
      if (!rst_n) h_cnt <= 0;
      else h_cnt <= (h_cnt == H_TOTAL - 1) ? 0 : h_cnt + 1;
  end

  always @(posedge clk_pix or negedge rst_n) begin
      if (!rst_n) v_cnt <= 0;
      else if (h_cnt == H_TOTAL - 1) v_cnt <= (v_cnt == V_TOTAL - 1) ? 0 : v_cnt + 1;
  end

  always @(posedge clk_pix or negedge rst_n) begin
      if (!rst_n) begin
          hsync <= 1'b0;
          vsync <= 1'b0;
          v_de <= 1'b0;
      end else begin
          hsync <= (h_cnt >= (H_ACTIVE + H_FP)) && (h_cnt < (H_ACTIVE + H_FP + H_SYNC));
          vsync <= (v_cnt >= (V_ACTIVE + V_FP)) && (v_cnt < (V_ACTIVE + V_FP + V_SYNC));
          v_de  <= (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
      end
  end

  assign pixel_x = v_de ? h_cnt : 12'd0;
  assign pixel_y = v_de ? v_cnt : 12'd0;
endmodule

module hdmi_timing_parameter;
  reg clk_pix;
  reg rst_n;
  wire hsync;
  wire vsync;
  wire v_de;
  wire [11:0] pixel_x;
  wire [11:0] pixel_y;


`ifdef PARAM_WAVE
initial begin
  $dumpfile("tb_hdmi_timing_parameter.vcd");
  $dumpvars(0, rst_n, hsync, vsync, v_de, pixel_x, pixel_y, hdmi_timing_gen0.h_cnt, hdmi_timing_gen0.v_cnt);
end
`endif

initial begin
  clk_pix = 1'b0;
  rst_n = 1'b0;
end

integer errors;

  always #5 clk_pix = ~clk_pix;



  task probe;
    input [11:0] h;
    input [11:0] v;
    input expected_de;
    input expected_hsync;
    input expected_vsync;
    begin
      @(negedge clk_pix);
      hdmi_timing_gen0.h_cnt = h;
      hdmi_timing_gen0.v_cnt = v;
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
    if (!v_de || hdmi_timing_gen0.h_cnt != 12'd1 || hdmi_timing_gen0.v_cnt != 12'd0)
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


  hdmi_timing_gen hdmi_timing_gen0(
    .clk_pix(clk_pix),
    .rst_n(rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .v_de(v_de),
    .pixel_x(pixel_x),
    .pixel_y(pixel_y)
  );
endmodule

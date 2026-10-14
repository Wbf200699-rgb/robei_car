module hdmi_timing_gen(
input wire clk_pix,
input wire rst_n,
output reg hsync,
output reg vsync,
output reg v_de,
output wire [11:0] pixel_x,
output wire [11:0] pixel_y
);

  //----Code starts here: integrated by Robei-----
  // 1024x600 时序参数
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

  always @(posedge clk_pix) begin
      hsync <= (h_cnt >= (H_ACTIVE + H_FP)) && (h_cnt < (H_ACTIVE + H_FP + H_SYNC));
      vsync <= (v_cnt >= (V_ACTIVE + V_FP)) && (v_cnt < (V_ACTIVE + V_FP + V_SYNC));
      v_de  <= (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
  end

  assign pixel_x = v_de ? h_cnt : 12'd0;
  assign pixel_y = v_de ? v_cnt : 12'd0;
endmodule    //hdmi_timing_gen

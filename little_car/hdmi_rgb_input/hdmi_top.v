module hdmi_top(
input wire clk,
input wire rst_n,
input wire [23:0] rgb_data,
output wire [11:0] pixel_x,
output wire [11:0] pixel_y,
output wire pixel_de,
output wire pixel_clk,
output wire hdmi_clk_p,
output wire hdmi_clk_n,
output wire hdmi_d0_p,
output wire hdmi_d0_n,
output wire hdmi_d1_p,
output wire hdmi_d1_n,
output wire hdmi_d2_p,
output wire hdmi_d2_n
);

  wire clk_pix;
  wire clk_5x;
  wire clk_locked;
  wire video_rst_n;

  wire hsync;
  wire vsync;
  wire v_de;

  wire [7:0] data_r;
  wire [7:0] data_g;
  wire [7:0] data_b;

  wire [9:0] tmds_b;
  wire [9:0] tmds_g;
  wire [9:0] tmds_r;
  wire [9:0] tmds_clk_pattern;
  wire gnd_bit;

  assign video_rst_n = rst_n & clk_locked;

  assign pixel_de = v_de;
  assign pixel_clk = clk_pix;

  clk_wiz_0 clk_wiz_inst(
    .clk_in1(clk),
    .resetn(rst_n),
    .clk_out1(clk_pix),
    .clk_out2(clk_5x),
    .locked(clk_locked)
  );

  hdmi_timing_gen timing_inst(
    .clk_pix(clk_pix),
    .rst_n(video_rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .v_de(v_de),
    .pixel_x(pixel_x),
    .pixel_y(pixel_y)
  );

  hdmi_rgb_splitter rgb_splitter_inst(
    .rgb_data(rgb_data),
    .pixel_de(v_de),
    .data_r(data_r),
    .data_g(data_g),
    .data_b(data_b)
  );

  // HDMI/TMDS channel mapping:
  // D0 = Blue channel and control signals HSYNC/VSYNC
  // D1 = Green channel
  // D2 = Red channel
  hdmi_tmds_encoder encoder_b(
    .clk_pix(clk_pix),
    .rst_n(video_rst_n),
    .data_in(data_b),
    .c0(hsync),
    .c1(vsync),
    .v_de(v_de),
    .tmds_10b(tmds_b)
  );

  hdmi_tmds_encoder encoder_g(
    .clk_pix(clk_pix),
    .rst_n(video_rst_n),
    .data_in(data_g),
    .c0(gnd_bit),
    .c1(gnd_bit),
    .v_de(v_de),
    .tmds_10b(tmds_g)
  );

  hdmi_tmds_encoder encoder_r(
    .clk_pix(clk_pix),
    .rst_n(video_rst_n),
    .data_in(data_r),
    .c0(gnd_bit),
    .c1(gnd_bit),
    .v_de(v_de),
    .tmds_10b(tmds_r)
  );

  (* keep_hierarchy = "yes", dont_touch = "true", hu_set = "hdmi_serdes_b" *)
  hdmi_serializer serializer_b(
    .clk_pix(clk_pix),
    .clk_5x(clk_5x),
    .rst_n(video_rst_n),
    .data_10b(tmds_b),
    .ser_p(hdmi_d0_p),
    .ser_n(hdmi_d0_n)
  );

  (* keep_hierarchy = "yes", dont_touch = "true", hu_set = "hdmi_serdes_g" *)
  hdmi_serializer serializer_g(
    .clk_pix(clk_pix),
    .clk_5x(clk_5x),
    .rst_n(video_rst_n),
    .data_10b(tmds_g),
    .ser_p(hdmi_d1_p),
    .ser_n(hdmi_d1_n)
  );

  (* keep_hierarchy = "yes", dont_touch = "true", hu_set = "hdmi_serdes_r" *)
  hdmi_serializer serializer_r(
    .clk_pix(clk_pix),
    .clk_5x(clk_5x),
    .rst_n(video_rst_n),
    .data_10b(tmds_r),
    .ser_p(hdmi_d2_p),
    .ser_n(hdmi_d2_n)
  );

  (* keep_hierarchy = "yes", dont_touch = "true", hu_set = "hdmi_serdes_clk" *)
  hdmi_serializer serializer_clk(
    .clk_pix(clk_pix),
    .clk_5x(clk_5x),
    .rst_n(video_rst_n),
    .data_10b(tmds_clk_pattern),
    .ser_p(hdmi_clk_p),
    .ser_n(hdmi_clk_n)
  );

  hdmi_const const_inst(
    .tmds_clk_pattern(tmds_clk_pattern),
    .gnd_bit(gnd_bit)
  );

endmodule    //hdmi_top

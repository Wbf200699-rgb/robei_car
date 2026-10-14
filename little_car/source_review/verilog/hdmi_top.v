// =============================================================================
// 模块: hdmi_top
// 作用: 完整 HDMI 发送流水线的层次封装。
// 说明: 时钟 -> 1024x600 时序 -> RGB 拆分 -> 三路 TMDS 编码 -> 四路串化差分输出。
// 说明: 蓝色通道承载 hsync/vsync 控制码，红绿通道消隐期控制输入固定为 0。
//
// Robei 源: little_car/model/hdmi_top.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module hdmi_top (
  input wire clk,  // 50 MHz 系统时钟输入。
  input wire rst_n,  // 低有效复位；0 清零模块内部状态。
  output wire hdmi_clk_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d0_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d1_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d2_p,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_clk_n,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d0_n,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d1_n,  // HDMI TMDS 差分物理输出端。
  output wire hdmi_d2_n,  // HDMI TMDS 差分物理输出端。
  input wire [23:0] rgb_data,  // GUI 输出或 HDMI 输入的 RGB888 像素。
  output wire [11:0] pixel_x,  // 当前有效显示区水平像素坐标。
  output wire [11:0] pixel_y,  // 当前有效显示区垂直像素坐标。
  output wire pixel_de,  // 视频数据有效；1 表示位于可见区域。
  output wire pixel_clk  // 导出给 GUI 渲染器使用的 50 MHz HDMI 像素时钟。
);

  // ===========================================================================
  // 层次内部网络
  // 每条注释给出驱动端以及所有接收端，便于从顶层沿数据流追踪。
  // ===========================================================================
  // clk_wiz_01.clk_out1 -> hdmi_timing_gen2.clk_pix, hdmi_tmds_encoder_B.clk_pix, hdmi_serializer_B.clk_pix, hdmi_tmds_encoder_G.clk_pix, hdmi_serializer_G.clk_pix, hdmi_tmds_encoder_R.clk_pix, hdmi_serializer_R.clk_pix, hdmi_serializer_CLK.clk_pix
  wire clk_wiz_01_clk_out1;  // MMCM/BUFG 产生的 50 MHz HDMI 像素时钟。 具体驱动端和全部接收端见上一行。
  // clk_wiz_01.locked -> hdmi_timing_gen2.rst_n, hdmi_tmds_encoder_B.rst_n, hdmi_tmds_encoder_G.rst_n, hdmi_tmds_encoder_R.rst_n, hdmi_serializer_B.rst_n, hdmi_serializer_G.rst_n, hdmi_serializer_R.rst_n, hdmi_serializer_CLK.rst_n
  wire clk_wiz_01_locked;  // MMCM 原始锁定后再稳定等待 50,000 个像素周期的可靠时钟就绪标志。 具体驱动端和全部接收端见上一行。
  // hdmi_timing_gen2.v_de -> hdmi_tmds_encoder_B.v_de, hdmi_tmds_encoder_G.v_de, hdmi_tmds_encoder_R.v_de, hdmi_rgb_splitter11.pixel_de
  wire hdmi_timing_gen2_v_de;  // 当前像素位于 1024x600 可见区的有效标志。 具体驱动端和全部接收端见上一行。
  // hdmi_timing_gen2.hsync -> hdmi_tmds_encoder_B.c0
  wire hdmi_timing_gen2_hsync;  // 1024x600 模式水平同步信号；极性和区间由 H_SYNC_START/END 定义。 具体驱动端和全部接收端见上一行。
  // hdmi_timing_gen2.vsync -> hdmi_tmds_encoder_B.c1
  wire hdmi_timing_gen2_vsync;  // 1024x600 模式垂直同步信号；极性和区间由 V_SYNC_START/END 定义。 具体驱动端和全部接收端见上一行。
  // hdmi_tmds_encoder_B.tmds_10b -> hdmi_serializer_B.data_10b
  wire [9:0] hdmi_tmds_encoder_B_tmds_10b;  // 按 running disparity 平衡后的 10 bit TMDS 码字。 具体驱动端和全部接收端见上一行。
  // clk_wiz_01.clk_out2 -> hdmi_serializer_B.clk_5x, hdmi_serializer_G.clk_5x, hdmi_serializer_R.clk_5x, hdmi_serializer_CLK.clk_5x
  wire clk_wiz_01_clk_out2;  // MMCM/BUFG 产生的 250 MHz TMDS 5 倍串化时钟。 具体驱动端和全部接收端见上一行。
  // hdmi_tmds_encoder_G.tmds_10b -> hdmi_serializer_G.data_10b
  wire [9:0] hdmi_tmds_encoder_G_tmds_10b;  // 按 running disparity 平衡后的 10 bit TMDS 码字。 具体驱动端和全部接收端见上一行。
  // hdmi_tmds_encoder_R.tmds_10b -> hdmi_serializer_R.data_10b
  wire [9:0] hdmi_tmds_encoder_R_tmds_10b;  // 按 running disparity 平衡后的 10 bit TMDS 码字。 具体驱动端和全部接收端见上一行。
  // hdmi_const11.tmds_clk_pattern -> hdmi_serializer_CLK.data_10b
  wire [9:0] hdmi_const11_tmds_clk_pattern;  // TMDS 时钟通道固定 10 bit 码型 1111100000。 具体驱动端和全部接收端见上一行。
  // hdmi_const11.gnd_bit -> hdmi_tmds_encoder_G.c0, hdmi_tmds_encoder_G.c1, hdmi_tmds_encoder_R.c0, hdmi_tmds_encoder_R.c1
  wire hdmi_const11_gnd_bit;  // HDMI 辅助连接使用的固定逻辑 0。 具体驱动端和全部接收端见上一行。
  // hdmi_rgb_splitter11.data_r -> hdmi_tmds_encoder_R.data_in
  wire [7:0] hdmi_rgb_splitter11_data_r;  // pixel_de 门控后的 RGB888 红色字节；消隐区强制为 0。 具体驱动端和全部接收端见上一行。
  // hdmi_rgb_splitter11.data_g -> hdmi_tmds_encoder_G.data_in
  wire [7:0] hdmi_rgb_splitter11_data_g;  // pixel_de 门控后的 RGB888 绿色字节；消隐区强制为 0。 具体驱动端和全部接收端见上一行。
  // hdmi_rgb_splitter11.data_b -> hdmi_tmds_encoder_B.data_in
  wire [7:0] hdmi_rgb_splitter11_data_b;  // pixel_de 门控后的 RGB888 蓝色字节；消隐区强制为 0。 具体驱动端和全部接收端见上一行。

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
(* keep_hierarchy = "yes", dont_touch = "true", hu_set = "hdmi_serdes_b" *)
(* keep_hierarchy = "yes", dont_touch = "true", hu_set = "hdmi_serdes_g" *)
(* keep_hierarchy = "yes", dont_touch = "true", hu_set = "hdmi_serdes_r" *)
(* keep_hierarchy = "yes", dont_touch = "true", hu_set = "hdmi_serdes_clk" *)

  // ---------------------------------------------------------------------------
  // 子模块实例: clk_wiz_01
  // 类型: clk_wiz_0
  // 作用: 使用 7 系列 MMCM/BUFG 原语产生 HDMI 像素与 5 倍串化时钟。
  // 连接说明: 50 MHz 输入经 VCO 后产生 50 MHz clk_out1 和 250 MHz clk_out2。
  // 连接说明: MMCM LOCKED 后额外等待 50,000 个像素周期，降低复位后 HDMI 偶发失锁风险。
  // ---------------------------------------------------------------------------
  clk_wiz_0
  clk_wiz_01 (
    .clk_in1(clk),
    .resetn(rst_n),
    .clk_out1(pixel_clk),
    .clk_out2(clk_wiz_01_clk_out2),
    .locked(clk_wiz_01_locked)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_timing_gen2
  // 类型: hdmi_timing_gen
  // 作用: 生成 1024x600 视频时序、同步信号、数据有效和像素坐标。
  // 连接说明: 水平总周期 1324、垂直总行数 645；有效区外 pixel_x/pixel_y 输出 0。
  // ---------------------------------------------------------------------------
  hdmi_timing_gen
  hdmi_timing_gen2 (
    .clk_pix(clk_wiz_01_clk_out1),
    .rst_n(clk_wiz_01_locked),
    .hsync(hdmi_timing_gen2_hsync),
    .vsync(hdmi_timing_gen2_vsync),
    .v_de(pixel_de),
    .pixel_x(pixel_x),
    .pixel_y(pixel_y)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_tmds_encoder_B
  // 类型: hdmi_tmds_encoder
  // 作用: 单颜色通道 TMDS 8 bit 到 10 bit 编码器。
  // 连接说明: 消隐期输出控制码；有效期根据输入 1 的数量和 running disparity 选择 XOR/XNOR 与反相。
  // ---------------------------------------------------------------------------
  hdmi_tmds_encoder
  hdmi_tmds_encoder_B (
    .clk_pix(clk_wiz_01_clk_out1),
    .rst_n(clk_wiz_01_locked),
    .data_in(hdmi_rgb_splitter11_data_b),
    .c0(hdmi_timing_gen2_hsync),
    .c1(hdmi_timing_gen2_vsync),
    .v_de(hdmi_timing_gen2_v_de),
    .tmds_10b(hdmi_tmds_encoder_B_tmds_10b)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_tmds_encoder_G
  // 类型: hdmi_tmds_encoder
  // 作用: 单颜色通道 TMDS 8 bit 到 10 bit 编码器。
  // 连接说明: 消隐期输出控制码；有效期根据输入 1 的数量和 running disparity 选择 XOR/XNOR 与反相。
  // ---------------------------------------------------------------------------
  hdmi_tmds_encoder
  hdmi_tmds_encoder_G (
    .clk_pix(clk_wiz_01_clk_out1),
    .rst_n(clk_wiz_01_locked),
    .data_in(hdmi_rgb_splitter11_data_g),
    .c0(hdmi_const11_gnd_bit),
    .c1(hdmi_const11_gnd_bit),
    .v_de(hdmi_timing_gen2_v_de),
    .tmds_10b(hdmi_tmds_encoder_G_tmds_10b)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_tmds_encoder_R
  // 类型: hdmi_tmds_encoder
  // 作用: 单颜色通道 TMDS 8 bit 到 10 bit 编码器。
  // 连接说明: 消隐期输出控制码；有效期根据输入 1 的数量和 running disparity 选择 XOR/XNOR 与反相。
  // ---------------------------------------------------------------------------
  hdmi_tmds_encoder
  hdmi_tmds_encoder_R (
    .clk_pix(clk_wiz_01_clk_out1),
    .rst_n(clk_wiz_01_locked),
    .data_in(hdmi_rgb_splitter11_data_r),
    .c0(hdmi_const11_gnd_bit),
    .c1(hdmi_const11_gnd_bit),
    .v_de(hdmi_timing_gen2_v_de),
    .tmds_10b(hdmi_tmds_encoder_R_tmds_10b)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_serializer_B
  // 类型: hdmi_serializer
  // 作用: 把一个像素周期的 10 bit TMDS 字并串转换为差分输出。
  // 连接说明: 250 MHz 双沿 ODDR 实现等效 500 Mbit/s；OBUFDS 使用 TMDS_33 标准。
  // ---------------------------------------------------------------------------
  hdmi_serializer
  hdmi_serializer_B (
    .clk_pix(clk_wiz_01_clk_out1),
    .clk_5x(clk_wiz_01_clk_out2),
    .rst_n(clk_wiz_01_locked),
    .data_10b(hdmi_tmds_encoder_B_tmds_10b),
    .ser_p(hdmi_d0_p),
    .ser_n(hdmi_d0_n)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_serializer_G
  // 类型: hdmi_serializer
  // 作用: 把一个像素周期的 10 bit TMDS 字并串转换为差分输出。
  // 连接说明: 250 MHz 双沿 ODDR 实现等效 500 Mbit/s；OBUFDS 使用 TMDS_33 标准。
  // ---------------------------------------------------------------------------
  hdmi_serializer
  hdmi_serializer_G (
    .clk_pix(clk_wiz_01_clk_out1),
    .clk_5x(clk_wiz_01_clk_out2),
    .rst_n(clk_wiz_01_locked),
    .data_10b(hdmi_tmds_encoder_G_tmds_10b),
    .ser_p(hdmi_d1_p),
    .ser_n(hdmi_d1_n)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_serializer_R
  // 类型: hdmi_serializer
  // 作用: 把一个像素周期的 10 bit TMDS 字并串转换为差分输出。
  // 连接说明: 250 MHz 双沿 ODDR 实现等效 500 Mbit/s；OBUFDS 使用 TMDS_33 标准。
  // ---------------------------------------------------------------------------
  hdmi_serializer
  hdmi_serializer_R (
    .clk_pix(clk_wiz_01_clk_out1),
    .clk_5x(clk_wiz_01_clk_out2),
    .rst_n(clk_wiz_01_locked),
    .data_10b(hdmi_tmds_encoder_R_tmds_10b),
    .ser_p(hdmi_d2_p),
    .ser_n(hdmi_d2_n)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_serializer_CLK
  // 类型: hdmi_serializer
  // 作用: 把一个像素周期的 10 bit TMDS 字并串转换为差分输出。
  // 连接说明: 250 MHz 双沿 ODDR 实现等效 500 Mbit/s；OBUFDS 使用 TMDS_33 标准。
  // ---------------------------------------------------------------------------
  hdmi_serializer
  hdmi_serializer_CLK (
    .clk_pix(clk_wiz_01_clk_out1),
    .clk_5x(clk_wiz_01_clk_out2),
    .rst_n(clk_wiz_01_locked),
    .data_10b(hdmi_const11_tmds_clk_pattern),
    .ser_p(hdmi_clk_p),
    .ser_n(hdmi_clk_n)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_const11
  // 类型: hdmi_const
  // 作用: HDMI 常量源。
  // 连接说明: 提供 TMDS 时钟通道 10 bit 模式和逻辑零。
  // ---------------------------------------------------------------------------
  hdmi_const
  hdmi_const11 (
    .tmds_clk_pattern(hdmi_const11_tmds_clk_pattern),
    .gnd_bit(hdmi_const11_gnd_bit)
  );

  // ---------------------------------------------------------------------------
  // 子模块实例: hdmi_rgb_splitter11
  // 类型: hdmi_rgb_splitter
  // 作用: RGB888 通道拆分与消隐门控。
  // 连接说明: pixel_de=0 时三色数据强制为零。
  // ---------------------------------------------------------------------------
  hdmi_rgb_splitter
  hdmi_rgb_splitter11 (
    .rgb_data(rgb_data),
    .pixel_de(hdmi_timing_gen2_v_de),
    .data_r(hdmi_rgb_splitter11_data_r),
    .data_g(hdmi_rgb_splitter11_data_g),
    .data_b(hdmi_rgb_splitter11_data_b)
  );

endmodule

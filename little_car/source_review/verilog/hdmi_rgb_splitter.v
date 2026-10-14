// =============================================================================
// 模块: hdmi_rgb_splitter
// 作用: RGB888 通道拆分与消隐门控。
// 说明: pixel_de=0 时三色数据强制为零。
//
// Robei 源: little_car/model/hdmi_rgb_splitter.model
// 导出范围: 工程登记模块
// 注意: 本文件是通读副本；工程修改仍应同步回 build/*.v 和对应 .model。
// 重生成: python little_car/source_review/export_sources.py
// =============================================================================

module hdmi_rgb_splitter (
  input wire [23:0] rgb_data,  // GUI 输出或 HDMI 输入的 RGB888 像素。
  input wire pixel_de,  // 视频数据有效；1 表示位于可见区域。
  output wire [7:0] data_r,  // pixel_de 门控后的 RGB888 红色字节；消隐区强制为 0。
  output wire [7:0] data_g,  // pixel_de 门控后的 RGB888 绿色字节；消隐区强制为 0。
  output wire [7:0] data_b  // pixel_de 门控后的 RGB888 蓝色字节；消隐区强制为 0。
);

  // ===========================================================================
  // 模块行为实现
  // ===========================================================================
// -----------------------------------------------------------------------------
// 组合派生信号
// 以下连续赋值不保存状态，会随输入和寄存器值立即重新计算。
// -----------------------------------------------------------------------------
// [ASSIGN] 组合定义“data_r”：pixel_de 门控后的 RGB888 红色字节；消隐区强制为 0。 该组合结果直接由 `pixel_de`（像素 / 数据有效）、`rgb_data`（RGB 像素 / 数据）的当前值决定，不引入时钟周期延迟。
assign data_r = pixel_de ? rgb_data[23:16] : 8'h00;
// [ASSIGN] 组合定义“data_g”：pixel_de 门控后的 RGB888 绿色字节；消隐区强制为 0。 该组合结果直接由 `pixel_de`（像素 / 数据有效）、`rgb_data`（RGB 像素 / 数据）的当前值决定，不引入时钟周期延迟。
assign data_g = pixel_de ? rgb_data[15:8]  : 8'h00;
// [ASSIGN] 组合定义“data_b”：pixel_de 门控后的 RGB888 蓝色字节；消隐区强制为 0。 该组合结果直接由 `pixel_de`（像素 / 数据有效）、`rgb_data`（RGB 像素 / 数据）的当前值决定，不引入时钟周期延迟。
assign data_b = pixel_de ? rgb_data[7:0]   : 8'h00;

endmodule

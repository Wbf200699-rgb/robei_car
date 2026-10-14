module hdmi_rgb_splitter(
input wire [23:0] rgb_data,
input wire pixel_de,
output wire [7:0] data_r,
output wire [7:0] data_g,
output wire [7:0] data_b
);

  assign data_r = pixel_de ? rgb_data[23:16] : 8'h00;
  assign data_g = pixel_de ? rgb_data[15:8]  : 8'h00;
  assign data_b = pixel_de ? rgb_data[7:0]   : 8'h00;

endmodule    //hdmi_rgb_splitter

module hdmi_const(
output wire [9:0] tmds_clk_pattern,
output wire gnd_bit
);

  //----Code starts here: integrated by Robei-----
  assign tmds_clk_pattern = 10'b1111100000;
  assign gnd_bit = 1'b0;
endmodule    //hdmi_const

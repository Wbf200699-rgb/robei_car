module route_demo_info_source(
output wire [1:0] system_info,
output wire [63:0] route_info
);

  assign system_info = 2'b01;

  // Low bits are earlier steps:
  // 11 forward, 01 right, 10 left, 00 stop.
  assign route_info = {
    40'd0,
    2'b11, 2'b10, 2'b11, 2'b01,
    2'b11, 2'b01, 2'b11, 2'b10,
    2'b11, 2'b01, 2'b11, 2'b11
  };

endmodule    //route_demo_info_source

module temp_gui_status_source(
output wire [63:0] old_route_info,
output wire [5:0] active_route_steps,
output wire [5:0] completed_route_steps,
output wire [15:0] step_length,
output wire [15:0] speed,
output wire [15:0] total_route_length,
output wire [15:0] complete_route_length,
output wire [15:0] remaining_route_length,
output wire [7:0] now_x,
output wire [7:0] now_y,
output wire [7:0] target_x,
output wire [7:0] target_y
);

  assign old_route_info = 64'd0;
  assign active_route_steps = 6'd12;
  assign completed_route_steps = 6'd0;
  assign step_length = 16'd20;
  assign speed = 16'd25;
  assign total_route_length = 16'd240;
  assign complete_route_length = 16'd0;
  assign remaining_route_length = 16'd240;
  assign now_x = 8'd128;
  assign now_y = 8'd128;
  assign target_x = 8'd128;
  assign target_y = 8'd124;

endmodule

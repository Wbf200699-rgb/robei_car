module temp_c5_motor_config_source(
output wire motor_enable,
output wire [15:0] motor_forward_ms,
output wire [15:0] motor_turn_ms,
output wire [9:0] motor_forward_speed,
output wire [9:0] motor_turn_speed
);

  assign motor_enable = 1'b1;
  assign motor_forward_ms = 16'd2500;
  assign motor_turn_ms = 16'd550;
  assign motor_forward_speed = 10'd700;
  assign motor_turn_speed = 10'd650;

endmodule

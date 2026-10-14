module temp_motor_config_source(
output wire motor_enable,
output wire [15:0] motor_forward_ticks,
output wire [15:0] motor_turn_ticks,
output wire [7:0] motor_forward_duty,
output wire [7:0] motor_turn_duty
);

  assign motor_enable = 1'b1;
  assign motor_forward_ticks = 16'd2496;
  assign motor_turn_ticks = 16'd360;
  assign motor_forward_duty = 8'd120;
  assign motor_turn_duty = 8'd100;

endmodule

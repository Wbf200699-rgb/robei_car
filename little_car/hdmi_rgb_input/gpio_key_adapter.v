module gpio_key_adapter(
input wire key_pin1,
input wire key_pin2,
input wire key_pin3,
input wire key_pin4,
input wire key_pin5,
input wire key_active_low,
output wire key_up_n,
output wire key_down_n,
output wire key_left_n,
output wire key_right_n,
output wire key_mid_n
);

  wire sw1_n;
  wire sw2_n;
  wire sw3_n;
  wire sw4_n;
  wire sw5_n;

  assign sw1_n = key_active_low ? key_pin1 : ~key_pin1;
  assign sw2_n = key_active_low ? key_pin2 : ~key_pin2;
  assign sw3_n = key_active_low ? key_pin3 : ~key_pin3;
  assign sw4_n = key_active_low ? key_pin4 : ~key_pin4;
  assign sw5_n = key_active_low ? key_pin5 : ~key_pin5;

  assign key_up_n    = sw1_n;
  assign key_left_n  = sw2_n;
  assign key_right_n = sw3_n;
  assign key_down_n  = sw4_n;
  assign key_mid_n   = sw5_n;

endmodule

module key_low_merge(
input wire key_up_n,
input wire key_down_n,
input wire key_left_n,
input wire key_right_n,
input wire key_mid_n,
input wire bt_key_up_n,
input wire bt_key_down_n,
input wire bt_key_left_n,
input wire bt_key_right_n,
input wire bt_key_mid_n,
output wire merged_key_up_n,
output wire merged_key_down_n,
output wire merged_key_left_n,
output wire merged_key_right_n,
output wire merged_key_mid_n
);

  assign merged_key_up_n = key_up_n & bt_key_up_n;
  assign merged_key_down_n = key_down_n & bt_key_down_n;
  assign merged_key_left_n = key_left_n & bt_key_left_n;
  assign merged_key_right_n = key_right_n & bt_key_right_n;
  assign merged_key_mid_n = key_mid_n & bt_key_mid_n;

endmodule

module route_control_fsm(
input wire clk,
input wire rst_n,
input wire key_up_n,
input wire key_down_n,
input wire key_left_n,
input wire key_right_n,
input wire key_mid_n,
input wire run_step_pulse,
input wire obstacle_stop,
input wire [15:0] step_length_set,
input wire [15:0] speed_set,
output wire [1:0] menu_info,
output wire [4:0] state_info,
output wire [63:0] route_info,
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

  wire key_up_pulse;
  wire key_down_pulse;
  wire key_left_pulse;
  wire key_right_pulse;
  wire key_mid_pulse;

  key_pulse_filter key_up_filter(
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_up_n),
    .key_pulse(key_up_pulse)
  );

  key_pulse_filter key_down_filter(
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_down_n),
    .key_pulse(key_down_pulse)
  );

  key_pulse_filter key_left_filter(
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_left_n),
    .key_pulse(key_left_pulse)
  );

  key_pulse_filter key_right_filter(
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_right_n),
    .key_pulse(key_right_pulse)
  );

  key_pulse_filter key_mid_filter(
    .clk(clk),
    .rst_n(rst_n),
    .key_n(key_mid_n),
    .key_pulse(key_mid_pulse)
  );

  route_control_core route_control_core_inst(
    .clk(clk),
    .rst_n(rst_n),
    .key_up_pulse(key_up_pulse),
    .key_down_pulse(key_down_pulse),
    .key_left_pulse(key_left_pulse),
    .key_right_pulse(key_right_pulse),
    .key_mid_pulse(key_mid_pulse),
    .run_step_pulse(run_step_pulse),
    .obstacle_stop(obstacle_stop),
    .step_length_set(step_length_set),
    .speed_set(speed_set),
    .menu_info(menu_info),
    .state_info(state_info),
    .route_info(route_info),
    .old_route_info(old_route_info),
    .active_route_steps(active_route_steps),
    .completed_route_steps(completed_route_steps),
    .step_length(step_length),
    .speed(speed),
    .total_route_length(total_route_length),
    .complete_route_length(complete_route_length),
    .remaining_route_length(remaining_route_length),
    .now_x(now_x),
    .now_y(now_y),
    .target_x(target_x),
    .target_y(target_y)
  );

endmodule

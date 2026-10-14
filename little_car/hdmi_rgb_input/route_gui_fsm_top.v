module route_gui_fsm_top(
input wire clk,
input wire rst_n,
input wire key_pin1,
input wire key_pin2,
input wire key_pin3,
input wire key_pin4,
input wire key_pin5,
input wire ultrasonic_echo,
input wire bt_uart_rx,
input wire c5_uart_rx,
output wire bt_uart_tx,
output wire c5_uart_tx,
output wire ultrasonic_trig,
output wire buzzer_out,
output wire hdmi_clk_p,
output wire hdmi_clk_n,
output wire hdmi_d0_p,
output wire hdmi_d0_n,
output wire hdmi_d1_p,
output wire hdmi_d1_n,
output wire hdmi_d2_p,
output wire hdmi_d2_n
);

  wire [23:0] rgb_data;
  wire [11:0] pixel_x;
  wire [11:0] pixel_y;
  wire pixel_de;
  wire pixel_clk;
  wire [1:0] menu_info;
  wire [4:0] state_info;
  wire [63:0] route_info;
  wire [63:0] old_route_info;
  wire [5:0] active_route_steps;
  wire [5:0] completed_route_steps;
  wire [15:0] step_length;
  wire [15:0] speed;
  wire [15:0] total_route_length;
  wire [15:0] complete_route_length;
  wire [15:0] remaining_route_length;
  wire [7:0] now_x;
  wire [7:0] now_y;
  wire [7:0] target_x;
  wire [7:0] target_y;
  wire run_step_pulse;
  wire temp_run_step_pulse;
  wire obstacle_stop;
  wire temp_obstacle_stop;
  wire [15:0] step_length_set;
  wire [15:0] speed_set;
  wire motor_enable;
  wire [15:0] motor_forward_ms;
  wire [15:0] motor_turn_ms;
  wire [9:0] motor_forward_speed;
  wire [9:0] motor_turn_speed;
  wire key_up_n;
  wire key_down_n;
  wire key_left_n;
  wire key_right_n;
  wire key_mid_n;
  wire bt_key_up_n;
  wire bt_key_down_n;
  wire bt_key_left_n;
  wire bt_key_right_n;
  wire bt_key_mid_n;
  wire merged_key_up_n;
  wire merged_key_down_n;
  wire merged_key_left_n;
  wire merged_key_right_n;
  wire merged_key_mid_n;
  wire key_active_low;
  wire [10:0] c5_lf_speed;
  wire [10:0] c5_rf_speed;
  wire [10:0] c5_lr_speed;
  wire [10:0] c5_rr_speed;
  wire motor_motion_busy;
  wire [1:0] motor_current_cmd;
  wire c5_frame_busy;
  wire ultrasonic_enable;
  wire [15:0] ultrasonic_distance_cm;
  wire ultrasonic_distance_valid;
  wire ultrasonic_echo_timeout;
  wire power_on_pulse;
  wire route_add_pulse;
  wire run_enter_pulse;
  wire stop_enter_pulse;

  temp_key_gpio_config_source temp_key_gpio_config_source_inst(
    .key_active_low(key_active_low)
  );

  gpio_key_adapter gpio_key_adapter_inst(
    .key_pin1(key_pin1),
    .key_pin2(key_pin2),
    .key_pin3(key_pin3),
    .key_pin4(key_pin4),
    .key_pin5(key_pin5),
    .key_active_low(key_active_low),
    .key_up_n(key_up_n),
    .key_down_n(key_down_n),
    .key_left_n(key_left_n),
    .key_right_n(key_right_n),
    .key_mid_n(key_mid_n)
  );

  bt_route_uart bt_route_uart_inst(
    .clk(clk),
    .rst_n(rst_n),
    .bt_uart_rx(bt_uart_rx),
    .menu_info(menu_info),
    .state_info(state_info),
    .route_info(route_info),
    .active_route_steps(active_route_steps),
    .completed_route_steps(completed_route_steps),
    .now_x(now_x),
    .now_y(now_y),
    .target_x(target_x),
    .target_y(target_y),
    .bt_uart_tx(bt_uart_tx),
    .bt_key_up_n(bt_key_up_n),
    .bt_key_down_n(bt_key_down_n),
    .bt_key_left_n(bt_key_left_n),
    .bt_key_right_n(bt_key_right_n),
    .bt_key_mid_n(bt_key_mid_n)
  );

  key_low_merge key_low_merge_inst(
    .key_up_n(key_up_n),
    .key_down_n(key_down_n),
    .key_left_n(key_left_n),
    .key_right_n(key_right_n),
    .key_mid_n(key_mid_n),
    .bt_key_up_n(bt_key_up_n),
    .bt_key_down_n(bt_key_down_n),
    .bt_key_left_n(bt_key_left_n),
    .bt_key_right_n(bt_key_right_n),
    .bt_key_mid_n(bt_key_mid_n),
    .merged_key_up_n(merged_key_up_n),
    .merged_key_down_n(merged_key_down_n),
    .merged_key_left_n(merged_key_left_n),
    .merged_key_right_n(merged_key_right_n),
    .merged_key_mid_n(merged_key_mid_n)
  );

  temp_run_status_source temp_run_status_source_inst(
    .run_step_pulse(temp_run_step_pulse),
    .obstacle_stop(temp_obstacle_stop),
    .step_length_set(step_length_set),
    .speed_set(speed_set)
  );

  temp_c5_motor_config_source temp_c5_motor_config_source_inst(
    .motor_enable(motor_enable),
    .motor_forward_ms(motor_forward_ms),
    .motor_turn_ms(motor_turn_ms),
    .motor_forward_speed(motor_forward_speed),
    .motor_turn_speed(motor_turn_speed)
  );

  c5_route_controller c5_route_controller_inst(
    .clk(clk),
    .rst_n(rst_n),
    .motor_enable(motor_enable),
    .state_info(state_info),
    .route_info(route_info),
    .active_route_steps(active_route_steps),
    .forward_ms(motor_forward_ms),
    .turn_ms(motor_turn_ms),
    .forward_speed(motor_forward_speed),
    .turn_speed(motor_turn_speed),
    .run_step_pulse(run_step_pulse),
    .lf_speed(c5_lf_speed),
    .rf_speed(c5_rf_speed),
    .lr_speed(c5_lr_speed),
    .rr_speed(c5_rr_speed),
    .motion_busy(motor_motion_busy),
    .current_cmd(motor_current_cmd)
  );

  c5_motor_frame_driver c5_motor_frame_driver_inst(
    .clk(clk),
    .rst_n(rst_n),
    .motor_enable(motor_enable),
    .lf_speed(c5_lf_speed),
    .rf_speed(c5_rf_speed),
    .lr_speed(c5_lr_speed),
    .rr_speed(c5_rr_speed),
    .c5_bus_tx(c5_uart_tx),
    .frame_busy(c5_frame_busy)
  );

  temp_ultrasonic_enable_source temp_ultrasonic_enable_source_inst(
    .ultrasonic_enable(ultrasonic_enable)
  );

  ultrasonic_obstacle_guard ultrasonic_obstacle_guard_inst(
    .clk(clk),
    .rst_n(rst_n),
    .enable(ultrasonic_enable),
    .echo(ultrasonic_echo),
    .step_length(step_length),
    .trig(ultrasonic_trig),
    .distance_cm(ultrasonic_distance_cm),
    .distance_valid(ultrasonic_distance_valid),
    .obstacle_stop(obstacle_stop),
    .echo_timeout(ultrasonic_echo_timeout)
  );

  buzzer_event_gen buzzer_event_gen_inst(
    .clk(clk),
    .rst_n(rst_n),
    .state_info(state_info),
    .active_route_steps(active_route_steps),
    .power_on_pulse(power_on_pulse),
    .route_add_pulse(route_add_pulse),
    .run_enter_pulse(run_enter_pulse),
    .stop_enter_pulse(stop_enter_pulse)
  );

  buzzer_ctrl buzzer_ctrl_inst(
    .clk(clk),
    .rst_n(rst_n),
    .power_on_pulse(power_on_pulse),
    .route_add_pulse(route_add_pulse),
    .run_enter_pulse(run_enter_pulse),
    .stop_enter_pulse(stop_enter_pulse),
    .buzzer_out(buzzer_out)
  );

  route_control_fsm route_control_fsm_inst(
    .clk(clk),
    .rst_n(rst_n),
    .key_up_n(merged_key_up_n),
    .key_down_n(merged_key_down_n),
    .key_left_n(merged_key_left_n),
    .key_right_n(merged_key_right_n),
    .key_mid_n(merged_key_mid_n),
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

  route_gui_renderer route_gui_renderer_inst(
    .clk_pix(pixel_clk),
    .rst_n(rst_n),
    .pixel_x(pixel_x),
    .pixel_y(pixel_y),
    .pixel_de(pixel_de),
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
    .target_y(target_y),
    .rgb_data(rgb_data)
  );

  hdmi_top hdmi_top_inst(
    .clk(clk),
    .rst_n(rst_n),
    .rgb_data(rgb_data),
    .pixel_x(pixel_x),
    .pixel_y(pixel_y),
    .pixel_de(pixel_de),
    .pixel_clk(pixel_clk),
    .hdmi_clk_p(hdmi_clk_p),
    .hdmi_clk_n(hdmi_clk_n),
    .hdmi_d0_p(hdmi_d0_p),
    .hdmi_d0_n(hdmi_d0_n),
    .hdmi_d1_p(hdmi_d1_p),
    .hdmi_d1_n(hdmi_d1_n),
    .hdmi_d2_p(hdmi_d2_p),
    .hdmi_d2_n(hdmi_d2_n)
  );

endmodule

`timescale 1ns / 1ps

module tb_route_control_motor_ultrasonic_top;

  localparam integer ULTRASONIC_FAR_CM = 60;
  localparam integer ULTRASONIC_NEAR_CM = 10;
  localparam integer ULTRASONIC_NEAR_AFTER_REPLY = 10;

  reg clk;
  reg rst_n;
  reg key_pin1;
  reg key_pin2;
  reg key_pin3;
  reg key_pin4;
  reg key_pin5;
  reg ultrasonic_echo;
  reg motor_lf_enc_a;
  reg motor_lf_enc_b;
  reg motor_rf_enc_a;
  reg motor_rf_enc_b;
  reg motor_lr_enc_a;
  reg motor_lr_enc_b;
  reg motor_rr_enc_a;
  reg motor_rr_enc_b;

  wire key_active_low;
  wire key_up_n;
  wire key_down_n;
  wire key_left_n;
  wire key_right_n;
  wire key_mid_n;
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
  wire [15:0] step_length_set;
  wire [15:0] speed_set;
  wire ultrasonic_enable;
  wire ultrasonic_trig;
  wire [15:0] ultrasonic_distance_cm;
  wire ultrasonic_distance_valid;
  wire obstacle_stop;
  wire ultrasonic_echo_timeout;
  wire motor_enable;
  wire [15:0] motor_forward_ticks;
  wire [15:0] motor_turn_ticks;
  wire [7:0] motor_forward_duty;
  wire [7:0] motor_turn_duty;
  wire motor_lf_tick;
  wire motor_rf_tick;
  wire motor_lr_tick;
  wire motor_rr_tick;
  wire motor_lf_enc_dir;
  wire motor_rf_enc_dir;
  wire motor_lr_enc_dir;
  wire motor_rr_enc_dir;
  wire [15:0] motor_lf_total_count;
  wire [15:0] motor_rf_total_count;
  wire [15:0] motor_lr_total_count;
  wire [15:0] motor_rr_total_count;
  wire run_step_pulse;
  wire [1:0] motor_lf_dir;
  wire [1:0] motor_rf_dir;
  wire [1:0] motor_lr_dir;
  wire [1:0] motor_rr_dir;
  wire [7:0] motor_lf_duty;
  wire [7:0] motor_rf_duty;
  wire [7:0] motor_lr_duty;
  wire [7:0] motor_rr_duty;
  wire motor_motion_busy;
  wire [1:0] motor_current_cmd;
  wire [15:0] motor_lf_count;
  wire [15:0] motor_rf_count;
  wire [15:0] motor_lr_count;
  wire [15:0] motor_rr_count;
  wire motor_lf_in1;
  wire motor_lf_in2;
  wire motor_rf_in1;
  wire motor_rf_in2;
  wire motor_lr_in1;
  wire motor_lr_in2;
  wire motor_rr_in1;
  wire motor_rr_in2;
  wire motor_lf_pwm;
  wire motor_rf_pwm;
  wire motor_lr_pwm;
  wire motor_rr_pwm;
  wire motor_stby_a;
  wire motor_stby_b;

  integer ultrasonic_reply_count;

  temp_key_gpio_config_source key_cfg_inst (
    .key_active_low(key_active_low)
  );

  gpio_key_adapter key_adapter_inst (
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

  temp_run_status_source run_status_inst (
    .run_step_pulse(),
    .obstacle_stop(),
    .step_length_set(step_length_set),
    .speed_set(speed_set)
  );

  temp_ultrasonic_enable_source ultrasonic_enable_inst (
    .ultrasonic_enable(ultrasonic_enable)
  );

  ultrasonic_obstacle_guard ultrasonic_inst (
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

  temp_motor_config_source motor_cfg_inst (
    .motor_enable(motor_enable),
    .motor_forward_ticks(motor_forward_ticks),
    .motor_turn_ticks(motor_turn_ticks),
    .motor_forward_duty(motor_forward_duty),
    .motor_turn_duty(motor_turn_duty)
  );

  motor_encoder_interface encoder_inst (
    .clk(clk),
    .rst_n(rst_n),
    .lf_enc_a(motor_lf_enc_a),
    .lf_enc_b(motor_lf_enc_b),
    .rf_enc_a(motor_rf_enc_a),
    .rf_enc_b(motor_rf_enc_b),
    .lr_enc_a(motor_lr_enc_a),
    .lr_enc_b(motor_lr_enc_b),
    .rr_enc_a(motor_rr_enc_a),
    .rr_enc_b(motor_rr_enc_b),
    .lf_tick(motor_lf_tick),
    .rf_tick(motor_rf_tick),
    .lr_tick(motor_lr_tick),
    .rr_tick(motor_rr_tick),
    .lf_dir(motor_lf_enc_dir),
    .rf_dir(motor_rf_enc_dir),
    .lr_dir(motor_lr_enc_dir),
    .rr_dir(motor_rr_enc_dir),
    .lf_total_count(motor_lf_total_count),
    .rf_total_count(motor_rf_total_count),
    .lr_total_count(motor_lr_total_count),
    .rr_total_count(motor_rr_total_count)
  );

  motor_route_controller motor_route_inst (
    .clk(clk),
    .rst_n(rst_n),
    .motor_enable(motor_enable),
    .state_info(state_info),
    .route_info(route_info),
    .active_route_steps(active_route_steps),
    .lf_tick(motor_lf_tick),
    .rf_tick(motor_rf_tick),
    .lr_tick(motor_lr_tick),
    .rr_tick(motor_rr_tick),
    .forward_ticks(motor_forward_ticks),
    .turn_ticks(motor_turn_ticks),
    .forward_duty(motor_forward_duty),
    .turn_duty(motor_turn_duty),
    .run_step_pulse(run_step_pulse),
    .lf_dir(motor_lf_dir),
    .rf_dir(motor_rf_dir),
    .lr_dir(motor_lr_dir),
    .rr_dir(motor_rr_dir),
    .lf_duty(motor_lf_duty),
    .rf_duty(motor_rf_duty),
    .lr_duty(motor_lr_duty),
    .rr_duty(motor_rr_duty),
    .motion_busy(motor_motion_busy),
    .current_cmd(motor_current_cmd),
    .lf_count(motor_lf_count),
    .rf_count(motor_rf_count),
    .lr_count(motor_lr_count),
    .rr_count(motor_rr_count)
  );

  motor_tb6612_driver motor_driver_inst (
    .clk(clk),
    .rst_n(rst_n),
    .motor_enable(motor_enable),
    .lf_dir(motor_lf_dir),
    .rf_dir(motor_rf_dir),
    .lr_dir(motor_lr_dir),
    .rr_dir(motor_rr_dir),
    .lf_duty(motor_lf_duty),
    .rf_duty(motor_rf_duty),
    .lr_duty(motor_lr_duty),
    .rr_duty(motor_rr_duty),
    .lf_in1(motor_lf_in1),
    .lf_in2(motor_lf_in2),
    .rf_in1(motor_rf_in1),
    .rf_in2(motor_rf_in2),
    .lr_in1(motor_lr_in1),
    .lr_in2(motor_lr_in2),
    .rr_in1(motor_rr_in1),
    .rr_in2(motor_rr_in2),
    .lf_pwm(motor_lf_pwm),
    .rf_pwm(motor_rf_pwm),
    .lr_pwm(motor_lr_pwm),
    .rr_pwm(motor_rr_pwm),
    .stby_a(motor_stby_a),
    .stby_b(motor_stby_b)
  );

  sys_route_control_fsm control_inst (
    .clk(clk),
    .rst_n(rst_n),
    .key_up_n(key_up_n),
    .key_down_n(key_down_n),
    .key_left_n(key_left_n),
    .key_right_n(key_right_n),
    .key_mid_n(key_mid_n),
    .menu_info(menu_info),
    .route_info(route_info),
    .run_step_pulse(run_step_pulse),
    .obstacle_stop(obstacle_stop),
    .step_length_set(step_length_set),
    .speed_set(speed_set),
    .state_info(state_info),
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

  initial begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  task show_state;
    input [8*28-1:0] tag;
    begin
      $display("[%0t] %0s state=%b menu=%b active=%0d done=%0d route=%h old=%h now=(%0d,%0d) target=(%0d,%0d) cmd=%b busy=%b dist=%0d stop=%b motor=%b/%b/%b/%b",
               $time, tag, state_info, menu_info, active_route_steps, completed_route_steps,
               route_info, old_route_info, now_x, now_y, target_x, target_y,
               motor_current_cmd, motor_motion_busy, ultrasonic_distance_cm, obstacle_stop,
               motor_lf_dir, motor_rf_dir, motor_lr_dir, motor_rr_dir);
    end
  endtask

  task press_gpio_key;
    input [2:0] key_id;
    begin
      case (key_id)
        3'd0: key_pin1 = 1'b0;
        3'd1: key_pin2 = 1'b0;
        3'd2: key_pin3 = 1'b0;
        3'd3: key_pin4 = 1'b0;
        default: key_pin5 = 1'b0;
      endcase
      #25_000_000;
      key_pin1 = 1'b1;
      key_pin2 = 1'b1;
      key_pin3 = 1'b1;
      key_pin4 = 1'b1;
      key_pin5 = 1'b1;
      #25_000_000;
    end
  endtask

  task send_encoder_ticks;
    input integer tick_count;
    integer k;
    begin
      motor_lf_enc_b = 1'b0;
      motor_rf_enc_b = 1'b0;
      motor_lr_enc_b = 1'b0;
      motor_rr_enc_b = 1'b0;

      for (k = 0; k < tick_count; k = k + 1) begin
        motor_lf_enc_a = 1'b0;
        motor_rf_enc_a = 1'b0;
        motor_lr_enc_a = 1'b0;
        motor_rr_enc_a = 1'b0;
        #200;
        motor_lf_enc_a = 1'b1;
        motor_rf_enc_a = 1'b1;
        motor_lr_enc_a = 1'b1;
        motor_rr_enc_a = 1'b1;
        #200;
      end

      motor_lf_enc_a = 1'b0;
      motor_rf_enc_a = 1'b0;
      motor_lr_enc_a = 1'b0;
      motor_rr_enc_a = 1'b0;
      #1_000;
    end
  endtask

  initial begin
    ultrasonic_reply_count = 0;
    ultrasonic_echo = 1'b0;

    forever begin
      wait (rst_n == 1'b1);
      wait (ultrasonic_trig == 1'b1);
      wait (ultrasonic_trig == 1'b0);
      #100_000;

      ultrasonic_echo = 1'b1;
      if (ultrasonic_reply_count < ULTRASONIC_NEAR_AFTER_REPLY)
        #(ULTRASONIC_FAR_CM * 58_000);
      else
        #(ULTRASONIC_NEAR_CM * 58_000);
      ultrasonic_echo = 1'b0;
      ultrasonic_reply_count = ultrasonic_reply_count + 1;
    end
  end

  initial begin
    rst_n = 1'b0;
    key_pin1 = 1'b1;
    key_pin2 = 1'b1;
    key_pin3 = 1'b1;
    key_pin4 = 1'b1;
    key_pin5 = 1'b1;
    motor_lf_enc_a = 1'b0;
    motor_lf_enc_b = 1'b0;
    motor_rf_enc_a = 1'b0;
    motor_rf_enc_b = 1'b0;
    motor_lr_enc_a = 1'b0;
    motor_lr_enc_b = 1'b0;
    motor_rr_enc_a = 1'b0;
    motor_rr_enc_b = 1'b0;

    #1_000;
    rst_n = 1'b1;
    #1_000;
    show_state("after reset");

    press_gpio_key(3'd4);
    show_state("enter change");

    press_gpio_key(3'd0);
    show_state("add forward");

    press_gpio_key(3'd1);
    show_state("add left+forward");

    press_gpio_key(3'd4);
    show_state("exit change");

    press_gpio_key(3'd3);
    show_state("focus run");

    press_gpio_key(3'd4);
    show_state("enter run");

    #100_000;
    send_encoder_ticks(2600);
    show_state("after motor step 1");

    wait (motor_motion_busy == 1'b0);
    #100_000;
    wait (motor_current_cmd != 2'b11);
    #100_000;
    send_encoder_ticks(1400);
    show_state("after motor step 2");

    #80_000_000;
    show_state("after ultrasonic near");

    #1_000_000;
    $display("[%0t] tb_route_control_motor_ultrasonic_top finished", $time);
    $finish;
  end

endmodule

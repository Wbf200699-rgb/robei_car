`timescale 1ns / 1ps

module tb_sys_route_gui_fsm_top;

  reg clk;
  reg rst_n;
  reg motor_lf_enc_a;
  reg motor_lf_enc_b;
  reg motor_rf_enc_a;
  reg motor_rf_enc_b;
  reg motor_lr_enc_a;
  reg motor_lr_enc_b;
  reg motor_rr_enc_a;
  reg motor_rr_enc_b;
  reg key_pin1;
  reg key_pin2;
  reg key_pin3;
  reg key_pin4;
  reg key_pin5;
  reg ultrasonic_echo;

  wire hdmi_clk_p;
  wire hdmi_clk_n;
  wire hdmi_d0_p;
  wire hdmi_d0_n;
  wire hdmi_d1_p;
  wire hdmi_d1_n;
  wire hdmi_d2_p;
  wire hdmi_d2_n;
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
  wire ultrasonic_trig;
  integer ultrasonic_reply_count;

  sys_route_gui_fsm_top dut (
    .clk(clk),
    .rst_n(rst_n),
    .hdmi_clk_p(hdmi_clk_p),
    .hdmi_clk_n(hdmi_clk_n),
    .hdmi_d0_p(hdmi_d0_p),
    .hdmi_d0_n(hdmi_d0_n),
    .hdmi_d1_p(hdmi_d1_p),
    .hdmi_d1_n(hdmi_d1_n),
    .hdmi_d2_p(hdmi_d2_p),
    .hdmi_d2_n(hdmi_d2_n),
    .motor_lf_enc_a(motor_lf_enc_a),
    .motor_lf_enc_b(motor_lf_enc_b),
    .motor_rf_enc_a(motor_rf_enc_a),
    .motor_rf_enc_b(motor_rf_enc_b),
    .motor_lr_enc_a(motor_lr_enc_a),
    .motor_lr_enc_b(motor_lr_enc_b),
    .motor_rr_enc_a(motor_rr_enc_a),
    .motor_rr_enc_b(motor_rr_enc_b),
    .motor_lf_in1(motor_lf_in1),
    .motor_lf_in2(motor_lf_in2),
    .motor_rf_in1(motor_rf_in1),
    .motor_rf_in2(motor_rf_in2),
    .motor_lr_in1(motor_lr_in1),
    .motor_lr_in2(motor_lr_in2),
    .motor_rr_in1(motor_rr_in1),
    .motor_rr_in2(motor_rr_in2),
    .motor_lf_pwm(motor_lf_pwm),
    .motor_rf_pwm(motor_rf_pwm),
    .motor_lr_pwm(motor_lr_pwm),
    .motor_rr_pwm(motor_rr_pwm),
    .motor_stby_a(motor_stby_a),
    .motor_stby_b(motor_stby_b),
    .key_pin1(key_pin1),
    .key_pin2(key_pin2),
    .key_pin3(key_pin3),
    .key_pin4(key_pin4),
    .key_pin5(key_pin5),
    .ultrasonic_echo(ultrasonic_echo),
    .ultrasonic_trig(ultrasonic_trig)
  );

  initial begin
    clk = 1'b0;
    forever #10 clk = ~clk;  // 50 MHz
  end

  task show_top;
    input [8*24-1:0] tag;
    begin
      $display("[%0t] %0s state=%b menu=%b active=%0d done=%0d route=%h old=%h now=(%0d,%0d) target=(%0d,%0d) motor_dir lf/rf/lr/rr=%b/%b/%b/%b pwm=%b/%b/%b/%b trig=%b echo=%b",
               $time, tag,
               dut.sys_route_control_fsm1_state_info,
               dut.sys_route_control_fsm1_menu_info,
               dut.sys_route_control_fsm1_active_route_steps,
               dut.sys_route_control_fsm1_completed_route_steps,
               dut.sys_route_control_fsm1_route_info,
               dut.sys_route_control_fsm1_old_route_info,
               dut.sys_route_control_fsm1_now_x,
               dut.sys_route_control_fsm1_now_y,
               dut.sys_route_control_fsm1_target_x,
               dut.sys_route_control_fsm1_target_y,
               dut.motor_route_controller9_lf_dir,
               dut.motor_route_controller9_rf_dir,
               dut.motor_route_controller9_lr_dir,
               dut.motor_route_controller9_rr_dir,
               motor_lf_pwm, motor_rf_pwm, motor_lr_pwm, motor_rr_pwm,
               ultrasonic_trig, ultrasonic_echo);
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

  task echo_distance_cm;
    input integer cm;
    begin
      wait (ultrasonic_trig == 1'b1);
      wait (ultrasonic_trig == 1'b0);
      #100_000;
      ultrasonic_echo = 1'b1;
      #(cm * 58_000);
      ultrasonic_echo = 1'b0;
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
      if (ultrasonic_reply_count < 2) begin
        #(60 * 58_000);
      end else begin
        #(10 * 58_000);
      end
      ultrasonic_echo = 1'b0;
      ultrasonic_reply_count = ultrasonic_reply_count + 1;
    end
  end

  initial begin
    rst_n = 1'b0;
    motor_lf_enc_a = 1'b0;
    motor_lf_enc_b = 1'b0;
    motor_rf_enc_a = 1'b0;
    motor_rf_enc_b = 1'b0;
    motor_lr_enc_a = 1'b0;
    motor_lr_enc_b = 1'b0;
    motor_rr_enc_a = 1'b0;
    motor_rr_enc_b = 1'b0;
    key_pin1 = 1'b1;
    key_pin2 = 1'b1;
    key_pin3 = 1'b1;
    key_pin4 = 1'b1;
    key_pin5 = 1'b1;

    #1_000;
    rst_n = 1'b1;
    #1_000;
    show_top("after reset");

    press_gpio_key(3'd4);  // enter change_route
    show_top("enter change");

    press_gpio_key(3'd0);  // forward
    show_top("add forward");

    press_gpio_key(3'd1);  // left + forward
    show_top("add left+forward");

    press_gpio_key(3'd4);  // exit change_route
    show_top("exit change");

    press_gpio_key(3'd3);  // focus run
    show_top("focus run");

    press_gpio_key(3'd4);  // enter run
    show_top("enter run");

    #100_000;
    send_encoder_ticks(2600);
    show_top("after motor step 1");

    send_encoder_ticks(1400);
    show_top("after motor step 2");

    #80_000_000;
    show_top("after ultrasonic near");

    #5_000_000;
    $display("[%0t] tb_sys_route_gui_fsm_top finished", $time);
    $finish;
  end

endmodule

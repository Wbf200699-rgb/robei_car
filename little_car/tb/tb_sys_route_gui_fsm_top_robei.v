module sys_route_gui_fsm_top_test();
reg clk;
reg rst_n;
wire hdmi_clk_p;
wire hdmi_clk_n;
wire hdmi_d0_p;
wire hdmi_d0_n;
wire hdmi_d1_p;
wire hdmi_d1_n;
wire hdmi_d2_p;
wire hdmi_d2_n;
reg motor_lf_enc_a;
reg motor_lf_enc_b;
reg motor_rf_enc_a;
reg motor_rf_enc_b;
reg motor_lr_enc_a;
reg motor_lr_enc_b;
reg motor_rr_enc_a;
reg motor_rr_enc_b;
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
reg key_pin1;
reg key_pin2;
reg key_pin3;
reg key_pin4;
reg key_pin5;
reg ultrasonic_echo;
wire ultrasonic_trig;
wire buzzer_out;

  //----Code starts here: integrated by Robei-----
  initial
  begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  task press_key;
    input [2:0] key_idx;
    begin
      case (key_idx)
        3'd1: key_pin1 = 1'b0;
        3'd2: key_pin2 = 1'b0;
        3'd3: key_pin3 = 1'b0;
        3'd4: key_pin4 = 1'b0;
        default: key_pin5 = 1'b0;
      endcase
      #25000000;
      case (key_idx)
        3'd1: key_pin1 = 1'b1;
        3'd2: key_pin2 = 1'b1;
        3'd3: key_pin3 = 1'b1;
        3'd4: key_pin4 = 1'b1;
        default: key_pin5 = 1'b1;
      endcase
      #5000000;
    end
  endtask

  always @(*) motor_lf_enc_b = motor_lf_in2;
  always @(*) motor_rf_enc_b = motor_rf_in2;
  always @(*) motor_lr_enc_b = motor_lr_in2;
  always @(*) motor_rr_enc_b = motor_rr_in2;

  always begin
    #100;
    if (motor_stby_a && (motor_lf_in1 ^ motor_lf_in2) && motor_lf_pwm)
      motor_lf_enc_a = ~motor_lf_enc_a;
    else
      motor_lf_enc_a = 1'b0;
  end

  always begin
    #100;
    if (motor_stby_a && (motor_rf_in1 ^ motor_rf_in2) && motor_rf_pwm)
      motor_rf_enc_a = ~motor_rf_enc_a;
    else
      motor_rf_enc_a = 1'b0;
  end

  always begin
    #100;
    if (motor_stby_b && (motor_lr_in1 ^ motor_lr_in2) && motor_lr_pwm)
      motor_lr_enc_a = ~motor_lr_enc_a;
    else
      motor_lr_enc_a = 1'b0;
  end

  always begin
    #100;
    if (motor_stby_b && (motor_rr_in1 ^ motor_rr_in2) && motor_rr_pwm)
      motor_rr_enc_a = ~motor_rr_enc_a;
    else
      motor_rr_enc_a = 1'b0;
  end

  initial
  begin
    rst_n = 1'b0;
    ultrasonic_echo = 1'b0;
    key_pin1 = 1'b1;
    key_pin2 = 1'b1;
    key_pin3 = 1'b1;
    key_pin4 = 1'b1;
    key_pin5 = 1'b1;
    motor_lf_enc_a = 1'b0;
    motor_rf_enc_a = 1'b0;
    motor_lr_enc_a = 1'b0;
    motor_rr_enc_a = 1'b0;

    #500;
    rst_n = 1'b1;

    press_key(3'd5);
    press_key(3'd1);
    press_key(3'd3);
    press_key(3'd5);
    press_key(3'd4);
    press_key(3'd5);

    #20000000;
    $finish;
  end

  initial begin
    $dumpfile("tb/tb_sys_route_gui_fsm_top_robei.vcd");
    $dumpvars(0, sys_route_gui_fsm_top_test);
  end
//---Module instantiation---
  sys_route_gui_fsm_top sys_route_gui_fsm_top0(
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
    .ultrasonic_trig(ultrasonic_trig),
    .buzzer_out(buzzer_out)
);

endmodule    //sys_route_gui_fsm_top_test

`timescale 1ns/1ps

module tb_c5_spin_rate_pi;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg calibration_done = 1'b1;
  reg fused_sample_valid = 1'b0;
  reg [15:0] fused_gyro_z = 16'd0;
  reg [10:0] base_lf_speed = 11'd0;
  reg [10:0] base_rf_speed = 11'd0;
  reg [10:0] base_lr_speed = 11'd0;
  reg [10:0] base_rr_speed = 11'd0;
  wire [10:0] lf_speed;
  wire [10:0] rf_speed;
  wire [10:0] lr_speed;
  wire [10:0] rr_speed;
  wire gyro_polarity_valid;
  wire gyro_positive_is_right;

  integer index;
  integer signed heading_accumulator;
  integer left_rate;
  integer right_rate;

  always #10 clk = ~clk;

  c5_motion_stabilizer dut (
    .clk(clk), .rst_n(rst_n), .calibration_done(calibration_done),
    .fused_sample_valid(fused_sample_valid), .imu_degraded(1'b0),
    .imu_disagreement(1'b0), .fused_gyro_z(fused_gyro_z),
    .flow_sample_valid(1'b0), .flow_forward_q8(16'd0),
    .flow_lateral_q8(16'd0), .local_calibration_start(1'b0),
    .external_force_stop(1'b0), .obstacle_stop(1'b0),
    .cal_lf_default_speed(10'd700), .cal_rf_default_speed(10'd700),
    .cal_lr_default_speed(10'd700), .cal_rr_default_speed(10'd700),
    .base_lf_speed(base_lf_speed), .base_rf_speed(base_rf_speed),
    .base_lr_speed(base_lr_speed), .base_rr_speed(base_rr_speed),
    .lf_speed(lf_speed), .rf_speed(rf_speed), .lr_speed(lr_speed),
    .rr_speed(rr_speed), .correction_active(),
    .gyro_polarity_valid(gyro_polarity_valid),
    .gyro_positive_is_right(gyro_positive_is_right),
    .yaw_correction(), .flow_correction(), .force_stop(),
    .local_calibration_state(), .local_calibration_flags(),
    .learned_yaw_trim(), .calibration_gyro_peak()
  );

  task command_right;
    begin
      base_lf_speed = 11'sd700;
      base_rf_speed = -11'sd700;
      base_lr_speed = 11'sd700;
      base_rr_speed = -11'sd700;
    end
  endtask

  task command_left;
    begin
      base_lf_speed = -11'sd700;
      base_rf_speed = 11'sd700;
      base_lr_speed = -11'sd700;
      base_rr_speed = 11'sd700;
    end
  endtask

  task command_stop;
    begin
      base_lf_speed = 11'sd0;
      base_rf_speed = 11'sd0;
      base_lr_speed = 11'sd0;
      base_rr_speed = 11'sd0;
    end
  endtask

  task sample_rate;
    input integer signed rate;
    begin
      fused_gyro_z = rate[15:0];
      fused_sample_valid = 1'b1;
      @(posedge clk);
      #1;
      fused_sample_valid = 1'b0;
      @(posedge clk);
      #1;
    end
  endtask

  initial begin
    repeat (4) @(posedge clk);
    rst_n = 1'b1;

    // Learn physical gyro polarity from the healthy right-turn direction.
    command_right();
    for (index = 0; index < 16; index = index + 1)
      sample_rate(5600);
    if (!gyro_polarity_valid || !gyro_positive_is_right)
      $fatal(1, "right-turn polarity learning failed");

    // The simulated chassis is 12.5 percent weaker when turning left:
    // right rate = wheel magnitude * 8, left rate = wheel magnitude * 7.
    command_left();
    for (index = 0; index < 120; index = index + 1) begin
      left_rate = -($signed(lf_speed) * -7);
      sample_rate(left_rate);
    end
    $display("TRACE left_control target=%0d correction=%0d command=%0d",
             dut.spin_common_rate_reference,
             $signed(dut.correction_reg), -$signed(lf_speed));
    $display("TRACE pid active=%0d correction_active=%0d error=%0d sampled=%0d integral=%0d direction=%0d/%0d",
             dut.spin_pid_active, dut.correction_active,
             $signed(dut.selected_pid_error), $signed(dut.sampled_error),
             $signed(dut.error_integral), dut.spin_direction_right,
             dut.spinning_right);
    if (-$signed(lf_speed) <= 11'sd700)
      $fatal(1, "PI did not boost the weak left turn");
    if ((-left_rate < 5200) || (-left_rate > 5900))
      $fatal(1, "left turn did not converge near the shared rate: %0d", -left_rate);

    // Stopping must clear the PI output immediately.  A later right turn must
    // start from the configured base command without stale left-turn integral.
    command_stop();
    @(posedge clk);
    #1;
    if ($signed(lf_speed) != 0 || $signed(rf_speed) != 0)
      $fatal(1, "stop retained a spin correction");
    command_right();
    @(posedge clk);
    #1;
    if ($signed(lf_speed) != 700 || $signed(rf_speed) != -700)
      $fatal(1, "direction change retained stale integral");

    // Equal-duration turns after learning should nearly cancel despite the
    // asymmetric plant.  This models the user's repeated left/right test.
    heading_accumulator = 0;
    for (index = 0; index < 100; index = index + 1) begin
      right_rate = $signed(lf_speed) * 8;
      heading_accumulator = heading_accumulator + right_rate;
      sample_rate(right_rate);
    end
    command_stop();
    @(posedge clk);
    command_left();
    for (index = 0; index < 100; index = index + 1) begin
      left_rate = -($signed(lf_speed) * -7);
      heading_accumulator = heading_accumulator + left_rate;
      sample_rate(left_rate);
    end
    $display("TRACE cancellation target=%0d left_cmd=%0d error=%0d",
             dut.spin_common_rate_reference,
             -$signed(lf_speed), heading_accumulator);
    if ((heading_accumulator > 45000) || (heading_accumulator < -45000))
      $fatal(1, "equal left/right turns still drift: %0d", heading_accumulator);

    $display("RESULT spin_rate_target=%0d", dut.spin_common_rate_reference);
    $display("RESULT weak_left_command=%0d final_heading_error=%0d",
             -$signed(lf_speed), heading_accumulator);
    $display("RESULT TEST_PASS c5_spin_rate_pi");
    $finish;
  end
endmodule

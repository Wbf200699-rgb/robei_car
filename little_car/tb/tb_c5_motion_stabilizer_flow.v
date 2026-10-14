`timescale 1ns/1ps
`define C5_CAL_SIM_FAST

module tb_c5_motion_stabilizer_flow;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg calibration_done = 1'b1;
  reg fused_sample_valid = 1'b0;
  reg signed [15:0] fused_gyro_z = 16'sd0;
  reg flow_sample_valid = 1'b0;
  reg signed [15:0] flow_forward_q8 = 16'sd0;
  reg signed [15:0] flow_lateral_q8 = 16'sd0;
  reg signed [10:0] base_lf_speed = 11'sd0;
  reg signed [10:0] base_rf_speed = 11'sd0;
  reg signed [10:0] base_lr_speed = 11'sd0;
  reg signed [10:0] base_rr_speed = 11'sd0;
  wire signed [10:0] lf_speed;
  wire signed [10:0] rf_speed;
  wire signed [10:0] lr_speed;
  wire signed [10:0] rr_speed;
  wire signed [10:0] flow_correction;
  integer index;

  always #10 clk = ~clk;

  c5_motion_stabilizer dut (
    .clk(clk), .rst_n(rst_n), .calibration_done(calibration_done),
    .fused_sample_valid(fused_sample_valid),
    .imu_degraded(1'b0), .imu_disagreement(1'b0),
    .fused_gyro_z(fused_gyro_z),
    .flow_sample_valid(flow_sample_valid),
    .flow_forward_q8(flow_forward_q8),
    .flow_lateral_q8(flow_lateral_q8),
    .local_calibration_start(1'b0),
    .external_force_stop(1'b0), .obstacle_stop(1'b0),
    .cal_lf_default_speed(10'd700), .cal_rf_default_speed(10'd700),
    .cal_lr_default_speed(10'd700), .cal_rr_default_speed(10'd700),
    .base_lf_speed(base_lf_speed), .base_rf_speed(base_rf_speed),
    .base_lr_speed(base_lr_speed), .base_rr_speed(base_rr_speed),
    .lf_speed(lf_speed), .rf_speed(rf_speed),
    .lr_speed(lr_speed), .rr_speed(rr_speed),
    .correction_active(), .gyro_polarity_valid(),
    .gyro_positive_is_right(), .yaw_correction(),
    .flow_correction(flow_correction), .force_stop(),
    .local_calibration_state(), .local_calibration_flags(),
    .learned_yaw_trim(), .calibration_gyro_peak()
  );

  task gyro_sample;
    input signed [15:0] value;
    begin
      @(negedge clk);
      fused_gyro_z = value;
      fused_sample_valid = 1'b1;
      @(negedge clk);
      fused_sample_valid = 1'b0;
    end
  endtask

  task flow_sample;
    input signed [15:0] forward_value;
    input signed [15:0] lateral_value;
    begin
      @(negedge clk);
      fused_gyro_z = 16'sd0;
      fused_sample_valid = 1'b1;
      flow_forward_q8 = forward_value;
      flow_lateral_q8 = lateral_value;
      flow_sample_valid = 1'b1;
      @(negedge clk);
      fused_sample_valid = 1'b0;
      flow_sample_valid = 1'b0;
    end
  endtask

  initial begin
    repeat (4) @(posedge clk);
    rst_n = 1'b1;

    // Prime the fused-stream age flag. The first sample after reset makes the
    // sensor stream healthy; subsequent turn samples are eligible for sign
    // learning.
    gyro_sample(16'sd0);

    // Learn that positive gyro is a physical right turn.
    base_lf_speed = 11'sd650;
    base_rf_speed = -11'sd650;
    base_lr_speed = 11'sd650;
    base_rr_speed = -11'sd650;
    for (index = 0; index < 4; index = index + 1)
      gyro_sample(16'sd116);
    if (!dut.gyro_polarity_valid)
      $fatal(1, "gyro polarity learning failed");

    base_lf_speed = 11'sd700;
    base_rf_speed = 11'sd700;
    base_lr_speed = 11'sd700;
    base_rr_speed = 11'sd700;
    repeat (2) @(posedge clk);

    // Positive rightward drift while driving forward must request a negative
    // differential correction. Four windows also verify the slew limiter.
    for (index = 0; index < 64; index = index + 1)
      flow_sample(16'sd8, 16'sd8);
    repeat (3) @(posedge clk);
    if ($signed(flow_correction) >= 0 || $signed(flow_correction) < -32)
      $fatal(1, "forward lateral correction sign/limit invalid: %0d",
             $signed(flow_correction));
    if (($signed(lf_speed) >= 700) || ($signed(rf_speed) <= 700))
      $fatal(1, "flow correction was not applied to wheel differential");

    // Keep IMU health alive while withholding optical samples. The independent
    // flow watchdog must clear stale correction after its 96-cycle test limit.
    fused_sample_valid = 1'b1;
    repeat (100) @(posedge clk);
    fused_sample_valid = 1'b0;
    repeat (2) @(posedge clk);
    if ($signed(flow_correction) != 0)
      $fatal(1, "stale flow correction did not return to zero: %0d",
             $signed(flow_correction));

    $display("RESULT flow_sign=right_drift_to_negative_correction");
    $display("RESULT flow_limit=32 timeout_cycles=96");
    $display("RESULT TEST_PASS c5_motion_stabilizer_flow");
    $finish;
  end
endmodule

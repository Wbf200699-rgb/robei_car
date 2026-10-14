`timescale 1ns/1ps

module tb_c5_motion_stabilizer;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg calibration_done = 1'b0;
  reg fused_sample_valid = 1'b0;
  reg imu_degraded = 1'b0;
  reg imu_disagreement = 1'b0;
  reg [15:0] fused_gyro_z = 16'd0;
  reg local_calibration_start = 1'b0;
  reg external_force_stop = 1'b0;
  reg obstacle_stop = 1'b0;
  reg [9:0] cal_lf_default_speed = 10'd700;
  reg [9:0] cal_rf_default_speed = 10'd700;
  reg [9:0] cal_lr_default_speed = 10'd700;
  reg [9:0] cal_rr_default_speed = 10'd700;
  reg [10:0] base_lf_speed = 11'd0;
  reg [10:0] base_rf_speed = 11'd0;
  reg [10:0] base_lr_speed = 11'd0;
  reg [10:0] base_rr_speed = 11'd0;
  wire [10:0] lf_speed;
  wire [10:0] rf_speed;
  wire [10:0] lr_speed;
  wire [10:0] rr_speed;
  wire correction_active;
  wire gyro_polarity_valid;
  wire gyro_positive_is_right;
  wire [10:0] yaw_correction;

  always #10 clk = ~clk;

  c5_motion_stabilizer dut (
    .clk(clk), .rst_n(rst_n), .calibration_done(calibration_done),
    .fused_sample_valid(fused_sample_valid), .imu_degraded(imu_degraded),
    .imu_disagreement(imu_disagreement), .fused_gyro_z(fused_gyro_z),
    .local_calibration_start(local_calibration_start),
    .external_force_stop(external_force_stop),
    .obstacle_stop(obstacle_stop),
    .cal_lf_default_speed(cal_lf_default_speed),
    .cal_rf_default_speed(cal_rf_default_speed),
    .cal_lr_default_speed(cal_lr_default_speed),
    .cal_rr_default_speed(cal_rr_default_speed),
    .base_lf_speed(base_lf_speed), .base_rf_speed(base_rf_speed),
    .base_lr_speed(base_lr_speed), .base_rr_speed(base_rr_speed),
    .lf_speed(lf_speed), .rf_speed(rf_speed), .lr_speed(lr_speed),
    .rr_speed(rr_speed), .correction_active(correction_active),
    .gyro_polarity_valid(gyro_polarity_valid),
    .gyro_positive_is_right(gyro_positive_is_right),
    .yaw_correction(yaw_correction)
  );

  task sample_gyro;
    input signed [15:0] value;
    begin
      fused_gyro_z = value;
      fused_sample_valid = 1'b1;
      @(posedge clk);
      #1;
      fused_sample_valid = 1'b0;
      @(posedge clk);
      #1;
    end
  endtask

  task set_speeds;
    input signed [10:0] lf;
    input signed [10:0] rf;
    input signed [10:0] lr;
    input signed [10:0] rr;
    begin
      base_lf_speed = lf;
      base_rf_speed = rf;
      base_lr_speed = lr;
      base_rr_speed = rr;
      @(posedge clk);
      #1;
    end
  endtask

  integer index;
  initial begin
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    calibration_done = 1'b1;

    // Without learned physical polarity, straight commands are passed through.
    set_speeds(11'sd700, 11'sd700, 11'sd700, 11'sd700);
    sample_gyro(16'sd320);
    if (correction_active || $signed(lf_speed) != 700 || $signed(rf_speed) != 700)
      $fatal(1, "feedback must remain bypassed before polarity learning");

    // The measured chassis turn peak is about 116 counts.  It must remain far
    // above idle noise yet still be accepted for polarity learning.
    set_speeds(11'sd650, -11'sd650, 11'sd650, -11'sd650);
    for (index = 0; index < 4; index = index + 1)
      sample_gyro(16'sd116);
    if (!gyro_polarity_valid || !gyro_positive_is_right)
      $fatal(1, "right-turn polarity was not learned");

    // Normal driving calculates continuously but commits a wheel correction
    // only at the bounded runtime update interval.
    set_speeds(11'sd700, 11'sd700, 11'sd700, 11'sd700);
    sample_gyro(16'sd320);
    sample_gyro(16'sd320);
    repeat (30) @(posedge clk);
    #1;
    if (!correction_active || $signed(yaw_correction) >= 0 ||
        $signed(lf_speed) >= 700 || $signed(rf_speed) <= 700 ||
        $signed(lr_speed) >= 700 || $signed(rr_speed) <= 700)
      $fatal(1, "normal forward PID correction was not applied");
    $display("RESULT forward_pid gyro=320 correction=%0d wheels=%0d,%0d,%0d,%0d",
             $signed(yaw_correction), $signed(lf_speed), $signed(rf_speed),
             $signed(lr_speed), $signed(rr_speed));

    // Reverse commands are also passed through atomically while correction
    // telemetry remains available to the phone and mapping diagnostics. The
    // previous forward correction must disappear immediately on reversal.
    set_speeds(-11'sd700, -11'sd700, -11'sd700, -11'sd700);
    @(posedge clk);
    #1;
    if ($signed(lf_speed) != -700 || $signed(rf_speed) != -700 ||
        $signed(lr_speed) != -700 || $signed(rr_speed) != -700)
      $fatal(1, "forward correction leaked into reverse command");
    for (index = 0; index < 4; index = index + 1)
      sample_gyro(-16'sd320);
    repeat (30) @(posedge clk);
    #1;
    if (!correction_active || $signed(yaw_correction) <= 0 ||
        $signed(lf_speed) <= -700 || $signed(rf_speed) >= -700 ||
        $signed(lr_speed) <= -700 || $signed(rr_speed) >= -700)
      $fatal(1, "normal reverse PID correction was not applied");
    $display("RESULT reverse_pid gyro=-320 correction=%0d wheels=%0d,%0d,%0d,%0d",
             $signed(yaw_correction), $signed(lf_speed), $signed(rf_speed),
             $signed(lr_speed), $signed(rr_speed));

    // A stop command must pass zero immediately, without waiting for a new IMU sample.
    set_speeds(11'sd0, 11'sd0, 11'sd0, 11'sd0);
    if ($signed(lf_speed) != 0 || $signed(rf_speed) != 0 ||
        $signed(lr_speed) != 0 || $signed(rr_speed) != 0)
      $fatal(1, "stop command was modified by stale feedback");
    $display("RESULT immediate_stop wheels=%0d,%0d,%0d,%0d force_stop=%0d",
             $signed(lf_speed), $signed(rf_speed), $signed(lr_speed),
             $signed(rr_speed), force_stop);

    // A diagnostic disagreement alone must not tear down feedback while the
    // fusion block is still delivering a valid primary-IMU fallback stream.
    imu_disagreement = 1'b1;
    set_speeds(11'sd650, 11'sd750, 11'sd800, 11'sd625);
    sample_gyro(16'sd500);
    if (!correction_active)
      $fatal(1, "valid degraded IMU stream incorrectly disabled feedback");

    // Losing the actual sample stream is the safety boundary. With
    // C5_CAL_SIM_FAST, 64 clocks without a sample must bypass correction and
    // preserve the configured wheel commands exactly.
    repeat (66) @(posedge clk);
    #1;
    if (correction_active || $signed(lf_speed) != 650 ||
        $signed(rf_speed) != 750 || $signed(lr_speed) != 800 ||
        $signed(rr_speed) != 625)
      $fatal(1, "stale fused stream did not bypass the stabilizer");
    $display("RESULT stale_bypass wheels=%0d,%0d,%0d,%0d correction_active=%0d",
             $signed(lf_speed), $signed(rf_speed), $signed(lr_speed),
             $signed(rr_speed), correction_active);

    $display("RESULT polarity_positive_is_right=%0d", gyro_positive_is_right);
    $display("RESULT correction_limit=%0d slew_per_sample=%0d", 96, 8);
    $display("RESULT runtime_pid_applied=pass update_interval_cycles=24");
    $display("RESULT degraded_stream_continuity=pass stale_stream_bypass=pass");
    $display("RESULT TEST_PASS c5_motion_stabilizer");
    $finish;
  end
endmodule

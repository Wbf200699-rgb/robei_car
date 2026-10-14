`timescale 1ns/1ps
`define C5_CAL_SIM_FAST

module tb_c5_local_calibration;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg calibration_done = 1'b1;
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
  wire force_stop;
  wire [3:0] local_calibration_state;
  wire [7:0] local_calibration_flags;
  wire [10:0] learned_yaw_trim;
  wire [15:0] calibration_gyro_peak;
  integer left_average;
  integer right_average;
  integer differential_yaw;
  integer chassis_bias;
  reg [2:0] sample_divider = 3'd0;
  reg sample_stream_enabled = 1'b1;

  c5_motion_stabilizer dut (
    .clk(clk),
    .rst_n(rst_n),
    .calibration_done(calibration_done),
    .fused_sample_valid(fused_sample_valid),
    .imu_degraded(imu_degraded),
    .imu_disagreement(imu_disagreement),
    .fused_gyro_z(fused_gyro_z),
    .local_calibration_start(local_calibration_start),
    .external_force_stop(external_force_stop),
    .obstacle_stop(obstacle_stop),
    .cal_lf_default_speed(cal_lf_default_speed),
    .cal_rf_default_speed(cal_rf_default_speed),
    .cal_lr_default_speed(cal_lr_default_speed),
    .cal_rr_default_speed(cal_rr_default_speed),
    .base_lf_speed(base_lf_speed),
    .base_rf_speed(base_rf_speed),
    .base_lr_speed(base_lr_speed),
    .base_rr_speed(base_rr_speed),
    .lf_speed(lf_speed),
    .rf_speed(rf_speed),
    .lr_speed(lr_speed),
    .rr_speed(rr_speed),
    .correction_active(correction_active),
    .gyro_polarity_valid(gyro_polarity_valid),
    .gyro_positive_is_right(gyro_positive_is_right),
    .yaw_correction(yaw_correction),
    .force_stop(force_stop),
    .local_calibration_state(local_calibration_state),
    .local_calibration_flags(local_calibration_flags),
    .learned_yaw_trim(learned_yaw_trim),
    .calibration_gyro_peak(calibration_gyro_peak)
  );

  always #5 clk = ~clk;

  // Close the wheel outputs back into the next gyro sample. A persistent
  // forward chassis bias can therefore shrink as the DUT iterates instead of
  // presenting an impossible fixed error that no controller could correct.
  always @(negedge clk) begin
    if (!rst_n) begin
      sample_divider = 3'd0;
      fused_sample_valid <= 1'b0;
      fused_gyro_z <= 16'd0;
    end else if (!sample_stream_enabled) begin
      fused_sample_valid <= 1'b0;
    end else if (sample_divider == 3'd0) begin
      sample_divider = 3'd7;
      fused_sample_valid <= 1'b1;
      left_average = ($signed(lf_speed) + $signed(lr_speed)) / 2;
      right_average = ($signed(rf_speed) + $signed(rr_speed)) / 2;
      differential_yaw = (left_average - right_average) * 8;
      if (($signed(lf_speed) > 0) && ($signed(rf_speed) > 0) &&
          ($signed(lr_speed) > 0) && ($signed(rr_speed) > 0))
        chassis_bias = 320;
      else if (($signed(lf_speed) < 0) && ($signed(rf_speed) < 0) &&
               ($signed(lr_speed) < 0) && ($signed(rr_speed) < 0))
        chassis_bias = -320;
      else
        chassis_bias = 0;
      fused_gyro_z <= differential_yaw + chassis_bias;
    end else begin
      sample_divider = sample_divider - 3'd1;
      fused_sample_valid <= 1'b0;
    end
  end

  task pulse_start;
    begin
      @(negedge clk);
      local_calibration_start = 1'b1;
      @(negedge clk);
      local_calibration_start = 1'b0;
    end
  endtask

  task wait_state;
    input [3:0] wanted;
    integer timeout;
    begin
      timeout = 0;
      while ((local_calibration_state != wanted) && (timeout < 5000)) begin
        @(posedge clk);
        timeout = timeout + 1;
      end
      if (local_calibration_state != wanted) begin
        $display("FAIL timeout waiting for state %h, current=%h", wanted,
                 local_calibration_state);
        $fatal(1);
      end
    end
  endtask

  initial begin
    $dumpfile("little_car/build/tb_c5_local_calibration.vcd");
    $dumpvars(0, tb_c5_local_calibration);
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    wait (fused_sample_valid);
    @(posedge clk);

    pulse_start();
    wait_state(4'h2);
    if ($signed(lf_speed) != -750 || $signed(rf_speed) != 750 ||
        $signed(lr_speed) != -750 || $signed(rr_speed) != 750) begin
      $display("FAIL left spin signs %0d,%0d,%0d,%0d", $signed(lf_speed),
               $signed(rf_speed), $signed(lr_speed), $signed(rr_speed));
      $fatal(1);
    end

    // Intentional rotation can briefly trip the dual-IMU dynamic difference
    // threshold.  That transient must suppress bad samples without aborting
    // the complete local calibration sequence.
    @(negedge clk);
    imu_disagreement = 1'b1;
    imu_degraded = 1'b1;
    repeat (4) @(negedge clk);
    imu_disagreement = 1'b0;
    imu_degraded = 1'b0;
    if (local_calibration_state == 4'hB) begin
      $display("FAIL transient IMU health flag aborted calibration");
      $fatal(1);
    end

    wait_state(4'h4);
    if ($signed(lf_speed) != 750 || $signed(rf_speed) != -750 ||
        $signed(lr_speed) != 750 || $signed(rr_speed) != -750) begin
      $display("FAIL right spin signs %0d,%0d,%0d,%0d", $signed(lf_speed),
               $signed(rf_speed), $signed(lr_speed), $signed(rr_speed));
      $fatal(1);
    end

    wait_state(4'hA);
    if (!local_calibration_flags[1] || local_calibration_flags[2] ||
        !gyro_polarity_valid || ($signed(learned_yaw_trim) >= 0) ||
        (calibration_gyro_peak < 16'd500)) begin
      $display("FAIL result flags=%02x polarity=%0d trim=%0d peak=%0d",
               local_calibration_flags, gyro_polarity_valid,
               $signed(learned_yaw_trim), calibration_gyro_peak);
      $fatal(1);
    end
    if ($signed(lf_speed) != 0 || $signed(rf_speed) != 0 ||
        $signed(lr_speed) != 0 || $signed(rr_speed) != 0) begin
      $display("FAIL calibration did not finish stopped");
      $fatal(1);
    end

    // Verify the learned baseline is actually applied to a later straight
    // command, rather than being telemetry-only state.
    base_lf_speed = 11'd700;
    base_rf_speed = 11'd700;
    base_lr_speed = 11'd700;
    base_rr_speed = 11'd700;
    fused_gyro_z = 16'd0;
    repeat (6) @(posedge clk);
    if ($signed(lf_speed) != 700 || $signed(rf_speed) != 700 ||
        $signed(lr_speed) != 700 || $signed(rr_speed) != 700) begin
      $display("FAIL learned correction bypassed runtime rate limit");
      $fatal(1);
    end
    repeat (24) @(posedge clk);
    if (!correction_active || ($signed(yaw_correction) >= 0) ||
        !($signed(lf_speed) < $signed(rf_speed))) begin
      $display("FAIL learned correction inactive corr=%0d wheels=%0d/%0d",
               $signed(yaw_correction), $signed(lf_speed), $signed(rf_speed));
      $fatal(1);
    end

    base_lf_speed = 11'd0;
    base_rf_speed = 11'd0;
    base_lr_speed = 11'd0;
    base_rr_speed = 11'd0;

    // A brief obstacle during the forward measurement must stop the chassis,
    // discard the partial measurement, settle, and resume instead of failing.
    pulse_start();
    wait_state(4'h6);
    @(negedge clk);
    obstacle_stop = 1'b1;
    @(posedge clk);
    #1;
    if ($signed(lf_speed) != 0 || $signed(rf_speed) != 0 ||
        $signed(lr_speed) != 0 || $signed(rr_speed) != 0 || !force_stop) begin
      $display("FAIL obstacle did not stop calibration immediately");
      $fatal(1);
    end
    repeat (10) @(posedge clk);
    if (local_calibration_state != 4'h6) begin
      $display("FAIL transient obstacle aborted state=%h",
               local_calibration_state);
      $fatal(1);
    end
    @(negedge clk);
    obstacle_stop = 1'b0;
    repeat (25) @(posedge clk);
    if (local_calibration_state != 4'h6 || $signed(lf_speed) <= 0 ||
        $signed(rf_speed) <= 0) begin
      $display("FAIL calibration did not resume after clear state=%h wheels=%0d/%0d",
               local_calibration_state, $signed(lf_speed), $signed(rf_speed));
      $fatal(1);
    end
    wait_state(4'hA);

    // A transient sample gap models a range audit or one-channel recovery.
    // The chassis must stop, collect a fresh run of samples, and redo only the
    // interrupted phase instead of restarting the complete calibration round.
    pulse_start();
    wait_state(4'h2);
    sample_stream_enabled = 1'b0;
    repeat (70) @(posedge clk);
    #1;
    if (!dut.calibration_sensor_pause || local_calibration_state == 4'hD ||
        $signed(lf_speed) != 0 || $signed(rf_speed) != 0 ||
        $signed(lr_speed) != 0 || $signed(rr_speed) != 0) begin
      $display("FAIL transient sample gap was not held safely state=%h pause=%0d",
               local_calibration_state, dut.calibration_sensor_pause);
      $fatal(1);
    end
    sample_stream_enabled = 1'b1;
    wait_state(4'h1);
    if (dut.calibration_sensor_pause) begin
      $display("FAIL sample stream recovery remained paused");
      $fatal(1);
    end
    wait_state(4'h2);
    wait_state(4'hA);

    // Reproduce the field failure: if the stream pauses during the forward
    // trim phase, recovery must return to forward sampling. Returning to the
    // initial left spin would make periodic sensor maintenance loop forever.
    pulse_start();
    wait_state(4'h6);
    sample_stream_enabled = 1'b0;
    repeat (70) @(posedge clk);
    #1;
    if (!dut.calibration_sensor_pause || local_calibration_state == 4'hD ||
        $signed(lf_speed) != 0 || $signed(rf_speed) != 0 ||
        $signed(lr_speed) != 0 || $signed(rr_speed) != 0) begin
      $display("FAIL forward sample gap was not held safely state=%h pause=%0d",
               local_calibration_state, dut.calibration_sensor_pause);
      $fatal(1);
    end
    sample_stream_enabled = 1'b1;
    wait_state(4'h1);
    wait_state(4'h6);
    wait_state(4'hA);

    // Starting during BMI088 startup calibration must wait safely rather than
    // fail on the command cycle. Once consecutive fused samples return, the
    // normal left-spin phase starts without requiring a second App command.
    calibration_done = 1'b0;
    pulse_start();
    repeat (20) @(posedge clk);
    #1;
    if (local_calibration_state != 4'h1 ||
        !dut.calibration_sensor_pause || $signed(lf_speed) != 0 ||
        $signed(rf_speed) != 0 || $signed(lr_speed) != 0 ||
        $signed(rr_speed) != 0) begin
      $display("FAIL startup sensor wait state=%h pause=%0d",
               local_calibration_state, dut.calibration_sensor_pause);
      $fatal(1);
    end
    calibration_done = 1'b1;
    wait (!dut.calibration_sensor_pause);
    wait_state(4'h2);
    wait_state(4'hA);

    // A path that remains blocked beyond the bounded wait must still fail
    // safely, proving that the transient handling does not bypass avoidance.
    pulse_start();
    wait_state(4'h6);
    @(negedge clk);
    obstacle_stop = 1'b1;
    wait_state(4'hB);
    obstacle_stop = 1'b0;
    if (!local_calibration_flags[2] || local_calibration_flags[0]) begin
      $display("FAIL persistent-obstacle flags=%02x", local_calibration_flags);
      $fatal(1);
    end

    // Losing the actual sample pulse must produce a distinct sensor failure
    // before the state machine can continue moving open-loop.
    pulse_start();
    wait_state(4'h2);
    sample_stream_enabled = 1'b0;
    wait_state(4'hD);
    if (!local_calibration_flags[2] || local_calibration_flags[0]) begin
      $display("FAIL sensor-timeout flags=%02x", local_calibration_flags);
      $fatal(1);
    end
    sample_stream_enabled = 1'b1;
    wait (fused_sample_valid);
    @(posedge clk);

    pulse_start();
    wait_state(4'h2);
    @(negedge clk);
    external_force_stop = 1'b1;
    #1;
    if (!force_stop) begin
      $display("FAIL external stop was not forwarded immediately");
      $fatal(1);
    end
    @(negedge clk);
    external_force_stop = 1'b0;
    wait_state(4'hB);
    if (!local_calibration_flags[2] || local_calibration_flags[0]) begin
      $display("FAIL abort flags=%02x", local_calibration_flags);
      $fatal(1);
    end

    $display("RESULT learned_yaw_trim=%0d gyro_peak=%0d flags=%02x",
             $signed(learned_yaw_trim), calibration_gyro_peak,
             local_calibration_flags);
    $display("RESULT TEST_PASS c5_local_calibration");
    $finish;
  end
endmodule

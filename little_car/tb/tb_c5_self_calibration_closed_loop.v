`timescale 1ns/1ps
`define C5_CAL_SIM_FAST

module tb_c5_self_calibration_closed_loop;
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

  reg [2:0] sample_divider = 3'd0;
  reg signed [15:0] plant_yaw_rate = 16'sd0;
  integer left_average;
  integer right_average;
  integer differential_yaw;
  integer chassis_bias;
  integer baseline_abs_sum;
  integer final_abs_sum;
  integer final_abs_max;
  integer settled_correction;
  integer sample_index;
  integer timeout;

  localparam integer YAW_PER_WHEEL_DELTA = 8;
  localparam integer FORWARD_CHASSIS_BIAS = 320;

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

  function integer abs_integer;
    input integer value;
    begin
      abs_integer = (value < 0) ? -value : value;
    end
  endfunction

  // This is a deliberately small deterministic chassis model, not a second
  // controller. A left/right command difference creates yaw, while equal
  // forward commands retain a fixed rightward mechanical bias. The DUT output
  // therefore changes the next IMU sample and closes the simulated loop.
  always @(negedge clk) begin
    if (!rst_n) begin
      sample_divider = 3'd0;
      fused_sample_valid = 1'b0;
      fused_gyro_z = 16'd0;
      plant_yaw_rate = 16'sd0;
    end else if (sample_divider == 3'd0) begin
      sample_divider = 3'd7;
      left_average = ($signed(lf_speed) + $signed(lr_speed)) / 2;
      right_average = ($signed(rf_speed) + $signed(rr_speed)) / 2;
      differential_yaw = (left_average - right_average) *
                         YAW_PER_WHEEL_DELTA;
      if (($signed(lf_speed) > 0) && ($signed(rf_speed) > 0) &&
          ($signed(lr_speed) > 0) && ($signed(rr_speed) > 0))
        chassis_bias = FORWARD_CHASSIS_BIAS;
      else if (($signed(lf_speed) < 0) && ($signed(rf_speed) < 0) &&
               ($signed(lr_speed) < 0) && ($signed(rr_speed) < 0))
        chassis_bias = -FORWARD_CHASSIS_BIAS;
      else
        chassis_bias = 0;
      plant_yaw_rate = differential_yaw + chassis_bias;
      fused_gyro_z = plant_yaw_rate;
      fused_sample_valid = 1'b1;
    end else begin
      sample_divider = sample_divider - 3'd1;
      fused_sample_valid = 1'b0;
    end
  end

  task set_base_speed;
    input signed [10:0] value;
    begin
      base_lf_speed = value;
      base_rf_speed = value;
      base_lr_speed = value;
      base_rr_speed = value;
    end
  endtask

  task pulse_calibration_start;
    begin
      @(negedge clk);
      local_calibration_start = 1'b1;
      @(negedge clk);
      local_calibration_start = 1'b0;
    end
  endtask

  task wait_for_sample;
    begin
      @(posedge fused_sample_valid);
      #1;
    end
  endtask

  initial begin
    $dumpfile("little_car/build/tb_c5_self_calibration_closed_loop.vcd");
    $dumpvars(0, tb_c5_self_calibration_closed_loop);

    repeat (4) @(posedge clk);
    rst_n = 1'b1;

    // Establish the uncompensated baseline while polarity learning still
    // keeps the feedback controller bypassed.
    set_base_speed(11'sd700);
    baseline_abs_sum = 0;
    for (sample_index = 0; sample_index < 8; sample_index = sample_index + 1) begin
      wait_for_sample();
      baseline_abs_sum = baseline_abs_sum + abs_integer($signed(plant_yaw_rate));
    end
    baseline_abs_sum = baseline_abs_sum / 8;
    if (baseline_abs_sum != FORWARD_CHASSIS_BIAS || correction_active) begin
      $display("FAIL baseline yaw=%0d active=%b", baseline_abs_sum,
               correction_active);
      $fatal(1);
    end

    set_base_speed(11'sd0);
    pulse_calibration_start();
    wait (local_calibration_state == 4'h2);
    #1;
    if (($signed(lf_speed) != -750) || ($signed(rf_speed) != 750) ||
        ($signed(lr_speed) != -750) || ($signed(rr_speed) != 750) ||
        (dut.calibration_magnitude(10'd625) != 11'd750) ||
        (dut.calibration_magnitude(10'd900) != 11'd750)) begin
      $display("FAIL calibration drive magnitude wheels=%0d,%0d,%0d,%0d clamp=%0d/%0d",
               $signed(lf_speed), $signed(rf_speed),
               $signed(lr_speed), $signed(rr_speed),
               dut.calibration_magnitude(10'd625),
               dut.calibration_magnitude(10'd900));
      $fatal(1);
    end
    timeout = 0;
    while ((local_calibration_state != 4'hA) &&
           (local_calibration_state != 4'hB) && (timeout < 4000)) begin
      @(posedge clk);
      timeout = timeout + 1;
    end
    if (local_calibration_state != 4'hA) begin
      $display("FAIL calibration state=%h flags=%02x timeout=%0d",
               local_calibration_state, local_calibration_flags, timeout);
      $fatal(1);
    end
    if (!gyro_polarity_valid || !gyro_positive_is_right ||
        !$signed(learned_yaw_trim) || ($signed(learned_yaw_trim) >= 0) ||
        !local_calibration_flags[1] || local_calibration_flags[2]) begin
      $display("FAIL result polarity=%b/%b trim=%0d flags=%02x",
               gyro_polarity_valid, gyro_positive_is_right,
               $signed(learned_yaw_trim), local_calibration_flags);
      $fatal(1);
    end
    if (($signed(lf_speed) != 0) || ($signed(rf_speed) != 0) ||
        ($signed(lr_speed) != 0) || ($signed(rr_speed) != 0)) begin
      $display("FAIL calibration ended with nonzero wheels %0d,%0d,%0d,%0d",
               $signed(lf_speed), $signed(rf_speed),
               $signed(lr_speed), $signed(rr_speed));
      $fatal(1);
    end

    // Re-run the same straight command. Ignore the initial controller settling
    // samples and quantify the final 16 samples of the actual closed loop.
    set_base_speed(11'sd700);
    final_abs_sum = 0;
    final_abs_max = 0;
    for (sample_index = 0; sample_index < 64; sample_index = sample_index + 1) begin
      wait_for_sample();
      if (sample_index >= 48) begin
        final_abs_sum = final_abs_sum + abs_integer($signed(plant_yaw_rate));
        if (abs_integer($signed(plant_yaw_rate)) > final_abs_max)
          final_abs_max = abs_integer($signed(plant_yaw_rate));
      end
    end
    final_abs_sum = final_abs_sum / 16;
    if (!correction_active || (final_abs_sum * 8 >= baseline_abs_sum) ||
        (final_abs_max > 64)) begin
      $display("FAIL closed loop baseline=%0d final_avg=%0d final_max=%0d corr=%0d trim=%0d",
               baseline_abs_sum, final_abs_sum, final_abs_max,
               $signed(yaw_correction), $signed(learned_yaw_trim));
      $fatal(1);
    end
    settled_correction = $signed(yaw_correction);

    set_base_speed(11'sd0);
    @(posedge clk);
    #1;
    if (($signed(lf_speed) != 0) || ($signed(rf_speed) != 0) ||
        ($signed(lr_speed) != 0) || ($signed(rr_speed) != 0))
      $fatal(1, "stop was not immediate after closed-loop test");

    $display("RESULT baseline_yaw_abs=%0d", baseline_abs_sum);
    $display("RESULT calibrated_yaw_abs_avg=%0d max=%0d", final_abs_sum,
             final_abs_max);
    $display("RESULT learned_yaw_trim=%0d final_correction=%0d gyro_peak=%0d",
             $signed(learned_yaw_trim), settled_correction,
             calibration_gyro_peak);
    $display("RESULT TEST_PASS c5_self_calibration_closed_loop");
    $finish;
  end
endmodule

`timescale 1ns/1ps
`define C5_CAL_SIM_FAST

module tb_c5_calibration_spin_retry;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg fused_sample_valid = 1'b0;
  reg [15:0] fused_gyro_z = 16'd0;
  reg local_calibration_start = 1'b0;
  reg [2:0] sample_divider = 3'd0;
  reg [3:0] previous_state = 4'h0;
  integer left_spin_entries = 0;
  integer right_spin_entries = 0;
  integer left_average;
  integer right_average;
  integer differential_yaw;
  integer timeout;
  reg left_strong_sample_sent = 1'b0;
  reg right_strong_sample_sent = 1'b0;

  wire [10:0] lf_speed;
  wire [10:0] rf_speed;
  wire [10:0] lr_speed;
  wire [10:0] rr_speed;
  wire [3:0] local_calibration_state;
  wire [7:0] local_calibration_flags;
  wire gyro_polarity_valid;

  c5_motion_stabilizer dut (
    .clk(clk), .rst_n(rst_n), .calibration_done(1'b1),
    .fused_sample_valid(fused_sample_valid), .imu_degraded(1'b0),
    .imu_disagreement(1'b0), .fused_gyro_z(fused_gyro_z),
    .base_lf_speed(11'd0), .base_rf_speed(11'd0),
    .base_lr_speed(11'd0), .base_rr_speed(11'd0),
    .lf_speed(lf_speed), .rf_speed(rf_speed),
    .lr_speed(lr_speed), .rr_speed(rr_speed), .correction_active(),
    .gyro_polarity_valid(gyro_polarity_valid),
    .gyro_positive_is_right(), .yaw_correction(),
    .local_calibration_start(local_calibration_start),
    .external_force_stop(1'b0), .obstacle_stop(1'b0),
    .cal_lf_default_speed(10'd700), .cal_rf_default_speed(10'd700),
    .cal_lr_default_speed(10'd700), .cal_rr_default_speed(10'd700),
    .force_stop(), .local_calibration_state(local_calibration_state),
    .local_calibration_flags(local_calibration_flags),
    .learned_yaw_trim(), .calibration_gyro_peak(),
    .flow_sample_valid(1'b0), .flow_forward_q8(16'sd0),
    .flow_lateral_q8(16'sd0), .flow_correction()
  );

  always #5 clk = ~clk;

  // Count independent spin command attempts. The first attempt in each
  // direction deliberately reports zero yaw, reproducing a C5 command that
  // was transmitted but did not establish chassis rotation. The second
  // attempt exposes only one strong gyro sample. This matches the physical
  // case where wheel breakaway is obvious but the 700 ms spin contains fewer
  // than four samples above the normal polarity-learning threshold.
  always @(posedge clk) begin
    previous_state <= local_calibration_state;
    if (local_calibration_state != previous_state) begin
      if (local_calibration_state == 4'h2)
        left_spin_entries <= left_spin_entries + 1;
      if (local_calibration_state == 4'h4)
        right_spin_entries <= right_spin_entries + 1;
    end
  end

  always @(negedge clk) begin
    if (!rst_n) begin
      sample_divider = 3'd0;
      fused_sample_valid <= 1'b0;
      fused_gyro_z <= 16'd0;
    end else if (sample_divider == 3'd0) begin
      sample_divider = 3'd7;
      fused_sample_valid <= 1'b1;
      left_average = ($signed(lf_speed) + $signed(lr_speed)) / 2;
      right_average = ($signed(rf_speed) + $signed(rr_speed)) / 2;
      differential_yaw = (left_average - right_average) * 8;
      if ((local_calibration_state == 4'h2) && (left_spin_entries <= 1))
        fused_gyro_z <= 16'd0;
      else if ((local_calibration_state == 4'h2) && !left_strong_sample_sent) begin
        fused_gyro_z <= differential_yaw;
        left_strong_sample_sent <= 1'b1;
      end else if (local_calibration_state == 4'h2)
        fused_gyro_z <= 16'd0;
      else if ((local_calibration_state == 4'h4) && (right_spin_entries <= 1))
        fused_gyro_z <= 16'd0;
      else if ((local_calibration_state == 4'h4) && !right_strong_sample_sent) begin
        fused_gyro_z <= differential_yaw;
        right_strong_sample_sent <= 1'b1;
      end else if (local_calibration_state == 4'h4)
        fused_gyro_z <= 16'd0;
      else if (($signed(lf_speed) > 0) && ($signed(rf_speed) > 0) &&
               ($signed(lr_speed) > 0) && ($signed(rr_speed) > 0))
        fused_gyro_z <= differential_yaw + 320;
      else if (($signed(lf_speed) < 0) && ($signed(rf_speed) < 0) &&
               ($signed(lr_speed) < 0) && ($signed(rr_speed) < 0))
        fused_gyro_z <= differential_yaw - 320;
      else
        fused_gyro_z <= differential_yaw;
    end else begin
      sample_divider = sample_divider - 3'd1;
      fused_sample_valid <= 1'b0;
    end
  end

  initial begin
    $dumpfile("little_car/build/tb_c5_calibration_spin_retry.vcd");
    $dumpvars(0, tb_c5_calibration_spin_retry);
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    wait (fused_sample_valid);
    @(negedge clk);
    local_calibration_start = 1'b1;
    @(negedge clk);
    local_calibration_start = 1'b0;

    timeout = 0;
    while ((local_calibration_state != 4'hA) &&
           (local_calibration_state != 4'hB) &&
           (local_calibration_state != 4'hD) &&
           (local_calibration_state != 4'hE) &&
           (local_calibration_state != 4'hF) && (timeout < 12000)) begin
      @(posedge clk);
      timeout = timeout + 1;
    end

    if (local_calibration_state != 4'hA ||
        left_spin_entries != 2 || right_spin_entries != 2 ||
        !gyro_polarity_valid || local_calibration_flags[2]) begin
      $display("FAIL spin retry state=%h left=%0d right=%0d flags=%02x polarity=%0d",
               local_calibration_state, left_spin_entries, right_spin_entries,
               local_calibration_flags, gyro_polarity_valid);
      $fatal(1);
    end
    $display("RESULT spin_retries left=%0d right=%0d", left_spin_entries - 1,
             right_spin_entries - 1);
    $display("RESULT TEST_PASS c5_calibration_spin_retry");
    $finish;
  end
endmodule

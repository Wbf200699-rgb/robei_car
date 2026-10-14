`timescale 1ns/1ps
`define C5_CAL_SIM_FAST

module tb_c5_calibration_runtime_fallback;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg fused_sample_valid = 1'b0;
  reg [15:0] fused_gyro_z = 16'd0;
  reg local_calibration_start = 1'b0;
  reg [2:0] sample_divider = 3'd0;

  wire [10:0] lf_speed;
  wire [10:0] rf_speed;
  wire [10:0] lr_speed;
  wire [10:0] rr_speed;
  wire [3:0] local_calibration_state;
  wire [7:0] local_calibration_flags;
  wire [10:0] learned_yaw_trim;

  c5_motion_stabilizer dut (
    .clk(clk), .rst_n(rst_n), .calibration_done(1'b1),
    .fused_sample_valid(fused_sample_valid), .imu_degraded(1'b0),
    .imu_disagreement(1'b0), .fused_gyro_z(fused_gyro_z),
    .flow_sample_valid(1'b0), .flow_forward_q8(16'sd0),
    .flow_lateral_q8(16'sd0),
    .local_calibration_start(local_calibration_start),
    .external_force_stop(1'b0), .obstacle_stop(1'b0),
    .cal_lf_default_speed(10'd700), .cal_rf_default_speed(10'd700),
    .cal_lr_default_speed(10'd700), .cal_rr_default_speed(10'd700),
    .base_lf_speed(11'd0), .base_rf_speed(11'd0),
    .base_lr_speed(11'd0), .base_rr_speed(11'd0),
    .lf_speed(lf_speed), .rf_speed(rf_speed),
    .lr_speed(lr_speed), .rr_speed(rr_speed),
    .correction_active(), .gyro_polarity_valid(),
    .gyro_positive_is_right(), .yaw_correction(), .flow_correction(),
    .force_stop(), .local_calibration_state(local_calibration_state),
    .local_calibration_flags(local_calibration_flags),
    .learned_yaw_trim(learned_yaw_trim), .calibration_gyro_peak()
  );

  always #5 clk = ~clk;

  // The turn signs are valid, but straight motion retains a deliberately
  // uncorrectable yaw disturbance. This forces all six learning rounds to run
  // and verifies that a healthy sensor stream falls back to runtime feedback
  // instead of reporting a calibration failure.
  always @(negedge clk) begin
    fused_sample_valid <= 1'b0;
    if (!rst_n) begin
      sample_divider <= 3'd0;
      fused_gyro_z <= 16'd0;
    end else if (sample_divider == 3'd0) begin
      sample_divider <= 3'd7;
      fused_sample_valid <= 1'b1;
      case (local_calibration_state)
        4'h2: fused_gyro_z <= -16'sd900;
        4'h4: fused_gyro_z <= 16'sd900;
        4'h6: fused_gyro_z <= 16'sd320;
        4'h8: fused_gyro_z <= -16'sd320;
        default: fused_gyro_z <= 16'sd0;
      endcase
    end else begin
      sample_divider <= sample_divider - 3'd1;
    end
  end

  initial begin
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
    wait (fused_sample_valid);
    @(negedge clk);
    local_calibration_start = 1'b1;
    @(negedge clk);
    local_calibration_start = 1'b0;

    wait ((local_calibration_state == 4'hA) ||
          (local_calibration_state == 4'hB) ||
          (local_calibration_state == 4'hD) ||
          (local_calibration_state == 4'hE) ||
          (local_calibration_state == 4'hF));
    if (local_calibration_state != 4'hA)
      $fatal(1, "healthy nonconverged run failed state=%h flags=%h",
             local_calibration_state, local_calibration_flags);
    if (!local_calibration_flags[1] || local_calibration_flags[2])
      $fatal(1, "fallback completion flags invalid=%h", local_calibration_flags);
    if (dut.calibration_round_count != 3'd5)
      $fatal(1, "fallback did not consume six rounds count=%0d",
             dut.calibration_round_count + 1);

    $display("RESULT rounds=6 learned_trim=%0d result_valid=1 failed=0",
             $signed(learned_yaw_trim));
    $display("RESULT TEST_PASS c5_calibration_runtime_fallback");
    $finish;
  end

  initial begin
    #200000;
    $fatal(1, "timeout");
  end
endmodule

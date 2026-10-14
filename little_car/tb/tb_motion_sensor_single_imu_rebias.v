`timescale 1ns/1ps

module tb_motion_sensor_single_imu_rebias;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  always #5 clk = ~clk;

  wire fused_sample_valid;
  wire signed [15:0] fused_gyro_z;
  wire imu_degraded;
  reg vehicle_motion_expected = 1'b0;
  reg [7:0] forced_flow_squal = 8'd72;
  reg [31:0] forced_flow_accum_x = 32'd0;
  reg [31:0] forced_flow_accum_y = 32'd0;
  integer failures = 0;

  motion_sensor_subsystem dut (
    .clk(clk),
    .rst_n(rst_n),
    .vehicle_motion_expected(vehicle_motion_expected),
    .sensor_spi_miso(1'b0),
    .sensor_spi_sclk(),
    .sensor_spi_mosi(),
    .pmw3901_cs_n(),
    .bmi0_acc_cs_n(),
    .bmi0_gyro_cs_n(),
    .bmi1_acc_cs_n(),
    .bmi1_gyro_cs_n(),
    .sensor_debug_uart_tx(),
    .pmw3901_online(),
    .bmi0_online(),
    .bmi1_online(),
    .calibration_done(),
    .fused_sample_valid(fused_sample_valid),
    .imu_degraded(imu_degraded),
    .imu_disagreement(),
    .imu_source(),
    .pmw3901_product_id(),
    .pmw3901_inverse_id(),
    .bmi0_accel_id(),
    .bmi0_gyro_id(),
    .bmi1_accel_id(),
    .bmi1_gyro_id(),
    .flow_sample_valid(),
    .flow_squal(),
    .flow_delta_x(),
    .flow_delta_y(),
    .flow_accum_x(),
    .flow_accum_y(),
    .fused_accel_x(),
    .fused_accel_y(),
    .fused_accel_z(),
    .fused_gyro_x(),
    .fused_gyro_y(),
    .fused_gyro_z(fused_gyro_z),
    .sample_sequence(),
    .telemetry_data()
  );

  initial begin
    $dumpfile("little_car/build/tb_motion_sensor_single_imu_rebias.vcd");
    $dumpvars(0, tb_motion_sensor_single_imu_rebias);
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (5) @(posedge clk);

    // Reproduce the on-car failure: IMU0 is offline, PMW3901 reports a still
    // chassis, and the surviving IMU1 has a persistent -4080-count false yaw.
    force dut.pair_sample_strobe = 1'b1;
    force dut.calibration_done = 1'b1;
    force dut.bmi0_online = 1'b0;
    force dut.bmi1_online = 1'b1;
    force dut.pmw3901_online = 1'b1;
    force dut.flow_data_seen = 1'b1;
    force dut.flow_squal = forced_flow_squal;
    force dut.flow_delta_x = 16'sd0;
    force dut.flow_delta_y = 16'sd0;
    force dut.flow_accum_x = forced_flow_accum_x;
    force dut.flow_accum_y = forced_flow_accum_y;
    force dut.bmi1_accel_x_raw = 16'sd200;
    force dut.bmi1_accel_y_raw = 16'sd250;
    force dut.bmi1_accel_z_raw = -16'sd5400;
    force dut.bmi1_gyro_x_raw = -16'sd40;
    force dut.bmi1_gyro_y_raw = 16'sd25;
    force dut.bmi1_gyro_z_raw = -16'sd4080;

    // A deliberate calibration turn can look optically still. The explicit
    // motion intent must prevent that turn from being learned as zero bias.
    vehicle_motion_expected = 1'b1;
    repeat (20) @(posedge clk);
    if (dut.bmi1_rebias_pending || !fused_sample_valid ||
        !dut.fused_data_ready) begin
      $display("ERROR deliberate motion armed re-bias or lost fused stream");
      failures = failures + 1;
    end

    // When stopped, automatic re-bias withdraws the known-biased stream. The
    // vehicle controller must wait rather than integrate it into heading.
    vehicle_motion_expected = 1'b0;
    repeat (12) @(posedge clk);
    $display("RESULT pending_gate pending=%0d fused_valid=%0d ready=%0d",
             dut.bmi1_rebias_pending, fused_sample_valid,
             dut.fused_data_ready);
    if (!dut.bmi1_rebias_pending || fused_sample_valid ||
        dut.fused_data_ready) begin
      $display("ERROR known-biased gyro remained visible during re-bias");
      failures = failures + 1;
    end

    // Finish the 64 averaging samples plus pipeline slack.
    repeat (78) @(posedge clk);

    $display("RESULT single_imu_rebias pending=%0d bias_z=%0d corrected_z=%0d fused_z=%0d",
             dut.bmi1_rebias_pending, $signed(dut.bmi1_gyro_bias_z),
             $signed(dut.bmi1_gyro_z_corrected), $signed(fused_gyro_z));
    if (dut.bmi1_rebias_pending ||
        ($signed(dut.bmi1_gyro_bias_z) != -16'sd4080) ||
        ($signed(dut.bmi1_gyro_z_corrected) != 0) ||
        ($signed(fused_gyro_z) != 0)) begin
      $display("ERROR surviving IMU did not recover its static zero offset");
      failures = failures + 1;
    end

    // A loss of optical stillness must cancel an automatically armed window;
    // it must not leave the only IMU permanently wedged in pending state.
    rst_n = 1'b0;
    vehicle_motion_expected = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    forced_flow_squal = 8'd72;
    forced_flow_accum_x = 32'd0;
    forced_flow_accum_y = 32'd0;
    repeat (12) @(posedge clk);
    forced_flow_squal = 8'd0;
    repeat (3) @(posedge clk);
    $display("RESULT optical_abort pending=%0d auto=%0d",
             dut.bmi1_rebias_pending, dut.bmi1_auto_rebias);
    if (dut.bmi1_rebias_pending || dut.bmi1_auto_rebias) begin
      $display("ERROR automatic re-bias did not abort after optical loss");
      failures = failures + 1;
    end

    // Per-poll deltas can each be tiny during a slow turn.  Accumulated flow
    // across the eight confirmation samples must prevent false zero learning.
    rst_n = 1'b0;
    vehicle_motion_expected = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    forced_flow_squal = 8'd72;
    forced_flow_accum_x = 32'd0;
    repeat (12) begin
      @(negedge clk);
      forced_flow_accum_x = forced_flow_accum_x + 32'd2;
    end
    repeat (3) @(posedge clk);
    $display("RESULT cumulative_motion_reject pending=%0d candidate_count=%0d",
             dut.bmi1_rebias_pending,
             dut.single_imu_rebias_candidate_count);
    if (dut.bmi1_rebias_pending) begin
      $display("ERROR slow accumulated optical motion was learned as gyro bias");
      failures = failures + 1;
    end

    // Motion intent arriving on the last averaging sample must cancel before
    // the candidate bias is committed.
    rst_n = 1'b0;
    vehicle_motion_expected = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    dut.bmi1_rebias_pending = 1'b1;
    dut.bmi1_auto_rebias = 1'b1;
    dut.bmi1_rebias_count = 7'd63;
    dut.bmi1_rebias_sum_z = -32'sd257040;
    vehicle_motion_expected = 1'b1;
    @(posedge clk);
    #1;
    if (dut.bmi1_rebias_pending || dut.bmi1_auto_rebias ||
        ($signed(dut.bmi1_gyro_bias_z) != 0)) begin
      $display("ERROR motion edge allowed final-sample bias commit");
      failures = failures + 1;
    end

    if (failures == 0)
      $display("RESULT TEST_PASS motion_sensor_single_imu_rebias");
    else
      $display("RESULT TEST_FAIL motion_sensor_single_imu_rebias failures=%0d",
               failures);
    $finish;
  end
endmodule

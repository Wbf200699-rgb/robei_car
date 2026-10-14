`timescale 1ns/1ps

module motion_sensor_fusion_test;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg sensor_spi_miso = 1'b1;
  reg pair_strobe = 1'b0;
  reg signed [15:0] imu0_gx = 16'sd0;
  reg signed [15:0] imu0_gy = 16'sd0;
  reg signed [15:0] imu0_gz = 16'sd0;
  reg signed [15:0] imu1_gx = 16'sd0;
  reg signed [15:0] imu1_gy = 16'sd0;
  reg signed [15:0] imu1_gz = 16'sd0;

  wire sensor_spi_sclk;
  wire sensor_spi_mosi;
  wire pmw3901_cs_n;
  wire bmi0_acc_cs_n;
  wire bmi0_gyro_cs_n;
  wire bmi1_acc_cs_n;
  wire bmi1_gyro_cs_n;
  wire sensor_debug_uart_tx;
  wire pmw3901_online;
  wire bmi0_online;
  wire bmi1_online;
  wire calibration_done;
  wire fused_sample_valid;
  wire imu_degraded;
  wire imu_disagreement;
  wire [1:0] imu_source;
  wire [7:0] pmw3901_product_id;
  wire [7:0] pmw3901_inverse_id;
  wire [7:0] bmi0_accel_id;
  wire [7:0] bmi0_gyro_id;
  wire [7:0] bmi1_accel_id;
  wire [7:0] bmi1_gyro_id;
  wire flow_sample_valid;
  wire [7:0] flow_squal;
  wire [15:0] flow_delta_x;
  wire [15:0] flow_delta_y;
  wire [31:0] flow_accum_x;
  wire [31:0] flow_accum_y;
  wire [15:0] fused_accel_x;
  wire [15:0] fused_accel_y;
  wire [15:0] fused_accel_z;
  wire [15:0] fused_gyro_x;
  wire [15:0] fused_gyro_y;
  wire [15:0] fused_gyro_z;
  wire [15:0] sample_sequence;
  wire [319:0] telemetry_data;

  always #5 clk = ~clk;

  motion_sensor_subsystem dut (
    .clk(clk),
    .rst_n(rst_n),
    .vehicle_motion_expected(1'b0),
    .sensor_spi_miso(sensor_spi_miso),
    .sensor_spi_sclk(sensor_spi_sclk),
    .sensor_spi_mosi(sensor_spi_mosi),
    .pmw3901_cs_n(pmw3901_cs_n),
    .bmi0_acc_cs_n(bmi0_acc_cs_n),
    .bmi0_gyro_cs_n(bmi0_gyro_cs_n),
    .bmi1_acc_cs_n(bmi1_acc_cs_n),
    .bmi1_gyro_cs_n(bmi1_gyro_cs_n),
    .sensor_debug_uart_tx(sensor_debug_uart_tx),
    .pmw3901_online(pmw3901_online),
    .bmi0_online(bmi0_online),
    .bmi1_online(bmi1_online),
    .calibration_done(calibration_done),
    .fused_sample_valid(fused_sample_valid),
    .imu_degraded(imu_degraded),
    .imu_disagreement(imu_disagreement),
    .imu_source(imu_source),
    .pmw3901_product_id(pmw3901_product_id),
    .pmw3901_inverse_id(pmw3901_inverse_id),
    .bmi0_accel_id(bmi0_accel_id),
    .bmi0_gyro_id(bmi0_gyro_id),
    .bmi1_accel_id(bmi1_accel_id),
    .bmi1_gyro_id(bmi1_gyro_id),
    .flow_sample_valid(flow_sample_valid),
    .flow_squal(flow_squal),
    .flow_delta_x(flow_delta_x),
    .flow_delta_y(flow_delta_y),
    .flow_accum_x(flow_accum_x),
    .flow_accum_y(flow_accum_y),
    .fused_accel_x(fused_accel_x),
    .fused_accel_y(fused_accel_y),
    .fused_accel_z(fused_accel_z),
    .fused_gyro_x(fused_gyro_x),
    .fused_gyro_y(fused_gyro_y),
    .fused_gyro_z(fused_gyro_z),
    .sample_sequence(sample_sequence),
    .telemetry_data(telemetry_data)
  );

  task pulse_sample;
    begin
      @(negedge clk);
      pair_strobe = 1'b1;
      @(posedge clk);
      #1;
      pair_strobe = 1'b0;
    end
  endtask

  task expect_fusion;
    input [1:0] expected_source;
    input signed [15:0] expected_gz;
    input expected_degraded;
    input [127:0] label;
    begin
      if (!fused_sample_valid || imu_source !== expected_source ||
          $signed(fused_gyro_z) !== expected_gz ||
          imu_degraded !== expected_degraded) begin
        $display("FAIL %0s valid=%0d source=%0d gyro_z=%0d degraded=%0d",
                 label, fused_sample_valid, imu_source,
                 $signed(fused_gyro_z), imu_degraded);
        $fatal(1);
      end
      $display("RESULT %0s source=%0d gyro_z=%0d degraded=%0d",
               label, imu_source, $signed(fused_gyro_z), imu_degraded);
    end
  endtask

  initial begin
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // Isolate the fusion policy from the SPI bring-up state machine.  These
    // forced values represent already initialized BMI088 devices.
    force dut.pair_sample_strobe = pair_strobe;
    force dut.calibration_done = 1'b1;
    force dut.bmi0_online = 1'b1;
    force dut.bmi1_online = 1'b1;
    force dut.bmi0_accel_x_raw = 16'sd0;
    force dut.bmi0_accel_y_raw = 16'sd0;
    force dut.bmi0_accel_z_raw = 16'sd5460;
    force dut.bmi1_accel_x_raw = 16'sd0;
    force dut.bmi1_accel_y_raw = 16'sd0;
    force dut.bmi1_accel_z_raw = 16'sd5460;
    force dut.bmi0_gyro_bias_x = 16'sd0;
    force dut.bmi0_gyro_bias_y = 16'sd0;
    force dut.bmi0_gyro_bias_z = 16'sd0;
    force dut.bmi1_gyro_bias_x = 16'sd0;
    force dut.bmi1_gyro_bias_y = 16'sd0;
    force dut.bmi1_gyro_bias_z = 16'sd0;
    force dut.bmi0_gyro_x_raw = imu0_gx;
    force dut.bmi0_gyro_y_raw = imu0_gy;
    force dut.bmi0_gyro_z_raw = imu0_gz;
    force dut.bmi1_gyro_x_raw = imu1_gx;
    force dut.bmi1_gyro_y_raw = imu1_gy;
    force dut.bmi1_gyro_z_raw = imu1_gz;

    imu0_gz = 16'sd1000;
    imu1_gz = 16'sd1100;
    pulse_sample();
    expect_fusion(2'd0, 16'sd1050, 1'b0, "dual_healthy_average");

    imu0_gz = 16'sd0;
    imu1_gz = 16'sd3000;
    pulse_sample();
    expect_fusion(2'd2, 16'sd3000, 1'b1, "imu0_stuck_use_imu1");
    repeat (9) pulse_sample();
    if (!dut.bmi0_gyro_stuck) $fatal(1, "IMU0 stuck fault did not latch");

    imu0_gz = 16'sd500;
    imu1_gz = 16'sd500;
    repeat (2) pulse_sample();
    expect_fusion(2'd0, 16'sd500, 1'b0, "imu0_activity_recovers");

    imu0_gz = -16'sd2800;
    imu1_gz = 16'sd0;
    pulse_sample();
    expect_fusion(2'd1, -16'sd2800, 1'b1, "imu1_stuck_use_imu0");

    imu0_gz = 16'sd1000;
    imu1_gz = 16'sd4000;
    pulse_sample();
    expect_fusion(2'd2, 16'sd4000, 1'b1, "disagreement_keeps_stronger_motion");

    $display("RESULT TEST_PASS motion_sensor_fusion");
    $finish;
  end
endmodule

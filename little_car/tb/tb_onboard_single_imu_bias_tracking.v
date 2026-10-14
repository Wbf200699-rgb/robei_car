`timescale 1ns/1ps

module tb_onboard_single_imu_bias_tracking;
  localparam integer CLK_HZ = 100000;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  always #5 clk = ~clk;

  reg gyro_bias_learn_pulse = 1'b0;
  reg fused_sample_valid = 1'b0;
  reg signed [15:0] fused_gyro_z = 16'sd180;
  wire [9:0] pose_heading;

  onboard_occupancy_map #(.CLK_HZ(CLK_HZ)) dut (
    .clk(clk), .rst_n(rst_n),
    .map_start_pulse(1'b0), .map_pause_pulse(1'b0),
    .map_clear_pulse(1'b0), .map_sync_pulse(1'b0),
    .gyro_bias_learn_pulse(gyro_bias_learn_pulse),
    .field_config_valid(1'b0), .field_pose_x_cell(8'd64),
    .field_pose_y_cell(8'd64), .field_heading_1024(10'd0),
    .lidar_sample_valid(1'b0), .lidar_new_scan(1'b0),
    .lidar_quality(6'd0), .lidar_angle_q6(15'd0),
    .lidar_distance_q2(16'd0), .sensor_calibration_done(1'b1),
    .fused_sample_valid(fused_sample_valid),
    .fused_gyro_z(fused_gyro_z), .gyro_polarity_valid(1'b1),
    .gyro_positive_is_right(1'b1), .flow_sample_valid(1'b0),
    .flow_squal(8'd0), .flow_delta_x(16'd0), .flow_delta_y(16'd0),
    .uwb_position_valid(1'b0), .uwb_position_online(1'b0),
    .uwb_position_x_cm(16'd0), .uwb_position_y_cm(16'd0),
    .lf_speed(11'sd0), .rf_speed(11'sd0),
    .lr_speed(11'sd0), .rr_speed(11'sd0),
    .storage_busy(1'b0), .storage_mem_req(1'b0),
    .storage_mem_we(1'b0), .storage_mem_addr(14'd0),
    .storage_mem_wdata(4'sd0), .storage_load_apply(1'b0),
    .storage_loaded_epoch(8'd0), .storage_loaded_pose_x_mm(16'd0),
    .storage_loaded_pose_y_mm(16'd0), .storage_loaded_heading(10'd0),
    .storage_loaded_scan_count(16'd0),
    .storage_loaded_occupied_count(16'd0),
    .storage_loaded_free_count(16'd0), .grid_chunk_accept(1'b0),
    .pose_heading(pose_heading)
  );

  integer i;
  integer failures = 0;
  integer heading_before_turn;
  integer turn_delta;

  task sample_gyro;
    begin
      @(negedge clk);
      fused_sample_valid = 1'b1;
      @(negedge clk);
      fused_sample_valid = 1'b0;
    end
  endtask

  initial begin
    $dumpfile("little_car/build/tb_onboard_single_imu_bias_tracking.vcd");
    $dumpvars(0, tb_onboard_single_imu_bias_tracking);
    repeat (5) @(posedge clk);
    rst_n = 1'b1;

    // Learn the measured IMU1-only residual offset explicitly.
    @(negedge clk);
    gyro_bias_learn_pulse = 1'b1;
    @(negedge clk);
    gyro_bias_learn_pulse = 1'b0;
    for (i = 0; i < 256; i = i + 1)
      sample_gyro();

    // Keep feeding the same residual for one second. It must not rotate the
    // board pose, either before or after the slow background tracker settles.
    for (i = 0; i < CLK_HZ; i = i + 1) begin
      @(negedge clk);
      fused_sample_valid = ((i % 1000) == 0);
    end
    fused_sample_valid = 1'b0;
    repeat (20) @(posedge clk);
    $display("RESULT stationary_single_imu heading_1024=%0d bias=%0d",
             pose_heading, $signed(dut.gyro_residual_bias));
    if (pose_heading > 10'd1) begin
      $display("ERROR stationary IMU1 residual changed heading");
      failures = failures + 1;
    end

    // Add a real 90 dps rotation on top of the learned residual. One second
    // must still produce approximately 90 degrees (256 heading codes).
    heading_before_turn = pose_heading;
    fused_gyro_z = 16'sd11988;
    for (i = 0; i < CLK_HZ; i = i + 1) begin
      @(negedge clk);
      fused_sample_valid = ((i % 1000) == 0);
    end
    fused_sample_valid = 1'b0;
    repeat (20) @(posedge clk);
    turn_delta = (pose_heading - heading_before_turn) & 1023;
    $display("RESULT deliberate_turn heading_delta_1024=%0d", turn_delta);
    if ((turn_delta < 250) || (turn_delta > 265)) begin
      $display("ERROR deliberate turn was absorbed by bias tracking");
      failures = failures + 1;
    end

    if (failures == 0)
      $display("RESULT TEST_PASS onboard_single_imu_bias_tracking");
    else
      $display("RESULT TEST_FAIL onboard_single_imu_bias_tracking failures=%0d",
               failures);
    $finish;
  end
endmodule

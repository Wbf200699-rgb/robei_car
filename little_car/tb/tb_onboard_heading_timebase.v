`timescale 1ns/1ps

module tb_onboard_heading_timebase;
  localparam integer CLK_HZ = 100000;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  always #5 clk = ~clk;

  reg fused_sample_valid = 1'b0;
  reg signed [15:0] fused_gyro_z = 16'sd11808; // 90 dps at 131.2 LSB/dps
  wire [9:0] pose_heading;

  onboard_occupancy_map #(.CLK_HZ(CLK_HZ)) dut (
    .clk(clk), .rst_n(rst_n),
    .map_start_pulse(1'b0), .map_pause_pulse(1'b0),
    .map_clear_pulse(1'b0), .map_sync_pulse(1'b0),
    .gyro_bias_learn_pulse(1'b0), .field_config_valid(1'b0),
    .field_pose_x_cell(8'd64), .field_pose_y_cell(8'd64),
    .field_heading_1024(10'd0), .lidar_sample_valid(1'b0),
    .lidar_new_scan(1'b0), .lidar_quality(6'd0),
    .lidar_angle_q6(15'd0), .lidar_distance_q2(16'd0),
    .sensor_calibration_done(1'b1),
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

  integer cycle_count;
  integer failures = 0;
  integer delta;

  initial begin
    $dumpfile("little_car/build/tb_onboard_heading_timebase.vcd");
    $dumpvars(0, tb_onboard_heading_timebase);
    repeat (5) @(posedge clk);
    rst_n = 1'b1;

    // Supply a new gyro sample only every 40 ms.  The board must still
    // integrate the held angular rate for the complete one-second interval.
    // A per-sample implementation would report only about one quarter turn.
    for (cycle_count = 0; cycle_count < CLK_HZ; cycle_count = cycle_count + 1) begin
      @(negedge clk);
      fused_sample_valid = ((cycle_count % (CLK_HZ / 25)) == 0);
    end
    fused_sample_valid = 1'b0;
    repeat (20) @(posedge clk);

    delta = pose_heading;
    $display("RESULT sparse_25hz_input_one_second heading_1024=%0d", delta);
    if ((delta < 10'd250) || (delta > 10'd265)) begin
      $display("ERROR expected about 256 heading codes (90 degrees)");
      failures = failures + 1;
    end

    if (failures == 0)
      $display("RESULT TEST_PASS onboard_heading_timebase");
    else
      $display("RESULT TEST_FAIL onboard_heading_timebase failures=%0d", failures);
    $finish;
  end
endmodule

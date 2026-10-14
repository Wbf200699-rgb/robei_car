`timescale 1ns/1ps

module tb_onboard_occupancy_map_uwb;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg map_start_pulse = 1'b0;
  reg fused_sample_valid = 1'b0;
  reg uwb_position_valid = 1'b0;
  reg uwb_position_online = 1'b1;
  reg flow_sample_valid = 1'b0;
  reg [7:0] flow_squal = 8'd0;
  reg signed [15:0] flow_delta_x = 16'sd0;
  reg signed [15:0] flow_delta_y = 16'sd0;
  reg signed [15:0] uwb_position_x_cm = 16'sd0;
  reg signed [15:0] uwb_position_y_cm = 16'sd0;
  wire [15:0] pose_x_mm;
  wire [15:0] pose_y_mm;
  wire uwb_fusion_active;
  wire [15:0] uwb_reject_count;
  wire flow_body_sample_valid;
  wire signed [15:0] flow_body_forward_q8;
  wire signed [15:0] flow_body_lateral_q8;
  integer pose_before;
  integer pose_after;

  always #10 clk = ~clk;

  // Exercise the legacy optional single-tag path explicitly. Production uses
  // the safer default 0 and performs four-tag timestamp alignment on Android.
  onboard_occupancy_map #(
    .CLK_HZ(10000),
    .ENABLE_SINGLE_TAG_UWB_FUSION(1)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .map_start_pulse(map_start_pulse), .map_pause_pulse(1'b0),
    .map_clear_pulse(1'b0), .map_sync_pulse(1'b0),
    .gyro_bias_learn_pulse(1'b0),
    .lidar_sample_valid(1'b0), .lidar_new_scan(1'b0),
    .lidar_quality(6'd0), .lidar_angle_q6(15'd0),
    .lidar_distance_q2(16'd0),
    .sensor_calibration_done(1'b1),
    .fused_sample_valid(fused_sample_valid), .fused_gyro_z(16'sd0),
    .gyro_polarity_valid(1'b1), .gyro_positive_is_right(1'b1),
    .flow_sample_valid(flow_sample_valid), .flow_squal(flow_squal),
    .flow_delta_x(flow_delta_x), .flow_delta_y(flow_delta_y),
    .uwb_position_valid(uwb_position_valid),
    .uwb_position_online(uwb_position_online),
    .uwb_position_x_cm(uwb_position_x_cm),
    .uwb_position_y_cm(uwb_position_y_cm),
    .lf_speed(11'd0), .rf_speed(11'd0),
    .lr_speed(11'd0), .rr_speed(11'd0),
    .storage_busy(1'b0), .storage_mem_req(1'b0),
    .storage_mem_we(1'b0), .storage_mem_addr(14'd0),
    .storage_mem_wdata(4'd0), .storage_load_apply(1'b0),
    .storage_loaded_epoch(8'd0),
    .storage_loaded_pose_x_mm(16'd3200),
    .storage_loaded_pose_y_mm(16'd3200),
    .storage_loaded_heading(10'd0),
    .storage_loaded_scan_count(16'd0),
    .storage_loaded_occupied_count(16'd0),
    .storage_loaded_free_count(16'd0),
    .grid_chunk_accept(1'b0), .storage_mem_rdata(),
    .storage_mem_rvalid(), .grid_chunk_valid(), .grid_chunk_seq(),
    .grid_chunk_base(), .grid_chunk_data(), .grid_chunk_crc(),
    .map_active(), .map_paused(), .map_valid(), .map_busy(),
    .map_epoch(), .scan_count(), .occupied_count(), .free_count(),
    .dropped_sample_count(), .pose_x_mm(pose_x_mm),
    .pose_y_mm(pose_y_mm), .pose_heading(), .pose_flags(),
    .flow_body_sample_valid(flow_body_sample_valid),
    .flow_body_forward_q8(flow_body_forward_q8),
    .flow_body_lateral_q8(flow_body_lateral_q8),
    .uwb_fusion_active(uwb_fusion_active),
    .uwb_reject_count(uwb_reject_count)
  );

  task pulse_imu;
    begin
      @(negedge clk); fused_sample_valid = 1'b1;
      @(negedge clk); fused_sample_valid = 1'b0;
    end
  endtask

  task pulse_uwb;
    input signed [15:0] x_value;
    input signed [15:0] y_value;
    begin
      @(negedge clk);
      uwb_position_x_cm = x_value;
      uwb_position_y_cm = y_value;
      uwb_position_valid = 1'b1;
      @(negedge clk);
      uwb_position_valid = 1'b0;
    end
  endtask

  initial begin
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    wait (dut.map_valid);
    pulse_imu();
    @(negedge clk); map_start_pulse = 1'b1;
    @(negedge clk); map_start_pulse = 1'b0;
    pulse_uwb(16'sd100, 16'sd200);
    #1;
    if (!dut.uwb_origin_valid || pose_x_mm != 16'd3200 ||
        pose_y_mm != 16'd3200)
      $fatal(1, "first UWB origin mismatch lock=%0d active=%0d x=%0d y=%0d",
             dut.uwb_origin_valid, dut.map_active, pose_x_mm, pose_y_mm);

    pose_before = $signed(dut.pose_x_q8);
    pulse_uwb(16'sd120, 16'sd200);
    #1;
    pose_after = $signed(dut.pose_x_q8);
    if (!uwb_fusion_active)
      $fatal(1, "normal UWB innovation did not activate fusion");
    if ((pose_after <= pose_before) || (pose_after > pose_before + 64))
      $fatal(1, "UWB correction was not bounded: before=%0d after=%0d",
             pose_before, pose_after);

    pulse_uwb(16'sd620, 16'sd200);
    if (uwb_reject_count == 16'd0)
      $fatal(1, "implausible UWB jump was not rejected");

    $display("RESULT origin_tag_cm=100,200");
    $display("RESULT center_mm=%0d,%0d correction_mm=%0d",
             pose_x_mm, pose_y_mm, pose_after - pose_before);
    $display("RESULT reject_count=%0d", uwb_reject_count);
    $display("RESULT TEST_PASS onboard_occupancy_map_uwb");
    $finish;
  end
endmodule

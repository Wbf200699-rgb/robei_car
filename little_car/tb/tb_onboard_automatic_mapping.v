`timescale 1ns/1ps

module tb_onboard_automatic_mapping;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  always #10 clk = ~clk;

  reg map_start_pulse = 1'b0;
  reg map_pause_pulse = 1'b0;
  reg map_clear_pulse = 1'b0;
  reg map_sync_pulse = 1'b0;
  reg lidar_sample_valid = 1'b0;
  reg lidar_new_scan = 1'b0;
  reg [5:0] lidar_quality = 6'd40;
  reg [14:0] lidar_angle_q6 = 15'd0;
  reg [15:0] lidar_distance_q2 = 16'd2000;
  reg sensor_calibration_done = 1'b1;
  reg fused_sample_valid = 1'b0;
  reg signed [15:0] fused_gyro_z = 16'sd0;
  reg gyro_polarity_valid = 1'b1;
  reg gyro_positive_is_right = 1'b1;
  reg flow_sample_valid = 1'b0;
  reg [7:0] flow_squal = 8'd80;
  reg signed [15:0] flow_delta_x = 16'sd0;
  reg signed [15:0] flow_delta_y = 16'sd0;
  reg signed [10:0] lf_speed = 11'sd0;
  reg signed [10:0] rf_speed = 11'sd0;
  reg signed [10:0] lr_speed = 11'sd0;
  reg signed [10:0] rr_speed = 11'sd0;
  reg storage_busy = 1'b0;
  reg storage_mem_req = 1'b0;
  reg storage_mem_we = 1'b0;
  reg [13:0] storage_mem_addr = 14'd0;
  reg signed [3:0] storage_mem_wdata = 4'sd0;
  reg storage_load_apply = 1'b0;
  reg [7:0] storage_loaded_epoch = 8'd0;
  reg [15:0] storage_loaded_pose_x_mm = 16'd3200;
  reg [15:0] storage_loaded_pose_y_mm = 16'd3200;
  reg [9:0] storage_loaded_heading = 10'd0;
  reg [15:0] storage_loaded_scan_count = 16'd0;
  reg [15:0] storage_loaded_occupied_count = 16'd0;
  reg [15:0] storage_loaded_free_count = 16'd0;
  reg grid_chunk_accept = 1'b0;

  wire signed [3:0] storage_mem_rdata;
  wire storage_mem_rvalid;
  wire grid_chunk_valid;
  wire [7:0] grid_chunk_seq;
  wire [13:0] grid_chunk_base;
  wire [255:0] grid_chunk_data;
  wire [7:0] grid_chunk_crc;
  wire map_active;
  wire map_paused;
  wire map_valid;
  wire map_busy;
  wire [7:0] map_epoch;
  wire [15:0] scan_count;
  wire [15:0] occupied_count;
  wire [15:0] free_count;
  wire [15:0] dropped_sample_count;
  wire [15:0] pose_x_mm;
  wire [15:0] pose_y_mm;
  wire [9:0] pose_heading;
  wire [7:0] pose_flags;

  onboard_occupancy_map #(.CLK_HZ(100000)) dut (
    .clk(clk), .rst_n(rst_n),
    .map_start_pulse(map_start_pulse),
    .map_pause_pulse(map_pause_pulse),
    .map_clear_pulse(map_clear_pulse),
    .map_sync_pulse(map_sync_pulse),
    .gyro_bias_learn_pulse(1'b0),
    .lidar_sample_valid(lidar_sample_valid),
    .lidar_new_scan(lidar_new_scan),
    .lidar_quality(lidar_quality),
    .lidar_angle_q6(lidar_angle_q6),
    .lidar_distance_q2(lidar_distance_q2),
    .sensor_calibration_done(sensor_calibration_done),
    .fused_sample_valid(fused_sample_valid),
    .fused_gyro_z(fused_gyro_z),
    .gyro_polarity_valid(gyro_polarity_valid),
    .gyro_positive_is_right(gyro_positive_is_right),
    .flow_sample_valid(flow_sample_valid),
    .flow_squal(flow_squal),
    .flow_delta_x(flow_delta_x),
    .flow_delta_y(flow_delta_y),
    .lf_speed(lf_speed), .rf_speed(rf_speed),
    .lr_speed(lr_speed), .rr_speed(rr_speed),
    .storage_busy(storage_busy),
    .storage_mem_req(storage_mem_req),
    .storage_mem_we(storage_mem_we),
    .storage_mem_addr(storage_mem_addr),
    .storage_mem_wdata(storage_mem_wdata),
    .storage_mem_rdata(storage_mem_rdata),
    .storage_mem_rvalid(storage_mem_rvalid),
    .storage_load_apply(storage_load_apply),
    .storage_loaded_epoch(storage_loaded_epoch),
    .storage_loaded_pose_x_mm(storage_loaded_pose_x_mm),
    .storage_loaded_pose_y_mm(storage_loaded_pose_y_mm),
    .storage_loaded_heading(storage_loaded_heading),
    .storage_loaded_scan_count(storage_loaded_scan_count),
    .storage_loaded_occupied_count(storage_loaded_occupied_count),
    .storage_loaded_free_count(storage_loaded_free_count),
    .grid_chunk_accept(grid_chunk_accept),
    .grid_chunk_valid(grid_chunk_valid),
    .grid_chunk_seq(grid_chunk_seq),
    .grid_chunk_base(grid_chunk_base),
    .grid_chunk_data(grid_chunk_data),
    .grid_chunk_crc(grid_chunk_crc),
    .map_active(map_active), .map_paused(map_paused),
    .map_valid(map_valid), .map_busy(map_busy),
    .map_epoch(map_epoch), .scan_count(scan_count),
    .occupied_count(occupied_count), .free_count(free_count),
    .dropped_sample_count(dropped_sample_count),
    .pose_x_mm(pose_x_mm), .pose_y_mm(pose_y_mm),
    .pose_heading(pose_heading), .pose_flags(pose_flags)
  );

  task pulse_start;
    begin
      @(negedge clk); map_start_pulse = 1'b1;
      @(negedge clk); map_start_pulse = 1'b0;
    end
  endtask

  task pulse_pause;
    begin
      @(negedge clk); map_pause_pulse = 1'b1;
      @(negedge clk); map_pause_pulse = 1'b0;
    end
  endtask

  task pulse_clear;
    begin
      @(negedge clk); map_clear_pulse = 1'b1;
      @(negedge clk); map_clear_pulse = 1'b0;
    end
  endtask

  task pulse_imu;
    input signed [15:0] gyro_z;
    begin
      @(negedge clk);
      fused_gyro_z = gyro_z;
      fused_sample_valid = 1'b1;
      @(negedge clk);
      fused_sample_valid = 1'b0;
      fused_gyro_z = 16'sd0;
    end
  endtask

  task align_odom_window;
    begin
      while (dut.odom_tick_count != 27'd0)
        @(posedge clk);
    end
  endtask

  task send_flow_window;
    input signed [15:0] dx;
    input signed [15:0] dy;
    begin
      align_odom_window();
      repeat (100) @(posedge clk);
      @(negedge clk);
      flow_delta_x = dx;
      flow_delta_y = dy;
      flow_sample_valid = 1'b1;
      @(negedge clk);
      flow_sample_valid = 1'b0;
      flow_delta_x = 16'sd0;
      flow_delta_y = 16'sd0;
      while (dut.odom_tick_count != 27'd999)
        @(posedge clk);
      @(posedge clk);
      #1;
    end
  endtask

  task send_empty_odom_window;
    begin
      align_odom_window();
      while (dut.odom_tick_count != 27'd999)
        @(posedge clk);
      @(posedge clk);
      #1;
    end
  endtask

  task send_lidar;
    input [5:0] point_quality;
    input [14:0] point_angle_q6;
    input [15:0] point_distance_q2;
    input point_new_scan;
    begin
      wait (dut.map_state == 4'd0);
      @(negedge clk);
      lidar_quality = point_quality;
      lidar_angle_q6 = point_angle_q6;
      lidar_distance_q2 = point_distance_q2;
      lidar_new_scan = point_new_scan;
      lidar_sample_valid = 1'b1;
      @(negedge clk);
      lidar_sample_valid = 1'b0;
      lidar_new_scan = 1'b0;
      wait (dut.map_state == 4'd0);
      repeat (3) @(posedge clk);
    end
  endtask

  integer index;
  integer failures = 0;
  integer pose_before_flow;
  initial begin
    $dumpfile("little_car/build/tb_onboard_automatic_mapping.vcd");
    $dumpvars(0, tb_onboard_automatic_mapping);

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    wait (map_valid);

    // Learn a small residual stationary bias before any mapping session starts.
    for (index = 0; index < 128; index = index + 1)
      pulse_imu(16'sd64);
    if (($signed(dut.gyro_residual_bias) < 16'sd48) ||
        ($signed(dut.gyro_residual_bias) > 16'sd60)) begin
      $display("ERROR residual gyro bias=%0d", $signed(dut.gyro_residual_bias));
      failures = failures + 1;
    end

    // Straight motion produces much larger PMW Y than X. Forty Y counts per
    // 10 ms should select Y, positive polarity, and a divide-by-two scale for
    // the nominal 19-Q8-cell wheel prediction.
    lf_speed = 11'sd700;
    rf_speed = 11'sd700;
    lr_speed = 11'sd700;
    rr_speed = 11'sd700;
    repeat (10) @(posedge clk);
    for (index = 0; index < 16; index = index + 1)
      send_flow_window(16'sd2, 16'sd40);
    wait (dut.flow_scale_valid);
    repeat (4) @(posedge clk);
    if (!dut.flow_forward_axis_y || dut.flow_forward_negative ||
        ($signed(dut.flow_scale_shift) != 1)) begin
      $display("ERROR flow calibration axis_y=%0d negative=%0d shift=%0d",
               dut.flow_forward_axis_y, dut.flow_forward_negative,
               $signed(dut.flow_scale_shift));
      failures = failures + 1;
    end

    lf_speed = 11'sd0;
    rf_speed = 11'sd0;
    lr_speed = 11'sd0;
    rr_speed = 11'sd0;
    pulse_clear();
    wait (map_valid);

    // A map command may be armed before calibration, but no lidar point is
    // allowed to modify permanent memory until heading polarity is valid.
    gyro_polarity_valid = 1'b0;
    pulse_imu(16'sd64);
    pulse_start();
    send_lidar(6'd40, 15'd0, 16'd2000, 1'b1);
    if (dut.map_mem[{7'd64, 7'd74}] !== 4'sd0) begin
      $display("ERROR stale/unoriented pose changed map cell=%0d",
               dut.map_mem[{7'd64, 7'd74}]);
      failures = failures + 1;
    end
    pulse_pause();
    gyro_polarity_valid = 1'b1;
    pulse_imu(16'sd64);
    pulse_start();

    // Half the learned optical-flow displacement should reduce the selected
    // step from wheel-only 19 to the 3:1 blended value 12.
    lf_speed = 11'sd700;
    rf_speed = 11'sd700;
    lr_speed = 11'sd700;
    rr_speed = 11'sd700;
    repeat (10) @(posedge clk);
    pose_before_flow = $signed(dut.pose_x_q8);
    send_flow_window(16'sd1, 16'sd20);
    // odom_step_q8 is the committed result. flow_fusion_used is a one-window
    // diagnostic pulse and may already be low when this task returns.
    if ($signed(dut.odom_step_q8) != 16'sd12) begin
      $display("ERROR flow fusion used=%0d selected_step=%0d",
               dut.flow_fusion_used, $signed(dut.odom_step_q8));
      failures = failures + 1;
    end
    send_flow_window(16'sd1, 16'sd20);
    if ($signed(dut.pose_x_q8) < pose_before_flow + 10) begin
      $display("ERROR fused flow did not advance pose before=%0d after=%0d",
               pose_before_flow, $signed(dut.pose_x_q8));
      failures = failures + 1;
    end
    send_empty_odom_window();
    if (dut.flow_fusion_used || ($signed(dut.odom_step_q8) != 16'sd19)) begin
      $display("ERROR wheel fallback used=%0d selected_step=%0d",
               dut.flow_fusion_used, $signed(dut.odom_step_q8));
      failures = failures + 1;
    end

    lf_speed = 11'sd0;
    rf_speed = 11'sd0;
    lr_speed = 11'sd0;
    rr_speed = 11'sd0;
    send_empty_odom_window();
    send_empty_odom_window();
    pulse_clear();
    wait (map_valid);
    pulse_imu(16'sd64);
    pulse_start();

    // The first observation seeds the per-direction outlier filter. Three
    // further low-quality confirmations are required to reach occupied=3.
    send_lidar(6'd8, 15'd0, 16'd2000, 1'b1);
    send_lidar(6'd8, 15'd0, 16'd2000, 1'b0);
    if (dut.map_mem[{7'd64, 7'd74}] >= 4'sd3) begin
      $display("ERROR low-quality endpoint committed too early value=%0d",
               dut.map_mem[{7'd64, 7'd74}]);
      failures = failures + 1;
    end
    send_lidar(6'd8, 15'd0, 16'd2000, 1'b0);
    send_lidar(6'd8, 15'd0, 16'd2000, 1'b0);
    if (dut.map_mem[{7'd64, 7'd74}] != 4'sd3) begin
      $display("ERROR repeated endpoint value=%0d expected=3",
               dut.map_mem[{7'd64, 7'd74}]);
      failures = failures + 1;
    end
    send_lidar(6'd8, 15'd0, 16'd2000, 1'b0);

    // A valid return beyond the grid is clipped for ray clearing but must not
    // create a false occupied wall at the last in-bounds cell. RPLIDAR angles
    // increase clockwise, so +90 degrees is the vehicle-right / negative-map-Y
    // ray. This remains independent of the earlier forward obstacle.
    send_lidar(6'd40, 15'd5760, 16'd20000, 1'b0);
    send_lidar(6'd40, 15'd5760, 16'd20000, 1'b0);
    send_lidar(6'd40, 15'd5760, 16'd20000, 1'b0);
    if ((dut.map_mem[{7'd1, 7'd64}] > -4'sd2) ||
        (occupied_count != 16'd1)) begin
      $display("ERROR clipped ray boundary=%0d occupied=%0d",
               dut.map_mem[{7'd1, 7'd64}], occupied_count);
      failures = failures + 1;
    end

    // Once IMU freshness expires, lidar remains parsed but cannot alter the
    // persistent map. A new paired sample automatically re-enables mapping.
    pulse_imu(16'sd64);
    for (index = 0; index < 11; index = index + 1)
      send_empty_odom_window();
    send_lidar(6'd40, 15'd11520, 16'd2000, 1'b0);
    if (dut.map_mem[{7'd64, 7'd54}] !== 4'sd0) begin
      $display("ERROR stale IMU point entered map value=%0d",
               dut.map_mem[{7'd64, 7'd54}]);
      failures = failures + 1;
    end
    pulse_imu(16'sd64);
    send_lidar(6'd40, 15'd11520, 16'd2000, 1'b0);
    send_lidar(6'd40, 15'd11520, 16'd2000, 1'b0);
    send_lidar(6'd40, 15'd11520, 16'd2000, 1'b0);
    if (dut.map_mem[{7'd64, 7'd54}] < 4'sd4) begin
      $display("ERROR map did not recover after fresh IMU value=%0d",
               dut.map_mem[{7'd64, 7'd54}]);
      failures = failures + 1;
    end

    $display("RESULT flow_axis_y=%0d polarity_negative=%0d scale_shift=%0d",
             dut.flow_forward_axis_y, dut.flow_forward_negative,
             $signed(dut.flow_scale_shift));
    $display("RESULT gyro_residual_bias=%0d flow_step=%0d fallback_step=19",
             $signed(dut.gyro_residual_bias), 12);
    $display("RESULT map_counts occupied=%0d free=%0d clipped_boundary=%0d",
             occupied_count, free_count, dut.map_mem[{7'd1, 7'd64}]);
    $display("RESULT pose_flags=%02x dropped=%0d", pose_flags,
             dropped_sample_count);
    if (failures == 0)
      $display("RESULT TEST_PASS onboard_automatic_mapping");
    else
      $display("RESULT TEST_FAIL onboard_automatic_mapping failures=%0d",
               failures);
    $finish;
  end
endmodule

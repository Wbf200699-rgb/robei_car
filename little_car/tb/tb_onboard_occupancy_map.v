`timescale 1ns/1ps

module tb_onboard_occupancy_map;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  always #10 clk = ~clk;

  reg map_start_pulse = 1'b0;
  reg map_pause_pulse = 1'b0;
  reg map_clear_pulse = 1'b0;
  reg map_sync_pulse = 1'b0;
  reg gyro_bias_learn_pulse = 1'b0;
  reg field_config_valid = 1'b0;
  reg [7:0] field_pose_x_cell = 8'd64;
  reg [7:0] field_pose_y_cell = 8'd64;
  reg [9:0] field_heading_1024 = 10'd0;
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
  reg [7:0] flow_squal = 8'd64;
  reg signed [15:0] flow_delta_x = 16'sd0;
  reg signed [15:0] flow_delta_y = 16'sd0;
  reg signed [10:0] lf_speed = 11'sd0;
  reg signed [10:0] rf_speed = 11'sd0;
  reg signed [10:0] lr_speed = 11'sd0;
  reg signed [10:0] rr_speed = 11'sd0;
  reg storage_busy = 1'b1;
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

  onboard_occupancy_map #(
    .CLK_HZ(100000)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .map_start_pulse(map_start_pulse),
    .map_pause_pulse(map_pause_pulse),
    .map_clear_pulse(map_clear_pulse),
    .map_sync_pulse(map_sync_pulse),
    .gyro_bias_learn_pulse(gyro_bias_learn_pulse),
    .field_config_valid(field_config_valid),
    .field_pose_x_cell(field_pose_x_cell),
    .field_pose_y_cell(field_pose_y_cell),
    .field_heading_1024(field_heading_1024),
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

  task pulse_imu_sample;
    begin
      @(negedge clk); fused_sample_valid = 1'b1;
      @(negedge clk); fused_sample_valid = 1'b0;
    end
  endtask

  task send_front_sample;
    input first;
    begin
      wait (dut.map_state == 0);
      @(negedge clk);
      lidar_new_scan = first;
      lidar_sample_valid = 1'b1;
      @(negedge clk);
      lidar_sample_valid = 1'b0;
      lidar_new_scan = 1'b0;
      wait (dut.map_state == 0);
      repeat (4) @(posedge clk);
    end
  endtask

  integer index;
  integer heading_before_resume;
  integer heading_before_quarter_turn;
  integer quarter_turn_delta;
  integer failures = 0;
  initial begin
    $dumpfile("little_car/build/tb_onboard_occupancy_map.vcd");
    $dumpvars(0, tb_onboard_occupancy_map);
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (20) @(posedge clk);
    if (!dut.clear_active || (dut.clear_addr != 14'd0)) begin
      $display("ERROR clear advanced while storage busy addr=%0d", dut.clear_addr);
      failures = failures + 1;
    end

    // Model the final write of an SD auto-load, then apply its metadata while
    // the storage controller still owns RAM port A. Once storage_busy drops,
    // the power-on clear cursor must remain cancelled and preserve this cell.
    @(negedge clk);
    storage_mem_addr = 14'd123;
    storage_mem_wdata = 4'sd5;
    storage_mem_we = 1'b1;
    storage_mem_req = 1'b1;
    @(negedge clk);
    storage_mem_req = 1'b0;
    storage_mem_we = 1'b0;
    storage_loaded_epoch = 8'd9;
    storage_loaded_pose_x_mm = 16'd3000;
    storage_loaded_pose_y_mm = 16'd3100;
    storage_loaded_heading = 10'd137;
    storage_loaded_scan_count = 16'd42;
    storage_loaded_occupied_count = 16'd7;
    storage_loaded_free_count = 16'd91;
    storage_load_apply = 1'b1;
    @(negedge clk);
    storage_load_apply = 1'b0;
    storage_busy = 1'b0;
    repeat (20) @(posedge clk);
    if (dut.clear_active || (dut.map_mem[123] !== 4'sd5) || !map_valid ||
        (map_epoch != 8'd9) || (scan_count != 16'd42) ||
        (pose_heading != 10'd137)) begin
      $display("ERROR loaded map was cleared active=%0d cell=%0d epoch=%0d scan=%0d heading=%0d",
               dut.clear_active, dut.map_mem[123], map_epoch, scan_count,
               pose_heading);
      failures = failures + 1;
    end

    // Restart with storage released so the rest of this test starts from a
    // normally cleared map rather than the intentionally partial load above.
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    wait (map_valid);
    pulse_imu_sample();
    field_config_valid = 1'b1;
    field_pose_x_cell = 8'd43;
    field_pose_y_cell = 8'd51;
    field_heading_1024 = 10'd0;
    pulse_start();
    if (pose_x_mm != 16'd2150 || pose_y_mm != 16'd2550 ||
        pose_heading != 10'd0) begin
      $display("ERROR app field pose was not applied x=%0d y=%0d heading=%0d",
               pose_x_mm, pose_y_mm, pose_heading);
      failures = failures + 1;
    end
    // Static calibrated noise remains inside the heading deadband and must not
    // make the map vehicle spin while stopped.
    repeat (200) begin
      fused_gyro_z = 16'sd20;
      pulse_imu_sample();
    end
    fused_gyro_z = 16'sd0;
    if (pose_heading != 10'd0) begin
      $display("ERROR stationary gyro noise drifted heading=%0d", pose_heading);
      failures = failures + 1;
    end

    // A stationary residual offset above the normal deadband must also remain
    // frozen.  This reproduces the hardware symptom where the mapping vehicle
    // rotated while localization stayed near the correct pose.
    @(negedge clk); gyro_bias_learn_pulse = 1'b1;
    @(negedge clk); gyro_bias_learn_pulse = 1'b0;
    repeat (256) begin
      fused_gyro_z = 16'sd200;
      pulse_imu_sample();
    end
    fused_gyro_z = 16'sd0;
    if (pose_heading != 10'd0 || dut.gyro_bias_learn_active ||
        dut.gyro_residual_bias != 17'sd200) begin
      $display("ERROR stationary bias learning heading=%0d active=%0d bias=%0d",
               pose_heading, dut.gyro_bias_learn_active,
               dut.gyro_residual_bias);
      failures = failures + 1;
    end

    // A BMI088 value of 11808 counts is 90 dps at the configured +/-250 dps
    // range. Feed sparse 25 Hz samples for one real second; the deterministic
    // 100 Hz board timebase must still advance approximately one quarter turn.
    // This guards both physical-unit conversion and resistance to SPI jitter.
    heading_before_quarter_turn = pose_heading;
    repeat (25) begin
      fused_gyro_z = 16'sd12008; // 11808 motion + learned 200-count residual.
      pulse_imu_sample();
      repeat (3998) @(posedge clk);
    end
    fused_gyro_z = 16'sd0;
    // The board protocol follows the Android vehicle-view convention.  With
    // this learned sensor polarity a positive raw sample must advance the
    // displayed heading; applying another mathematical-coordinate inversion
    // here mirrors the physical turn on the phone.
    quarter_turn_delta = (pose_heading - heading_before_quarter_turn) & 10'h3ff;
    if ((quarter_turn_delta < 10'd249) || (quarter_turn_delta > 10'd261)) begin
      $display("ERROR 90-degree gyro integration delta=%0d expected=249..261",
               quarter_turn_delta);
      failures = failures + 1;
    end else begin
      $display("RESULT gyro_90deg_delta=%0d", quarter_turn_delta);
    end
    repeat (25) begin
      fused_gyro_z = -16'sd11608; // -11808 corrected after the +200 residual.
      pulse_imu_sample();
      repeat (3998) @(posedge clk);
    end
    fused_gyro_z = 16'sd0;
    if (pose_heading != heading_before_quarter_turn[9:0]) begin
      $display("ERROR reverse 90-degree integration did not return heading=%0d",
               pose_heading);
      failures = failures + 1;
    end

    send_front_sample(1'b1);
    send_front_sample(1'b0);
    send_front_sample(1'b0);

    if (dut.map_mem[{7'd51, 7'd53}] < 4'sd3) begin
      $display("ERROR endpoint cell=%0d expected occupied", dut.map_mem[{7'd51, 7'd53}]);
      failures = failures + 1;
    end
    for (index = 43; index < 53; index = index + 1) begin
      if (dut.map_mem[{7'd51, index[6:0]}] > -4'sd2) begin
        $display("ERROR free cell x=%0d value=%0d", index,
                 dut.map_mem[{7'd64, index[6:0]}]);
        failures = failures + 1;
      end
    end
    if (occupied_count != 16'd1 || free_count < 16'd9) begin
      $display("ERROR counts occupied=%0d free=%0d", occupied_count, free_count);
      failures = failures + 1;
    end

    @(negedge clk); map_sync_pulse = 1'b1;
    @(negedge clk); map_sync_pulse = 1'b0;
    wait (grid_chunk_valid);
    if (grid_chunk_base != 14'd0) begin
      $display("ERROR first chunk base=%0d", grid_chunk_base);
      failures = failures + 1;
    end

    // A deliberate manual chassis rotation must still be observed even when no
    // wheel command is active. Power-on reset clears the learned polarity, so
    // also verify that this display-only heading remains live with bit 1 low.
    @(negedge clk);
    gyro_polarity_valid = 1'b0;
    fused_gyro_z = 16'sd10000;
    pulse_imu_sample();
    repeat (1000) @(posedge clk);
    fused_gyro_z = 16'sd0;
    repeat (2) @(posedge clk);
    if (pose_heading == 10'd0) begin
      $display("ERROR heading did not integrate before polarity learning flags=%02x",
               pose_flags);
      failures = failures + 1;
    end

    // The learned residual bias is +200 counts, so -9600 produces the exact
    // opposite corrected rate of the preceding +10000 sample. A signed-width
    // promotion bug previously zero-extended this negative increment and left
    // an 11.25-degree jump instead of returning to zero.
    @(negedge clk);
    fused_gyro_z = -16'sd9600;
    pulse_imu_sample();
    repeat (1000) @(posedge clk);
    fused_gyro_z = 16'sd0;
    repeat (2) @(posedge clk);
    if (pose_heading != 10'd0) begin
      $display("ERROR signed heading increment did not cancel heading=%0d",
               pose_heading);
      failures = failures + 1;
    end
    gyro_polarity_valid = 1'b1;

    heading_before_resume = pose_heading;
    @(negedge clk); map_pause_pulse = 1'b1;
    @(negedge clk); map_pause_pulse = 1'b0;
    field_heading_1024 = 10'd700;
    pulse_start();
    if (pose_heading != heading_before_resume[9:0]) begin
      $display("ERROR resume reapplied initial heading old=%0d new=%0d",
               heading_before_resume, pose_heading);
      failures = failures + 1;
    end

    $display("RESULT map_counts occupied=%0d free=%0d dropped=%0d",
             occupied_count, free_count, dropped_sample_count);
    $display("RESULT pose_mm=%0d,%0d heading=%0d", pose_x_mm, pose_y_mm, pose_heading);
    $display("RESULT first_chunk_crc=%02X", grid_chunk_crc);
    if (failures == 0)
      $display("RESULT TEST_PASS onboard_occupancy_map");
    else
      $display("RESULT TEST_FAIL onboard_occupancy_map failures=%0d", failures);
    $finish;
  end
endmodule

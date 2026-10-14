`timescale 1ns/1ps
`define ONBOARD_EXPLORATION_SIM_FAST

module tb_onboard_exploration_controller;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg map_active = 1'b0;
  reg map_paused = 1'b0;
  reg manual_mode = 1'b0;
  reg sensor_calibration_done = 1'b1;
  reg gyro_polarity_valid = 1'b1;
  reg lidar_online = 1'b1;
  reg scan_active = 1'b1;
  reg lidar_new_scan = 1'b0;
  reg obstacle_stop = 1'b0;
  reg [15:0] front = 16'd2000;
  reg [15:0] front_right = 16'd2000;
  reg [15:0] right = 16'd2000;
  reg [15:0] rear_right = 16'd2000;
  reg [15:0] rear = 16'd2000;
  reg [15:0] rear_left = 16'd2000;
  reg [15:0] left = 16'd2000;
  reg [15:0] front_left = 16'd2000;
  reg [15:0] pose_x = 16'd3200;
  reg [15:0] pose_y = 16'd3200;
  reg [9:0] heading = 10'd0;
  reg field_config_valid = 1'b0;
  reg [7:0] field_low_x_cell = 8'd10;
  reg [7:0] field_high_x_cell = 8'd40;
  reg [7:0] field_low_y_cell = 8'd10;
  reg [7:0] field_high_y_cell = 8'd44;
  wire auto_mode;
  wire [2:0] auto_cmd;
  wire [3:0] state;
  wire [3:0] stop_reason;
  integer failures = 0;

  always #10 clk = ~clk;

  onboard_exploration_controller dut (
    .clk(clk), .rst_n(rst_n), .map_active(map_active),
    .map_paused(map_paused), .manual_mode(manual_mode),
    .sensor_calibration_done(sensor_calibration_done),
    .gyro_polarity_valid(gyro_polarity_valid),
    .lidar_online(lidar_online), .scan_active(scan_active),
    .lidar_new_scan(lidar_new_scan), .obstacle_stop(obstacle_stop),
    .safe_distance_mm(16'd450), .sector_front_mm(front),
    .sector_front_right_mm(front_right), .sector_right_mm(right),
    .sector_rear_right_mm(rear_right), .sector_rear_mm(rear),
    .sector_rear_left_mm(rear_left), .sector_left_mm(left),
    .sector_front_left_mm(front_left), .pose_x_mm(pose_x),
    .pose_y_mm(pose_y), .pose_heading(heading),
    .field_config_valid(field_config_valid),
    .field_low_x_cell(field_low_x_cell),
    .field_high_x_cell(field_high_x_cell),
    .field_low_y_cell(field_low_y_cell),
    .field_high_y_cell(field_high_y_cell),
    .auto_mode(auto_mode), .auto_cmd(auto_cmd),
    .exploration_state(state), .stop_reason(stop_reason)
  );

  task scan_pulse;
    begin
      lidar_new_scan = 1'b1;
      @(posedge clk);
      lidar_new_scan = 1'b0;
    end
  endtask

  task wait_ms;
    input integer count;
    begin
      repeat (count * 4 + 2) @(posedge clk);
    end
  endtask

  initial begin
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    scan_pulse();
    map_active = 1'b1;
    wait_ms(3);
    if (!auto_mode || auto_cmd != 3'd1) begin
      $display("ERROR autonomous start mode=%0d cmd=%0d", auto_mode, auto_cmd);
      failures = failures + 1;
    end
    $display("RESULT open_space state=%0d cmd=%0d reason=%0d", state, auto_cmd, stop_reason);

    // A live scan with no valid front sectors is not evidence of free space.
    front = 16'hffff;
    front_left = 16'hffff;
    front_right = 16'hffff;
    scan_pulse();
    wait_ms(16);
    if (auto_cmd == 3'd1) begin
      $display("ERROR invalid front scan still drives forward");
      failures = failures + 1;
    end
    $display("RESULT invalid_front state=%0d cmd=%0d reason=%0d", state, auto_cmd, stop_reason);

    obstacle_stop = 1'b1;
    front = 16'd250;
    left = 16'd700;
    front_left = 16'd650;
    right = 16'd1800;
    front_right = 16'd1600;
    scan_pulse();
    wait_ms(8);
    if (auto_cmd != 3'd2 && auto_cmd != 3'd3 && auto_cmd != 3'd4) begin
      $display("ERROR obstacle escape cmd=%0d", auto_cmd);
      failures = failures + 1;
    end
    $display("RESULT obstacle_escape state=%0d cmd=%0d left=%0d right=%0d", state, auto_cmd, left, right);

    // The left-front and right-rear corners sweep outward during a left turn.
    // Both turn directions blocked must settle instead of scraping the wall.
    rear = 16'd200;
    left = 16'd200;
    right = 16'd200;
    rear_left = 16'd200;
    rear_right = 16'd200;
    scan_pulse();
    wait_ms(16);
    if (auto_cmd == 3'd3 || auto_cmd == 3'd4) begin
      $display("ERROR blocked chassis footprint still turns cmd=%0d", auto_cmd);
      failures = failures + 1;
    end
    $display("RESULT footprint_block state=%0d cmd=%0d", state, auto_cmd);

    manual_mode = 1'b1;
    wait_ms(2);
    if (auto_mode || auto_cmd != 3'd0) begin
      $display("ERROR manual override mode=%0d cmd=%0d", auto_mode, auto_cmd);
      failures = failures + 1;
    end
    $display("RESULT manual_override mode=%0d cmd=%0d reason=%0d", auto_mode, auto_cmd, stop_reason);

    manual_mode = 1'b0;
    obstacle_stop = 1'b0;
    front = 16'd2000;
    front_left = 16'd2000;
    front_right = 16'd2000;
    rear = 16'd2000;
    left = 16'd2000;
    right = 16'd2000;
    rear_left = 16'd2000;
    rear_right = 16'd2000;
    pose_x = 16'd6000;
    heading = 10'd0;
    scan_pulse();
    wait_ms(16);
    if (auto_cmd == 3'd1) begin
      $display("ERROR map boundary still drives forward");
      failures = failures + 1;
    end
    $display("RESULT default_boundary pose_x=%0d heading=%0d state=%0d cmd=%0d", pose_x, heading, state, auto_cmd);

    // App-configured bounds are expressed in 50 mm cells. Verify that the
    // widened cell-to-mm conversion protects a non-default competition field.
    field_config_valid = 1'b1;
    pose_x = 16'd2050;
    heading = 10'd0;
    scan_pulse();
    wait_ms(16);
    if (auto_cmd == 3'd1) begin
      $display("ERROR configured field boundary still drives forward");
      failures = failures + 1;
    end
    $display("RESULT configured_boundary high_x_cell=%0d pose_x=%0d state=%0d cmd=%0d", field_high_x_cell, pose_x, state, auto_cmd);

    map_paused = 1'b1;
    wait_ms(2);
    if (auto_mode || auto_cmd != 3'd0) begin
      $display("ERROR pause did not stop mode=%0d cmd=%0d", auto_mode, auto_cmd);
      failures = failures + 1;
    end
    $display("RESULT pause mode=%0d cmd=%0d reason=%0d", auto_mode, auto_cmd, stop_reason);

    $display("RESULT final_state=%0d stop_reason=%0d", state, stop_reason);
    if (failures == 0)
      $display("RESULT TEST_PASS onboard_exploration");
    else
      $display("RESULT TEST_FAIL onboard_exploration failures=%0d", failures);
    $finish;
  end
endmodule

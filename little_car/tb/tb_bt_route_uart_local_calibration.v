`timescale 1ns/1ps
`define BT_MAP_SIM_FAST

module tb_bt_route_uart_local_calibration;
  localparam integer CLKS_PER_BIT = 50000000 / 115200;
  localparam integer STATUS_LINE_BYTES = 99;

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg bt_uart_rx = 1'b1;
  reg [7:0] line [0:127];
  reg [7:0] rx_byte;
  reg [7:0] local_calibration_flags = 8'h52;
  integer line_length;
  integer index;
  integer status_attempts;
  reg start_seen = 1'b0;
  reg stop_seen = 1'b0;

  wire bt_uart_tx;
  wire manual_mode;
  wire [2:0] manual_cmd;
  wire motor_stop_pulse;
  wire local_calibration_start_pulse;

  always #10 clk = ~clk;
  always @(posedge clk) begin
    if (local_calibration_start_pulse) start_seen <= 1'b1;
    if (motor_stop_pulse) stop_seen <= 1'b1;
  end

  bt_route_uart dut (
    .clk(clk), .rst_n(rst_n), .bt_uart_rx(bt_uart_rx),
    .menu_info(2'd0), .state_info(5'd0), .route_info(64'd0),
    .active_route_steps(6'd0), .completed_route_steps(6'd0),
    .now_x(8'd0), .now_y(8'd0), .target_x(8'd0), .target_y(8'd0),
    .setting_mode(1'b0), .selected_param(3'd0), .step_length_set(16'd20),
    .overall_speed(10'd700), .lf_default_speed(10'd700),
    .rf_default_speed(10'd700), .lr_default_speed(10'd700),
    .rr_default_speed(10'd700), .lidar_online(1'b1),
    .lidar_obstacle_stop(1'b0), .ultrasonic_obstacle_stop(1'b0),
    .merged_obstacle_stop(1'b0), .motor_blocked(1'b0),
    .ultrasonic_distance_cm(16'hffff), .ultrasonic_distance_valid(1'b0),
    .ultrasonic_echo_timeout(1'b0), .lidar_front_distance_mm(16'hffff),
    .lidar_safe_mm(16'd450), .lidar_sector_front_mm(16'hffff),
    .lidar_sector_front_right_mm(16'hffff), .lidar_sector_right_mm(16'hffff),
    .lidar_sector_rear_right_mm(16'hffff), .lidar_sector_rear_mm(16'hffff),
    .lidar_sector_rear_left_mm(16'hffff), .lidar_sector_left_mm(16'hffff),
    .lidar_sector_front_left_mm(16'hffff), .c5_frame_count(16'h1234),
    .c5_last_lf_pulse(12'd1500), .c5_last_rf_pulse(12'd1500),
    .c5_last_lr_pulse(12'd1500), .c5_last_rr_pulse(12'd1500),
    .c5_stop_frames_complete(1'b1), .c5_stop_frames_remaining(2'd0),
    .c5_frame_busy(1'b0), .map_chunk_valid(1'b0), .map_scan_seq(8'd0),
    .map_tick_10ms(16'd0), .map_part(3'd0), .map_chunk_data(144'd0),
    .map_chunk_crc(8'd0),
    .grid_chunk_valid(1'b0), .grid_chunk_seq(8'd0), .grid_chunk_base(14'd0),
    .grid_chunk_data(256'd0), .grid_chunk_crc(8'd0),
    .onboard_map_active(1'b0), .onboard_map_paused(1'b1),
    .onboard_map_valid(1'b0), .onboard_map_busy(1'b0), .onboard_map_epoch(8'd0),
    .onboard_map_scan_count(16'd0), .onboard_map_occupied_count(16'd0),
    .onboard_map_free_count(16'd0), .onboard_map_dropped_count(16'd0),
    .onboard_pose_x_mm(16'd0), .onboard_pose_y_mm(16'd0),
    .onboard_pose_heading(10'd0), .onboard_pose_flags(8'd0),
    .flash_ready(1'b0), .flash_busy(1'b0), .flash_operation(2'd0),
    .flash_progress(8'd0), .flash_error(8'd0), .flash_jedec_id(24'd0),
    .flash_saved_sequence(32'd0), .vl53_initialization_done(1'b0),
    .vl53_sensor_online(5'd0), .vl53_distance_valid(5'd0),
    .vl53_distance_mm(80'd0), .vl53_error_count(16'd0),
    .motion_telemetry_data(320'd0),
    .local_calibration_state(4'hA),
    .local_calibration_flags(local_calibration_flags),
    .learned_yaw_trim(-11'sd21),
    .yaw_correction(11'sd12),
    .calibration_gyro_peak(16'd500),
    .bt_uart_tx(bt_uart_tx), .bt_connected(), .bt_key_up_n(),
    .bt_key_down_n(), .bt_key_left_n(), .bt_key_right_n(), .bt_key_mid_n(),
    .bt_cfg_mode_pulse(), .bt_cfg_next_pulse(), .bt_cfg_prev_pulse(),
    .bt_cfg_inc_pulse(), .bt_cfg_dec_pulse(), .bt_cfg_select(),
    .bt_cfg_select_valid(), .manual_mode(manual_mode), .manual_cmd(manual_cmd),
    .lidar_enable(), .motor_stop_pulse(motor_stop_pulse),
    .local_calibration_start_pulse(local_calibration_start_pulse),
    .map_chunk_accept()
  );

  task read_uart_byte;
    output [7:0] value;
    integer bit_index;
    begin
      @(negedge bt_uart_tx);
      repeat (CLKS_PER_BIT + (CLKS_PER_BIT / 2)) @(posedge clk);
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        value[bit_index] = bt_uart_tx;
        repeat (CLKS_PER_BIT) @(posedge clk);
      end
      if (bt_uart_tx !== 1'b1)
        $fatal(1, "UART stop bit is not high at line byte %0d", line_length);
      repeat (CLKS_PER_BIT / 4) @(posedge clk);
    end
  endtask

  task read_status_line;
    begin
      status_attempts = 0;
      read_line();
      while ((line_length < 2 || line[0] != "@" || line[1] != "S") &&
             status_attempts < 5) begin
        status_attempts = status_attempts + 1;
        read_line();
      end
    end
  endtask

  task read_line;
    begin
      line_length = 0;
      while ((line_length == 0) || (line[line_length - 1] != 8'h0a)) begin
        read_uart_byte(rx_byte);
        line[line_length] = rx_byte;
        line_length = line_length + 1;
      end
    end
  endtask

  task send_uart_byte;
    input [7:0] value;
    integer bit_index;
    begin
      bt_uart_rx = 1'b0;
      repeat (CLKS_PER_BIT) @(posedge clk);
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        bt_uart_rx = value[bit_index];
        repeat (CLKS_PER_BIT) @(posedge clk);
      end
      bt_uart_rx = 1'b1;
      repeat (CLKS_PER_BIT) @(posedge clk);
    end
  endtask

  initial begin
    repeat (8) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    wait (dut.bt_startup_ready);
    force dut.link_active = 1'b1;

    // Startup now also has an independent @G0 producer. Select the route
    // status frame explicitly instead of depending on scheduler order.
    read_status_line();
    if (line_length != STATUS_LINE_BYTES || line[0] != "@" || line[1] != "S")
      $fatal(1, "status line shape bytes=%0d", line_length);
    if ({line[85], line[86], line[87], line[88], line[89], line[90],
         line[91], line[92], line[93], line[94], line[95], line[96], line[97]}
        !== "A52FEB00C01F4")
      $fatal(1, "calibration status tail mismatch");

    repeat (4) @(posedge clk);
    if (dut.route_status_changed)
      $fatal(1, "route status change detector did not settle");

    // Bits 7:6 are live obstacle/sensor-ready telemetry. Their power-up
    // chatter must not flood BLE with full status frames.
    local_calibration_flags = 8'h92;
    #1;
    if (dut.route_status_changed)
      $fatal(1, "obstacle flag edge triggered a route status frame");
    local_calibration_flags = 8'h12;
    #1;
    if (dut.route_status_changed)
      $fatal(1, "sensor-ready flag edge triggered a route status frame");

    // A control/result bit still requires immediate status synchronization.
    local_calibration_flags = 8'h13;
    #1;
    if (!dut.route_status_changed)
      $fatal(1, "calibration control flag did not trigger route status");
    // Restore before the next active clock edge so this combinational check
    // does not enqueue a real frame and interfere with the command test.
    local_calibration_flags = 8'h52;
    #1;

    fork
      send_uart_byte("X");
      read_line();
    join
    if (line_length != 4 || {line[0], line[1], line[2]} !== "@KX")
      $fatal(1, "X acknowledgement missing");
    repeat (4) @(posedge clk);
    if (!start_seen || !stop_seen || !manual_mode || manual_cmd != 3'd0)
      $fatal(1, "X control pulse/manual stop mismatch");

    $display("RESULT status_payload_hex=96 calibration_tail=A52FEB00C01F4");
    $display("RESULT command_X start_pulse=1 force_stop_request=1 manual_stop=1");
    $display("RESULT TEST_PASS bt_route_uart_local_calibration");
    release dut.link_active;
    $finish;
  end

  initial begin
    #30000000;
    $fatal(1, "timeout");
  end
endmodule

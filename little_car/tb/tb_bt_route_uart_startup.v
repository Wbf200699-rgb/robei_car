`timescale 1ns/1ps
`define BT_MAP_SIM_FAST

module tb_bt_route_uart_startup;
  localparam integer CLKS_PER_BIT = 50000000 / 115200;

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg bt_uart_rx = 1'b1;
  reg [7:0] received_byte;
  reg [7:0] line [0:31];
  integer index;
  integer line_length;

  wire bt_uart_tx;
  wire bt_connected;

  always #10 clk = ~clk;

  bt_route_uart dut (
    .clk(clk), .rst_n(rst_n), .bt_uart_rx(bt_uart_rx),
    .menu_info(2'd0), .state_info(5'd0), .route_info(64'd0),
    .active_route_steps(6'd0), .completed_route_steps(6'd0),
    .now_x(8'd0), .now_y(8'd0), .target_x(8'd0), .target_y(8'd0),
    .setting_mode(1'b0), .selected_param(3'd0), .step_length_set(16'd20),
    .overall_speed(10'd700), .lf_default_speed(10'd700),
    .rf_default_speed(10'd700), .lr_default_speed(10'd700),
    .rr_default_speed(10'd700), .lidar_online(1'b0),
    .lidar_obstacle_stop(1'b0), .ultrasonic_obstacle_stop(1'b0),
    .merged_obstacle_stop(1'b0), .motor_blocked(1'b0),
    .ultrasonic_distance_cm(16'hffff), .ultrasonic_distance_valid(1'b0),
    .ultrasonic_echo_timeout(1'b0), .lidar_front_distance_mm(16'hffff),
    .lidar_safe_mm(16'd450), .lidar_sector_front_mm(16'hffff),
    .lidar_sector_front_right_mm(16'hffff), .lidar_sector_right_mm(16'hffff),
    .lidar_sector_rear_right_mm(16'hffff), .lidar_sector_rear_mm(16'hffff),
    .lidar_sector_rear_left_mm(16'hffff), .lidar_sector_left_mm(16'hffff),
    .lidar_sector_front_left_mm(16'hffff), .c5_frame_count(16'd0),
    .c5_last_lf_pulse(12'd1500), .c5_last_rf_pulse(12'd1500),
    .c5_last_lr_pulse(12'd1500), .c5_last_rr_pulse(12'd1500),
    .c5_stop_frames_complete(1'b1), .c5_stop_frames_remaining(2'd0),
    .c5_frame_busy(1'b0), .map_chunk_valid(1'b0), .map_scan_seq(8'd0),
    .map_tick_10ms(16'd0), .map_part(3'd0), .map_chunk_data(144'd0),
    .map_chunk_crc(8'd0), .grid_chunk_valid(1'b0), .grid_chunk_seq(8'd0),
    .grid_chunk_base(14'd0), .grid_chunk_data(256'd0), .grid_chunk_crc(8'd0),
    .onboard_map_active(1'b0), .onboard_map_paused(1'b1),
    .onboard_map_valid(1'b0), .onboard_map_busy(1'b0),
    .onboard_map_epoch(8'd0), .onboard_map_scan_count(16'd0),
    .onboard_map_occupied_count(16'd0), .onboard_map_free_count(16'd0),
    .onboard_map_dropped_count(16'd0), .onboard_pose_x_mm(16'd0),
    .onboard_pose_y_mm(16'd0), .onboard_pose_heading(10'd0),
    .onboard_pose_flags(8'd0), .flash_ready(1'b0), .flash_busy(1'b0),
    .flash_operation(2'd0), .flash_progress(8'd0), .flash_error(8'd0),
    .flash_jedec_id(24'd0), .flash_saved_sequence(32'd0),
    .vl53_initialization_done(1'b0), .vl53_sensor_online(5'd0),
    .vl53_distance_valid(5'd0), .vl53_distance_mm(80'd0),
    .vl53_error_count(16'd0), .motion_telemetry_data(320'd0),
    .local_calibration_state(4'd0), .local_calibration_flags(8'd0),
    .learned_yaw_trim(11'd0), .yaw_correction(11'd0),
    .calibration_gyro_peak(16'd0), .bt_uart_tx(bt_uart_tx),
    .bt_connected(bt_connected)
  );

  function [7:0] expected_startup_byte;
    input integer byte_index;
    begin
      case (byte_index)
        0: expected_startup_byte = "A";
        1: expected_startup_byte = "T";
        2, 3, 4: expected_startup_byte = ".";
        5: expected_startup_byte = 8'h0d;
        6: expected_startup_byte = 8'h0a;
        7: expected_startup_byte = "A";
        8: expected_startup_byte = "T";
        9: expected_startup_byte = "+";
        10: expected_startup_byte = "R";
        11: expected_startup_byte = "E";
        12: expected_startup_byte = "S";
        13: expected_startup_byte = "E";
        14: expected_startup_byte = "T";
        15: expected_startup_byte = 8'h0d;
        default: expected_startup_byte = 8'h0a;
      endcase
    end
  endfunction

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
        $fatal(1, "UART stop bit was not high");
      repeat (CLKS_PER_BIT / 4) @(posedge clk);
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

  task read_line;
    begin
      line_length = 0;
      while ((line_length == 0) || (line[line_length - 1] != 8'h0a)) begin
        read_uart_byte(received_byte);
        line[line_length] = received_byte;
        line_length = line_length + 1;
      end
    end
  endtask

  initial begin
    repeat (8) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    wait (dut.bt_startup_ready);
    repeat (2000) begin
      @(posedge clk);
      if (bt_uart_tx !== 1'b1)
        $fatal(1, "unsolicited telemetry was sent before heartbeat");
    end

    fork
      send_uart_byte("P");
      read_line();
    join
    if (line_length != 6 ||
        {line[0], line[1], line[2], line[3], line[4]} !== "@PONG")
      $fatal(1, "heartbeat reply mismatch");
    if (!bt_connected)
      $fatal(1, "heartbeat did not mark link active");

    // Accelerate only the post-heartbeat loss timer. FPGA reset itself must
    // remain silent, while a proven link that later disappears may recover the
    // CH9141K with one documented serial reset sequence.
    repeat (4) @(posedge clk);
    force dut.link_active = 1'b0;
    dut.bt_recovery_count = dut.BT_RECOVERY_TRIGGER_CYCLES - 2;
    for (index = 0; index < 17; index = index + 1) begin
      read_uart_byte(received_byte);
      if (received_byte !== expected_startup_byte(index))
        $fatal(1, "recovery byte %0d mismatch got=%02x expected=%02x",
               index, received_byte, expected_startup_byte(index));
    end
    release dut.link_active;
    wait (dut.bt_startup_ready);

    // A normal FPGA reset arms one delayed recovery. This covers the case where
    // CH9141K was already wedged before the FPGA restarted and therefore cannot
    // provide a heartbeat. The sequence remains silent until the timeout.
    @(negedge clk);
    rst_n = 1'b0;
    repeat (4) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    repeat (dut.BT_STARTUP_RECOVERY_CYCLES - 2) begin
      @(posedge clk);
      if (bt_uart_tx !== 1'b1)
        $fatal(1, "startup recovery fired before timeout");
    end

    for (index = 0; index < 17; index = index + 1) begin
      read_uart_byte(received_byte);
      if (received_byte !== expected_startup_byte(index))
        $fatal(1, "startup recovery byte %0d mismatch got=%02x expected=%02x",
               index, received_byte, expected_startup_byte(index));
    end
    wait (dut.bt_startup_ready);

    $display("RESULT startup_sequence=delayed_single_recovery");
    $display("RESULT pre_heartbeat_uart_idle=1 pong=1 recovery_bytes=17 startup_recovery_bytes=17");
    $display("RESULT TEST_PASS bt_route_uart_startup");
    $finish;
  end

  initial begin
    #10000000;
    $fatal(1, "timeout");
  end
endmodule

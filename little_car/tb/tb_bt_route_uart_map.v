`timescale 1ns/1ps

module tb_bt_route_uart_map;
  localparam integer CLKS_PER_BIT = 50000000 / 115200;

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg bt_uart_rx = 1'b1;
  reg map_chunk_valid = 1'b0;
  reg [7:0] map_scan_seq = 8'h2a;
  reg [15:0] map_tick_10ms = 16'h1234;
  reg [2:0] map_part = 3'd3;
  reg [143:0] map_chunk_data = 144'd0;
  reg [7:0] map_chunk_crc = 8'ha5;

  wire bt_uart_tx;
  wire map_chunk_accept;
  reg accept_seen = 1'b0;
  integer accept_count = 0;
  reg [7:0] line [0:127];
  reg [7:0] rx_byte;
  integer line_length;
  integer word_index;
  integer data_index;
  integer map_found;
  reg [11:0] word_value;

  always #10 clk = ~clk;
  always @(posedge clk) begin
    if (map_chunk_accept) begin
      accept_seen <= 1'b1;
      accept_count <= accept_count + 1;
      // Model a ready/valid producer: valid goes low on the cycle after the
      // consumer accepts the item.
      map_chunk_valid <= 1'b0;
    end
  end

  bt_route_uart dut (
    .clk(clk),
    .rst_n(rst_n),
    .bt_uart_rx(bt_uart_rx),
    .menu_info(2'd0),
    .state_info(5'd0),
    .route_info(64'd0),
    .active_route_steps(6'd0),
    .completed_route_steps(6'd0),
    .now_x(8'd0),
    .now_y(8'd0),
    .target_x(8'd0),
    .target_y(8'd0),
    .setting_mode(1'b0),
    .selected_param(3'd0),
    .step_length_set(16'd20),
    .overall_speed(10'd700),
    .lf_default_speed(10'd700),
    .rf_default_speed(10'd700),
    .lr_default_speed(10'd700),
    .rr_default_speed(10'd700),
    .lidar_online(1'b1),
    .lidar_obstacle_stop(1'b0),
    .ultrasonic_obstacle_stop(1'b0),
    .merged_obstacle_stop(1'b0),
    .motor_blocked(1'b0),
    .ultrasonic_distance_cm(16'hffff),
    .ultrasonic_distance_valid(1'b0),
    .ultrasonic_echo_timeout(1'b0),
    .lidar_front_distance_mm(16'hffff),
    .lidar_safe_mm(16'd450),
    .lidar_sector_front_mm(16'hffff),
    .lidar_sector_front_right_mm(16'hffff),
    .lidar_sector_right_mm(16'hffff),
    .lidar_sector_rear_right_mm(16'hffff),
    .lidar_sector_rear_mm(16'hffff),
    .lidar_sector_rear_left_mm(16'hffff),
    .lidar_sector_left_mm(16'hffff),
    .lidar_sector_front_left_mm(16'hffff),
    .c5_frame_count(16'd0),
    .c5_last_lf_pulse(12'd1500),
    .c5_last_rf_pulse(12'd1500),
    .c5_last_lr_pulse(12'd1500),
    .c5_last_rr_pulse(12'd1500),
    .c5_stop_frames_complete(1'b1),
    .c5_stop_frames_remaining(2'd0),
    .c5_frame_busy(1'b0),
    .map_chunk_valid(map_chunk_valid),
    .map_scan_seq(map_scan_seq),
    .map_tick_10ms(map_tick_10ms),
    .map_part(map_part),
    .map_chunk_data(map_chunk_data),
    .map_chunk_crc(map_chunk_crc),
    .vl53_initialization_done(1'b0),
    .vl53_sensor_online(5'd0),
    .vl53_distance_valid(5'd0),
    .vl53_distance_mm(80'd0),
    .vl53_error_count(16'd0),
    .motion_telemetry_data(320'd0),
    .bt_uart_tx(bt_uart_tx),
    .bt_connected(),
    .bt_key_up_n(),
    .bt_key_down_n(),
    .bt_key_left_n(),
    .bt_key_right_n(),
    .bt_key_mid_n(),
    .bt_cfg_mode_pulse(),
    .bt_cfg_next_pulse(),
    .bt_cfg_prev_pulse(),
    .bt_cfg_inc_pulse(),
    .bt_cfg_dec_pulse(),
    .bt_cfg_select(),
    .bt_cfg_select_valid(),
    .manual_mode(),
    .manual_cmd(),
    .lidar_enable(),
    .motor_stop_pulse(),
    .map_chunk_accept(map_chunk_accept)
  );

  function [7:0] hex_ascii;
    input [3:0] value;
    begin
      hex_ascii = value + ((value < 4'd10) ? 8'd48 : 8'd55);
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
        $fatal(1, "UART stop bit is not high");
      repeat (CLKS_PER_BIT / 4) @(posedge clk);
    end
  endtask

  initial begin
    // Twelve recognizable words encoded in the sampler's nibble-stream order.
    for (word_index = 0; word_index < 12; word_index = word_index + 1) begin
      word_value = 12'h100 + word_index;
      map_chunk_data[word_index * 12 +: 4] = word_value[11:8];
      map_chunk_data[word_index * 12 + 4 +: 4] = word_value[7:4];
      map_chunk_data[word_index * 12 + 8 +: 4] = word_value[3:0];
    end

    repeat (8) @(posedge clk);
    rst_n = 1'b1;
    wait (dut.bt_startup_ready);
    force dut.link_active = 1'b1;
    // Model a continuously changing route/safety state. Map telemetry still
    // has to make forward progress instead of waiting for this flag to clear.
    force dut.route_status_pending = 1'b1;
    map_chunk_valid = 1'b1;

    // Ignore initial @S status and stop when the first complete @R line arrives.
    line_length = 0;
    map_found = 0;
    while (!map_found) begin
      read_uart_byte(rx_byte);
      if (line_length == 0) begin
        if (rx_byte == "@") begin
          line[0] = rx_byte;
          line_length = 1;
        end
      end else begin
        line[line_length] = rx_byte;
        line_length = line_length + 1;
        if (rx_byte == 8'h0a) begin
          if ((line_length >= 2) && (line[1] == "R"))
            map_found = 1;
          else
            line_length = 0;
        end
      end
    end

    if (line_length != 50)
      $fatal(1, "map line length=%0d, expected 50", line_length);
    if ({line[1], line[2]} !== "R1")
      $fatal(1, "map version header mismatch");
    if ({line[3], line[4]} !== "2A")
      $fatal(1, "map sequence mismatch");
    if ({line[5], line[6], line[7], line[8]} !== "1234")
      $fatal(1, "map tick mismatch");
    if ((line[9] !== "3") || (line[10] !== "6"))
      $fatal(1, "map part/count mismatch");

    for (data_index = 0; data_index < 36; data_index = data_index + 1)
      if (line[11 + data_index] !==
          hex_ascii(map_chunk_data[data_index * 4 +: 4]))
        $fatal(1, "map payload mismatch at nibble %0d", data_index);

    if ({line[47], line[48]} !== "A5")
      $fatal(1, "map CRC text mismatch");
    if (line[49] !== 8'h0a)
      $fatal(1, "map line terminator mismatch");

    repeat (CLKS_PER_BIT * 80) @(posedge clk);
    if (!accept_seen)
      $fatal(1, "map chunk was not acknowledged when transmission started");
    if (accept_count != 1)
      $fatal(1, "map chunk acknowledged/sent more than once: %0d", accept_count);

    release dut.route_status_pending;
    release dut.link_active;
    $display("RESULT map_line_bytes=%0d seq=2A tick=1234 part=3/6 crc=A5", line_length);
    $display("RESULT map_accept_count=%0d duplicate_frames=0", accept_count);
    $display("RESULT TEST_PASS bt_route_uart_map");
    $finish;
  end

  initial begin
    #30000000;
    $fatal(1, "timeout waiting for @R map frame");
  end
endmodule

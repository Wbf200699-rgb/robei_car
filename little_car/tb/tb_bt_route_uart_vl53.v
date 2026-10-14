`timescale 1ns/1ps

module tb_bt_route_uart_vl53;
  localparam integer CLKS_PER_BIT = 50000000 / 115200;

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg bt_uart_rx = 1'b1;
  wire bt_uart_tx;

  reg [7:0] line [0:127];
  reg [7:0] pong_line [0:15];
  reg [7:0] rx_byte;
  integer line_length;
  integer pong_length;

  always #10 clk = ~clk;

  bt_route_uart dut (
    .clk(clk), .rst_n(rst_n), .bt_uart_rx(bt_uart_rx),
    .menu_info(2'd0), .state_info(5'd0), .route_info(64'd0),
    .active_route_steps(6'd0), .completed_route_steps(6'd0),
    .now_x(8'd0), .now_y(8'd0), .target_x(8'd0), .target_y(8'd0),
    .setting_mode(1'b0), .selected_param(3'd0), .step_length_set(16'd0),
    .overall_speed(10'd0), .lf_default_speed(10'd0),
    .rf_default_speed(10'd0), .lr_default_speed(10'd0),
    .rr_default_speed(10'd0), .lidar_online(1'b0),
    .lidar_obstacle_stop(1'b0), .ultrasonic_obstacle_stop(1'b0),
    .merged_obstacle_stop(1'b0), .motor_blocked(1'b0),
    .ultrasonic_distance_cm(16'hffff), .ultrasonic_distance_valid(1'b0),
    .ultrasonic_echo_timeout(1'b0), .lidar_front_distance_mm(16'hffff),
    .lidar_safe_mm(16'd0), .lidar_sector_front_mm(16'hffff),
    .lidar_sector_front_right_mm(16'hffff), .lidar_sector_right_mm(16'hffff),
    .lidar_sector_rear_right_mm(16'hffff), .lidar_sector_rear_mm(16'hffff),
    .lidar_sector_rear_left_mm(16'hffff), .lidar_sector_left_mm(16'hffff),
    .lidar_sector_front_left_mm(16'hffff), .c5_frame_count(16'd0),
    .c5_last_lf_pulse(12'd0), .c5_last_rf_pulse(12'd0),
    .c5_last_lr_pulse(12'd0), .c5_last_rr_pulse(12'd0),
    .c5_stop_frames_complete(1'b0), .c5_stop_frames_remaining(2'd0),
    .c5_frame_busy(1'b0), .map_chunk_valid(1'b0), .map_scan_seq(8'd0),
    .map_tick_10ms(16'd0), .map_part(3'd0), .map_chunk_data(144'd0),
    .map_chunk_crc(8'd0),
    .vl53_initialization_done(1'b1), .vl53_sensor_online(5'b10101),
    .vl53_distance_valid(5'b10011),
    .vl53_distance_mm({16'h0abc, 16'h0789, 16'h0120, 16'h0456, 16'h0123}),
    .vl53_error_count(16'h002a),
    .motion_telemetry_data(320'd0),
    .bt_uart_tx(bt_uart_tx), .bt_connected(), .bt_key_up_n(),
    .bt_key_down_n(), .bt_key_left_n(), .bt_key_right_n(), .bt_key_mid_n(),
    .bt_cfg_mode_pulse(), .bt_cfg_next_pulse(), .bt_cfg_prev_pulse(),
    .bt_cfg_inc_pulse(), .bt_cfg_dec_pulse(), .bt_cfg_select(),
    .bt_cfg_select_valid(), .manual_mode(), .manual_cmd(), .lidar_enable(),
    .motor_stop_pulse(), .map_chunk_accept()
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
        $fatal(1, "UART stop bit is not high");
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

  initial begin
    repeat (8) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    wait (dut.bt_startup_ready);
    force dut.link_active = 1'b1;

    // Keep a route update pending while VL53 telemetry becomes due. The
    // sensor frame must still win one arbitration slot and remain atomic.
    dut.route_status_pending = 1'b1;
    dut.prev_lidar_enable = 1'b1;
    dut.status_count = 32'd0;
    dut.vl53_status_count = dut.VL53_STATUS_CYCLES - 1;

    fork
      begin
        wait (dut.sending_vl53);
        repeat (CLKS_PER_BIT * 24) @(posedge clk);
        send_uart_byte("P");
      end
      begin
        line_length = 0;
        while ((line_length == 0) || (line[line_length - 1] != 8'h0a)) begin
          read_uart_byte(rx_byte);
          line[line_length] = rx_byte;
          line_length = line_length + 1;
        end

        pong_length = 0;
        while ((pong_length == 0) || (pong_line[pong_length - 1] != 8'h0a)) begin
          read_uart_byte(rx_byte);
          pong_line[pong_length] = rx_byte;
          pong_length = pong_length + 1;
        end
      end
    join

    if (line_length != 33)
      $fatal(1, "VL53 line length=%0d, expected 33", line_length);
    if ({line[0], line[1], line[2], line[3], line[4], line[5], line[6], line[7],
         line[8], line[9], line[10], line[11], line[12], line[13], line[14],
         line[15], line[16], line[17], line[18], line[19], line[20], line[21],
         line[22], line[23], line[24], line[25], line[26], line[27], line[28],
         line[29], line[30], line[31]} !== "@V00951301230456012007890ABC002A")
      $fatal(1, "VL53 frame payload mismatch");
    if (line[32] !== 8'h0a)
      $fatal(1, "VL53 frame terminator mismatch");
    if (pong_length != 6 ||
        {pong_line[0], pong_line[1], pong_line[2], pong_line[3], pong_line[4]}
        !== "@PONG" || pong_line[5] !== 8'h0a)
      $fatal(1, "PONG was missing or interleaved with the VL53 frame");

    $display("RESULT vl53_frame=@V00951301230456012007890ABC002A");
    $display("RESULT atomic_followup=@PONG");
    $display("RESULT TEST_PASS bt_route_uart_vl53");
    release dut.link_active;
    $finish;
  end

  initial begin
    #30000000;
    $fatal(1, "timeout waiting for VL53 telemetry");
  end
endmodule

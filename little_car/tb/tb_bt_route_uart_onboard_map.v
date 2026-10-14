`timescale 1ns/1ps

module tb_bt_route_uart_onboard_map;
  localparam integer CLKS_PER_BIT = 50000000 / 115200;

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg bt_uart_rx = 1'b1;
  reg grid_chunk_valid = 1'b0;
  reg [255:0] grid_chunk_data = 256'd0;
  reg [7:0] onboard_pose_flags = 8'ha5;
  wire bt_uart_tx;
  wire grid_chunk_accept;
  wire map_start_pulse;
  wire map_pause_pulse;
  wire map_clear_pulse;
  wire map_save_pulse;
  wire map_load_pulse;
  wire map_sync_pulse;
  wire gyro_bias_learn_pulse;
  wire manual_mode;
  wire [2:0] manual_cmd;
  wire motor_stop_pulse;

  reg [7:0] line [0:255];
  reg [7:0] rx_byte;
  integer line_length;
  integer found;
  integer nibble_index;
  integer start_count = 0;
  integer pause_count = 0;
  integer clear_count = 0;
  integer save_count = 0;
  integer load_count = 0;
  integer sync_count = 0;
  integer bias_learn_count = 0;
  integer accept_count = 0;
  integer stop_count = 0;
  reg [7:0] grid_sequence_before_flags;

  always #10 clk = ~clk;

  always @(posedge clk) begin
    if (map_start_pulse) start_count <= start_count + 1;
    if (map_pause_pulse) pause_count <= pause_count + 1;
    if (map_clear_pulse) clear_count <= clear_count + 1;
    if (map_save_pulse) save_count <= save_count + 1;
    if (map_load_pulse) load_count <= load_count + 1;
    if (map_sync_pulse) sync_count <= sync_count + 1;
    if (gyro_bias_learn_pulse) bias_learn_count <= bias_learn_count + 1;
    if (motor_stop_pulse) stop_count <= stop_count + 1;
    if (grid_chunk_accept) begin
      accept_count <= accept_count + 1;
      grid_chunk_valid <= 1'b0;
    end
  end

  bt_route_uart dut (
    .clk(clk), .rst_n(rst_n), .bt_uart_rx(bt_uart_rx),
    .menu_info(2'd0), .state_info(5'd0), .route_info(64'd0),
    .active_route_steps(6'd0), .completed_route_steps(6'd0),
    .now_x(8'd0), .now_y(8'd0), .target_x(8'd0), .target_y(8'd0),
    .setting_mode(1'b0), .selected_param(3'd0),
    .step_length_set(16'd20), .overall_speed(10'd700),
    .lf_default_speed(10'd700), .rf_default_speed(10'd700),
    .lr_default_speed(10'd700), .rr_default_speed(10'd700),
    .lidar_online(1'b1), .lidar_obstacle_stop(1'b0),
    .ultrasonic_obstacle_stop(1'b0), .merged_obstacle_stop(1'b0),
    .motor_blocked(1'b0), .ultrasonic_distance_cm(16'hffff),
    .ultrasonic_distance_valid(1'b0), .ultrasonic_echo_timeout(1'b0),
    .lidar_front_distance_mm(16'hffff), .lidar_safe_mm(16'd450),
    .lidar_sector_front_mm(16'hffff),
    .lidar_sector_front_right_mm(16'hffff),
    .lidar_sector_right_mm(16'hffff),
    .lidar_sector_rear_right_mm(16'hffff),
    .lidar_sector_rear_mm(16'hffff),
    .lidar_sector_rear_left_mm(16'hffff),
    .lidar_sector_left_mm(16'hffff),
    .lidar_sector_front_left_mm(16'hffff),
    .c5_frame_count(16'd0), .c5_last_lf_pulse(12'd1500),
    .c5_last_rf_pulse(12'd1500), .c5_last_lr_pulse(12'd1500),
    .c5_last_rr_pulse(12'd1500), .c5_stop_frames_complete(1'b1),
    .c5_stop_frames_remaining(2'd0), .c5_frame_busy(1'b0),
    .map_chunk_valid(1'b0), .map_scan_seq(8'd0),
    .map_tick_10ms(16'd0), .map_part(3'd0),
    .map_chunk_data(144'd0), .map_chunk_crc(8'd0),
    .grid_chunk_valid(grid_chunk_valid), .grid_chunk_seq(8'ha7),
    .grid_chunk_base(14'h0180), .grid_chunk_data(grid_chunk_data),
    .grid_chunk_crc(8'h5c),
    .onboard_map_active(1'b1), .onboard_map_paused(1'b0),
    .onboard_map_valid(1'b1), .onboard_map_busy(1'b0),
    .onboard_map_epoch(8'h2a), .onboard_map_scan_count(16'h1234),
    .onboard_map_occupied_count(16'h0011),
    .onboard_map_free_count(16'h0022),
    .onboard_map_dropped_count(16'h0003),
    .onboard_pose_x_mm(16'h0c80), .onboard_pose_y_mm(16'h0c4e),
    .onboard_pose_heading(10'h155), .onboard_pose_flags(onboard_pose_flags),
    .flash_ready(1'b1), .flash_busy(1'b0), .flash_operation(2'd2),
    .flash_progress(8'h7f), .flash_error(8'h00),
    .flash_jedec_id(24'h20ba18), .flash_saved_sequence(32'h01020304),
    .vl53_initialization_done(1'b0), .vl53_sensor_online(5'd0),
    .vl53_distance_valid(5'd0), .vl53_distance_mm(80'd0),
    .vl53_error_count(16'd0), .motion_telemetry_data(320'd0),
    .local_calibration_state(4'ha), .local_calibration_flags(8'h42),
    .learned_yaw_trim(11'd0), .yaw_correction(11'd0),
    .calibration_gyro_peak(16'd0),
    .bt_uart_tx(bt_uart_tx), .bt_connected(), .bt_key_up_n(),
    .bt_key_down_n(), .bt_key_left_n(), .bt_key_right_n(), .bt_key_mid_n(),
    .bt_cfg_mode_pulse(), .bt_cfg_next_pulse(), .bt_cfg_prev_pulse(),
    .bt_cfg_inc_pulse(), .bt_cfg_dec_pulse(), .bt_cfg_select(),
    .bt_cfg_select_valid(), .manual_mode(manual_mode), .manual_cmd(manual_cmd),
    .lidar_enable(), .motor_stop_pulse(motor_stop_pulse), .local_calibration_start_pulse(),
    .map_chunk_accept(), .onboard_map_start_pulse(map_start_pulse),
    .onboard_map_pause_pulse(map_pause_pulse),
    .onboard_map_clear_pulse(map_clear_pulse),
    .onboard_map_save_pulse(map_save_pulse),
    .onboard_map_load_pulse(map_load_pulse),
    .onboard_map_sync_pulse(map_sync_pulse),
    .gyro_bias_learn_pulse(gyro_bias_learn_pulse),
    .grid_chunk_accept(grid_chunk_accept)
  );

  function [7:0] hex_ascii;
    input [3:0] value;
    begin
      hex_ascii = value + ((value < 4'd10) ? 8'd48 : 8'd55);
    end
  endfunction

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
      repeat (CLKS_PER_BIT * 2) @(posedge clk);
    end
  endtask

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

  task read_until_grid_type;
    input [7:0] subtype;
    begin
      line_length = 0;
      found = 0;
      while (!found) begin
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
            if ((line_length >= 3) && (line[1] == "G") &&
                (line[2] == subtype))
              found = 1;
            else
              line_length = 0;
          end
        end
      end
    end
  endtask

  initial begin
    for (nibble_index = 0; nibble_index < 64; nibble_index = nibble_index + 1)
      grid_chunk_data[nibble_index * 4 +: 4] = nibble_index[3:0];

    repeat (8) @(posedge clk);
    rst_n = 1'b1;
    wait (dut.bt_startup_ready);
    force dut.link_active = 1'b1;

    read_until_grid_type("0");
    $write("DEBUG G0=");
    for (nibble_index = 0; nibble_index < line_length; nibble_index = nibble_index + 1)
      $write("%c", line[nibble_index]);
    if (line_length != 58) $fatal(1, "G0 length=%0d", line_length);
    if ({line[3], line[4]} !== "00") $fatal(1, "G0 sequence mismatch");
    if ({line[5], line[6]} !== "D5") $fatal(1, "G0 flags mismatch");
    if ({line[7], line[8]} !== "2A") $fatal(1, "G0 epoch mismatch");
    if ({line[9], line[10], line[11], line[12]} !== "0C80")
      $fatal(1, "G0 X pose mismatch");
    if ({line[13], line[14], line[15], line[16]} !== "0C4E")
      $fatal(1, "G0 Y pose mismatch");
    if ({line[41], line[42]} !== "00") $fatal(1, "G0 flash error mismatch");
    if ({line[43], line[44], line[45], line[46], line[47], line[48]} !==
        "20BA18") $fatal(1, "G0 JEDEC mismatch");
    if ({line[49], line[50], line[51], line[52], line[53], line[54],
         line[55], line[56]} !== "01020304")
      $fatal(1, "G0 saved sequence mismatch");

    // Calibration and gyro-polarity validity live only in pose_flags. Their
    // transition must independently produce a fresh G0 even while the map and
    // pose are stationary, otherwise the App can retain a stale pre-calibration
    // snapshot and refuse to start exploration.
    wait (dut.prev_onboard_pose_flags == 8'ha5);
    grid_sequence_before_flags = dut.grid_status_seq;
    onboard_pose_flags = 8'ha7;
    repeat (8) @(posedge clk);
    if (dut.prev_onboard_pose_flags != 8'ha7 ||
        !(dut.grid_status_pending || dut.sending_grid_status ||
          (dut.grid_status_seq != grid_sequence_before_flags)))
      $fatal(1, "G0 pose-flag change was not scheduled");

    // Decode every control byte through the real 115200-baud receiver.
    send_uart_byte("J");
    repeat (20) @(posedge clk);
    if (manual_mode || manual_cmd != 3'd0)
      $fatal(1, "J did not release stopped manual ownership");
    send_uart_byte("K");
    repeat (20) @(posedge clk);
    if (!manual_mode || manual_cmd != 3'd0)
      $fatal(1, "K did not atomically enter persistent stop");
    send_uart_byte("6");
    send_uart_byte("7");
    send_uart_byte("8");
    send_uart_byte("9");
    send_uart_byte("Y");
    repeat (20) @(posedge clk);
    if ((start_count != 1) || (pause_count != 1) || (clear_count != 1) ||
        (save_count != 1) || (load_count != 1) || (sync_count != 1) ||
        (bias_learn_count != 1))
      $fatal(1, "command pulses=%0d,%0d,%0d,%0d,%0d,%0d,%0d",
             start_count, pause_count, clear_count,
             save_count, load_count, sync_count, bias_learn_count);
    if (stop_count < 2) $fatal(1, "J/K stop pulse count=%0d", stop_count);

    // Commands also schedule ACK and @S frames. Start the byte reader only
    // after those complete so it cannot mistake a data-bit transition for a
    // fresh UART start bit.
    wait (!dut.pending_reply && !dut.sending_reply && !dut.sending_status &&
          !dut.sending_map && !dut.sending_vl53 && !dut.sending_motion &&
          !dut.sending_grid_status && !dut.sending_grid_chunk &&
          !dut.route_status_pending && !dut.tx_busy);
    repeat (CLKS_PER_BIT * 2) @(posedge clk);
    grid_chunk_valid = 1'b1;
    read_until_grid_type("1");
    if (line_length != 76) $fatal(1, "G1 length=%0d", line_length);
    if ({line[3], line[4]} !== "A7") $fatal(1, "G1 sequence mismatch");
    if ({line[5], line[6], line[7], line[8]} !== "0180")
      $fatal(1, "G1 base mismatch");
    for (nibble_index = 0; nibble_index < 64; nibble_index = nibble_index + 1)
      if (line[9 + nibble_index] !== hex_ascii(nibble_index[3:0]))
        $fatal(1, "G1 payload mismatch nibble=%0d", nibble_index);
    if ({line[73], line[74]} !== "5C") $fatal(1, "G1 CRC mismatch");
    repeat (CLKS_PER_BIT * 20) @(posedge clk);
    if (accept_count != 1) $fatal(1, "G1 accept count=%0d", accept_count);

    $display("RESULT grid_status_bytes=58 flags=D5 epoch=2A flash=20BA18");
    $display("RESULT grid_chunk_bytes=76 seq=A7 base=0180 crc=5C");
    $display("RESULT map_command_pulses=%0d,%0d,%0d,%0d,%0d,%0d bias=%0d",
             start_count, pause_count, clear_count,
             save_count, load_count, sync_count, bias_learn_count);
    $display("RESULT TEST_PASS bt_route_uart_onboard_map");
    release dut.link_active;
    $finish;
  end

  initial begin
    #1000000000;
    $fatal(1, "timeout waiting for onboard map protocol");
  end
endmodule

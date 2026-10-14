`timescale 1ns/1ps

module tb_linkpg_tag_position_uart;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg uwb_uart_rx = 1'b1;
  wire uwb_uart_tx;
  wire position_valid;
  wire position_online;
  wire signed [15:0] position_x_cm;
  wire signed [15:0] position_y_cm;
  wire signed [15:0] position_z_cm;

  reg [7:0] frame [0:63];
  integer index;
  integer tx_capture_count = 0;
  reg [7:0] tx_capture [0:15];

  always #10 clk = ~clk;

  linkpg_tag_position_uart dut (
    .clk(clk),
    .rst_n(rst_n),
    .uwb_uart_rx(uwb_uart_rx),
    .uwb_uart_tx(uwb_uart_tx),
    .position_valid(position_valid),
    .position_online(position_online),
    .position_good(),
    .position_x_cm(position_x_cm),
    .position_y_cm(position_y_cm),
    .position_z_cm(position_z_cm),
    .distance_valid_mask(),
    .distance_a_cm(),
    .distance_b_cm(),
    .distance_c_cm(),
    .distance_d_cm(),
    .rx_packet_count(),
    .rx_crc_error_count(),
    .rx_framing_error_count(),
    .start_request_count(),
    .start_ack_seen()
  );

  function [15:0] crc16_next;
    input [15:0] crc_in;
    input [7:0] data_in;
    integer bit_number;
    reg [15:0] value;
    begin
      value = crc_in ^ data_in;
      for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1)
        value = value[0] ? ((value >> 1) ^ 16'hA001) : (value >> 1);
      crc16_next = value;
    end
  endfunction

  task send_uart_byte;
    input [7:0] value;
    integer bit_number;
    begin
      @(negedge clk);
      uwb_uart_rx = 1'b0;
      repeat (4) @(posedge clk);
      for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
        uwb_uart_rx = value[bit_number];
        repeat (4) @(posedge clk);
      end
      uwb_uart_rx = 1'b1;
      repeat (4) @(posedge clk);
    end
  endtask

  task send_position_frame;
    input signed [15:0] x_value;
    input signed [15:0] y_value;
    input signed [15:0] z_value;
    input corrupt_crc;
    integer byte_number;
    reg [15:0] crc;
    begin
      // Tag-local AC DA frame, output flags 0x0002 (coordinates only).
      frame[0] = 8'h01; frame[1] = 8'h03; frame[2] = 8'd12;
      frame[3] = 8'hAC; frame[4] = 8'hDA;
      frame[5] = 8'h00; frame[6] = 8'h02;
      frame[7] = 8'h00; frame[8] = 8'h01;
      frame[9] = x_value[15:8]; frame[10] = x_value[7:0];
      frame[11] = y_value[15:8]; frame[12] = y_value[7:0];
      frame[13] = z_value[15:8]; frame[14] = z_value[7:0];
      crc = 16'hffff;
      for (byte_number = 0; byte_number < 15; byte_number = byte_number + 1)
        crc = crc16_next(crc, frame[byte_number]);
      frame[15] = crc[7:0] ^ (corrupt_crc ? 8'h01 : 8'h00);
      frame[16] = crc[15:8];
      for (byte_number = 0; byte_number < 17; byte_number = byte_number + 1)
        send_uart_byte(frame[byte_number]);
    end
  endtask

  task send_long_position_frame;
    input signed [15:0] x_value;
    input signed [15:0] y_value;
    input signed [15:0] z_value;
    integer byte_number;
    reg [15:0] crc;
    begin
      // Tag-local AC DA frame with both distance and coordinate output flags.
      // Position tag ID is at bytes 41..42 in this long vendor format.
      for (byte_number = 0; byte_number < 64; byte_number = byte_number + 1)
        frame[byte_number] = 8'd0;
      frame[0] = 8'h01; frame[1] = 8'h03; frame[2] = 8'd46;
      frame[3] = 8'hAC; frame[4] = 8'hDA;
      frame[5] = 8'h00; frame[6] = 8'h03;
      frame[41] = 8'h00; frame[42] = 8'h01;
      frame[43] = x_value[15:8]; frame[44] = x_value[7:0];
      frame[45] = y_value[15:8]; frame[46] = y_value[7:0];
      frame[47] = z_value[15:8]; frame[48] = z_value[7:0];
      crc = 16'hffff;
      for (byte_number = 0; byte_number < 49; byte_number = byte_number + 1)
        crc = crc16_next(crc, frame[byte_number]);
      frame[49] = crc[7:0]; frame[50] = crc[15:8];
      for (byte_number = 0; byte_number < 51; byte_number = byte_number + 1)
        send_uart_byte(frame[byte_number]);
    end
  endtask

  always @(posedge clk) begin
    if (dut.tx_start && tx_capture_count < 16) begin
      tx_capture[tx_capture_count] <= dut.tx_data;
      tx_capture_count <= tx_capture_count + 1;
    end
  end

  initial begin
    repeat (5) @(posedge clk);
    rst_n = 1'b1;

    // With no incoming stream, the bounded recovery command must exactly
    // match the vendor V6.6 register-59 start frame.
    wait (tx_capture_count >= 11);
    if (tx_capture[0] !== 8'h01 || tx_capture[1] !== 8'h10 ||
        tx_capture[3] !== 8'h3B || tx_capture[8] !== 8'h04 ||
        tx_capture[9] !== 8'hA3 || tx_capture[10] !== 8'h18)
      $fatal(1, "LinkPG start frame mismatch");

    send_position_frame(16'sd123, -16'sd45, 16'sd10, 1'b0);
    wait (position_valid);
    @(posedge clk);
    if (!position_online || position_x_cm !== 16'sd123 ||
        position_y_cm !== -16'sd45 || position_z_cm !== 16'sd10)
      $fatal(1, "first position was not decoded");

    send_position_frame(16'sd125, -16'sd43, 16'sd11, 1'b0);
    send_position_frame(16'sd124, -16'sd44, 16'sd10, 1'b0);
    wait (position_valid);
    @(posedge clk);
    if (position_x_cm < 16'sd123 || position_x_cm > 16'sd125 ||
        position_y_cm < -16'sd45 || position_y_cm > -16'sd43)
      $fatal(1, "position filter result is outside sample bounds");

    send_long_position_frame(16'sd130, -16'sd40, 16'sd12);
    wait (position_valid);
    @(posedge clk);
    if (position_x_cm < 16'sd124 || position_x_cm > 16'sd130 ||
        position_y_cm < -16'sd44 || position_y_cm > -16'sd40)
      $fatal(1, "long AC DA position frame was not decoded");

    send_position_frame(16'sd200, 16'sd200, 16'sd0, 1'b1);
    repeat (20) @(posedge clk);
    if (dut.rx_crc_error_count == 16'd0)
      $fatal(1, "corrupt position frame was not rejected");

    $display("RESULT start_frame=0110003B0001020004A318");
    $display("RESULT position_cm=%0d,%0d,%0d",
             $signed(position_x_cm), $signed(position_y_cm),
             $signed(position_z_cm));
    $display("RESULT packets=%0d crc_errors=%0d framing_errors=%0d start_requests=%0d online=%0d good=%0d",
             dut.rx_packet_count, dut.rx_crc_error_count,
             dut.rx_framing_error_count, dut.start_request_count,
             position_online, dut.position_good);
    $display("RESULT TEST_PASS linkpg_tag_position_uart");
    $finish;
  end

  initial begin
    #20000000;
    $display("RESULT TEST_FAIL linkpg_tag_position_uart timeout=1");
    $finish;
  end
endmodule

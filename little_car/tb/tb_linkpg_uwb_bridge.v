`timescale 1ns/1ps

module tb_linkpg_uwb_bridge;
  localparam integer CLK_PERIOD_NS = 20;
  localparam integer UART_CLKS = 434;
  localparam integer UART_BIT_NS = CLK_PERIOD_NS * UART_CLKS;

  reg clk;
  reg rst_n;
  reg uwb_uart_rx;
  wire uwb_uart_tx;
  reg bt_uart_rx_in;
  reg bt_uart_tx_tap;
  wire bt_uart_rx_merged;
  wire uwb_link_online;
  wire [15:0] rx_packet_count;
  wire [15:0] rx_crc_error_count;
  wire [15:0] tx_packet_count;
  wire [15:0] telemetry_drop_count;
  wire telemetry_ready;

  reg [7:0] frame_bytes [0:31];
  reg [7:0] uwb_tx_bytes [0:255];
  reg [7:0] merged_bytes [0:15];
  integer uwb_tx_count;
  integer merged_count;
  integer errors;
  integer long_report_start;
  integer long_report_target;
  integer report_index;

  linkpg_uwb_bridge dut (
      .clk(clk),
      .rst_n(rst_n),
      .uwb_uart_rx(uwb_uart_rx),
      .uwb_uart_tx(uwb_uart_tx),
      .bt_uart_rx_in(bt_uart_rx_in),
      .bt_uart_tx_tap(bt_uart_tx_tap),
      .bt_uart_rx_merged(bt_uart_rx_merged),
      .uwb_link_online(uwb_link_online),
      .rx_packet_count(rx_packet_count),
      .rx_crc_error_count(rx_crc_error_count),
      .tx_packet_count(tx_packet_count),
      .telemetry_drop_count(telemetry_drop_count),
      .telemetry_ready(telemetry_ready)
  );

  always #(CLK_PERIOD_NS / 2) clk = ~clk;

  function [15:0] crc16_next;
    input [15:0] crc_in;
    input [7:0] data_in;
    integer i;
    reg [15:0] value;
    begin
      value = crc_in ^ data_in;
      for (i = 0; i < 8; i = i + 1) begin
        if (value[0])
          value = (value >> 1) ^ 16'hA001;
        else
          value = value >> 1;
      end
      crc16_next = value;
    end
  endfunction

  function [7:0] crc8_next;
    input [7:0] crc_in;
    input [7:0] data_in;
    integer i;
    reg [7:0] value;
    begin
      value = crc_in ^ data_in;
      for (i = 0; i < 8; i = i + 1) begin
        if (value[7])
          value = (value << 1) ^ 8'h07;
        else
          value = value << 1;
      end
      crc8_next = value;
    end
  endfunction

  function [7:0] hex_ascii;
    input [3:0] value;
    begin
      hex_ascii = (value < 10) ? ("0" + value) : ("A" + value - 10);
    end
  endfunction

  task uart_send_uwb;
    input [7:0] value;
    integer i;
    begin
      uwb_uart_rx = 1'b0;
      #(UART_BIT_NS);
      for (i = 0; i < 8; i = i + 1) begin
        uwb_uart_rx = value[i];
        #(UART_BIT_NS);
      end
      uwb_uart_rx = 1'b1;
      #(UART_BIT_NS);
    end
  endtask

  task uart_send_tap;
    input [7:0] value;
    integer i;
    begin
      bt_uart_tx_tap = 1'b0;
      #(UART_BIT_NS);
      for (i = 0; i < 8; i = i + 1) begin
        bt_uart_tx_tap = value[i];
        #(UART_BIT_NS);
      end
      bt_uart_tx_tap = 1'b1;
      #(UART_BIT_NS);
    end
  endtask

  task send_command_frame;
    input [7:0] sequence_in;
    input [7:0] command_byte;
    input corrupt_modbus_crc;
    integer i;
    reg [7:0] app_crc;
    reg [15:0] modbus_crc;
    begin
      app_crc = 8'h00;
      app_crc = crc8_next(app_crc, "C");
      app_crc = crc8_next(app_crc, "1");
      app_crc = crc8_next(app_crc, hex_ascii(sequence_in[7:4]));
      app_crc = crc8_next(app_crc, hex_ascii(sequence_in[3:0]));
      app_crc = crc8_next(app_crc, command_byte);

      frame_bytes[0] = 8'h01;
      frame_bytes[1] = 8'h03;
      frame_bytes[2] = 8'd11;
      frame_bytes[3] = 8'hED;
      frame_bytes[4] = 8'hDA;
      frame_bytes[5] = 8'h00;
      frame_bytes[6] = 8'hFF;
      frame_bytes[7] = "C";
      frame_bytes[8] = "1";
      frame_bytes[9] = hex_ascii(sequence_in[7:4]);
      frame_bytes[10] = hex_ascii(sequence_in[3:0]);
      frame_bytes[11] = command_byte;
      frame_bytes[12] = hex_ascii(app_crc[7:4]);
      frame_bytes[13] = hex_ascii(app_crc[3:0]);

      modbus_crc = 16'hFFFF;
      for (i = 0; i < 14; i = i + 1)
        modbus_crc = crc16_next(modbus_crc, frame_bytes[i]);
      frame_bytes[14] = modbus_crc[7:0] ^ (corrupt_modbus_crc ? 8'h01 : 8'h00);
      frame_bytes[15] = modbus_crc[15:8];

      for (i = 0; i < 16; i = i + 1)
        uart_send_uwb(frame_bytes[i]);
    end
  endtask

  task expect_byte;
    input [7:0] actual;
    input [7:0] expected;
    input [8*32-1:0] label_text;
    begin
      if (actual !== expected) begin
        $display("ERROR %0s actual=%02x expected=%02x", label_text, actual, expected);
        errors = errors + 1;
      end
    end
  endtask

  // Decode every byte emitted toward the LinkPG module.
  always begin : monitor_uwb_tx
    integer i;
    reg [7:0] value;
    @(negedge uwb_uart_tx);
    #(UART_BIT_NS + UART_BIT_NS / 2);
    for (i = 0; i < 8; i = i + 1) begin
      value[i] = uwb_uart_tx;
      #(UART_BIT_NS);
    end
    #(UART_BIT_NS / 2);
    uwb_tx_bytes[uwb_tx_count] = value;
    uwb_tx_count = uwb_tx_count + 1;
  end

  // Decode injected command bytes on the merged Bluetooth RX line.
  always begin : monitor_merged_rx
    integer i;
    reg [7:0] value;
    @(negedge bt_uart_rx_merged);
    #(UART_BIT_NS + UART_BIT_NS / 2);
    for (i = 0; i < 8; i = i + 1) begin
      value[i] = bt_uart_rx_merged;
      #(UART_BIT_NS);
    end
    #(UART_BIT_NS / 2);
    merged_bytes[merged_count] = value;
    merged_count = merged_count + 1;
  end

  initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    uwb_uart_rx = 1'b1;
    bt_uart_rx_in = 1'b1;
    bt_uart_tx_tap = 1'b1;
    uwb_tx_count = 0;
    merged_count = 0;
    errors = 0;

    #(CLK_PERIOD_NS * 20);
    rst_n = 1'b1;
    #(UART_BIT_NS * 20);

    // A valid command from main anchor ID 255 must become one BLE RX byte.
    send_command_frame(8'h2A, "W", 1'b0);
    wait (merged_count == 1);
    expect_byte(merged_bytes[0], "W", "injected command");
    wait (tx_packet_count == 1);
    if (!uwb_link_online) begin
      $display("ERROR UWB link did not become online");
      errors = errors + 1;
    end
    if (rx_packet_count != 1) begin
      $display("ERROR valid RX packet count=%0d", rx_packet_count);
      errors = errors + 1;
    end
    // The first outbound packet is AT+DataSend="A12AW0xx","255" CR LF.
    expect_byte(uwb_tx_bytes[0], "A", "ack prefix A");
    expect_byte(uwb_tx_bytes[12], 8'h22, "ack opening quote");
    expect_byte(uwb_tx_bytes[13], "A", "ack envelope type");
    expect_byte(uwb_tx_bytes[15], "2", "ack sequence high");
    expect_byte(uwb_tx_bytes[16], "A", "ack sequence low");
    expect_byte(uwb_tx_bytes[17], "W", "ack command");
    expect_byte(uwb_tx_bytes[18], "0", "ack accepted result");

    // Feed an existing Bluetooth report. It must be wrapped as one F1 packet.
    uart_send_tap("@");
    uart_send_tap("K");
    uart_send_tap("Q");
    uart_send_tap(8'h0A);
    wait (tx_packet_count == 2);
    expect_byte(uwb_tx_bytes[30 + 13], "F", "report envelope type");
    expect_byte(uwb_tx_bytes[30 + 17], "0", "report part");
    expect_byte(uwb_tx_bytes[30 + 18], "1", "report total");
    expect_byte(uwb_tx_bytes[30 + 19], "0", "report length high");
    expect_byte(uwb_tx_bytes[30 + 20], "3", "report length low");
    expect_byte(uwb_tx_bytes[30 + 21], "@", "report data 0");
    expect_byte(uwb_tx_bytes[30 + 22], "K", "report data 1");
    expect_byte(uwb_tx_bytes[30 + 23], "Q", "report data 2");

    // Repeating the same sequence and command inside the duplicate window must
    // return result 1 and must not inject a second movement command.
    send_command_frame(8'h2A, "W", 1'b0);
    wait (tx_packet_count == 3);
    if (merged_count != 1) begin
      $display("ERROR duplicate command was injected, merged_count=%0d", merged_count);
      errors = errors + 1;
    end
    // Second packet was 35 bytes, so duplicate ack begins at byte 65.
    expect_byte(uwb_tx_bytes[65 + 18], "1", "duplicate result");

    // A bad Modbus CRC must be counted and ignored without a reply/injection.
    send_command_frame(8'h2B, "C", 1'b1);
    #(UART_BIT_NS * 30);
    if (rx_crc_error_count != 1) begin
      $display("ERROR CRC error count=%0d", rx_crc_error_count);
      errors = errors + 1;
    end
    if (merged_count != 1 || tx_packet_count != 3) begin
      $display("ERROR bad CRC changed command/TX counts merged=%0d tx=%0d",
               merged_count, tx_packet_count);
      errors = errors + 1;
    end

    // A partial frame followed by silence must time out. The next complete
    // frame must then be parsed normally instead of being consumed as payload
    // belonging to the abandoned frame.
    uart_send_uwb(8'h01);
    uart_send_uwb(8'h03);
    uart_send_uwb(8'd11);
    uart_send_uwb(8'hED);
    uart_send_uwb(8'hDA);
    uart_send_uwb(8'h00);
    #(12_000_000);
    send_command_frame(8'h2C, "C", 1'b0);
    wait (merged_count == 2);
    wait (tx_packet_count == 4);
    expect_byte(merged_bytes[1], "C", "command after truncated frame");
    if (rx_packet_count != 3) begin
      $display("ERROR recovered RX packet count=%0d", rx_packet_count);
      errors = errors + 1;
    end

    // The 49-byte @R body is the largest normal Bluetooth report. It must fit
    // in one UWB F1 fragment: length 0x31, part 0 of 1, with every data byte
    // retained. The terminating newline belongs to the UART framing only and
    // is intentionally excluded from the UWB payload.
    // tx_packet_count changes in the bridge clock domain, while this UART
    // monitor records the final byte in a separate simulation process. Wait
    // for both sides and all arbitration queues to become idle before taking
    // the byte-array offset for the next packet.
    wait (telemetry_ready && !dut.ack_pending && !dut.tx_payload_ready &&
          (dut.stream_state == 0) && !dut.byte_tx_busy);
    #(UART_BIT_NS * 2);
    long_report_start = uwb_tx_count;
    long_report_target = tx_packet_count + 1;
    uart_send_tap("@");
    uart_send_tap("R");
    for (report_index = 0; report_index < 47; report_index = report_index + 1)
      uart_send_tap("A" + (report_index % 26));
    uart_send_tap(8'h0A);
    wait (tx_packet_count == long_report_target);
    #(UART_BIT_NS * 2);
    expect_byte(uwb_tx_bytes[long_report_start + 13], "F", "map envelope type");
    expect_byte(uwb_tx_bytes[long_report_start + 17], "0", "map part");
    expect_byte(uwb_tx_bytes[long_report_start + 18], "1", "map total");
    expect_byte(uwb_tx_bytes[long_report_start + 19], "3", "map length high");
    expect_byte(uwb_tx_bytes[long_report_start + 20], "1", "map length low");
    expect_byte(uwb_tx_bytes[long_report_start + 21], "@", "map data first");
    expect_byte(uwb_tx_bytes[long_report_start + 22], "R", "map data type");
    expect_byte(uwb_tx_bytes[long_report_start + 69], "U", "map data last");
    if ((uwb_tx_count - long_report_start) != 81) begin
      $display("ERROR map UWB packet bytes=%0d expected=81",
               uwb_tx_count - long_report_start);
      errors = errors + 1;
    end

    if (telemetry_drop_count != 0) begin
      $display("ERROR unexpected telemetry drops=%0d", telemetry_drop_count);
      errors = errors + 1;
    end

    if (errors == 0)
      $display("RESULT TEST_PASS linkpg_uwb_bridge");
    else
      $display("RESULT TEST_FAIL linkpg_uwb_bridge errors=%0d", errors);
    $finish;
  end

  initial begin
    #(350_000_000);
    $display("RESULT TEST_TIMEOUT linkpg_uwb_bridge");
    $finish;
  end
endmodule

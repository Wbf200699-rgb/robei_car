module linkpg_uwb_bridge(
input wire clk,
input wire rst_n,
input wire uwb_uart_rx,
output wire uwb_uart_tx,
input wire bt_uart_rx_in,
input wire bt_uart_tx_tap,
output wire bt_uart_rx_merged,
output reg uwb_link_online,
output reg [15:0] rx_packet_count,
output reg [15:0] rx_crc_error_count,
output reg [15:0] tx_packet_count,
output reg [15:0] telemetry_drop_count,
output wire telemetry_ready,
output reg debug_last_ack_valid,
output reg [7:0] debug_last_sequence,
output reg [1:0] debug_last_result
);

  //----Code starts here: integrated by Robei-----
  localparam integer CLK_HZ = 50000000;
  localparam integer BAUD = 115200;
  localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
  localparam integer HALF_BIT_CLKS = CLKS_PER_BIT / 2;
  localparam integer BT_IDLE_GUARD_CLKS = CLKS_PER_BIT * 12;
  localparam integer COMMAND_WAIT_CLKS = CLK_HZ / 10;
  localparam integer DUPLICATE_WINDOW_CLKS = CLK_HZ * 2;
  localparam integer UWB_LINK_TIMEOUT_CLKS = CLK_HZ * 3;
  localparam integer FRAME_IDLE_TIMEOUT_CLKS = CLK_HZ / 100;
  localparam integer TX_GAP_CLKS = CLK_HZ / 40;

  localparam [1:0] UART_IDLE  = 2'd0;
  localparam [1:0] UART_START = 2'd1;
  localparam [1:0] UART_DATA  = 2'd2;
  localparam [1:0] UART_STOP  = 2'd3;

  localparam [2:0] BUILD_IDLE        = 3'd0;
  localparam [2:0] BUILD_REPORT_DATA = 3'd1;
  localparam [2:0] BUILD_REPORT_CRC  = 3'd2;

  localparam [2:0] STREAM_IDLE    = 3'd0;
  localparam [2:0] STREAM_PREFIX  = 3'd1;
  localparam [2:0] STREAM_PAYLOAD = 3'd2;
  localparam [2:0] STREAM_SUFFIX  = 3'd3;
  localparam [2:0] STREAM_WAIT    = 3'd4;

  localparam [1:0] PAYLOAD_NONE   = 2'd0;
  localparam [1:0] PAYLOAD_ACK    = 2'd1;
  localparam [1:0] PAYLOAD_REPORT = 2'd2;

  function [15:0] crc16_modbus_next;
    input [15:0] crc_in;
    input [7:0] data_in;
    integer bit_index;
    reg [15:0] value;
    begin
      value = crc_in ^ data_in;
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (value[0])
          value = (value >> 1) ^ 16'hA001;
        else
          value = value >> 1;
      end
      crc16_modbus_next = value;
    end
  endfunction

  function [7:0] crc8_next;
    input [7:0] crc_in;
    input [7:0] data_in;
    integer bit_index;
    reg [7:0] value;
    begin
      value = crc_in ^ data_in;
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (value[7])
          value = (value << 1) ^ 8'h07;
        else
          value = value << 1;
      end
      crc8_next = value;
    end
  endfunction

  function is_hex_ascii;
    input [7:0] value;
    begin
      is_hex_ascii = ((value >= "0") && (value <= "9")) ||
                     ((value >= "A") && (value <= "F")) ||
                     ((value >= "a") && (value <= "f"));
    end
  endfunction

  function [3:0] hex_value;
    input [7:0] value;
    begin
      if ((value >= "0") && (value <= "9"))
        hex_value = value - "0";
      else if ((value >= "A") && (value <= "F"))
        hex_value = value - "A" + 4'd10;
      else
        hex_value = value - "a" + 4'd10;
    end
  endfunction

  function [7:0] hex_ascii;
    input [3:0] value;
    begin
      if (value < 10)
        hex_ascii = "0" + value;
      else
        hex_ascii = "A" + value - 4'd10;
    end
  endfunction

  function allowed_command;
    input [7:0] value;
    begin
      case (value)
        "P", "p", "?", "W", "U", "S", "D", "A", "L", "R", "C", "M",
        "H", "O", "E", "Q", "V", "T", "1", "2", "3", "4", "5", "Z",
        "+", "-", "N", "B", "G": allowed_command = 1'b1;
        default: allowed_command = 1'b0;
      endcase
    end
  endfunction

  function [7:0] command_crc8;
    input [7:0] sequence_in;
    input [7:0] command_byte;
    reg [7:0] value;
    begin
      value = crc8_next(8'h00, "C");
      value = crc8_next(value, "1");
      value = crc8_next(value, hex_ascii(sequence_in[7:4]));
      value = crc8_next(value, hex_ascii(sequence_in[3:0]));
      value = crc8_next(value, command_byte);
      command_crc8 = value;
    end
  endfunction

  function [7:0] ack_crc8;
    input [7:0] sequence_in;
    input [7:0] command_byte;
    input [1:0] result;
    reg [7:0] value;
    begin
      value = crc8_next(8'h00, "A");
      value = crc8_next(value, "1");
      value = crc8_next(value, hex_ascii(sequence_in[7:4]));
      value = crc8_next(value, hex_ascii(sequence_in[3:0]));
      value = crc8_next(value, command_byte);
      value = crc8_next(value, "0" + result);
      ack_crc8 = value;
    end
  endfunction

  function [7:0] prefix_byte;
    input [3:0] index;
    begin
      case (index)
        4'd0: prefix_byte = "A";
        4'd1: prefix_byte = "T";
        4'd2: prefix_byte = "+";
        4'd3: prefix_byte = "D";
        4'd4: prefix_byte = "a";
        4'd5: prefix_byte = "t";
        4'd6: prefix_byte = "a";
        4'd7: prefix_byte = "S";
        4'd8: prefix_byte = "e";
        4'd9: prefix_byte = "n";
        4'd10: prefix_byte = "d";
        4'd11: prefix_byte = "=";
        default: prefix_byte = 8'h22;
      endcase
    end
  endfunction

  function [7:0] suffix_byte;
    input [3:0] index;
    begin
      case (index)
        4'd0: suffix_byte = 8'h22;
        4'd1: suffix_byte = ",";
        4'd2: suffix_byte = 8'h22;
        4'd3: suffix_byte = "2";
        4'd4: suffix_byte = "5";
        4'd5: suffix_byte = "5";
        4'd6: suffix_byte = 8'h22;
        4'd7: suffix_byte = 8'h0D;
        default: suffix_byte = 8'h0A;
      endcase
    end
  endfunction

  // UWB receive UART. The two-flop synchronizer is retained even though the
  // LinkPG and FPGA grounds are shared because the clocks are unrelated.
  reg [1:0] uwb_rx_sync;
  reg [1:0] uwb_rx_state;
  reg [15:0] uwb_rx_clock;
  reg [2:0] uwb_rx_bit;
  reg [7:0] uwb_rx_shift;
  reg [7:0] uwb_rx_byte;
  reg uwb_rx_valid;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uwb_rx_sync <= 2'b11;
      uwb_rx_state <= UART_IDLE;
      uwb_rx_clock <= 16'd0;
      uwb_rx_bit <= 3'd0;
      uwb_rx_shift <= 8'd0;
      uwb_rx_byte <= 8'd0;
      uwb_rx_valid <= 1'b0;
    end else begin
      uwb_rx_sync <= {uwb_rx_sync[0], uwb_uart_rx};
      uwb_rx_valid <= 1'b0;
      case (uwb_rx_state)
        UART_IDLE: begin
          uwb_rx_clock <= 16'd0;
          uwb_rx_bit <= 3'd0;
          if (!uwb_rx_sync[1])
            uwb_rx_state <= UART_START;
        end
        UART_START: begin
          if (uwb_rx_clock >= HALF_BIT_CLKS - 1) begin
            uwb_rx_clock <= 16'd0;
            if (!uwb_rx_sync[1])
              uwb_rx_state <= UART_DATA;
            else
              uwb_rx_state <= UART_IDLE;
          end else begin
            uwb_rx_clock <= uwb_rx_clock + 16'd1;
          end
        end
        UART_DATA: begin
          if (uwb_rx_clock >= CLKS_PER_BIT - 1) begin
            uwb_rx_clock <= 16'd0;
            uwb_rx_shift[uwb_rx_bit] <= uwb_rx_sync[1];
            if (uwb_rx_bit == 3'd7) begin
              uwb_rx_bit <= 3'd0;
              uwb_rx_state <= UART_STOP;
            end else begin
              uwb_rx_bit <= uwb_rx_bit + 3'd1;
            end
          end else begin
            uwb_rx_clock <= uwb_rx_clock + 16'd1;
          end
        end
        default: begin
          if (uwb_rx_clock >= CLKS_PER_BIT - 1) begin
            uwb_rx_clock <= 16'd0;
            uwb_rx_state <= UART_IDLE;
            if (uwb_rx_sync[1]) begin
              uwb_rx_byte <= uwb_rx_shift;
              uwb_rx_valid <= 1'b1;
            end
          end else begin
            uwb_rx_clock <= uwb_rx_clock + 16'd1;
          end
        end
      endcase
    end
  end

  // The LinkPG frame parser buffers only the beginning of the pass payload.
  // A command envelope is seven bytes, so no large receive RAM is required.
  reg [6:0] uwb_frame_index;
  reg [6:0] uwb_frame_expected;
  reg [7:0] uwb_frame_byte_count;
  reg [15:0] uwb_frame_crc;
  reg [7:0] uwb_marker_high;
  reg [7:0] uwb_marker_low;
  reg [7:0] uwb_sender_high;
  reg [7:0] uwb_sender_low;
  reg [7:0] uwb_payload [0:7];
  reg [18:0] uwb_frame_idle_count;

  reg new_command_valid;
  reg [7:0] new_command_sequence;
  reg [7:0] new_command_byte;
  reg parser_ack_valid;
  reg [7:0] parser_ack_sequence;
  reg [7:0] parser_ack_command;
  reg [1:0] parser_ack_result;

  reg last_command_valid;
  reg [7:0] last_command_sequence;
  reg [7:0] last_command_byte;
  reg [26:0] duplicate_age;

  wire [15:0] received_crc_next = crc16_modbus_next(uwb_frame_crc, uwb_rx_byte);
  wire [7:0] received_command_sequence = {hex_value(uwb_payload[2]), hex_value(uwb_payload[3])};
  wire [7:0] received_command_crc = {hex_value(uwb_payload[5]), hex_value(uwb_payload[6])};
  wire received_command_format_ok =
      (uwb_frame_byte_count >= 8'd11) &&
      (uwb_payload[0] == "C") && (uwb_payload[1] == "1") &&
      is_hex_ascii(uwb_payload[2]) && is_hex_ascii(uwb_payload[3]) &&
      is_hex_ascii(uwb_payload[5]) && is_hex_ascii(uwb_payload[6]) &&
      allowed_command(uwb_payload[4]) &&
      (received_command_crc == command_crc8(received_command_sequence, uwb_payload[4]));

  wire command_accept_ready;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uwb_frame_index <= 7'd0;
      uwb_frame_expected <= 7'd0;
      uwb_frame_byte_count <= 8'd0;
      uwb_frame_crc <= 16'hFFFF;
      uwb_marker_high <= 8'd0;
      uwb_marker_low <= 8'd0;
      uwb_sender_high <= 8'd0;
      uwb_sender_low <= 8'd0;
      uwb_frame_idle_count <= 19'd0;
      new_command_valid <= 1'b0;
      new_command_sequence <= 8'd0;
      new_command_byte <= 8'd0;
      parser_ack_valid <= 1'b0;
      parser_ack_sequence <= 8'd0;
      parser_ack_command <= 8'd0;
      parser_ack_result <= 2'd0;
      last_command_valid <= 1'b0;
      last_command_sequence <= 8'd0;
      last_command_byte <= 8'd0;
      duplicate_age <= 27'd0;
      rx_packet_count <= 16'd0;
      rx_crc_error_count <= 16'd0;
    end else begin
      new_command_valid <= 1'b0;
      parser_ack_valid <= 1'b0;

      if (last_command_valid) begin
        if (duplicate_age >= DUPLICATE_WINDOW_CLKS - 1) begin
          last_command_valid <= 1'b0;
          duplicate_age <= 27'd0;
        end else begin
          duplicate_age <= duplicate_age + 27'd1;
        end
      end

      // A truncated Modbus frame must not leave the parser consuming the next
      // valid frame as stale payload. Ten milliseconds is over one hundred
      // UART character times at 115200 baud, while remaining short enough for
      // an application retry to recover without a visible control pause.
      if ((uwb_frame_index != 0) && !uwb_rx_valid) begin
        if (uwb_frame_idle_count >= FRAME_IDLE_TIMEOUT_CLKS - 1) begin
          uwb_frame_index <= 7'd0;
          uwb_frame_expected <= 7'd0;
          uwb_frame_byte_count <= 8'd0;
          uwb_frame_crc <= 16'hFFFF;
          uwb_frame_idle_count <= 19'd0;
        end else begin
          uwb_frame_idle_count <= uwb_frame_idle_count + 19'd1;
        end
      end else begin
        uwb_frame_idle_count <= 19'd0;
      end

      if (uwb_rx_valid) begin
        if (uwb_frame_index == 0) begin
          if (uwb_rx_byte == 8'h01) begin
            uwb_frame_index <= 7'd1;
            uwb_frame_crc <= crc16_modbus_next(16'hFFFF, uwb_rx_byte);
          end
        end else if (uwb_frame_index == 1) begin
          if (uwb_rx_byte == 8'h03) begin
            uwb_frame_index <= 7'd2;
            uwb_frame_crc <= received_crc_next;
          end else begin
            uwb_frame_index <= (uwb_rx_byte == 8'h01) ? 7'd1 : 7'd0;
            uwb_frame_crc <= (uwb_rx_byte == 8'h01) ?
                             crc16_modbus_next(16'hFFFF, uwb_rx_byte) : 16'hFFFF;
          end
        end else if (uwb_frame_index == 2) begin
          if ((uwb_rx_byte >= 8'd4) && (uwb_rx_byte <= 8'd84)) begin
            uwb_frame_byte_count <= uwb_rx_byte;
            uwb_frame_expected <= uwb_rx_byte + 7'd5;
            uwb_frame_index <= 7'd3;
            uwb_frame_crc <= received_crc_next;
          end else begin
            uwb_frame_index <= 7'd0;
            uwb_frame_crc <= 16'hFFFF;
          end
        end else begin
          uwb_frame_crc <= received_crc_next;
          if (uwb_frame_index == 3) uwb_marker_high <= uwb_rx_byte;
          if (uwb_frame_index == 4) uwb_marker_low <= uwb_rx_byte;
          if (uwb_frame_index == 5) uwb_sender_high <= uwb_rx_byte;
          if (uwb_frame_index == 6) uwb_sender_low <= uwb_rx_byte;
          if ((uwb_frame_index >= 7) && (uwb_frame_index <= 14))
            uwb_payload[uwb_frame_index - 7] <= uwb_rx_byte;

          if (uwb_frame_index == uwb_frame_expected - 1) begin
            uwb_frame_index <= 7'd0;
            uwb_frame_crc <= 16'hFFFF;
            if (received_crc_next != 16'h0000) begin
              rx_crc_error_count <= rx_crc_error_count + 16'd1;
            end else if ((uwb_marker_high == 8'hED) && (uwb_marker_low == 8'hDA) &&
                         (uwb_sender_high == 8'h00) && (uwb_sender_low == 8'hFF)) begin
              rx_packet_count <= rx_packet_count + 16'd1;
              if (received_command_format_ok) begin
                if (last_command_valid &&
                    (received_command_sequence == last_command_sequence) &&
                    (uwb_payload[4] == last_command_byte)) begin
                  parser_ack_valid <= 1'b1;
                  parser_ack_sequence <= received_command_sequence;
                  parser_ack_command <= uwb_payload[4];
                  parser_ack_result <= 2'd1;
                end else if (command_accept_ready) begin
                  new_command_valid <= 1'b1;
                  new_command_sequence <= received_command_sequence;
                  new_command_byte <= uwb_payload[4];
                  last_command_valid <= 1'b1;
                  last_command_sequence <= received_command_sequence;
                  last_command_byte <= uwb_payload[4];
                  duplicate_age <= 27'd0;
                end else begin
                  parser_ack_valid <= 1'b1;
                  parser_ack_sequence <= received_command_sequence;
                  parser_ack_command <= uwb_payload[4];
                  parser_ack_result <= 2'd2;
                end
              end
            end
          end else begin
            uwb_frame_index <= uwb_frame_index + 7'd1;
          end
        end
      end
    end
  end

  // A valid data-pass frame refreshes link state independently of whether its
  // application envelope is a command supported by this firmware revision.
  reg [27:0] uwb_link_idle_count;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uwb_link_online <= 1'b0;
      uwb_link_idle_count <= 28'd0;
    end else if (uwb_rx_valid && (uwb_frame_index == uwb_frame_expected - 1) &&
                 (received_crc_next == 16'h0000) &&
                 (uwb_marker_high == 8'hED) && (uwb_marker_low == 8'hDA)) begin
      uwb_link_online <= 1'b1;
      uwb_link_idle_count <= 28'd0;
    end else if (uwb_link_online) begin
      if (uwb_link_idle_count >= UWB_LINK_TIMEOUT_CLKS - 1) begin
        uwb_link_online <= 1'b0;
        uwb_link_idle_count <= 28'd0;
      end else begin
        uwb_link_idle_count <= uwb_link_idle_count + 28'd1;
      end
    end
  end

  // UWB commands are serialized back into the existing Bluetooth receive
  // parser. Waiting for more than one idle UART character prevents normal BLE
  // traffic from being cut in half. The bitwise AND is a wired-idle merge: both
  // UART sources idle high, and either source may assert a start/data zero.
  reg command_pending;
  reg [7:0] pending_command_sequence;
  reg [7:0] pending_command_byte;
  reg [23:0] command_wait_count;
  reg [15:0] bt_idle_high_count;
  reg [1:0] inject_state;
  reg [15:0] inject_clock;
  reg [2:0] inject_bit;
  reg [7:0] inject_shift;
  reg inject_uart_line;
  reg inject_collision;
  reg inject_ack_valid;
  reg [7:0] inject_ack_sequence;
  reg [7:0] inject_ack_command;
  reg [1:0] inject_ack_result;

  assign command_accept_ready = !command_pending && (inject_state == UART_IDLE) && !new_command_valid;
  assign bt_uart_rx_merged = bt_uart_rx_in & inject_uart_line;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      command_pending <= 1'b0;
      pending_command_sequence <= 8'd0;
      pending_command_byte <= 8'd0;
      command_wait_count <= 24'd0;
      bt_idle_high_count <= 16'd0;
      inject_state <= UART_IDLE;
      inject_clock <= 16'd0;
      inject_bit <= 3'd0;
      inject_shift <= 8'd0;
      inject_uart_line <= 1'b1;
      inject_collision <= 1'b0;
      inject_ack_valid <= 1'b0;
      inject_ack_sequence <= 8'd0;
      inject_ack_command <= 8'd0;
      inject_ack_result <= 2'd0;
    end else begin
      inject_ack_valid <= 1'b0;

      if ((inject_state == UART_IDLE) && bt_uart_rx_in) begin
        if (bt_idle_high_count < BT_IDLE_GUARD_CLKS)
          bt_idle_high_count <= bt_idle_high_count + 16'd1;
      end else begin
        bt_idle_high_count <= 16'd0;
      end

      if (new_command_valid && !command_pending) begin
        command_pending <= 1'b1;
        pending_command_sequence <= new_command_sequence;
        pending_command_byte <= new_command_byte;
        command_wait_count <= 24'd0;
      end

      if ((inject_state != UART_IDLE) && !bt_uart_rx_in)
        inject_collision <= 1'b1;

      case (inject_state)
        UART_IDLE: begin
          inject_uart_line <= 1'b1;
          inject_clock <= 16'd0;
          if (command_pending) begin
            if (bt_idle_high_count >= BT_IDLE_GUARD_CLKS - 1) begin
              inject_state <= UART_START;
              inject_clock <= 16'd0;
              inject_bit <= 3'd0;
              inject_shift <= pending_command_byte;
              inject_uart_line <= 1'b0;
              inject_collision <= 1'b0;
              command_wait_count <= 24'd0;
            end else if (command_wait_count >= COMMAND_WAIT_CLKS - 1) begin
              command_pending <= 1'b0;
              command_wait_count <= 24'd0;
              inject_ack_valid <= 1'b1;
              inject_ack_sequence <= pending_command_sequence;
              inject_ack_command <= pending_command_byte;
              inject_ack_result <= 2'd2;
            end else begin
              command_wait_count <= command_wait_count + 24'd1;
            end
          end
        end
        UART_START: begin
          if (inject_clock >= CLKS_PER_BIT - 1) begin
            inject_clock <= 16'd0;
            inject_uart_line <= inject_shift[0];
            inject_state <= UART_DATA;
          end else begin
            inject_clock <= inject_clock + 16'd1;
          end
        end
        UART_DATA: begin
          if (inject_clock >= CLKS_PER_BIT - 1) begin
            inject_clock <= 16'd0;
            if (inject_bit == 3'd7) begin
              inject_bit <= 3'd0;
              inject_uart_line <= 1'b1;
              inject_state <= UART_STOP;
            end else begin
              inject_bit <= inject_bit + 3'd1;
              inject_uart_line <= inject_shift[inject_bit + 3'd1];
            end
          end else begin
            inject_clock <= inject_clock + 16'd1;
          end
        end
        default: begin
          if (inject_clock >= CLKS_PER_BIT - 1) begin
            inject_clock <= 16'd0;
            inject_state <= UART_IDLE;
            inject_uart_line <= 1'b1;
            command_pending <= 1'b0;
            inject_ack_valid <= 1'b1;
            inject_ack_sequence <= pending_command_sequence;
            inject_ack_command <= pending_command_byte;
            inject_ack_result <= inject_collision ? 2'd3 : 2'd0;
          end else begin
            inject_clock <= inject_clock + 16'd1;
          end
        end
      endcase
    end
  end

  // Decode the already-generated Bluetooth TX line. Capturing the final UART
  // stream means UWB carries exactly the same @S/@L/@K/@PONG representation as
  // BLE, without duplicating the large status formatter or its state snapshot.
  reg [1:0] tap_rx_state;
  reg [15:0] tap_rx_clock;
  reg [2:0] tap_rx_bit;
  reg [7:0] tap_rx_shift;
  reg [7:0] tap_rx_byte;
  reg tap_rx_valid;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tap_rx_state <= UART_IDLE;
      tap_rx_clock <= 16'd0;
      tap_rx_bit <= 3'd0;
      tap_rx_shift <= 8'd0;
      tap_rx_byte <= 8'd0;
      tap_rx_valid <= 1'b0;
    end else begin
      tap_rx_valid <= 1'b0;
      case (tap_rx_state)
        UART_IDLE: begin
          tap_rx_clock <= 16'd0;
          tap_rx_bit <= 3'd0;
          if (!bt_uart_tx_tap)
            tap_rx_state <= UART_START;
        end
        UART_START: begin
          if (tap_rx_clock >= HALF_BIT_CLKS - 1) begin
            tap_rx_clock <= 16'd0;
            if (!bt_uart_tx_tap)
              tap_rx_state <= UART_DATA;
            else
              tap_rx_state <= UART_IDLE;
          end else begin
            tap_rx_clock <= tap_rx_clock + 16'd1;
          end
        end
        UART_DATA: begin
          if (tap_rx_clock >= CLKS_PER_BIT - 1) begin
            tap_rx_clock <= 16'd0;
            tap_rx_shift[tap_rx_bit] <= bt_uart_tx_tap;
            if (tap_rx_bit == 3'd7) begin
              tap_rx_bit <= 3'd0;
              tap_rx_state <= UART_STOP;
            end else begin
              tap_rx_bit <= tap_rx_bit + 3'd1;
            end
          end else begin
            tap_rx_clock <= tap_rx_clock + 16'd1;
          end
        end
        default: begin
          if (tap_rx_clock >= CLKS_PER_BIT - 1) begin
            tap_rx_clock <= 16'd0;
            tap_rx_state <= UART_IDLE;
            if (bt_uart_tx_tap) begin
              tap_rx_byte <= tap_rx_shift;
              tap_rx_valid <= 1'b1;
            end
          end else begin
            tap_rx_clock <= tap_rx_clock + 16'd1;
          end
        end
      endcase
    end
  end

  reg [7:0] report_buffer [0:95];
  reg report_capture_active;
  reg [6:0] report_capture_length;
  reg report_capture_event;
  reg [6:0] captured_report_length;
  reg capture_event_consumed;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      report_capture_active <= 1'b0;
      report_capture_length <= 7'd0;
      report_capture_event <= 1'b0;
      captured_report_length <= 7'd0;
      telemetry_drop_count <= 16'd0;
    end else begin
      if (report_capture_event && capture_event_consumed)
        report_capture_event <= 1'b0;

      if (tap_rx_valid) begin
        if (!report_capture_active) begin
          if (tap_rx_byte == "@") begin
            if (!report_capture_event) begin
              report_capture_length <= 7'd1;
              report_capture_active <= 1'b1;
            end else begin
              telemetry_drop_count <= telemetry_drop_count + 16'd1;
            end
          end
        end else if (tap_rx_byte == 8'h0A) begin
          report_capture_active <= 1'b0;
          if (report_capture_length >= 2) begin
            captured_report_length <= report_capture_length;
            report_capture_event <= 1'b1;
          end else begin
            telemetry_drop_count <= telemetry_drop_count + 16'd1;
          end
        end else if (report_capture_length < 7'd96) begin
          // The leading '@' is implicit. Storing subsequent characters at
          // length-1 leaves every RAM word with one dynamic write site.
          report_buffer[report_capture_length - 7'd1] <= tap_rx_byte;
          report_capture_length <= report_capture_length + 7'd1;
        end else begin
          report_capture_active <= 1'b0;
          report_capture_length <= 7'd0;
          telemetry_drop_count <= telemetry_drop_count + 16'd1;
        end
      end
    end
  end

  // Ack events are queued ahead of periodic telemetry. One ack slot is enough
  // because each LinkPG command takes substantially longer than one FPGA clock
  // to arrive, and duplicate server retries are rate-limited by the server.
  reg ack_pending;
  reg [7:0] ack_sequence;
  reg [7:0] ack_command;
  reg [1:0] ack_result;
  reg ack_event_consumed;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ack_pending <= 1'b0;
      ack_sequence <= 8'd0;
      ack_command <= 8'd0;
      ack_result <= 2'd0;
      debug_last_ack_valid <= 1'b0;
      debug_last_sequence <= 8'd0;
      debug_last_result <= 2'd0;
    end else begin
      if (ack_event_consumed)
        ack_pending <= 1'b0;

      if (inject_ack_valid) begin
        if (!ack_pending || ack_event_consumed) begin
          ack_pending <= 1'b1;
          ack_sequence <= inject_ack_sequence;
          ack_command <= inject_ack_command;
          ack_result <= inject_ack_result;
          debug_last_ack_valid <= 1'b1;
          debug_last_sequence <= inject_ack_sequence;
          debug_last_result <= inject_ack_result;
        end
      end else if (parser_ack_valid) begin
        if (!ack_pending || ack_event_consumed) begin
          ack_pending <= 1'b1;
          ack_sequence <= parser_ack_sequence;
          ack_command <= parser_ack_command;
          ack_result <= parser_ack_result;
          debug_last_ack_valid <= 1'b1;
          debug_last_sequence <= parser_ack_sequence;
          debug_last_result <= parser_ack_result;
        end
      end
    end
  end

  reg report_pending;
  reg [7:0] report_message_sequence;
  reg [6:0] report_length;
  reg report_part;
  reg [1:0] report_total;
  reg report_fragment_inflight;
  reg report_build_started;
  reg report_packet_sent;

  wire [6:0] report_fragment_offset = report_part ? 7'd64 : 7'd0;
  wire [6:0] report_fragment_length =
      report_part ? (report_length - 7'd64) :
      ((report_length > 7'd64) ? 7'd64 : report_length);

  // High-resolution map chunks start only when this capture slot is free.
  // Existing control replies remain independent and retain their low latency.
  assign telemetry_ready = !report_capture_active && !report_capture_event &&
                           !report_pending && !report_fragment_inflight;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      report_pending <= 1'b0;
      report_message_sequence <= 8'd0;
      report_length <= 7'd0;
      report_part <= 1'b0;
      report_total <= 2'd0;
      report_fragment_inflight <= 1'b0;
      capture_event_consumed <= 1'b0;
    end else begin
      capture_event_consumed <= 1'b0;
      if (report_capture_event && !report_pending && !report_fragment_inflight) begin
        report_pending <= 1'b1;
        report_length <= captured_report_length;
        report_part <= 1'b0;
        report_total <= (captured_report_length > 7'd64) ? 2'd2 : 2'd1;
      end

      if (report_packet_sent) begin
        report_fragment_inflight <= 1'b0;
        if (report_part + 1'b1 < report_total) begin
          report_part <= report_part + 1'b1;
        end else begin
          report_pending <= 1'b0;
          report_part <= 1'b0;
          report_message_sequence <= report_message_sequence + 8'd1;
          // Keep the capture event asserted until every fragment has left the
          // UART. This locks report_buffer and prevents a newer BLE line from
          // overwriting bytes while the current report is being packetized.
          capture_event_consumed <= 1'b1;
        end
      end

      if (report_build_started)
        report_fragment_inflight <= 1'b1;
    end
  end

  // Payload builder. At most 74 bytes are produced, below the vendor's 80-byte
  // data-pass limit. Report data is copied in one byte per clock so the buffer
  // maps cleanly and does not create a large combinational concatenation.
  reg [7:0] tx_payload_buffer [0:73];
  reg [6:0] tx_payload_length;
  reg [1:0] tx_payload_kind;
  reg tx_payload_ready;
  reg [2:0] build_state;
  reg [6:0] build_index;
  reg [7:0] build_crc;
  reg payload_sent;

  wire [7:0] current_ack_crc = ack_crc8(ack_sequence, ack_command, ack_result);
  wire [6:0] current_report_data_index = report_fragment_offset + build_index;
  wire [7:0] current_report_data_byte =
      (current_report_data_index == 0) ? "@" :
      report_buffer[current_report_data_index - 7'd1];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_payload_length <= 7'd0;
      tx_payload_kind <= PAYLOAD_NONE;
      tx_payload_ready <= 1'b0;
      build_state <= BUILD_IDLE;
      build_index <= 7'd0;
      build_crc <= 8'd0;
      ack_event_consumed <= 1'b0;
      report_build_started <= 1'b0;
      report_packet_sent <= 1'b0;
    end else begin
      ack_event_consumed <= 1'b0;
      report_build_started <= 1'b0;
      report_packet_sent <= 1'b0;
      if (payload_sent) begin
        tx_payload_ready <= 1'b0;
        if (tx_payload_kind == PAYLOAD_REPORT)
          report_packet_sent <= 1'b1;
        tx_payload_kind <= PAYLOAD_NONE;
      end

      case (build_state)
        BUILD_IDLE: begin
          if (!tx_payload_ready) begin
            if (ack_pending) begin
              tx_payload_buffer[0] <= "A";
              tx_payload_buffer[1] <= "1";
              tx_payload_buffer[2] <= hex_ascii(ack_sequence[7:4]);
              tx_payload_buffer[3] <= hex_ascii(ack_sequence[3:0]);
              tx_payload_buffer[4] <= ack_command;
              tx_payload_buffer[5] <= "0" + ack_result;
              tx_payload_buffer[6] <= hex_ascii(current_ack_crc[7:4]);
              tx_payload_buffer[7] <= hex_ascii(current_ack_crc[3:0]);
              tx_payload_length <= 7'd8;
              tx_payload_kind <= PAYLOAD_ACK;
              tx_payload_ready <= 1'b1;
              ack_event_consumed <= 1'b1;
            end else if (report_pending && !report_fragment_inflight) begin
              tx_payload_buffer[0] <= "F";
              tx_payload_buffer[1] <= "1";
              tx_payload_buffer[2] <= hex_ascii(report_message_sequence[7:4]);
              tx_payload_buffer[3] <= hex_ascii(report_message_sequence[3:0]);
              tx_payload_buffer[4] <= hex_ascii({3'd0, report_part});
              tx_payload_buffer[5] <= hex_ascii({2'd0, report_total});
              tx_payload_buffer[6] <= hex_ascii({1'b0, report_fragment_length[6:4]});
              tx_payload_buffer[7] <= hex_ascii(report_fragment_length[3:0]);
              build_crc <= crc8_next(
                  crc8_next(
                    crc8_next(
                      crc8_next(
                        crc8_next(
                          crc8_next(
                            crc8_next(
                              crc8_next(8'h00, "F"), "1"),
                            hex_ascii(report_message_sequence[7:4])),
                          hex_ascii(report_message_sequence[3:0])),
                        hex_ascii({3'd0, report_part})),
                      hex_ascii({2'd0, report_total})),
                    hex_ascii({1'b0, report_fragment_length[6:4]})),
                  hex_ascii(report_fragment_length[3:0]));
              build_index <= 7'd0;
              build_state <= BUILD_REPORT_DATA;
              report_build_started <= 1'b1;
            end
          end
        end
        BUILD_REPORT_DATA: begin
          tx_payload_buffer[8 + build_index] <= current_report_data_byte;
          build_crc <= crc8_next(build_crc, current_report_data_byte);
          if (build_index + 7'd1 >= report_fragment_length) begin
            build_index <= 7'd0;
            build_state <= BUILD_REPORT_CRC;
          end else begin
            build_index <= build_index + 7'd1;
          end
        end
        default: begin
          tx_payload_buffer[8 + report_fragment_length] <= hex_ascii(build_crc[7:4]);
          tx_payload_buffer[9 + report_fragment_length] <= hex_ascii(build_crc[3:0]);
          tx_payload_length <= report_fragment_length + 7'd10;
          tx_payload_kind <= PAYLOAD_REPORT;
          tx_payload_ready <= 1'b1;
          build_state <= BUILD_IDLE;
        end
      endcase
    end
  end

  // One-byte UWB UART transmitter used by the AT command stream formatter.
  reg [7:0] byte_tx_data;
  reg byte_tx_start;
  reg byte_tx_busy;
  reg [1:0] byte_tx_state;
  reg [15:0] byte_tx_clock;
  reg [2:0] byte_tx_bit;
  reg [7:0] byte_tx_shift;
  reg byte_tx_line;

  assign uwb_uart_tx = byte_tx_line;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      byte_tx_busy <= 1'b0;
      byte_tx_state <= UART_IDLE;
      byte_tx_clock <= 16'd0;
      byte_tx_bit <= 3'd0;
      byte_tx_shift <= 8'd0;
      byte_tx_line <= 1'b1;
    end else if (!byte_tx_busy) begin
      byte_tx_line <= 1'b1;
      byte_tx_state <= UART_IDLE;
      byte_tx_clock <= 16'd0;
      if (byte_tx_start) begin
        byte_tx_busy <= 1'b1;
        byte_tx_state <= UART_START;
        byte_tx_shift <= byte_tx_data;
        byte_tx_bit <= 3'd0;
        byte_tx_line <= 1'b0;
      end
    end else begin
      case (byte_tx_state)
        UART_START: begin
          if (byte_tx_clock >= CLKS_PER_BIT - 1) begin
            byte_tx_clock <= 16'd0;
            byte_tx_line <= byte_tx_shift[0];
            byte_tx_state <= UART_DATA;
          end else begin
            byte_tx_clock <= byte_tx_clock + 16'd1;
          end
        end
        UART_DATA: begin
          if (byte_tx_clock >= CLKS_PER_BIT - 1) begin
            byte_tx_clock <= 16'd0;
            if (byte_tx_bit == 3'd7) begin
              byte_tx_bit <= 3'd0;
              byte_tx_line <= 1'b1;
              byte_tx_state <= UART_STOP;
            end else begin
              byte_tx_bit <= byte_tx_bit + 3'd1;
              byte_tx_line <= byte_tx_shift[byte_tx_bit + 3'd1];
            end
          end else begin
            byte_tx_clock <= byte_tx_clock + 16'd1;
          end
        end
        default: begin
          if (byte_tx_clock >= CLKS_PER_BIT - 1) begin
            byte_tx_clock <= 16'd0;
            byte_tx_busy <= 1'b0;
            byte_tx_state <= UART_IDLE;
            byte_tx_line <= 1'b1;
          end else begin
            byte_tx_clock <= byte_tx_clock + 16'd1;
          end
        end
      endcase
    end
  end

  // Stream one complete AT+DataSend command. payload_sent is asserted only
  // after the final UART stop bit, so the payload RAM cannot be overwritten
  // while a byte is still being serialized.
  reg [2:0] stream_state;
  reg [6:0] stream_index;
  reg stream_saw_busy;
  reg [20:0] tx_gap_count;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      stream_state <= STREAM_IDLE;
      stream_index <= 7'd0;
      stream_saw_busy <= 1'b0;
      tx_gap_count <= 21'd0;
      byte_tx_data <= 8'd0;
      byte_tx_start <= 1'b0;
      payload_sent <= 1'b0;
      tx_packet_count <= 16'd0;
    end else begin
      byte_tx_start <= 1'b0;
      payload_sent <= 1'b0;
      if (tx_gap_count != 0)
        tx_gap_count <= tx_gap_count - 21'd1;

      case (stream_state)
        STREAM_IDLE: begin
          stream_index <= 7'd0;
          stream_saw_busy <= 1'b0;
          if (tx_payload_ready && (tx_gap_count == 0))
            stream_state <= STREAM_PREFIX;
        end
        STREAM_PREFIX: begin
          if (!byte_tx_busy && !byte_tx_start) begin
            byte_tx_data <= prefix_byte(stream_index[3:0]);
            byte_tx_start <= 1'b1;
            if (stream_index == 7'd12) begin
              stream_index <= 7'd0;
              stream_state <= STREAM_PAYLOAD;
            end else begin
              stream_index <= stream_index + 7'd1;
            end
          end
        end
        STREAM_PAYLOAD: begin
          if (!byte_tx_busy && !byte_tx_start) begin
            byte_tx_data <= tx_payload_buffer[stream_index];
            byte_tx_start <= 1'b1;
            if (stream_index + 7'd1 >= tx_payload_length) begin
              stream_index <= 7'd0;
              stream_state <= STREAM_SUFFIX;
            end else begin
              stream_index <= stream_index + 7'd1;
            end
          end
        end
        STREAM_SUFFIX: begin
          if (!byte_tx_busy && !byte_tx_start) begin
            byte_tx_data <= suffix_byte(stream_index[3:0]);
            byte_tx_start <= 1'b1;
            if (stream_index == 7'd8) begin
              stream_index <= 7'd0;
              stream_saw_busy <= 1'b0;
              stream_state <= STREAM_WAIT;
            end else begin
              stream_index <= stream_index + 7'd1;
            end
          end
        end
        default: begin
          if (byte_tx_busy)
            stream_saw_busy <= 1'b1;
          else if (stream_saw_busy) begin
            payload_sent <= 1'b1;
            tx_packet_count <= tx_packet_count + 16'd1;
            tx_gap_count <= TX_GAP_CLKS;
            stream_state <= STREAM_IDLE;
            stream_saw_busy <= 1'b0;
          end
        end
      endcase
    end
  end
  //----Code ends here: integrated by Robei-----

endmodule

module uwb_debug_uart(
input wire clk,
input wire rst_n,
input wire [15:0] rx_packet_count,
input wire [15:0] rx_crc_error_count,
input wire [15:0] tx_packet_count,
input wire uwb_link_online,
input wire debug_last_ack_valid,
input wire [7:0] debug_last_sequence,
input wire [1:0] debug_last_result,
output reg uart_tx
);

  //----Code starts here: integrated by Robei-----
`ifdef UWB_DEBUG_SIM_FAST
  localparam integer REPORT_PERIOD_CLKS = 50;
  localparam integer CLKS_PER_BIT = 4;
`else
  localparam integer REPORT_PERIOD_CLKS = 25000000;
  localparam integer CLKS_PER_BIT = 434;
`endif
  localparam [5:0] LINE_LENGTH = 6'd26;

  function [7:0] hex_ascii;
    input [3:0] value;
    begin
      if (value < 4'd10)
        hex_ascii = 8'h30 + value;
      else
        hex_ascii = 8'h41 + value - 4'd10;
    end
  endfunction

  function [7:0] report_byte;
    input [5:0] index;
    input [15:0] rx_packets;
    input [15:0] crc_errors;
    input [15:0] tx_packets;
    input link_online;
    input ack_valid;
    input [7:0] ack_sequence;
    input [1:0] ack_result;
    begin
      case (index)
        6'd0:  report_byte = "@";
        6'd1:  report_byte = "U";
        6'd2:  report_byte = hex_ascii(rx_packets[15:12]);
        6'd3:  report_byte = hex_ascii(rx_packets[11:8]);
        6'd4:  report_byte = hex_ascii(rx_packets[7:4]);
        6'd5:  report_byte = hex_ascii(rx_packets[3:0]);
        6'd6:  report_byte = ",";
        6'd7:  report_byte = hex_ascii(crc_errors[15:12]);
        6'd8:  report_byte = hex_ascii(crc_errors[11:8]);
        6'd9:  report_byte = hex_ascii(crc_errors[7:4]);
        6'd10: report_byte = hex_ascii(crc_errors[3:0]);
        6'd11: report_byte = ",";
        6'd12: report_byte = hex_ascii(tx_packets[15:12]);
        6'd13: report_byte = hex_ascii(tx_packets[11:8]);
        6'd14: report_byte = hex_ascii(tx_packets[7:4]);
        6'd15: report_byte = hex_ascii(tx_packets[3:0]);
        6'd16: report_byte = ",";
        6'd17: report_byte = link_online ? "1" : "0";
        6'd18: report_byte = ",";
        6'd19: report_byte = ack_valid ? "1" : "0";
        6'd20: report_byte = ",";
        6'd21: report_byte = hex_ascii(ack_sequence[7:4]);
        6'd22: report_byte = hex_ascii(ack_sequence[3:0]);
        6'd23: report_byte = ",";
        6'd24: report_byte = "0" + ack_result;
        default: report_byte = 8'h0A;
      endcase
    end
  endfunction

  // Snapshot all counters at the beginning of a line so one UART frame never
  // mixes values captured at different times.
  reg [15:0] snapshot_rx_packets;
  reg [15:0] snapshot_crc_errors;
  reg [15:0] snapshot_tx_packets;
  reg snapshot_link_online;
  reg snapshot_ack_valid;
  reg [7:0] snapshot_ack_sequence;
  reg [1:0] snapshot_ack_result;

  reg [24:0] report_clock_count;
  reg report_sending;
  reg [5:0] report_char_index;
  reg [7:0] tx_data;
  reg tx_start;

  reg tx_busy;
  reg [8:0] tx_clock_count;
  reg [3:0] tx_bit_index;
  reg [7:0] tx_shift;

  // Independent 115200 8N1 byte transmitter. It only drives the board's USB
  // debug UART pin and cannot contend with the LinkPG or Bluetooth UARTs.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uart_tx <= 1'b1;
      tx_busy <= 1'b0;
      tx_clock_count <= 9'd0;
      tx_bit_index <= 4'd0;
      tx_shift <= 8'd0;
    end else if (!tx_busy) begin
      uart_tx <= 1'b1;
      tx_clock_count <= 9'd0;
      tx_bit_index <= 4'd0;
      if (tx_start) begin
        uart_tx <= 1'b0;
        tx_busy <= 1'b1;
        tx_shift <= tx_data;
      end
    end else if (tx_clock_count >= CLKS_PER_BIT - 1) begin
      tx_clock_count <= 9'd0;
      if (tx_bit_index < 4'd8) begin
        uart_tx <= tx_shift[tx_bit_index];
        tx_bit_index <= tx_bit_index + 4'd1;
      end else if (tx_bit_index == 4'd8) begin
        uart_tx <= 1'b1;
        tx_bit_index <= 4'd9;
      end else begin
        uart_tx <= 1'b1;
        tx_busy <= 1'b0;
      end
    end else begin
      tx_clock_count <= tx_clock_count + 9'd1;
    end
  end

  // Emit one fixed-width diagnostic line every 500 ms. Hex fields avoid the
  // large divider networks that decimal formatting would add during synthesis.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      snapshot_rx_packets <= 16'd0;
      snapshot_crc_errors <= 16'd0;
      snapshot_tx_packets <= 16'd0;
      snapshot_link_online <= 1'b0;
      snapshot_ack_valid <= 1'b0;
      snapshot_ack_sequence <= 8'd0;
      snapshot_ack_result <= 2'd0;
      report_clock_count <= 25'd0;
      report_sending <= 1'b0;
      report_char_index <= 6'd0;
      tx_data <= 8'd0;
      tx_start <= 1'b0;
    end else begin
      tx_start <= 1'b0;

      if (report_clock_count >= REPORT_PERIOD_CLKS - 1) begin
        if (!report_sending) begin
          snapshot_rx_packets <= rx_packet_count;
          snapshot_crc_errors <= rx_crc_error_count;
          snapshot_tx_packets <= tx_packet_count;
          snapshot_link_online <= uwb_link_online;
          snapshot_ack_valid <= debug_last_ack_valid;
          snapshot_ack_sequence <= debug_last_sequence;
          snapshot_ack_result <= debug_last_result;
          report_clock_count <= 25'd0;
          report_sending <= 1'b1;
          report_char_index <= 6'd0;
        end
      end else begin
        report_clock_count <= report_clock_count + 25'd1;
      end

      if (report_sending && !tx_busy && !tx_start) begin
        if (report_char_index < LINE_LENGTH) begin
          tx_data <= report_byte(
              report_char_index,
              snapshot_rx_packets,
              snapshot_crc_errors,
              snapshot_tx_packets,
              snapshot_link_online,
              snapshot_ack_valid,
              snapshot_ack_sequence,
              snapshot_ack_result);
          tx_start <= 1'b1;
          report_char_index <= report_char_index + 6'd1;
        end else begin
          report_sending <= 1'b0;
        end
      end
    end
  end
  //----Code ends here: integrated by Robei-----
endmodule

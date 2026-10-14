module uwb_uart_scope_probe_top (
  input  wire clk,
  input  wire uwb_n14,
  output wire uwb_n13,
  output wire usb_uart_tx
);
  // The board clock is 50 MHz. Rounding 50 MHz / 115200 gives 434 clocks/bit.
  localparam integer CLKS_PER_BIT = 434;
  localparam integer QUERY_INTERVAL_CLKS = 25000000;
  localparam integer QUERY_LENGTH = 8;

  // Standard Modbus-RTU read: slave 1, function 03, registers 0..5.
  // CRC16 is transmitted low byte first: C5 C8.
  function [7:0] query_byte;
    input [2:0] index;
    begin
      case (index)
        3'd0: query_byte = 8'h01;
        3'd1: query_byte = 8'h03;
        3'd2: query_byte = 8'h00;
        3'd3: query_byte = 8'h00;
        3'd4: query_byte = 8'h00;
        3'd5: query_byte = 8'h06;
        3'd6: query_byte = 8'hC5;
        default: query_byte = 8'hC8;
      endcase
    end
  endfunction

  reg [7:0] tx_data = 8'd0;
  reg tx_start = 1'b0;
  reg tx_busy = 1'b0;
  reg [8:0] baud_count = 9'd0;
  reg [3:0] tx_phase = 4'd0;
  reg [7:0] tx_shift = 8'd0;
  reg tx_line = 1'b1;

  reg [24:0] interval_count = 25'd0;
  reg sending_query = 1'b0;
  reg [2:0] byte_index = 3'd0;

  assign uwb_n13 = tx_line;

  // Echo each outgoing query to COM20. Between queries, pass the LinkPG TXD
  // input through unchanged, so one capture contains both request and reply.
  assign usb_uart_tx = tx_busy ? tx_line : uwb_n14;

  always @(posedge clk) begin
    if (!tx_busy) begin
      tx_line <= 1'b1;
      baud_count <= 9'd0;
      tx_phase <= 4'd0;
      if (tx_start) begin
        tx_busy <= 1'b1;
        tx_shift <= tx_data;
        tx_line <= 1'b0;
      end
    end else if (baud_count >= CLKS_PER_BIT - 1) begin
      baud_count <= 9'd0;
      if (tx_phase < 4'd8) begin
        tx_line <= tx_shift[tx_phase];
        tx_phase <= tx_phase + 4'd1;
      end else if (tx_phase == 4'd8) begin
        tx_line <= 1'b1;
        tx_phase <= 4'd9;
      end else begin
        tx_busy <= 1'b0;
        tx_line <= 1'b1;
      end
    end else begin
      baud_count <= baud_count + 9'd1;
    end
  end

  always @(posedge clk) begin
    tx_start <= 1'b0;
    if (!sending_query) begin
      if (interval_count >= QUERY_INTERVAL_CLKS - 1) begin
        interval_count <= 25'd0;
        byte_index <= 3'd0;
        sending_query <= 1'b1;
      end else begin
        interval_count <= interval_count + 25'd1;
      end
    end else if (!tx_busy && !tx_start) begin
      tx_data <= query_byte(byte_index);
      tx_start <= 1'b1;
      if (byte_index + 3'd1 >= QUERY_LENGTH) begin
        sending_query <= 1'b0;
      end else begin
        byte_index <= byte_index + 3'd1;
      end
    end
  end
endmodule

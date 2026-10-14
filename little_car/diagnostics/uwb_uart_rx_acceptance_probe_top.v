module uwb_uart_rx_acceptance_probe_top (
  input  wire clk,
  input  wire uwb_n14,
  output wire uwb_n13,
  output wire usb_uart_tx
);

  localparam integer CLKS_PER_BIT = 434;
  localparam integer FIRST_GAP_CLKS = 150000000;
  localparam integer NEXT_GAP_CLKS = 50000000;

  // Sequence: read mode, stop, read twice, restart mode 4, then read again.
  function [3:0] message_length;
    input [2:0] stage;
    begin
      message_length = (stage == 3'd1 || stage == 3'd4) ? 4'd11 : 4'd8;
    end
  endfunction

  function [7:0] message_byte;
    input [2:0] stage;
    input [2:0] index;
    begin
      if (stage == 3'd1) begin
        case (index)
          3'd0: message_byte = 8'h01;
          3'd1: message_byte = 8'h10;
          3'd2: message_byte = 8'h00;
          3'd3: message_byte = 8'h3B;
          3'd4: message_byte = 8'h00;
          3'd5: message_byte = 8'h01;
          3'd6: message_byte = 8'h02;
          default: message_byte = 8'h00;
        endcase
      end else if (stage == 3'd4) begin
        case (index)
          3'd0: message_byte = 8'h01;
          3'd1: message_byte = 8'h10;
          3'd2: message_byte = 8'h00;
          3'd3: message_byte = 8'h3B;
          3'd4: message_byte = 8'h00;
          3'd5: message_byte = 8'h01;
          3'd6: message_byte = 8'h02;
          default: message_byte = 8'h00;
        endcase
      end else begin
        case (index)
          3'd0: message_byte = 8'h01;
          3'd1: message_byte = 8'h03;
          3'd2: message_byte = 8'h00;
          3'd3: message_byte = 8'h3B;
          3'd4: message_byte = 8'h00;
          3'd5: message_byte = 8'h01;
          3'd6: message_byte = 8'hF5;
          default: message_byte = 8'hC7;
        endcase
      end
    end
  endfunction

  reg [7:0] tx_data = 8'd0;
  reg tx_start = 1'b0;
  reg tx_busy = 1'b0;
  reg [8:0] baud_count = 9'd0;
  reg [3:0] tx_phase = 4'd0;
  reg [7:0] tx_shift = 8'd0;
  reg tx_line = 1'b1;

  reg [27:0] gap_count = 28'd0;
  reg [2:0] stage = 3'd0;
  reg sending_message = 1'b0;
  reg [3:0] byte_index = 4'd0;

  assign uwb_n13 = tx_line;
  // COM20 sees only the tag's TXD, so positioning gaps and Modbus replies are
  // unambiguous even if a request overlaps a normal positioning frame.
  assign usb_uart_tx = uwb_n14;

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
    if (!sending_message) begin
      if (stage < 3'd6) begin
        if (gap_count >= ((stage == 3'd0) ? FIRST_GAP_CLKS - 1 : NEXT_GAP_CLKS - 1)) begin
          gap_count <= 28'd0;
          byte_index <= 4'd0;
          sending_message <= 1'b1;
        end else begin
          gap_count <= gap_count + 28'd1;
        end
      end
    end else if (!tx_busy && !tx_start) begin
      if ((stage == 3'd1 || stage == 3'd4) && byte_index >= 4'd7) begin
        case (byte_index)
          4'd7: tx_data <= 8'h00;
          4'd8: tx_data <= (stage == 3'd4) ? 8'h04 : 8'h00;
          4'd9: tx_data <= (stage == 3'd4) ? 8'hA3 : 8'hA2;
          default: tx_data <= (stage == 3'd4) ? 8'h18 : 8'hDB;
        endcase
      end else begin
        tx_data <= message_byte(stage, byte_index[2:0]);
      end
      tx_start <= 1'b1;
      if (byte_index + 4'd1 >= message_length(stage)) begin
        sending_message <= 1'b0;
        stage <= stage + 3'd1;
      end else begin
        byte_index <= byte_index + 4'd1;
      end
    end
  end

endmodule

module uwb_uart_modbus_probe_top (
  input  wire clk,
  input  wire uwb_n14,
  output wire uwb_n13,
  output wire usb_uart_tx
);
  localparam integer CLKS_PER_BIT = 434;
  localparam integer SEND_DELAY_CLKS = 50000000;
  localparam integer MESSAGE_LENGTH = 8;

  function [7:0] message_byte;
    input [2:0] index;
    begin
      case (index)
        3'd0: message_byte = 8'h01;
        3'd1: message_byte = 8'h03;
        3'd2: message_byte = 8'h00;
        3'd3: message_byte = 8'h00;
        3'd4: message_byte = 8'h00;
        3'd5: message_byte = 8'h06;
        3'd6: message_byte = 8'hC5;
        default: message_byte = 8'hC8;
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

  reg [25:0] delay_count = 26'd0;
  reg [1:0] repeat_count = 2'd0;
  reg sending_message = 1'b0;
  reg [2:0] byte_index = 3'd0;

  assign uwb_n13 = tx_line;
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
      if (repeat_count < 3) begin
        if (delay_count >= SEND_DELAY_CLKS - 1) begin
          delay_count <= 26'd0;
          byte_index <= 3'd0;
          sending_message <= 1'b1;
        end else begin
          delay_count <= delay_count + 26'd1;
        end
      end
    end else if (!tx_busy && !tx_start) begin
      tx_data <= message_byte(byte_index);
      tx_start <= 1'b1;
      if (byte_index + 3'd1 >= MESSAGE_LENGTH) begin
        sending_message <= 1'b0;
        repeat_count <= repeat_count + 2'd1;
      end else begin
        byte_index <= byte_index + 3'd1;
      end
    end
  end
endmodule

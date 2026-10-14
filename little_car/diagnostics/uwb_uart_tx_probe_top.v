module uwb_uart_tx_probe_top (
  input  wire clk,
  output wire uwb_n13,
  output wire usb_uart_tx
);
  localparam integer CLKS_PER_BIT = 434;
  localparam integer SEND_DELAY_CLKS = 50000000;
  localparam integer MESSAGE_LENGTH = 30;

  function [7:0] message_byte;
    input [5:0] index;
    begin
      case (index)
        6'd0:  message_byte = "A";
        6'd1:  message_byte = "T";
        6'd2:  message_byte = "+";
        6'd3:  message_byte = "D";
        6'd4:  message_byte = "a";
        6'd5:  message_byte = "t";
        6'd6:  message_byte = "a";
        6'd7:  message_byte = "S";
        6'd8:  message_byte = "e";
        6'd9:  message_byte = "n";
        6'd10: message_byte = "d";
        6'd11: message_byte = "=";
        6'd12: message_byte = 8'h22;
        6'd13: message_byte = "A";
        6'd14: message_byte = "1";
        6'd15: message_byte = "0";
        6'd16: message_byte = "8";
        6'd17: message_byte = "P";
        6'd18: message_byte = "0";
        6'd19: message_byte = "E";
        6'd20: message_byte = "E";
        6'd21: message_byte = 8'h22;
        6'd22: message_byte = ",";
        6'd23: message_byte = 8'h22;
        6'd24: message_byte = "2";
        6'd25: message_byte = "5";
        6'd26: message_byte = "5";
        6'd27: message_byte = 8'h22;
        6'd28: message_byte = 8'h0D;
        default: message_byte = 8'h0A;
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
  reg [5:0] byte_index = 6'd0;

  assign uwb_n13 = tx_line;
  assign usb_uart_tx = tx_line;

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
          byte_index <= 6'd0;
          sending_message <= 1'b1;
        end else begin
          delay_count <= delay_count + 26'd1;
        end
      end
    end else if (!tx_busy && !tx_start) begin
      tx_data <= message_byte(byte_index);
      tx_start <= 1'b1;
      if (byte_index + 6'd1 >= MESSAGE_LENGTH) begin
        sending_message <= 1'b0;
        repeat_count <= repeat_count + 2'd1;
      end else begin
        byte_index <= byte_index + 6'd1;
      end
    end
  end
endmodule

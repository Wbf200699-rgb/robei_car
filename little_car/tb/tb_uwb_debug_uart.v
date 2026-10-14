`timescale 1ns/1ps

module tb_uwb_debug_uart;
  localparam integer CLKS_PER_BIT = 4;
  localparam integer LINE_LENGTH = 26;

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg [15:0] rx_packet_count = 16'h1234;
  reg [15:0] rx_crc_error_count = 16'hABCD;
  reg [15:0] tx_packet_count = 16'h00EF;
  reg uwb_link_online = 1'b1;
  reg debug_last_ack_valid = 1'b1;
  reg [7:0] debug_last_sequence = 8'hB2;
  reg [1:0] debug_last_result = 2'd0;
  wire uart_tx;

  integer index;
  reg [7:0] received_byte;

  always #10 clk = ~clk;

  uwb_debug_uart dut (
    .clk(clk),
    .rst_n(rst_n),
    .rx_packet_count(rx_packet_count),
    .rx_crc_error_count(rx_crc_error_count),
    .tx_packet_count(tx_packet_count),
    .uwb_link_online(uwb_link_online),
    .debug_last_ack_valid(debug_last_ack_valid),
    .debug_last_sequence(debug_last_sequence),
    .debug_last_result(debug_last_result),
    .uart_tx(uart_tx)
  );

  function [7:0] expected_byte;
    input integer byte_index;
    begin
      case (byte_index)
        0: expected_byte = "@";
        1: expected_byte = "U";
        2: expected_byte = "1";
        3: expected_byte = "2";
        4: expected_byte = "3";
        5: expected_byte = "4";
        6: expected_byte = ",";
        7: expected_byte = "A";
        8: expected_byte = "B";
        9: expected_byte = "C";
        10: expected_byte = "D";
        11: expected_byte = ",";
        12: expected_byte = "0";
        13: expected_byte = "0";
        14: expected_byte = "E";
        15: expected_byte = "F";
        16: expected_byte = ",";
        17: expected_byte = "1";
        18: expected_byte = ",";
        19: expected_byte = "1";
        20: expected_byte = ",";
        21: expected_byte = "B";
        22: expected_byte = "2";
        23: expected_byte = ",";
        24: expected_byte = "0";
        default: expected_byte = 8'h0A;
      endcase
    end
  endfunction

  task receive_uart_byte;
    output [7:0] value;
    integer bit_index;
    begin
      @(negedge uart_tx);
      repeat (CLKS_PER_BIT + (CLKS_PER_BIT / 2)) @(posedge clk);
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        value[bit_index] = uart_tx;
        repeat (CLKS_PER_BIT) @(posedge clk);
      end
      repeat (CLKS_PER_BIT) @(posedge clk);
    end
  endtask

  initial begin
    repeat (5) @(posedge clk);
    rst_n = 1'b1;

    for (index = 0; index < LINE_LENGTH; index = index + 1) begin
      receive_uart_byte(received_byte);
      if (received_byte !== expected_byte(index)) begin
        $display("RESULT TEST_FAIL uwb_debug index=%0d expected=%02x got=%02x",
                 index, expected_byte(index), received_byte);
        $finish;
      end
      $write("%c", received_byte);
    end

    $display("RESULT TEST_PASS uwb_debug_uart");
    $finish;
  end

  initial begin
    #100000;
    $display("RESULT TEST_FAIL uwb_debug timeout");
    $finish;
  end
endmodule

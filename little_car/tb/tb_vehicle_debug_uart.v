`timescale 1ns/1ps
`define VEHICLE_DEBUG_SIM_FAST

module tb_vehicle_debug_uart;
  localparam integer CLKS_PER_BIT = 4;
  localparam integer FRAME_LENGTH = 22;

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  wire debug_uart_tx;
  integer index;
  reg [7:0] received_byte;

  always #10 clk = ~clk;

  vehicle_debug_uart dut (
    .clk(clk),
    .rst_n(rst_n),
    .bt_connected(1'b1),
    .lidar_enable(1'b1),
    .lidar_scan_active(1'b1),
    .lidar_online(1'b0),
    .uwb_position_online(1'b1),
    .uwb_position_good(1'b0),
    .uwb_start_ack_seen(1'b1),
    .uwb_position_x_cm(-16'sd2),
    .uwb_position_y_cm(16'h0123),
    .uwb_packet_count(16'h4567),
    .uwb_crc_error_count(16'h89ab),
    .uwb_start_request_count(16'hcdef),
    .map_pose_x_mm(16'h1357),
    .map_pose_y_mm(16'h2468),
    .map_pose_heading(10'h2aa),
    .flow_correction(11'h12a),
    .yaw_correction(11'h7e1),
    .motion_telemetry_data({312'd0, 8'ha5}),
    .debug_uart_tx(debug_uart_tx)
  );

  function [7:0] expected_byte;
    input integer byte_index;
    begin
      case (byte_index)
        0: expected_byte = 8'ha5;
        1: expected_byte = 8'h5a;
        2: expected_byte = 8'h02;
        3: expected_byte = 8'heb;
        4: expected_byte = 8'hfe;
        5: expected_byte = 8'hff;
        6: expected_byte = 8'h23;
        7: expected_byte = 8'h01;
        8: expected_byte = 8'h67;
        9: expected_byte = 8'h45;
        10: expected_byte = 8'hab;
        11: expected_byte = 8'h89;
        12: expected_byte = 8'hef;
        13: expected_byte = 8'hcd;
        14: expected_byte = 8'h57;
        15: expected_byte = 8'h13;
        16: expected_byte = 8'h68;
        17: expected_byte = 8'h24;
        18: expected_byte = 8'haa;
        19: expected_byte = 8'h02;
        20: expected_byte = 8'ha5;
        default: expected_byte = 8'h0a;
      endcase
    end
  endfunction

  task receive_uart_byte;
    output [7:0] value;
    integer bit_index;
    begin
      @(negedge debug_uart_tx);
      repeat (CLKS_PER_BIT + (CLKS_PER_BIT / 2)) @(posedge clk);
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        value[bit_index] = debug_uart_tx;
        repeat (CLKS_PER_BIT) @(posedge clk);
      end
      repeat (CLKS_PER_BIT) @(posedge clk);
    end
  endtask

  initial begin
    repeat (5) @(posedge clk);
    rst_n = 1'b1;

    for (index = 0; index < FRAME_LENGTH; index = index + 1) begin
      receive_uart_byte(received_byte);
      if (received_byte !== expected_byte(index))
        $fatal(1, "debug frame byte %0d expected=%02x got=%02x",
               index, expected_byte(index), received_byte);
    end

    $display("RESULT frame=A55A02EBFEFF23016745AB89EFCD57136824AA02A50A");
    $display("RESULT TEST_PASS vehicle_debug_uart");
    $finish;
  end

  initial begin
    #200000;
    $fatal(1, "vehicle debug UART timeout");
  end
endmodule

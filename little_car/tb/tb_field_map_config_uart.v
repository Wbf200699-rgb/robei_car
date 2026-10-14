`timescale 1ns/1ps

module tb_field_map_config_uart;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg rx = 1'b1;
  wire config_valid;
  wire config_update_pulse;
  wire [7:0] pose_x_cell;
  wire [7:0] pose_y_cell;
  wire [9:0] heading_1024;
  wire [7:0] low_x_cell;
  wire [7:0] high_x_cell;
  wire [7:0] low_y_cell;
  wire [7:0] high_y_cell;
  integer failures = 0;

  always #5 clk = ~clk;

  field_map_config_uart #(.CLK_HZ(1000000), .BAUD(100000)) dut (
    .clk(clk), .rst_n(rst_n), .bt_uart_rx(rx),
    .config_valid(config_valid), .config_update_pulse(config_update_pulse),
    .pose_x_cell(pose_x_cell), .pose_y_cell(pose_y_cell),
    .heading_1024(heading_1024),
    .low_x_cell(low_x_cell), .high_x_cell(high_x_cell),
    .low_y_cell(low_y_cell), .high_y_cell(high_y_cell)
  );

  task uart_byte;
    input [7:0] value;
    integer bit_index;
    begin
      rx = 1'b0; repeat (10) @(posedge clk);
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        rx = value[bit_index]; repeat (10) @(posedge clk);
      end
      rx = 1'b1; repeat (10) @(posedge clk);
    end
  endtask

  task send_valid_frame;
    begin
      // {2B 33 2AA 15 45 16 48} using !#$%&'()*,./:;<>
      uart_byte("{");
      uart_byte("$"); uart_byte("/");
      uart_byte("%"); uart_byte("%");
      uart_byte("$"); uart_byte("."); uart_byte(".");
      uart_byte("#"); uart_byte("'");
      uart_byte("&"); uart_byte("'");
      uart_byte("#"); uart_byte("(");
      uart_byte("&"); uart_byte("*");
      uart_byte("}");
    end
  endtask

  initial begin
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    send_valid_frame();
    repeat (20) @(posedge clk);
    if (!config_valid || pose_x_cell != 8'h2b || pose_y_cell != 8'h33 ||
        heading_1024 != 10'h2aa || low_x_cell != 8'h15 ||
        high_x_cell != 8'h45 || low_y_cell != 8'h16 || high_y_cell != 8'h48) begin
      $display("ERROR decoded config x=%h y=%h h=%h bounds=%h,%h,%h,%h",
               pose_x_cell, pose_y_cell, heading_1024,
               low_x_cell, high_x_cell, low_y_cell, high_y_cell);
      failures = failures + 1;
    end
    uart_byte("{"); uart_byte("!"); uart_byte("X"); uart_byte("}");
    repeat (20) @(posedge clk);
    if (pose_x_cell != 8'h2b || heading_1024 != 10'h2aa) begin
      $display("ERROR malformed frame replaced last valid configuration");
      failures = failures + 1;
    end
    if (failures == 0) $display("RESULT TEST_PASS field_map_config_uart");
    else $display("RESULT TEST_FAIL field_map_config_uart failures=%0d", failures);
    $finish;
  end
endmodule

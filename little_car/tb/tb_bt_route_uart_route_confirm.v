`timescale 1ns/1ps
`define BT_MAP_SIM_FAST

module tb_bt_route_uart_route_confirm;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg bt_uart_rx = 1'b1;
  wire manual_mode;
  wire [2:0] manual_cmd;
  wire bt_key_mid_n;
  wire motor_stop_pulse;

  always #10 clk = ~clk;

  bt_route_uart dut (
    .clk(clk),
    .rst_n(rst_n),
    .bt_uart_rx(bt_uart_rx),
    .manual_mode(manual_mode),
    .manual_cmd(manual_cmd),
    .bt_key_mid_n(bt_key_mid_n),
    .motor_stop_pulse(motor_stop_pulse)
  );

  task inject_command;
    input [7:0] command;
    begin
      @(negedge clk);
      force dut.rx_data = command;
      force dut.rx_valid = 1'b1;
      @(negedge clk);
      release dut.rx_valid;
      release dut.rx_data;
      @(posedge clk);
    end
  endtask

  initial begin
    repeat (8) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    wait (dut.bt_startup_ready);

    inject_command("O");
    if (manual_mode !== 1'b0)
      $fatal(1, "O did not enter automatic route mode");

    inject_command("C");
    if (manual_mode !== 1'b0)
      $fatal(1, "automatic route confirm changed to manual mode");
    if (bt_key_mid_n !== 1'b0)
      $fatal(1, "automatic route confirm did not assert the middle key");
    if (manual_cmd !== 3'd0)
      $fatal(1, "route confirm did not retain the stopped motor command");

    inject_command("H");
    if (manual_mode !== 1'b1)
      $fatal(1, "H did not enter manual mode");

    inject_command("C");
    if (manual_mode !== 1'b1)
      $fatal(1, "manual stop unexpectedly left manual mode");
    if (manual_cmd !== 3'd0)
      $fatal(1, "manual stop command was not retained");

    $display("RESULT auto_confirm_mode=0 middle_key=0 manual_stop_mode=1");
    $display("RESULT TEST_PASS bt_route_uart_route_confirm");
    $finish;
  end

  initial begin
    #10000000;
    $fatal(1, "timeout");
  end
endmodule

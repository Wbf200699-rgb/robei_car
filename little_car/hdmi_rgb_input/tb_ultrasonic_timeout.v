`timescale 1ns / 1ps

module tb_ultrasonic_timeout;

  reg clk;
  reg rst_n;
  reg enable;
  reg echo;
  reg [15:0] step_length;

  wire trig;
  wire [15:0] distance_cm;
  wire distance_valid;
  wire obstacle_stop;
  wire echo_timeout;

  ultrasonic_obstacle_guard dut (
    .clk(clk),
    .rst_n(rst_n),
    .enable(enable),
    .echo(echo),
    .step_length(step_length),
    .trig(trig),
    .distance_cm(distance_cm),
    .distance_valid(distance_valid),
    .obstacle_stop(obstacle_stop),
    .echo_timeout(echo_timeout)
  );

  initial begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  initial begin
    rst_n = 1'b0;
    enable = 1'b0;
    echo = 1'b0;
    step_length = 16'd20;

    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    enable = 1'b1;

    wait (distance_valid == 1'b1);
    @(posedge clk);

    if (echo_timeout !== 1'b1)
      $fatal(1, "ultrasonic timeout tb: expected echo_timeout");
    if (distance_cm !== 16'hFFFF)
      $fatal(1, "ultrasonic timeout tb: expected distance_cm=FFFF, got %h", distance_cm);
    if (obstacle_stop !== 1'b0)
      $fatal(1, "ultrasonic timeout tb: obstacle_stop must stay low on timeout");

    $display("[%0t] PASS tb_ultrasonic_timeout", $time);
    $finish;
  end

endmodule

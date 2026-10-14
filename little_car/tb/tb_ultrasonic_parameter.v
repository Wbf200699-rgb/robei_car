`timescale 1ns/1ps

module tb_ultrasonic_parameter;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg echo = 1'b0;
  reg [15:0] step_length = 16'd20;
  reg enable = 1'b1;
  wire trig;
  wire [15:0] distance_cm;
  wire distance_valid, obstacle_stop, echo_timeout;
  integer errors;
  integer trig_cycles;

  always #10 clk = ~clk;

  ultrasonic_obstacle_guard dut (
    .clk(clk), .rst_n(rst_n), .echo(echo), .step_length(step_length),
    .trig(trig), .distance_cm(distance_cm), .distance_valid(distance_valid),
    .obstacle_stop(obstacle_stop), .echo_timeout(echo_timeout), .enable(enable)
  );

  task measure_cm;
    input integer requested_cm;
    input expected_stop;
    integer sample_cycles;
    begin
      @(negedge clk);
      dut.state = dut.ST_MEASURE;
      dut.echo_cycles = 0;
      dut.cm_cycle_cnt = 0;
      dut.measured_cm = 0;
      dut.echo_d0 = 1;
      dut.echo_d1 = 1;
      dut.echo_d2 = 1;
      echo = 1;
      sample_cycles = requested_cm * 2900;
      repeat (sample_cycles) @(posedge clk);
      @(negedge clk);
      echo = 0;
      wait (distance_valid);
      #1;
      $display("RESULT echo_case requested_cm=%0d measured_cm=%0d pulse_cycles=%0d stop=%0d", requested_cm, distance_cm, sample_cycles, obstacle_stop);
      if (distance_cm < requested_cm - 1 || distance_cm > requested_cm + 1 ||
          obstacle_stop !== expected_stop || echo_timeout)
        errors = errors + 1;
      @(posedge clk);
    end
  endtask

  initial begin
    errors = 0;
    trig_cycles = 0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    wait (trig);
    while (trig) begin
      @(posedge clk);
      #1;
      trig_cycles = trig_cycles + 1;
    end
    if (trig_cycles < 499 || trig_cycles > 501)
      errors = errors + 1;

    measure_cm(5,  1'b1);
    measure_cm(19, 1'b1);
    measure_cm(20, 1'b0);
    measure_cm(35, 1'b0);

    @(negedge clk);
    echo = 0;
    dut.state = dut.ST_WAIT_ECHO;
    dut.cnt = 32'd1499999;
    @(posedge clk);
    #1;
    if (!distance_valid || !echo_timeout || distance_cm != 16'hffff || obstacle_stop)
      errors = errors + 1;
    $display("RESULT timeout_cycles=1500000 timeout_us=30000 timeout_distance=FFFF");
    $display("RESULT trigger_cycles=%0d trigger_us=10 period_cycles=3000000 period_ms=60 cycles_per_cm=2900", trig_cycles);
    if (errors == 0)
      $display("RESULT TEST_PASS ultrasonic_parameter");
    else
      $display("RESULT TEST_FAIL ultrasonic_parameter errors=%0d", errors);
    $finish;
  end

  initial begin
    #10000000;
    $display("RESULT TEST_FAIL ultrasonic_parameter timeout=1");
    $finish;
  end
endmodule

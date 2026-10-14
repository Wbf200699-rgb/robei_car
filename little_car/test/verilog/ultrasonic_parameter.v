`timescale 1ns/1ps
module ultrasonic_obstacle_guard(
  input wire clk,
  input wire rst_n,
  input wire echo,
  input wire [15:0] step_length,
  output reg trig,
  output reg [15:0] distance_cm,
  output reg distance_valid,
  output reg obstacle_stop,
  output reg echo_timeout,
  input wire enable
);
  localparam [2:0] ST_TRIG      = 3'd0;
  localparam [2:0] ST_WAIT_ECHO = 3'd1;
  localparam [2:0] ST_MEASURE   = 3'd2;
  localparam [2:0] ST_GAP       = 3'd3;

  localparam [31:0] TRIG_CYCLES     = 32'd500;      // 10 us at 50 MHz
  localparam [31:0] ECHO_TIMEOUT    = 32'd1500000;  // 30 ms at 50 MHz
  localparam [31:0] MEASURE_PERIOD  = 32'd3000000;  // 60 ms at 50 MHz
  localparam [31:0] CYCLES_PER_CM   = 32'd2900;     // 58 us/cm * 50 cycles/us
  localparam [15:0] DEFAULT_STEP_CM = 16'd20;

  reg [2:0] state;
  reg [31:0] cnt;
  reg [31:0] echo_cycles;
  reg [31:0] cm_cycle_cnt;
  reg [15:0] measured_cm;
  reg echo_d0;
  reg echo_d1;
  reg echo_d2;
  reg [15:0] compare_length;

  wire echo_sync;
  wire echo_rise;
  wire echo_fall;

  assign echo_sync = echo_d1;
  assign echo_rise = echo_d1 & ~echo_d2;
  assign echo_fall = ~echo_d1 & echo_d2;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      echo_d0 <= 1'b0;
      echo_d1 <= 1'b0;
      echo_d2 <= 1'b0;
    end else begin
      echo_d0 <= echo;
      echo_d1 <= echo_d0;
      echo_d2 <= echo_d1;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= ST_TRIG;
      cnt <= 32'd0;
      echo_cycles <= 32'd0;
      cm_cycle_cnt <= 32'd0;
      measured_cm <= 16'd0;
      trig <= 1'b0;
      distance_cm <= 16'd0;
      distance_valid <= 1'b0;
      obstacle_stop <= 1'b0;
      echo_timeout <= 1'b0;
      compare_length <= DEFAULT_STEP_CM;
    end else begin
      distance_valid <= 1'b0;
      obstacle_stop <= 1'b0;

      if (step_length == 16'd0)
        compare_length <= DEFAULT_STEP_CM;
      else
        compare_length <= step_length;

      if (!enable) begin
        state <= ST_TRIG;
        cnt <= 32'd0;
        echo_cycles <= 32'd0;
        cm_cycle_cnt <= 32'd0;
        measured_cm <= 16'd0;
        trig <= 1'b0;
        obstacle_stop <= 1'b0;
        echo_timeout <= 1'b0;
      end else begin
        case (state)
        ST_TRIG: begin
          trig <= 1'b1;
          echo_timeout <= 1'b0;
          echo_cycles <= 32'd0;
          cm_cycle_cnt <= 32'd0;
          measured_cm <= 16'd0;

          if (cnt >= TRIG_CYCLES - 32'd1) begin
            cnt <= 32'd0;
            trig <= 1'b0;
            state <= ST_WAIT_ECHO;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        ST_WAIT_ECHO: begin
          trig <= 1'b0;

          if (echo_rise || echo_sync) begin
            cnt <= 32'd0;
            echo_cycles <= 32'd0;
            cm_cycle_cnt <= 32'd0;
            measured_cm <= 16'd0;
            state <= ST_MEASURE;
          end else if (cnt >= ECHO_TIMEOUT - 32'd1) begin
            cnt <= 32'd0;
            echo_timeout <= 1'b1;
            distance_cm <= 16'hFFFF;
            distance_valid <= 1'b1;
            obstacle_stop <= 1'b0;
            state <= ST_GAP;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        ST_MEASURE: begin
          trig <= 1'b0;

          if (echo_fall) begin
            if (measured_cm == 16'd0 && echo_cycles != 32'd0)
              distance_cm <= 16'd1;
            else
              distance_cm <= measured_cm;

            distance_valid <= 1'b1;
            echo_timeout <= 1'b0;

            if (((measured_cm == 16'd0 && echo_cycles != 32'd0) ? 16'd1 : measured_cm) < compare_length)
              obstacle_stop <= 1'b1;

            cnt <= 32'd0;
            state <= ST_GAP;
          end else if (echo_cycles >= ECHO_TIMEOUT - 32'd1) begin
            cnt <= 32'd0;
            echo_cycles <= 32'd0;
            cm_cycle_cnt <= 32'd0;
            measured_cm <= 16'd0;
            echo_timeout <= 1'b1;
            distance_cm <= 16'hFFFF;
            distance_valid <= 1'b1;
            obstacle_stop <= 1'b0;
            state <= ST_GAP;
          end else begin
            echo_cycles <= echo_cycles + 32'd1;
            if (cm_cycle_cnt >= CYCLES_PER_CM - 32'd1) begin
              cm_cycle_cnt <= 32'd0;
              if (measured_cm != 16'hFFFF)
                measured_cm <= measured_cm + 16'd1;
            end else begin
              cm_cycle_cnt <= cm_cycle_cnt + 32'd1;
            end
          end
        end

        ST_GAP: begin
          trig <= 1'b0;

          if (cnt >= MEASURE_PERIOD - TRIG_CYCLES - 32'd1) begin
            cnt <= 32'd0;
            state <= ST_TRIG;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        default: begin
          state <= ST_TRIG;
          cnt <= 32'd0;
          trig <= 1'b0;
        end
        endcase
      end
    end
  end

endmodule

module ultrasonic_parameter;
  reg clk;
  reg rst_n;
  reg echo;
  reg [15:0] step_length;
  wire trig;
  wire [15:0] distance_cm;
  wire distance_valid;
  wire obstacle_stop;
  wire echo_timeout;
  reg enable;


`ifdef PARAM_WAVE
initial begin
  $dumpfile("tb_ultrasonic_parameter.vcd");
  $dumpvars(0, rst_n, echo, trig, distance_cm, distance_valid, obstacle_stop, echo_timeout, ultrasonic_obstacle_guard0.state);
end
`endif

initial begin
  clk = 1'b0;
  rst_n = 1'b0;
  echo = 1'b0;
  step_length = 16'd20;
  enable = 1'b1;
end

integer errors;
  integer trig_cycles;

  always #10 clk = ~clk;



  task measure_cm;
    input integer requested_cm;
    input expected_stop;
    integer sample_cycles;
    begin
      @(negedge clk);
      ultrasonic_obstacle_guard0.state = ultrasonic_obstacle_guard0.ST_MEASURE;
      ultrasonic_obstacle_guard0.echo_cycles = 0;
      ultrasonic_obstacle_guard0.cm_cycle_cnt = 0;
      ultrasonic_obstacle_guard0.measured_cm = 0;
      ultrasonic_obstacle_guard0.echo_d0 = 1;
      ultrasonic_obstacle_guard0.echo_d1 = 1;
      ultrasonic_obstacle_guard0.echo_d2 = 1;
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
    ultrasonic_obstacle_guard0.state = ultrasonic_obstacle_guard0.ST_WAIT_ECHO;
    ultrasonic_obstacle_guard0.cnt = 32'd1499999;
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


  ultrasonic_obstacle_guard ultrasonic_obstacle_guard0(
    .clk(clk),
    .rst_n(rst_n),
    .echo(echo),
    .step_length(step_length),
    .trig(trig),
    .distance_cm(distance_cm),
    .distance_valid(distance_valid),
    .obstacle_stop(obstacle_stop),
    .echo_timeout(echo_timeout),
    .enable(enable)
  );
endmodule

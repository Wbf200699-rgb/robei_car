module ultrasonic_obstacle_guard_test();
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

  //----Code starts here: integrated by Robei-----
  initial
  begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  task send_echo_cm;
    input integer cm;
    integer high_time_ns;
    begin
      // 58us/cm, convert to ns
      high_time_ns = cm * 58000;
      @(posedge trig);
      #20000;
      echo = 1'b1;
      #(high_time_ns);
      echo = 1'b0;
    end
  endtask

  initial
  begin
    rst_n = 1'b0;
    echo = 1'b0;
    step_length = 16'd20;
    enable = 1'b0;

    #200;
    rst_n = 1'b1;
    #200;
    enable = 1'b1;

    // case 1: obstacle within step_length -> obstacle_stop should pulse
    fork
      send_echo_cm(10);
    join

    #70000000;

    // case 2: far target -> valid distance, no obstacle_stop
    fork
      send_echo_cm(35);
    join

    #70000000;

    // case 3: no echo -> timeout
    #70000000;

    $finish;
  end

  initial begin
    $dumpfile("tb/tb_ultrasonic_obstacle_guard_robei.vcd");
    $dumpvars(0, ultrasonic_obstacle_guard_test);
  end
//---Module instantiation---
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

endmodule    //ultrasonic_obstacle_guard_test

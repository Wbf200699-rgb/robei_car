module route_contol_core_test();
reg clk;
reg rst_n;
reg key_up_pulse;
reg key_down_pulse;
reg key_left_pulse;
reg key_right_pulse;
reg key_mid_pulse;
reg run_step_pulse;
reg obstacle_stop;
reg [15:0] step_length_set;
reg [15:0] speed_set;
wire [1:0] menu_info;
wire [63:0] route_info;
wire [4:0] state_info;
wire [63:0] old_route_info;
wire [5:0] active_route_steps;
wire [5:0] completed_route_steps;
wire [15:0] step_length;
wire [15:0] speed;
wire [15:0] total_route_length;
wire [15:0] complete_route_length;
wire [15:0] remaining_route_length;
wire [7:0] now_x;
wire [7:0] now_y;
wire [7:0] target_x;
wire [7:0] target_y;

  //----Code starts here: integrated by Robei-----
  initial
  begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  task pulse_mid;
    begin
      key_mid_pulse = 1'b1;
      #20;
      key_mid_pulse = 1'b0;
      #40;
    end
  endtask

  task pulse_up;
    begin
      key_up_pulse = 1'b1;
      #20;
      key_up_pulse = 1'b0;
      #40;
    end
  endtask

  task pulse_down;
    begin
      key_down_pulse = 1'b1;
      #20;
      key_down_pulse = 1'b0;
      #40;
    end
  endtask

  task pulse_right;
    begin
      key_right_pulse = 1'b1;
      #20;
      key_right_pulse = 1'b0;
      #40;
    end
  endtask

  task pulse_run_step;
    begin
      run_step_pulse = 1'b1;
      #20;
      run_step_pulse = 1'b0;
      #40;
    end
  endtask

  initial
  begin
    rst_n = 1'b0;
    key_up_pulse = 1'b0;
    key_down_pulse = 1'b0;
    key_left_pulse = 1'b0;
    key_right_pulse = 1'b0;
    key_mid_pulse = 1'b0;
    run_step_pulse = 1'b0;
    obstacle_stop = 1'b0;
    step_length_set = 16'd20;
    speed_set = 16'd25;

    #100;
    rst_n = 1'b1;
    #100;

    pulse_mid();
    pulse_up();
    pulse_right();
    pulse_mid();
    pulse_down();
    pulse_mid();
    repeat (3) pulse_run_step();
    #200;
    obstacle_stop = 1'b1;
    #40;
    obstacle_stop = 1'b0;
    #200;
    pulse_mid();
    #500;
    $finish;
  end

  initial begin
    $dumpfile("tb/tb_route_contol_core_fast.vcd");
    $dumpvars(0, route_contol_core_test);
  end
//---Module instantiation---
  route_contol_core route_contol_core0(
    .clk(clk),
    .rst_n(rst_n),
    .key_up_pulse(key_up_pulse),
    .key_down_pulse(key_down_pulse),
    .key_left_pulse(key_left_pulse),
    .key_right_pulse(key_right_pulse),
    .key_mid_pulse(key_mid_pulse),
    .menu_info(menu_info),
    .route_info(route_info),
    .run_step_pulse(run_step_pulse),
    .obstacle_stop(obstacle_stop),
    .step_length_set(step_length_set),
    .speed_set(speed_set),
    .state_info(state_info),
    .old_route_info(old_route_info),
    .active_route_steps(active_route_steps),
    .completed_route_steps(completed_route_steps),
    .step_length(step_length),
    .speed(speed),
    .total_route_length(total_route_length),
    .complete_route_length(complete_route_length),
    .remaining_route_length(remaining_route_length),
    .now_x(now_x),
    .now_y(now_y),
    .target_x(target_x),
    .target_y(target_y)
);

endmodule    //route_contol_core_test

module sys_route_control_fsm_test();
reg clk;
reg rst_n;
reg key_up_n;
reg key_down_n;
reg key_left_n;
reg key_right_n;
reg key_mid_n;
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

  task press_mid;
    begin
      key_mid_n = 1'b0;
      #25000000;
      key_mid_n = 1'b1;
      #5000000;
    end
  endtask

  task press_left;
    begin
      key_left_n = 1'b0;
      #25000000;
      key_left_n = 1'b1;
      #5000000;
    end
  endtask

  task press_right;
    begin
      key_right_n = 1'b0;
      #25000000;
      key_right_n = 1'b1;
      #5000000;
    end
  endtask

  initial
  begin
    rst_n = 1'b0;
    key_up_n = 1'b1;
    key_down_n = 1'b1;
    key_left_n = 1'b1;
    key_right_n = 1'b1;
    key_mid_n = 1'b1;
    run_step_pulse = 1'b0;
    obstacle_stop = 1'b0;
    step_length_set = 16'd20;
    speed_set = 16'd80;

    #200;
    rst_n = 1'b1;
    #1000000;

    press_mid;
    press_mid;
    press_right;
    press_mid;
    press_left;
    press_mid;

    #1000;
    run_step_pulse = 1'b1;
    #20;
    run_step_pulse = 1'b0;
    #1000000;

    run_step_pulse = 1'b1;
    #20;
    run_step_pulse = 1'b0;
    #1000000;

    $finish;
  end

  initial begin
    $dumpfile("tb/tb_sys_route_control_fsm_robei.vcd");
    $dumpvars(0, sys_route_control_fsm_test);
  end
//---Module instantiation---
  sys_route_control_fsm sys_route_control_fsm0(
    .clk(clk),
    .rst_n(rst_n),
    .key_up_n(key_up_n),
    .key_down_n(key_down_n),
    .key_left_n(key_left_n),
    .key_right_n(key_right_n),
    .key_mid_n(key_mid_n),
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

endmodule    //sys_route_control_fsm_test

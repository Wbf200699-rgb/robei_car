`timescale 1ns / 1ps

module tb_sys_route_control_fsm;

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

  sys_route_control_fsm dut (
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

  initial begin
    clk = 1'b0;
    forever #10 clk = ~clk;  // 50 MHz
  end

  task show_state;
    input [8*24-1:0] tag;
    begin
      $display("[%0t] %0s state=%b menu=%b active=%0d done=%0d route=%h old=%h now=(%0d,%0d) target=(%0d,%0d) total=%0d done_len=%0d left=%0d",
               $time, tag, state_info, menu_info, active_route_steps, completed_route_steps,
               route_info, old_route_info, now_x, now_y, target_x, target_y,
               total_route_length, complete_route_length, remaining_route_length);
    end
  endtask

  task press_key;
    input [2:0] key_id;
    begin
      case (key_id)
        3'd0: key_up_n = 1'b0;
        3'd1: key_down_n = 1'b0;
        3'd2: key_left_n = 1'b0;
        3'd3: key_right_n = 1'b0;
        default: key_mid_n = 1'b0;
      endcase
      #25_000_000;   // long enough for 1,000,000-cycle debounce at 50 MHz
      key_up_n = 1'b1;
      key_down_n = 1'b1;
      key_left_n = 1'b1;
      key_right_n = 1'b1;
      key_mid_n = 1'b1;
      #25_000_000;
    end
  endtask

  task pulse_run_step;
    begin
      @(posedge clk);
      run_step_pulse = 1'b1;
      @(posedge clk);
      run_step_pulse = 1'b0;
      #1_000;
    end
  endtask

  task pulse_obstacle;
    begin
      @(posedge clk);
      obstacle_stop = 1'b1;
      @(posedge clk);
      obstacle_stop = 1'b0;
      #1_000;
    end
  endtask

  initial begin
    rst_n = 1'b0;
    key_up_n = 1'b1;
    key_down_n = 1'b1;
    key_left_n = 1'b1;
    key_right_n = 1'b1;
    key_mid_n = 1'b1;
    run_step_pulse = 1'b0;
    obstacle_stop = 1'b0;
    step_length_set = 16'd20;
    speed_set = 16'd25;

    #1_000;
    rst_n = 1'b1;
    #1_000;
    show_state("after reset");

    press_key(3'd4);
    show_state("enter change");

    press_key(3'd0);
    show_state("add forward");

    press_key(3'd2);
    show_state("add left+forward");

    press_key(3'd3);
    show_state("add right+forward");

    press_key(3'd1);
    show_state("undo last");

    press_key(3'd4);
    show_state("exit change");

    press_key(3'd1);
    show_state("focus run");

    press_key(3'd4);
    show_state("enter run");

    pulse_run_step();
    show_state("run step 1");

    pulse_run_step();
    show_state("run step 2");

    pulse_obstacle();
    show_state("obstacle stop");

    press_key(3'd4);
    show_state("re-enter change from stop");

    #1_000_000;
    $display("[%0t] tb_sys_route_control_fsm finished", $time);
    $finish;
  end

endmodule

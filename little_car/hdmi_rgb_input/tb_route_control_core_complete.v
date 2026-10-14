`timescale 1ns / 1ps

module tb_route_control_core_complete;

  localparam [4:0] STATE_MENU = 5'b00000;
  localparam [4:0] STATE_RUN  = 5'b00011;
  localparam [1:0] MENU_FOCUS_RUN = 2'b10;

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
  wire [4:0] state_info;
  wire [63:0] route_info;
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

  route_contol_core dut (
    .clk(clk),
    .rst_n(rst_n),
    .key_up_pulse(key_up_pulse),
    .key_down_pulse(key_down_pulse),
    .key_left_pulse(key_left_pulse),
    .key_right_pulse(key_right_pulse),
    .key_mid_pulse(key_mid_pulse),
    .run_step_pulse(run_step_pulse),
    .obstacle_stop(obstacle_stop),
    .step_length_set(step_length_set),
    .speed_set(speed_set),
    .menu_info(menu_info),
    .state_info(state_info),
    .route_info(route_info),
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
    forever #10 clk = ~clk;
  end

  task pulse_key;
    inout key_sig;
    begin
      key_sig = 1'b1;
      @(posedge clk);
      key_sig = 1'b0;
      @(posedge clk);
    end
  endtask

  task pulse_run_step;
    begin
      run_step_pulse = 1'b1;
      @(posedge clk);
      run_step_pulse = 1'b0;
      @(posedge clk);
    end
  endtask

  initial begin
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

    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);

    pulse_key(key_mid_pulse);   // enter CHANGE
    pulse_key(key_up_pulse);    // add forward
    pulse_key(key_up_pulse);    // add forward
    pulse_key(key_mid_pulse);   // back to MENU
    pulse_key(key_down_pulse);  // focus RUN

    if (menu_info !== MENU_FOCUS_RUN)
      $fatal(1, "complete tb: expected RUN focus, got menu=%b", menu_info);

    pulse_key(key_mid_pulse);   // enter RUN

    if (state_info !== STATE_RUN || active_route_steps !== 6'd2)
      $fatal(1, "complete tb: expected RUN with 2 active steps, got state=%b active=%0d", state_info, active_route_steps);

    pulse_run_step();
    if (active_route_steps !== 6'd1 || completed_route_steps !== 6'd1)
      $fatal(1, "complete tb: after step1 expected active=1 done=1, got active=%0d done=%0d", active_route_steps, completed_route_steps);

    pulse_run_step();
    if (active_route_steps !== 6'd0 || completed_route_steps !== 6'd2)
      $fatal(1, "complete tb: after step2 expected active=0 done=2, got active=%0d done=%0d", active_route_steps, completed_route_steps);

    repeat (2) @(posedge clk);

    if (state_info !== STATE_MENU)
      $fatal(1, "complete tb: expected return to MENU, got state=%b", state_info);
    if (route_info !== 64'd0)
      $fatal(1, "complete tb: route_info should be empty after completion");
    if (now_y !== 8'd126)
      $fatal(1, "complete tb: now_y expected 126 after two forward steps, got %0d", now_y);

    $display("[%0t] PASS tb_route_control_core_complete", $time);
    $finish;
  end

endmodule

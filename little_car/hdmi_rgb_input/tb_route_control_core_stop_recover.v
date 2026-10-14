`timescale 1ns / 1ps

module tb_route_control_core_stop_recover;

  localparam [4:0] STATE_CHANGE = 5'b00001;
  localparam [4:0] STATE_RUN    = 5'b00011;
  localparam [4:0] STATE_STOP   = 5'b00100;

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

    pulse_key(key_mid_pulse);   // CHANGE
    pulse_key(key_up_pulse);    // add one step
    pulse_key(key_mid_pulse);   // MENU
    pulse_key(key_down_pulse);  // focus RUN
    pulse_key(key_mid_pulse);   // RUN

    if (state_info !== STATE_RUN || active_route_steps !== 6'd1)
      $fatal(1, "stop tb: expected RUN with one active step");

    obstacle_stop = 1'b1;
    @(posedge clk);
    obstacle_stop = 1'b0;
    @(posedge clk);

    if (state_info !== STATE_STOP)
      $fatal(1, "stop tb: expected STOP after obstacle, got %b", state_info);
    if (active_route_steps !== 6'd0 || route_info !== 64'd0)
      $fatal(1, "stop tb: active route should be cleared on STOP");

    pulse_key(key_mid_pulse);

    if (state_info !== STATE_CHANGE)
      $fatal(1, "stop tb: expected CHANGE after key_mid in STOP, got %b", state_info);
    if (completed_route_steps !== 6'd0 || old_route_info !== 64'd0)
      $fatal(1, "stop tb: old route history should be cleared on STOP recovery");

    $display("[%0t] PASS tb_route_control_core_stop_recover", $time);
    $finish;
  end

endmodule

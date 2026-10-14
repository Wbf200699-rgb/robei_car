`timescale 1ns/1ps

module tb_c5_route_idle_obstacle;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg motor_enable = 1'b1;
  reg [4:0] state_info = 5'b00001;
  reg [63:0] route_info = 64'd0;
  reg [5:0] active_route_steps = 6'd0;
  reg [15:0] forward_ms = 16'd1000;
  reg [15:0] turn_ms = 16'd500;
  reg [9:0] forward_speed = 10'd700;
  reg [9:0] turn_speed = 10'd700;
  reg [15:0] step_length = 16'd20;
  reg [9:0] overall_speed = 10'd700;
  reg [9:0] lf_default_speed = 10'd700;
  reg [9:0] rf_default_speed = 10'd700;
  reg [9:0] lr_default_speed = 10'd700;
  reg [9:0] rr_default_speed = 10'd700;
  reg manual_mode = 1'b0;
  reg [2:0] manual_cmd = 3'd0;
  reg obstacle_stop = 1'b0;

  wire run_step_pulse;
  wire [10:0] lf_speed;
  wire [10:0] rf_speed;
  wire [10:0] lr_speed;
  wire [10:0] rr_speed;
  wire motion_busy;
  wire [1:0] current_cmd;
  wire motor_blocked;

  always #10 clk = ~clk;

  c5_route_controller dut (
    .clk(clk), .rst_n(rst_n), .motor_enable(motor_enable),
    .state_info(state_info), .route_info(route_info),
    .active_route_steps(active_route_steps), .forward_ms(forward_ms),
    .turn_ms(turn_ms), .forward_speed(forward_speed),
    .turn_speed(turn_speed), .step_length(step_length),
    .overall_speed(overall_speed), .lf_default_speed(lf_default_speed),
    .rf_default_speed(rf_default_speed), .lr_default_speed(lr_default_speed),
    .rr_default_speed(rr_default_speed), .manual_mode(manual_mode),
    .manual_cmd(manual_cmd), .obstacle_stop(obstacle_stop),
    .run_step_pulse(run_step_pulse), .lf_speed(lf_speed),
    .rf_speed(rf_speed), .lr_speed(lr_speed), .rr_speed(rr_speed),
    .motion_busy(motion_busy), .current_cmd(current_cmd),
    .motor_blocked(motor_blocked)
  );

  initial begin
    #100;
    rst_n = 1'b1;
    #100;

    obstacle_stop = 1'b1;
    repeat (8) @(posedge clk);
    if (motor_blocked || lf_speed || rf_speed || lr_speed || rr_speed) begin
      $display("RESULT TEST_FAIL idle_obstacle_blocked");
      $finish;
    end

    state_info = 5'b00011;
    active_route_steps = 6'd1;
    route_info[1:0] = 2'b11;
    repeat (8) @(posedge clk);
    if (!motor_blocked || lf_speed || rf_speed || lr_speed || rr_speed) begin
      $display("RESULT TEST_FAIL active_obstacle_not_blocked");
      $finish;
    end

    $display("RESULT TEST_PASS c5_route_idle_obstacle");
    $finish;
  end
endmodule

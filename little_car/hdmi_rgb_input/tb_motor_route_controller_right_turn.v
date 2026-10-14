`timescale 1ns / 1ps

module tb_motor_route_controller_right_turn;

  localparam [4:0] STATE_RUN = 5'b00011;
  localparam [1:0] ROUTE_RIGHT = 2'b01;
  localparam [1:0] MOTOR_STOP = 2'b00;
  localparam [1:0] MOTOR_FORWARD = 2'b01;
  localparam [1:0] MOTOR_REVERSE = 2'b10;

  reg clk;
  reg rst_n;
  reg motor_enable;
  reg [4:0] state_info;
  reg [63:0] route_info;
  reg [5:0] active_route_steps;
  reg lf_tick;
  reg rf_tick;
  reg lr_tick;
  reg rr_tick;
  reg [15:0] forward_ticks;
  reg [15:0] turn_ticks;
  reg [7:0] forward_duty;
  reg [7:0] turn_duty;

  wire run_step_pulse;
  wire [1:0] lf_dir;
  wire [1:0] rf_dir;
  wire [1:0] lr_dir;
  wire [1:0] rr_dir;
  wire [7:0] lf_duty;
  wire [7:0] rf_duty;
  wire [7:0] lr_duty;
  wire [7:0] rr_duty;
  wire motion_busy;
  wire [1:0] current_cmd;
  wire [15:0] lf_count;
  wire [15:0] rf_count;
  wire [15:0] lr_count;
  wire [15:0] rr_count;

  motor_route_controller dut (
    .clk(clk),
    .rst_n(rst_n),
    .motor_enable(motor_enable),
    .state_info(state_info),
    .route_info(route_info),
    .active_route_steps(active_route_steps),
    .lf_tick(lf_tick),
    .rf_tick(rf_tick),
    .lr_tick(lr_tick),
    .rr_tick(rr_tick),
    .forward_ticks(forward_ticks),
    .turn_ticks(turn_ticks),
    .forward_duty(forward_duty),
    .turn_duty(turn_duty),
    .run_step_pulse(run_step_pulse),
    .lf_dir(lf_dir),
    .rf_dir(rf_dir),
    .lr_dir(lr_dir),
    .rr_dir(rr_dir),
    .lf_duty(lf_duty),
    .rf_duty(rf_duty),
    .lr_duty(lr_duty),
    .rr_duty(rr_duty),
    .motion_busy(motion_busy),
    .current_cmd(current_cmd),
    .lf_count(lf_count),
    .rf_count(rf_count),
    .lr_count(lr_count),
    .rr_count(rr_count)
  );

  initial begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  task pulse_all_ticks;
    input integer count;
    integer i;
    begin
      for (i = 0; i < count; i = i + 1) begin
        @(negedge clk);
        lf_tick = 1'b1;
        rf_tick = 1'b1;
        lr_tick = 1'b1;
        rr_tick = 1'b1;
        @(negedge clk);
        lf_tick = 1'b0;
        rf_tick = 1'b0;
        lr_tick = 1'b0;
        rr_tick = 1'b0;
      end
    end
  endtask

  initial begin
    rst_n = 1'b0;
    motor_enable = 1'b0;
    state_info = 5'd0;
    route_info = 64'd0;
    active_route_steps = 6'd0;
    lf_tick = 1'b0;
    rf_tick = 1'b0;
    lr_tick = 1'b0;
    rr_tick = 1'b0;
    forward_ticks = 16'd6;
    turn_ticks = 16'd4;
    forward_duty = 8'd120;
    turn_duty = 8'd90;

    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);

    motor_enable = 1'b1;
    state_info = STATE_RUN;
    active_route_steps = 6'd1;
    route_info[1:0] = ROUTE_RIGHT;

    wait (motion_busy == 1'b1);
    @(posedge clk);

    if (current_cmd !== ROUTE_RIGHT)
      $fatal(1, "right-turn tb: current_cmd expected ROUTE_RIGHT, got %b", current_cmd);
    if (lf_dir !== MOTOR_FORWARD || rf_dir !== MOTOR_REVERSE ||
        lr_dir !== MOTOR_FORWARD || rr_dir !== MOTOR_REVERSE)
      $fatal(1, "right-turn tb: direction combo mismatch lf=%b rf=%b lr=%b rr=%b", lf_dir, rf_dir, lr_dir, rr_dir);
    if (lf_duty !== turn_duty || rf_duty !== turn_duty ||
        lr_duty !== turn_duty || rr_duty !== turn_duty)
      $fatal(1, "right-turn tb: duty mismatch");

    pulse_all_ticks(4);
    wait (run_step_pulse == 1'b1);
    @(posedge clk);

    if (motion_busy !== 1'b0)
      $fatal(1, "right-turn tb: motion_busy should clear after turn");
    if (lf_dir !== MOTOR_STOP || rf_dir !== MOTOR_STOP ||
        lr_dir !== MOTOR_STOP || rr_dir !== MOTOR_STOP)
      $fatal(1, "right-turn tb: motor directions should stop after turn");

    $display("[%0t] PASS tb_motor_route_controller_right_turn", $time);
    $finish;
  end

endmodule

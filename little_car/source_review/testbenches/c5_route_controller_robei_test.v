// =============================================================================
// Testbench: c5_route_controller_robei_test
// 验证目标: 验证手动/自动运动、前进避障阻断以及倒车和转向脱困。
// 原始文件: little_car/test/verilog/c5_route_controller_robei_test.v
// 本副本随 export_sources.py 重新生成，请在原测试目录修改测试逻辑。
// =============================================================================

`timescale 1ns/1ps
module c5_route_controller(
  input wire clk,
  input wire rst_n,
  input wire motor_enable,
  input wire [4:0] state_info,
  input wire [63:0] route_info,
  input wire [5:0] active_route_steps,
  input wire [15:0] forward_ms,
  input wire [15:0] turn_ms,
  input wire [9:0] forward_speed,
  input wire [9:0] turn_speed,
  input wire [15:0] step_length,
  input wire [9:0] overall_speed,
  input wire [9:0] lf_default_speed,
  input wire [9:0] rf_default_speed,
  input wire [9:0] lr_default_speed,
  input wire [9:0] rr_default_speed,
  input wire manual_mode,
  input wire [2:0] manual_cmd,
  input wire obstacle_stop,
  output reg run_step_pulse,
  output reg [10:0] lf_speed,
  output reg [10:0] rf_speed,
  output reg [10:0] lr_speed,
  output reg [10:0] rr_speed,
  output reg motion_busy,
  output reg [1:0] current_cmd,
  output reg motor_blocked
);
        localparam integer CLK_HZ = 50000000;
        localparam integer CYCLES_PER_MS = CLK_HZ / 1000;

        localparam [4:0] STATE_RUN = 5'b00011;

        localparam [1:0] ROUTE_STOP    = 2'b00;
        localparam [1:0] ROUTE_RIGHT   = 2'b01;
        localparam [1:0] ROUTE_LEFT    = 2'b10;
        localparam [1:0] ROUTE_FORWARD = 2'b11;
        localparam [2:0] MANUAL_STOP    = 3'd0;
        localparam [2:0] MANUAL_FORWARD = 3'd1;
        localparam [2:0] MANUAL_REVERSE = 3'd2;
        localparam [2:0] MANUAL_LEFT    = 3'd3;
        localparam [2:0] MANUAL_RIGHT   = 3'd4;
        localparam [9:0] CAL_BASE_SPEED = 10'd700;
        localparam [9:0] SPEED_MIN = 10'd100;
        localparam [9:0] SPEED_MAX = 10'd900;

        reg [31:0] elapsed_cycles;
        reg [31:0] target_cycles;
        reg restart_pending;

        wire in_run;
        wire motion_done;
        wire [9:0] effective_forward_speed;
        wire [9:0] effective_turn_speed;

        assign in_run = (state_info == STATE_RUN);
        assign motion_done = motion_busy && (elapsed_cycles >= target_cycles);
        assign effective_forward_speed = (overall_speed == 10'd0) ? forward_speed : overall_speed;
        assign effective_turn_speed = (overall_speed == 10'd0) ? turn_speed : overall_speed;

        function [9:0] adjusted_speed;
          input [9:0] wheel_default;
          input [9:0] common_speed;
          integer signed value;
          begin
            value = common_speed + wheel_default - CAL_BASE_SPEED;
            if (value < SPEED_MIN)
              adjusted_speed = SPEED_MIN;
            else if (value > SPEED_MAX)
              adjusted_speed = SPEED_MAX;
            else
              adjusted_speed = value[9:0];
          end
        endfunction

        function [9:0] turn_mag;
          input [9:0] speed;
          begin
            if (speed > 10'd150)
              turn_mag = speed - 10'd50;
            else
              turn_mag = speed;
          end
        endfunction

        function [10:0] pos_speed;
          input [9:0] speed;
          begin
            pos_speed = {1'b0, speed};
          end
        endfunction

        function [10:0] neg_speed;
          input [9:0] speed;
          begin
            neg_speed = (~{1'b0, speed}) + 11'd1;
          end
        endfunction

        function [31:0] ms_to_cycles;
          input [15:0] ms;
          begin
            if (ms == 16'd0)
              ms_to_cycles = CYCLES_PER_MS - 1;
            else
              ms_to_cycles = ({16'd0, ms} * CYCLES_PER_MS) - 32'd1;
          end
        endfunction

        function [15:0] step_to_ms;
          input [15:0] steps;
          reg [31:0] ms_calc;
          begin
            if (steps == 16'd0) begin
              step_to_ms = forward_ms;
            end else begin
              ms_calc = ({16'd0, steps} << 7) - ({16'd0, steps} << 1) -
                        {16'd0, steps};
              if (ms_calc > 32'd60000)
                step_to_ms = 16'd60000;
              else
                step_to_ms = ms_calc[15:0];
            end
          end
        endfunction

        task stop_motors;
          begin
            lf_speed <= 11'd0;
            rf_speed <= 11'd0;
            lr_speed <= 11'd0;
            rr_speed <= 11'd0;
          end
        endtask

        always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
            run_step_pulse <= 1'b0;
            lf_speed <= 11'd0;
            rf_speed <= 11'd0;
            lr_speed <= 11'd0;
            rr_speed <= 11'd0;
            motion_busy <= 1'b0;
            current_cmd <= ROUTE_STOP;
            elapsed_cycles <= 32'd0;
            target_cycles <= 32'd0;
            restart_pending <= 1'b0;
            motor_blocked <= 1'b0;
          end else begin
            run_step_pulse <= 1'b0;
            motor_blocked <= 1'b0;

            if (!motor_enable) begin
              stop_motors();
              motion_busy <= 1'b0;
              current_cmd <= ROUTE_STOP;
              elapsed_cycles <= 32'd0;
              target_cycles <= 32'd0;
              restart_pending <= 1'b0;
            end else if (manual_mode) begin
              motion_busy <= 1'b0;
              current_cmd <= ROUTE_STOP;
              elapsed_cycles <= 32'd0;
              target_cycles <= 32'd0;
              restart_pending <= 1'b0;
              if (obstacle_stop && (manual_cmd == MANUAL_FORWARD)) begin
                stop_motors();
                motor_blocked <= 1'b1;
              end else begin
                case (manual_cmd)
                  MANUAL_FORWARD: begin
                    lf_speed <= pos_speed(adjusted_speed(lf_default_speed, effective_forward_speed));
                    rf_speed <= pos_speed(adjusted_speed(rf_default_speed, effective_forward_speed));
                    lr_speed <= pos_speed(adjusted_speed(lr_default_speed, effective_forward_speed));
                    rr_speed <= pos_speed(adjusted_speed(rr_default_speed, effective_forward_speed));
                  end
                  MANUAL_REVERSE: begin
                    lf_speed <= neg_speed(adjusted_speed(lf_default_speed, effective_forward_speed));
                    rf_speed <= neg_speed(adjusted_speed(rf_default_speed, effective_forward_speed));
                    lr_speed <= neg_speed(adjusted_speed(lr_default_speed, effective_forward_speed));
                    rr_speed <= neg_speed(adjusted_speed(rr_default_speed, effective_forward_speed));
                  end
                  MANUAL_LEFT: begin
                    lf_speed <= neg_speed(turn_mag(adjusted_speed(lf_default_speed, effective_turn_speed)));
                    rf_speed <= pos_speed(turn_mag(adjusted_speed(rf_default_speed, effective_turn_speed)));
                    lr_speed <= neg_speed(turn_mag(adjusted_speed(lr_default_speed, effective_turn_speed)));
                    rr_speed <= pos_speed(turn_mag(adjusted_speed(rr_default_speed, effective_turn_speed)));
                  end
                  MANUAL_RIGHT: begin
                    lf_speed <= pos_speed(turn_mag(adjusted_speed(lf_default_speed, effective_turn_speed)));
                    rf_speed <= neg_speed(turn_mag(adjusted_speed(rf_default_speed, effective_turn_speed)));
                    lr_speed <= pos_speed(turn_mag(adjusted_speed(lr_default_speed, effective_turn_speed)));
                    rr_speed <= neg_speed(turn_mag(adjusted_speed(rr_default_speed, effective_turn_speed)));
                  end
                  default: stop_motors();
                endcase
              end
            end else if (obstacle_stop) begin
              stop_motors();
              motion_busy <= 1'b0;
              current_cmd <= ROUTE_STOP;
              elapsed_cycles <= 32'd0;
              target_cycles <= 32'd0;
              restart_pending <= 1'b0;
              motor_blocked <= 1'b1;
            end else if (!in_run || active_route_steps == 6'd0) begin
              stop_motors();
              motion_busy <= 1'b0;
              current_cmd <= ROUTE_STOP;
              elapsed_cycles <= 32'd0;
              target_cycles <= 32'd0;
              restart_pending <= 1'b0;
            end else if (!motion_busy && restart_pending) begin
              restart_pending <= 1'b0;
            end else if (!motion_busy) begin
              current_cmd <= route_info[1:0];
              motion_busy <= 1'b1;
              elapsed_cycles <= 32'd0;
              restart_pending <= 1'b0;

              case (route_info[1:0])
                ROUTE_FORWARD: begin
                  lf_speed <= pos_speed(adjusted_speed(lf_default_speed, effective_forward_speed));
                  rf_speed <= pos_speed(adjusted_speed(rf_default_speed, effective_forward_speed));
                  lr_speed <= pos_speed(adjusted_speed(lr_default_speed, effective_forward_speed));
                  rr_speed <= pos_speed(adjusted_speed(rr_default_speed, effective_forward_speed));
                  target_cycles <= ms_to_cycles(step_to_ms(step_length));
                end

                ROUTE_LEFT: begin
                  lf_speed <= neg_speed(turn_mag(adjusted_speed(lf_default_speed, effective_turn_speed)));
                  rf_speed <= pos_speed(turn_mag(adjusted_speed(rf_default_speed, effective_turn_speed)));
                  lr_speed <= neg_speed(turn_mag(adjusted_speed(lr_default_speed, effective_turn_speed)));
                  rr_speed <= pos_speed(turn_mag(adjusted_speed(rr_default_speed, effective_turn_speed)));
                  target_cycles <= ms_to_cycles(turn_ms);
                end

                ROUTE_RIGHT: begin
                  lf_speed <= pos_speed(turn_mag(adjusted_speed(lf_default_speed, effective_turn_speed)));
                  rf_speed <= neg_speed(turn_mag(adjusted_speed(rf_default_speed, effective_turn_speed)));
                  lr_speed <= pos_speed(turn_mag(adjusted_speed(lr_default_speed, effective_turn_speed)));
                  rr_speed <= neg_speed(turn_mag(adjusted_speed(rr_default_speed, effective_turn_speed)));
                  target_cycles <= ms_to_cycles(turn_ms);
                end

                default: begin
                  stop_motors();
                  target_cycles <= CYCLES_PER_MS - 1;
                end
              endcase
            end else begin
              if (motion_done || current_cmd == ROUTE_STOP) begin
                stop_motors();
                motion_busy <= 1'b0;
                elapsed_cycles <= 32'd0;
                run_step_pulse <= 1'b1;
                restart_pending <= 1'b1;
              end else begin
                elapsed_cycles <= elapsed_cycles + 32'd1;
              end
            end
          end
        end
endmodule

module c5_route_controller_robei_test;
  reg clk;
  reg rst_n;
  reg motor_enable;
  reg [4:0] state_info;
  reg [63:0] route_info;
  reg [5:0] active_route_steps;
  reg [15:0] forward_ms;
  reg [15:0] turn_ms;
  reg [9:0] forward_speed;
  reg [9:0] turn_speed;
  reg [15:0] step_length;
  reg [9:0] overall_speed;
  reg [9:0] lf_default_speed;
  reg [9:0] rf_default_speed;
  reg [9:0] lr_default_speed;
  reg [9:0] rr_default_speed;
  reg manual_mode;
  reg [2:0] manual_cmd;
  reg obstacle_stop;
  wire run_step_pulse;
  wire [10:0] lf_speed;
  wire [10:0] rf_speed;
  wire [10:0] lr_speed;
  wire [10:0] rr_speed;
  wire motion_busy;
  wire [1:0] current_cmd;
  wire motor_blocked;


integer fd;
integer errors;

initial clk=0;
always #10 clk=~clk;

task log_case;
  input [8*24-1:0] label_text;
  begin
    #100;
    $display("RESULT %0s speeds=%0d,%0d,%0d,%0d blocked=%0d",label_text,
      $signed(lf_speed),$signed(rf_speed),$signed(lr_speed),$signed(rr_speed),motor_blocked);
    $fwrite(fd,"%0s,%0d,%0d,%0d,%0d,%0d\n",label_text,$signed(lf_speed),
      $signed(rf_speed),$signed(lr_speed),$signed(rr_speed),motor_blocked);
  end
endtask

initial begin
  fd=$fopen("c5_route_results.csv","w");
  $fwrite(fd,"case,lf_speed,rf_speed,lr_speed,rr_speed,motor_blocked\n");
  errors=0; rst_n=0; motor_enable=1; state_info=0; route_info=0;
  active_route_steps=0; forward_ms=1000; turn_ms=500; forward_speed=700;
  turn_speed=700; step_length=20; overall_speed=700;
  lf_default_speed=650; rf_default_speed=750; lr_default_speed=800; rr_default_speed=625;
  manual_mode=1; manual_cmd=0; obstacle_stop=0;
  #200; rst_n=1;
  manual_cmd=1; log_case("manual_forward_trimmed");
  if($signed(lf_speed)!=650 || $signed(rf_speed)!=750 || $signed(lr_speed)!=800 || $signed(rr_speed)!=625) errors=errors+1;
  obstacle_stop=1; log_case("forward_blocked");
  if(lf_speed!=0 || rf_speed!=0 || lr_speed!=0 || rr_speed!=0 || !motor_blocked) errors=errors+1;
  manual_cmd=2; log_case("reverse_escape");
  if($signed(lf_speed)!=-650 || $signed(rf_speed)!=-750 || $signed(lr_speed)!=-800 || $signed(rr_speed)!=-625 || motor_blocked) errors=errors+1;
  manual_cmd=3; log_case("left_escape");
  manual_cmd=4; log_case("right_escape");
  obstacle_stop=0; manual_cmd=0; log_case("manual_stop");
  if(lf_speed!=0 || rf_speed!=0 || lr_speed!=0 || rr_speed!=0) errors=errors+1;
  manual_mode=0; state_info=5'b00011; active_route_steps=1; route_info[1:0]=2'b11;
  log_case("automatic_forward");
  if(errors==0) $display("RESULT TEST_PASS c5_route");
  else $display("RESULT TEST_FAIL c5_route errors=%0d",errors);
  $fclose(fd); $finish;
end


  c5_route_controller c5_route_controller0(
    .clk(clk),
    .rst_n(rst_n),
    .motor_enable(motor_enable),
    .state_info(state_info),
    .route_info(route_info),
    .active_route_steps(active_route_steps),
    .forward_ms(forward_ms),
    .turn_ms(turn_ms),
    .forward_speed(forward_speed),
    .turn_speed(turn_speed),
    .step_length(step_length),
    .overall_speed(overall_speed),
    .lf_default_speed(lf_default_speed),
    .rf_default_speed(rf_default_speed),
    .lr_default_speed(lr_default_speed),
    .rr_default_speed(rr_default_speed),
    .manual_mode(manual_mode),
    .manual_cmd(manual_cmd),
    .obstacle_stop(obstacle_stop),
    .run_step_pulse(run_step_pulse),
    .lf_speed(lf_speed),
    .rf_speed(rf_speed),
    .lr_speed(lr_speed),
    .rr_speed(rr_speed),
    .motion_busy(motion_busy),
    .current_cmd(current_cmd),
    .motor_blocked(motor_blocked)
  );
endmodule

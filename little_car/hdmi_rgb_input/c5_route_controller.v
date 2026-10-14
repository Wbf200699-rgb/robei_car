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
input wire auto_mode,
input wire [2:0] auto_cmd,
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

  //----Code starts here: integrated by Robei-----
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
          // Onboard exploration must stay above the C5 static-friction region,
          // but does not need the full manually selected speed while mapping.
          localparam [9:0] AUTO_SPEED_MAX = 10'd550;

          reg [31:0] elapsed_cycles;
          reg [31:0] target_cycles;
          reg restart_pending;

          wire in_run;
          wire motion_done;
          wire [9:0] effective_forward_speed;
          wire [9:0] effective_turn_speed;
          wire [9:0] selected_forward_speed;
          wire [9:0] selected_turn_speed;
          wire control_mode;
          wire [2:0] control_cmd;

          assign in_run = (state_info == STATE_RUN);
          assign motion_done = motion_busy && (elapsed_cycles >= target_cycles);
          assign selected_forward_speed =
              (overall_speed == 10'd0) ? forward_speed : overall_speed;
          assign selected_turn_speed =
              (overall_speed == 10'd0) ? turn_speed : overall_speed;
          assign effective_forward_speed =
              (auto_mode && selected_forward_speed > AUTO_SPEED_MAX) ?
              AUTO_SPEED_MAX : selected_forward_speed;
          assign effective_turn_speed =
              (auto_mode && selected_turn_speed > AUTO_SPEED_MAX) ?
              AUTO_SPEED_MAX : selected_turn_speed;
          // A local autonomous command is used only while the phone has not
          // explicitly entered manual mode.  This keeps manual emergency
          // control deterministic while allowing mapping to survive BLE loss.
          assign control_mode = manual_mode || auto_mode;
          assign control_cmd = manual_mode ? manual_cmd : auto_cmd;

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
              end else if (control_mode) begin
                motion_busy <= 1'b0;
                current_cmd <= ROUTE_STOP;
                elapsed_cycles <= 32'd0;
                target_cycles <= 32'd0;
                restart_pending <= 1'b0;
                if (obstacle_stop && (control_cmd == MANUAL_FORWARD)) begin
                  stop_motors();
                  motor_blocked <= 1'b1;
                end else begin
                  case (control_cmd)
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
              end else if (!in_run || active_route_steps == 6'd0) begin
                // Obstacle chatter while idle is not a blocked motor event.
                // Keeping motor_blocked low here also prevents needless
                // immediate status reports when no route can move the car.
                stop_motors();
                motion_busy <= 1'b0;
                current_cmd <= ROUTE_STOP;
                elapsed_cycles <= 32'd0;
                target_cycles <= 32'd0;
                restart_pending <= 1'b0;
              end else if (obstacle_stop) begin
                stop_motors();
                motion_busy <= 1'b0;
                current_cmd <= ROUTE_STOP;
                elapsed_cycles <= 32'd0;
                target_cycles <= 32'd0;
                restart_pending <= 1'b0;
                motor_blocked <= 1'b1;
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
endmodule    //c5_route_controller

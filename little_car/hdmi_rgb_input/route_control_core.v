module route_contol_core(
input wire clk,
input wire rst_n,
input wire key_up_pulse,
input wire key_down_pulse,
input wire key_left_pulse,
input wire key_right_pulse,
input wire key_mid_pulse,
input wire run_step_pulse,
input wire obstacle_stop,
input wire [15:0] step_length_set,
input wire [15:0] speed_set,
output reg [1:0] menu_info,
output reg [4:0] state_info,
output reg [63:0] route_info,
output reg [63:0] old_route_info,
output reg [5:0] active_route_steps,
output reg [5:0] completed_route_steps,
output wire [15:0] step_length,
output wire [15:0] speed,
output wire [15:0] total_route_length,
output wire [15:0] complete_route_length,
output wire [15:0] remaining_route_length,
output reg [7:0] now_x,
output reg [7:0] now_y,
output reg [7:0] target_x,
output reg [7:0] target_y
);

  localparam [4:0] STATE_MENU   = 5'b00000;
  localparam [4:0] STATE_CHANGE = 5'b00001;
  localparam [4:0] STATE_RUN    = 5'b00011;
  localparam [4:0] STATE_STOP   = 5'b00100;

  localparam [1:0] MENU_FOCUS_CHANGE = 2'b00;
  localparam [1:0] MENU_IN_CHANGE    = 2'b01;
  localparam [1:0] MENU_FOCUS_RUN    = 2'b10;
  localparam [1:0] MENU_IN_RUN       = 2'b11;

  localparam [1:0] ROUTE_STOP    = 2'b00;
  localparam [1:0] ROUTE_RIGHT   = 2'b01;
  localparam [1:0] ROUTE_LEFT    = 2'b10;
  localparam [1:0] ROUTE_FORWARD = 2'b11;

  localparam [1:0] DIR_UP    = 2'd0;
  localparam [1:0] DIR_RIGHT = 2'd1;
  localparam [1:0] DIR_DOWN  = 2'd2;
  localparam [1:0] DIR_LEFT  = 2'd3;
  localparam integer ROUTE_WORDS = 16;
  localparam integer ROUTE_LAST = ROUTE_WORDS - 1;

  integer i;
  reg focus_run;
  reg [1:0] now_dir;
  reg [1:0] target_dir;
  reg [5:0] step_count;
  reg [15:0] step_is_turn;
  reg [1:0] route_cmd_mem [0:ROUTE_WORDS-1];
  reg [1:0] old_route_cmd_mem [0:ROUTE_WORDS-1];

  assign step_length = (step_length_set == 16'd0) ? 16'd20 : step_length_set;
  assign speed = speed_set;
  assign total_route_length = {10'd0, completed_route_steps + active_route_steps} * step_length;
  assign complete_route_length = {10'd0, completed_route_steps} * step_length;
  assign remaining_route_length = {10'd0, active_route_steps} * step_length;

  always @(*) begin
    case (state_info)
      STATE_CHANGE: menu_info = MENU_IN_CHANGE;
      STATE_RUN:    menu_info = MENU_IN_RUN;
      STATE_STOP:   menu_info = MENU_FOCUS_CHANGE;
      default:      menu_info = focus_run ? MENU_FOCUS_RUN : MENU_FOCUS_CHANGE;
    endcase
  end

  always @(*) begin
    route_info = 64'd0;
    old_route_info = 64'd0;
    for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
      route_info[i*2 +: 2] = route_cmd_mem[i];
      old_route_info[i*2 +: 2] = old_route_cmd_mem[i];
    end
  end

  task move_target_forward;
    begin
      case (target_dir)
        DIR_UP:    target_y <= target_y - 8'd1;
        DIR_RIGHT: target_x <= target_x + 8'd1;
        DIR_DOWN:  target_y <= target_y + 8'd1;
        default:   target_x <= target_x - 8'd1;
      endcase
    end
  endtask

  task move_target_backward;
    begin
      case (target_dir)
        DIR_UP:    target_y <= target_y + 8'd1;
        DIR_RIGHT: target_x <= target_x - 8'd1;
        DIR_DOWN:  target_y <= target_y - 8'd1;
        default:   target_x <= target_x + 8'd1;
      endcase
    end
  endtask

  task move_now_forward;
    begin
      case (now_dir)
        DIR_UP:    now_y <= now_y - 8'd1;
        DIR_RIGHT: now_x <= now_x + 8'd1;
        DIR_DOWN:  now_y <= now_y + 8'd1;
        default:   now_x <= now_x - 8'd1;
      endcase
    end
  endtask

  task clear_active_route;
    begin
      for (i = 0; i < ROUTE_WORDS; i = i + 1)
        route_cmd_mem[i] <= ROUTE_STOP;
      active_route_steps <= 6'd0;
      step_count <= 6'd0;
      step_is_turn <= 16'd0;
      target_x <= now_x;
      target_y <= now_y;
      target_dir <= now_dir;
    end
  endtask

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      focus_run <= 1'b0;
      state_info <= STATE_MENU;
      active_route_steps <= 6'd0;
      completed_route_steps <= 6'd0;
      step_count <= 6'd0;
      step_is_turn <= 32'd0;
      now_x <= 8'd128;
      now_y <= 8'd128;
      target_x <= 8'd128;
      target_y <= 8'd128;
      now_dir <= DIR_UP;
      target_dir <= DIR_UP;
      for (i = 0; i < ROUTE_WORDS; i = i + 1) begin
        route_cmd_mem[i] <= ROUTE_STOP;
        old_route_cmd_mem[i] <= ROUTE_STOP;
      end
    end else begin
      case (state_info)
        STATE_CHANGE: begin
          if (key_mid_pulse) begin
            state_info <= STATE_MENU;
          end else if (key_down_pulse) begin
            if (step_count != 6'd0) begin
              if (step_is_turn[step_count - 6'd1]) begin
                route_cmd_mem[active_route_steps - 6'd1] <= ROUTE_STOP;
                route_cmd_mem[active_route_steps - 6'd2] <= ROUTE_STOP;
                active_route_steps <= active_route_steps - 6'd2;
                step_count <= step_count - 6'd1;
                step_is_turn[step_count - 6'd1] <= 1'b0;
                move_target_backward();
                if (route_cmd_mem[active_route_steps - 6'd2] == ROUTE_LEFT)
                  target_dir <= target_dir + 2'd1;
                else
                  target_dir <= target_dir - 2'd1;
              end else begin
                route_cmd_mem[active_route_steps - 6'd1] <= ROUTE_STOP;
                active_route_steps <= active_route_steps - 6'd1;
                step_count <= step_count - 6'd1;
                step_is_turn[step_count - 6'd1] <= 1'b0;
                move_target_backward();
              end
            end
          end else if (key_up_pulse) begin
            if (active_route_steps < 6'd32 && step_count < 6'd32) begin
              route_cmd_mem[active_route_steps] <= ROUTE_FORWARD;
              active_route_steps <= active_route_steps + 6'd1;
              step_count <= step_count + 6'd1;
              step_is_turn[step_count] <= 1'b0;
              move_target_forward();
            end
          end else if (key_left_pulse) begin
            if (active_route_steps < 6'd15 && step_count < 6'd16) begin
              route_cmd_mem[active_route_steps] <= ROUTE_LEFT;
              route_cmd_mem[active_route_steps + 6'd1] <= ROUTE_FORWARD;
              active_route_steps <= active_route_steps + 6'd2;
              step_count <= step_count + 6'd1;
              step_is_turn[step_count] <= 1'b1;
              target_dir <= target_dir - 2'd1;
              case (target_dir - 2'd1)
                DIR_UP:    target_y <= target_y - 8'd1;
                DIR_RIGHT: target_x <= target_x + 8'd1;
                DIR_DOWN:  target_y <= target_y + 8'd1;
                default:   target_x <= target_x - 8'd1;
              endcase
            end
          end else if (key_right_pulse) begin
            if (active_route_steps < 6'd15 && step_count < 6'd16) begin
              route_cmd_mem[active_route_steps] <= ROUTE_RIGHT;
              route_cmd_mem[active_route_steps + 6'd1] <= ROUTE_FORWARD;
              active_route_steps <= active_route_steps + 6'd2;
              step_count <= step_count + 6'd1;
              step_is_turn[step_count] <= 1'b1;
              target_dir <= target_dir + 2'd1;
              case (target_dir + 2'd1)
                DIR_UP:    target_y <= target_y - 8'd1;
                DIR_RIGHT: target_x <= target_x + 8'd1;
                DIR_DOWN:  target_y <= target_y + 8'd1;
                default:   target_x <= target_x - 8'd1;
              endcase
            end
          end
        end

        STATE_RUN: begin
          if (obstacle_stop) begin
            state_info <= STATE_STOP;
            clear_active_route();
          end else if (run_step_pulse && active_route_steps != 6'd0) begin
            old_route_cmd_mem[completed_route_steps] <= route_cmd_mem[0];
            completed_route_steps <= completed_route_steps + 6'd1;

            if (route_cmd_mem[0] == ROUTE_RIGHT)
              now_dir <= now_dir + 2'd1;
            else if (route_cmd_mem[0] == ROUTE_LEFT)
              now_dir <= now_dir - 2'd1;
            else
              move_now_forward();

            for (i = 0; i < ROUTE_LAST; i = i + 1)
              route_cmd_mem[i] <= route_cmd_mem[i + 1];
            route_cmd_mem[ROUTE_LAST] <= ROUTE_STOP;
            active_route_steps <= active_route_steps - 6'd1;
          end else if (active_route_steps == 6'd0) begin
            state_info <= STATE_MENU;
            focus_run <= 1'b0;
          end
        end

        STATE_STOP: begin
          if (key_mid_pulse) begin
            state_info <= STATE_CHANGE;
            focus_run <= 1'b0;
            clear_active_route();
          end
        end

        default: begin
          state_info <= STATE_MENU;

          if (key_up_pulse)
            focus_run <= 1'b0;
          else if (key_down_pulse)
            focus_run <= 1'b1;

          if (key_mid_pulse) begin
            if (focus_run)
              state_info <= STATE_RUN;
            else
              state_info <= STATE_CHANGE;
          end
        end
      endcase
    end
  end

endmodule

module route_control_core(
input wire clk,
input wire rst_n,
input wire key_up_pulse,
input wire key_down_pulse,
input wire key_left_pulse,
input wire key_right_pulse,
input wire key_mid_pulse,
input wire run_step_pulse,
input wire obstacle_stop,
input wire [15:0] step_length_set,
input wire [15:0] speed_set,
output wire [1:0] menu_info,
output wire [4:0] state_info,
output wire [63:0] route_info,
output wire [63:0] old_route_info,
output wire [5:0] active_route_steps,
output wire [5:0] completed_route_steps,
output wire [15:0] step_length,
output wire [15:0] speed,
output wire [15:0] total_route_length,
output wire [15:0] complete_route_length,
output wire [15:0] remaining_route_length,
output wire [7:0] now_x,
output wire [7:0] now_y,
output wire [7:0] target_x,
output wire [7:0] target_y
);

  route_contol_core impl (
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

endmodule

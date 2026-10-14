module onboard_exploration_controller (
  input wire clk,
  input wire rst_n,
  input wire map_active,
  input wire map_paused,
  input wire manual_mode,
  input wire sensor_calibration_done,
  input wire gyro_polarity_valid,
  input wire lidar_online,
  input wire scan_active,
  input wire lidar_new_scan,
  input wire obstacle_stop,
  input wire [15:0] safe_distance_mm,
  input wire [15:0] sector_front_mm,
  input wire [15:0] sector_front_right_mm,
  input wire [15:0] sector_right_mm,
  input wire [15:0] sector_rear_right_mm,
  input wire [15:0] sector_rear_mm,
  input wire [15:0] sector_rear_left_mm,
  input wire [15:0] sector_left_mm,
  input wire [15:0] sector_front_left_mm,
  input wire [15:0] pose_x_mm,
  input wire [15:0] pose_y_mm,
  input wire [9:0] pose_heading,
  input wire field_config_valid,
  input wire [7:0] field_low_x_cell,
  input wire [7:0] field_high_x_cell,
  input wire [7:0] field_low_y_cell,
  input wire [7:0] field_high_y_cell,
  output wire auto_mode,
  output reg [2:0] auto_cmd,
  output reg [3:0] exploration_state,
  output reg [3:0] stop_reason
);

  parameter integer CLK_HZ = 50000000;

  localparam [2:0] CMD_STOP    = 3'd0;
  localparam [2:0] CMD_FORWARD = 3'd1;
  localparam [2:0] CMD_REVERSE = 3'd2;
  localparam [2:0] CMD_LEFT    = 3'd3;
  localparam [2:0] CMD_RIGHT   = 3'd4;

  localparam [3:0] EXP_STOP    = 4'd0;
  localparam [3:0] EXP_DECIDE  = 4'd1;
  localparam [3:0] EXP_FORWARD = 4'd2;
  localparam [3:0] EXP_REVERSE = 4'd3;
  localparam [3:0] EXP_LEFT    = 4'd4;
  localparam [3:0] EXP_RIGHT   = 4'd5;
  localparam [3:0] EXP_SETTLE  = 4'd6;

  localparam [3:0] STOP_NONE       = 4'd0;
  localparam [3:0] STOP_NOT_ARMED  = 4'd1;
  localparam [3:0] STOP_CALIBRATION = 4'd2;
  localparam [3:0] STOP_LIDAR      = 4'd3;
  localparam [3:0] STOP_SCAN_STALE = 4'd4;

`ifdef ONBOARD_EXPLORATION_SIM_FAST
  localparam integer MS_DIVISOR = 4;
  localparam [15:0] FORWARD_MS = 16'd12;
  localparam [15:0] REVERSE_MS = 16'd8;
  localparam [15:0] TURN_MS = 16'd10;
  localparam [15:0] SETTLE_MS = 16'd4;
  localparam [15:0] SCAN_TIMEOUT_MS = 16'd25;
`else
  localparam integer MS_DIVISOR = CLK_HZ / 1000;
  // Bound blind travel to one short lease. EXP_SETTLE still requires a scan
  // captured after the stop before the next movement is selected.
  localparam [15:0] FORWARD_MS = 16'd90;
  localparam [15:0] REVERSE_MS = 16'd120;
  localparam [15:0] TURN_MS = 16'd300;
  localparam [15:0] SETTLE_MS = 16'd650;
  localparam [15:0] SCAN_TIMEOUT_MS = 16'd1000;
`endif

  localparam [15:0] MAP_LOW_LIMIT_MM = 16'd450;
  localparam [15:0] MAP_HIGH_LIMIT_MM = 16'd5950;

  reg [31:0] ms_divider;
  reg [15:0] action_ms;
  reg [15:0] scan_age_ms;
  reg turn_right_preferred;
  reg settle_scan_seen;

  wire request_active = map_active && !map_paused && !manual_mode;
  wire sensors_ready = sensor_calibration_done && gyro_polarity_valid;
  wire lidar_ready = lidar_online && scan_active;
  wire scan_fresh = scan_age_ms < SCAN_TIMEOUT_MS;
  assign auto_mode = request_active;

  function distance_below;
    input [15:0] distance;
    input [15:0] threshold;
    begin
      distance_below = (distance != 16'hffff) &&
                       (distance >= 16'd50) &&
                       (distance < threshold);
    end
  endfunction

  function [15:0] clearance_value;
    input [15:0] distance;
    begin
      clearance_value = (distance == 16'hffff) ? 16'hfffe : distance;
    end
  endfunction

  wire [15:0] front_threshold = safe_distance_mm + 16'd120;
  wire front_data_valid = (sector_front_mm != 16'hffff) ||
      (sector_front_left_mm != 16'hffff) ||
      (sector_front_right_mm != 16'hffff);
  wire front_blocked = !front_data_valid || obstacle_stop ||
      distance_below(sector_front_mm, front_threshold) ||
      distance_below(sector_front_left_mm, safe_distance_mm) ||
      distance_below(sector_front_right_mm, safe_distance_mm);
  wire rear_blocked = distance_below(sector_rear_mm, safe_distance_mm) ||
      distance_below(sector_rear_left_mm, safe_distance_mm) ||
      distance_below(sector_rear_right_mm, safe_distance_mm);
  wire [15:0] left_clearance =
      (clearance_value(sector_left_mm) < clearance_value(sector_front_left_mm)) ?
      clearance_value(sector_left_mm) : clearance_value(sector_front_left_mm);
  wire [15:0] right_clearance =
      (clearance_value(sector_right_mm) < clearance_value(sector_front_right_mm)) ?
      clearance_value(sector_right_mm) : clearance_value(sector_front_right_mm);



  wire left_turn_blocked =
      distance_below(sector_front_left_mm, front_threshold) ||
      distance_below(sector_left_mm, safe_distance_mm) ||
      distance_below(sector_rear_right_mm, safe_distance_mm);
  wire right_turn_blocked =
      distance_below(sector_front_right_mm, front_threshold) ||
      distance_below(sector_right_mm, safe_distance_mm) ||
      distance_below(sector_rear_left_mm, safe_distance_mm);





  wire [15:0] field_low_x_ext = {8'd0, field_low_x_cell};
  wire [15:0] field_high_x_ext = {8'd0, field_high_x_cell};
  wire [15:0] field_low_y_ext = {8'd0, field_low_y_cell};
  wire [15:0] field_high_y_ext = {8'd0, field_high_y_cell};
  wire [15:0] field_low_x_mm = (field_low_x_ext << 5) +
      (field_low_x_ext << 4) + (field_low_x_ext << 1);
  wire [15:0] field_high_x_mm = (field_high_x_ext << 5) +
      (field_high_x_ext << 4) + (field_high_x_ext << 1);
  wire [15:0] field_low_y_mm = (field_low_y_ext << 5) +
      (field_low_y_ext << 4) + (field_low_y_ext << 1);
  wire [15:0] field_high_y_mm = (field_high_y_ext << 5) +
      (field_high_y_ext << 4) + (field_high_y_ext << 1);
  wire local_map_boundary_ahead =
      ((pose_heading < 10'd128 || pose_heading >= 10'd896) &&
       pose_x_mm >= MAP_HIGH_LIMIT_MM) ||
      ((pose_heading >= 10'd128 && pose_heading < 10'd384) &&
       pose_y_mm >= MAP_HIGH_LIMIT_MM) ||
      ((pose_heading >= 10'd384 && pose_heading < 10'd640) &&
       pose_x_mm <= MAP_LOW_LIMIT_MM) ||
      ((pose_heading >= 10'd640 && pose_heading < 10'd896) &&
       pose_y_mm <= MAP_LOW_LIMIT_MM);
  wire configured_boundary_ahead =
      ((pose_heading < 10'd128 || pose_heading >= 10'd896) &&
       pose_x_mm >= field_high_x_mm) ||
      ((pose_heading >= 10'd128 && pose_heading < 10'd384) &&
       pose_y_mm >= field_high_y_mm) ||
      ((pose_heading >= 10'd384 && pose_heading < 10'd640) &&
       pose_x_mm <= field_low_x_mm) ||
      ((pose_heading >= 10'd640 && pose_heading < 10'd896) &&
       pose_y_mm <= field_low_y_mm);
  wire boundary_ahead = field_config_valid ?
      configured_boundary_ahead : local_map_boundary_ahead;

  wire millisecond_tick = (ms_divider >= MS_DIVISOR - 1);

  always @* begin
    case (exploration_state)
      EXP_FORWARD: auto_cmd = CMD_FORWARD;
      EXP_REVERSE: auto_cmd = CMD_REVERSE;
      EXP_LEFT: auto_cmd = CMD_LEFT;
      EXP_RIGHT: auto_cmd = CMD_RIGHT;
      default: auto_cmd = CMD_STOP;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ms_divider <= 32'd0;
      action_ms <= 16'd0;
      scan_age_ms <= SCAN_TIMEOUT_MS;
      turn_right_preferred <= 1'b0;
      settle_scan_seen <= 1'b0;
      exploration_state <= EXP_STOP;
      stop_reason <= STOP_NOT_ARMED;
    end else begin
      if (lidar_new_scan) begin
        scan_age_ms <= 16'd0;
        if (exploration_state == EXP_SETTLE)
          settle_scan_seen <= 1'b1;
      end

      if (millisecond_tick) begin
        ms_divider <= 32'd0;
        if (!lidar_new_scan && scan_age_ms != 16'hffff)
          scan_age_ms <= scan_age_ms + 16'd1;
        if (action_ms != 16'd0)
          action_ms <= action_ms - 16'd1;

        if (!request_active) begin
          exploration_state <= EXP_STOP;
          action_ms <= 16'd0;
          stop_reason <= STOP_NOT_ARMED;
        end else if (!sensors_ready) begin
          exploration_state <= EXP_STOP;
          action_ms <= 16'd0;
          stop_reason <= STOP_CALIBRATION;
        end else if (!lidar_ready) begin
          exploration_state <= EXP_STOP;
          action_ms <= 16'd0;
          stop_reason <= STOP_LIDAR;
        end else if (!scan_fresh) begin
          exploration_state <= EXP_STOP;
          action_ms <= 16'd0;
          stop_reason <= STOP_SCAN_STALE;
        end else begin
          stop_reason <= STOP_NONE;
          case (exploration_state)
            EXP_STOP, EXP_DECIDE: begin
              if (front_blocked || boundary_ahead) begin
                turn_right_preferred <= right_clearance > left_clearance;
                if (!rear_blocked) begin
                  exploration_state <= EXP_REVERSE;
                  action_ms <= REVERSE_MS;
                end else if (!right_turn_blocked &&
                             (left_turn_blocked || right_clearance > left_clearance)) begin
                  exploration_state <= EXP_RIGHT;
                  action_ms <= TURN_MS;
                end else if (!left_turn_blocked) begin
                  exploration_state <= EXP_LEFT;
                  action_ms <= TURN_MS;
                end else begin
                  exploration_state <= EXP_SETTLE;
                  action_ms <= SETTLE_MS;
                end
              end else begin
                exploration_state <= EXP_FORWARD;
                action_ms <= FORWARD_MS;
              end
            end

            EXP_FORWARD: begin
              if (front_blocked || boundary_ahead) begin
                exploration_state <= EXP_SETTLE;
                action_ms <= SETTLE_MS;
                settle_scan_seen <= 1'b0;
                turn_right_preferred <= right_clearance > left_clearance;
              end else if (action_ms == 16'd0) begin
                exploration_state <= EXP_SETTLE;
                action_ms <= SETTLE_MS;
                settle_scan_seen <= 1'b0;
              end
            end

            EXP_REVERSE: begin
              if (action_ms == 16'd0 || rear_blocked) begin
                if (turn_right_preferred && !right_turn_blocked) begin
                  exploration_state <= EXP_RIGHT;
                  action_ms <= TURN_MS;
                end else if (!left_turn_blocked) begin
                  exploration_state <= EXP_LEFT;
                  action_ms <= TURN_MS;
                end else if (!right_turn_blocked) begin
                  exploration_state <= EXP_RIGHT;
                  action_ms <= TURN_MS;
                end else begin
                  exploration_state <= EXP_SETTLE;
                  action_ms <= SETTLE_MS;
                  settle_scan_seen <= 1'b0;
                end
              end
            end

            EXP_LEFT, EXP_RIGHT: begin
              if (action_ms == 16'd0 ||
                  (exploration_state == EXP_LEFT && left_turn_blocked) ||
                  (exploration_state == EXP_RIGHT && right_turn_blocked)) begin
                exploration_state <= EXP_SETTLE;
                action_ms <= SETTLE_MS;
                settle_scan_seen <= 1'b0;
              end
            end

            EXP_SETTLE: begin
              if (action_ms == 16'd0 && settle_scan_seen) begin
                exploration_state <= EXP_DECIDE;
                settle_scan_seen <= 1'b0;
              end
            end

            default: begin
              exploration_state <= EXP_STOP;
            end
          endcase
        end
      end else begin
        ms_divider <= ms_divider + 32'd1;
      end
    end
  end

endmodule

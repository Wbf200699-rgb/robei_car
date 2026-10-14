module onboard_occupancy_map (
  input wire clk,
  input wire rst_n,
  input wire map_start_pulse,
  input wire map_pause_pulse,
  input wire map_clear_pulse,
  input wire map_sync_pulse,
  input wire gyro_bias_learn_pulse,
  input wire field_config_valid,
  input wire [7:0] field_pose_x_cell,
  input wire [7:0] field_pose_y_cell,
  input wire [9:0] field_heading_1024,
  input wire lidar_sample_valid,
  input wire lidar_new_scan,
  input wire [5:0] lidar_quality,
  input wire [14:0] lidar_angle_q6,
  input wire [15:0] lidar_distance_q2,
  input wire sensor_calibration_done,
  input wire fused_sample_valid,
  input wire [15:0] fused_gyro_z,
  input wire gyro_polarity_valid,
  input wire gyro_positive_is_right,
  input wire flow_sample_valid,
  input wire [7:0] flow_squal,
  input wire [15:0] flow_delta_x,
  input wire [15:0] flow_delta_y,
  input wire uwb_position_valid,
  input wire uwb_position_online,
  input wire [15:0] uwb_position_x_cm,
  input wire [15:0] uwb_position_y_cm,
  input wire [10:0] lf_speed,
  input wire [10:0] rf_speed,
  input wire [10:0] lr_speed,
  input wire [10:0] rr_speed,
  input wire storage_busy,
  input wire storage_mem_req,
  input wire storage_mem_we,
  input wire [13:0] storage_mem_addr,
  input wire [3:0] storage_mem_wdata,
  input wire storage_load_apply,
  input wire [7:0] storage_loaded_epoch,
  input wire [15:0] storage_loaded_pose_x_mm,
  input wire [15:0] storage_loaded_pose_y_mm,
  input wire [9:0] storage_loaded_heading,
  input wire [15:0] storage_loaded_scan_count,
  input wire [15:0] storage_loaded_occupied_count,
  input wire [15:0] storage_loaded_free_count,
  input wire grid_chunk_accept,
  output reg [3:0] storage_mem_rdata,
  output reg storage_mem_rvalid,
  output reg grid_chunk_valid,
  output reg [7:0] grid_chunk_seq,
  output reg [13:0] grid_chunk_base,
  output reg [255:0] grid_chunk_data,
  output reg [7:0] grid_chunk_crc,
  output reg map_active,
  output reg map_paused,
  output reg map_valid,
  output wire map_busy,
  output reg [7:0] map_epoch,
  output reg [15:0] scan_count,
  output reg [15:0] occupied_count,
  output reg [15:0] free_count,
  output reg [15:0] dropped_sample_count,
  output wire [15:0] pose_x_mm,
  output wire [15:0] pose_y_mm,
  output wire [9:0] pose_heading,
  output reg [7:0] pose_flags,
  output reg flow_body_sample_valid,
  output reg [15:0] flow_body_forward_q8,
  output reg [15:0] flow_body_lateral_q8,
  output reg uwb_fusion_active,
  output reg [15:0] uwb_reject_count
);

  parameter integer CLK_HZ = 50000000;




  parameter integer ENABLE_SINGLE_TAG_UWB_FUSION = 0;

  localparam integer MAP_SIZE = 128;
  localparam integer MAP_CENTER = 64;
  localparam integer MAP_RESOLUTION_MM = 50;
  localparam integer ODOM_TICK_CYCLES = CLK_HZ / 100;
  localparam integer REPORT_GAP_CYCLES = CLK_HZ / 5;
  localparam [5:0] MIN_LIDAR_QUALITY = 6'd8;
  localparam [15:0] MIN_DISTANCE_Q2 = 16'd200;
  localparam [15:0] MAX_DISTANCE_Q2 = 16'd20000;
  localparam [7:0] MIN_FLOW_SQUAL = 8'd20;
  localparam [12:0] LIDAR_REPEAT_TOLERANCE_MM = 13'd250;
  localparam [3:0] IMU_STALE_LIMIT_TICKS = 4'd10;
  // BMI088 is configured for +/-250 dps (131.2 LSB/dps).  Counts inside this
  // band are residual static noise after the startup calibration and must not
  // be integrated forever while the vehicle is stopped.
  localparam [17:0] GYRO_HEADING_DEADBAND = 18'd32;
  // With no wheel command, reject the measured single-IMU residual drift but
  // still accept deliberate hand rotation. 256 counts is about 1.95 dps at
  // the configured +/-250 dps range. The previous 96-count band was narrower
  // than IMU1's warm operating offset, so a stationary vehicle accumulated a
  // slow false turn even after an explicit zero-bias learning request.
  localparam [17:0] GYRO_STATIONARY_DEADBAND = 18'd256;
  // Bias tracking uses a wider gate than heading integration. This breaks the
  // old deadlock where an offset just above the heading deadband was both
  // integrated and forbidden from converging. 768 counts is about 5.85 dps;
  // the slow IIR below does not materially absorb an ordinary hand turn.
  localparam [17:0] GYRO_BIAS_TRACK_LIMIT = 18'd768;
  localparam signed [15:0] GYRO_BIAS_LEARN_LIMIT = 16'sd1024;
  localparam integer FLOW_LEARN_WINDOWS = 16;
  localparam signed [11:0] FLOW_LEARN_SPEED_MIN = 12'sd250;
  localparam integer FLOW_SENSOR_REAR_OFFSET_MM = 200;
  localparam integer FLOW_SENSOR_HEIGHT_MM = 105;


  localparam signed [11:0] FLOW_REAR_LEVER_GAIN = 12'sd201;
  localparam [23:0] FLOW_BODY_STEP_LIMIT_Q8 = 24'd1024;
  localparam [17:0] FLOW_TURN_LEARN_GYRO_MIN = 18'd64;


  localparam [23:0] UWB_INNOVATION_LIMIT_Q8 = 24'd4096;

  localparam [2:0] FLOW_CAL_COLLECT = 3'd0;
  localparam [2:0] FLOW_CAL_CHOOSE  = 3'd1;
  localparam [2:0] FLOW_CAL_EVAL    = 3'd2;
  localparam [2:0] FLOW_CAL_COMMIT  = 3'd3;
  localparam [2:0] FLOW_CAL_LOCKED  = 3'd4;

  localparam [3:0] MAP_IDLE = 4'd0;
  localparam [3:0] MAP_PROJECT = 4'd1;
  localparam [3:0] MAP_RAY_SETUP = 4'd2;
  localparam [3:0] MAP_ENDPOINT = 4'd3;
  localparam [3:0] MAP_RAY_ERROR = 4'd4;
  localparam [3:0] MAP_RAY_READ = 4'd5;
  localparam [3:0] MAP_RAY_WAIT = 4'd6;
  localparam [3:0] MAP_RAY_WRITE = 4'd7;

  localparam [3:0] REPORT_IDLE = 4'd0;
  localparam [3:0] REPORT_ADDR = 4'd1;
  localparam [3:0] REPORT_WAIT = 4'd2;
  localparam [3:0] REPORT_CAPTURE = 4'd3;
  localparam [3:0] REPORT_CRC = 4'd4;
  localparam [3:0] REPORT_READY = 4'd5;




  (* ram_style = "block" *) reg signed [3:0] map_mem [0:16383];
  reg [13:0] map_port_a_addr;
  reg map_port_a_we;
  reg signed [3:0] map_port_a_wdata;
  reg signed [3:0] map_port_a_rdata;
  reg [13:0] report_mem_addr;
  reg signed [3:0] map_port_b_rdata;
  reg storage_read_pending;





  wire [13:0] memory_port_a_addr =
      storage_busy ? storage_mem_addr : map_port_a_addr;
  wire memory_port_a_we =
      storage_busy ? (storage_mem_req && storage_mem_we) : map_port_a_we;
  wire signed [3:0] memory_port_a_wdata =
      storage_busy ? storage_mem_wdata : map_port_a_wdata;

  always @(posedge clk) begin
    if (memory_port_a_we)
      map_mem[memory_port_a_addr] <= memory_port_a_wdata;
    map_port_a_rdata <= map_mem[memory_port_a_addr];
    map_port_b_rdata <= map_mem[report_mem_addr];

    if (!rst_n) begin
      storage_read_pending <= 1'b0;
      storage_mem_rvalid <= 1'b0;
      storage_mem_rdata <= 4'sd0;
    end else begin
      storage_read_pending <= storage_busy && storage_mem_req && !storage_mem_we;
      storage_mem_rvalid <= storage_read_pending;
      if (storage_read_pending)
        storage_mem_rdata <= map_port_a_rdata;
    end
  end

  reg clear_active;
  reg [13:0] clear_addr;
  reg [3:0] map_state;
  reg [3:0] report_state;
  reg [26:0] odom_tick_count;
  reg [26:0] report_gap_count;




  reg [25:0] heading_q16;
  reg signed [23:0] pose_x_q8;
  reg signed [23:0] pose_y_q8;
  reg signed [11:0] mean_linear_speed;
  reg signed [15:0] odom_step_q8;
  reg signed [15:0] odom_lateral_step_q8;
  reg signed [25:0] odom_dx_q8;
  reg signed [25:0] odom_dy_q8;
  reg signed [12:0] wheel_speed_sum_pipe;
  reg signed [11:0] wheel_linear_speed_pipe;
  reg signed [15:0] odom_step_pipe_q8;
  reg signed [9:0] heading_cos_pipe_q8;
  reg signed [9:0] heading_sin_pipe_q8;
  reg signed [25:0] odom_forward_x_pipe_q8;
  reg signed [25:0] odom_forward_y_pipe_q8;
  reg signed [25:0] odom_lateral_x_pipe_q8;
  reg signed [25:0] odom_lateral_y_pipe_q8;
  reg signed [25:0] odom_dx_pipe_q8;
  reg signed [25:0] odom_dy_pipe_q8;








  reg signed [23:0] flow_tick_sum_x;
  reg signed [23:0] flow_tick_sum_y;
  reg [7:0] flow_tick_sample_count;
  reg [3:0] flow_age_ticks;
  reg [2:0] flow_cal_state;
  reg [5:0] flow_cal_window_count;
  reg [31:0] flow_learn_abs_x;
  reg [31:0] flow_learn_abs_y;
  reg signed [31:0] flow_learn_aligned_x;
  reg signed [31:0] flow_learn_aligned_y;
  reg [31:0] flow_learn_wheel_q8;
  reg flow_forward_axis_y;
  reg flow_forward_negative;
  reg flow_right_negative;
  reg flow_lateral_valid;
  reg flow_scale_valid;
  reg signed [4:0] flow_scale_shift;
  reg [3:0] flow_scale_eval_index;
  reg [3:0] flow_scale_best_index;
  reg [31:0] flow_scale_candidate_value;
  reg [31:0] flow_scale_target_total;
  reg [31:0] flow_scale_best_error;
  reg [5:0] flow_reject_streak;
  reg flow_fusion_used;
  reg signed [31:0] flow_turn_aligned_x;
  reg signed [31:0] flow_turn_aligned_y;
  reg [7:0] flow_turn_sample_count;
  reg signed [23:0] heading_tick_delta_q16;





  reg uwb_origin_valid;
  reg signed [15:0] uwb_origin_center_x_cm;
  reg signed [15:0] uwb_origin_center_y_cm;





  reg [3:0] imu_age_ticks;
  reg signed [16:0] gyro_residual_bias;
  reg signed [24:0] gyro_residual_bias_q8;
  reg gyro_bias_learn_active;
  reg [7:0] gyro_bias_learn_count;
  reg signed [31:0] gyro_bias_learn_sum;

  reg [12:0] captured_distance_mm;
  reg [4:0] captured_direction;
  reg [7:0] captured_pose_x_cell;
  reg [7:0] captured_pose_y_cell;
  reg [1:0] captured_hit_increment;
  reg [31:0] lidar_direction_seen;
  reg [12:0] lidar_direction_distance_mm [0:31];
  reg signed [9:0] captured_cos;
  reg signed [9:0] captured_sin;
  reg signed [22:0] projected_x_q8;
  reg signed [22:0] projected_y_q8;
  reg signed [15:0] projected_x_cells;
  reg signed [15:0] projected_y_cells;

  reg [7:0] ray_x;
  reg [7:0] ray_y;
  reg [7:0] ray_end_x;
  reg [7:0] ray_end_y;
  reg [7:0] ray_dx;
  reg [7:0] ray_dy;
  reg signed [1:0] ray_sx;
  reg signed [1:0] ray_sy;
  reg signed [9:0] ray_error;
  reg ray_endpoint;
  reg ray_hit_valid;
  reg signed [3:0] next_cell_value;
  reg [1:0] old_cell_class;
  reg [1:0] new_cell_class;

  reg [7:0] report_index;
  reg [6:0] report_crc_index;
  reg [7:0] report_crc_work;
  reg report_full_sync;
  reg [7:0] report_full_chunks_left;

  function [10:0] abs11;
    input signed [10:0] value;
    begin
      abs11 = value[10] ? $unsigned(-value) : $unsigned(value);
    end
  endfunction

  function [23:0] abs24;
    input signed [23:0] value;
    begin
      abs24 = value[23] ? $unsigned(-value) : $unsigned(value);
    end
  endfunction

  function [31:0] abs32;
    input signed [31:0] value;
    begin
      abs32 = value[31] ? $unsigned(-value) : $unsigned(value);
    end
  endfunction

  function signed [23:0] apply_flow_scale;
    input signed [23:0] value;
    input signed [4:0] scale_shift;
    begin
      case (scale_shift)
        -5'sd4: apply_flow_scale = value <<< 4;
        -5'sd3: apply_flow_scale = value <<< 3;
        -5'sd2: apply_flow_scale = value <<< 2;
        -5'sd1: apply_flow_scale = value <<< 1;
         5'sd0: apply_flow_scale = value;
         5'sd1: apply_flow_scale = value >>> 1;
         5'sd2: apply_flow_scale = value >>> 2;
         5'sd3: apply_flow_scale = value >>> 3;
         5'sd4: apply_flow_scale = value >>> 4;
         5'sd5: apply_flow_scale = value >>> 5;
         5'sd6: apply_flow_scale = value >>> 6;
         5'sd7: apply_flow_scale = value >>> 7;
         5'sd8: apply_flow_scale = value >>> 8;
         5'sd9: apply_flow_scale = value >>> 9;
         5'sd10: apply_flow_scale = value >>> 10;
        default: apply_flow_scale = value >>> 11;
      endcase
    end
  endfunction

  function signed [15:0] clamp_step_q8;
    input signed [25:0] value;
    begin
      if (value > 26'sd32767)
        clamp_step_q8 = 16'sd32767;
      else if (value < -26'sd32768)
        clamp_step_q8 = -16'sd32768;
      else
        clamp_step_q8 = value[15:0];
    end
  endfunction





  function signed [5:0] uwb_tag_offset_x_cm;
    input [2:0] heading_octant;
    begin
      case (heading_octant)
        3'd0: uwb_tag_offset_x_cm = 6'sd11;
        3'd1: uwb_tag_offset_x_cm = 6'sd15;
        3'd2: uwb_tag_offset_x_cm = 6'sd10;
        3'd3: uwb_tag_offset_x_cm = 6'sd1;
        3'd4: uwb_tag_offset_x_cm = -6'sd11;
        3'd5: uwb_tag_offset_x_cm = -6'sd15;
        3'd6: uwb_tag_offset_x_cm = -6'sd10;
        default: uwb_tag_offset_x_cm = -6'sd1;
      endcase
    end
  endfunction

  function signed [5:0] uwb_tag_offset_y_cm;
    input [2:0] heading_octant;
    begin
      case (heading_octant)
        3'd0: uwb_tag_offset_y_cm = -6'sd10;
        3'd1: uwb_tag_offset_y_cm = 6'sd1;
        3'd2: uwb_tag_offset_y_cm = 6'sd11;
        3'd3: uwb_tag_offset_y_cm = 6'sd15;
        3'd4: uwb_tag_offset_y_cm = 6'sd10;
        3'd5: uwb_tag_offset_y_cm = -6'sd1;
        3'd6: uwb_tag_offset_y_cm = -6'sd11;
        default: uwb_tag_offset_y_cm = -6'sd15;
      endcase
    end
  endfunction

  wire wheel_forward_straight =
      ($signed(lf_speed) > 11'sd100) &&
      ($signed(rf_speed) > 11'sd100) &&
      ($signed(lr_speed) > 11'sd100) &&
      ($signed(rr_speed) > 11'sd100);
  wire wheel_reverse_straight =
      ($signed(lf_speed) < -11'sd100) &&
      ($signed(rf_speed) < -11'sd100) &&
      ($signed(lr_speed) < -11'sd100) &&
      ($signed(rr_speed) < -11'sd100);
  wire wheel_spin_right =
      ($signed(lf_speed) > 11'sd100) &&
      ($signed(rf_speed) < -11'sd100) &&
      ($signed(lr_speed) > 11'sd100) &&
      ($signed(rr_speed) < -11'sd100);
  wire wheel_spin_left =
      ($signed(lf_speed) < -11'sd100) &&
      ($signed(rf_speed) > 11'sd100) &&
      ($signed(lr_speed) < -11'sd100) &&
      ($signed(rr_speed) > 11'sd100);
  wire wheel_spin_motion = wheel_spin_right || wheel_spin_left;
  wire wheel_straight_motion = wheel_forward_straight || wheel_reverse_straight;
  wire wheel_stationary =
      (abs11(lf_speed) <= 11'd32) && (abs11(rf_speed) <= 11'd32) &&
      (abs11(lr_speed) <= 11'd32) && (abs11(rr_speed) <= 11'd32);

  // Heading is useful as soon as static IMU calibration has completed.  The
  // default gyro polarity remains active until motion calibration learns a
  // better one; do not freeze the displayed vehicle merely because that
  // runtime-only polarity flag was cleared by power-on reset.
  wire heading_tracking_ready = sensor_calibration_done &&
      (imu_age_ticks < IMU_STALE_LIMIT_TICKS);
  // Translation and obstacle projection remain conservative: both require a
  // learned polarity so an unverified sign cannot mirror the stored map.
  wire pose_tracking_ready = heading_tracking_ready && gyro_polarity_valid;
  wire good_flow_sample = flow_sample_valid &&
      (flow_squal >= MIN_FLOW_SQUAL) &&
      (flow_delta_x != 16'h7fff) && (flow_delta_x != 16'h8000) &&
      (flow_delta_y != 16'h7fff) && (flow_delta_y != 16'h8000);

  wire signed [23:0] flow_learn_x_this_tick = wheel_reverse_straight ?
      -$signed(flow_tick_sum_x) : $signed(flow_tick_sum_x);
  wire signed [23:0] flow_learn_y_this_tick = wheel_reverse_straight ?
      -$signed(flow_tick_sum_y) : $signed(flow_tick_sum_y);
  wire signed [23:0] flow_axis_counts = flow_forward_axis_y ?
      $signed(flow_tick_sum_y) : $signed(flow_tick_sum_x);
  wire signed [23:0] flow_forward_counts = flow_forward_negative ?
      -$signed(flow_axis_counts) : $signed(flow_axis_counts);
  wire signed [23:0] flow_lateral_axis_counts = flow_forward_axis_y ?
      $signed(flow_tick_sum_x) : $signed(flow_tick_sum_y);
  wire signed [23:0] flow_lateral_counts = flow_right_negative ?
      -$signed(flow_lateral_axis_counts) :
      $signed(flow_lateral_axis_counts);
  wire signed [23:0] flow_scaled_step =
      apply_flow_scale(flow_forward_counts, flow_scale_shift);
  wire signed [23:0] flow_scaled_lateral_sensor =
      apply_flow_scale(flow_lateral_counts, flow_scale_shift);
  wire signed [23:0] wheel_step_extended =
      {{8{odom_step_pipe_q8[15]}}, odom_step_pipe_q8};
  wire flow_direction_consistent =
      (($signed(wheel_step_extended) >= 0) && ($signed(flow_scaled_step) >= 0)) ||
      (($signed(wheel_step_extended) <= 0) && ($signed(flow_scaled_step) <= 0));
  wire flow_window_usable = flow_scale_valid && wheel_straight_motion &&
      (flow_tick_sample_count != 8'd0) && flow_direction_consistent &&
      (abs24(flow_scaled_step) <= ((abs24(wheel_step_extended) << 1) + 24'd32));
  wire signed [25:0] blended_step_wide =
      $signed({{2{flow_scaled_step[23]}}, flow_scaled_step}) +
      ($signed({{2{flow_scaled_step[23]}}, flow_scaled_step}) <<< 1) +
      $signed({{2{wheel_step_extended[23]}}, wheel_step_extended});
  wire signed [25:0] selected_step_wide = flow_window_usable ?
      ($signed(blended_step_wide) >>> 2) :
      $signed({{2{wheel_step_extended[23]}}, wheel_step_extended});
  wire signed [15:0] selected_odom_step_now = clamp_step_q8(selected_step_wide);

  wire signed [31:0] heading_tick_extended =
      {{8{heading_tick_delta_q16[23]}}, heading_tick_delta_q16};
  wire signed [31:0] lever_gain_product =
      ($signed(heading_tick_extended) <<< 7) +
      ($signed(heading_tick_extended) <<< 6) +
      ($signed(heading_tick_extended) <<< 3) +
      $signed(heading_tick_extended);
  wire signed [23:0] flow_lever_lateral_q8 =
      $signed(lever_gain_product) >>> 21;
  wire signed [24:0] flow_center_lateral_wide =
      $signed({flow_scaled_lateral_sensor[23], flow_scaled_lateral_sensor}) +
      $signed({flow_lever_lateral_q8[23], flow_lever_lateral_q8});
  wire signed [23:0] flow_center_lateral_q8 =
      flow_center_lateral_wide[23:0];
  wire flow_body_window_usable = flow_scale_valid && flow_lateral_valid &&
      (flow_tick_sample_count != 8'd0) && (flow_age_ticks < 4'd10) &&
      (abs24(flow_scaled_step) <= FLOW_BODY_STEP_LIMIT_Q8) &&
      (abs24(flow_center_lateral_q8) <= FLOW_BODY_STEP_LIMIT_Q8);
  wire signed [15:0] selected_lateral_step_now = flow_body_window_usable ?
      clamp_step_q8({{2{flow_center_lateral_q8[23]}},
                     flow_center_lateral_q8}) :
      16'sd0;

  wire [31:0] flow_scale_candidate_error =
      (flow_scale_candidate_value >= flow_scale_target_total) ?
      (flow_scale_candidate_value - flow_scale_target_total) :
      (flow_scale_target_total - flow_scale_candidate_value);

  wire signed [16:0] fused_gyro_extended =
      {fused_gyro_z[15], fused_gyro_z};
  wire signed [24:0] fused_gyro_q8 =
      $signed({{8{fused_gyro_extended[16]}}, fused_gyro_extended}) <<< 8;
  wire signed [24:0] gyro_bias_error_q8 =
      $signed(fused_gyro_q8) - $signed(gyro_residual_bias_q8);
  wire signed [17:0] gyro_corrected =
      $signed({fused_gyro_extended[16], fused_gyro_extended}) -
      $signed({gyro_residual_bias[16], gyro_residual_bias});
  wire signed [20:0] gyro_corrected_extended =
      {{3{gyro_corrected[17]}}, gyro_corrected};
  // heading_q16 uses 1024 codes per revolution with 16 fractional bits.  Do
  // not scale each SPI sample as though the paired-IMU cadence were constant:
  // PMW3901 polls, BMI088 range guards and gyro recovery deliberately stretch
  // that cadence.  Instead, hold the latest valid angular rate and integrate
  // it on the map's deterministic 100 Hz odometry tick.  At +/-250 dps
  // (131.2 LSB/dps), one 10 ms tick contributes raw_count * 14.21 heading-Q16
  // units.  16 - 2 + 1/4 = 14.25 is a close multiplier-free approximation.
  wire signed [22:0] heading_tick_increment =
      ($signed(gyro_corrected_extended) <<< 4) -
      ($signed(gyro_corrected_extended) <<< 1) +
      ($signed(gyro_corrected_extended) >>> 2);
  // heading_q16 is an unsigned modulo accumulator. Explicitly sign-extend the
  // 21-bit gyro increment before mixing signed and unsigned operands; relying
  // on expression promotion zero-extended negative increments and injected a
  // characteristic 2^21 count (11.25 degree) jump per affected sample.
  wire signed [25:0] heading_tick_increment_extended =
      {{3{heading_tick_increment[22]}}, heading_tick_increment};
  // Keep the board heading in the convention consumed by the Android vehicle
  // view. The phone draws this modulo-1024 value in screen coordinates, so a
  // second mathematical-coordinate inversion here mirrors physical turns.
  wire signed [22:0] normalized_heading_tick_increment =
      gyro_positive_is_right ? $signed(heading_tick_increment) :
      -$signed(heading_tick_increment);
  wire signed [17:0] physical_yaw_rate = gyro_positive_is_right ?
      $signed(gyro_corrected) : -$signed(gyro_corrected);
  wire [17:0] physical_yaw_abs = physical_yaw_rate[17] ?
      $unsigned(-physical_yaw_rate) : $unsigned(physical_yaw_rate);
  wire heading_rotation_valid = wheel_stationary ?
      (physical_yaw_abs > GYRO_STATIONARY_DEADBAND) :
      (physical_yaw_abs > GYRO_HEADING_DEADBAND);

  wire signed [5:0] uwb_offset_x_cm =
      uwb_tag_offset_x_cm(heading_q16[25:23]);
  wire signed [5:0] uwb_offset_y_cm =
      uwb_tag_offset_y_cm(heading_q16[25:23]);
  wire signed [16:0] uwb_center_x_wide =
      $signed({uwb_position_x_cm[15], uwb_position_x_cm}) -
      $signed({{11{uwb_offset_x_cm[5]}}, uwb_offset_x_cm});
  wire signed [16:0] uwb_center_y_wide =
      $signed({uwb_position_y_cm[15], uwb_position_y_cm}) -
      $signed({{11{uwb_offset_y_cm[5]}}, uwb_offset_y_cm});
  wire signed [15:0] uwb_center_x_cm = uwb_center_x_wide[15:0];
  wire signed [15:0] uwb_center_y_cm = uwb_center_y_wide[15:0];
  wire signed [16:0] uwb_delta_x_cm =
      $signed({uwb_center_x_cm[15], uwb_center_x_cm}) -
      $signed({uwb_origin_center_x_cm[15], uwb_origin_center_x_cm});
  wire signed [16:0] uwb_delta_y_cm =
      $signed({uwb_center_y_cm[15], uwb_center_y_cm}) -
      $signed({uwb_origin_center_y_cm[15], uwb_origin_center_y_cm});


  wire signed [23:0] uwb_delta_x_q8 =
      ($signed(uwb_delta_x_cm) <<< 5) +
      ($signed(uwb_delta_x_cm) <<< 4) +
      ($signed(uwb_delta_x_cm) <<< 1) + $signed(uwb_delta_x_cm);
  wire signed [23:0] uwb_delta_y_q8 =
      ($signed(uwb_delta_y_cm) <<< 5) +
      ($signed(uwb_delta_y_cm) <<< 4) +
      ($signed(uwb_delta_y_cm) <<< 1) + $signed(uwb_delta_y_cm);
  wire signed [23:0] uwb_target_x_q8 = 24'sd16384 + uwb_delta_x_q8;
  wire signed [23:0] uwb_target_y_q8 = 24'sd16384 + uwb_delta_y_q8;
  wire signed [23:0] uwb_innovation_x_q8 =
      uwb_target_x_q8 - $signed(pose_x_q8);
  wire signed [23:0] uwb_innovation_y_q8 =
      uwb_target_y_q8 - $signed(pose_y_q8);
  wire [23:0] uwb_innovation_x_abs = uwb_innovation_x_q8[23] ?
      $unsigned(-uwb_innovation_x_q8) : $unsigned(uwb_innovation_x_q8);
  wire [23:0] uwb_innovation_y_abs = uwb_innovation_y_q8[23] ?
      $unsigned(-uwb_innovation_y_q8) : $unsigned(uwb_innovation_y_q8);
  wire uwb_measurement_usable = (ENABLE_SINGLE_TAG_UWB_FUSION != 0) &&
      uwb_position_valid && uwb_position_online &&
      map_active && !map_paused && pose_tracking_ready && !clear_active &&
      !storage_busy && uwb_origin_valid &&
      (uwb_innovation_x_abs <= UWB_INNOVATION_LIMIT_Q8) &&
      (uwb_innovation_y_abs <= UWB_INNOVATION_LIMIT_Q8);
  wire signed [24:0] uwb_pose_x_candidate =
      $signed({pose_x_q8[23], pose_x_q8}) +
      ($signed({uwb_innovation_x_q8[23], uwb_innovation_x_q8}) >>> 4);
  wire signed [24:0] uwb_pose_y_candidate =
      $signed({pose_y_q8[23], pose_y_q8}) +
      ($signed({uwb_innovation_y_q8[23], uwb_innovation_y_q8}) >>> 4);
  wire signed [26:0] pose_x_step_candidate =
      $signed({{3{pose_x_q8[23]}}, pose_x_q8}) +
      $signed({odom_dx_pipe_q8[25], odom_dx_pipe_q8});
  wire signed [26:0] pose_y_step_candidate =
      $signed({{3{pose_y_q8[23]}}, pose_y_q8}) +
      $signed({odom_dy_pipe_q8[25], odom_dy_pipe_q8});

  wire lidar_measurement_good = lidar_sample_valid &&
      (lidar_quality >= MIN_LIDAR_QUALITY) &&
      (lidar_angle_q6 < 15'd23040) &&
      (lidar_distance_q2 >= MIN_DISTANCE_Q2) &&
      (lidar_distance_q2 <= MAX_DISTANCE_Q2);
  wire lidar_sample_good = lidar_measurement_good && pose_tracking_ready;

  wire [8:0] lidar_angle_degree = lidar_angle_q6[14:6];
  wire [15:0] degree_times_91 =
      ({7'd0, lidar_angle_degree} << 6) +
      ({7'd0, lidar_angle_degree} << 4) +
      ({7'd0, lidar_angle_degree} << 3) +
      ({7'd0, lidar_angle_degree} << 1) +
      {7'd0, lidar_angle_degree};


  wire [4:0] lidar_direction = (degree_times_91 + 16'd512) >> 10;
  // RPLIDAR reports increasing angles clockwise from the vehicle front, while
  // the map lookup table is counter-clockwise-positive. Subtraction converts
  // a physical right-side return into the map's negative-angle/right side.
  wire [4:0] world_direction = heading_q16[25:21] - lidar_direction;
  wire [12:0] lidar_distance_mm = lidar_distance_q2[14:2];
  wire [12:0] lidar_repeat_delta_mm =
      (lidar_distance_mm >= lidar_direction_distance_mm[world_direction]) ?
      (lidar_distance_mm - lidar_direction_distance_mm[world_direction]) :
      (lidar_direction_distance_mm[world_direction] - lidar_distance_mm);
  wire lidar_sample_spatially_confirmed =
      lidar_direction_seen[world_direction] &&
      (lidar_repeat_delta_mm <= LIDAR_REPEAT_TOLERANCE_MM);

  wire signed [12:0] wheel_speed_sum_now =
      $signed(lf_speed) + $signed(rf_speed) +
      $signed(lr_speed) + $signed(rr_speed);

  wire [7:0] pose_x_cell = pose_x_q8[15:8];
  wire [7:0] pose_y_cell = pose_y_q8[15:8];
  assign pose_x_mm = pose_x_cell * MAP_RESOLUTION_MM;
  assign pose_y_mm = pose_y_cell * MAP_RESOLUTION_MM;
  assign pose_heading = heading_q16[25:16];
  assign map_busy = clear_active || (map_state != MAP_IDLE) || storage_busy;

  wire signed [15:0] endpoint_x_unclamped =
      $signed({8'd0, captured_pose_x_cell}) + $signed(projected_x_cells);
  wire signed [15:0] endpoint_y_unclamped =
      $signed({8'd0, captured_pose_y_cell}) + $signed(projected_y_cells);
  wire endpoint_inside_map =
      (endpoint_x_unclamped >= 16'sd1) &&
      (endpoint_x_unclamped <= 16'sd126) &&
      (endpoint_y_unclamped >= 16'sd1) &&
      (endpoint_y_unclamped <= 16'sd126);

  wire signed [10:0] ray_error_twice = $signed(ray_error) <<< 1;
  wire ray_step_x = ray_error_twice > -$signed({3'b000, ray_dy});
  wire ray_step_y = ray_error_twice < $signed({3'b000, ray_dx});
  wire signed [10:0] ray_error_next =
      $signed(ray_error) +
      (ray_step_x ? -$signed({3'b000, ray_dy}) : 11'sd0) +
      (ray_step_y ?  $signed({3'b000, ray_dx}) : 11'sd0);

  function signed [9:0] cos_q8;
    input [4:0] direction;
    begin
      case (direction)
        5'd0: cos_q8 = 10'sd256;  5'd1: cos_q8 = 10'sd251;
        5'd2: cos_q8 = 10'sd237;  5'd3: cos_q8 = 10'sd213;
        5'd4: cos_q8 = 10'sd181;  5'd5: cos_q8 = 10'sd142;
        5'd6: cos_q8 = 10'sd98;   5'd7: cos_q8 = 10'sd50;
        5'd8: cos_q8 = 10'sd0;    5'd9: cos_q8 = -10'sd50;
        5'd10: cos_q8 = -10'sd98; 5'd11: cos_q8 = -10'sd142;
        5'd12: cos_q8 = -10'sd181; 5'd13: cos_q8 = -10'sd213;
        5'd14: cos_q8 = -10'sd237; 5'd15: cos_q8 = -10'sd251;
        5'd16: cos_q8 = -10'sd256; 5'd17: cos_q8 = -10'sd251;
        5'd18: cos_q8 = -10'sd237; 5'd19: cos_q8 = -10'sd213;
        5'd20: cos_q8 = -10'sd181; 5'd21: cos_q8 = -10'sd142;
        5'd22: cos_q8 = -10'sd98; 5'd23: cos_q8 = -10'sd50;
        5'd24: cos_q8 = 10'sd0; 5'd25: cos_q8 = 10'sd50;
        5'd26: cos_q8 = 10'sd98; 5'd27: cos_q8 = 10'sd142;
        5'd28: cos_q8 = 10'sd181; 5'd29: cos_q8 = 10'sd213;
        5'd30: cos_q8 = 10'sd237; default: cos_q8 = 10'sd251;
      endcase
    end
  endfunction

  function signed [9:0] sin_q8;
    input [4:0] direction;
    begin
      case (direction)
        5'd0: sin_q8 = 10'sd0;    5'd1: sin_q8 = 10'sd50;
        5'd2: sin_q8 = 10'sd98;   5'd3: sin_q8 = 10'sd142;
        5'd4: sin_q8 = 10'sd181;  5'd5: sin_q8 = 10'sd213;
        5'd6: sin_q8 = 10'sd237;  5'd7: sin_q8 = 10'sd251;
        5'd8: sin_q8 = 10'sd256;  5'd9: sin_q8 = 10'sd251;
        5'd10: sin_q8 = 10'sd237; 5'd11: sin_q8 = 10'sd213;
        5'd12: sin_q8 = 10'sd181; 5'd13: sin_q8 = 10'sd142;
        5'd14: sin_q8 = 10'sd98; 5'd15: sin_q8 = 10'sd50;
        5'd16: sin_q8 = 10'sd0; 5'd17: sin_q8 = -10'sd50;
        5'd18: sin_q8 = -10'sd98; 5'd19: sin_q8 = -10'sd142;
        5'd20: sin_q8 = -10'sd181; 5'd21: sin_q8 = -10'sd213;
        5'd22: sin_q8 = -10'sd237; 5'd23: sin_q8 = -10'sd251;
        5'd24: sin_q8 = -10'sd256; 5'd25: sin_q8 = -10'sd251;
        5'd26: sin_q8 = -10'sd237; 5'd27: sin_q8 = -10'sd213;
        5'd28: sin_q8 = -10'sd181; 5'd29: sin_q8 = -10'sd142;
        5'd30: sin_q8 = -10'sd98; default: sin_q8 = -10'sd50;
      endcase
    end
  endfunction

  function [1:0] classify_cell;
    input signed [3:0] value;
    begin



      if (value >= 4'sd4)
        classify_cell = 2'd2;
      else if (value <= -4'sd2)
        classify_cell = 2'd1;
      else
        classify_cell = 2'd0;
    end
  endfunction

  function signed [3:0] update_log_odds;
    input signed [3:0] old_value;
    input endpoint;
    input [1:0] hit_increment;
    reg signed [5:0] candidate;
    begin
      if (endpoint) begin
        case (hit_increment)
          2'd3: candidate = $signed(old_value) + 6'sd3;
          2'd2: candidate = $signed(old_value) + 6'sd2;
          default: candidate = $signed(old_value) + 6'sd1;
        endcase
      end else begin
        candidate = $signed(old_value) - 6'sd1;
      end
      if (candidate > 6'sd7)
        update_log_odds = 4'sd7;
      else if (candidate < -6'sd7)
        update_log_odds = -4'sd7;
      else
        update_log_odds = candidate[3:0];
    end
  endfunction

  function [7:0] hex_ascii;
    input [3:0] value;
    begin
      hex_ascii = value + ((value < 4'd10) ? 8'd48 : 8'd55);
    end
  endfunction

  function [7:0] crc8_next;
    input [7:0] crc_in;
    input [7:0] data_in;
    integer bit_index;
    reg [7:0] value;
    begin
      value = crc_in ^ data_in;
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (value[7]) value = (value << 1) ^ 8'h07;
        else value = value << 1;
      end
      crc8_next = value;
    end
  endfunction

  function [7:0] grid_header_crc;
    input [7:0] sequence_in;
    input [13:0] base_in;
    reg [7:0] value;
    begin
      value = crc8_next(8'h00, "G");
      value = crc8_next(value, "1");
      value = crc8_next(value, hex_ascii(sequence_in[7:4]));
      value = crc8_next(value, hex_ascii(sequence_in[3:0]));
      value = crc8_next(value, hex_ascii({2'd0, base_in[13:12]}));
      value = crc8_next(value, hex_ascii(base_in[11:8]));
      value = crc8_next(value, hex_ascii(base_in[7:4]));
      value = crc8_next(value, hex_ascii(base_in[3:0]));
      grid_header_crc = value;
    end
  endfunction

  function [7:0] clamp_cell;
    input signed [15:0] value;
    begin
      if (value < 16'sd1) clamp_cell = 8'd1;
      else if (value > 16'sd126) clamp_cell = 8'd126;
      else clamp_cell = value[7:0];
    end
  endfunction

  function signed [15:0] q8_mm_to_cells;
    input signed [22:0] value;
    reg signed [28:0] wide_value;
    reg signed [28:0] scaled_value;
    reg signed [28:0] absolute_scaled_value;
    begin
      wide_value = value;
      scaled_value = (wide_value <<< 5) + (wide_value <<< 3) + wide_value;



      if (scaled_value < 0) begin
        absolute_scaled_value = -scaled_value;
        q8_mm_to_cells = -$signed(
            (absolute_scaled_value + 29'sd262144) >>> 19);
      end else begin
        absolute_scaled_value = scaled_value;
        q8_mm_to_cells =
            (absolute_scaled_value + 29'sd262144) >>> 19;
      end
    end
  endfunction

  function [7:0] mm_to_cell;
    input [15:0] value_mm;
    reg [23:0] wide_value;
    begin
      wide_value = ({8'd0, value_mm} << 5) +
                   ({8'd0, value_mm} << 3) +
                   {8'd0, value_mm};
      mm_to_cell = wide_value >> 11;
    end
  endfunction

  wire [7:0] loaded_pose_x_cell = mm_to_cell(storage_loaded_pose_x_mm);
  wire [7:0] loaded_pose_y_cell = mm_to_cell(storage_loaded_pose_y_mm);

  always @* begin
    map_port_a_addr = {ray_y[6:0], ray_x[6:0]};
    map_port_a_we = 1'b0;
    map_port_a_wdata = 4'sd0;
    if (clear_active) begin
      map_port_a_addr = clear_addr;
      map_port_a_we = 1'b1;
      map_port_a_wdata = 4'sd0;
    end else if (map_state == MAP_RAY_WRITE) begin
      map_port_a_addr = {ray_y[6:0], ray_x[6:0]};
      map_port_a_we = 1'b1;
      map_port_a_wdata = next_cell_value;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      map_active <= 1'b0;
      map_paused <= 1'b1;
      map_valid <= 1'b0;
      map_epoch <= 8'd0;
      scan_count <= 16'd0;
      occupied_count <= 16'd0;
      free_count <= 16'd0;
      dropped_sample_count <= 16'd0;
      pose_flags <= 8'd0;
      clear_active <= 1'b1;
      clear_addr <= 14'd0;
      map_state <= MAP_IDLE;
      odom_tick_count <= 27'd0;
      heading_q16 <= 26'd0;
      pose_x_q8 <= MAP_CENTER << 8;
      pose_y_q8 <= MAP_CENTER << 8;
      mean_linear_speed <= 12'sd0;
      odom_step_q8 <= 16'sd0;
      odom_lateral_step_q8 <= 16'sd0;
      odom_dx_q8 <= 26'sd0;
      odom_dy_q8 <= 26'sd0;
      wheel_speed_sum_pipe <= 13'sd0;
      wheel_linear_speed_pipe <= 12'sd0;
      odom_step_pipe_q8 <= 16'sd0;
      heading_cos_pipe_q8 <= 10'sd256;
      heading_sin_pipe_q8 <= 10'sd0;
      odom_forward_x_pipe_q8 <= 26'sd0;
      odom_forward_y_pipe_q8 <= 26'sd0;
      odom_lateral_x_pipe_q8 <= 26'sd0;
      odom_lateral_y_pipe_q8 <= 26'sd0;
      odom_dx_pipe_q8 <= 26'sd0;
      odom_dy_pipe_q8 <= 26'sd0;
      flow_tick_sum_x <= 24'sd0;
      flow_tick_sum_y <= 24'sd0;
      flow_tick_sample_count <= 8'd0;
      flow_age_ticks <= 4'hf;
      flow_cal_state <= FLOW_CAL_COLLECT;
      flow_cal_window_count <= 6'd0;
      flow_learn_abs_x <= 32'd0;
      flow_learn_abs_y <= 32'd0;
      flow_learn_aligned_x <= 32'sd0;
      flow_learn_aligned_y <= 32'sd0;
      flow_learn_wheel_q8 <= 32'd0;
      flow_forward_axis_y <= 1'b1;
      flow_forward_negative <= 1'b0;
      flow_right_negative <= 1'b0;
      flow_lateral_valid <= 1'b0;
      flow_scale_valid <= 1'b0;
      flow_scale_shift <= 5'sd0;
      flow_scale_eval_index <= 4'd0;
      flow_scale_best_index <= 4'd4;
      flow_scale_candidate_value <= 32'd0;
      flow_scale_target_total <= 32'd0;
      flow_scale_best_error <= 32'hffffffff;
      flow_reject_streak <= 6'd0;
      flow_fusion_used <= 1'b0;
      flow_turn_aligned_x <= 32'sd0;
      flow_turn_aligned_y <= 32'sd0;
      flow_turn_sample_count <= 8'd0;
      heading_tick_delta_q16 <= 24'sd0;
      flow_body_sample_valid <= 1'b0;
      flow_body_forward_q8 <= 16'sd0;
      flow_body_lateral_q8 <= 16'sd0;
      uwb_origin_valid <= 1'b0;
      uwb_origin_center_x_cm <= 16'sd0;
      uwb_origin_center_y_cm <= 16'sd0;
      uwb_fusion_active <= 1'b0;
      uwb_reject_count <= 16'd0;
      imu_age_ticks <= 4'hf;
      gyro_residual_bias <= 17'sd0;
      gyro_residual_bias_q8 <= 25'sd0;
      gyro_bias_learn_active <= 1'b0;
      gyro_bias_learn_count <= 8'd0;
      gyro_bias_learn_sum <= 32'sd0;
      captured_distance_mm <= 13'd0;
      captured_direction <= 5'd0;
      captured_pose_x_cell <= MAP_CENTER;
      captured_pose_y_cell <= MAP_CENTER;
      captured_hit_increment <= 2'd1;
      lidar_direction_seen <= 32'd0;
      captured_cos <= 10'sd0;
      captured_sin <= 10'sd0;
      projected_x_q8 <= 23'sd0;
      projected_y_q8 <= 23'sd0;
      projected_x_cells <= 16'sd0;
      projected_y_cells <= 16'sd0;
      ray_x <= MAP_CENTER;
      ray_y <= MAP_CENTER;
      ray_end_x <= MAP_CENTER;
      ray_end_y <= MAP_CENTER;
      ray_dx <= 8'd0;
      ray_dy <= 8'd0;
      ray_sx <= 2'sd1;
      ray_sy <= 2'sd1;
      ray_error <= 10'sd0;
      ray_endpoint <= 1'b0;
      ray_hit_valid <= 1'b0;
      next_cell_value <= 4'sd0;
      old_cell_class <= 2'd0;
      new_cell_class <= 2'd0;
    end else begin




      wheel_speed_sum_pipe <= wheel_speed_sum_now;
      wheel_linear_speed_pipe <= wheel_speed_sum_pipe >>> 2;
      odom_step_pipe_q8 <=
          (($signed(wheel_linear_speed_pipe) <<< 3) -
           $signed(wheel_linear_speed_pipe)) >>> 8;
      heading_cos_pipe_q8 <= cos_q8(heading_q16[25:21]);
      heading_sin_pipe_q8 <= sin_q8(heading_q16[25:21]);
      odom_forward_x_pipe_q8 <=
          ($signed(odom_step_q8) * $signed(heading_cos_pipe_q8)) >>> 8;
      odom_forward_y_pipe_q8 <=
          ($signed(odom_step_q8) * $signed(heading_sin_pipe_q8)) >>> 8;
      odom_lateral_x_pipe_q8 <=
          -($signed(odom_lateral_step_q8) *
            $signed(heading_sin_pipe_q8)) >>> 8;
      odom_lateral_y_pipe_q8 <=
          ($signed(odom_lateral_step_q8) *
           $signed(heading_cos_pipe_q8)) >>> 8;
      odom_dx_pipe_q8 <= odom_forward_x_pipe_q8 + odom_lateral_x_pipe_q8;
      odom_dy_pipe_q8 <= odom_forward_y_pipe_q8 + odom_lateral_y_pipe_q8;

      flow_body_sample_valid <= 1'b0;




      if (odom_tick_count >= ODOM_TICK_CYCLES - 1) begin
        if (heading_tracking_ready && !gyro_bias_learn_active &&
            heading_rotation_valid)
          heading_tick_delta_q16 <=
              {{1{normalized_heading_tick_increment[22]}},
               normalized_heading_tick_increment};
        else
          heading_tick_delta_q16 <= 24'sd0;
      end





      if (odom_tick_count >= ODOM_TICK_CYCLES - 1) begin
        if (good_flow_sample) begin
          flow_tick_sum_x <= {{8{flow_delta_x[15]}}, flow_delta_x};
          flow_tick_sum_y <= {{8{flow_delta_y[15]}}, flow_delta_y};
          flow_tick_sample_count <= 8'd1;
        end else begin
          flow_tick_sum_x <= 24'sd0;
          flow_tick_sum_y <= 24'sd0;
          flow_tick_sample_count <= 8'd0;
        end
      end else if (good_flow_sample) begin
        flow_tick_sum_x <= flow_tick_sum_x +
                           {{8{flow_delta_x[15]}}, flow_delta_x};
        flow_tick_sum_y <= flow_tick_sum_y +
                           {{8{flow_delta_y[15]}}, flow_delta_y};
        if (flow_tick_sample_count != 8'hff)
          flow_tick_sample_count <= flow_tick_sample_count + 8'd1;
      end

      if (good_flow_sample)
        flow_age_ticks <= 4'd0;
      else if ((odom_tick_count >= ODOM_TICK_CYCLES - 1) &&
               (flow_age_ticks != 4'hf))
        flow_age_ticks <= flow_age_ticks + 4'd1;

      if (fused_sample_valid)
        imu_age_ticks <= 4'd0;
      else if ((odom_tick_count >= ODOM_TICK_CYCLES - 1) &&
               (imu_age_ticks != 4'hf))
        imu_age_ticks <= imu_age_ticks + 4'd1;




      if (good_flow_sample && wheel_spin_motion &&
          (physical_yaw_abs >= FLOW_TURN_LEARN_GYRO_MIN) &&
          !flow_lateral_valid) begin
        flow_turn_aligned_x <= flow_turn_aligned_x +
            (wheel_spin_left ?
             -$signed({{16{flow_delta_x[15]}}, flow_delta_x}) :
              $signed({{16{flow_delta_x[15]}}, flow_delta_x}));
        flow_turn_aligned_y <= flow_turn_aligned_y +
            (wheel_spin_left ?
             -$signed({{16{flow_delta_y[15]}}, flow_delta_y}) :
              $signed({{16{flow_delta_y[15]}}, flow_delta_y}));
        if (flow_turn_sample_count != 8'hff)
          flow_turn_sample_count <= flow_turn_sample_count + 8'd1;
      end

      if (flow_scale_valid && !flow_lateral_valid &&
          (flow_turn_sample_count >= 8'd8)) begin
        if (flow_forward_axis_y &&
            (abs32(flow_turn_aligned_x) >= 32'd32)) begin



          flow_right_negative <= !flow_turn_aligned_x[31];
          flow_lateral_valid <= 1'b1;
        end else if (!flow_forward_axis_y &&
                     (abs32(flow_turn_aligned_y) >= 32'd32)) begin
          flow_right_negative <= !flow_turn_aligned_y[31];
          flow_lateral_valid <= 1'b1;
        end
      end




      if (gyro_bias_learn_pulse) begin
        gyro_bias_learn_active <= 1'b1;
        gyro_bias_learn_count <= 8'd0;
        gyro_bias_learn_sum <= 32'sd0;
      end else if (gyro_bias_learn_active && fused_sample_valid) begin
        if (!wheel_stationary) begin
          // Movement invalidates collected samples but leaves the request
          // armed, so learning restarts automatically once the car is still.
          gyro_bias_learn_count <= 8'd0;
          gyro_bias_learn_sum <= 32'sd0;
        end else if (gyro_bias_learn_count == 8'hff) begin
          gyro_residual_bias <=
              ($signed(gyro_bias_learn_sum) + $signed(fused_gyro_extended)) >>> 8;
          gyro_residual_bias_q8 <=
              (($signed(gyro_bias_learn_sum) + $signed(fused_gyro_extended)) >>> 8) <<< 8;
          gyro_bias_learn_active <= 1'b0;
          gyro_bias_learn_count <= 8'd0;
          gyro_bias_learn_sum <= 32'sd0;
        end else begin
          gyro_bias_learn_count <= gyro_bias_learn_count + 8'd1;
          gyro_bias_learn_sum <= gyro_bias_learn_sum +
              $signed(fused_gyro_extended);
        end
      end else if (fused_sample_valid && sensor_calibration_done &&
          wheel_stationary &&
          (physical_yaw_abs <= GYRO_BIAS_TRACK_LIMIT) &&
          ($signed(fused_gyro_z) < GYRO_BIAS_LEARN_LIMIT) &&
          ($signed(fused_gyro_z) > -GYRO_BIAS_LEARN_LIMIT)) begin
        // Track temperature-dependent residual offset over several seconds.
        // A 1/256 step is deliberately slower than the previous 1/64 update:
        // stationary drift converges, while a brief manual chassis rotation
        // cannot significantly move the learned zero-rate reference.
        gyro_residual_bias_q8 <= gyro_residual_bias_q8 +
            ($signed(gyro_bias_error_q8) >>> 8);
        gyro_residual_bias <= $signed(gyro_residual_bias_q8) >>> 8;
      end




      case (flow_cal_state)
        FLOW_CAL_COLLECT: begin
          if ((odom_tick_count >= ODOM_TICK_CYCLES - 1) &&
              wheel_straight_motion &&
              (($signed(wheel_linear_speed_pipe) >= FLOW_LEARN_SPEED_MIN) ||
               ($signed(wheel_linear_speed_pipe) <= -FLOW_LEARN_SPEED_MIN)) &&
              (flow_tick_sample_count != 8'd0)) begin
            flow_learn_abs_x <= flow_learn_abs_x + abs24(flow_tick_sum_x);
            flow_learn_abs_y <= flow_learn_abs_y + abs24(flow_tick_sum_y);
            flow_learn_aligned_x <= flow_learn_aligned_x +
                {{8{flow_learn_x_this_tick[23]}}, flow_learn_x_this_tick};
            flow_learn_aligned_y <= flow_learn_aligned_y +
                {{8{flow_learn_y_this_tick[23]}}, flow_learn_y_this_tick};
            flow_learn_wheel_q8 <= flow_learn_wheel_q8 +
                abs24(wheel_step_extended);
            if (flow_cal_window_count >= FLOW_LEARN_WINDOWS - 1) begin
              flow_cal_state <= FLOW_CAL_CHOOSE;
              flow_cal_window_count <= flow_cal_window_count;
            end else begin
              flow_cal_window_count <= flow_cal_window_count + 6'd1;
            end
          end
        end

        FLOW_CAL_CHOOSE: begin
          if ((flow_learn_abs_x >
               (flow_learn_abs_y + (flow_learn_abs_y >> 1))) &&
              (abs32(flow_learn_aligned_x) >= 32'd64) &&
              (flow_learn_wheel_q8 >= 32'd64)) begin
            flow_forward_axis_y <= 1'b0;
            flow_forward_negative <= flow_learn_aligned_x[31];



            flow_scale_candidate_value <=
                abs32(flow_learn_aligned_x) << 4;
            flow_scale_target_total <= flow_learn_wheel_q8;
            flow_scale_eval_index <= 4'd0;
            flow_scale_best_index <= 4'd4;
            flow_scale_best_error <= 32'hffffffff;
            flow_cal_state <= FLOW_CAL_EVAL;
          end else if ((flow_learn_abs_y >
                        (flow_learn_abs_x + (flow_learn_abs_x >> 1))) &&
                       (abs32(flow_learn_aligned_y) >= 32'd64) &&
                       (flow_learn_wheel_q8 >= 32'd64)) begin
            flow_forward_axis_y <= 1'b1;
            flow_forward_negative <= flow_learn_aligned_y[31];
            flow_scale_candidate_value <=
                abs32(flow_learn_aligned_y) << 4;
            flow_scale_target_total <= flow_learn_wheel_q8;
            flow_scale_eval_index <= 4'd0;
            flow_scale_best_index <= 4'd4;
            flow_scale_best_error <= 32'hffffffff;
            flow_cal_state <= FLOW_CAL_EVAL;
          end else begin
            flow_cal_window_count <= 6'd0;
            flow_learn_abs_x <= 32'd0;
            flow_learn_abs_y <= 32'd0;
            flow_learn_aligned_x <= 32'sd0;
            flow_learn_aligned_y <= 32'sd0;
            flow_learn_wheel_q8 <= 32'd0;
            flow_cal_state <= FLOW_CAL_COLLECT;
          end
        end

        FLOW_CAL_EVAL: begin
          if (flow_scale_candidate_error < flow_scale_best_error) begin
            flow_scale_best_error <= flow_scale_candidate_error;
            flow_scale_best_index <= flow_scale_eval_index;
          end
          if (flow_scale_eval_index == 4'd15) begin
            flow_cal_state <= FLOW_CAL_COMMIT;
          end else begin
            flow_scale_eval_index <= flow_scale_eval_index + 4'd1;
            flow_scale_candidate_value <= flow_scale_candidate_value >> 1;
          end
        end

        FLOW_CAL_COMMIT: begin
          if (flow_scale_best_error <=
              ((flow_scale_target_total >> 1) + 32'd16)) begin
            flow_scale_shift <=
                $signed({1'b0, flow_scale_best_index}) - 5'sd4;
            flow_scale_valid <= 1'b1;
            flow_reject_streak <= 6'd0;
            flow_cal_state <= FLOW_CAL_LOCKED;
          end else begin
            flow_scale_valid <= 1'b0;
            flow_cal_window_count <= 6'd0;
            flow_learn_abs_x <= 32'd0;
            flow_learn_abs_y <= 32'd0;
            flow_learn_aligned_x <= 32'sd0;
            flow_learn_aligned_y <= 32'sd0;
            flow_learn_wheel_q8 <= 32'd0;
            flow_cal_state <= FLOW_CAL_COLLECT;
          end
        end

        FLOW_CAL_LOCKED: begin
          if ((odom_tick_count >= ODOM_TICK_CYCLES - 1) &&
              wheel_straight_motion && (flow_tick_sample_count != 8'd0)) begin
            if (flow_window_usable) begin
              flow_reject_streak <= 6'd0;
            end else if (flow_reject_streak >= 6'd31) begin
              flow_scale_valid <= 1'b0;
              flow_reject_streak <= 6'd0;
              flow_cal_window_count <= 6'd0;
              flow_learn_abs_x <= 32'd0;
              flow_learn_abs_y <= 32'd0;
              flow_learn_aligned_x <= 32'sd0;
              flow_learn_aligned_y <= 32'sd0;
              flow_learn_wheel_q8 <= 32'd0;
              flow_cal_state <= FLOW_CAL_COLLECT;
            end else begin
              flow_reject_streak <= flow_reject_streak + 6'd1;
            end
          end
        end

        default: flow_cal_state <= FLOW_CAL_COLLECT;
      endcase

      pose_flags[0] <= sensor_calibration_done;
      pose_flags[1] <= gyro_polarity_valid;
      pose_flags[2] <= flow_scale_valid && flow_lateral_valid &&
                       (flow_age_ticks < 4'd10);
      pose_flags[3] <= map_active && !map_paused;
      pose_flags[4] <= storage_busy;
      pose_flags[5] <= clear_active;
      pose_flags[6] <= map_valid;
      pose_flags[7] <= (dropped_sample_count != 16'd0);

      if (map_start_pulse) begin
        map_active <= 1'b1;
        map_paused <= 1'b0;
        // J resumes a paused session without jumping back to its initial pose.
        // App-provided coordinates are consumed only when starting a new map.
        if (!map_active) begin
          if (field_config_valid) begin
            pose_x_q8 <= {field_pose_x_cell, 8'd0};
            pose_y_q8 <= {field_pose_y_cell, 8'd0};
            heading_q16 <= {field_heading_1024, 16'd0};
          end
          odom_step_q8 <= 16'sd0;
          odom_lateral_step_q8 <= 16'sd0;
          odom_dx_q8 <= 26'sd0;
          odom_dy_q8 <= 26'sd0;
          odom_dx_pipe_q8 <= 26'sd0;
          odom_dy_pipe_q8 <= 26'sd0;
          uwb_origin_valid <= 1'b0;
          uwb_fusion_active <= 1'b0;
        end
        lidar_direction_seen <= 32'd0;
      end
      if (map_pause_pulse && map_active)
        map_paused <= 1'b1;

      if (map_clear_pulse && !storage_busy) begin
        map_active <= 1'b0;
        map_paused <= 1'b1;
        clear_active <= 1'b1;
        clear_addr <= 14'd0;
        map_state <= MAP_IDLE;
        map_valid <= 1'b0;
        map_epoch <= map_epoch + 8'd1;
        scan_count <= 16'd0;
        occupied_count <= 16'd0;
        free_count <= 16'd0;
        dropped_sample_count <= 16'd0;
        lidar_direction_seen <= 32'd0;
        heading_q16 <= 26'd0;
        pose_x_q8 <= MAP_CENTER << 8;
        pose_y_q8 <= MAP_CENTER << 8;
        odom_step_q8 <= 16'sd0;
        odom_lateral_step_q8 <= 16'sd0;
        odom_dx_q8 <= 26'sd0;
        odom_dy_q8 <= 26'sd0;
        odom_dx_pipe_q8 <= 26'sd0;
        odom_dy_pipe_q8 <= 26'sd0;
        uwb_origin_valid <= 1'b0;
        uwb_fusion_active <= 1'b0;
      end else if (storage_load_apply) begin
        map_active <= 1'b0;
        map_paused <= 1'b1;
        map_valid <= 1'b1;
        clear_active <= 1'b0;
        clear_addr <= 14'd0;
        map_epoch <= storage_loaded_epoch;
        scan_count <= storage_loaded_scan_count;
        occupied_count <= storage_loaded_occupied_count;
        free_count <= storage_loaded_free_count;
        dropped_sample_count <= 16'd0;
        heading_q16 <= {storage_loaded_heading, 16'd0};
        pose_x_q8 <= {loaded_pose_x_cell, 8'd0};
        pose_y_q8 <= {loaded_pose_y_cell, 8'd0};
        odom_step_q8 <= 16'sd0;
        odom_lateral_step_q8 <= 16'sd0;
        odom_dx_q8 <= 26'sd0;
        odom_dy_q8 <= 26'sd0;
        odom_dx_pipe_q8 <= 26'sd0;
        odom_dy_pipe_q8 <= 26'sd0;


        uwb_origin_valid <= 1'b0;
        uwb_fusion_active <= 1'b0;
      end else if (clear_active) begin



        if (!storage_busy) begin
          if (clear_addr == 14'd16383) begin
            clear_active <= 1'b0;
            clear_addr <= 14'd0;
            map_valid <= 1'b1;
          end else begin
            clear_addr <= clear_addr + 14'd1;
          end
        end
      end




      // Use a wider zero-rate rejection band while stopped.  Unlike a hard
      // wheel-motion gate this still follows deliberate manual chassis turns,
      // while preventing residual BMI088 offset from rotating the map vehicle.
      if ((odom_tick_count >= ODOM_TICK_CYCLES - 1) &&
          heading_tracking_ready &&
          !gyro_bias_learn_active &&
          heading_rotation_valid) begin
        if (gyro_positive_is_right)
          heading_q16 <= $signed(heading_q16) +
              heading_tick_increment_extended;
        else
          heading_q16 <= $signed(heading_q16) -
              heading_tick_increment_extended;
      end





      if ((ENABLE_SINGLE_TAG_UWB_FUSION != 0) &&
          uwb_position_valid && uwb_position_online && map_active &&
          !map_paused && !clear_active && !storage_busy) begin
        if (!uwb_origin_valid) begin
          uwb_origin_center_x_cm <= uwb_center_x_cm;
          uwb_origin_center_y_cm <= uwb_center_y_cm;
          uwb_origin_valid <= 1'b1;
        end else if (uwb_measurement_usable) begin
          uwb_fusion_active <= 1'b1;
          if ((uwb_pose_x_candidate >= 25'sd256) &&
              (uwb_pose_x_candidate <= 25'sd32512))
            pose_x_q8 <= uwb_pose_x_candidate[23:0];
          if ((uwb_pose_y_candidate >= 25'sd256) &&
              (uwb_pose_y_candidate <= 25'sd32512))
            pose_y_q8 <= uwb_pose_y_candidate[23:0];
        end else if (pose_tracking_ready &&
                     (uwb_reject_count != 16'hffff)) begin
          uwb_reject_count <= uwb_reject_count + 16'd1;
        end
      end

      if ((ENABLE_SINGLE_TAG_UWB_FUSION == 0) || !uwb_position_online)
        uwb_fusion_active <= 1'b0;

      if (odom_tick_count >= ODOM_TICK_CYCLES - 1) begin
        odom_tick_count <= 27'd0;
        mean_linear_speed <= wheel_linear_speed_pipe;




        if (map_active && !map_paused && pose_tracking_ready)
          odom_step_q8 <= selected_odom_step_now;
        else
          odom_step_q8 <= 16'sd0;
        if (map_active && !map_paused && pose_tracking_ready)
          odom_lateral_step_q8 <= selected_lateral_step_now;
        else
          odom_lateral_step_q8 <= 16'sd0;
        flow_fusion_used <= flow_body_window_usable;
        flow_body_sample_valid <= flow_body_window_usable;
        flow_body_forward_q8 <= flow_body_window_usable ?
            clamp_step_q8({{2{flow_scaled_step[23]}}, flow_scaled_step}) :
            16'sd0;
        flow_body_lateral_q8 <= selected_lateral_step_now;
        odom_dx_q8 <= odom_dx_pipe_q8;
        odom_dy_q8 <= odom_dy_pipe_q8;
        if (map_active && !map_paused && pose_tracking_ready &&
            !clear_active && !storage_busy && !uwb_measurement_usable) begin
          if ((pose_x_step_candidate >= 27'sd256) &&
              (pose_x_step_candidate <= 27'sd32512))
            pose_x_q8 <= pose_x_step_candidate[23:0];
          if ((pose_y_step_candidate >= 27'sd256) &&
              (pose_y_step_candidate <= 27'sd32512))
            pose_y_q8 <= pose_y_step_candidate[23:0];
        end
      end else begin
        odom_tick_count <= odom_tick_count + 27'd1;
      end

      if (lidar_sample_good && lidar_new_scan && map_active && !map_paused &&
          !clear_active && !storage_busy && (scan_count != 16'hffff))
        scan_count <= scan_count + 16'd1;

      if (!clear_active && !storage_busy) begin
        case (map_state)
          MAP_IDLE: begin
            if (map_active && !map_paused && lidar_sample_good) begin
              // A wall must repeat in the same 11.25-degree world sector
              // before either its free ray or endpoint changes the map. This
              // rejects isolated multipath/range jumps with only 32 x 13 bits
              // of state and avoids expensive neighborhood combinational logic.
              lidar_direction_distance_mm[world_direction] <= lidar_distance_mm;
              lidar_direction_seen[world_direction] <= 1'b1;
              if (lidar_sample_spatially_confirmed) begin
                captured_distance_mm <= lidar_distance_mm;
                captured_direction <= world_direction;
                captured_pose_x_cell <= pose_x_cell;
                captured_pose_y_cell <= pose_y_cell;
                captured_hit_increment <=
                    (lidar_quality >= 6'd20) ? 2'd2 : 2'd1;
                captured_cos <= cos_q8(world_direction);
                captured_sin <= sin_q8(world_direction);
                map_state <= MAP_PROJECT;
              end
            end
          end

          MAP_PROJECT: begin
            projected_x_q8 <= $signed({1'b0, captured_distance_mm}) * $signed(captured_cos);
            projected_y_q8 <= $signed({1'b0, captured_distance_mm}) * $signed(captured_sin);
            map_state <= MAP_RAY_SETUP;
          end

          MAP_RAY_SETUP: begin


            projected_x_cells <= q8_mm_to_cells(projected_x_q8);
            projected_y_cells <= q8_mm_to_cells(projected_y_q8);
            map_state <= MAP_ENDPOINT;
          end

          MAP_ENDPOINT: begin
            ray_x <= captured_pose_x_cell;
            ray_y <= captured_pose_y_cell;
            ray_end_x <= clamp_cell(endpoint_x_unclamped);
            ray_end_y <= clamp_cell(endpoint_y_unclamped);
            ray_hit_valid <= endpoint_inside_map;
            if (clamp_cell(endpoint_x_unclamped) >= captured_pose_x_cell) begin
              ray_dx <= clamp_cell(endpoint_x_unclamped) - captured_pose_x_cell;
              ray_sx <= 2'sd1;
            end else begin
              ray_dx <= captured_pose_x_cell - clamp_cell(endpoint_x_unclamped);
              ray_sx <= -2'sd1;
            end
            if (clamp_cell(endpoint_y_unclamped) >= captured_pose_y_cell) begin
              ray_dy <= clamp_cell(endpoint_y_unclamped) - captured_pose_y_cell;
              ray_sy <= 2'sd1;
            end else begin
              ray_dy <= captured_pose_y_cell - clamp_cell(endpoint_y_unclamped);
              ray_sy <= -2'sd1;
            end
            map_state <= MAP_RAY_ERROR;
          end

          MAP_RAY_ERROR: begin
            ray_error <= $signed({2'b00, ray_dx}) -
                         $signed({2'b00, ray_dy});
            map_state <= MAP_RAY_READ;
          end

          MAP_RAY_READ: begin
            ray_endpoint <= (ray_x == ray_end_x) && (ray_y == ray_end_y);
            map_state <= MAP_RAY_WAIT;
          end

          MAP_RAY_WAIT: begin
            next_cell_value <= update_log_odds(
                map_port_a_rdata, ray_endpoint && ray_hit_valid,
                captured_hit_increment);
            old_cell_class <= classify_cell(map_port_a_rdata);
            new_cell_class <= classify_cell(update_log_odds(
                map_port_a_rdata, ray_endpoint && ray_hit_valid,
                captured_hit_increment));
            map_state <= MAP_RAY_WRITE;
          end

          MAP_RAY_WRITE: begin
            if ((old_cell_class != 2'd2) && (new_cell_class == 2'd2))
              occupied_count <= occupied_count + 16'd1;
            else if ((old_cell_class == 2'd2) && (new_cell_class != 2'd2) &&
                     (occupied_count != 16'd0))
              occupied_count <= occupied_count - 16'd1;
            if ((old_cell_class != 2'd1) && (new_cell_class == 2'd1))
              free_count <= free_count + 16'd1;
            else if ((old_cell_class == 2'd1) && (new_cell_class != 2'd1) &&
                     (free_count != 16'd0))
              free_count <= free_count - 16'd1;

            if (ray_endpoint) begin
              map_state <= MAP_IDLE;
              map_valid <= 1'b1;
            end else begin
              ray_error <= ray_error_next[9:0];
              if (ray_step_x) begin
                ray_x <= $signed({1'b0, ray_x}) + ray_sx;
              end
              if (ray_step_y) begin
                ray_y <= $signed({1'b0, ray_y}) + ray_sy;
              end
              map_state <= MAP_RAY_READ;
            end
          end

          default: map_state <= MAP_IDLE;
        endcase

        if (lidar_sample_good && (map_state != MAP_IDLE) &&
            (dropped_sample_count != 16'hffff))
          dropped_sample_count <= dropped_sample_count + 16'd1;
      end else begin
        map_state <= MAP_IDLE;
      end
    end
  end




  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      report_state <= REPORT_IDLE;
      report_mem_addr <= 14'd0;
      report_gap_count <= 27'd0;
      report_index <= 8'd0;
      report_crc_index <= 7'd0;
      report_crc_work <= 8'd0;
      report_full_sync <= 1'b0;
      report_full_chunks_left <= 8'd0;
      grid_chunk_valid <= 1'b0;
      grid_chunk_seq <= 8'd0;
      grid_chunk_base <= 14'd0;
      grid_chunk_data <= 256'd0;
      grid_chunk_crc <= 8'd0;
    end else begin
      if (map_sync_pulse) begin
        report_full_sync <= 1'b1;
        report_full_chunks_left <= 8'd128;
        grid_chunk_base <= 14'd0;
        report_gap_count <= REPORT_GAP_CYCLES - 1;
        report_index <= 8'd0;
        report_state <= REPORT_IDLE;
        grid_chunk_valid <= 1'b0;
      end else if (storage_busy || clear_active) begin
        report_state <= REPORT_IDLE;
        grid_chunk_valid <= 1'b0;
        report_gap_count <= 27'd0;
      end else begin
        case (report_state)
          REPORT_IDLE: begin
            if (report_gap_count >= REPORT_GAP_CYCLES - 1) begin
              report_gap_count <= 27'd0;
              if (map_valid && (map_active || report_full_sync)) begin
                report_index <= 8'd0;
                report_mem_addr <= grid_chunk_base;
                grid_chunk_data <= 256'd0;
                report_crc_work <= grid_header_crc(grid_chunk_seq, grid_chunk_base);
                report_state <= REPORT_ADDR;
              end
            end else begin
              report_gap_count <= report_gap_count + 27'd1;
            end
          end

          REPORT_ADDR: begin
            report_mem_addr <= grid_chunk_base + report_index;
            report_state <= REPORT_WAIT;
          end

          REPORT_WAIT: report_state <= REPORT_CAPTURE;

          REPORT_CAPTURE: begin
            grid_chunk_data[report_index * 2 +: 2] <= classify_cell(map_port_b_rdata);
            if (report_index == 8'd127) begin
              report_crc_index <= 7'd0;
              report_state <= REPORT_CRC;
            end else begin
              report_index <= report_index + 8'd1;
              report_state <= REPORT_ADDR;
            end
          end

          REPORT_CRC: begin
            report_crc_work <= crc8_next(
                report_crc_work,
                hex_ascii(grid_chunk_data[report_crc_index * 4 +: 4]));
            if (report_crc_index == 7'd63) begin
              grid_chunk_crc <= crc8_next(
                  report_crc_work,
                  hex_ascii(grid_chunk_data[report_crc_index * 4 +: 4]));
              grid_chunk_valid <= 1'b1;
              report_state <= REPORT_READY;
            end else begin
              report_crc_index <= report_crc_index + 7'd1;
            end
          end

          REPORT_READY: begin
            if (grid_chunk_accept) begin
              grid_chunk_valid <= 1'b0;
              grid_chunk_seq <= grid_chunk_seq + 8'd1;
              if (grid_chunk_base == 14'd16256)
                grid_chunk_base <= 14'd0;
              else
                grid_chunk_base <= grid_chunk_base + 14'd128;
              if (report_full_sync) begin
                if (report_full_chunks_left <= 8'd1) begin
                  report_full_sync <= 1'b0;
                  report_full_chunks_left <= 8'd0;
                end else begin
                  report_full_chunks_left <= report_full_chunks_left - 8'd1;
                end
              end
              report_state <= REPORT_IDLE;
            end
          end

          default: report_state <= REPORT_IDLE;
        endcase
      end
    end
  end

endmodule

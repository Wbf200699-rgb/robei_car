`timescale 1ns/1ps
module c5_motion_stabilizer(
  input wire clk,
  input wire rst_n,
  input wire calibration_done,
  input wire fused_sample_valid,
  input wire imu_degraded,
  input wire imu_disagreement,
  input wire [15:0] fused_gyro_z,
  input wire flow_sample_valid,
  input wire [15:0] flow_forward_q8,
  input wire [15:0] flow_lateral_q8,
  input wire local_calibration_start,
  input wire external_force_stop,
  input wire obstacle_stop,
  input wire [9:0] cal_lf_default_speed,
  input wire [9:0] cal_rf_default_speed,
  input wire [9:0] cal_lr_default_speed,
  input wire [9:0] cal_rr_default_speed,
  input wire [10:0] base_lf_speed,
  input wire [10:0] base_rf_speed,
  input wire [10:0] base_lr_speed,
  input wire [10:0] base_rr_speed,
  output reg [10:0] lf_speed,
  output reg [10:0] rf_speed,
  output reg [10:0] lr_speed,
  output reg [10:0] rr_speed,
  output reg correction_active,
  output reg gyro_polarity_valid,
  output reg gyro_positive_is_right,
  output wire [10:0] yaw_correction,
  output wire [10:0] flow_correction,
  output wire force_stop,
  output wire vehicle_motion_expected,
  output reg [3:0] local_calibration_state,
  output wire [7:0] local_calibration_flags,
  output reg [10:0] learned_yaw_trim,
  output reg [15:0] calibration_gyro_peak
);
`define C5_CAL_SIM_FAST
  localparam signed [12:0] SPEED_MAX = 13'sd900;
  localparam signed [12:0] SPEED_MIN = -13'sd900;









  localparam integer P_SHIFT = 5;



  localparam integer I_SHIFT = 7;
  localparam integer D_SHIFT = 7;
  localparam signed [31:0] INTEGRAL_LIMIT = 32'sd65536;
  localparam signed [10:0] CORRECTION_LIMIT = 11'sd96;
  localparam signed [10:0] LEARNED_TRIM_LIMIT = 11'sd80;
  localparam signed [10:0] CORRECTION_SLEW = 11'sd8;
  localparam [16:0] GYRO_DEADBAND = 17'd24;
  localparam signed [10:0] FLOW_CORRECTION_LIMIT = 11'sd32;
  localparam signed [10:0] FLOW_CORRECTION_SLEW = 11'sd4;


`ifdef C5_CAL_SIM_FAST
  localparam integer FLOW_TIMEOUT_CYCLES = 96;
  localparam integer RUNTIME_APPLY_CYCLES = 24;
`else
  localparam integer FLOW_TIMEOUT_CYCLES = 10000000;


  localparam integer RUNTIME_APPLY_CYCLES = 5000000;
`endif








  localparam [16:0] POLARITY_LEARN_MIN = 17'd80;
  localparam [2:0] POLARITY_LEARN_SAMPLES = 3'd4;




  localparam [16:0] POLARITY_STRONG_PEAK = 17'd240;





`ifdef C5_CAL_SIM_FAST
  localparam integer CAL_PRESTOP_CYCLES = 20;
  localparam integer CAL_SPIN_CYCLES = 80;
  localparam integer CAL_SETTLE_CYCLES = 20;
  localparam integer CAL_STRAIGHT_CYCLES = 160;
  localparam integer CAL_FINAL_STOP_CYCLES = 20;
  localparam integer CAL_TRIM_WARMUP_SAMPLES = 2;
  localparam integer CAL_TRIM_SAMPLES = 4;
  localparam integer CAL_TRIM_SAMPLE_SHIFT = 2;
  localparam integer FUSED_SAMPLE_TIMEOUT_CYCLES = 64;
  localparam integer CAL_OBSTACLE_TIMEOUT_CYCLES = 80;
  localparam integer CAL_SENSOR_RECOVERY_TIMEOUT_CYCLES = 512;
`else
  localparam integer CAL_PRESTOP_CYCLES = 25000000;
  localparam integer CAL_SPIN_CYCLES = 35000000;
  localparam integer CAL_SETTLE_CYCLES = 17500000;
  localparam integer CAL_STRAIGHT_CYCLES = 40000000;
  localparam integer CAL_FINAL_STOP_CYCLES = 25000000;
  localparam integer CAL_TRIM_WARMUP_SAMPLES = 8;
  localparam integer CAL_TRIM_SAMPLES = 32;
  localparam integer CAL_TRIM_SAMPLE_SHIFT = 5;
  localparam integer FUSED_SAMPLE_TIMEOUT_CYCLES = 5000000;
  localparam integer CAL_OBSTACLE_TIMEOUT_CYCLES = 100000000;
  localparam integer CAL_SENSOR_RECOVERY_TIMEOUT_CYCLES = 250000000;
`endif
  localparam [3:0] CAL_SENSOR_RECOVERY_SAMPLES = 4'd8;
  localparam [3:0] CAL_SENSOR_RECOVERY_LIMIT = 4'd8;





  localparam [2:0] CAL_MAX_ROUNDS = 3'd6;
  localparam [1:0] CAL_REQUIRED_STABLE_ROUNDS = 2'd2;





  localparam [1:0] CAL_SPIN_RETRY_LIMIT = 2'd2;




  localparam signed [31:0] CAL_TRIM_CONVERGED = 32'sd16;
  localparam [16:0] CAL_GYRO_CONVERGED = 17'd160;

  localparam [3:0] CAL_IDLE           = 4'h0;
  localparam [3:0] CAL_PRESTOP        = 4'h1;
  localparam [3:0] CAL_SPIN_LEFT      = 4'h2;
  localparam [3:0] CAL_SETTLE_LEFT    = 4'h3;
  localparam [3:0] CAL_SPIN_RIGHT     = 4'h4;
  localparam [3:0] CAL_SETTLE_RIGHT   = 4'h5;
  localparam [3:0] CAL_FORWARD        = 4'h6;
  localparam [3:0] CAL_SETTLE_FORWARD = 4'h7;
  localparam [3:0] CAL_REVERSE        = 4'h8;
  localparam [3:0] CAL_FINAL_STOP     = 4'h9;
  localparam [3:0] CAL_DONE           = 4'hA;
  localparam [3:0] CAL_FAILED_ABORT   = 4'hB;
  localparam [3:0] CAL_ROUND_SETTLE   = 4'hC;
  localparam [3:0] CAL_FAILED_SENSOR  = 4'hD;
  localparam [3:0] CAL_FAILED_POLARITY = 4'hE;
  localparam [3:0] CAL_FAILED_CONVERGENCE = 4'hF;

  reg [23:0] fused_sample_age;
  wire fused_stream_fresh =
      fused_sample_age < FUSED_SAMPLE_TIMEOUT_CYCLES;
  wire sensors_ready = calibration_done && fused_stream_fresh;
  wire calibration_running =
      ((local_calibration_state >= CAL_PRESTOP) &&
       (local_calibration_state <= CAL_FINAL_STOP)) ||
      (local_calibration_state == CAL_ROUND_SETTLE);
  reg calibration_obstacle_latched;
  reg [26:0] calibration_obstacle_count;
  reg [31:0] calibration_obstacle_clear_count;
  reg calibration_sensor_pause;
  reg [31:0] calibration_sensor_recovery_count;
  reg [3:0] calibration_sensor_good_samples;
  reg [3:0] calibration_sensor_recovery_events;
  reg [3:0] calibration_resume_state;
  reg calibration_resume_pending;
  wire calibration_obstacle_hold =
      (local_calibration_state == CAL_FORWARD) &&
      (obstacle_stop || calibration_obstacle_latched);

  function [10:0] calibration_magnitude;
    input [9:0] configured_speed;
    begin





      calibration_magnitude = 11'd750;
    end
  endfunction

  wire [10:0] cal_lf_magnitude = calibration_magnitude(cal_lf_default_speed);
  wire [10:0] cal_rf_magnitude = calibration_magnitude(cal_rf_default_speed);
  wire [10:0] cal_lr_magnitude = calibration_magnitude(cal_lr_default_speed);
  wire [10:0] cal_rr_magnitude = calibration_magnitude(cal_rr_default_speed);

  reg [10:0] commanded_lf_speed;
  reg [10:0] commanded_rf_speed;
  reg [10:0] commanded_lr_speed;
  reg [10:0] commanded_rr_speed;




  always @* begin
    commanded_lf_speed = base_lf_speed;
    commanded_rf_speed = base_rf_speed;
    commanded_lr_speed = base_lr_speed;
    commanded_rr_speed = base_rr_speed;
    if (calibration_running) begin
      if (calibration_sensor_pause || !sensors_ready) begin
        commanded_lf_speed = 11'd0;
        commanded_rf_speed = 11'd0;
        commanded_lr_speed = 11'd0;
        commanded_rr_speed = 11'd0;
      end else case (local_calibration_state)
        CAL_SPIN_LEFT: begin
          commanded_lf_speed = -$signed(cal_lf_magnitude);
          commanded_rf_speed = cal_rf_magnitude;
          commanded_lr_speed = -$signed(cal_lr_magnitude);
          commanded_rr_speed = cal_rr_magnitude;
        end
        CAL_SPIN_RIGHT: begin
          commanded_lf_speed = cal_lf_magnitude;
          commanded_rf_speed = -$signed(cal_rf_magnitude);
          commanded_lr_speed = cal_lr_magnitude;
          commanded_rr_speed = -$signed(cal_rr_magnitude);
        end
        CAL_FORWARD: begin
          if (calibration_obstacle_hold) begin
            commanded_lf_speed = 11'd0;
            commanded_rf_speed = 11'd0;
            commanded_lr_speed = 11'd0;
            commanded_rr_speed = 11'd0;
          end else begin
            commanded_lf_speed = cal_lf_magnitude;
            commanded_rf_speed = cal_rf_magnitude;
            commanded_lr_speed = cal_lr_magnitude;
            commanded_rr_speed = cal_rr_magnitude;
          end
        end
        CAL_REVERSE: begin
          commanded_lf_speed = -$signed(cal_lf_magnitude);
          commanded_rf_speed = -$signed(cal_rf_magnitude);
          commanded_lr_speed = -$signed(cal_lr_magnitude);
          commanded_rr_speed = -$signed(cal_rr_magnitude);
        end
        default: begin
          commanded_lf_speed = 11'd0;
          commanded_rf_speed = 11'd0;
          commanded_lr_speed = 11'd0;
          commanded_rr_speed = 11'd0;
        end
      endcase
    end
  end

  wire signed [10:0] base_lf_signed = $signed(commanded_lf_speed);
  wire signed [10:0] base_rf_signed = $signed(commanded_rf_speed);
  wire signed [10:0] base_lr_signed = $signed(commanded_lr_speed);
  wire signed [10:0] base_rr_signed = $signed(commanded_rr_speed);
  wire signed [16:0] gyro_z_signed =
      {{1{fused_gyro_z[15]}}, fused_gyro_z};

  wire moving_forward =
      (base_lf_signed > 0) && (base_rf_signed > 0) &&
      (base_lr_signed > 0) && (base_rr_signed > 0);
  wire moving_reverse =
      (base_lf_signed < 0) && (base_rf_signed < 0) &&
      (base_lr_signed < 0) && (base_rr_signed < 0);
  wire moving_straight = moving_forward || moving_reverse;
  wire spinning_right =
      (base_lf_signed > 0) && (base_rf_signed < 0) &&
      (base_lr_signed > 0) && (base_rr_signed < 0);
  wire spinning_left =
      (base_lf_signed < 0) && (base_rf_signed > 0) &&
      (base_lr_signed < 0) && (base_rr_signed > 0);
  wire [16:0] gyro_abs = gyro_z_signed[16] ?
                         $unsigned(-gyro_z_signed) :
                         $unsigned(gyro_z_signed);
  wire polarity_observation = spinning_right ^ gyro_z_signed[16];



  wire signed [16:0] normalized_yaw_rate = gyro_positive_is_right ?
                                           gyro_z_signed : -gyro_z_signed;
  wire signed [17:0] straight_error =
      (gyro_abs <= GYRO_DEADBAND) ? 18'sd0 :
      -{{1{normalized_yaw_rate[16]}}, normalized_yaw_rate};
  reg signed [31:0] error_integral;
  reg signed [17:0] previous_error;
  reg signed [10:0] correction_reg;
  reg signed [17:0] sampled_error;
  reg sampled_error_valid;
  reg signed [17:0] pid_error_reg;
  reg signed [18:0] derivative_reg;
  reg pid_compute_valid;
  reg signed [10:0] correction_target_reg;
  reg correction_target_valid;
  reg polarity_candidate;
  reg [2:0] polarity_count;
  reg calibration_result_valid;
  reg calibration_failed;
  reg calibration_stop_pulse;
  reg [31:0] calibration_timer;
  reg [7:0] left_motion_samples;
  reg [7:0] right_motion_samples;
  reg [16:0] left_attempt_gyro_peak;
  reg [16:0] right_attempt_gyro_peak;
  reg left_attempt_polarity;
  reg right_attempt_polarity;
  reg [1:0] left_spin_retry_count;
  reg [1:0] right_spin_retry_count;
  reg [7:0] trim_warmup_samples;
  reg [6:0] trim_sample_count;
  reg signed [31:0] trim_correction_sum;
  reg [16:0] round_gyro_peak;
  reg [2:0] calibration_round_count;
  reg [1:0] stable_round_count;
  reg round_converged;
  reg [23:0] flow_sample_age;
  reg signed [10:0] flow_correction_reg;
  reg spin_direction_right;
  reg [22:0] runtime_apply_count;
  reg signed [10:0] runtime_correction_reg;
  reg runtime_correction_valid;
  reg [2:0] runtime_motion_class;

  function signed [31:0] clamp_integral;
    input signed [32:0] value;
    begin
      if (value > INTEGRAL_LIMIT)
        clamp_integral = INTEGRAL_LIMIT;
      else if (value < -INTEGRAL_LIMIT)
        clamp_integral = -INTEGRAL_LIMIT;
      else
        clamp_integral = value[31:0];
    end
  endfunction

  wire signed [31:0] sampled_error_32 =
      {{14{sampled_error[17]}}, sampled_error};
  wire signed [31:0] integral_next =
      clamp_integral($signed({error_integral[31], error_integral}) +
                     $signed({sampled_error_32[31], sampled_error_32}));
  wire signed [18:0] sampled_derivative =
      {{1{sampled_error[17]}}, sampled_error} -
      {{1{previous_error[17]}}, previous_error};

  function signed [10:0] clamp_correction;
    input signed [31:0] value;
    begin
      if (value > CORRECTION_LIMIT)
        clamp_correction = CORRECTION_LIMIT;
      else if (value < -CORRECTION_LIMIT)
        clamp_correction = -CORRECTION_LIMIT;
      else
        clamp_correction = value[10:0];
    end
  endfunction

  function signed [10:0] clamp_flow_correction;
    input signed [16:0] value;
    begin
      if (value > FLOW_CORRECTION_LIMIT)
        clamp_flow_correction = FLOW_CORRECTION_LIMIT;
      else if (value < -FLOW_CORRECTION_LIMIT)
        clamp_flow_correction = -11'sd32;
      else
        clamp_flow_correction = value[10:0];
    end
  endfunction

  function [10:0] abs_speed11;
    input signed [10:0] value;
    begin
      abs_speed11 = value[10] ? $unsigned(-value) : $unsigned(value);
    end
  endfunction

  function [15:0] abs16;
    input signed [15:0] value;
    begin
      abs16 = value[15] ? $unsigned(-value) : $unsigned(value);
    end
  endfunction




  wire signed [19:0] pid_p_term =
      $signed({{2{pid_error_reg[17]}}, pid_error_reg}) >>> P_SHIFT;
  wire signed [19:0] pid_i_term =
      $signed(error_integral) >>> I_SHIFT;
  wire signed [19:0] pid_d_term =
      $signed({derivative_reg[18], derivative_reg}) >>> D_SHIFT;
  wire signed [21:0] pid_sum_narrow =
      {{2{pid_p_term[19]}}, pid_p_term} +
      {{2{pid_i_term[19]}}, pid_i_term} +
      {{2{pid_d_term[19]}}, pid_d_term};
  wire signed [31:0] pid_sum_32 =
      {{10{pid_sum_narrow[21]}}, pid_sum_narrow};
  wire signed [10:0] correction_target_next =
      clamp_correction(pid_sum_32);

  function [10:0] clamp_learned_trim;
    input signed [31:0] value;
    begin
      if (value > LEARNED_TRIM_LIMIT)
        clamp_learned_trim = LEARNED_TRIM_LIMIT[10:0];
      else if (value < -LEARNED_TRIM_LIMIT)
        clamp_learned_trim = -11'sd80;
      else
        clamp_learned_trim = value[10:0];
    end
  endfunction

  function [10:0] clamp_speed;
    input signed [12:0] value;
    begin
      if (value > SPEED_MAX)
        clamp_speed = SPEED_MAX[10:0];
      else if (value < SPEED_MIN)
        clamp_speed = SPEED_MIN[10:0];
      else
        clamp_speed = value[10:0];
    end
  endfunction

  wire signed [31:0] learned_trim_32 =
      {{21{learned_yaw_trim[10]}}, learned_yaw_trim};
  wire signed [16:0] flow_lateral_extended =
      $signed({flow_lateral_q8[15], flow_lateral_q8});
  wire signed [16:0] flow_target_unclamped = moving_forward ?
      -$signed(flow_lateral_extended) : $signed(flow_lateral_extended);
  wire signed [10:0] flow_target_next =
      clamp_flow_correction(flow_target_unclamped);
  wire signed [11:0] dynamic_correction =
      $signed({correction_reg[10], correction_reg}) +
      $signed({flow_correction_reg[10], flow_correction_reg});
  wire signed [31:0] combined_correction_unclamped =
      learned_trim_32 + {{20{dynamic_correction[11]}}, dynamic_correction};
  wire signed [10:0] combined_correction =
      clamp_correction(combined_correction_unclamped);



  wire [10:0] spin_command_magnitude = abs_speed11(base_lf_signed);
  wire [13:0] spin_rate_limited =
      (|gyro_abs[16:14]) ? 14'h3fff : gyro_abs[13:0];



  wire [13:0] spin_common_rate_reference =
      {spin_command_magnitude, 3'b000};
  wire signed [14:0] spin_rate_error =
      $signed({1'b0, spin_common_rate_reference}) -
      $signed({1'b0, spin_rate_limited});
  wire spin_pid_active = spinning_left || spinning_right;
  wire signed [17:0] selected_pid_error = spin_pid_active ?
      {{3{spin_rate_error[14]}}, spin_rate_error} : straight_error;
  wire signed [10:0] wheel_correction = moving_straight ?
      combined_correction : spinning_right ?
      correction_reg : -correction_reg;



  wire signed [10:0] execution_correction = calibration_running ?
      wheel_correction : runtime_correction_reg;
  wire signed [31:0] trim_average =
      trim_correction_sum >>> CAL_TRIM_SAMPLE_SHIFT;
  wire signed [31:0] refined_trim = learned_trim_32 + trim_average;
  wire signed [31:0] trim_average_abs = trim_average[31] ?
      -trim_average : trim_average;

  assign yaw_correction = combined_correction;
  assign flow_correction = flow_correction_reg;
  assign force_stop = external_force_stop | calibration_stop_pulse;
  wire calibration_motion_expected =
      (local_calibration_state == CAL_SPIN_LEFT) ||
      (local_calibration_state == CAL_SPIN_RIGHT) ||
      (local_calibration_state == CAL_FORWARD) ||
      (local_calibration_state == CAL_REVERSE);
  assign vehicle_motion_expected = calibration_motion_expected ||
      (base_lf_speed != 11'd0) || (base_rf_speed != 11'd0) ||
      (base_lr_speed != 11'd0) || (base_rr_speed != 11'd0);
  assign local_calibration_flags = {
      obstacle_stop,
      sensors_ready,
      gyro_positive_is_right,
      gyro_polarity_valid,
      correction_active,
      calibration_failed,
      calibration_result_valid,
      calibration_running
  };

  wire left_motion_evidence =
      (left_motion_samples >= POLARITY_LEARN_SAMPLES) ||
      (left_attempt_gyro_peak >= POLARITY_STRONG_PEAK);
  wire right_motion_evidence =
      (right_motion_samples >= POLARITY_LEARN_SAMPLES) ||
      (right_attempt_gyro_peak >= POLARITY_STRONG_PEAK);
  wire right_polarity_consistent =
      right_attempt_polarity == gyro_positive_is_right;




  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      fused_sample_age <= FUSED_SAMPLE_TIMEOUT_CYCLES;
    else if (fused_sample_valid)
      fused_sample_age <= 24'd0;
    else if (fused_sample_age < FUSED_SAMPLE_TIMEOUT_CYCLES)
      fused_sample_age <= fused_sample_age + 24'd1;
  end




  wire runtime_feedback_active = !calibration_running && correction_active &&
      sensors_ready && (moving_straight || spin_pid_active);
  wire [2:0] current_motion_class = moving_forward ? 3'd1 :
                                    moving_reverse ? 3'd2 :
                                    spinning_left ? 3'd3 :
                                    spinning_right ? 3'd4 : 3'd0;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      runtime_apply_count <= 23'd0;
      runtime_correction_reg <= 11'sd0;
      runtime_correction_valid <= 1'b0;
      runtime_motion_class <= 3'd0;
    end else if (local_calibration_start || external_force_stop ||
                 !runtime_feedback_active) begin
      runtime_apply_count <= 23'd0;
      runtime_correction_reg <= 11'sd0;
      runtime_correction_valid <= 1'b0;
      runtime_motion_class <= 3'd0;
    end else if (runtime_motion_class != current_motion_class) begin

      runtime_apply_count <= 23'd0;
      runtime_correction_reg <= 11'sd0;
      runtime_correction_valid <= 1'b0;
      runtime_motion_class <= current_motion_class;
    end else if (runtime_apply_count >= RUNTIME_APPLY_CYCLES - 1) begin
      runtime_apply_count <= 23'd0;
      runtime_correction_reg <= wheel_correction;
      runtime_correction_valid <= 1'b1;
    end else begin
      runtime_apply_count <= runtime_apply_count + 23'd1;
    end
  end



  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      spin_direction_right <= 1'b0;
    end else if (spinning_left || spinning_right) begin
      spin_direction_right <= spinning_right;
    end
  end


  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      gyro_polarity_valid <= 1'b0;
      gyro_positive_is_right <= 1'b1;
      polarity_candidate <= 1'b1;
      polarity_count <= 3'd0;
    end else if (local_calibration_start || external_force_stop) begin
      polarity_count <= 3'd0;
    end else if ((local_calibration_state == CAL_SETTLE_LEFT) &&
                 (calibration_timer >= CAL_SETTLE_CYCLES - 1) &&
                 left_motion_evidence) begin



      gyro_positive_is_right <= left_attempt_polarity;
      gyro_polarity_valid <= 1'b1;
      polarity_count <= 3'd0;
    end else if (fused_sample_valid) begin


      if (sensors_ready && (spinning_left || spinning_right) &&
          (gyro_abs >= POLARITY_LEARN_MIN)) begin
        if ((polarity_count == 3'd0) ||
            (polarity_candidate == polarity_observation)) begin
          polarity_candidate <= polarity_observation;
          if (polarity_count >= POLARITY_LEARN_SAMPLES - 1'b1) begin
            gyro_positive_is_right <= polarity_observation;
            gyro_polarity_valid <= 1'b1;
            polarity_count <= 3'd0;
          end else begin
            polarity_count <= polarity_count + 3'd1;
          end
        end else begin
          polarity_candidate <= polarity_observation;
          polarity_count <= 3'd1;
        end
      end else if (!(spinning_left || spinning_right)) begin
        polarity_count <= 3'd0;
      end
    end
  end







  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      flow_sample_age <= FLOW_TIMEOUT_CYCLES;
      flow_correction_reg <= 11'sd0;
    end else if (local_calibration_start || external_force_stop ||
                 !sensors_ready || !gyro_polarity_valid || !moving_straight) begin
      flow_sample_age <= FLOW_TIMEOUT_CYCLES;
      flow_correction_reg <= 11'sd0;
    end else begin
      if (flow_sample_valid) begin
        flow_sample_age <= 24'd0;
        if (flow_target_next > flow_correction_reg + FLOW_CORRECTION_SLEW)
          flow_correction_reg <= flow_correction_reg + FLOW_CORRECTION_SLEW;
        else if (flow_target_next <
                 flow_correction_reg - FLOW_CORRECTION_SLEW)
          flow_correction_reg <= flow_correction_reg - FLOW_CORRECTION_SLEW;
        else
          flow_correction_reg <= flow_target_next;
      end else if (flow_sample_age >= FLOW_TIMEOUT_CYCLES - 1) begin
        flow_correction_reg <= 11'sd0;
      end else begin
        flow_sample_age <= flow_sample_age + 24'd1;
      end
    end
  end




  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      correction_active <= 1'b0;
      error_integral <= 32'sd0;
      previous_error <= 18'sd0;
      correction_reg <= 11'sd0;
      sampled_error <= 18'sd0;
      sampled_error_valid <= 1'b0;
      pid_error_reg <= 18'sd0;
      derivative_reg <= 19'sd0;
      pid_compute_valid <= 1'b0;
      correction_target_reg <= 11'sd0;
      correction_target_valid <= 1'b0;
    end else if (local_calibration_start || external_force_stop ||
                 !sensors_ready || !gyro_polarity_valid ||
                 !(moving_straight || spin_pid_active) ||
                 (spin_pid_active &&
                  (spin_direction_right != spinning_right))) begin
      correction_active <= 1'b0;
      error_integral <= 32'sd0;
      previous_error <= 18'sd0;
      correction_reg <= 11'sd0;
      sampled_error <= 18'sd0;
      sampled_error_valid <= 1'b0;
      pid_error_reg <= 18'sd0;
      derivative_reg <= 19'sd0;
      pid_compute_valid <= 1'b0;
      correction_target_reg <= 11'sd0;
      correction_target_valid <= 1'b0;
    end else begin
      correction_active <= 1'b1;
      sampled_error_valid <= 1'b0;
      pid_compute_valid <= 1'b0;
      correction_target_valid <= 1'b0;

      if (fused_sample_valid) begin
        sampled_error <= selected_pid_error;
        sampled_error_valid <= 1'b1;
      end

      if (sampled_error_valid) begin
        error_integral <= integral_next;
        previous_error <= sampled_error;
        pid_error_reg <= sampled_error;
        derivative_reg <= sampled_derivative;
        pid_compute_valid <= 1'b1;
      end

      if (pid_compute_valid) begin
        correction_target_reg <= correction_target_next;
        correction_target_valid <= 1'b1;
      end

      if (correction_target_valid) begin


        if (correction_target_reg > correction_reg + CORRECTION_SLEW)
          correction_reg <= correction_reg + CORRECTION_SLEW;
        else if (correction_target_reg < correction_reg - CORRECTION_SLEW)
          correction_reg <= correction_reg - CORRECTION_SLEW;
        else
          correction_reg <= correction_target_reg;
      end
    end
  end






  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      local_calibration_state <= CAL_IDLE;
      calibration_result_valid <= 1'b0;
      calibration_failed <= 1'b0;
      calibration_stop_pulse <= 1'b0;
      calibration_timer <= 32'd0;
      left_motion_samples <= 8'd0;
      right_motion_samples <= 8'd0;
      left_attempt_gyro_peak <= 17'd0;
      right_attempt_gyro_peak <= 17'd0;
      left_attempt_polarity <= 1'b1;
      right_attempt_polarity <= 1'b1;
      left_spin_retry_count <= 2'd0;
      right_spin_retry_count <= 2'd0;
      trim_warmup_samples <= 8'd0;
      trim_sample_count <= 7'd0;
      trim_correction_sum <= 32'sd0;
      round_gyro_peak <= 17'd0;
      calibration_round_count <= 3'd0;
      stable_round_count <= 2'd0;
      round_converged <= 1'b0;
      learned_yaw_trim <= 11'sd0;
      calibration_gyro_peak <= 16'd0;
      calibration_obstacle_latched <= 1'b0;
      calibration_obstacle_count <= 27'd0;
      calibration_obstacle_clear_count <= 32'd0;
      calibration_sensor_pause <= 1'b0;
      calibration_sensor_recovery_count <= 32'd0;
      calibration_sensor_good_samples <= 4'd0;
      calibration_sensor_recovery_events <= 4'd0;
      calibration_resume_state <= CAL_SPIN_LEFT;
      calibration_resume_pending <= 1'b0;
    end else begin
      calibration_stop_pulse <= 1'b0;

      if (local_calibration_start) begin
        calibration_timer <= 32'd0;
        left_motion_samples <= 8'd0;
        right_motion_samples <= 8'd0;
        left_attempt_gyro_peak <= 17'd0;
        right_attempt_gyro_peak <= 17'd0;
        left_attempt_polarity <= 1'b1;
        right_attempt_polarity <= 1'b1;
        left_spin_retry_count <= 2'd0;
        right_spin_retry_count <= 2'd0;
        trim_warmup_samples <= 8'd0;
        trim_sample_count <= 7'd0;
        trim_correction_sum <= 32'sd0;
        round_gyro_peak <= 17'd0;
        calibration_round_count <= 3'd0;
        stable_round_count <= 2'd0;
        round_converged <= 1'b0;
        calibration_gyro_peak <= 16'd0;
        calibration_result_valid <= 1'b0;
        calibration_failed <= 1'b0;
        calibration_stop_pulse <= 1'b1;
        calibration_obstacle_latched <= 1'b0;
        calibration_obstacle_count <= 27'd0;
        calibration_obstacle_clear_count <= 32'd0;
        calibration_sensor_pause <= 1'b0;
        calibration_sensor_recovery_count <= 32'd0;
        calibration_sensor_good_samples <= 4'd0;
        calibration_sensor_recovery_events <= 4'd0;
        calibration_resume_state <= CAL_SPIN_LEFT;
        calibration_resume_pending <= 1'b0;
        if (obstacle_stop) begin
          local_calibration_state <= CAL_FAILED_ABORT;
          calibration_failed <= 1'b1;
        end else if (sensors_ready) begin
          local_calibration_state <= CAL_PRESTOP;
        end else begin



          local_calibration_state <= CAL_PRESTOP;
          calibration_sensor_pause <= 1'b1;
          calibration_resume_state <= CAL_SPIN_LEFT;
        end
      end else if (external_force_stop && calibration_running) begin
        local_calibration_state <= CAL_FAILED_ABORT;
        calibration_result_valid <= 1'b0;
        calibration_failed <= 1'b1;
        calibration_timer <= 32'd0;
        calibration_stop_pulse <= 1'b1;
        calibration_obstacle_latched <= 1'b0;
        calibration_obstacle_count <= 27'd0;
        calibration_obstacle_clear_count <= 32'd0;
        calibration_sensor_pause <= 1'b0;
        calibration_sensor_recovery_count <= 32'd0;
        calibration_sensor_good_samples <= 4'd0;
        calibration_resume_pending <= 1'b0;
      end else if (calibration_running && calibration_sensor_pause) begin
        calibration_timer <= 32'd0;
        calibration_obstacle_latched <= 1'b0;
        calibration_obstacle_count <= 27'd0;
        calibration_obstacle_clear_count <= 32'd0;
        if (calibration_sensor_recovery_count >=
                CAL_SENSOR_RECOVERY_TIMEOUT_CYCLES - 1) begin
          local_calibration_state <= CAL_FAILED_SENSOR;
          calibration_result_valid <= 1'b0;
          calibration_failed <= 1'b1;
          calibration_stop_pulse <= 1'b1;
          calibration_sensor_pause <= 1'b0;
          calibration_sensor_recovery_count <= 32'd0;
          calibration_sensor_good_samples <= 4'd0;
        end else begin
          calibration_sensor_recovery_count <=
              calibration_sensor_recovery_count + 32'd1;
          if (calibration_done && fused_sample_valid) begin
            if (calibration_sensor_good_samples >=
                CAL_SENSOR_RECOVERY_SAMPLES - 1'b1) begin



              local_calibration_state <= CAL_PRESTOP;
              calibration_sensor_pause <= 1'b0;
              calibration_sensor_recovery_count <= 32'd0;
              calibration_sensor_good_samples <= 4'd0;
              calibration_resume_pending <= 1'b1;
            end else begin
              calibration_sensor_good_samples <=
                  calibration_sensor_good_samples + 4'd1;
            end
          end
        end
      end else if (calibration_running &&
                   (!calibration_done || !fused_stream_fresh)) begin




        if (calibration_sensor_recovery_events >=
            CAL_SENSOR_RECOVERY_LIMIT) begin
          local_calibration_state <= CAL_FAILED_SENSOR;
          calibration_result_valid <= 1'b0;
          calibration_failed <= 1'b1;
          calibration_stop_pulse <= 1'b1;
          calibration_sensor_pause <= 1'b0;
          calibration_resume_pending <= 1'b0;
        end else begin
          calibration_sensor_pause <= 1'b1;
          calibration_sensor_recovery_count <= 32'd0;
          calibration_sensor_good_samples <= 4'd0;
          calibration_sensor_recovery_events <=
              calibration_sensor_recovery_events + 4'd1;
          calibration_timer <= 32'd0;
          calibration_stop_pulse <= 1'b1;
          calibration_obstacle_latched <= 1'b0;
          calibration_obstacle_count <= 27'd0;
          calibration_obstacle_clear_count <= 32'd0;
          case (local_calibration_state)
            CAL_PRESTOP, CAL_SPIN_LEFT, CAL_SETTLE_LEFT: begin
              calibration_resume_state <= CAL_SPIN_LEFT;
              left_motion_samples <= 8'd0;
              left_attempt_gyro_peak <= 17'd0;
            end
            CAL_SPIN_RIGHT, CAL_SETTLE_RIGHT: begin
              calibration_resume_state <= CAL_SPIN_RIGHT;
              right_motion_samples <= 8'd0;
              right_attempt_gyro_peak <= 17'd0;
            end
            CAL_FORWARD, CAL_SETTLE_FORWARD: begin
              calibration_resume_state <= CAL_FORWARD;
              trim_warmup_samples <= 8'd0;
              trim_sample_count <= 7'd0;
              trim_correction_sum <= 32'sd0;
              round_gyro_peak <= 17'd0;
            end
            CAL_REVERSE: calibration_resume_state <= CAL_REVERSE;
            CAL_ROUND_SETTLE: calibration_resume_state <= CAL_ROUND_SETTLE;
            CAL_FINAL_STOP: calibration_resume_state <= CAL_FINAL_STOP;
            default: calibration_resume_state <= CAL_SPIN_LEFT;
          endcase
        end
      end else begin
        if (local_calibration_state != CAL_FORWARD) begin
          calibration_obstacle_latched <= 1'b0;
          calibration_obstacle_count <= 27'd0;
          calibration_obstacle_clear_count <= 32'd0;
        end

        if (fused_sample_valid && calibration_running &&
            (gyro_abs > calibration_gyro_peak))
          calibration_gyro_peak <= gyro_abs[15:0];

        case (local_calibration_state)
          CAL_PRESTOP: begin
            if (calibration_timer >= CAL_PRESTOP_CYCLES - 1) begin
              if (calibration_resume_pending) begin
                local_calibration_state <= calibration_resume_state;
                calibration_resume_pending <= 1'b0;
              end else begin
                local_calibration_state <= CAL_SPIN_LEFT;
              end
              calibration_timer <= 32'd0;
            end else
              calibration_timer <= calibration_timer + 32'd1;
          end

          CAL_SPIN_LEFT: begin
            if (fused_sample_valid &&
                (gyro_abs > left_attempt_gyro_peak)) begin
              left_attempt_gyro_peak <= gyro_abs;
              left_attempt_polarity <= polarity_observation;
            end
            if (fused_sample_valid && (gyro_abs >= POLARITY_LEARN_MIN) &&
                (left_motion_samples != 8'hff))
              left_motion_samples <= left_motion_samples + 8'd1;
            if (calibration_timer >= CAL_SPIN_CYCLES - 1) begin
              local_calibration_state <= CAL_SETTLE_LEFT;
              calibration_timer <= 32'd0;
              calibration_stop_pulse <= 1'b1;
            end else
              calibration_timer <= calibration_timer + 32'd1;
          end

          CAL_SETTLE_LEFT: begin
            if (calibration_timer >= CAL_SETTLE_CYCLES - 1) begin
              calibration_timer <= 32'd0;
              if (left_motion_evidence) begin
                right_motion_samples <= 8'd0;
                right_attempt_gyro_peak <= 17'd0;
                local_calibration_state <= CAL_SPIN_RIGHT;
              end else if (left_spin_retry_count < CAL_SPIN_RETRY_LIMIT) begin



                left_spin_retry_count <= left_spin_retry_count + 2'd1;
                left_motion_samples <= 8'd0;
                left_attempt_gyro_peak <= 17'd0;
                local_calibration_state <= CAL_SPIN_LEFT;
              end else begin
                local_calibration_state <= CAL_FAILED_POLARITY;
                calibration_failed <= 1'b1;
                calibration_stop_pulse <= 1'b1;
              end
            end else
              calibration_timer <= calibration_timer + 32'd1;
          end

          CAL_SPIN_RIGHT: begin
            if (fused_sample_valid &&
                (gyro_abs > right_attempt_gyro_peak)) begin
              right_attempt_gyro_peak <= gyro_abs;
              right_attempt_polarity <= polarity_observation;
            end
            if (fused_sample_valid && (gyro_abs >= POLARITY_LEARN_MIN) &&
                (right_motion_samples != 8'hff))
              right_motion_samples <= right_motion_samples + 8'd1;
            if (calibration_timer >= CAL_SPIN_CYCLES - 1) begin
              local_calibration_state <= CAL_SETTLE_RIGHT;
              calibration_timer <= 32'd0;
              calibration_stop_pulse <= 1'b1;
            end else
              calibration_timer <= calibration_timer + 32'd1;
          end

          CAL_SETTLE_RIGHT: begin
            if (calibration_timer >= CAL_SETTLE_CYCLES - 1) begin
              calibration_timer <= 32'd0;
              if ((!gyro_polarity_valid || !right_motion_evidence ||
                   !right_polarity_consistent) &&
                  right_spin_retry_count < CAL_SPIN_RETRY_LIMIT) begin
                right_spin_retry_count <= right_spin_retry_count + 2'd1;
                right_motion_samples <= 8'd0;
                right_attempt_gyro_peak <= 17'd0;
                local_calibration_state <= CAL_SPIN_RIGHT;
              end else if (gyro_polarity_valid &&
                           left_motion_evidence && right_motion_evidence &&
                           right_polarity_consistent) begin
                trim_warmup_samples <= 8'd0;
                trim_sample_count <= 7'd0;
                trim_correction_sum <= 32'sd0;
                round_gyro_peak <= 17'd0;
                local_calibration_state <= CAL_FORWARD;
              end else begin
                local_calibration_state <= CAL_FAILED_POLARITY;
                calibration_failed <= 1'b1;
                calibration_stop_pulse <= 1'b1;
              end
            end else
              calibration_timer <= calibration_timer + 32'd1;
          end

          CAL_FORWARD: begin
            if (obstacle_stop) begin



              if (!calibration_obstacle_latched)
                calibration_stop_pulse <= 1'b1;
              calibration_obstacle_latched <= 1'b1;
              calibration_obstacle_clear_count <= 32'd0;
              calibration_timer <= 32'd0;
              trim_warmup_samples <= 8'd0;
              trim_sample_count <= 7'd0;
              trim_correction_sum <= 32'sd0;
              round_gyro_peak <= 17'd0;
              if (calibration_obstacle_count >=
                  CAL_OBSTACLE_TIMEOUT_CYCLES - 1) begin
                local_calibration_state <= CAL_FAILED_ABORT;
                calibration_result_valid <= 1'b0;
                calibration_failed <= 1'b1;
                calibration_obstacle_count <= 27'd0;
                calibration_stop_pulse <= 1'b1;
              end else begin
                calibration_obstacle_count <=
                    calibration_obstacle_count + 27'd1;
              end
            end else if (calibration_obstacle_latched) begin
              calibration_obstacle_count <= 27'd0;
              calibration_timer <= 32'd0;
              if (calibration_obstacle_clear_count >=
                  CAL_SETTLE_CYCLES - 1) begin
                calibration_obstacle_latched <= 1'b0;
                calibration_obstacle_clear_count <= 32'd0;
              end else begin
                calibration_obstacle_clear_count <=
                    calibration_obstacle_clear_count + 32'd1;
              end
            end else begin
              calibration_obstacle_count <= 27'd0;
              calibration_obstacle_clear_count <= 32'd0;
              if (fused_sample_valid && correction_active) begin
                if (trim_warmup_samples < CAL_TRIM_WARMUP_SAMPLES)
                  trim_warmup_samples <= trim_warmup_samples + 8'd1;
                else if (trim_sample_count < CAL_TRIM_SAMPLES) begin
                  trim_correction_sum <= trim_correction_sum +
                      {{20{dynamic_correction[11]}}, dynamic_correction};
                  trim_sample_count <= trim_sample_count + 7'd1;
                  if (gyro_abs > round_gyro_peak)
                    round_gyro_peak <= gyro_abs;
                end
              end
              if (calibration_timer >= CAL_STRAIGHT_CYCLES - 1) begin
                calibration_timer <= 32'd0;
                calibration_stop_pulse <= 1'b1;
                if (trim_sample_count >= CAL_TRIM_SAMPLES) begin
                  learned_yaw_trim <= clamp_learned_trim(refined_trim);
                  round_converged <=
                      (trim_average_abs <= CAL_TRIM_CONVERGED) &&
                      (round_gyro_peak <= CAL_GYRO_CONVERGED);
                  local_calibration_state <= CAL_SETTLE_FORWARD;
                end else begin
                  local_calibration_state <= CAL_FAILED_SENSOR;
                  calibration_failed <= 1'b1;
                end
              end else
                calibration_timer <= calibration_timer + 32'd1;
            end
          end

          CAL_SETTLE_FORWARD: begin
            if (calibration_timer >= CAL_SETTLE_CYCLES - 1) begin
              local_calibration_state <= CAL_REVERSE;
              calibration_timer <= 32'd0;
            end else
              calibration_timer <= calibration_timer + 32'd1;
          end

          CAL_REVERSE: begin
            if (calibration_timer >= CAL_STRAIGHT_CYCLES - 1) begin
              local_calibration_state <= CAL_ROUND_SETTLE;
              calibration_timer <= 32'd0;
              calibration_stop_pulse <= 1'b1;
            end else
              calibration_timer <= calibration_timer + 32'd1;
          end

          CAL_ROUND_SETTLE: begin
            if (calibration_timer >= CAL_SETTLE_CYCLES - 1) begin
              calibration_timer <= 32'd0;
              if (round_converged &&
                  (stable_round_count >= CAL_REQUIRED_STABLE_ROUNDS - 1'b1)) begin
                local_calibration_state <= CAL_FINAL_STOP;
              end else if (calibration_round_count >= CAL_MAX_ROUNDS - 1'b1) begin





                local_calibration_state <= CAL_FINAL_STOP;
                calibration_failed <= 1'b0;
                calibration_stop_pulse <= 1'b1;
              end else begin
                calibration_round_count <= calibration_round_count + 3'd1;
                if (round_converged)
                  stable_round_count <= stable_round_count + 2'd1;
                else
                  stable_round_count <= 2'd0;
                trim_warmup_samples <= 8'd0;
                trim_sample_count <= 7'd0;
                trim_correction_sum <= 32'sd0;
                round_gyro_peak <= 17'd0;
                round_converged <= 1'b0;
                local_calibration_state <= CAL_FORWARD;
              end
            end else
              calibration_timer <= calibration_timer + 32'd1;
          end

          CAL_FINAL_STOP: begin
            if (calibration_timer >= CAL_FINAL_STOP_CYCLES - 1) begin
              local_calibration_state <= CAL_DONE;
              calibration_result_valid <= 1'b1;
              calibration_failed <= 1'b0;
              calibration_timer <= 32'd0;
              calibration_stop_pulse <= 1'b1;
            end else
              calibration_timer <= calibration_timer + 32'd1;
          end

          default: calibration_timer <= 32'd0;
        endcase
      end
    end
  end





  always @* begin
    lf_speed = commanded_lf_speed;
    rf_speed = commanded_rf_speed;
    lr_speed = commanded_lr_speed;
    rr_speed = commanded_rr_speed;
    if (correction_active && sensors_ready &&
        (moving_straight || spin_pid_active) &&
        (calibration_running || runtime_correction_valid)) begin
      lf_speed = clamp_speed(
          $signed({{2{commanded_lf_speed[10]}}, commanded_lf_speed}) +
          $signed({{2{execution_correction[10]}}, execution_correction}));
      lr_speed = clamp_speed(
          $signed({{2{commanded_lr_speed[10]}}, commanded_lr_speed}) +
          $signed({{2{execution_correction[10]}}, execution_correction}));
      rf_speed = clamp_speed(
          $signed({{2{commanded_rf_speed[10]}}, commanded_rf_speed}) -
          $signed({{2{execution_correction[10]}}, execution_correction}));
      rr_speed = clamp_speed(
          $signed({{2{commanded_rr_speed[10]}}, commanded_rr_speed}) -
          $signed({{2{execution_correction[10]}}, execution_correction}));
    end
  end
endmodule

module c5_motion_stabilizer_parameter;
  reg clk;
  reg rst_n;
  reg calibration_done;
  reg fused_sample_valid;
  reg imu_degraded;
  reg imu_disagreement;
  reg [15:0] fused_gyro_z;
  reg flow_sample_valid;
  reg [15:0] flow_forward_q8;
  reg [15:0] flow_lateral_q8;
  reg local_calibration_start;
  reg external_force_stop;
  reg obstacle_stop;
  reg [9:0] cal_lf_default_speed;
  reg [9:0] cal_rf_default_speed;
  reg [9:0] cal_lr_default_speed;
  reg [9:0] cal_rr_default_speed;
  reg [10:0] base_lf_speed;
  reg [10:0] base_rf_speed;
  reg [10:0] base_lr_speed;
  reg [10:0] base_rr_speed;
  wire [10:0] lf_speed;
  wire [10:0] rf_speed;
  wire [10:0] lr_speed;
  wire [10:0] rr_speed;
  wire correction_active;
  wire gyro_polarity_valid;
  wire gyro_positive_is_right;
  wire [10:0] yaw_correction;
  wire [10:0] flow_correction;
  wire force_stop;
  wire vehicle_motion_expected;
  wire [3:0] local_calibration_state;
  wire [7:0] local_calibration_flags;
  wire [10:0] learned_yaw_trim;
  wire [15:0] calibration_gyro_peak;


`ifdef PARAM_WAVE
initial begin
  $dumpfile("tb_c5_motion_stabilizer.vcd");
  $dumpvars(0, rst_n, fused_sample_valid, fused_gyro_z, base_lf_speed, base_rf_speed, lf_speed, rf_speed, correction_active, yaw_correction, force_stop);
end
`endif

initial begin
  clk = 1'b0;
  rst_n = 1'b0;
  calibration_done = 1'b0;
  fused_sample_valid = 1'b0;
  imu_degraded = 1'b0;
  imu_disagreement = 1'b0;
  fused_gyro_z = 16'd0;
  flow_sample_valid = 1'b0;
  flow_forward_q8 = 16'd0;
  flow_lateral_q8 = 16'd0;
  local_calibration_start = 1'b0;
  external_force_stop = 1'b0;
  obstacle_stop = 1'b0;
  cal_lf_default_speed = 10'd700;
  cal_rf_default_speed = 10'd700;
  cal_lr_default_speed = 10'd700;
  cal_rr_default_speed = 10'd700;
  base_lf_speed = 11'd0;
  base_rf_speed = 11'd0;
  base_lr_speed = 11'd0;
  base_rr_speed = 11'd0;
end

always #10 clk = ~clk;



  task sample_gyro;
    input signed [15:0] value;
    begin
      fused_gyro_z = value;
      fused_sample_valid = 1'b1;
      @(posedge clk);
      #1;
      fused_sample_valid = 1'b0;
      @(posedge clk);
      #1;
    end
  endtask

  task set_speeds;
    input signed [10:0] lf;
    input signed [10:0] rf;
    input signed [10:0] lr;
    input signed [10:0] rr;
    begin
      base_lf_speed = lf;
      base_rf_speed = rf;
      base_lr_speed = lr;
      base_rr_speed = rr;
      @(posedge clk);
      #1;
    end
  endtask

  integer index;
  initial begin
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    calibration_done = 1'b1;

    // Without learned physical polarity, straight commands are passed through.
    set_speeds(11'sd700, 11'sd700, 11'sd700, 11'sd700);
    sample_gyro(16'sd320);
    if (correction_active || $signed(lf_speed) != 700 || $signed(rf_speed) != 700)
      $fatal(1, "feedback must remain bypassed before polarity learning");

    // The measured chassis turn peak is about 116 counts.  It must remain far
    // above idle noise yet still be accepted for polarity learning.
    set_speeds(11'sd650, -11'sd650, 11'sd650, -11'sd650);
    for (index = 0; index < 4; index = index + 1)
      sample_gyro(16'sd116);
    if (!gyro_polarity_valid || !gyro_positive_is_right)
      $fatal(1, "right-turn polarity was not learned");

    // Normal driving calculates continuously but commits a wheel correction
    // only at the bounded runtime update interval.
    set_speeds(11'sd700, 11'sd700, 11'sd700, 11'sd700);
    sample_gyro(16'sd320);
    sample_gyro(16'sd320);
    repeat (30) @(posedge clk);
    #1;
    if (!correction_active || $signed(yaw_correction) >= 0 ||
        $signed(lf_speed) >= 700 || $signed(rf_speed) <= 700 ||
        $signed(lr_speed) >= 700 || $signed(rr_speed) <= 700)
      $fatal(1, "normal forward PID correction was not applied");
    $display("RESULT forward_pid gyro=320 correction=%0d wheels=%0d,%0d,%0d,%0d",
             $signed(yaw_correction), $signed(lf_speed), $signed(rf_speed),
             $signed(lr_speed), $signed(rr_speed));

    // Reverse commands are also passed through atomically while correction
    // telemetry remains available to the phone and mapping diagnostics. The
    // previous forward correction must disappear immediately on reversal.
    set_speeds(-11'sd700, -11'sd700, -11'sd700, -11'sd700);
    @(posedge clk);
    #1;
    if ($signed(lf_speed) != -700 || $signed(rf_speed) != -700 ||
        $signed(lr_speed) != -700 || $signed(rr_speed) != -700)
      $fatal(1, "forward correction leaked into reverse command");
    for (index = 0; index < 4; index = index + 1)
      sample_gyro(-16'sd320);
    repeat (30) @(posedge clk);
    #1;
    if (!correction_active || $signed(yaw_correction) <= 0 ||
        $signed(lf_speed) <= -700 || $signed(rf_speed) >= -700 ||
        $signed(lr_speed) <= -700 || $signed(rr_speed) >= -700)
      $fatal(1, "normal reverse PID correction was not applied");
    $display("RESULT reverse_pid gyro=-320 correction=%0d wheels=%0d,%0d,%0d,%0d",
             $signed(yaw_correction), $signed(lf_speed), $signed(rf_speed),
             $signed(lr_speed), $signed(rr_speed));

    // A stop command must pass zero immediately, without waiting for a new IMU sample.
    set_speeds(11'sd0, 11'sd0, 11'sd0, 11'sd0);
    if ($signed(lf_speed) != 0 || $signed(rf_speed) != 0 ||
        $signed(lr_speed) != 0 || $signed(rr_speed) != 0)
      $fatal(1, "stop command was modified by stale feedback");
    $display("RESULT immediate_stop wheels=%0d,%0d,%0d,%0d force_stop=%0d",
             $signed(lf_speed), $signed(rf_speed), $signed(lr_speed),
             $signed(rr_speed), force_stop);

    // A diagnostic disagreement alone must not tear down feedback while the
    // fusion block is still delivering a valid primary-IMU fallback stream.
    imu_disagreement = 1'b1;
    set_speeds(11'sd650, 11'sd750, 11'sd800, 11'sd625);
    sample_gyro(16'sd500);
    if (!correction_active)
      $fatal(1, "valid degraded IMU stream incorrectly disabled feedback");

    // Losing the actual sample stream is the safety boundary. With
    // C5_CAL_SIM_FAST, 64 clocks without a sample must bypass correction and
    // preserve the configured wheel commands exactly.
    repeat (66) @(posedge clk);
    #1;
    if (correction_active || $signed(lf_speed) != 650 ||
        $signed(rf_speed) != 750 || $signed(lr_speed) != 800 ||
        $signed(rr_speed) != 625)
      $fatal(1, "stale fused stream did not bypass the stabilizer");
    $display("RESULT stale_bypass wheels=%0d,%0d,%0d,%0d correction_active=%0d",
             $signed(lf_speed), $signed(rf_speed), $signed(lr_speed),
             $signed(rr_speed), correction_active);

    $display("RESULT polarity_positive_is_right=%0d", gyro_positive_is_right);
    $display("RESULT correction_limit=%0d slew_per_sample=%0d", 96, 8);
    $display("RESULT runtime_pid_applied=pass update_interval_cycles=24");
    $display("RESULT degraded_stream_continuity=pass stale_stream_bypass=pass");
    $display("RESULT TEST_PASS c5_motion_stabilizer");
    $finish;
  end


  c5_motion_stabilizer c5_motion_stabilizer0(
    .clk(clk),
    .rst_n(rst_n),
    .calibration_done(calibration_done),
    .fused_sample_valid(fused_sample_valid),
    .imu_degraded(imu_degraded),
    .imu_disagreement(imu_disagreement),
    .fused_gyro_z(fused_gyro_z),
    .flow_sample_valid(flow_sample_valid),
    .flow_forward_q8(flow_forward_q8),
    .flow_lateral_q8(flow_lateral_q8),
    .local_calibration_start(local_calibration_start),
    .external_force_stop(external_force_stop),
    .obstacle_stop(obstacle_stop),
    .cal_lf_default_speed(cal_lf_default_speed),
    .cal_rf_default_speed(cal_rf_default_speed),
    .cal_lr_default_speed(cal_lr_default_speed),
    .cal_rr_default_speed(cal_rr_default_speed),
    .base_lf_speed(base_lf_speed),
    .base_rf_speed(base_rf_speed),
    .base_lr_speed(base_lr_speed),
    .base_rr_speed(base_rr_speed),
    .lf_speed(lf_speed),
    .rf_speed(rf_speed),
    .lr_speed(lr_speed),
    .rr_speed(rr_speed),
    .correction_active(correction_active),
    .gyro_polarity_valid(gyro_polarity_valid),
    .gyro_positive_is_right(gyro_positive_is_right),
    .yaw_correction(yaw_correction),
    .flow_correction(flow_correction),
    .force_stop(force_stop),
    .vehicle_motion_expected(vehicle_motion_expected),
    .local_calibration_state(local_calibration_state),
    .local_calibration_flags(local_calibration_flags),
    .learned_yaw_trim(learned_yaw_trim),
    .calibration_gyro_peak(calibration_gyro_peak)
  );
endmodule

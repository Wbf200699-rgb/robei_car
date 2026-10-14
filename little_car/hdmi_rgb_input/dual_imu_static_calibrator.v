module dual_imu_static_calibrator (
  input wire clk,
  input wire rst_n,
  input wire start,
  input wire sample_valid,
  input wire imu0_online,
  input wire imu1_online,
  input wire signed [15:0] imu0_accel_x,
  input wire signed [15:0] imu0_accel_y,
  input wire signed [15:0] imu0_accel_z,
  input wire signed [15:0] imu0_gyro_x,
  input wire signed [15:0] imu0_gyro_y,
  input wire signed [15:0] imu0_gyro_z,
  input wire signed [15:0] imu1_accel_x,
  input wire signed [15:0] imu1_accel_y,
  input wire signed [15:0] imu1_accel_z,
  input wire signed [15:0] imu1_gyro_x,
  input wire signed [15:0] imu1_gyro_y,
  input wire signed [15:0] imu1_gyro_z,
  output reg busy,
  output reg done,
  output reg pass,
  output reg [7:0] result_flags,
  output reg [1:0] imu0_vertical_axis,
  output reg imu0_vertical_positive,
  output reg [1:0] imu1_vertical_axis,
  output reg imu1_vertical_positive,
  output reg signed [15:0] imu0_accel_mean_x,
  output reg signed [15:0] imu0_accel_mean_y,
  output reg signed [15:0] imu0_accel_mean_z,
  output reg signed [15:0] imu0_gyro_mean_x,
  output reg signed [15:0] imu0_gyro_mean_y,
  output reg signed [15:0] imu0_gyro_mean_z,
  output reg signed [15:0] imu1_accel_mean_x,
  output reg signed [15:0] imu1_accel_mean_y,
  output reg signed [15:0] imu1_accel_mean_z,
  output reg signed [15:0] imu1_gyro_mean_x,
  output reg signed [15:0] imu1_gyro_mean_y,
  output reg signed [15:0] imu1_gyro_mean_z,
  output reg [15:0] imu0_gyro_peak,
  output reg [15:0] imu1_gyro_peak
);

`ifdef DUAL_IMU_CAL_SIM_FAST
  localparam integer SAMPLE_COUNT = 8;
  localparam integer SAMPLE_SHIFT = 3;
`else
  localparam integer SAMPLE_COUNT = 256;
  localparam integer SAMPLE_SHIFT = 8;
`endif

  // BMI088 is configured for +/-6 g, so one gravity is about 5460 counts.
  // These limits tolerate mounting tilt while rejecting a floating bus or a
  // clipped accelerometer. Gyro input is already corrected by the startup
  // bias estimator in motion_sensor_subsystem.
  localparam [15:0] GRAVITY_MIN = 16'd4000;
  localparam [15:0] GRAVITY_MAX = 16'd7000;
  localparam [15:0] SECONDARY_ACCEL_MAX = 16'd2500;
  localparam [15:0] GYRO_MEAN_MAX = 16'd128;
  localparam [15:0] GYRO_PEAK_MAX = 16'd500;

  localparam [1:0] STATE_IDLE = 2'd0;
  localparam [1:0] STATE_COLLECT = 2'd1;
  localparam [1:0] STATE_EVALUATE = 2'd2;

  reg [1:0] state;
  reg [7:0] sample_count;
  reg all_samples_online;

  reg signed [31:0] imu0_accel_sum_x;
  reg signed [31:0] imu0_accel_sum_y;
  reg signed [31:0] imu0_accel_sum_z;
  reg signed [31:0] imu0_gyro_sum_x;
  reg signed [31:0] imu0_gyro_sum_y;
  reg signed [31:0] imu0_gyro_sum_z;
  reg signed [31:0] imu1_accel_sum_x;
  reg signed [31:0] imu1_accel_sum_y;
  reg signed [31:0] imu1_accel_sum_z;
  reg signed [31:0] imu1_gyro_sum_x;
  reg signed [31:0] imu1_gyro_sum_y;
  reg signed [31:0] imu1_gyro_sum_z;

  function [15:0] abs16;
    input signed [15:0] value;
    begin
      abs16 = value[15] ? $unsigned(-value) : $unsigned(value);
    end
  endfunction

  function signed [31:0] sign_extend16;
    input signed [15:0] value;
    begin
      sign_extend16 = {{16{value[15]}}, value};
    end
  endfunction

  function [15:0] max3_abs;
    input signed [15:0] x;
    input signed [15:0] y;
    input signed [15:0] z;
    reg [15:0] xy;
    begin
      xy = (abs16(x) >= abs16(y)) ? abs16(x) : abs16(y);
      max3_abs = (xy >= abs16(z)) ? xy : abs16(z);
    end
  endfunction

  function [1:0] dominant_axis;
    input signed [15:0] x;
    input signed [15:0] y;
    input signed [15:0] z;
    begin
      if ((abs16(x) >= abs16(y)) && (abs16(x) >= abs16(z)))
        dominant_axis = 2'd0;
      else if (abs16(y) >= abs16(z))
        dominant_axis = 2'd1;
      else
        dominant_axis = 2'd2;
    end
  endfunction

  function [15:0] selected_abs;
    input [1:0] axis;
    input signed [15:0] x;
    input signed [15:0] y;
    input signed [15:0] z;
    begin
      case (axis)
        2'd0: selected_abs = abs16(x);
        2'd1: selected_abs = abs16(y);
        default: selected_abs = abs16(z);
      endcase
    end
  endfunction

  function selected_positive;
    input [1:0] axis;
    input signed [15:0] x;
    input signed [15:0] y;
    input signed [15:0] z;
    begin
      case (axis)
        2'd0: selected_positive = !x[15];
        2'd1: selected_positive = !y[15];
        default: selected_positive = !z[15];
      endcase
    end
  endfunction

  function [15:0] largest_nonvertical_abs;
    input [1:0] axis;
    input signed [15:0] x;
    input signed [15:0] y;
    input signed [15:0] z;
    begin
      case (axis)
        2'd0: largest_nonvertical_abs = (abs16(y) >= abs16(z)) ? abs16(y) : abs16(z);
        2'd1: largest_nonvertical_abs = (abs16(x) >= abs16(z)) ? abs16(x) : abs16(z);
        default: largest_nonvertical_abs = (abs16(x) >= abs16(y)) ? abs16(x) : abs16(y);
      endcase
    end
  endfunction

  wire [1:0] imu0_axis_eval = dominant_axis(
      imu0_accel_mean_x, imu0_accel_mean_y, imu0_accel_mean_z);
  wire [1:0] imu1_axis_eval = dominant_axis(
      imu1_accel_mean_x, imu1_accel_mean_y, imu1_accel_mean_z);
  wire [15:0] imu0_gravity_eval = selected_abs(
      imu0_axis_eval, imu0_accel_mean_x, imu0_accel_mean_y, imu0_accel_mean_z);
  wire [15:0] imu1_gravity_eval = selected_abs(
      imu1_axis_eval, imu1_accel_mean_x, imu1_accel_mean_y, imu1_accel_mean_z);
  wire [15:0] imu0_secondary_eval = largest_nonvertical_abs(
      imu0_axis_eval, imu0_accel_mean_x, imu0_accel_mean_y, imu0_accel_mean_z);
  wire [15:0] imu1_secondary_eval = largest_nonvertical_abs(
      imu1_axis_eval, imu1_accel_mean_x, imu1_accel_mean_y, imu1_accel_mean_z);
  wire imu0_gravity_valid = (imu0_gravity_eval >= GRAVITY_MIN) &&
                            (imu0_gravity_eval <= GRAVITY_MAX) &&
                            (imu0_secondary_eval <= SECONDARY_ACCEL_MAX);
  wire imu1_gravity_valid = (imu1_gravity_eval >= GRAVITY_MIN) &&
                            (imu1_gravity_eval <= GRAVITY_MAX) &&
                            (imu1_secondary_eval <= SECONDARY_ACCEL_MAX);
  wire imu0_gyro_quiet = (abs16(imu0_gyro_mean_x) <= GYRO_MEAN_MAX) &&
                         (abs16(imu0_gyro_mean_y) <= GYRO_MEAN_MAX) &&
                         (abs16(imu0_gyro_mean_z) <= GYRO_MEAN_MAX) &&
                         (imu0_gyro_peak <= GYRO_PEAK_MAX);
  wire imu1_gyro_quiet = (abs16(imu1_gyro_mean_x) <= GYRO_MEAN_MAX) &&
                         (abs16(imu1_gyro_mean_y) <= GYRO_MEAN_MAX) &&
                         (abs16(imu1_gyro_mean_z) <= GYRO_MEAN_MAX) &&
                         (imu1_gyro_peak <= GYRO_PEAK_MAX);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= STATE_IDLE;
      sample_count <= 8'd0;
      all_samples_online <= 1'b0;
      busy <= 1'b0;
      done <= 1'b0;
      pass <= 1'b0;
      result_flags <= 8'd0;
      imu0_vertical_axis <= 2'd0;
      imu0_vertical_positive <= 1'b0;
      imu1_vertical_axis <= 2'd0;
      imu1_vertical_positive <= 1'b0;
      imu0_accel_mean_x <= 16'sd0;
      imu0_accel_mean_y <= 16'sd0;
      imu0_accel_mean_z <= 16'sd0;
      imu0_gyro_mean_x <= 16'sd0;
      imu0_gyro_mean_y <= 16'sd0;
      imu0_gyro_mean_z <= 16'sd0;
      imu1_accel_mean_x <= 16'sd0;
      imu1_accel_mean_y <= 16'sd0;
      imu1_accel_mean_z <= 16'sd0;
      imu1_gyro_mean_x <= 16'sd0;
      imu1_gyro_mean_y <= 16'sd0;
      imu1_gyro_mean_z <= 16'sd0;
      imu0_gyro_peak <= 16'd0;
      imu1_gyro_peak <= 16'd0;
      imu0_accel_sum_x <= 32'sd0;
      imu0_accel_sum_y <= 32'sd0;
      imu0_accel_sum_z <= 32'sd0;
      imu0_gyro_sum_x <= 32'sd0;
      imu0_gyro_sum_y <= 32'sd0;
      imu0_gyro_sum_z <= 32'sd0;
      imu1_accel_sum_x <= 32'sd0;
      imu1_accel_sum_y <= 32'sd0;
      imu1_accel_sum_z <= 32'sd0;
      imu1_gyro_sum_x <= 32'sd0;
      imu1_gyro_sum_y <= 32'sd0;
      imu1_gyro_sum_z <= 32'sd0;
    end else begin
      if (start) begin
        state <= STATE_COLLECT;
        sample_count <= 8'd0;
        all_samples_online <= imu0_online && imu1_online;
        busy <= 1'b1;
        done <= 1'b0;
        pass <= 1'b0;
        result_flags <= 8'd0;
        imu0_gyro_peak <= 16'd0;
        imu1_gyro_peak <= 16'd0;
        imu0_accel_sum_x <= 32'sd0;
        imu0_accel_sum_y <= 32'sd0;
        imu0_accel_sum_z <= 32'sd0;
        imu0_gyro_sum_x <= 32'sd0;
        imu0_gyro_sum_y <= 32'sd0;
        imu0_gyro_sum_z <= 32'sd0;
        imu1_accel_sum_x <= 32'sd0;
        imu1_accel_sum_y <= 32'sd0;
        imu1_accel_sum_z <= 32'sd0;
        imu1_gyro_sum_x <= 32'sd0;
        imu1_gyro_sum_y <= 32'sd0;
        imu1_gyro_sum_z <= 32'sd0;
      end else if ((state == STATE_COLLECT) && sample_valid) begin
        if (!imu0_online || !imu1_online)
          all_samples_online <= 1'b0;
        imu0_accel_sum_x <= imu0_accel_sum_x + sign_extend16(imu0_accel_x);
        imu0_accel_sum_y <= imu0_accel_sum_y + sign_extend16(imu0_accel_y);
        imu0_accel_sum_z <= imu0_accel_sum_z + sign_extend16(imu0_accel_z);
        imu0_gyro_sum_x <= imu0_gyro_sum_x + sign_extend16(imu0_gyro_x);
        imu0_gyro_sum_y <= imu0_gyro_sum_y + sign_extend16(imu0_gyro_y);
        imu0_gyro_sum_z <= imu0_gyro_sum_z + sign_extend16(imu0_gyro_z);
        imu1_accel_sum_x <= imu1_accel_sum_x + sign_extend16(imu1_accel_x);
        imu1_accel_sum_y <= imu1_accel_sum_y + sign_extend16(imu1_accel_y);
        imu1_accel_sum_z <= imu1_accel_sum_z + sign_extend16(imu1_accel_z);
        imu1_gyro_sum_x <= imu1_gyro_sum_x + sign_extend16(imu1_gyro_x);
        imu1_gyro_sum_y <= imu1_gyro_sum_y + sign_extend16(imu1_gyro_y);
        imu1_gyro_sum_z <= imu1_gyro_sum_z + sign_extend16(imu1_gyro_z);
        if (max3_abs(imu0_gyro_x, imu0_gyro_y, imu0_gyro_z) > imu0_gyro_peak)
          imu0_gyro_peak <= max3_abs(imu0_gyro_x, imu0_gyro_y, imu0_gyro_z);
        if (max3_abs(imu1_gyro_x, imu1_gyro_y, imu1_gyro_z) > imu1_gyro_peak)
          imu1_gyro_peak <= max3_abs(imu1_gyro_x, imu1_gyro_y, imu1_gyro_z);

        if (sample_count >= SAMPLE_COUNT - 1) begin
          imu0_accel_mean_x <= (imu0_accel_sum_x + sign_extend16(imu0_accel_x)) >>> SAMPLE_SHIFT;
          imu0_accel_mean_y <= (imu0_accel_sum_y + sign_extend16(imu0_accel_y)) >>> SAMPLE_SHIFT;
          imu0_accel_mean_z <= (imu0_accel_sum_z + sign_extend16(imu0_accel_z)) >>> SAMPLE_SHIFT;
          imu0_gyro_mean_x <= (imu0_gyro_sum_x + sign_extend16(imu0_gyro_x)) >>> SAMPLE_SHIFT;
          imu0_gyro_mean_y <= (imu0_gyro_sum_y + sign_extend16(imu0_gyro_y)) >>> SAMPLE_SHIFT;
          imu0_gyro_mean_z <= (imu0_gyro_sum_z + sign_extend16(imu0_gyro_z)) >>> SAMPLE_SHIFT;
          imu1_accel_mean_x <= (imu1_accel_sum_x + sign_extend16(imu1_accel_x)) >>> SAMPLE_SHIFT;
          imu1_accel_mean_y <= (imu1_accel_sum_y + sign_extend16(imu1_accel_y)) >>> SAMPLE_SHIFT;
          imu1_accel_mean_z <= (imu1_accel_sum_z + sign_extend16(imu1_accel_z)) >>> SAMPLE_SHIFT;
          imu1_gyro_mean_x <= (imu1_gyro_sum_x + sign_extend16(imu1_gyro_x)) >>> SAMPLE_SHIFT;
          imu1_gyro_mean_y <= (imu1_gyro_sum_y + sign_extend16(imu1_gyro_y)) >>> SAMPLE_SHIFT;
          imu1_gyro_mean_z <= (imu1_gyro_sum_z + sign_extend16(imu1_gyro_z)) >>> SAMPLE_SHIFT;
          busy <= 1'b0;
          state <= STATE_EVALUATE;
        end else begin
          sample_count <= sample_count + 8'd1;
        end
      end else if (state == STATE_EVALUATE) begin
        imu0_vertical_axis <= imu0_axis_eval;
        imu0_vertical_positive <= selected_positive(
            imu0_axis_eval, imu0_accel_mean_x, imu0_accel_mean_y, imu0_accel_mean_z);
        imu1_vertical_axis <= imu1_axis_eval;
        imu1_vertical_positive <= selected_positive(
            imu1_axis_eval, imu1_accel_mean_x, imu1_accel_mean_y, imu1_accel_mean_z);
        pass <= all_samples_online && imu0_gravity_valid &&
                imu1_gravity_valid && imu0_gyro_quiet && imu1_gyro_quiet;
        result_flags <= {
            (imu0_axis_eval == imu1_axis_eval),
            imu1_gyro_quiet,
            imu0_gyro_quiet,
            imu1_gravity_valid,
            imu0_gravity_valid,
            all_samples_online,
            (all_samples_online && imu0_gravity_valid && imu1_gravity_valid &&
             imu0_gyro_quiet && imu1_gyro_quiet),
            1'b1
        };
        done <= 1'b1;
        state <= STATE_IDLE;
      end
    end
  end
endmodule

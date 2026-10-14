`timescale 1ns/1ps
module dual_imu_static_calibrator(
  input wire clk,
  input wire rst_n,
  input wire start,
  input wire sample_valid,
  input wire imu0_online,
  input wire imu1_online,
  input wire [15:0] imu0_accel_x,
  input wire [15:0] imu0_accel_y,
  input wire [15:0] imu0_accel_z,
  input wire [15:0] imu0_gyro_x,
  input wire [15:0] imu0_gyro_y,
  input wire [15:0] imu0_gyro_z,
  input wire [15:0] imu1_accel_x,
  input wire [15:0] imu1_accel_y,
  input wire [15:0] imu1_accel_z,
  input wire [15:0] imu1_gyro_x,
  input wire [15:0] imu1_gyro_y,
  input wire [15:0] imu1_gyro_z,
  output reg busy,
  output reg done,
  output reg pass,
  output reg [7:0] result_flags,
  output reg [1:0] imu0_vertical_axis,
  output reg imu0_vertical_positive,
  output reg [1:0] imu1_vertical_axis,
  output reg imu1_vertical_positive,
  output reg [15:0] imu0_accel_mean_x,
  output reg [15:0] imu0_accel_mean_y,
  output reg [15:0] imu0_accel_mean_z,
  output reg [15:0] imu0_gyro_mean_x,
  output reg [15:0] imu0_gyro_mean_y,
  output reg [15:0] imu0_gyro_mean_z,
  output reg [15:0] imu1_accel_mean_x,
  output reg [15:0] imu1_accel_mean_y,
  output reg [15:0] imu1_accel_mean_z,
  output reg [15:0] imu1_gyro_mean_x,
  output reg [15:0] imu1_gyro_mean_y,
  output reg [15:0] imu1_gyro_mean_z,
  output reg [15:0] imu0_gyro_peak,
  output reg [15:0] imu1_gyro_peak
);
`define DUAL_IMU_CAL_SIM_FAST
`ifdef DUAL_IMU_CAL_SIM_FAST
  localparam integer SAMPLE_COUNT = 8;
  localparam integer SAMPLE_SHIFT = 3;
`else
  localparam integer SAMPLE_COUNT = 256;
  localparam integer SAMPLE_SHIFT = 8;
`endif





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

module dual_imu_calibration_parameter;
  reg clk;
  reg rst_n;
  reg start;
  reg sample_valid;
  reg imu0_online;
  reg imu1_online;
  reg [15:0] imu0_accel_x;
  reg [15:0] imu0_accel_y;
  reg [15:0] imu0_accel_z;
  reg [15:0] imu0_gyro_x;
  reg [15:0] imu0_gyro_y;
  reg [15:0] imu0_gyro_z;
  reg [15:0] imu1_accel_x;
  reg [15:0] imu1_accel_y;
  reg [15:0] imu1_accel_z;
  reg [15:0] imu1_gyro_x;
  reg [15:0] imu1_gyro_y;
  reg [15:0] imu1_gyro_z;
  wire busy;
  wire done;
  wire pass;
  wire [7:0] result_flags;
  wire [1:0] imu0_vertical_axis;
  wire imu0_vertical_positive;
  wire [1:0] imu1_vertical_axis;
  wire imu1_vertical_positive;
  wire [15:0] imu0_accel_mean_x;
  wire [15:0] imu0_accel_mean_y;
  wire [15:0] imu0_accel_mean_z;
  wire [15:0] imu0_gyro_mean_x;
  wire [15:0] imu0_gyro_mean_y;
  wire [15:0] imu0_gyro_mean_z;
  wire [15:0] imu1_accel_mean_x;
  wire [15:0] imu1_accel_mean_y;
  wire [15:0] imu1_accel_mean_z;
  wire [15:0] imu1_gyro_mean_x;
  wire [15:0] imu1_gyro_mean_y;
  wire [15:0] imu1_gyro_mean_z;
  wire [15:0] imu0_gyro_peak;
  wire [15:0] imu1_gyro_peak;


`ifdef PARAM_WAVE
initial begin
  $dumpfile("tb_dual_imu_static_calibrator.vcd");
  $dumpvars(0, rst_n, start, sample_valid, imu0_online, imu1_online, busy, done, pass, result_flags, imu0_gyro_peak, imu1_gyro_peak);
end
`endif

initial begin
  clk = 1'b0;
  rst_n = 1'b0;
  start = 1'b0;
  sample_valid = 1'b0;
  imu0_online = 1'b1;
  imu1_online = 1'b1;
  imu0_accel_x = 16'sd30;
  imu0_accel_y = -16'sd160;
  imu0_accel_z = -16'sd5504;
  imu0_gyro_x = 16'sd2;
  imu0_gyro_y = -16'sd3;
  imu0_gyro_z = 16'sd4;
  imu1_accel_x = 16'sd1176;
  imu1_accel_y = -16'sd5350;
  imu1_accel_z = 16'sd165;
  imu1_gyro_x = 16'sd30;
  imu1_gyro_y = -16'sd1;
  imu1_gyro_z = -16'sd11;
end

always #5 clk = ~clk;

  task pulse_start;
    begin
      @(negedge clk); start = 1'b1;
      @(negedge clk); start = 1'b0;
    end
  endtask

  task send_samples;
    input integer count;
    integer index;
    begin
      for (index = 0; index < count; index = index + 1) begin
        @(negedge clk); sample_valid = 1'b1;
        @(negedge clk); sample_valid = 1'b0;
      end
    end
  endtask

  task send_alternating_gyro_samples;
    input integer count;
    integer index;
    begin
      for (index = 0; index < count; index = index + 1) begin
        imu0_gyro_x = index[0] ? -16'sd10 : 16'sd10;
        imu0_gyro_y = index[0] ? 16'sd12 : -16'sd12;
        imu0_gyro_z = index[0] ? -16'sd8 : 16'sd8;
        imu1_gyro_x = index[0] ? -16'sd14 : 16'sd14;
        imu1_gyro_y = index[0] ? 16'sd6 : -16'sd6;
        imu1_gyro_z = index[0] ? -16'sd4 : 16'sd4;
        @(negedge clk); sample_valid = 1'b1;
        @(negedge clk); sample_valid = 1'b0;
      end
    end
  endtask

  initial begin


    repeat (4) @(posedge clk);
    rst_n = 1'b1;

    pulse_start();
    send_samples(8);
    wait(done);
    #1;
    if (!pass || imu0_vertical_axis != 2'd2 || imu0_vertical_positive || imu1_vertical_axis != 2'd1 || imu1_vertical_positive ||
        $signed(imu0_accel_mean_z) != -5504 || $signed(imu1_accel_mean_y) != -5350 ||
        !result_flags[4] || !result_flags[3] || !result_flags[2] || !result_flags[1]) begin
      $display("FAIL static pass=%b result_flags=%02x axes=%0d/%b,%0d/%b means=%0d,%0d",
               pass, result_flags, imu0_vertical_axis, imu0_vertical_positive, imu1_vertical_axis, imu1_vertical_positive,
               $signed(imu0_accel_mean_z), $signed(imu1_accel_mean_y));
      $fatal(1);
    end
    $display("RESULT static result_flags=%02x axes=%0d/%0d,%0d/%0d gyro_mean=%0d,%0d,%0d,%0d,%0d,%0d peaks=%0d,%0d",
             result_flags, imu0_vertical_axis, imu0_vertical_positive, imu1_vertical_axis, imu1_vertical_positive,
             $signed(imu0_gyro_mean_x), $signed(imu0_gyro_mean_y), $signed(imu0_gyro_mean_z),
             $signed(imu1_gyro_mean_x), $signed(imu1_gyro_mean_y), $signed(imu1_gyro_mean_z), imu0_gyro_peak, imu1_gyro_peak);

    // Mixed positive and negative samples expose accidental zero extension of
    // signed 16-bit inputs before the 32-bit accumulation.
    pulse_start();
    send_alternating_gyro_samples(8);
    wait(done);
    #1;
    if (!pass || $signed(imu0_gyro_mean_x) != 0 || $signed(imu0_gyro_mean_y) != 0 ||
        $signed(imu0_gyro_mean_z) != 0 || $signed(imu1_gyro_mean_x) != 0 ||
        $signed(imu1_gyro_mean_y) != 0 || $signed(imu1_gyro_mean_z) != 0) begin
      $display("FAIL signed means result_flags=%02x m0=%0d,%0d,%0d m1=%0d,%0d,%0d",
               result_flags, $signed(imu0_gyro_mean_x), $signed(imu0_gyro_mean_y), $signed(imu0_gyro_mean_z),
               $signed(imu1_gyro_mean_x), $signed(imu1_gyro_mean_y), $signed(imu1_gyro_mean_z));
      $fatal(1);
    end
    $display("RESULT signed_average result_flags=%02x gyro_mean=%0d,%0d,%0d,%0d,%0d,%0d",
             result_flags, $signed(imu0_gyro_mean_x), $signed(imu0_gyro_mean_y), $signed(imu0_gyro_mean_z),
             $signed(imu1_gyro_mean_x), $signed(imu1_gyro_mean_y), $signed(imu1_gyro_mean_z));
    imu0_gyro_x = 16'sd2;
    imu0_gyro_y = -16'sd3;
    imu0_gyro_z = 16'sd4;
    imu1_gyro_x = 16'sd30;
    imu1_gyro_y = -16'sd1;
    imu1_gyro_z = -16'sd11;

    // A transiently offline sensor invalidates this calibration run even if
    // the remaining samples look numerically plausible.
    pulse_start();
    send_samples(3);
    imu1_online = 1'b0;
    send_samples(1);
    imu1_online = 1'b1;
    send_samples(4);
    wait(done);
    #1;
    if (pass || result_flags[2]) begin
      $display("FAIL offline sample accepted result_flags=%02x", result_flags);
      $fatal(1);
    end
    $display("RESULT offline_rejection pass=%0d result_flags=%02x", pass, result_flags);

    // Excessive stationary angular rate must fail the quietness gate.
    imu1_gyro_y = 16'sd700;
    pulse_start();
    send_samples(8);
    wait(done);
    #1;
    if (pass || result_flags[6]) begin
      $display("FAIL noisy gyro accepted result_flags=%02x peak=%0d", result_flags, imu1_gyro_peak);
      $fatal(1);
    end
    $display("RESULT noisy_rejection pass=%0d result_flags=%02x imu1_peak=%0d",
             pass, result_flags, imu1_gyro_peak);

    $display("RESULT axes imu0=%0d sign=%b imu1=%0d sign=%b", imu0_vertical_axis, imu0_vertical_positive, imu1_vertical_axis, imu1_vertical_positive);
    $display("RESULT means imu0_a=%0d,%0d,%0d imu1_a=%0d,%0d,%0d",
             $signed(imu0_accel_mean_x), $signed(imu0_accel_mean_y), $signed(imu0_accel_mean_z),
             $signed(imu1_accel_mean_x), $signed(imu1_accel_mean_y), $signed(imu1_accel_mean_z));
    $display("RESULT TEST_PASS dual_imu_static_calibrator");
    $finish;
  end

  // Never allow a failed wait(done) to leave VVP writing an unbounded VCD.
  // The fast calibration test normally completes in a few microseconds, so
  // 1 ms leaves ample margin while capping every abnormal run.
  initial begin
    #1000000;
    $fatal(1, "timeout waiting for dual-IMU calibration completion");
  end


  dual_imu_static_calibrator dual_imu_static_calibrator0(
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
    .sample_valid(sample_valid),
    .imu0_online(imu0_online),
    .imu1_online(imu1_online),
    .imu0_accel_x(imu0_accel_x),
    .imu0_accel_y(imu0_accel_y),
    .imu0_accel_z(imu0_accel_z),
    .imu0_gyro_x(imu0_gyro_x),
    .imu0_gyro_y(imu0_gyro_y),
    .imu0_gyro_z(imu0_gyro_z),
    .imu1_accel_x(imu1_accel_x),
    .imu1_accel_y(imu1_accel_y),
    .imu1_accel_z(imu1_accel_z),
    .imu1_gyro_x(imu1_gyro_x),
    .imu1_gyro_y(imu1_gyro_y),
    .imu1_gyro_z(imu1_gyro_z),
    .busy(busy),
    .done(done),
    .pass(pass),
    .result_flags(result_flags),
    .imu0_vertical_axis(imu0_vertical_axis),
    .imu0_vertical_positive(imu0_vertical_positive),
    .imu1_vertical_axis(imu1_vertical_axis),
    .imu1_vertical_positive(imu1_vertical_positive),
    .imu0_accel_mean_x(imu0_accel_mean_x),
    .imu0_accel_mean_y(imu0_accel_mean_y),
    .imu0_accel_mean_z(imu0_accel_mean_z),
    .imu0_gyro_mean_x(imu0_gyro_mean_x),
    .imu0_gyro_mean_y(imu0_gyro_mean_y),
    .imu0_gyro_mean_z(imu0_gyro_mean_z),
    .imu1_accel_mean_x(imu1_accel_mean_x),
    .imu1_accel_mean_y(imu1_accel_mean_y),
    .imu1_accel_mean_z(imu1_accel_mean_z),
    .imu1_gyro_mean_x(imu1_gyro_mean_x),
    .imu1_gyro_mean_y(imu1_gyro_mean_y),
    .imu1_gyro_mean_z(imu1_gyro_mean_z),
    .imu0_gyro_peak(imu0_gyro_peak),
    .imu1_gyro_peak(imu1_gyro_peak)
  );
endmodule

`timescale 1ns/1ps
`define DUAL_IMU_CAL_SIM_FAST

module tb_dual_imu_static_calibrator;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg start = 1'b0;
  reg sample_valid = 1'b0;
  reg imu0_online = 1'b1;
  reg imu1_online = 1'b1;
  reg signed [15:0] a0x = 16'sd30;
  reg signed [15:0] a0y = -16'sd160;
  reg signed [15:0] a0z = -16'sd5504;
  reg signed [15:0] g0x = 16'sd2;
  reg signed [15:0] g0y = -16'sd3;
  reg signed [15:0] g0z = 16'sd4;
  reg signed [15:0] a1x = 16'sd1176;
  reg signed [15:0] a1y = -16'sd5350;
  reg signed [15:0] a1z = 16'sd165;
  reg signed [15:0] g1x = 16'sd30;
  reg signed [15:0] g1y = -16'sd1;
  reg signed [15:0] g1z = -16'sd11;

  wire busy;
  wire done;
  wire pass;
  wire [7:0] flags;
  wire [1:0] axis0;
  wire sign0;
  wire [1:0] axis1;
  wire sign1;
  wire signed [15:0] ma0x, ma0y, ma0z, mg0x, mg0y, mg0z;
  wire signed [15:0] ma1x, ma1y, ma1z, mg1x, mg1y, mg1z;
  wire [15:0] peak0, peak1;

  dual_imu_static_calibrator dut (
    .clk(clk), .rst_n(rst_n), .start(start), .sample_valid(sample_valid),
    .imu0_online(imu0_online), .imu1_online(imu1_online),
    .imu0_accel_x(a0x), .imu0_accel_y(a0y), .imu0_accel_z(a0z),
    .imu0_gyro_x(g0x), .imu0_gyro_y(g0y), .imu0_gyro_z(g0z),
    .imu1_accel_x(a1x), .imu1_accel_y(a1y), .imu1_accel_z(a1z),
    .imu1_gyro_x(g1x), .imu1_gyro_y(g1y), .imu1_gyro_z(g1z),
    .busy(busy), .done(done), .pass(pass), .result_flags(flags),
    .imu0_vertical_axis(axis0), .imu0_vertical_positive(sign0),
    .imu1_vertical_axis(axis1), .imu1_vertical_positive(sign1),
    .imu0_accel_mean_x(ma0x), .imu0_accel_mean_y(ma0y), .imu0_accel_mean_z(ma0z),
    .imu0_gyro_mean_x(mg0x), .imu0_gyro_mean_y(mg0y), .imu0_gyro_mean_z(mg0z),
    .imu1_accel_mean_x(ma1x), .imu1_accel_mean_y(ma1y), .imu1_accel_mean_z(ma1z),
    .imu1_gyro_mean_x(mg1x), .imu1_gyro_mean_y(mg1y), .imu1_gyro_mean_z(mg1z),
    .imu0_gyro_peak(peak0), .imu1_gyro_peak(peak1)
  );

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
        g0x = index[0] ? -16'sd10 : 16'sd10;
        g0y = index[0] ? 16'sd12 : -16'sd12;
        g0z = index[0] ? -16'sd8 : 16'sd8;
        g1x = index[0] ? -16'sd14 : 16'sd14;
        g1y = index[0] ? 16'sd6 : -16'sd6;
        g1z = index[0] ? -16'sd4 : 16'sd4;
        @(negedge clk); sample_valid = 1'b1;
        @(negedge clk); sample_valid = 1'b0;
      end
    end
  endtask

  initial begin
    $dumpfile("little_car/build/tb_dual_imu_static_calibrator.vcd");
    $dumpvars(0, tb_dual_imu_static_calibrator);
    repeat (4) @(posedge clk);
    rst_n = 1'b1;

    pulse_start();
    send_samples(8);
    wait(done);
    #1;
    if (!pass || axis0 != 2'd2 || sign0 || axis1 != 2'd1 || sign1 ||
        $signed(ma0z) != -5504 || $signed(ma1y) != -5350 ||
        !flags[4] || !flags[3] || !flags[2] || !flags[1]) begin
      $display("FAIL static pass=%b flags=%02x axes=%0d/%b,%0d/%b means=%0d,%0d",
               pass, flags, axis0, sign0, axis1, sign1,
               $signed(ma0z), $signed(ma1y));
      $fatal(1);
    end
    $display("RESULT static flags=%02x axes=%0d/%0d,%0d/%0d gyro_mean=%0d,%0d,%0d,%0d,%0d,%0d peaks=%0d,%0d",
             flags, axis0, sign0, axis1, sign1,
             $signed(mg0x), $signed(mg0y), $signed(mg0z),
             $signed(mg1x), $signed(mg1y), $signed(mg1z), peak0, peak1);

    // Mixed positive and negative samples expose accidental zero extension of
    // signed 16-bit inputs before the 32-bit accumulation.
    pulse_start();
    send_alternating_gyro_samples(8);
    wait(done);
    #1;
    if (!pass || $signed(mg0x) != 0 || $signed(mg0y) != 0 ||
        $signed(mg0z) != 0 || $signed(mg1x) != 0 ||
        $signed(mg1y) != 0 || $signed(mg1z) != 0) begin
      $display("FAIL signed means flags=%02x m0=%0d,%0d,%0d m1=%0d,%0d,%0d",
               flags, $signed(mg0x), $signed(mg0y), $signed(mg0z),
               $signed(mg1x), $signed(mg1y), $signed(mg1z));
      $fatal(1);
    end
    $display("RESULT signed_average flags=%02x gyro_mean=%0d,%0d,%0d,%0d,%0d,%0d",
             flags, $signed(mg0x), $signed(mg0y), $signed(mg0z),
             $signed(mg1x), $signed(mg1y), $signed(mg1z));
    g0x = 16'sd2;
    g0y = -16'sd3;
    g0z = 16'sd4;
    g1x = 16'sd30;
    g1y = -16'sd1;
    g1z = -16'sd11;

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
    if (pass || flags[2]) begin
      $display("FAIL offline sample accepted flags=%02x", flags);
      $fatal(1);
    end
    $display("RESULT offline_rejection pass=%0d flags=%02x", pass, flags);

    // Excessive stationary angular rate must fail the quietness gate.
    g1y = 16'sd700;
    pulse_start();
    send_samples(8);
    wait(done);
    #1;
    if (pass || flags[6]) begin
      $display("FAIL noisy gyro accepted flags=%02x peak=%0d", flags, peak1);
      $fatal(1);
    end
    $display("RESULT noisy_rejection pass=%0d flags=%02x imu1_peak=%0d",
             pass, flags, peak1);

    $display("RESULT axes imu0=%0d sign=%b imu1=%0d sign=%b", axis0, sign0, axis1, sign1);
    $display("RESULT means imu0_a=%0d,%0d,%0d imu1_a=%0d,%0d,%0d",
             $signed(ma0x), $signed(ma0y), $signed(ma0z),
             $signed(ma1x), $signed(ma1y), $signed(ma1z));
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
endmodule

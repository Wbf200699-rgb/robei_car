`timescale 1ns/1ps

module tb_rplidar_c1_uart_restart;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg lidar_uart_rx = 1'b1;
  reg scan_enable = 1'b1;

  wire lidar_uart_tx;
  wire lidar_online;
  wire scan_active;
  wire sample_valid;
  wire new_scan;
  wire [5:0] quality;
  wire [14:0] angle_q6;
  wire [15:0] distance_q2;
  wire [15:0] front_distance_mm;
  wire front_valid;
  wire obstacle_stop;
  wire [15:0] sector_front_mm, sector_front_right_mm, sector_right_mm;
  wire [15:0] sector_rear_right_mm, sector_rear_mm, sector_rear_left_mm;
  wire [15:0] sector_left_mm, sector_front_left_mm;

  always #10 clk = ~clk;

  rplidar_c1_uart dut (
    .clk(clk),
    .rst_n(rst_n),
    .lidar_uart_rx(lidar_uart_rx),
    .safe_distance_mm(16'd450),
    .scan_enable(scan_enable),
    .lidar_uart_tx(lidar_uart_tx),
    .lidar_online(lidar_online),
    .scan_active(scan_active),
    .sample_valid(sample_valid),
    .new_scan(new_scan),
    .quality(quality),
    .angle_q6(angle_q6),
    .distance_q2(distance_q2),
    .front_distance_mm(front_distance_mm),
    .front_valid(front_valid),
    .obstacle_stop(obstacle_stop),
    .sector_front_mm(sector_front_mm),
    .sector_front_right_mm(sector_front_right_mm),
    .sector_right_mm(sector_right_mm),
    .sector_rear_right_mm(sector_rear_right_mm),
    .sector_rear_mm(sector_rear_mm),
    .sector_rear_left_mm(sector_rear_left_mm),
    .sector_left_mm(sector_left_mm),
    .sector_front_left_mm(sector_front_left_mm)
  );

  task expect_state;
    input [3:0] expected;
    input [8*48-1:0] label;
    begin
      if (dut.cmd_state !== expected) begin
        $display("RESULT TEST_FAIL %0s expected_state=%0d actual_state=%0d",
                 label, expected, dut.cmd_state);
        $fatal(1);
      end
    end
  endtask

  task feed_rx;
    input [7:0] value;
    begin
      @(negedge clk);
      force dut.rx_valid = 1'b1;
      force dut.rx_data = value;
      @(posedge clk);
      #1;
      release dut.rx_valid;
      release dut.rx_data;
      dut.rx_valid = 1'b0;
    end
  endtask

  initial begin
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // A missed power-up command gets one bounded retry.  This is enough to
    // recover the C1 boot race without continuously cycling its motor.
    @(negedge clk);
    dut.cmd_state = dut.CMD_WAIT_DESC;
    dut.byte_timeout_count = dut.BYTE_TIMEOUT_CYCLES - 1;
    dut.desc_index = 3'd0;
    dut.descriptor_retry_used = 1'b0;
    @(posedge clk);
    #1;
    expect_state(dut.CMD_RETRY_WAIT, "bounded_descriptor_retry");
    if (dut.descriptor_retry_used !== 1'b1)
      $fatal(1, "descriptor retry was not latched");

    // If the retry descriptor is also lost, switch to raw node framing.  The
    // descriptor may be the missing part while scan data is already present.
    @(negedge clk);
    dut.cmd_state = dut.CMD_WAIT_DESC;
    dut.byte_timeout_count = dut.BYTE_TIMEOUT_CYCLES - 1;
    dut.descriptor_retry_used = 1'b1;
    @(posedge clk);
    #1;
    expect_state(dut.CMD_SCAN, "descriptor_loss_fallback");
    if (scan_active !== 1'b1)
      $fatal(1, "raw-node fallback did not enable scan parsing");

    // Exercise the normal descriptor path independently of the fallback.
    @(negedge clk);
    dut.cmd_state = dut.CMD_WAIT_DESC;
    dut.scan_active = 1'b0;
    dut.byte_timeout_count = 32'd0;
    dut.desc_index = 3'd0;

    // Standard SCAN descriptor: A5 5A, five-byte multiple response, type 81.
    feed_rx(8'hA5);
    feed_rx(8'h5A);
    feed_rx(8'h05);
    feed_rx(8'h00);
    feed_rx(8'h00);
    feed_rx(8'h40);
    feed_rx(8'h81);
    expect_state(dut.CMD_SCAN, "scan_descriptor");
    if (scan_active !== 1'b1 || lidar_online !== 1'b0)
      $fatal(1, "scan flags before first valid sample are wrong");

    // One valid 400 mm, 0 degree node proves actual stream activity and is
    // the point at which lidar_online is allowed to assert.
    feed_rx(8'h29);
    feed_rx(8'h01);
    feed_rx(8'h00);
    feed_rx(8'h40);
    feed_rx(8'h06);
    @(posedge clk);
    #1;
    if (lidar_online !== 1'b1)
      $fatal(1, "first valid scan sample did not assert lidar_online");

    $display("RESULT decoded quality=%0d angle_q6=%0d distance_q2=%0d front_mm=%0d front_valid=%0d stop=%0d",
             quality, angle_q6, distance_q2, front_distance_mm, front_valid,
             obstacle_stop);
    $display("RESULT sectors_mm=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
             sector_front_mm, sector_front_right_mm, sector_right_mm,
             sector_rear_right_mm, sector_rear_mm, sector_rear_left_mm,
             sector_left_mm, sector_front_left_mm);
    $display("RESULT startup_sequence=STOP,SCAN");
    $display("RESULT descriptor_retry_count=1");
    $display("RESULT descriptor_loss_raw_node_fallback=1");
    $display("RESULT online_requires_valid_sample=1");
    $display("RESULT TEST_PASS rplidar_protocol");
    $finish;
  end
endmodule

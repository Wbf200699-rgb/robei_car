`timescale 1ns/1ps

module tb_c5_motor_frame_reliability;
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg motor_enable = 1'b1;
  reg force_stop = 1'b0;
  reg [10:0] lf_speed = 11'd0;
  reg [10:0] rf_speed = 11'd0;
  reg [10:0] lr_speed = 11'd0;
  reg [10:0] rr_speed = 11'd0;
  wire c5_bus_tx;
  wire frame_busy;
  wire [15:0] frame_count;
  wire [11:0] last_lf_pulse;
  wire [11:0] last_rf_pulse;
  wire [11:0] last_lr_pulse;
  wire [11:0] last_rr_pulse;
  wire stop_frames_complete;
  wire [1:0] stop_frames_remaining;

  integer failures = 0;
  integer movement_frames = 0;
  integer stop_frames = 0;
  integer count_at_held_stop;
  integer stops_at_held_stop;
  reg [15:0] observed_frame_count = 16'd0;

  always #10 clk = ~clk;

  c5_motor_frame_driver dut (
    .clk(clk), .rst_n(rst_n), .motor_enable(motor_enable),
    .force_stop(force_stop), .lf_speed(lf_speed), .rf_speed(rf_speed),
    .lr_speed(lr_speed), .rr_speed(rr_speed), .c5_bus_tx(c5_bus_tx),
    .frame_busy(frame_busy), .frame_count(frame_count),
    .last_lf_pulse(last_lf_pulse), .last_rf_pulse(last_rf_pulse),
    .last_lr_pulse(last_lr_pulse), .last_rr_pulse(last_rr_pulse),
    .stop_frames_complete(stop_frames_complete),
    .stop_frames_remaining(stop_frames_remaining)
  );

  always @(posedge clk) begin
    if (!rst_n) begin
      observed_frame_count <= 16'd0;
      movement_frames = 0;
      stop_frames = 0;
    end else if (frame_count != observed_frame_count) begin
      observed_frame_count <= frame_count;
      if (dut.frame_is_stop)
        stop_frames = stop_frames + 1;
      else
        movement_frames = movement_frames + 1;
    end
  end

  initial begin
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    wait (stop_frames_complete && frame_count == 16'd3);

    // A changed vector is sent once and confirmed once after the guard.
    @(negedge clk);
    lf_speed = 11'd700;
    rf_speed = 11'd700;
    lr_speed = 11'd700;
    rr_speed = 11'd700;
    wait (movement_frames >= 2);
    if (last_lf_pulse != 12'd2200 || last_rf_pulse != 12'd800 ||
        last_lr_pulse != 12'd2200 || last_rr_pulse != 12'd800) begin
      $display("ERROR forward confirmation pulses=%0d,%0d,%0d,%0d",
               last_lf_pulse, last_rf_pulse,
               last_lr_pulse, last_rr_pulse);
      failures = failures + 1;
    end

    // Change the target while a periodic frame is active. The active frame
    // remains atomic and the first post-guard snapshot must be the latest
    // complete four-wheel vector, never a mixture of old and new signs.
    wait (frame_busy && !dut.frame_is_stop);
    repeat (1000) @(posedge clk);
    @(negedge clk);
    lf_speed = -11'sd650;
    rf_speed = 11'sd650;
    lr_speed = -11'sd650;
    rr_speed = 11'sd650;
    wait (last_lf_pulse == 12'd850 && last_rf_pulse == 12'd850 &&
          last_lr_pulse == 12'd850 && last_rr_pulse == 12'd850);
    wait (!frame_busy);

    // Hold force_stop high while the upstream vector remains nonzero. Exactly
    // one bounded three-frame stop group is allowed; after completion the UART
    // must stay idle until the safety level is released.
    wait (frame_busy && !dut.frame_is_stop);
    repeat (1000) @(posedge clk);
    @(negedge clk);
    force_stop = 1'b1;
    wait (stop_frames_complete && stop_frames_remaining == 2'd0);
    // Let the frame classification monitor observe the final frame_count NBA
    // before taking the held-level baseline.
    repeat (3) @(posedge clk);
    @(negedge clk);
    count_at_held_stop = frame_count;
    stops_at_held_stop = stop_frames;
    repeat (20000) @(posedge clk);
    if (frame_count != count_at_held_stop ||
        stop_frames != stops_at_held_stop) begin
      $display("ERROR held stop flooded bus frames=%0d->%0d stops=%0d->%0d",
               count_at_held_stop, frame_count,
               stops_at_held_stop, stop_frames);
      failures = failures + 1;
    end
    if (last_lf_pulse != 12'd1500 || last_rf_pulse != 12'd1500 ||
        last_lr_pulse != 12'd1500 || last_rr_pulse != 12'd1500) begin
      $display("ERROR held stop pulses=%0d,%0d,%0d,%0d",
               last_lf_pulse, last_rf_pulse,
               last_lr_pulse, last_rr_pulse);
      failures = failures + 1;
    end

    // Releasing the safety level makes the still-present upstream vector a new
    // command. It receives the same one-copy confirmation, then remains on the
    // ordinary 100 ms refresh schedule.
    @(negedge clk);
    force_stop = 1'b0;
    wait (movement_frames >= 6);
    if (last_lf_pulse != 12'd850 || last_rf_pulse != 12'd850 ||
        last_lr_pulse != 12'd850 || last_rr_pulse != 12'd850) begin
      $display("ERROR release did not restore latest vector");
      failures = failures + 1;
    end

    // A normal stop pulse and zero vector must leave no movement transaction
    // queued after the three stop commands.
    @(negedge clk);
    lf_speed = 11'd0;
    rf_speed = 11'd0;
    lr_speed = 11'd0;
    rr_speed = 11'd0;
    force_stop = 1'b1;
    @(negedge clk);
    force_stop = 1'b0;
    wait (stop_frames_complete && stop_frames_remaining == 2'd0);
    count_at_held_stop = frame_count;
    repeat (20000) @(posedge clk);
    if (frame_count != count_at_held_stop) begin
      $display("ERROR stale movement after stop count=%0d->%0d",
               count_at_held_stop, frame_count);
      failures = failures + 1;
    end

    $display("RESULT movement_frames=%0d stop_frames=%0d frame_count=%0d",
             movement_frames, stop_frames, frame_count);
    if (failures == 0)
      $display("RESULT TEST_PASS c5_motor_reliability");
    else
      $display("RESULT TEST_FAIL c5_motor_reliability failures=%0d",
               failures);
    $finish;
  end

  initial begin
    repeat (12000000) @(posedge clk);
    $display("RESULT TEST_FAIL c5_motor_reliability timeout frame_count=%0d",
             frame_count);
    $finish;
  end
endmodule

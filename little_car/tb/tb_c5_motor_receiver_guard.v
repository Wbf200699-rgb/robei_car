`timescale 1ns/1ps

module tb_c5_motor_receiver_guard;
  localparam integer RECEIVER_BUSY_CYCLES = 4000;
  localparam integer EXPECTED_GUARD_CYCLES = 5000;

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

  integer cycle_count = 0;
  integer receiver_busy_count = 0;
  integer frame_start_count = 0;
  integer frame_complete_count = 0;
  integer collision_count = 0;
  integer minimum_gap_cycles = 32'h7fffffff;
  integer last_complete_cycle = 0;
  integer gap_cycles;
  reg previous_frame_busy = 1'b0;

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

  // The factory STM32 ignores received bytes while it synchronously forwards
  // the preceding command to two UARTs. This compact model marks that receive
  // blackout after every complete frame. A new frame start inside the blackout
  // reproduces the condition that can leave only IDs 006, 007, 008 or 009 set.
  always @(posedge clk) begin
    cycle_count = cycle_count + 1;
    if (!rst_n) begin
      receiver_busy_count = 0;
      frame_start_count = 0;
      frame_complete_count = 0;
      collision_count = 0;
      minimum_gap_cycles = 32'h7fffffff;
      last_complete_cycle = 0;
      previous_frame_busy = 1'b0;
    end else begin
      if (receiver_busy_count > 0)
        receiver_busy_count = receiver_busy_count - 1;

      if (!previous_frame_busy && frame_busy) begin
        frame_start_count = frame_start_count + 1;
        if (receiver_busy_count > 0) begin
          collision_count = collision_count + 1;
          $display("ERROR receiver collision start=%0d busy_left=%0d",
                   frame_start_count, receiver_busy_count);
        end
        if (frame_complete_count > 0) begin
          gap_cycles = cycle_count - last_complete_cycle;
          if (gap_cycles < minimum_gap_cycles)
            minimum_gap_cycles = gap_cycles;
          if (gap_cycles < EXPECTED_GUARD_CYCLES) begin
            collision_count = collision_count + 1;
            $display("ERROR interframe gap=%0d expected>=%0d",
                     gap_cycles, EXPECTED_GUARD_CYCLES);
          end
        end
      end

      if (previous_frame_busy && !frame_busy) begin
        frame_complete_count = frame_complete_count + 1;
        last_complete_cycle = cycle_count;
        receiver_busy_count = RECEIVER_BUSY_CYCLES;
      end
      previous_frame_busy = frame_busy;
    end
  end

  initial begin
    repeat (5) @(posedge clk);
    rst_n = 1'b1;

    // Ask for motion while the startup stop group is still in progress. The
    // target must wait behind complete guarded transactions, never splice into
    // a stop frame or the receiver blackout.
    wait (frame_count == 16'd1);
    lf_speed = 11'd700;
    rf_speed = 11'd700;
    lr_speed = 11'd700;
    rr_speed = 11'd700;
    wait (frame_count >= 16'd4);

    // Change all four signs during an active movement transaction. The next
    // transaction must use one atomic snapshot after the guard.
    wait (frame_busy && !dut.frame_is_stop);
    repeat (1000) @(posedge clk);
    lf_speed = -11'sd650;
    rf_speed = 11'sd650;
    lr_speed = -11'sd650;
    rr_speed = 11'sd650;
    // Frame 5 is the already-running forward refresh. Frame 6 is the first
    // complete transaction allowed to snapshot the new reversal vector.
    wait (frame_count >= 16'd6);
    if (last_lf_pulse != 12'd850 || last_rf_pulse != 12'd850 ||
        last_lr_pulse != 12'd850 || last_rr_pulse != 12'd850) begin
      $display("ERROR reversal snapshot pulses=%0d,%0d,%0d,%0d",
               last_lf_pulse, last_rf_pulse, last_lr_pulse, last_rr_pulse);
      collision_count = collision_count + 1;
    end

    // Release stop in the middle of the next movement refresh. This was the
    // worst case for the old implementation: all three 9-byte stop commands
    // could fit inside the STM32's forwarding blackout and be discarded.
    wait (frame_busy && !dut.frame_is_stop);
    repeat (1000) @(posedge clk);
    lf_speed = 11'd0;
    rf_speed = 11'd0;
    lr_speed = 11'd0;
    rr_speed = 11'd0;
    force_stop = 1'b1;
    @(posedge clk);
    force_stop = 1'b0;
    wait (stop_frames_complete && stop_frames_remaining == 2'd0);

    repeat (100) @(posedge clk);
    $display("RESULT frame_starts=%0d frame_completes=%0d min_gap_cycles=%0d",
             frame_start_count, frame_complete_count, minimum_gap_cycles);
    if (collision_count == 0 &&
        minimum_gap_cycles >= EXPECTED_GUARD_CYCLES)
      $display("RESULT TEST_PASS c5_receiver_guard");
    else
      $display("RESULT TEST_FAIL c5_receiver_guard collisions=%0d",
               collision_count);
    $finish;
  end

  initial begin
    repeat (5000000) @(posedge clk);
    $display("RESULT TEST_FAIL c5_receiver_guard timeout frames=%0d busy=%0d",
             frame_count, frame_busy);
    $finish;
  end
endmodule

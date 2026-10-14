`timescale 1ns/1ps
`define C5_MOTOR_SIM_FAST

module tb_c5_motor_uart_protocol;
  localparam integer CLKS_PER_BIT = 50000000 / 115200;
  reg clk = 0, rst_n = 0, motor_enable = 1, force_stop = 0;
  reg [10:0] lf_speed = 0, rf_speed = 0, lr_speed = 0, rr_speed = 0;
  wire tx, busy, stop_complete;
  wire [15:0] frame_count;
  wire [11:0] lf_pulse, rf_pulse, lr_pulse, rr_pulse;
  wire [1:0] stop_remaining;
  integer failures = 0;
  integer i;
  reg [7:0] rx;

  always #10 clk = ~clk;

  c5_motor_frame_driver dut(
    .clk(clk), .rst_n(rst_n), .motor_enable(motor_enable),
    .force_stop(force_stop), .lf_speed(lf_speed), .rf_speed(rf_speed),
    .lr_speed(lr_speed), .rr_speed(rr_speed), .c5_bus_tx(tx),
    .frame_busy(busy), .frame_count(frame_count),
    .last_lf_pulse(lf_pulse), .last_rf_pulse(rf_pulse),
    .last_lr_pulse(lr_pulse), .last_rr_pulse(rr_pulse),
    .stop_frames_complete(stop_complete),
    .stop_frames_remaining(stop_remaining)
  );

  task read_byte;
    output [7:0] value;
    integer bit_no;
    begin
      @(negedge tx);
      repeat (CLKS_PER_BIT + CLKS_PER_BIT/2) @(posedge clk);
      for (bit_no = 0; bit_no < 8; bit_no = bit_no + 1) begin
        value[bit_no] = tx;
        repeat (CLKS_PER_BIT) @(posedge clk);
      end
      repeat (CLKS_PER_BIT/4) @(posedge clk);
    end
  endtask

  function [7:0] stop_broadcast;
    input integer index;
    begin
      case(index)
        0: stop_broadcast="#"; 1: stop_broadcast="2";
        2,3: stop_broadcast="5"; 4: stop_broadcast="P";
        5: stop_broadcast="D"; 6: stop_broadcast="S";
        7: stop_broadcast="T"; default: stop_broadcast="!";
      endcase
    end
  endfunction

  function [7:0] neutral_byte;
    input integer index;
    begin
      case(index)
        0: neutral_byte="{";
        1,16,31,46: neutral_byte="#";
        2,3,17,18,32,33,47,48: neutral_byte="0";
        4: neutral_byte="6"; 19: neutral_byte="7";
        34: neutral_byte="8"; 49: neutral_byte="9";
        5,20,35,50: neutral_byte="P";
        6,21,36,51: neutral_byte="1";
        7,22,37,52: neutral_byte="5";
        8,9,23,24,38,39,53,54: neutral_byte="0";
        10,25,40,55: neutral_byte="T";
        15,30,45,60: neutral_byte="!";
        61: neutral_byte="}";
        default: neutral_byte="0";
      endcase
    end
  endfunction

  function [7:0] forward_byte;
    input integer index;
    begin
      forward_byte = neutral_byte(index);
      case(index)
        6,7,36,37: forward_byte="2";
        8,9,38,39: forward_byte="0";
        21,51: forward_byte="0";
        22,52: forward_byte="8";
        23,24,53,54: forward_byte="0";
      endcase
    end
  endfunction

  task expect_broadcast;
    begin
      for(i=0;i<9;i=i+1) begin
        read_byte(rx);
        if(rx !== stop_broadcast(i)) begin
          $display("ERROR broadcast[%0d]=%02x", i, rx); failures=failures+1;
        end
      end
    end
  endtask

  task expect_neutral;
    begin
      for(i=0;i<62;i=i+1) begin
        read_byte(rx);
        if(rx !== neutral_byte(i)) begin
          $display("ERROR neutral[%0d]=%02x", i, rx); failures=failures+1;
        end
      end
    end
  endtask

  task expect_forward;
    begin
      for(i=0;i<62;i=i+1) begin
        read_byte(rx);
        if(rx !== forward_byte(i)) begin
          $display("ERROR forward[%0d]=%02x", i, rx); failures=failures+1;
        end
      end
    end
  endtask

  initial begin
    repeat(5) @(posedge clk); rst_n=1;
    // The GitHub baseline emits three bounded bus-wide stop commands at
    // startup. Older versions of this test incorrectly expected a full
    // neutral wheel frame between two broadcasts.
    expect_broadcast(); expect_broadcast(); expect_broadcast();
    wait(stop_complete);

    lf_speed=700; rf_speed=700; lr_speed=700; rr_speed=700;
    expect_forward(); expect_forward();

    // Stop while no queued refresh is active. It must be the bounded three
    // broadcast sequence, with no movement frame after the final broadcast.
    @(negedge clk);
    lf_speed=0; rf_speed=0; lr_speed=0; rr_speed=0; force_stop=1;
    @(negedge clk); force_stop=0;
    expect_broadcast(); expect_broadcast(); expect_broadcast();
    wait(stop_complete);
    repeat(2000) @(posedge clk);
    if(busy || stop_remaining != 0) begin
      $display("ERROR stop residue busy=%0d remaining=%0d",busy,stop_remaining);
      failures=failures+1;
    end

    $display("RESULT frames=%0d pulses=%0d,%0d,%0d,%0d",
             frame_count,lf_pulse,rf_pulse,lr_pulse,rr_pulse);
    if(failures==0) $display("RESULT TEST_PASS c5_uart_guarded_recovery");
    else $display("RESULT TEST_FAIL c5_uart_guarded_recovery failures=%0d",failures);
    $finish;
  end
endmodule

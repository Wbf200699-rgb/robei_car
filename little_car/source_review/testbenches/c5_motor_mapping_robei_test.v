// =============================================================================
// Testbench: c5_motor_mapping_robei_test
// 验证目标: 验证前进、后退、原地左右转及四轮校准到脉宽的映射。
// 原始文件: little_car/test/verilog/c5_motor_mapping_robei_test.v
// 本副本随 export_sources.py 重新生成，请在原测试目录修改测试逻辑。
// =============================================================================

`timescale 1ns/1ps
module c5_motor_frame_driver(
  input wire clk,
  input wire rst_n,
  input wire motor_enable,
  input wire force_stop,
  input wire [10:0] lf_speed,
  input wire [10:0] rf_speed,
  input wire [10:0] lr_speed,
  input wire [10:0] rr_speed,
  output wire c5_bus_tx,
  output reg frame_busy,
  output reg [15:0] frame_count,
  output reg [11:0] last_lf_pulse,
  output reg [11:0] last_rf_pulse,
  output reg [11:0] last_lr_pulse,
  output reg [11:0] last_rr_pulse,
  output reg stop_frames_complete,
  output reg [1:0] stop_frames_remaining
);
            localparam integer CLK_HZ = 50000000;
            localparam integer BAUD = 115200;
            localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
            localparam integer MOVING_REFRESH_MS = 100;
            localparam integer MOVING_REFRESH_CYCLES = (CLK_HZ / 1000) * MOVING_REFRESH_MS;
          localparam integer STARTUP_DELAY_MS = 500;
            localparam integer STARTUP_CYCLES = (CLK_HZ / 1000) * STARTUP_DELAY_MS;
          localparam [6:0] FRAME_LAST = 7'd61;
          localparam [6:0] STOP_FRAME_LAST = 7'd8;
            localparam [1:0] UART_IDLE  = 2'd0;
            localparam [1:0] UART_START = 2'd1;
            localparam [1:0] UART_DATA  = 2'd2;
            localparam [1:0] UART_STOP  = 2'd3;

            wire [10:0] effective_lf_speed;
            wire [10:0] effective_rf_speed;
            wire [10:0] effective_lr_speed;
            wire [10:0] effective_rr_speed;
            wire uart_done;
            wire command_changed;
            wire refresh_due;
            wire any_motion;
            wire last_any_motion;
            wire stop_transition;
          wire send_stop_now;
          wire [6:0] active_frame_last;

            reg [10:0] last_lf_speed;
            reg [10:0] last_rf_speed;
            reg [10:0] last_lr_speed;
            reg [10:0] last_rr_speed;
            reg [11:0] frame_lf_pulse;
            reg [11:0] frame_rf_pulse;
            reg [11:0] frame_lr_pulse;
            reg [11:0] frame_rr_pulse;
            reg [31:0] refresh_count;
            reg [6:0] send_index;
            reg [7:0] uart_data;
            reg uart_start;
            reg uart_tx;
            reg uart_busy;
            reg uart_busy_d;
            reg [1:0] uart_state;
            reg [15:0] uart_clk_count;
            reg [2:0] uart_bit_index;
            reg [7:0] uart_shifter;
            reg sending;
            reg pending_send;
            reg frame_is_stop;
            reg [24:0] startup_count;
            reg startup_ready;

            assign effective_lf_speed = motor_enable ? lf_speed : 11'd0;
            assign effective_rf_speed = motor_enable ? rf_speed : 11'd0;
            assign effective_lr_speed = motor_enable ? lr_speed : 11'd0;
            assign effective_rr_speed = motor_enable ? rr_speed : 11'd0;
            assign command_changed = (effective_lf_speed != last_lf_speed) ||
                                     (effective_rf_speed != last_rf_speed) ||
                                     (effective_lr_speed != last_lr_speed) ||
                                     (effective_rr_speed != last_rr_speed);
            assign any_motion = (effective_lf_speed != 11'd0) ||
                                (effective_rf_speed != 11'd0) ||
                                (effective_lr_speed != 11'd0) ||
                                (effective_rr_speed != 11'd0);
            assign last_any_motion = (last_lf_speed != 11'd0) ||
                                     (last_rf_speed != 11'd0) ||
                                     (last_lr_speed != 11'd0) ||
                                     (last_rr_speed != 11'd0);
            assign stop_transition = !any_motion && last_any_motion;
          assign refresh_due = any_motion &&
                               (refresh_count >= MOVING_REFRESH_CYCLES - 1);
            assign send_stop_now = force_stop || stop_transition ||
                                   (stop_frames_remaining != 2'd0) || !any_motion;
          assign uart_done = uart_busy_d && !uart_busy;
          assign active_frame_last = frame_is_stop ? STOP_FRAME_LAST : FRAME_LAST;
            assign c5_bus_tx = uart_tx;

            function [10:0] speed_abs;
              input [10:0] speed;
              begin
                if (speed[10])
                  speed_abs = (~speed) + 11'd1;
                else
                  speed_abs = speed;
              end
            endfunction

            function [11:0] left_motor_pulse;
              input [10:0] speed;
              reg [10:0] mag;
              begin
                mag = speed_abs(speed);
                if (speed[10])
                  left_motor_pulse = 12'd1500 - {1'b0, mag};
                else
                  left_motor_pulse = 12'd1500 + {1'b0, mag};
              end
            endfunction

            function [11:0] right_motor_pulse;
              input [10:0] speed;
              reg [10:0] mag;
              begin
                mag = speed_abs(speed);
                if (speed[10])
                  right_motor_pulse = 12'd1500 + {1'b0, mag};
                else
                  right_motor_pulse = 12'd1500 - {1'b0, mag};
              end
            endfunction

            function [7:0] digit4;
              input [11:0] value;
              input [1:0] index;
              integer digit;
              integer remainder;
              begin
                remainder = value;
                if (remainder >= 2000) begin
                  remainder = remainder - 2000;
                  if (index == 2'd0) digit = 2;
                end else if (remainder >= 1000) begin
                  remainder = remainder - 1000;
                  if (index == 2'd0) digit = 1;
                end else if (index == 2'd0) begin
                  digit = 0;
                end

                if (index != 2'd0) begin
                  if (remainder >= 900) begin remainder = remainder - 900; digit = 9; end
                  else if (remainder >= 800) begin remainder = remainder - 800; digit = 8; end
                  else if (remainder >= 700) begin remainder = remainder - 700; digit = 7; end
                  else if (remainder >= 600) begin remainder = remainder - 600; digit = 6; end
                  else if (remainder >= 500) begin remainder = remainder - 500; digit = 5; end
                  else if (remainder >= 400) begin remainder = remainder - 400; digit = 4; end
                  else if (remainder >= 300) begin remainder = remainder - 300; digit = 3; end
                  else if (remainder >= 200) begin remainder = remainder - 200; digit = 2; end
                  else if (remainder >= 100) begin remainder = remainder - 100; digit = 1; end
                  else digit = 0;
                end

                if (index >= 2'd2) begin
                  if (remainder >= 90) begin remainder = remainder - 90; digit = 9; end
                  else if (remainder >= 80) begin remainder = remainder - 80; digit = 8; end
                  else if (remainder >= 70) begin remainder = remainder - 70; digit = 7; end
                  else if (remainder >= 60) begin remainder = remainder - 60; digit = 6; end
                  else if (remainder >= 50) begin remainder = remainder - 50; digit = 5; end
                  else if (remainder >= 40) begin remainder = remainder - 40; digit = 4; end
                  else if (remainder >= 30) begin remainder = remainder - 30; digit = 3; end
                  else if (remainder >= 20) begin remainder = remainder - 20; digit = 2; end
                  else if (remainder >= 10) begin remainder = remainder - 10; digit = 1; end
                  else digit = 0;
                end

                case (index)
                  2'd3: digit = remainder;
                  default: begin end
                endcase
                digit4 = 8'd48 + digit[7:0];
              end
            endfunction

          function [7:0] frame_char;
              input [6:0] index;
              begin
              if (frame_is_stop) begin
                case (index)
                  7'd0: frame_char = "#";
                  7'd1: frame_char = "2";
                  7'd2, 7'd3: frame_char = "5";
                  7'd4: frame_char = "P";
                  7'd5: frame_char = "D";
                  7'd6: frame_char = "S";
                  7'd7: frame_char = "T";
                  default: frame_char = "!";
                endcase
              end else begin
                case (index)
                  7'd0:  frame_char = "{";
                  7'd1, 7'd16, 7'd31, 7'd46: frame_char = "#";
                  7'd2, 7'd3, 7'd17, 7'd18, 7'd32, 7'd33, 7'd47, 7'd48: frame_char = "0";
                  7'd4:  frame_char = "6";
                  7'd19: frame_char = "7";
                  7'd34: frame_char = "8";
                  7'd49: frame_char = "9";
                  7'd5, 7'd20, 7'd35, 7'd50: frame_char = "P";
                  7'd6:  frame_char = digit4(frame_lf_pulse, 2'd0);
                  7'd7:  frame_char = digit4(frame_lf_pulse, 2'd1);
                  7'd8:  frame_char = digit4(frame_lf_pulse, 2'd2);
                  7'd9:  frame_char = digit4(frame_lf_pulse, 2'd3);
                  7'd21: frame_char = digit4(frame_rf_pulse, 2'd0);
                  7'd22: frame_char = digit4(frame_rf_pulse, 2'd1);
                  7'd23: frame_char = digit4(frame_rf_pulse, 2'd2);
                  7'd24: frame_char = digit4(frame_rf_pulse, 2'd3);
                  7'd36: frame_char = digit4(frame_lr_pulse, 2'd0);
                  7'd37: frame_char = digit4(frame_lr_pulse, 2'd1);
                  7'd38: frame_char = digit4(frame_lr_pulse, 2'd2);
                  7'd39: frame_char = digit4(frame_lr_pulse, 2'd3);
                  7'd51: frame_char = digit4(frame_rr_pulse, 2'd0);
                  7'd52: frame_char = digit4(frame_rr_pulse, 2'd1);
                  7'd53: frame_char = digit4(frame_rr_pulse, 2'd2);
                  7'd54: frame_char = digit4(frame_rr_pulse, 2'd3);
                  7'd10, 7'd25, 7'd40, 7'd55: frame_char = "T";
                  7'd11, 7'd12, 7'd13, 7'd14,
                  7'd26, 7'd27, 7'd28, 7'd29,
                  7'd41, 7'd42, 7'd43, 7'd44,
                  7'd56, 7'd57, 7'd58, 7'd59: frame_char = "0";
                  7'd15, 7'd30, 7'd45, 7'd60: frame_char = "!";
                default: frame_char = "}";
                endcase
              end
              end
            endfunction

            always @(posedge clk or negedge rst_n) begin
              if (!rst_n) begin
                last_lf_speed <= 11'h7ff;
                last_rf_speed <= 11'h7ff;
                last_lr_speed <= 11'h7ff;
                last_rr_speed <= 11'h7ff;
                frame_lf_pulse <= 12'd1500;
                frame_rf_pulse <= 12'd1500;
                frame_lr_pulse <= 12'd1500;
                frame_rr_pulse <= 12'd1500;
                refresh_count <= 32'd0;
                send_index <= 7'd0;
                uart_data <= 8'd0;
                uart_start <= 1'b0;
                uart_tx <= 1'b1;
                uart_busy <= 1'b0;
                uart_busy_d <= 1'b0;
                uart_state <= UART_IDLE;
                uart_clk_count <= 16'd0;
                uart_bit_index <= 3'd0;
                uart_shifter <= 8'd0;
                sending <= 1'b0;
                pending_send <= 1'b0;
                frame_is_stop <= 1'b0;
                startup_count <= 25'd0;
                startup_ready <= 1'b0;
                frame_busy <= 1'b0;
                frame_count <= 16'd0;
                last_lf_pulse <= 12'd1500;
                last_rf_pulse <= 12'd1500;
                last_lr_pulse <= 12'd1500;
                last_rr_pulse <= 12'd1500;
                stop_frames_complete <= 1'b0;
                stop_frames_remaining <= 2'd0;
              end else begin
                uart_start <= 1'b0;
                uart_busy_d <= uart_busy;

                case (uart_state)
                  UART_IDLE: begin
                    uart_tx <= 1'b1;
                    uart_busy <= 1'b0;
                    uart_clk_count <= 16'd0;
                    uart_bit_index <= 3'd0;
                    if (uart_start) begin
                      uart_busy <= 1'b1;
                      uart_shifter <= uart_data;
                      uart_state <= UART_START;
                      uart_tx <= 1'b0;
                    end
                  end

                  UART_START: begin
                    uart_busy <= 1'b1;
                    uart_tx <= 1'b0;
                    if (uart_clk_count == CLKS_PER_BIT - 1) begin
                      uart_clk_count <= 16'd0;
                      uart_state <= UART_DATA;
                      uart_tx <= uart_shifter[0];
                    end else begin
                      uart_clk_count <= uart_clk_count + 16'd1;
                    end
                  end

                  UART_DATA: begin
                    uart_busy <= 1'b1;
                    uart_tx <= uart_shifter[uart_bit_index];
                    if (uart_clk_count == CLKS_PER_BIT - 1) begin
                      uart_clk_count <= 16'd0;
                      if (uart_bit_index == 3'd7) begin
                        uart_bit_index <= 3'd0;
                        uart_state <= UART_STOP;
                        uart_tx <= 1'b1;
                      end else begin
                        uart_bit_index <= uart_bit_index + 3'd1;
                        uart_tx <= uart_shifter[uart_bit_index + 3'd1];
                      end
                    end else begin
                      uart_clk_count <= uart_clk_count + 16'd1;
                    end
                  end

                  default: begin
                    uart_busy <= 1'b1;
                    uart_tx <= 1'b1;
                    if (uart_clk_count == CLKS_PER_BIT - 1) begin
                      uart_clk_count <= 16'd0;
                      uart_busy <= 1'b0;
                      uart_state <= UART_IDLE;
                    end else begin
                      uart_clk_count <= uart_clk_count + 16'd1;
                    end
                  end
                endcase

                if (!startup_ready) begin
                  refresh_count <= 32'd0;
                  frame_busy <= 1'b0;
                  if (startup_count >= STARTUP_CYCLES - 1) begin
                    startup_count <= startup_count;
                    startup_ready <= 1'b1;
                    stop_frames_remaining <= 2'd3;
                    stop_frames_complete <= 1'b0;
                  end else begin
                    startup_count <= startup_count + 25'd1;
                  end
              end else if (!sending && any_motion && !refresh_due) begin
                refresh_count <= refresh_count + 32'd1;
              end else if (!sending && !any_motion) begin
                refresh_count <= 32'd0;
                end

                if (startup_ready && (force_stop || stop_transition)) begin
                  stop_frames_remaining <= 2'd3;
                  stop_frames_complete <= 1'b0;
                end

                if (startup_ready && !sending) begin
                  frame_busy <= 1'b0;
                  if ((force_stop || (stop_frames_remaining != 2'd0) ||
                       pending_send || command_changed || refresh_due) && !uart_busy) begin
                    if (send_stop_now) begin
                      frame_lf_pulse <= 12'd1500;
                      frame_rf_pulse <= 12'd1500;
                      frame_lr_pulse <= 12'd1500;
                      frame_rr_pulse <= 12'd1500;
                      last_lf_pulse <= 12'd1500;
                      last_rf_pulse <= 12'd1500;
                      last_lr_pulse <= 12'd1500;
                      last_rr_pulse <= 12'd1500;
                      last_lf_speed <= 11'd0;
                      last_rf_speed <= 11'd0;
                      last_lr_speed <= 11'd0;
                      last_rr_speed <= 11'd0;
                      frame_is_stop <= 1'b1;
                    end else begin
                      frame_lf_pulse <= left_motor_pulse(effective_lf_speed);
                      frame_rf_pulse <= right_motor_pulse(effective_rf_speed);
                      frame_lr_pulse <= left_motor_pulse(effective_lr_speed);
                      frame_rr_pulse <= right_motor_pulse(effective_rr_speed);
                      last_lf_pulse <= left_motor_pulse(effective_lf_speed);
                      last_rf_pulse <= right_motor_pulse(effective_rf_speed);
                      last_lr_pulse <= left_motor_pulse(effective_lr_speed);
                      last_rr_pulse <= right_motor_pulse(effective_rr_speed);
                      last_lf_speed <= effective_lf_speed;
                      last_rf_speed <= effective_rf_speed;
                      last_lr_speed <= effective_lr_speed;
                      last_rr_speed <= effective_rr_speed;
                      frame_is_stop <= 1'b0;
                      stop_frames_complete <= 1'b0;
                    end
                    refresh_count <= 32'd0;
                    pending_send <= 1'b0;
                    sending <= 1'b1;
                    frame_busy <= 1'b1;
                    send_index <= 7'd0;
                uart_data <= send_stop_now ? "#" : "{";
                    uart_start <= 1'b1;
                  end
                end else if (startup_ready) begin
                  frame_busy <= 1'b1;
                  if (uart_done) begin
                  if (send_index == active_frame_last) begin
                      sending <= 1'b0;
                      frame_busy <= 1'b0;
                      frame_count <= frame_count + 16'd1;
                      if (frame_is_stop) begin
                        if (stop_frames_remaining > 2'd1) begin
                          stop_frames_remaining <= stop_frames_remaining - 2'd1;
                        end else begin
                          stop_frames_remaining <= 2'd0;
                          stop_frames_complete <= 1'b1;
                        end
                      end
                    end else begin
                      send_index <= send_index + 7'd1;
                      uart_data <= frame_char(send_index + 7'd1);
                      uart_start <= 1'b1;
                    end
                  end
                end
              end
            end
endmodule

module c5_motor_mapping_robei_test;
  reg clk;
  reg rst_n;
  reg motor_enable;
  reg force_stop;
  reg [10:0] lf_speed;
  reg [10:0] rf_speed;
  reg [10:0] lr_speed;
  reg [10:0] rr_speed;
  wire c5_bus_tx;
  wire frame_busy;
  wire [15:0] frame_count;
  wire [11:0] last_lf_pulse;
  wire [11:0] last_rf_pulse;
  wire [11:0] last_lr_pulse;
  wire [11:0] last_rr_pulse;
  wire stop_frames_complete;
  wire [1:0] stop_frames_remaining;


integer fd;
integer errors;
integer base_count;

initial begin
  clk = 1'b0;
end
always #10 clk = ~clk;

task apply_speeds;
  input [10:0] lf;
  input [10:0] rf;
  input [10:0] lr;
  input [10:0] rr;
  input [8*20-1:0] label_text;
  begin
    base_count = frame_count;
    lf_speed = lf;
    rf_speed = rf;
    lr_speed = lr;
    rr_speed = rr;
    wait (frame_count != base_count);
    #1000;
    $display("RESULT %0s pulses=%0d,%0d,%0d,%0d", label_text,
             last_lf_pulse,last_rf_pulse,last_lr_pulse,last_rr_pulse);
    $fwrite(fd,"%0s,%0d,%0d,%0d,%0d\n",label_text,last_lf_pulse,
            last_rf_pulse,last_lr_pulse,last_rr_pulse);
  end
endtask

initial begin
  fd = $fopen("c5_mapping_results.csv", "w");
  $fwrite(fd,"case,lf_pulse,rf_pulse,lr_pulse,rr_pulse\n");
  errors = 0;
  rst_n = 1'b0;
  motor_enable = 1'b1;
  force_stop = 1'b0;
  lf_speed = 0; rf_speed = 0; lr_speed = 0; rr_speed = 0;
  #200; rst_n = 1'b1;
  #20;
  c5_motor_frame_driver0.startup_count = 24999999;
  wait (stop_frames_complete);
  apply_speeds(11'd700,11'd700,11'd700,11'd700,"forward_700");
  if (last_lf_pulse!=2200 || last_rf_pulse!=800 || last_lr_pulse!=2200 || last_rr_pulse!=800) errors=errors+1;
  apply_speeds(-11'sd700,-11'sd700,-11'sd700,-11'sd700,"reverse_700");
  if (last_lf_pulse!=800 || last_rf_pulse!=2200 || last_lr_pulse!=800 || last_rr_pulse!=2200) errors=errors+1;
  apply_speeds(-11'sd650,11'd650,-11'sd650,11'd650,"spin_left_650");
  if (last_lf_pulse!=850 || last_rf_pulse!=850 || last_lr_pulse!=850 || last_rr_pulse!=850) errors=errors+1;
  apply_speeds(11'd650,-11'sd650,11'd650,-11'sd650,"spin_right_650");
  if (last_lf_pulse!=2150 || last_rf_pulse!=2150 || last_lr_pulse!=2150 || last_rr_pulse!=2150) errors=errors+1;
  apply_speeds(11'd650,11'd750,11'd800,11'd625,"wheel_trim_sample");
  if (errors==0) $display("RESULT TEST_PASS c5_mapping");
  else $display("RESULT TEST_FAIL c5_mapping errors=%0d",errors);
  $fclose(fd);
  $finish;
end


  c5_motor_frame_driver c5_motor_frame_driver0(
    .clk(clk),
    .rst_n(rst_n),
    .motor_enable(motor_enable),
    .force_stop(force_stop),
    .lf_speed(lf_speed),
    .rf_speed(rf_speed),
    .lr_speed(lr_speed),
    .rr_speed(rr_speed),
    .c5_bus_tx(c5_bus_tx),
    .frame_busy(frame_busy),
    .frame_count(frame_count),
    .last_lf_pulse(last_lf_pulse),
    .last_rf_pulse(last_rf_pulse),
    .last_lr_pulse(last_lr_pulse),
    .last_rr_pulse(last_rr_pulse),
    .stop_frames_complete(stop_frames_complete),
    .stop_frames_remaining(stop_frames_remaining)
  );
endmodule

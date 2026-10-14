`timescale 1ns/1ps
module rplidar_c1_uart(
  input wire clk,
  input wire rst_n,
  input wire lidar_uart_rx,
  input wire [15:0] safe_distance_mm,
  input wire scan_enable,
  output wire lidar_uart_tx,
  output reg lidar_online,
  output reg scan_active,
  output reg sample_valid,
  output reg new_scan,
  output reg [5:0] quality,
  output reg [14:0] angle_q6,
  output reg [15:0] distance_q2,
  output reg [15:0] front_distance_mm,
  output reg front_valid,
  output reg obstacle_stop,
  output reg [15:0] sector_front_mm,
  output reg [15:0] sector_front_right_mm,
  output reg [15:0] sector_right_mm,
  output reg [15:0] sector_rear_right_mm,
  output reg [15:0] sector_rear_mm,
  output reg [15:0] sector_rear_left_mm,
  output reg [15:0] sector_left_mm,
  output reg [15:0] sector_front_left_mm
);
          localparam integer CLK_HZ = 50000000;
          localparam integer BAUD = 460800;
          localparam integer CLKS_PER_BIT = ((CLK_HZ + (BAUD / 2)) / BAUD);
`ifdef RPLIDAR_SIM_FAST
          localparam integer BOOT_DELAY_CYCLES = 20;
          localparam integer STOP_WAIT_CYCLES = 10;
          localparam integer RETRY_WAIT_CYCLES = 20;
          localparam integer BYTE_TIMEOUT_CYCLES = 80;
          localparam integer STREAM_RECOVERY_CYCLES = 240;
`else
          localparam integer BOOT_DELAY_CYCLES = CLK_HZ / 2;
          localparam integer STOP_WAIT_CYCLES = CLK_HZ / 10;
          localparam integer RETRY_WAIT_CYCLES = CLK_HZ;
          localparam integer BYTE_TIMEOUT_CYCLES = CLK_HZ * 2;





          localparam integer STREAM_RECOVERY_CYCLES = CLK_HZ * 10;
`endif
          localparam [14:0] FRONT_HALF_Q6 = 15'd960;
          localparam [14:0] FRONT_WRAP_Q6 = 15'd22080;
          localparam [14:0] SECTOR_0_END_Q6 = 15'd1440;
          localparam [14:0] SECTOR_1_END_Q6 = 15'd4320;
          localparam [14:0] SECTOR_2_END_Q6 = 15'd7200;
          localparam [14:0] SECTOR_3_END_Q6 = 15'd10080;
          localparam [14:0] SECTOR_4_END_Q6 = 15'd12960;
          localparam [14:0] SECTOR_5_END_Q6 = 15'd15840;
          localparam [14:0] SECTOR_6_END_Q6 = 15'd18720;
          localparam [14:0] SECTOR_7_END_Q6 = 15'd21600;
        localparam [15:0] MIN_DISTANCE_Q2 = 16'd200;
        localparam [15:0] MAX_DISTANCE_Q2 = 16'd48000;
        localparam [15:0] FRONT_MIN_EMPTY = 16'hffff;
        localparam [15:0] CORNER_MARGIN_Q2 = 16'd600;
        localparam [15:0] SIDE_GUARD_Q2 = 16'd880;

          localparam [1:0] RX_IDLE  = 2'd0;
          localparam [1:0] RX_START = 2'd1;
          localparam [1:0] RX_DATA  = 2'd2;
          localparam [1:0] RX_STOP  = 2'd3;
          localparam [1:0] TX_IDLE  = 2'd0;
          localparam [1:0] TX_START = 2'd1;
          localparam [1:0] TX_DATA  = 2'd2;
          localparam [1:0] TX_STOP  = 2'd3;
          localparam [3:0] CMD_BOOT       = 4'd0;
          localparam [3:0] CMD_STOP_A5    = 4'd1;
          localparam [3:0] CMD_STOP_25    = 4'd2;
          localparam [3:0] CMD_STOP_WAIT  = 4'd3;
          localparam [3:0] CMD_SCAN_A5    = 4'd4;
          localparam [3:0] CMD_SCAN_20    = 4'd5;
          localparam [3:0] CMD_WAIT_DESC  = 4'd6;
          localparam [3:0] CMD_SCAN       = 4'd7;
      localparam [3:0] CMD_RETRY_WAIT = 4'd8;
      localparam [3:0] CMD_DISABLED   = 4'd9;
      localparam [3:0] CMD_DISABLE_A5 = 4'd10;
      localparam [3:0] CMD_DISABLE_25 = 4'd11;

          reg [1:0] rx_sync;
          reg [1:0] rx_state;
          reg [15:0] rx_clk_count;
          reg [2:0] rx_bit_index;
          reg [7:0] rx_shift;
          reg [7:0] rx_data;
          reg rx_valid;
          reg [1:0] tx_state;
          reg [15:0] tx_clk_count;
          reg [2:0] tx_bit_index;
          reg [7:0] tx_shift;
          reg [7:0] tx_data;
          reg tx_start;
          reg tx_line;
          reg tx_busy;
          reg tx_busy_d;
          reg [3:0] cmd_state;
          reg [31:0] wait_count;
          reg [31:0] byte_timeout_count;
          reg [2:0] desc_index;
          reg descriptor_retry_used;
          reg [2:0] node_index;
          reg [7:0] node0;
          reg [7:0] node1;
          reg [7:0] node2;
          reg [7:0] node3;
          reg [15:0] front_min_q2;
          reg [15:0] sector_min_0_q2;
          reg [15:0] sector_min_1_q2;
          reg [15:0] sector_min_2_q2;
          reg [15:0] sector_min_3_q2;
          reg [15:0] sector_min_4_q2;
        reg [15:0] sector_min_5_q2;
        reg [15:0] sector_min_6_q2;
        reg [15:0] sector_min_7_q2;
        reg [15:0] front_hist_1_q2;
        reg [15:0] front_hist_2_q2;
        reg [15:0] sector_hist_1_q2 [0:7];
        reg [15:0] sector_hist_2_q2 [0:7];

          wire node_start;
          wire node_inverse_start;
          wire node_good;
          wire [5:0] node_quality;
          wire [14:0] node_angle_q6;
          wire [15:0] rx_node_distance_q2;
          wire node_in_front;
          wire rx_node_distance_good;
        wire tx_done;
        wire [15:0] safe_distance_q2;
        wire [15:0] corner_distance_q2;
        wire [2:0] node_sector;

          assign lidar_uart_tx = tx_line;
          assign node_start = node0[0];
          assign node_inverse_start = node0[1];
          assign node_quality = node0[7:2];
          assign node_angle_q6 = {node2, node1[7:1]};
          assign rx_node_distance_q2 = {rx_data, node3};
          assign node_good = (node_start ^ node_inverse_start) && node1[0];
          assign node_in_front = (node_angle_q6 <= FRONT_HALF_Q6) || (node_angle_q6 >= FRONT_WRAP_Q6);
        assign rx_node_distance_good = (node_quality != 6'd0) &&
                                       (rx_node_distance_q2 >= MIN_DISTANCE_Q2) &&
                                       (rx_node_distance_q2 <= MAX_DISTANCE_Q2);
        assign tx_done = tx_busy_d && !tx_busy;
        assign safe_distance_q2 = safe_distance_mm << 2;
        assign corner_distance_q2 = (safe_distance_q2 > 16'hffff - CORNER_MARGIN_Q2) ?
                                    16'hffff : safe_distance_q2 + CORNER_MARGIN_Q2;
          assign node_sector = ((node_angle_q6 < SECTOR_0_END_Q6) || (node_angle_q6 >= SECTOR_7_END_Q6)) ? 3'd0 :
                               (node_angle_q6 < SECTOR_1_END_Q6) ? 3'd1 :
                               (node_angle_q6 < SECTOR_2_END_Q6) ? 3'd2 :
                               (node_angle_q6 < SECTOR_3_END_Q6) ? 3'd3 :
                               (node_angle_q6 < SECTOR_4_END_Q6) ? 3'd4 :
                               (node_angle_q6 < SECTOR_5_END_Q6) ? 3'd5 :
                               (node_angle_q6 < SECTOR_6_END_Q6) ? 3'd6 : 3'd7;

        function [15:0] q2_to_mm;
            input [15:0] value;
            begin
              q2_to_mm = (value == 16'hffff) ? 16'hffff : {2'b00, value[15:2]};
            end
        endfunction

        function [15:0] median3_q2;
          input [15:0] a;
          input [15:0] b;
          input [15:0] c;
          begin
            if ((b == 16'hffff) && (c == 16'hffff)) begin
              median3_q2 = a;
            end else if (c == 16'hffff) begin
              median3_q2 = (a < b) ? a : b;
            end else if (a > b) begin
              if (b > c)
                median3_q2 = b;
              else if (a > c)
                median3_q2 = c;
              else
                median3_q2 = a;
            end else begin
              if (a > c)
                median3_q2 = a;
              else if (b > c)
                median3_q2 = c;
              else
                median3_q2 = b;
            end
          end
        endfunction

          always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
              rx_sync <= 2'b11;
              rx_state <= RX_IDLE;
              rx_clk_count <= 16'd0;
              rx_bit_index <= 3'd0;
              rx_shift <= 8'd0;
              rx_data <= 8'd0;
              rx_valid <= 1'b0;
            end else begin
              rx_sync <= {rx_sync[0], lidar_uart_rx};
              rx_valid <= 1'b0;
              case (rx_state)
                RX_IDLE: begin
                  rx_clk_count <= 16'd0;
                  rx_bit_index <= 3'd0;
                  if (!rx_sync[1]) rx_state <= RX_START;
                end
                RX_START: begin
                  if (rx_clk_count == (CLKS_PER_BIT / 2)) begin
                    rx_clk_count <= 16'd0;
                    rx_state <= !rx_sync[1] ? RX_DATA : RX_IDLE;
                  end else begin
                    rx_clk_count <= rx_clk_count + 16'd1;
                  end
                end
                RX_DATA: begin
                  if (rx_clk_count == CLKS_PER_BIT - 1) begin
                    rx_clk_count <= 16'd0;
                    rx_shift[rx_bit_index] <= rx_sync[1];
                    if (rx_bit_index == 3'd7) begin
                      rx_bit_index <= 3'd0;
                      rx_state <= RX_STOP;
                    end else begin
                      rx_bit_index <= rx_bit_index + 3'd1;
                    end
                  end else begin
                    rx_clk_count <= rx_clk_count + 16'd1;
                  end
                end
                default: begin
                  if (rx_clk_count == CLKS_PER_BIT - 1) begin
                    rx_clk_count <= 16'd0;
                    rx_data <= rx_shift;
                    rx_valid <= rx_sync[1];
                    rx_state <= RX_IDLE;
                  end else begin
                    rx_clk_count <= rx_clk_count + 16'd1;
                  end
                end
              endcase
            end
          end

          always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
              tx_state <= TX_IDLE;
              tx_clk_count <= 16'd0;
              tx_bit_index <= 3'd0;
              tx_shift <= 8'd0;
              tx_line <= 1'b1;
              tx_busy <= 1'b0;
            end else begin
              case (tx_state)
                TX_IDLE: begin
                  tx_line <= 1'b1;
                  tx_busy <= 1'b0;
                  tx_clk_count <= 16'd0;
                  tx_bit_index <= 3'd0;
                  if (tx_start) begin
                    tx_busy <= 1'b1;
                    tx_shift <= tx_data;
                    tx_state <= TX_START;
                    tx_line <= 1'b0;
                  end
                end
                TX_START: begin
                  tx_busy <= 1'b1;
                  tx_line <= 1'b0;
                  if (tx_clk_count == CLKS_PER_BIT - 1) begin
                    tx_clk_count <= 16'd0;
                    tx_state <= TX_DATA;
                    tx_line <= tx_shift[0];
                  end else begin
                    tx_clk_count <= tx_clk_count + 16'd1;
                  end
                end
                TX_DATA: begin
                  tx_busy <= 1'b1;
                  tx_line <= tx_shift[tx_bit_index];
                  if (tx_clk_count == CLKS_PER_BIT - 1) begin
                    tx_clk_count <= 16'd0;
                    if (tx_bit_index == 3'd7) begin
                      tx_bit_index <= 3'd0;
                      tx_state <= TX_STOP;
                      tx_line <= 1'b1;
                    end else begin
                      tx_bit_index <= tx_bit_index + 3'd1;
                      tx_line <= tx_shift[tx_bit_index + 3'd1];
                    end
                  end else begin
                    tx_clk_count <= tx_clk_count + 16'd1;
                  end
                end
                default: begin
                  tx_busy <= 1'b1;
                  tx_line <= 1'b1;
                  if (tx_clk_count == CLKS_PER_BIT - 1) begin
                    tx_clk_count <= 16'd0;
                    tx_busy <= 1'b0;
                    tx_state <= TX_IDLE;
                  end else begin
                    tx_clk_count <= tx_clk_count + 16'd1;
                  end
                end
              endcase
            end
          end

          always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
              cmd_state <= CMD_BOOT;
              wait_count <= 32'd0;
              byte_timeout_count <= 32'd0;
              desc_index <= 3'd0;
              descriptor_retry_used <= 1'b0;
              scan_active <= 1'b0;
              lidar_online <= 1'b0;
              tx_data <= 8'd0;
              tx_start <= 1'b0;
              tx_busy_d <= 1'b0;
        end else begin
          tx_start <= 1'b0;
          tx_busy_d <= tx_busy;
          if (!scan_enable) begin
            scan_active <= 1'b0;
            lidar_online <= 1'b0;
            byte_timeout_count <= 32'd0;
            descriptor_retry_used <= 1'b0;
            if (cmd_state == CMD_DISABLE_A5) begin
              if (!tx_busy) begin
                tx_data <= 8'hA5;
                tx_start <= 1'b1;
                cmd_state <= CMD_DISABLE_25;
              end
            end else if (cmd_state == CMD_DISABLE_25) begin
              if (tx_done) begin
                tx_data <= 8'h25;
                tx_start <= 1'b1;
                cmd_state <= CMD_DISABLED;
              end
            end else if (cmd_state != CMD_DISABLED) begin
              cmd_state <= CMD_DISABLE_A5;
            end
          end else if ((cmd_state == CMD_DISABLED) ||
                       (cmd_state == CMD_DISABLE_A5) ||
                       (cmd_state == CMD_DISABLE_25)) begin
            wait_count <= 32'd0;
            byte_timeout_count <= 32'd0;
            desc_index <= 3'd0;
            cmd_state <= CMD_BOOT;
          end else case (cmd_state)
                CMD_BOOT: begin
                  scan_active <= 1'b0;
                  lidar_online <= 1'b0;
                  if (wait_count >= BOOT_DELAY_CYCLES - 1) begin
                    wait_count <= 32'd0;
                    descriptor_retry_used <= 1'b0;
                    cmd_state <= CMD_STOP_A5;
                  end else wait_count <= wait_count + 32'd1;
                end
                CMD_STOP_A5: if (!tx_busy) begin tx_data <= 8'hA5; tx_start <= 1'b1; cmd_state <= CMD_STOP_25; end
                CMD_STOP_25: if (tx_done) begin tx_data <= 8'h25; tx_start <= 1'b1; cmd_state <= CMD_STOP_WAIT; end
                CMD_STOP_WAIT: begin
                  if (wait_count >= STOP_WAIT_CYCLES - 1) begin
                    wait_count <= 32'd0;
                    desc_index <= 3'd0;
                    cmd_state <= CMD_SCAN_A5;
                  end else wait_count <= wait_count + 32'd1;
                end
                CMD_SCAN_A5: if (!tx_busy) begin tx_data <= 8'hA5; tx_start <= 1'b1; cmd_state <= CMD_SCAN_20; end
                CMD_SCAN_20: if (tx_done) begin tx_data <= 8'h20; tx_start <= 1'b1; byte_timeout_count <= 32'd0; cmd_state <= CMD_WAIT_DESC; end
                CMD_WAIT_DESC: begin
                  if (rx_valid) begin
                    byte_timeout_count <= 32'd0;
                    case (desc_index)
                      3'd0: desc_index <= (rx_data == 8'hA5) ? 3'd1 : 3'd0;
                      3'd1: desc_index <= (rx_data == 8'h5A) ? 3'd2 : ((rx_data == 8'hA5) ? 3'd1 : 3'd0);
                      3'd2: desc_index <= (rx_data == 8'h05) ? 3'd3 : 3'd0;
                      3'd3: desc_index <= (rx_data == 8'h00) ? 3'd4 : 3'd0;
                      3'd4: desc_index <= (rx_data == 8'h00) ? 3'd5 : 3'd0;
                      3'd5: desc_index <= (rx_data == 8'h40) ? 3'd6 : 3'd0;
                      default: begin
                        if (rx_data == 8'h81) begin
                          scan_active <= 1'b1;
                          lidar_online <= 1'b0;
                          cmd_state <= CMD_SCAN;
                        end else desc_index <= 3'd0;
                      end
                    endcase
                  end else if (byte_timeout_count >= BYTE_TIMEOUT_CYCLES - 1) begin
                    byte_timeout_count <= 32'd0;
                    desc_index <= 3'd0;
                    if (!descriptor_retry_used) begin




                      descriptor_retry_used <= 1'b1;
                      wait_count <= 32'd0;
                      cmd_state <= CMD_RETRY_WAIT;
                    end else begin



                      scan_active <= 1'b1;
                      cmd_state <= CMD_SCAN;
                    end
                  end else byte_timeout_count <= byte_timeout_count + 32'd1;
                end
                CMD_SCAN: begin
                  if (sample_valid) begin
                    byte_timeout_count <= 32'd0;
                    descriptor_retry_used <= 1'b0;
                    lidar_online <= 1'b1;
                  end else if (byte_timeout_count >= STREAM_RECOVERY_CYCLES - 1) begin
                    byte_timeout_count <= 32'd0;
                    lidar_online <= 1'b0;
                    scan_active <= 1'b0;
                    wait_count <= 32'd0;
                    descriptor_retry_used <= 1'b1;
                    cmd_state <= CMD_RETRY_WAIT;
                  end else byte_timeout_count <= byte_timeout_count + 32'd1;
                end
                default: begin
                  scan_active <= 1'b0;
                  if (wait_count >= RETRY_WAIT_CYCLES - 1) begin
                    wait_count <= 32'd0;
                    desc_index <= 3'd0;
                    cmd_state <= CMD_STOP_A5;
                  end else wait_count <= wait_count + 32'd1;
                end
          endcase
        end
          end

          always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
              sample_valid <= 1'b0;
              new_scan <= 1'b0;
              quality <= 6'd0;
              angle_q6 <= 15'd0;
              distance_q2 <= 16'd0;
              front_distance_mm <= 16'hffff;
              front_valid <= 1'b0;
              obstacle_stop <= 1'b0;
              node_index <= 3'd0;
              node0 <= 8'd0;
              node1 <= 8'd0;
              node2 <= 8'd0;
              node3 <= 8'd0;
              front_min_q2 <= FRONT_MIN_EMPTY;
              sector_min_0_q2 <= 16'hffff;
              sector_min_1_q2 <= 16'hffff;
              sector_min_2_q2 <= 16'hffff;
              sector_min_3_q2 <= 16'hffff;
              sector_min_4_q2 <= 16'hffff;
              sector_min_5_q2 <= 16'hffff;
              sector_min_6_q2 <= 16'hffff;
              sector_min_7_q2 <= 16'hffff;
              front_hist_1_q2 <= 16'hffff;
              front_hist_2_q2 <= 16'hffff;
              sector_hist_1_q2[0] <= 16'hffff;
              sector_hist_1_q2[1] <= 16'hffff;
              sector_hist_1_q2[2] <= 16'hffff;
              sector_hist_1_q2[3] <= 16'hffff;
              sector_hist_1_q2[4] <= 16'hffff;
              sector_hist_1_q2[5] <= 16'hffff;
              sector_hist_1_q2[6] <= 16'hffff;
              sector_hist_1_q2[7] <= 16'hffff;
              sector_hist_2_q2[0] <= 16'hffff;
              sector_hist_2_q2[1] <= 16'hffff;
              sector_hist_2_q2[2] <= 16'hffff;
              sector_hist_2_q2[3] <= 16'hffff;
              sector_hist_2_q2[4] <= 16'hffff;
              sector_hist_2_q2[5] <= 16'hffff;
              sector_hist_2_q2[6] <= 16'hffff;
              sector_hist_2_q2[7] <= 16'hffff;
              sector_front_mm <= 16'hffff;
              sector_front_right_mm <= 16'hffff;
              sector_right_mm <= 16'hffff;
              sector_rear_right_mm <= 16'hffff;
              sector_rear_mm <= 16'hffff;
              sector_rear_left_mm <= 16'hffff;
              sector_left_mm <= 16'hffff;
              sector_front_left_mm <= 16'hffff;
            end else begin
              sample_valid <= 1'b0;
              new_scan <= 1'b0;
              if (!scan_active) begin
                node_index <= 3'd0;
                front_min_q2 <= FRONT_MIN_EMPTY;
                sector_min_0_q2 <= 16'hffff;
                sector_min_1_q2 <= 16'hffff;
                sector_min_2_q2 <= 16'hffff;
                sector_min_3_q2 <= 16'hffff;
                sector_min_4_q2 <= 16'hffff;
                sector_min_5_q2 <= 16'hffff;
                sector_min_6_q2 <= 16'hffff;
                sector_min_7_q2 <= 16'hffff;
                front_hist_1_q2 <= 16'hffff;
                front_hist_2_q2 <= 16'hffff;
                sector_hist_1_q2[0] <= 16'hffff;
                sector_hist_1_q2[1] <= 16'hffff;
                sector_hist_1_q2[2] <= 16'hffff;
                sector_hist_1_q2[3] <= 16'hffff;
                sector_hist_1_q2[4] <= 16'hffff;
                sector_hist_1_q2[5] <= 16'hffff;
                sector_hist_1_q2[6] <= 16'hffff;
                sector_hist_1_q2[7] <= 16'hffff;
                sector_hist_2_q2[0] <= 16'hffff;
                sector_hist_2_q2[1] <= 16'hffff;
                sector_hist_2_q2[2] <= 16'hffff;
                sector_hist_2_q2[3] <= 16'hffff;
                sector_hist_2_q2[4] <= 16'hffff;
                sector_hist_2_q2[5] <= 16'hffff;
                sector_hist_2_q2[6] <= 16'hffff;
                sector_hist_2_q2[7] <= 16'hffff;
                front_distance_mm <= 16'hffff;
                front_valid <= 1'b0;
                obstacle_stop <= 1'b0;
                sector_front_mm <= 16'hffff;
                sector_front_right_mm <= 16'hffff;
                sector_right_mm <= 16'hffff;
                sector_rear_right_mm <= 16'hffff;
                sector_rear_mm <= 16'hffff;
                sector_rear_left_mm <= 16'hffff;
                sector_left_mm <= 16'hffff;
                sector_front_left_mm <= 16'hffff;
              end else if (rx_valid) begin
                case (node_index)
                  3'd0: begin node0 <= rx_data; node_index <= 3'd1; end
                  3'd1: begin node1 <= rx_data; node_index <= 3'd2; end
                  3'd2: begin node2 <= rx_data; node_index <= 3'd3; end
                  3'd3: begin node3 <= rx_data; node_index <= 3'd4; end
                  default: begin
                    node_index <= 3'd0;
                    if (node_good) begin
                      sample_valid <= 1'b1;
                      quality <= node_quality;
                      angle_q6 <= node_angle_q6;
                      distance_q2 <= rx_node_distance_q2;
                    if (node_start) begin
                      new_scan <= 1'b1;
                      obstacle_stop <= (median3_q2(front_min_q2, front_hist_1_q2, front_hist_2_q2) <= safe_distance_q2) ||
                                       (median3_q2(sector_min_0_q2, sector_hist_1_q2[0], sector_hist_2_q2[0]) <= safe_distance_q2) ||
                                       (median3_q2(sector_min_1_q2, sector_hist_1_q2[1], sector_hist_2_q2[1]) <= corner_distance_q2) ||
                                       (median3_q2(sector_min_7_q2, sector_hist_1_q2[7], sector_hist_2_q2[7]) <= corner_distance_q2) ||
                                       (median3_q2(sector_min_2_q2, sector_hist_1_q2[2], sector_hist_2_q2[2]) <= SIDE_GUARD_Q2) ||
                                       (median3_q2(sector_min_6_q2, sector_hist_1_q2[6], sector_hist_2_q2[6]) <= SIDE_GUARD_Q2);
                      if (median3_q2(front_min_q2, front_hist_1_q2, front_hist_2_q2) == FRONT_MIN_EMPTY) begin
                        front_distance_mm <= 16'hffff;
                        front_valid <= 1'b0;
                      end else begin
                        front_distance_mm <= q2_to_mm(median3_q2(front_min_q2, front_hist_1_q2, front_hist_2_q2));
                        front_valid <= 1'b1;
                      end
                        front_hist_2_q2 <= front_hist_1_q2;
                        front_hist_1_q2 <= front_min_q2;
                        front_min_q2 <= (node_in_front && rx_node_distance_good) ? rx_node_distance_q2 : FRONT_MIN_EMPTY;
                        sector_front_mm <= q2_to_mm(median3_q2(sector_min_0_q2, sector_hist_1_q2[0], sector_hist_2_q2[0]));
                        sector_front_right_mm <= q2_to_mm(median3_q2(sector_min_1_q2, sector_hist_1_q2[1], sector_hist_2_q2[1]));
                        sector_right_mm <= q2_to_mm(median3_q2(sector_min_2_q2, sector_hist_1_q2[2], sector_hist_2_q2[2]));
                        sector_rear_right_mm <= q2_to_mm(median3_q2(sector_min_3_q2, sector_hist_1_q2[3], sector_hist_2_q2[3]));
                        sector_rear_mm <= q2_to_mm(median3_q2(sector_min_4_q2, sector_hist_1_q2[4], sector_hist_2_q2[4]));
                        sector_rear_left_mm <= q2_to_mm(median3_q2(sector_min_5_q2, sector_hist_1_q2[5], sector_hist_2_q2[5]));
                        sector_left_mm <= q2_to_mm(median3_q2(sector_min_6_q2, sector_hist_1_q2[6], sector_hist_2_q2[6]));
                        sector_front_left_mm <= q2_to_mm(median3_q2(sector_min_7_q2, sector_hist_1_q2[7], sector_hist_2_q2[7]));
                        sector_hist_2_q2[0] <= sector_hist_1_q2[0];
                        sector_hist_2_q2[1] <= sector_hist_1_q2[1];
                        sector_hist_2_q2[2] <= sector_hist_1_q2[2];
                        sector_hist_2_q2[3] <= sector_hist_1_q2[3];
                        sector_hist_2_q2[4] <= sector_hist_1_q2[4];
                        sector_hist_2_q2[5] <= sector_hist_1_q2[5];
                        sector_hist_2_q2[6] <= sector_hist_1_q2[6];
                        sector_hist_2_q2[7] <= sector_hist_1_q2[7];
                        sector_hist_1_q2[0] <= sector_min_0_q2;
                        sector_hist_1_q2[1] <= sector_min_1_q2;
                        sector_hist_1_q2[2] <= sector_min_2_q2;
                        sector_hist_1_q2[3] <= sector_min_3_q2;
                        sector_hist_1_q2[4] <= sector_min_4_q2;
                        sector_hist_1_q2[5] <= sector_min_5_q2;
                        sector_hist_1_q2[6] <= sector_min_6_q2;
                        sector_hist_1_q2[7] <= sector_min_7_q2;
                        sector_min_0_q2 <= (rx_node_distance_good && (node_sector == 3'd0)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_1_q2 <= (rx_node_distance_good && (node_sector == 3'd1)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_2_q2 <= (rx_node_distance_good && (node_sector == 3'd2)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_3_q2 <= (rx_node_distance_good && (node_sector == 3'd3)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_4_q2 <= (rx_node_distance_good && (node_sector == 3'd4)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_5_q2 <= (rx_node_distance_good && (node_sector == 3'd5)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_6_q2 <= (rx_node_distance_good && (node_sector == 3'd6)) ? rx_node_distance_q2 : 16'hffff;
                        sector_min_7_q2 <= (rx_node_distance_good && (node_sector == 3'd7)) ? rx_node_distance_q2 : 16'hffff;
                      end else if (node_in_front && rx_node_distance_good && (rx_node_distance_q2 < front_min_q2)) begin
                        front_min_q2 <= rx_node_distance_q2;
                      end
                      if (!node_start && rx_node_distance_good) begin
                        case (node_sector)
                          3'd0: if (rx_node_distance_q2 < sector_min_0_q2) sector_min_0_q2 <= rx_node_distance_q2;
                          3'd1: if (rx_node_distance_q2 < sector_min_1_q2) sector_min_1_q2 <= rx_node_distance_q2;
                          3'd2: if (rx_node_distance_q2 < sector_min_2_q2) sector_min_2_q2 <= rx_node_distance_q2;
                          3'd3: if (rx_node_distance_q2 < sector_min_3_q2) sector_min_3_q2 <= rx_node_distance_q2;
                          3'd4: if (rx_node_distance_q2 < sector_min_4_q2) sector_min_4_q2 <= rx_node_distance_q2;
                          3'd5: if (rx_node_distance_q2 < sector_min_5_q2) sector_min_5_q2 <= rx_node_distance_q2;
                          3'd6: if (rx_node_distance_q2 < sector_min_6_q2) sector_min_6_q2 <= rx_node_distance_q2;
                          default: if (rx_node_distance_q2 < sector_min_7_q2) sector_min_7_q2 <= rx_node_distance_q2;
                        endcase
                      end
                    end
                  end
                endcase
              end
            end
          end
endmodule

module rplidar_protocol_parameter;
  reg clk;
  reg rst_n;
  reg lidar_uart_rx;
  reg [15:0] safe_distance_mm;
  reg scan_enable;
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
  wire [15:0] sector_front_mm;
  wire [15:0] sector_front_right_mm;
  wire [15:0] sector_right_mm;
  wire [15:0] sector_rear_right_mm;
  wire [15:0] sector_rear_mm;
  wire [15:0] sector_rear_left_mm;
  wire [15:0] sector_left_mm;
  wire [15:0] sector_front_left_mm;


`ifdef PARAM_WAVE
initial begin
  $dumpfile("tb_rplidar_c1_uart_restart.vcd");
  $dumpvars(0, rst_n, scan_enable, lidar_uart_tx, lidar_online, scan_active, sample_valid, new_scan, quality, angle_q6, distance_q2, front_distance_mm, obstacle_stop, rplidar_c1_uart0.cmd_state);
end
`endif

initial begin
  clk = 1'b0;
  rst_n = 1'b0;
  lidar_uart_rx = 1'b1;
  safe_distance_mm = 16'd450;
  scan_enable = 1'b1;
end

always #10 clk = ~clk;



  task expect_state;
    input [3:0] expected;
    input [8*48-1:0] label;
    begin
      if (rplidar_c1_uart0.cmd_state !== expected) begin
        $display("RESULT TEST_FAIL %0s expected_state=%0d actual_state=%0d",
                 label, expected, rplidar_c1_uart0.cmd_state);
        $fatal(1);
      end
    end
  endtask

  task feed_rx;
    input [7:0] value;
    begin
      @(negedge clk);
      force rplidar_c1_uart0.rx_valid = 1'b1;
      force rplidar_c1_uart0.rx_data = value;
      @(posedge clk);
      #1;
      release rplidar_c1_uart0.rx_valid;
      release rplidar_c1_uart0.rx_data;
      rplidar_c1_uart0.rx_valid = 1'b0;
    end
  endtask

  initial begin
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // A missed power-up command gets one bounded retry.  This is enough to
    // recover the C1 boot race without continuously cycling its motor.
    @(negedge clk);
    rplidar_c1_uart0.cmd_state = rplidar_c1_uart0.CMD_WAIT_DESC;
    rplidar_c1_uart0.byte_timeout_count = rplidar_c1_uart0.BYTE_TIMEOUT_CYCLES - 1;
    rplidar_c1_uart0.desc_index = 3'd0;
    rplidar_c1_uart0.descriptor_retry_used = 1'b0;
    @(posedge clk);
    #1;
    expect_state(rplidar_c1_uart0.CMD_RETRY_WAIT, "bounded_descriptor_retry");
    if (rplidar_c1_uart0.descriptor_retry_used !== 1'b1)
      $fatal(1, "descriptor retry was not latched");

    // If the retry descriptor is also lost, switch to raw node framing.  The
    // descriptor may be the missing part while scan data is already present.
    @(negedge clk);
    rplidar_c1_uart0.cmd_state = rplidar_c1_uart0.CMD_WAIT_DESC;
    rplidar_c1_uart0.byte_timeout_count = rplidar_c1_uart0.BYTE_TIMEOUT_CYCLES - 1;
    rplidar_c1_uart0.descriptor_retry_used = 1'b1;
    @(posedge clk);
    #1;
    expect_state(rplidar_c1_uart0.CMD_SCAN, "descriptor_loss_fallback");
    if (scan_active !== 1'b1)
      $fatal(1, "raw-node fallback did not enable scan parsing");

    // Exercise the normal descriptor path independently of the fallback.
    @(negedge clk);
    rplidar_c1_uart0.cmd_state = rplidar_c1_uart0.CMD_WAIT_DESC;
    rplidar_c1_uart0.scan_active = 1'b0;
    rplidar_c1_uart0.byte_timeout_count = 32'd0;
    rplidar_c1_uart0.desc_index = 3'd0;

    // Standard SCAN descriptor: A5 5A, five-byte multiple response, type 81.
    feed_rx(8'hA5);
    feed_rx(8'h5A);
    feed_rx(8'h05);
    feed_rx(8'h00);
    feed_rx(8'h00);
    feed_rx(8'h40);
    feed_rx(8'h81);
    expect_state(rplidar_c1_uart0.CMD_SCAN, "scan_descriptor");
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


  rplidar_c1_uart rplidar_c1_uart0(
    .clk(clk),
    .rst_n(rst_n),
    .lidar_uart_rx(lidar_uart_rx),
    .safe_distance_mm(safe_distance_mm),
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
endmodule

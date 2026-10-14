module vehicle_debug_uart (
  input wire clk,
  input wire rst_n,
  input wire bt_connected,
  input wire lidar_enable,
  input wire lidar_scan_active,
  input wire lidar_online,
  input wire uwb_position_online,
  input wire uwb_position_good,
  input wire uwb_start_ack_seen,
  input wire signed [15:0] uwb_position_x_cm,
  input wire signed [15:0] uwb_position_y_cm,
  input wire [15:0] uwb_packet_count,
  input wire [15:0] uwb_crc_error_count,
  input wire [15:0] uwb_start_request_count,
  input wire [15:0] map_pose_x_mm,
  input wire [15:0] map_pose_y_mm,
  input wire [9:0] map_pose_heading,
  input wire [10:0] flow_correction,
  input wire [10:0] yaw_correction,
  input wire [319:0] motion_telemetry_data,
  output reg debug_uart_tx
);

`ifdef VEHICLE_DEBUG_SIM_FAST
  localparam integer REPORT_PERIOD_CYCLES = 80;
  localparam integer CLKS_PER_BIT = 4;
`else
  localparam integer REPORT_PERIOD_CYCLES = 25000000; // 500 ms
  localparam integer CLKS_PER_BIT = 434;              // 115200 at 50 MHz
`endif
  localparam [4:0] LAST_BYTE = 5'd21;
  wire [7:0] motion_status = motion_telemetry_data[7:0];

  reg [7:0] snapshot_flags;
  reg signed [15:0] snapshot_uwb_x_cm;
  reg signed [15:0] snapshot_uwb_y_cm;
  reg [15:0] snapshot_packet_count;
  reg [15:0] snapshot_crc_error_count;
  reg [15:0] snapshot_start_request_count;
  reg [15:0] snapshot_pose_x_mm;
  reg [15:0] snapshot_pose_y_mm;
  reg [9:0] snapshot_pose_heading;
  reg [7:0] snapshot_motion_status;

  // Compact binary diagnostics keep the production design routable.  Every
  // 22-byte frame starts A5 5A 02 and ends 0A; all 16-bit values are little
  // endian.  The two-byte sync word lets a host recover after opening COM20
  // in the middle of a frame without the large ASCII/hex conversion network.
  function [7:0] report_byte;
    input [4:0] index;
    begin
      case (index)
        5'd0: report_byte = 8'hA5;
        5'd1: report_byte = 8'h5A;
        5'd2: report_byte = 8'h02;
        5'd3: report_byte = snapshot_flags;
        5'd4: report_byte = snapshot_uwb_x_cm[7:0];
        5'd5: report_byte = snapshot_uwb_x_cm[15:8];
        5'd6: report_byte = snapshot_uwb_y_cm[7:0];
        5'd7: report_byte = snapshot_uwb_y_cm[15:8];
        5'd8: report_byte = snapshot_packet_count[7:0];
        5'd9: report_byte = snapshot_packet_count[15:8];
        5'd10: report_byte = snapshot_crc_error_count[7:0];
        5'd11: report_byte = snapshot_crc_error_count[15:8];
        5'd12: report_byte = snapshot_start_request_count[7:0];
        5'd13: report_byte = snapshot_start_request_count[15:8];
        5'd14: report_byte = snapshot_pose_x_mm[7:0];
        5'd15: report_byte = snapshot_pose_x_mm[15:8];
        5'd16: report_byte = snapshot_pose_y_mm[7:0];
        5'd17: report_byte = snapshot_pose_y_mm[15:8];
        5'd18: report_byte = snapshot_pose_heading[7:0];
        5'd19: report_byte = {6'd0, snapshot_pose_heading[9:8]};
        5'd20: report_byte = snapshot_motion_status;
        default: report_byte = 8'h0A;
      endcase
    end
  endfunction

  reg [24:0] report_period_count;
  reg report_sending;
  reg [4:0] report_index;
  reg [7:0] tx_data;
  reg tx_start;
  reg tx_busy;
  reg tx_busy_d;
  reg [1:0] tx_state;
  reg [15:0] tx_clock_count;
  reg [2:0] tx_bit_index;
  reg [7:0] tx_shift;
  wire tx_done = tx_busy_d && !tx_busy;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      debug_uart_tx <= 1'b1;
      tx_busy <= 1'b0;
      tx_busy_d <= 1'b0;
      tx_state <= 2'd0;
      tx_clock_count <= 16'd0;
      tx_bit_index <= 3'd0;
      tx_shift <= 8'd0;
    end else begin
      tx_busy_d <= tx_busy;
      case (tx_state)
        2'd0: begin
          debug_uart_tx <= 1'b1;
          tx_busy <= 1'b0;
          tx_clock_count <= 16'd0;
          tx_bit_index <= 3'd0;
          if (tx_start) begin
            tx_shift <= tx_data;
            debug_uart_tx <= 1'b0;
            tx_busy <= 1'b1;
            tx_state <= 2'd1;
          end
        end
        2'd1: begin
          debug_uart_tx <= 1'b0;
          tx_busy <= 1'b1;
          if (tx_clock_count >= CLKS_PER_BIT - 1) begin
            tx_clock_count <= 16'd0;
            debug_uart_tx <= tx_shift[0];
            tx_state <= 2'd2;
          end else tx_clock_count <= tx_clock_count + 16'd1;
        end
        2'd2: begin
          debug_uart_tx <= tx_shift[tx_bit_index];
          tx_busy <= 1'b1;
          if (tx_clock_count >= CLKS_PER_BIT - 1) begin
            tx_clock_count <= 16'd0;
            if (tx_bit_index == 3'd7) begin
              tx_bit_index <= 3'd0;
              debug_uart_tx <= 1'b1;
              tx_state <= 2'd3;
            end else begin
              tx_bit_index <= tx_bit_index + 3'd1;
              debug_uart_tx <= tx_shift[tx_bit_index + 3'd1];
            end
          end else tx_clock_count <= tx_clock_count + 16'd1;
        end
        default: begin
          debug_uart_tx <= 1'b1;
          tx_busy <= 1'b1;
          if (tx_clock_count >= CLKS_PER_BIT - 1) begin
            tx_clock_count <= 16'd0;
            tx_busy <= 1'b0;
            tx_state <= 2'd0;
          end else tx_clock_count <= tx_clock_count + 16'd1;
        end
      endcase
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      snapshot_flags <= 8'd0;
      snapshot_uwb_x_cm <= 16'sd0;
      snapshot_uwb_y_cm <= 16'sd0;
      snapshot_packet_count <= 16'd0;
      snapshot_crc_error_count <= 16'd0;
      snapshot_start_request_count <= 16'd0;
      snapshot_pose_x_mm <= 16'd0;
      snapshot_pose_y_mm <= 16'd0;
      snapshot_pose_heading <= 10'd0;
      snapshot_motion_status <= 8'd0;
      report_period_count <= 25'd0;
      report_sending <= 1'b0;
      report_index <= 5'd0;
      tx_data <= 8'd0;
      tx_start <= 1'b0;
    end else begin
      tx_start <= 1'b0;
      if (!report_sending) begin
        if (report_period_count >= REPORT_PERIOD_CYCLES - 1) begin
          report_period_count <= 25'd0;
          report_sending <= 1'b1;
          report_index <= 5'd0;
          snapshot_flags <= {
              bt_connected, lidar_enable, lidar_scan_active, lidar_online,
              uwb_position_online, uwb_position_good, uwb_start_ack_seen,
              motion_status[0]
          };
          snapshot_uwb_x_cm <= uwb_position_x_cm;
          snapshot_uwb_y_cm <= uwb_position_y_cm;
          snapshot_packet_count <= uwb_packet_count;
          snapshot_crc_error_count <= uwb_crc_error_count;
          snapshot_start_request_count <= uwb_start_request_count;
          snapshot_pose_x_mm <= map_pose_x_mm;
          snapshot_pose_y_mm <= map_pose_y_mm;
          snapshot_pose_heading <= map_pose_heading;
          snapshot_motion_status <= motion_status;
          tx_data <= 8'hA5;
          tx_start <= 1'b1;
        end else report_period_count <= report_period_count + 25'd1;
      end else if (tx_done) begin
        if (report_index >= LAST_BYTE) begin
          report_sending <= 1'b0;
          report_index <= 5'd0;
        end else begin
          report_index <= report_index + 5'd1;
          tx_data <= report_byte(report_index + 5'd1);
          tx_start <= 1'b1;
        end
      end
    end
  end

endmodule

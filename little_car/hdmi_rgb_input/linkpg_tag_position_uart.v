module linkpg_tag_position_uart (
  input wire clk,
  input wire rst_n,
  input wire uwb_uart_rx,
  output wire uwb_uart_tx,
  output reg position_valid,
  output reg position_online,
  output reg position_good,
  output reg signed [15:0] position_x_cm,
  output reg signed [15:0] position_y_cm,
  output reg signed [15:0] position_z_cm,
  output reg [15:0] distance_valid_mask,
  output reg [15:0] distance_a_cm,
  output reg [15:0] distance_b_cm,
  output reg [15:0] distance_c_cm,
  output reg [15:0] distance_d_cm,
  output reg [15:0] rx_packet_count,
  output reg [15:0] rx_crc_error_count,
  output reg [15:0] rx_framing_error_count,
  output reg [15:0] start_request_count,
  output reg start_ack_seen
);

  parameter integer CLK_HZ = 50000000;
  localparam integer BAUD = 115200;
`ifdef LINKPG_POSITION_SIM_FAST
  localparam integer CLKS_PER_BIT = 4;
  localparam integer START_DELAY_CYCLES = 80;
  localparam integer START_RETRY_CYCLES = 160;
  localparam integer LINK_TIMEOUT_CYCLES = 400;
  localparam integer FRAME_TIMEOUT_CYCLES = 80;
`else
  localparam integer CLKS_PER_BIT = (CLK_HZ + (BAUD / 2)) / BAUD;
  localparam integer START_DELAY_CYCLES = CLK_HZ * 3;
  localparam integer START_RETRY_CYCLES = CLK_HZ * 3;
  localparam integer LINK_TIMEOUT_CYCLES = CLK_HZ * 2;
  localparam integer FRAME_TIMEOUT_CYCLES = CLK_HZ / 100;
`endif

  localparam [1:0] UART_IDLE  = 2'd0;
  localparam [1:0] UART_START = 2'd1;
  localparam [1:0] UART_DATA  = 2'd2;
  localparam [1:0] UART_STOP  = 2'd3;

  function [15:0] crc16_modbus_bit;
    input [15:0] value;
    begin
      crc16_modbus_bit = value[0] ?
          ((value >> 1) ^ 16'hA001) : (value >> 1);
    end
  endfunction

  function [16:0] signed_abs_diff16;
    input signed [15:0] a;
    input signed [15:0] b;
    reg signed [16:0] difference;
    begin
      difference = {a[15], a} - {b[15], b};
      signed_abs_diff16 = difference[16] ? -difference : difference;
    end
  endfunction

  function signed [15:0] average_signed16;
    input signed [15:0] a;
    input signed [15:0] b;
    reg signed [16:0] sum;
    begin
      sum = $signed({a[15], a}) + $signed({b[15], b});
      average_signed16 = sum >>> 1;
    end
  endfunction

  function [7:0] start_frame_byte;
    input [3:0] index;
    begin
      case (index)
        4'd0: start_frame_byte = 8'h01;
        4'd1: start_frame_byte = 8'h10;
        4'd2: start_frame_byte = 8'h00;
        4'd3: start_frame_byte = 8'h3B;
        4'd4: start_frame_byte = 8'h00;
        4'd5: start_frame_byte = 8'h01;
        4'd6: start_frame_byte = 8'h02;
        4'd7: start_frame_byte = 8'h00;
        4'd8: start_frame_byte = 8'h04;
        4'd9: start_frame_byte = 8'hA3;
        default: start_frame_byte = 8'h18;
      endcase
    end
  endfunction

  reg [1:0] rx_sync;
  reg [1:0] rx_state;
  reg [15:0] rx_clock_count;
  reg [2:0] rx_bit_index;
  reg [7:0] rx_shift;
  reg [7:0] rx_byte;
  reg rx_valid;
  reg uart_framing_error_pulse;

  // The tag UART is asynchronous to the 50 MHz system clock.  Sample each
  // 115200 8N1 bit through a two-flop synchronizer at its nominal center.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_sync <= 2'b11;
      rx_state <= UART_IDLE;
      rx_clock_count <= 16'd0;
      rx_bit_index <= 3'd0;
      rx_shift <= 8'd0;
      rx_byte <= 8'd0;
      rx_valid <= 1'b0;
      uart_framing_error_pulse <= 1'b0;
    end else begin
      rx_sync <= {rx_sync[0], uwb_uart_rx};
      rx_valid <= 1'b0;
      uart_framing_error_pulse <= 1'b0;
      case (rx_state)
        UART_IDLE: begin
          rx_clock_count <= 16'd0;
          rx_bit_index <= 3'd0;
          if (!rx_sync[1]) rx_state <= UART_START;
        end
        UART_START: begin
          if (rx_clock_count >= (CLKS_PER_BIT / 2) - 1) begin
            rx_clock_count <= 16'd0;
            rx_state <= !rx_sync[1] ? UART_DATA : UART_IDLE;
          end else rx_clock_count <= rx_clock_count + 16'd1;
        end
        UART_DATA: begin
          if (rx_clock_count >= CLKS_PER_BIT - 1) begin
            rx_clock_count <= 16'd0;
            rx_shift[rx_bit_index] <= rx_sync[1];
            if (rx_bit_index == 3'd7) begin
              rx_bit_index <= 3'd0;
              rx_state <= UART_STOP;
            end else rx_bit_index <= rx_bit_index + 3'd1;
          end else rx_clock_count <= rx_clock_count + 16'd1;
        end
        default: begin
          if (rx_clock_count >= CLKS_PER_BIT - 1) begin
            rx_clock_count <= 16'd0;
            rx_state <= UART_IDLE;
            if (rx_sync[1]) begin
              rx_byte <= rx_shift;
              rx_valid <= 1'b1;
            end else uart_framing_error_pulse <= 1'b1;
          end else rx_clock_count <= rx_clock_count + 16'd1;
        end
      endcase
    end
  end

  reg [1:0] tx_state;
  reg [15:0] tx_clock_count;
  reg [2:0] tx_bit_index;
  reg [7:0] tx_shift;
  reg [7:0] tx_data;
  reg tx_start;
  reg tx_line;
  reg tx_busy;
  assign uwb_uart_tx = tx_line;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_state <= UART_IDLE;
      tx_clock_count <= 16'd0;
      tx_bit_index <= 3'd0;
      tx_shift <= 8'd0;
      tx_line <= 1'b1;
      tx_busy <= 1'b0;
    end else begin
      case (tx_state)
        UART_IDLE: begin
          tx_line <= 1'b1;
          tx_busy <= 1'b0;
          tx_clock_count <= 16'd0;
          tx_bit_index <= 3'd0;
          if (tx_start) begin
            tx_shift <= tx_data;
            tx_line <= 1'b0;
            tx_busy <= 1'b1;
            tx_state <= UART_START;
          end
        end
        UART_START: begin
          tx_line <= 1'b0;
          tx_busy <= 1'b1;
          if (tx_clock_count >= CLKS_PER_BIT - 1) begin
            tx_clock_count <= 16'd0;
            tx_line <= tx_shift[0];
            tx_state <= UART_DATA;
          end else tx_clock_count <= tx_clock_count + 16'd1;
        end
        UART_DATA: begin
          tx_line <= tx_shift[tx_bit_index];
          tx_busy <= 1'b1;
          if (tx_clock_count >= CLKS_PER_BIT - 1) begin
            tx_clock_count <= 16'd0;
            if (tx_bit_index == 3'd7) begin
              tx_bit_index <= 3'd0;
              tx_line <= 1'b1;
              tx_state <= UART_STOP;
            end else begin
              tx_bit_index <= tx_bit_index + 3'd1;
              tx_line <= tx_shift[tx_bit_index + 3'd1];
            end
          end else tx_clock_count <= tx_clock_count + 16'd1;
        end
        default: begin
          tx_line <= 1'b1;
          tx_busy <= 1'b1;
          if (tx_clock_count >= CLKS_PER_BIT - 1) begin
            tx_clock_count <= 16'd0;
            tx_busy <= 1'b0;
            tx_state <= UART_IDLE;
          end else tx_clock_count <= tx_clock_count + 16'd1;
        end
      endcase
    end
  end

  reg [27:0] start_wait_count;
  reg start_sending;
  reg [3:0] start_byte_index;

  // Register 59 value 4 is the transient "start positioning" command used by
  // the vendor V6.6 tool.  Stay passive while positions arrive; otherwise
  // send the command after boot and repeat it no faster than once per 3 s.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      start_wait_count <= 28'd0;
      start_sending <= 1'b0;
      start_byte_index <= 4'd0;
      tx_data <= 8'd0;
      tx_start <= 1'b0;
      start_request_count <= 16'd0;
    end else begin
      tx_start <= 1'b0;
      if (position_online) begin
        start_wait_count <= 28'd0;
        start_sending <= 1'b0;
        start_byte_index <= 4'd0;
      end else if (!start_sending) begin
        if (start_wait_count >=
            ((start_request_count == 16'd0) ? START_DELAY_CYCLES - 1 :
                                              START_RETRY_CYCLES - 1)) begin
          start_wait_count <= 28'd0;
          start_sending <= 1'b1;
          start_byte_index <= 4'd0;
        end else start_wait_count <= start_wait_count + 28'd1;
      end else if (!tx_busy && !tx_start) begin
        tx_data <= start_frame_byte(start_byte_index);
        tx_start <= 1'b1;
        if (start_byte_index == 4'd10) begin
          start_sending <= 1'b0;
          start_byte_index <= 4'd0;
          if (start_request_count != 16'hffff)
            start_request_count <= start_request_count + 16'd1;
        end else start_byte_index <= start_byte_index + 4'd1;
      end
    end
  end

  reg [6:0] frame_index;
  reg [6:0] expected_length;
  reg [7:0] frame_function;
  reg [15:0] frame_crc;
  reg [15:0] crc_work;
  reg [3:0] crc_steps_remaining;
  reg crc_running;
  reg frame_complete_pending;
  reg complete_position_pending;
  reg complete_ack_pending;
  reg [18:0] frame_idle_count;
  reg [15:0] output_flags;
  reg [15:0] short_tag_id;
  reg [15:0] long_tag_id;
  reg [7:0] marker_high;
  reg [7:0] marker_low;
  reg start_ack_match;
  reg position_candidate_seen;
  reg signed [15:0] candidate_x_cm;
  reg signed [15:0] candidate_y_cm;
  reg signed [15:0] candidate_z_cm;
  reg signed [15:0] raw_x_cm;
  reg signed [15:0] raw_y_cm;
  reg signed [15:0] raw_z_cm;
  reg raw_position_strobe;
  wire [15:0] crc_bit_next = crc16_modbus_bit(crc_work);
  wire frame_is_ac = (marker_high == 8'hAC) && (marker_low == 8'hDA);
  wire short_position_format = frame_is_ac && output_flags[1] &&
      !output_flags[0];
  wire long_position_format = frame_is_ac && output_flags[1] &&
      output_flags[0];

  // Tag 1's local TTL output uses AC DA frames.  Keep only the coordinate
  // fields from its short (position-only) and long (distance+position) forms.
  // CRC is evaluated one bit per system clock: a byte arrives only every 4340
  // clocks, so this removes the eight-round combinational CRC without reducing
  // accepted UART throughput.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      frame_index <= 7'd0;
      expected_length <= 7'd0;
      frame_function <= 8'd0;
      frame_crc <= 16'hffff;
      crc_work <= 16'hffff;
      crc_steps_remaining <= 4'd0;
      crc_running <= 1'b0;
      frame_complete_pending <= 1'b0;
      complete_position_pending <= 1'b0;
      complete_ack_pending <= 1'b0;
      frame_idle_count <= 19'd0;
      output_flags <= 16'd0;
      short_tag_id <= 16'd0;
      long_tag_id <= 16'd0;
      marker_high <= 8'd0;
      marker_low <= 8'd0;
      start_ack_match <= 1'b0;
      position_candidate_seen <= 1'b0;
      candidate_x_cm <= 16'sd0;
      candidate_y_cm <= 16'sd0;
      candidate_z_cm <= 16'sd0;
      raw_x_cm <= 16'sd0;
      raw_y_cm <= 16'sd0;
      raw_z_cm <= 16'sd0;
      raw_position_strobe <= 1'b0;
      rx_packet_count <= 16'd0;
      rx_crc_error_count <= 16'd0;
      rx_framing_error_count <= 16'd0;
      start_ack_seen <= 1'b0;
      distance_valid_mask <= 16'd0;
      distance_a_cm <= 16'd0;
      distance_b_cm <= 16'd0;
      distance_c_cm <= 16'd0;
      distance_d_cm <= 16'd0;
    end else begin
      raw_position_strobe <= 1'b0;

      if (crc_running) begin
        if (crc_steps_remaining == 4'd1) begin
          crc_running <= 1'b0;
          crc_steps_remaining <= 4'd0;
          if (frame_complete_pending) begin
            frame_crc <= 16'hffff;
            frame_complete_pending <= 1'b0;
            if (crc_bit_next == 16'd0) begin
              if (rx_packet_count != 16'hffff)
                rx_packet_count <= rx_packet_count + 16'd1;
              if (complete_ack_pending) start_ack_seen <= 1'b1;
              if (complete_position_pending) begin
                raw_x_cm <= candidate_x_cm;
                raw_y_cm <= candidate_y_cm;
                raw_z_cm <= candidate_z_cm;
                raw_position_strobe <= 1'b1;
              end
            end else if (rx_crc_error_count != 16'hffff) begin
              rx_crc_error_count <= rx_crc_error_count + 16'd1;
            end
            complete_position_pending <= 1'b0;
            complete_ack_pending <= 1'b0;
          end else begin
            frame_crc <= crc_bit_next;
          end
        end else begin
          crc_work <= crc_bit_next;
          crc_steps_remaining <= crc_steps_remaining - 4'd1;
        end
      end

      if (uart_framing_error_pulse &&
          (rx_framing_error_count != 16'hffff))
        rx_framing_error_count <= rx_framing_error_count + 16'd1;

      if ((frame_index != 7'd0) && !rx_valid && !crc_running) begin
        if (frame_idle_count >= FRAME_TIMEOUT_CYCLES - 1) begin
          frame_index <= 7'd0;
          expected_length <= 7'd0;
          frame_crc <= 16'hffff;
          frame_complete_pending <= 1'b0;
          frame_idle_count <= 19'd0;
          if (rx_framing_error_count != 16'hffff)
            rx_framing_error_count <= rx_framing_error_count + 16'd1;
        end else frame_idle_count <= frame_idle_count + 19'd1;
      end

      if (rx_valid && !crc_running) begin
        frame_idle_count <= 19'd0;
        if (frame_index == 7'd0) begin
          if (rx_byte == 8'h01) begin
            crc_work <= 16'hffff ^ {8'd0, rx_byte};
            crc_steps_remaining <= 4'd8;
            crc_running <= 1'b1;
            frame_index <= 7'd1;
            expected_length <= 7'd0;
            output_flags <= 16'd0;
            short_tag_id <= 16'd0;
            long_tag_id <= 16'd0;
            marker_high <= 8'd0;
            marker_low <= 8'd0;
            start_ack_match <= 1'b0;
            position_candidate_seen <= 1'b0;
          end
        end else if (frame_index == 7'd1) begin
          if ((rx_byte == 8'h03) || (rx_byte == 8'h10)) begin
            crc_work <= frame_crc ^ {8'd0, rx_byte};
            crc_steps_remaining <= 4'd8;
            crc_running <= 1'b1;
            frame_function <= rx_byte;
            start_ack_match <= (rx_byte == 8'h10);
            frame_index <= 7'd2;
          end else if (rx_byte == 8'h01) begin
            crc_work <= 16'hffff ^ {8'd0, rx_byte};
            crc_steps_remaining <= 4'd8;
            crc_running <= 1'b1;
            frame_index <= 7'd1;
            expected_length <= 7'd0;
            output_flags <= 16'd0;
            short_tag_id <= 16'd0;
            long_tag_id <= 16'd0;
            marker_high <= 8'd0;
            marker_low <= 8'd0;
            start_ack_match <= 1'b0;
            position_candidate_seen <= 1'b0;
          end else begin
            frame_index <= 7'd0;
            frame_crc <= 16'hffff;
          end
        end else if (frame_index == 7'd2) begin
          crc_work <= frame_crc ^ {8'd0, rx_byte};
          crc_steps_remaining <= 4'd8;
          crc_running <= 1'b1;
          if (frame_function == 8'h03) begin
            if ((rx_byte >= 8'd4) && (rx_byte <= 8'd59)) begin
              expected_length <= rx_byte + 7'd5;
              frame_index <= 7'd3;
            end else begin
              frame_index <= 7'd0;
              expected_length <= 7'd0;
              frame_crc <= 16'hffff;
              crc_running <= 1'b0;
              if (rx_framing_error_count != 16'hffff)
                rx_framing_error_count <= rx_framing_error_count + 16'd1;
            end
          end else begin
            expected_length <= 7'd8;
            start_ack_match <= start_ack_match && (rx_byte == 8'h00);
            frame_index <= 7'd3;
          end
        end else begin
          crc_work <= frame_crc ^ {8'd0, rx_byte};
          crc_steps_remaining <= 4'd8;
          crc_running <= 1'b1;

          if (frame_function == 8'h10) begin
            if (frame_index == 7'd3)
              start_ack_match <= start_ack_match && (rx_byte == 8'h3B);
            else if (frame_index == 7'd4)
              start_ack_match <= start_ack_match && (rx_byte == 8'h00);
            else if (frame_index == 7'd5)
              start_ack_match <= start_ack_match && (rx_byte == 8'h01);
          end else begin
            case (frame_index)
              7'd3: marker_high <= rx_byte;
              7'd4: marker_low <= rx_byte;
              7'd5: output_flags[15:8] <= rx_byte;
              7'd6: output_flags[7:0] <= rx_byte;
              7'd7: short_tag_id[15:8] <= rx_byte;
              7'd8: short_tag_id[7:0] <= rx_byte;
              7'd9: if (short_position_format)
                candidate_x_cm[15:8] <= rx_byte;
              7'd10: if (short_position_format)
                candidate_x_cm[7:0] <= rx_byte;
              7'd11: if (short_position_format)
                candidate_y_cm[15:8] <= rx_byte;
              7'd12: if (short_position_format)
                candidate_y_cm[7:0] <= rx_byte;
              7'd13: if (short_position_format)
                candidate_z_cm[15:8] <= rx_byte;
              7'd14: if (short_position_format) begin
                candidate_z_cm[7:0] <= rx_byte;
                if (short_tag_id == 16'd1)
                  position_candidate_seen <= 1'b1;
              end
              7'd41: if (long_position_format)
                long_tag_id[15:8] <= rx_byte;
              7'd42: if (long_position_format)
                long_tag_id[7:0] <= rx_byte;
              7'd43: if (long_position_format)
                candidate_x_cm[15:8] <= rx_byte;
              7'd44: if (long_position_format)
                candidate_x_cm[7:0] <= rx_byte;
              7'd45: if (long_position_format)
                candidate_y_cm[15:8] <= rx_byte;
              7'd46: if (long_position_format)
                candidate_y_cm[7:0] <= rx_byte;
              7'd47: if (long_position_format)
                candidate_z_cm[15:8] <= rx_byte;
              7'd48: if (long_position_format) begin
                candidate_z_cm[7:0] <= rx_byte;
                if (long_tag_id == 16'd1)
                  position_candidate_seen <= 1'b1;
              end
              default: begin end
            endcase
          end

          if ((expected_length != 7'd0) &&
              (frame_index + 7'd1 >= expected_length)) begin
            frame_index <= 7'd0;
            expected_length <= 7'd0;
            frame_complete_pending <= 1'b1;
            complete_position_pending <=
                (frame_function == 8'h03) && position_candidate_seen;
            complete_ack_pending <=
                (frame_function == 8'h10) && start_ack_match;
          end else frame_index <= frame_index + 7'd1;
        end
      end
    end
  end

  reg have_position;
  reg [26:0] position_age_count;
  wire raw_position_plausible =
      ($signed(raw_x_cm) >= -16'sd1000) &&
      ($signed(raw_x_cm) <= 16'sd1000) &&
      ($signed(raw_y_cm) >= -16'sd1000) &&
      ($signed(raw_y_cm) <= 16'sd1000) &&
      (!have_position ||
       ((signed_abs_diff16(raw_x_cm, position_x_cm) <= 17'd150) &&
        (signed_abs_diff16(raw_y_cm, position_y_cm) <= 17'd150)));

  // A first-order half-average rejects isolated centimeter-scale jitter while
  // the 150 cm innovation gate rejects corrupted but CRC-valid coordinates.
  // Heading correction is deliberately not inferred from a single tag; IMU
  // yaw remains authoritative and UWB supplies low-frequency translation.
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      position_valid <= 1'b0;
      position_online <= 1'b0;
      position_good <= 1'b0;
      position_x_cm <= 16'sd0;
      position_y_cm <= 16'sd0;
      position_z_cm <= 16'sd0;
      have_position <= 1'b0;
      position_age_count <= LINK_TIMEOUT_CYCLES;
    end else begin
      position_valid <= 1'b0;
      if (raw_position_strobe) begin
        position_good <= raw_position_plausible;
        if (raw_position_plausible) begin
          position_valid <= 1'b1;
          position_online <= 1'b1;
          position_age_count <= 27'd0;
          if (!have_position) begin
            position_x_cm <= raw_x_cm;
            position_y_cm <= raw_y_cm;
            position_z_cm <= raw_z_cm;
            have_position <= 1'b1;
          end else begin
            position_x_cm <= average_signed16(position_x_cm, raw_x_cm);
            position_y_cm <= average_signed16(position_y_cm, raw_y_cm);
            position_z_cm <= average_signed16(position_z_cm, raw_z_cm);
          end
        end
      end else if (position_age_count >= LINK_TIMEOUT_CYCLES - 1) begin
        position_online <= 1'b0;
      end else position_age_count <= position_age_count + 27'd1;
    end
  end

endmodule

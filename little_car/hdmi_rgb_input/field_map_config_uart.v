module field_map_config_uart (
  input wire clk,
  input wire rst_n,
  input wire bt_uart_rx,
  output reg config_valid,
  output reg config_update_pulse,
  output reg [7:0] pose_x_cell,
  output reg [7:0] pose_y_cell,
  output reg [9:0] heading_1024,
  output reg [7:0] low_x_cell,
  output reg [7:0] high_x_cell,
  output reg [7:0] low_y_cell,
  output reg [7:0] high_y_cell
);

  parameter integer CLK_HZ = 50000000;
  parameter integer BAUD = 115200;
  localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
  localparam integer FRAME_TIMEOUT_CYCLES = CLK_HZ / 10;
  localparam [1:0] RX_IDLE = 2'd0;
  localparam [1:0] RX_START = 2'd1;
  localparam [1:0] RX_DATA = 2'd2;
  localparam [1:0] RX_STOP = 2'd3;

  reg [1:0] rx_sync;
  reg [1:0] rx_state;
  reg [15:0] rx_clock_count;
  reg [2:0] rx_bit_index;
  reg [7:0] rx_shift;
  reg [7:0] rx_data;
  reg rx_valid;

  reg frame_active;
  reg [4:0] frame_index;
  reg [22:0] frame_timeout_count;
  reg [7:0] next_pose_x;
  reg [7:0] next_pose_y;
  reg [11:0] next_heading;
  reg [7:0] next_low_x;
  reg [7:0] next_high_x;
  reg [7:0] next_low_y;
  reg [7:0] next_high_y;




  function nibble_valid;
    input [7:0] value;
    begin
      case (value)
        "!", "#", "$", "%", "&", "'", "(", ")",
        "*", ",", ".", "/", ":", ";", "<", ">": nibble_valid = 1'b1;
        default: nibble_valid = 1'b0;
      endcase
    end
  endfunction

  function [3:0] decode_nibble;
    input [7:0] value;
    begin
      case (value)
        "!": decode_nibble = 4'h0;
        "#": decode_nibble = 4'h1;
        "$": decode_nibble = 4'h2;
        "%": decode_nibble = 4'h3;
        "&": decode_nibble = 4'h4;
        "'": decode_nibble = 4'h5;
        "(": decode_nibble = 4'h6;
        ")": decode_nibble = 4'h7;
        "*": decode_nibble = 4'h8;
        ",": decode_nibble = 4'h9;
        ".": decode_nibble = 4'ha;
        "/": decode_nibble = 4'hb;
        ":": decode_nibble = 4'hc;
        ";": decode_nibble = 4'hd;
        "<": decode_nibble = 4'he;
        default: decode_nibble = 4'hf;
      endcase
    end
  endfunction

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_sync <= 2'b11;
      rx_state <= RX_IDLE;
      rx_clock_count <= 16'd0;
      rx_bit_index <= 3'd0;
      rx_shift <= 8'd0;
      rx_data <= 8'd0;
      rx_valid <= 1'b0;
    end else begin
      rx_sync <= {rx_sync[0], bt_uart_rx};
      rx_valid <= 1'b0;
      case (rx_state)
        RX_IDLE: begin
          rx_clock_count <= 16'd0;
          rx_bit_index <= 3'd0;
          if (!rx_sync[1]) rx_state <= RX_START;
        end
        RX_START: begin
          if (rx_clock_count == CLKS_PER_BIT / 2) begin
            rx_clock_count <= 16'd0;
            rx_state <= !rx_sync[1] ? RX_DATA : RX_IDLE;
          end else rx_clock_count <= rx_clock_count + 16'd1;
        end
        RX_DATA: begin
          if (rx_clock_count == CLKS_PER_BIT - 1) begin
            rx_clock_count <= 16'd0;
            rx_shift[rx_bit_index] <= rx_sync[1];
            if (rx_bit_index == 3'd7) begin
              rx_bit_index <= 3'd0;
              rx_state <= RX_STOP;
            end else rx_bit_index <= rx_bit_index + 3'd1;
          end else rx_clock_count <= rx_clock_count + 16'd1;
        end
        default: begin
          if (rx_clock_count == CLKS_PER_BIT - 1) begin
            rx_clock_count <= 16'd0;
            rx_data <= rx_shift;
            rx_valid <= rx_sync[1];
            rx_state <= RX_IDLE;
          end else rx_clock_count <= rx_clock_count + 16'd1;
        end
      endcase
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      config_valid <= 1'b0;
      config_update_pulse <= 1'b0;
      pose_x_cell <= 8'd64;
      pose_y_cell <= 8'd64;
      heading_1024 <= 10'd0;
      low_x_cell <= 8'd9;
      high_x_cell <= 8'd119;
      low_y_cell <= 8'd9;
      high_y_cell <= 8'd119;
      frame_active <= 1'b0;
      frame_index <= 5'd0;
      frame_timeout_count <= 23'd0;
      next_pose_x <= 8'd0;
      next_pose_y <= 8'd0;
      next_heading <= 12'd0;
      next_low_x <= 8'd0;
      next_high_x <= 8'd0;
      next_low_y <= 8'd0;
      next_high_y <= 8'd0;
    end else begin
      config_update_pulse <= 1'b0;
      if (frame_active) begin
        if (frame_timeout_count >= FRAME_TIMEOUT_CYCLES - 1) begin
          frame_active <= 1'b0;
          frame_index <= 5'd0;
          frame_timeout_count <= 23'd0;
        end else frame_timeout_count <= frame_timeout_count + 23'd1;
      end
      if (rx_valid) begin
        frame_timeout_count <= 23'd0;
        if (rx_data == "{") begin
          frame_active <= 1'b1;
          frame_index <= 5'd0;
          next_pose_x <= 8'd0;
          next_pose_y <= 8'd0;
          next_heading <= 12'd0;
          next_low_x <= 8'd0;
          next_high_x <= 8'd0;
          next_low_y <= 8'd0;
          next_high_y <= 8'd0;
        end else if (frame_active && frame_index == 5'd15 && rx_data == "}") begin
          frame_active <= 1'b0;
          frame_index <= 5'd0;
          if (next_heading <= 12'd1023 && next_low_x < next_high_x &&
              next_low_y < next_high_y && next_high_x < 8'd128 &&
              next_high_y < 8'd128) begin
            pose_x_cell <= next_pose_x;
            pose_y_cell <= next_pose_y;
            heading_1024 <= next_heading[9:0];
            low_x_cell <= next_low_x;
            high_x_cell <= next_high_x;
            low_y_cell <= next_low_y;
            high_y_cell <= next_high_y;
            config_valid <= 1'b1;
            config_update_pulse <= 1'b1;
          end
        end else if (frame_active && frame_index < 5'd15 && nibble_valid(rx_data)) begin
          case (frame_index)
            5'd0, 5'd1: next_pose_x <= {next_pose_x[3:0], decode_nibble(rx_data)};
            5'd2, 5'd3: next_pose_y <= {next_pose_y[3:0], decode_nibble(rx_data)};
            5'd4, 5'd5, 5'd6: next_heading <= {next_heading[7:0], decode_nibble(rx_data)};
            5'd7, 5'd8: next_low_x <= {next_low_x[3:0], decode_nibble(rx_data)};
            5'd9, 5'd10: next_high_x <= {next_high_x[3:0], decode_nibble(rx_data)};
            5'd11, 5'd12: next_low_y <= {next_low_y[3:0], decode_nibble(rx_data)};
            default: next_high_y <= {next_high_y[3:0], decode_nibble(rx_data)};
          endcase
          frame_index <= frame_index + 5'd1;
        end else if (frame_active) begin
          frame_active <= 1'b0;
          frame_index <= 5'd0;
        end
      end
    end
  end

endmodule

module temp_ultrasonic_echo_source(
input wire clk,
input wire rst_n,
input wire trig,
output reg echo,
output reg [15:0] fake_distance_cm
);

  localparam [2:0] ST_IDLE  = 3'd0;
  localparam [2:0] ST_DELAY = 3'd1;
  localparam [2:0] ST_ECHO  = 3'd2;
  localparam [2:0] ST_WAIT  = 3'd3;

  localparam [31:0] RESPONSE_DELAY_CYCLES = 32'd5000;    // 100 us at 50 MHz
  localparam [31:0] FAR_ECHO_CYCLES       = 32'd174000;  // 60 cm * 2900
  localparam [31:0] NEAR_ECHO_CYCLES      = 32'd29000;   // 10 cm * 2900

  reg [2:0] state;
  reg [31:0] cnt;
  reg [31:0] echo_target_cycles;
  reg [3:0] trig_count;
  reg trig_d0;
  reg trig_d1;

  wire trig_rise;

  assign trig_rise = trig_d0 & ~trig_d1;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      trig_d0 <= 1'b0;
      trig_d1 <= 1'b0;
    end else begin
      trig_d0 <= trig;
      trig_d1 <= trig_d0;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= ST_IDLE;
      cnt <= 32'd0;
      echo <= 1'b0;
      echo_target_cycles <= FAR_ECHO_CYCLES;
      fake_distance_cm <= 16'd60;
      trig_count <= 4'd0;
    end else begin
      case (state)
        ST_IDLE: begin
          echo <= 1'b0;
          cnt <= 32'd0;

          if (trig_rise) begin
            if (trig_count < 4'd8) begin
              echo_target_cycles <= FAR_ECHO_CYCLES;
              fake_distance_cm <= 16'd60;
              trig_count <= trig_count + 4'd1;
            end else begin
              echo_target_cycles <= NEAR_ECHO_CYCLES;
              fake_distance_cm <= 16'd10;
            end

            state <= ST_DELAY;
          end
        end

        ST_DELAY: begin
          echo <= 1'b0;

          if (cnt >= RESPONSE_DELAY_CYCLES - 32'd1) begin
            cnt <= 32'd0;
            echo <= 1'b1;
            state <= ST_ECHO;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        ST_ECHO: begin
          echo <= 1'b1;

          if (cnt >= echo_target_cycles - 32'd1) begin
            cnt <= 32'd0;
            echo <= 1'b0;
            state <= ST_WAIT;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        ST_WAIT: begin
          echo <= 1'b0;

          if (!trig) begin
            state <= ST_IDLE;
            cnt <= 32'd0;
          end
        end

        default: begin
          state <= ST_IDLE;
          cnt <= 32'd0;
          echo <= 1'b0;
        end
      endcase
    end
  end

endmodule

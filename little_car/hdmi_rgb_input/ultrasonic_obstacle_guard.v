module ultrasonic_obstacle_guard(
input wire clk,
input wire rst_n,
input wire enable,
input wire echo,
input wire [15:0] step_length,
output reg trig,
output reg [15:0] distance_cm,
output reg distance_valid,
output reg obstacle_stop,
output reg echo_timeout
);

  localparam [2:0] ST_TRIG      = 3'd0;
  localparam [2:0] ST_WAIT_ECHO = 3'd1;
  localparam [2:0] ST_MEASURE   = 3'd2;
  localparam [2:0] ST_GAP       = 3'd3;

  localparam [31:0] TRIG_CYCLES     = 32'd500;      // 10 us at 50 MHz
  localparam [31:0] ECHO_TIMEOUT    = 32'd1500000;  // 30 ms at 50 MHz
  localparam [31:0] MEASURE_PERIOD  = 32'd3000000;  // 60 ms at 50 MHz
  localparam [31:0] CYCLES_PER_CM   = 32'd2900;     // 58 us/cm * 50 cycles/us
  localparam [15:0] DEFAULT_STEP_CM = 16'd20;

  reg [2:0] state;
  reg [31:0] cnt;
  reg [31:0] echo_cycles;
  reg [31:0] cm_cycle_cnt;
  reg [15:0] measured_cm;
  reg echo_d0;
  reg echo_d1;
  reg echo_d2;
  reg [15:0] compare_length;

  wire echo_sync;
  wire echo_rise;
  wire echo_fall;

  assign echo_sync = echo_d1;
  assign echo_rise = echo_d1 & ~echo_d2;
  assign echo_fall = ~echo_d1 & echo_d2;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      echo_d0 <= 1'b0;
      echo_d1 <= 1'b0;
      echo_d2 <= 1'b0;
    end else begin
      echo_d0 <= echo;
      echo_d1 <= echo_d0;
      echo_d2 <= echo_d1;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= ST_TRIG;
      cnt <= 32'd0;
      echo_cycles <= 32'd0;
      cm_cycle_cnt <= 32'd0;
      measured_cm <= 16'd0;
      trig <= 1'b0;
      distance_cm <= 16'd0;
      distance_valid <= 1'b0;
      obstacle_stop <= 1'b0;
      echo_timeout <= 1'b0;
      compare_length <= DEFAULT_STEP_CM;
    end else begin
      distance_valid <= 1'b0;
      obstacle_stop <= 1'b0;

      if (step_length == 16'd0)
        compare_length <= DEFAULT_STEP_CM;
      else
        compare_length <= step_length;

      if (!enable) begin
        state <= ST_TRIG;
        cnt <= 32'd0;
        echo_cycles <= 32'd0;
        cm_cycle_cnt <= 32'd0;
        measured_cm <= 16'd0;
        trig <= 1'b0;
        obstacle_stop <= 1'b0;
        echo_timeout <= 1'b0;
      end else begin
        case (state)
        ST_TRIG: begin
          trig <= 1'b1;
          echo_timeout <= 1'b0;
          echo_cycles <= 32'd0;
          cm_cycle_cnt <= 32'd0;
          measured_cm <= 16'd0;

          if (cnt >= TRIG_CYCLES - 32'd1) begin
            cnt <= 32'd0;
            trig <= 1'b0;
            state <= ST_WAIT_ECHO;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        ST_WAIT_ECHO: begin
          trig <= 1'b0;

          if (echo_rise || echo_sync) begin
            cnt <= 32'd0;
            echo_cycles <= 32'd0;
            cm_cycle_cnt <= 32'd0;
            measured_cm <= 16'd0;
            state <= ST_MEASURE;
          end else if (cnt >= ECHO_TIMEOUT - 32'd1) begin
            cnt <= 32'd0;
            echo_timeout <= 1'b1;
            distance_cm <= 16'hFFFF;
            distance_valid <= 1'b1;
            obstacle_stop <= 1'b0;
            state <= ST_GAP;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        ST_MEASURE: begin
          trig <= 1'b0;

          if (echo_fall) begin
            if (measured_cm == 16'd0 && echo_cycles != 32'd0)
              distance_cm <= 16'd1;
            else
              distance_cm <= measured_cm;

            distance_valid <= 1'b1;
            echo_timeout <= 1'b0;

            if (((measured_cm == 16'd0 && echo_cycles != 32'd0) ? 16'd1 : measured_cm) < compare_length)
              obstacle_stop <= 1'b1;

            cnt <= 32'd0;
            state <= ST_GAP;
          end else if (echo_cycles >= ECHO_TIMEOUT - 32'd1) begin
            cnt <= 32'd0;
            echo_cycles <= 32'd0;
            cm_cycle_cnt <= 32'd0;
            measured_cm <= 16'd0;
            echo_timeout <= 1'b1;
            distance_cm <= 16'hFFFF;
            distance_valid <= 1'b1;
            obstacle_stop <= 1'b0;
            state <= ST_GAP;
          end else begin
            echo_cycles <= echo_cycles + 32'd1;
            if (cm_cycle_cnt >= CYCLES_PER_CM - 32'd1) begin
              cm_cycle_cnt <= 32'd0;
              if (measured_cm != 16'hFFFF)
                measured_cm <= measured_cm + 16'd1;
            end else begin
              cm_cycle_cnt <= cm_cycle_cnt + 32'd1;
            end
          end
        end

        ST_GAP: begin
          trig <= 1'b0;

          if (cnt >= MEASURE_PERIOD - TRIG_CYCLES - 32'd1) begin
            cnt <= 32'd0;
            state <= ST_TRIG;
          end else begin
            cnt <= cnt + 32'd1;
          end
        end

        default: begin
          state <= ST_TRIG;
          cnt <= 32'd0;
          trig <= 1'b0;
        end
        endcase
      end
    end
  end

endmodule

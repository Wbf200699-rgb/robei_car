module buzzer_ctrl(
  input wire clk,
  input wire rst_n,
  input wire power_on_pulse,
  input wire route_add_pulse,
  input wire run_enter_pulse,
  input wire stop_enter_pulse,
  output wire buzzer_out
);

  localparam integer CLK_HZ = 50000000;
  localparam integer TONE_HZ = 2000;
  localparam integer BEEP_MS = 80;
  localparam integer GAP_MS = 80;
  localparam ACTIVE_LOW = 1'b0;

  localparam integer HALF_PERIOD = CLK_HZ / (TONE_HZ * 2);
  localparam integer BEEP_CYCLES = (CLK_HZ / 1000) * BEEP_MS;
  localparam integer GAP_CYCLES  = (CLK_HZ / 1000) * GAP_MS;

  reg tone_bit;
  reg beep_active;
  reg gap_active;
  reg [31:0] tone_cnt;
  reg [31:0] beep_cnt;
  reg [31:0] gap_cnt;
  reg [1:0] pending_beeps;
  reg [1:0] current_beeps;
  wire raw_buzzer;

  assign raw_buzzer = beep_active ? tone_bit : 1'b0;
  assign buzzer_out = ACTIVE_LOW ? ~raw_buzzer : raw_buzzer;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tone_bit <= 1'b0;
      beep_active <= 1'b0;
      gap_active <= 1'b0;
      tone_cnt <= 32'd0;
      beep_cnt <= 32'd0;
      gap_cnt <= 32'd0;
      pending_beeps <= 2'd0;
      current_beeps <= 2'd0;
    end else begin
      if (stop_enter_pulse)
        pending_beeps <= 2'd2;
      else if (run_enter_pulse)
        pending_beeps <= 2'd1;
      else if (route_add_pulse)
        pending_beeps <= 2'd1;
      else if (power_on_pulse)
        pending_beeps <= 2'd1;

      if (beep_active) begin
        if (tone_cnt == HALF_PERIOD - 1) begin
          tone_cnt <= 32'd0;
          tone_bit <= ~tone_bit;
        end else begin
          tone_cnt <= tone_cnt + 32'd1;
        end

        if (beep_cnt == BEEP_CYCLES - 1) begin
          beep_active <= 1'b0;
          beep_cnt <= 32'd0;
          tone_cnt <= 32'd0;
          tone_bit <= 1'b0;
          if (current_beeps > 2'd1) begin
            current_beeps <= current_beeps - 2'd1;
            gap_active <= 1'b1;
            gap_cnt <= 32'd0;
          end else begin
            current_beeps <= 2'd0;
          end
        end else begin
          beep_cnt <= beep_cnt + 32'd1;
        end
      end else if (gap_active) begin
        if (gap_cnt == GAP_CYCLES - 1) begin
          gap_active <= 1'b0;
          gap_cnt <= 32'd0;
          beep_active <= 1'b1;
          beep_cnt <= 32'd0;
          tone_cnt <= 32'd0;
          tone_bit <= 1'b0;
        end else begin
          gap_cnt <= gap_cnt + 32'd1;
        end
      end else if (current_beeps != 2'd0) begin
        beep_active <= 1'b1;
        beep_cnt <= 32'd0;
        tone_cnt <= 32'd0;
        tone_bit <= 1'b0;
      end else if (pending_beeps != 2'd0) begin
        current_beeps <= pending_beeps;
        pending_beeps <= 2'd0;
      end
    end
  end

endmodule

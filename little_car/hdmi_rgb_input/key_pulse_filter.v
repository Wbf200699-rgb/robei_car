module key_pulse_filter(
input wire clk,
input wire rst_n,
input wire key_n,
output reg key_pulse
);

  reg [1:0] key_sync;
  reg key_level;
  reg key_level_d;
  reg [19:0] stable_cnt;

  wire key_pressed;

  assign key_pressed = ~key_sync[1];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      key_sync    <= 2'b11;
      key_level   <= 1'b0;
      key_level_d <= 1'b0;
      stable_cnt  <= 20'd0;
      key_pulse   <= 1'b0;
    end else begin
      key_sync <= {key_sync[0], key_n};

      if (key_pressed == key_level) begin
        stable_cnt <= 20'd0;
      end else if (stable_cnt >= 20'd999999) begin
        key_level  <= key_pressed;
        stable_cnt <= 20'd0;
      end else begin
        stable_cnt <= stable_cnt + 20'd1;
      end

      key_level_d <= key_level;
      key_pulse   <= key_level & ~key_level_d;
    end
  end

endmodule

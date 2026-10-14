module motor_encoder_interface(
input wire clk,
input wire rst_n,
input wire lf_enc_a,
input wire lf_enc_b,
input wire rf_enc_a,
input wire rf_enc_b,
input wire lr_enc_a,
input wire lr_enc_b,
input wire rr_enc_a,
input wire rr_enc_b,
output reg lf_tick,
output reg rf_tick,
output reg lr_tick,
output reg rr_tick,
output reg lf_dir,
output reg rf_dir,
output reg lr_dir,
output reg rr_dir,
output reg [15:0] lf_total_count,
output reg [15:0] rf_total_count,
output reg [15:0] lr_total_count,
output reg [15:0] rr_total_count
);

  reg lf_a_d0;
  reg lf_a_d1;
  reg lf_b_d0;
  reg lf_b_d1;
  reg rf_a_d0;
  reg rf_a_d1;
  reg rf_b_d0;
  reg rf_b_d1;
  reg lr_a_d0;
  reg lr_a_d1;
  reg lr_b_d0;
  reg lr_b_d1;
  reg rr_a_d0;
  reg rr_a_d1;
  reg rr_b_d0;
  reg rr_b_d1;

  wire lf_a_rise;
  wire rf_a_rise;
  wire lr_a_rise;
  wire rr_a_rise;

  assign lf_a_rise = lf_a_d0 & ~lf_a_d1;
  assign rf_a_rise = rf_a_d0 & ~rf_a_d1;
  assign lr_a_rise = lr_a_d0 & ~lr_a_d1;
  assign rr_a_rise = rr_a_d0 & ~rr_a_d1;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lf_a_d0 <= 1'b0;
      lf_a_d1 <= 1'b0;
      lf_b_d0 <= 1'b0;
      lf_b_d1 <= 1'b0;
      rf_a_d0 <= 1'b0;
      rf_a_d1 <= 1'b0;
      rf_b_d0 <= 1'b0;
      rf_b_d1 <= 1'b0;
      lr_a_d0 <= 1'b0;
      lr_a_d1 <= 1'b0;
      lr_b_d0 <= 1'b0;
      lr_b_d1 <= 1'b0;
      rr_a_d0 <= 1'b0;
      rr_a_d1 <= 1'b0;
      rr_b_d0 <= 1'b0;
      rr_b_d1 <= 1'b0;
      lf_tick <= 1'b0;
      rf_tick <= 1'b0;
      lr_tick <= 1'b0;
      rr_tick <= 1'b0;
      lf_dir <= 1'b0;
      rf_dir <= 1'b0;
      lr_dir <= 1'b0;
      rr_dir <= 1'b0;
      lf_total_count <= 16'd0;
      rf_total_count <= 16'd0;
      lr_total_count <= 16'd0;
      rr_total_count <= 16'd0;
    end else begin
      lf_a_d0 <= lf_enc_a;
      lf_a_d1 <= lf_a_d0;
      lf_b_d0 <= lf_enc_b;
      lf_b_d1 <= lf_b_d0;
      rf_a_d0 <= rf_enc_a;
      rf_a_d1 <= rf_a_d0;
      rf_b_d0 <= rf_enc_b;
      rf_b_d1 <= rf_b_d0;
      lr_a_d0 <= lr_enc_a;
      lr_a_d1 <= lr_a_d0;
      lr_b_d0 <= lr_enc_b;
      lr_b_d1 <= lr_b_d0;
      rr_a_d0 <= rr_enc_a;
      rr_a_d1 <= rr_a_d0;
      rr_b_d0 <= rr_enc_b;
      rr_b_d1 <= rr_b_d0;

      lf_tick <= lf_a_rise;
      rf_tick <= rf_a_rise;
      lr_tick <= lr_a_rise;
      rr_tick <= rr_a_rise;

      if (lf_a_rise) begin
        lf_dir <= lf_b_d1;
        if (lf_total_count != 16'hFFFF)
          lf_total_count <= lf_total_count + 16'd1;
      end

      if (rf_a_rise) begin
        rf_dir <= rf_b_d1;
        if (rf_total_count != 16'hFFFF)
          rf_total_count <= rf_total_count + 16'd1;
      end

      if (lr_a_rise) begin
        lr_dir <= lr_b_d1;
        if (lr_total_count != 16'hFFFF)
          lr_total_count <= lr_total_count + 16'd1;
      end

      if (rr_a_rise) begin
        rr_dir <= rr_b_d1;
        if (rr_total_count != 16'hFFFF)
          rr_total_count <= rr_total_count + 16'd1;
      end
    end
  end

endmodule

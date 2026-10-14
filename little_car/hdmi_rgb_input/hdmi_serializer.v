module hdmi_serializer(
input wire clk_pix,
input wire clk_5x,
input wire rst_n,
input wire [9:0] data_10b,
output wire ser_p,
output wire ser_n
);

  //----Code starts here: integrated by Robei-----
  reg [9:0] pix_word;
  reg [9:0] pix_bank0;
  reg [9:0] pix_bank1;
  reg pix_bank_sel;
  reg pix_wr_bank;
  reg pix_toggle;
  reg pix_toggle_sync0;
  reg pix_toggle_sync1;
  reg pix_toggle_sync2;
  reg pix_wr_bank_sync0;
  reg pix_wr_bank_sync1;
  reg pix_wr_bank_sync2;
  reg [9:0] shift_word;
  reg [2:0] pair_idx;
  reg ser_d1;
  reg ser_d2;
  wire load_word_5x;
  (* keep = "true" *) wire ser_data_out;

  always @(posedge clk_pix or negedge rst_n) begin
    if (!rst_n) begin
      pix_word <= 10'd0;
      pix_bank0 <= 10'd0;
      pix_bank1 <= 10'd0;
      pix_bank_sel <= 1'b0;
      pix_wr_bank <= 1'b0;
      pix_toggle <= 1'b0;
    end else begin
      pix_word <= data_10b;
      if (!pix_bank_sel)
        pix_bank0 <= data_10b;
      else
        pix_bank1 <= data_10b;
      pix_wr_bank <= pix_bank_sel;
      pix_bank_sel <= ~pix_bank_sel;
      pix_toggle <= ~pix_toggle;
    end
  end

  always @(posedge clk_5x or negedge rst_n) begin
    if (!rst_n) begin
      pix_toggle_sync0 <= 1'b0;
      pix_toggle_sync1 <= 1'b0;
      pix_toggle_sync2 <= 1'b0;
      pix_wr_bank_sync0 <= 1'b0;
      pix_wr_bank_sync1 <= 1'b0;
      pix_wr_bank_sync2 <= 1'b0;
      shift_word <= 10'd0;
      pair_idx <= 3'd0;
      ser_d1 <= 1'b0;
      ser_d2 <= 1'b0;
    end else begin
      pix_toggle_sync0 <= pix_toggle;
      pix_toggle_sync1 <= pix_toggle_sync0;
      pix_toggle_sync2 <= pix_toggle_sync1;
      pix_wr_bank_sync0 <= pix_wr_bank;
      pix_wr_bank_sync1 <= pix_wr_bank_sync0;
      pix_wr_bank_sync2 <= pix_wr_bank_sync1;

      if (load_word_5x) begin
        shift_word <= pix_wr_bank_sync2 ? pix_bank1 : pix_bank0;
        pair_idx <= 3'd0;
        ser_d1 <= pix_wr_bank_sync2 ? pix_bank1[0] : pix_bank0[0];
        ser_d2 <= pix_wr_bank_sync2 ? pix_bank1[1] : pix_bank0[1];
      end else begin
        shift_word <= {2'b00, shift_word[9:2]};
        if (pair_idx < 3'd4)
          pair_idx <= pair_idx + 3'd1;
        ser_d1 <= shift_word[2];
        ser_d2 <= shift_word[3];
      end
    end
  end

  assign load_word_5x = pix_toggle_sync1 ^ pix_toggle_sync2;

  (* keep = "true", dont_touch = "true" *)
  ODDR #(
    .DDR_CLK_EDGE("SAME_EDGE"),
    .INIT(1'b0),
    .SRTYPE("SYNC")
  ) oddr_inst (
    .Q(ser_data_out),
    .C(clk_5x),
    .CE(1'b1),
    .D1(ser_d1),
    .D2(ser_d2),
    .R(1'b0),
    .S(1'b0)
  );

  (* keep = "true", dont_touch = "true" *)
  OBUFDS #(
    .IOSTANDARD("TMDS_33")
  ) obuf_inst (
    .I  (ser_data_out),
    .O  (ser_p),
    .OB (ser_n)
  );
endmodule    //hdmi_serializer

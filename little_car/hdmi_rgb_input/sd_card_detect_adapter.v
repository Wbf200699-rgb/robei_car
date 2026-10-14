module sd_card_detect_adapter (
  input wire sd_cd,
  input wire sd_d1,
  input wire sd_d2,
  output wire sd_card_present
);

  wire companion_lines_high = sd_d1 & sd_d2;
  assign sd_card_present = ~sd_cd;

endmodule

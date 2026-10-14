`timescale 1ns/1ps

module STARTUPE2 #(
  parameter PROG_USR = "FALSE",
  parameter real SIM_CCLK_FREQ = 0.0
) (
  output wire CFGCLK,
  output wire CFGMCLK,
  output wire EOS,
  output wire PREQ,
  input wire CLK,
  input wire GSR,
  input wire GTS,
  input wire KEYCLEARB,
  input wire PACK,
  input wire USRCCLKO,
  input wire USRCCLKTS,
  input wire USRDONEO,
  input wire USRDONETS
);
  assign CFGCLK = 1'b0;
  assign CFGMCLK = 1'b0;
  assign EOS = 1'b1;
  assign PREQ = 1'b0;
endmodule

// Minimal behavioral stubs used only by Icarus top-level elaboration tests.
// Vivado and Robei synthesis use their native ODDR/OBUFDS primitive models.
module ODDR #(
  parameter DDR_CLK_EDGE = "OPPOSITE_EDGE",
  parameter INIT = 1'b0,
  parameter SRTYPE = "SYNC"
) (
  output reg Q,
  input wire C,
  input wire CE,
  input wire D1,
  input wire D2,
  input wire R,
  input wire S
);
  always @(posedge C or posedge R or posedge S) begin
    if (R) Q <= 1'b0;
    else if (S) Q <= 1'b1;
    else if (CE) Q <= D1;
  end
  always @(negedge C or posedge R or posedge S) begin
    if (R) Q <= 1'b0;
    else if (S) Q <= 1'b1;
    else if (CE) Q <= D2;
  end
endmodule

module OBUFDS #(
  parameter IOSTANDARD = "DEFAULT",
  parameter SLEW = "SLOW"
) (
  output wire O,
  output wire OB,
  input wire I
);
  assign O = I;
  assign OB = ~I;
endmodule

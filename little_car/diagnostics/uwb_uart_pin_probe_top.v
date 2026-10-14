module uwb_uart_pin_probe_top (
  input  wire clk,
  input  wire uwb_n14,
  input  wire uwb_n13,
  output wire usb_uart_tx
);
  // At 50 MHz, bit 29 selects each input for about 10.74 seconds.
  // The first window mirrors N14; the second window mirrors N13.
  reg [29:0] select_counter = 30'd0;

  always @(posedge clk)
    select_counter <= select_counter + 30'd1;

  assign usb_uart_tx = select_counter[29] ? uwb_n13 : uwb_n14;
endmodule

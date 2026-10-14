set_property PACKAGE_PIN Y18 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -period 20.000 -name sys_clk [get_ports clk]

set_property PACKAGE_PIN N14 [get_ports uwb_n14]
set_property IOSTANDARD LVCMOS33 [get_ports uwb_n14]
set_property PULLUP TRUE [get_ports uwb_n14]

set_property PACKAGE_PIN N13 [get_ports uwb_n13]
set_property IOSTANDARD LVCMOS33 [get_ports uwb_n13]
set_property DRIVE 8 [get_ports uwb_n13]
set_property SLEW SLOW [get_ports uwb_n13]

set_property PACKAGE_PIN J1 [get_ports usb_uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports usb_uart_tx]
set_property DRIVE 8 [get_ports usb_uart_tx]
set_property SLEW SLOW [get_ports usb_uart_tx]

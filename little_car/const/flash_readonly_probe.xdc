set_property PACKAGE_PIN Y18 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -period 20.000 -name clk50 [get_ports clk]

set_property PACKAGE_PIN J20 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PULLUP TRUE [get_ports rst_n]

set_property PACKAGE_PIN E18 [get_ports flash_cs_n]
set_property PACKAGE_PIN D16 [get_ports flash_clk]
set_property PACKAGE_PIN C18 [get_ports flash_mosi]
set_property PACKAGE_PIN D17 [get_ports flash_wp_n]
set_property PACKAGE_PIN J16 [get_ports flash_hold_n]
set_property PACKAGE_PIN C17 [get_ports flash_miso]
set_property IOSTANDARD LVCMOS33 [get_ports {flash_cs_n flash_clk flash_mosi flash_wp_n flash_hold_n flash_miso}]
set_property DRIVE 8 [get_ports {flash_cs_n flash_clk flash_mosi}]
set_property DRIVE 4 [get_ports {flash_wp_n flash_hold_n}]
set_property SLEW SLOW [get_ports {flash_cs_n flash_clk flash_mosi flash_wp_n flash_hold_n}]
# A weak pull-down is intentional in this diagnostic image. If every read
# changes from FF to 00, C17 is floating instead of being driven by the Flash.
set_property PULLDOWN TRUE [get_ports flash_miso]

# The board resource table names J1 as USB-UART RX, so the FPGA drives J1.
set_property PACKAGE_PIN J1 [get_ports debug_uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports debug_uart_tx]
set_property DRIVE 8 [get_ports debug_uart_tx]
set_property SLEW SLOW [get_ports debug_uart_tx]

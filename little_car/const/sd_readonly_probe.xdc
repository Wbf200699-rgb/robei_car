create_clock -name sys_clk -period 20.000 [get_ports {clk}]

set_property PACKAGE_PIN Y18 [get_ports {clk}]
set_property IOSTANDARD LVCMOS33 [get_ports {clk}]

set_property PACKAGE_PIN J20 [get_ports {rst_n}]
set_property IOSTANDARD LVCMOS33 [get_ports {rst_n}]
set_property PULLUP TRUE [get_ports {rst_n}]

set_property PACKAGE_PIN E13 [get_ports {sd_sclk}]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_sclk}]
set_property DRIVE 4 [get_ports {sd_sclk}]
set_property SLEW SLOW [get_ports {sd_sclk}]

set_property PACKAGE_PIN E14 [get_ports {sd_mosi}]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_mosi}]
set_property DRIVE 4 [get_ports {sd_mosi}]
set_property SLEW SLOW [get_ports {sd_mosi}]

set_property PACKAGE_PIN F14 [get_ports {sd_miso}]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_miso}]
set_property PULLUP TRUE [get_ports {sd_miso}]

set_property PACKAGE_PIN F16 [get_ports {sd_cs_n}]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_cs_n}]
set_property DRIVE 4 [get_ports {sd_cs_n}]
set_property SLEW SLOW [get_ports {sd_cs_n}]
set_property PULLUP TRUE [get_ports {sd_cs_n}]

set_property PACKAGE_PIN G13 [get_ports {sd_cd}]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_cd}]
set_property PULLUP TRUE [get_ports {sd_cd}]

set_property PACKAGE_PIN F13 [get_ports {sd_d1}]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_d1}]
set_property PULLUP TRUE [get_ports {sd_d1}]

set_property PACKAGE_PIN E17 [get_ports {sd_d2}]
set_property IOSTANDARD LVCMOS33 [get_ports {sd_d2}]
set_property PULLUP TRUE [get_ports {sd_d2}]

set_property PACKAGE_PIN J1 [get_ports {debug_uart_tx}]
set_property IOSTANDARD LVCMOS33 [get_ports {debug_uart_tx}]
set_property DRIVE 8 [get_ports {debug_uart_tx}]
set_property SLEW SLOW [get_ports {debug_uart_tx}]

set_property PACKAGE_PIN T1 [get_ports {probe_done_led}]
set_property IOSTANDARD LVCMOS33 [get_ports {probe_done_led}]
set_property DRIVE 8 [get_ports {probe_done_led}]
set_property SLEW SLOW [get_ports {probe_done_led}]

set_property PACKAGE_PIN L6 [get_ports {probe_ok_led}]
set_property IOSTANDARD LVCMOS33 [get_ports {probe_ok_led}]
set_property DRIVE 8 [get_ports {probe_ok_led}]
set_property SLEW SLOW [get_ports {probe_ok_led}]

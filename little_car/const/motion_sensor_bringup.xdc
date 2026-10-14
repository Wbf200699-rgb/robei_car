# Standalone PMW3901 + dual BMI088 bring-up constraints for XC7A50T FGG484.
set_property PACKAGE_PIN Y18 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -period 20.000 -name clk50 [get_ports clk]

set_property PACKAGE_PIN J20 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PULLUP TRUE [get_ports rst_n]

# H3 shared SPI bus and independent chip selects.
set_property PACKAGE_PIN A14 [get_ports sensor_spi_sclk]
set_property PACKAGE_PIN B15 [get_ports sensor_spi_mosi]
set_property PACKAGE_PIN A13 [get_ports sensor_spi_miso]
set_property PACKAGE_PIN B13 [get_ports pmw3901_cs_n]
set_property PACKAGE_PIN F20 [get_ports bmi0_acc_cs_n]
set_property PACKAGE_PIN F19 [get_ports bmi0_gyro_cs_n]
set_property PACKAGE_PIN C20 [get_ports bmi1_acc_cs_n]
set_property PACKAGE_PIN E19 [get_ports bmi1_gyro_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports {sensor_spi_sclk sensor_spi_mosi sensor_spi_miso pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]
set_property DRIVE 4 [get_ports {sensor_spi_sclk sensor_spi_mosi pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]
set_property SLEW SLOW [get_ports {sensor_spi_sclk sensor_spi_mosi pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]
set_property PULLUP TRUE [get_ports {sensor_spi_sclk sensor_spi_miso pmw3901_cs_n bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n}]

# Resource-table RX is the USB-UART receiver, so FPGA TX connects to J1.
set_property PACKAGE_PIN J1 [get_ports sensor_debug_uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports sensor_debug_uart_tx]
set_property DRIVE 8 [get_ports sensor_debug_uart_tx]
set_property SLEW SLOW [get_ports sensor_debug_uart_tx]

# Board LEDs: PMW3901, BMI088 #0 and BMI088 #1 online status.
set_property PACKAGE_PIN T1 [get_ports pmw_status_led]
set_property PACKAGE_PIN L6 [get_ports bmi0_status_led]
set_property PACKAGE_PIN K13 [get_ports bmi1_status_led]
set_property IOSTANDARD LVCMOS33 [get_ports {pmw_status_led bmi0_status_led bmi1_status_led}]
set_property DRIVE 4 [get_ports {pmw_status_led bmi0_status_led bmi1_status_led}]
set_property SLEW SLOW [get_ports {pmw_status_led bmi0_status_led bmi1_status_led}]

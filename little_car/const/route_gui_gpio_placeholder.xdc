# Placeholder GPIO constraints for sys_route_gui_fsm_top
# These pins are temporary assignments to let Vivado compile before the
# real motor/encoder/key/ultrasonic hardware arrives.
#
# Top module ports additionally covered here:
#   clk, rst_n
#   hdmi_clk_p/n, hdmi_d0_p/n, hdmi_d1_p/n, hdmi_d2_p/n
#   buzzer_out
#
# Top module ports covered here:
#   key_pin1..key_pin5
#   ultrasonic_echo, ultrasonic_trig
#   motor_* encoder inputs
#   motor_* dir/pwm outputs
#   motor_stby_a, motor_stby_b
#
# Clock/reset/HDMI pins are taken from the A7-50T board pin table and
# aligned with the older Vivado project constraints.

# --------------------------------------------------------------------
# Clock / reset
# --------------------------------------------------------------------
set_property PACKAGE_PIN Y18 [get_ports clk]
set_property PACKAGE_PIN J20 [get_ports rst_n]

set_property IOSTANDARD LVCMOS33 [get_ports {clk rst_n}]
set_property PULLUP true [get_ports rst_n]

create_clock -period 20.000 -name sys_clk [get_ports clk]

# Handwritten MMCM clocks in build/clk_wiz_0.v:
#   clk_out1 = 50 MHz
#   clk_out2 = 250 MHz
create_generated_clock -name clk_pix \
    -source [get_ports clk] \
    -multiply_by 1 -divide_by 1 \
    [get_pins -hierarchical *clk_pix_bufg_inst/O]

create_generated_clock -name clk_5x \
    -source [get_ports clk] \
    -multiply_by 5 -divide_by 1 \
    [get_pins -hierarchical *clk_5x_bufg_inst/O]

# route_control_core runs on sys_clk, while route_gui_renderer samples those
# status buses in the pixel domain only for display. They are not used for
# functional feedback, so exclude cross-domain timing between the control and
# video domains.
set_clock_groups -asynchronous \
    -group [get_clocks sys_clk] \
    -group [get_clocks {clk_pix clk_5x clk_pix_mmcm clk_5x_mmcm clkfb_mmcm}]

# --------------------------------------------------------------------
# HDMI TMDS
# --------------------------------------------------------------------
set_property PACKAGE_PIN C22 [get_ports hdmi_clk_p]
set_property PACKAGE_PIN B22 [get_ports hdmi_clk_n]
set_property PACKAGE_PIN B21 [get_ports hdmi_d0_p]
set_property PACKAGE_PIN A21 [get_ports hdmi_d0_n]
set_property PACKAGE_PIN B20 [get_ports hdmi_d1_p]
set_property PACKAGE_PIN A20 [get_ports hdmi_d1_n]
set_property PACKAGE_PIN A18 [get_ports hdmi_d2_p]
set_property PACKAGE_PIN A19 [get_ports hdmi_d2_n]

set_property IOSTANDARD TMDS_33 [get_ports {
    hdmi_clk_p hdmi_clk_n
    hdmi_d0_p hdmi_d0_n
    hdmi_d1_p hdmi_d1_n
    hdmi_d2_p hdmi_d2_n
}]

# --------------------------------------------------------------------
# Buzzer
# --------------------------------------------------------------------
set_property PACKAGE_PIN Y8 [get_ports buzzer_out]
set_property IOSTANDARD LVCMOS33 [get_ports buzzer_out]
set_property DRIVE 8 [get_ports buzzer_out]
set_property SLEW SLOW [get_ports buzzer_out]

# --------------------------------------------------------------------
# Key GPIO inputs
# --------------------------------------------------------------------
set_property PACKAGE_PIN J6 [get_ports key_pin1]
set_property PACKAGE_PIN E3 [get_ports key_pin2]
set_property PACKAGE_PIN H3 [get_ports key_pin3]
set_property PACKAGE_PIN J5 [get_ports key_pin4]
set_property PACKAGE_PIN J4 [get_ports key_pin5]

set_property IOSTANDARD LVCMOS33 [get_ports {key_pin1 key_pin2 key_pin3 key_pin4 key_pin5}]
set_property PULLUP true [get_ports {key_pin1 key_pin2 key_pin3 key_pin4 key_pin5}]

# --------------------------------------------------------------------
# Ultrasonic GPIO
# --------------------------------------------------------------------
set_property PACKAGE_PIN D21 [get_ports ultrasonic_echo]
set_property PACKAGE_PIN E22 [get_ports ultrasonic_trig]

set_property IOSTANDARD LVCMOS33 [get_ports {ultrasonic_echo ultrasonic_trig}]

# --------------------------------------------------------------------
# Motor encoder GPIO inputs
# --------------------------------------------------------------------
set_property PACKAGE_PIN R17 [get_ports motor_lf_enc_a]
set_property PACKAGE_PIN U17 [get_ports motor_lf_enc_b]
set_property PACKAGE_PIN W19 [get_ports motor_rf_enc_a]
set_property PACKAGE_PIN AA21 [get_ports motor_rf_enc_b]
set_property PACKAGE_PIN G16 [get_ports motor_lr_enc_a]
set_property PACKAGE_PIN D14 [get_ports motor_lr_enc_b]
set_property PACKAGE_PIN F18 [get_ports motor_rr_enc_a]
set_property PACKAGE_PIN G17 [get_ports motor_rr_enc_b]

set_property IOSTANDARD LVCMOS33 [get_ports {
    motor_lf_enc_a motor_lf_enc_b
    motor_rf_enc_a motor_rf_enc_b
    motor_lr_enc_a motor_lr_enc_b
    motor_rr_enc_a motor_rr_enc_b
}]
set_property PULLDOWN true [get_ports {
    motor_lf_enc_a motor_lf_enc_b
    motor_rf_enc_a motor_rf_enc_b
    motor_lr_enc_a motor_lr_enc_b
    motor_rr_enc_a motor_rr_enc_b
}]

# --------------------------------------------------------------------
# Motor control GPIO outputs
# --------------------------------------------------------------------
set_property PACKAGE_PIN R14 [get_ports motor_lf_in1]
set_property PACKAGE_PIN P15 [get_ports motor_lf_in2]
set_property PACKAGE_PIN AA18 [get_ports motor_rf_in1]
set_property PACKAGE_PIN V19 [get_ports motor_rf_in2]
set_property PACKAGE_PIN H14 [get_ports motor_lr_in1]
set_property PACKAGE_PIN J15 [get_ports motor_lr_in2]
set_property PACKAGE_PIN C19 [get_ports motor_rr_in1]
set_property PACKAGE_PIN D20 [get_ports motor_rr_in2]

set_property PACKAGE_PIN N13 [get_ports motor_lf_pwm]
set_property PACKAGE_PIN W17 [get_ports motor_rf_pwm]
set_property PACKAGE_PIN K14 [get_ports motor_lr_pwm]
set_property PACKAGE_PIN E16 [get_ports motor_rr_pwm]

set_property PACKAGE_PIN AB21 [get_ports motor_stby_a]
set_property PACKAGE_PIN R19 [get_ports motor_stby_b]

set_property IOSTANDARD LVCMOS33 [get_ports {
    motor_lf_in1 motor_lf_in2
    motor_rf_in1 motor_rf_in2
    motor_lr_in1 motor_lr_in2
    motor_rr_in1 motor_rr_in2
    motor_lf_pwm motor_rf_pwm motor_lr_pwm motor_rr_pwm
    motor_stby_a motor_stby_b
}]

set_property DRIVE 8 [get_ports {
    motor_lf_in1 motor_lf_in2
    motor_rf_in1 motor_rf_in2
    motor_lr_in1 motor_lr_in2
    motor_rr_in1 motor_rr_in2
    motor_lf_pwm motor_rf_pwm motor_lr_pwm motor_rr_pwm
    motor_stby_a motor_stby_b
}]

set_property SLEW SLOW [get_ports {
    motor_lf_in1 motor_lf_in2
    motor_rf_in1 motor_rf_in2
    motor_lr_in1 motor_lr_in2
    motor_rr_in1 motor_rr_in2
    motor_lf_pwm motor_rf_pwm motor_lr_pwm motor_rr_pwm
    motor_stby_a motor_stby_b
}]

# --------------------------------------------------------------------
# Board configuration voltage
# --------------------------------------------------------------------
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

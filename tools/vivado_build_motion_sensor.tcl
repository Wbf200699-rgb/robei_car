set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set car_dir [file join $repo_dir little_car]
set profile "normal"
if {$argc >= 1} {
  set profile [string tolower [lindex $argv 0]]
}
if {$profile ni {normal swap_bmi calibration}} {
  puts stderr "ERROR: profile must be normal, swap_bmi, or calibration"
  exit 2
}

set build_name motion_sensor_bringup
if {$profile eq "swap_bmi"} {
  set build_name motion_sensor_bringup_swap_bmi
} elseif {$profile eq "calibration"} {
  set build_name motion_sensor_bringup_calibration
}
set build_dir [file join $car_dir build $build_name]
file mkdir $build_dir

read_verilog [file join $car_dir build power_on_reset.v]
set motion_rtl [file join $car_dir build motion_sensor_subsystem.v]
if {$profile eq "calibration"} {
  # Non-project read_verilog does not reliably apply a fileset-level define.
  # Create a generated diagnostic compile unit so the production RTL and Robei
  # model remain unchanged while the calibration-only formatter is selected.
  set calibration_rtl [file join $build_dir motion_sensor_subsystem_calibration.v]
  set input_handle [open $motion_rtl r]
  set motion_text [read $input_handle]
  close $input_handle
  set output_handle [open $calibration_rtl w]
  puts $output_handle {`define SENSOR_CAL_PROBE}
  puts -nonewline $output_handle $motion_text
  close $output_handle
  read_verilog [file join $car_dir build dual_imu_static_calibrator.v]
  read_verilog $calibration_rtl
} else {
  read_verilog $motion_rtl
}
read_verilog [file join $car_dir build motion_sensor_bringup_top.v]
read_xdc [file join $car_dir const motion_sensor_bringup.xdc]

synth_design -top motion_sensor_bringup_top -part xc7a50tfgg484-1

# Diagnostic-only A/B profile: logical BMI0 talks to the physical BMI1 chip
# selects and logical BMI1 talks to the physical BMI0 chip selects. The shared
# SCLK/MOSI/MISO pins and all RTL are unchanged, so this distinguishes a
# physical module/CS-path fault from an initialization-order fault.
if {$profile eq "swap_bmi"} {
  reset_property PACKAGE_PIN [get_ports {
    bmi0_acc_cs_n bmi0_gyro_cs_n bmi1_acc_cs_n bmi1_gyro_cs_n
  }]
  set_property PACKAGE_PIN C20 [get_ports bmi0_acc_cs_n]
  set_property PACKAGE_PIN E19 [get_ports bmi0_gyro_cs_n]
  set_property PACKAGE_PIN F20 [get_ports bmi1_acc_cs_n]
  set_property PACKAGE_PIN F19 [get_ports bmi1_gyro_cs_n]
}
write_checkpoint -force [file join $build_dir post_synth.dcp]
report_utilization -file [file join $build_dir utilization_synth.rpt]
report_timing_summary -file [file join $build_dir timing_synth.rpt]

opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force [file join $build_dir post_route.dcp]
report_utilization -file [file join $build_dir utilization_route.rpt]
report_timing_summary -file [file join $build_dir timing_route.rpt]
report_drc -file [file join $build_dir drc_route.rpt]
set bitstream_path [file join $build_dir motion_sensor_bringup_top.bit]
write_bitstream -force $bitstream_path

set timing_paths [get_timing_paths -max_paths 1 -nworst 1]
if {[llength $timing_paths] > 0} {
  set worst_slack [get_property SLACK [lindex $timing_paths 0]]
  puts "RESULT worst_slack_ns=$worst_slack"
}
puts "RESULT profile=$profile"
puts "RESULT bitstream=$bitstream_path"
exit

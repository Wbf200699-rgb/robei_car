set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set car_dir [file join $repo_dir little_car]
set build_dir [file join $car_dir build flash_readonly_probe]
file mkdir $build_dir

read_verilog [file join $car_dir diagnostics flash_readonly_probe_top.v]
read_xdc [file join $car_dir const flash_readonly_probe.xdc]

synth_design -top flash_readonly_probe_top -part xc7a50tfgg484-1
write_checkpoint -force [file join $build_dir post_synth.dcp]
report_utilization -file [file join $build_dir utilization_synth.rpt]
report_timing_summary -file [file join $build_dir timing_synth.rpt]

opt_design
place_design
route_design
write_checkpoint -force [file join $build_dir post_route.dcp]
report_utilization -file [file join $build_dir utilization_route.rpt]
report_timing_summary -file [file join $build_dir timing_route.rpt]
report_drc -file [file join $build_dir drc_route.rpt]
report_io -file [file join $build_dir io_route.rpt]
write_bitstream -force [file join $build_dir flash_readonly_probe_top.bit]

set timing_paths [get_timing_paths -max_paths 1 -nworst 1]
if {[llength $timing_paths] > 0} {
  puts "RESULT worst_slack_ns=[get_property SLACK [lindex $timing_paths 0]]"
}
puts "RESULT bitstream=[file join $build_dir flash_readonly_probe_top.bit]"
exit

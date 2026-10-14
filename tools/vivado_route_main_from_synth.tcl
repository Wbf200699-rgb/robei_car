set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set build_dir [file join $repo_dir little_car build vivado_main]
set checkpoint_path [file join $build_dir post_synth.dcp]
set bitstream_path [file join $build_dir sys_route_gui_fsm_top.bit]

if {![file exists $checkpoint_path]} {
  puts stderr "ERROR: synthesis checkpoint is missing: $checkpoint_path"
  exit 2
}
if {[file exists $bitstream_path]} {
  file delete -force $bitstream_path
}

open_checkpoint $checkpoint_path
opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force [file join $build_dir post_route.dcp]
report_route_status -file [file join $build_dir route_status.rpt]
report_utilization -file [file join $build_dir utilization_route.rpt]
report_timing_summary -file [file join $build_dir timing_route.rpt]
report_drc -file [file join $build_dir drc_route.rpt]
write_bitstream -force $bitstream_path

set timing_paths [get_timing_paths -max_paths 1 -nworst 1]
if {[llength $timing_paths] > 0} {
  puts "RESULT worst_slack_ns=[get_property SLACK [lindex $timing_paths 0]]"
}
puts "RESULT bitstream=$bitstream_path"
exit 0

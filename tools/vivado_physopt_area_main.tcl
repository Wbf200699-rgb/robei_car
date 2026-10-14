set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set build_dir [file join $repo_dir little_car build vivado_main]
set checkpoint [file join $build_dir post_route_area.dcp]
set bitstream [file join $build_dir sys_route_gui_fsm_top.bit]

if {![file exists $checkpoint]} {
  puts stderr "ERROR: routed area checkpoint is missing: $checkpoint"
  exit 2
}
if {[file exists $bitstream]} {
  file delete -force $bitstream
}

open_checkpoint $checkpoint
phys_opt_design -directive AggressiveExplore
route_design -directive AggressiveExplore
write_checkpoint -force [file join $build_dir post_route_area_physopt.dcp]
report_route_status -file [file join $build_dir route_status_area_physopt.rpt]
report_utilization -file [file join $build_dir utilization_route_area_physopt.rpt]
report_timing_summary -file [file join $build_dir timing_route_area_physopt.rpt]
report_drc -file [file join $build_dir drc_route_area_physopt.rpt]
write_bitstream -force $bitstream

set paths [get_timing_paths -max_paths 1 -nworst 1]
if {[llength $paths] > 0} {
  puts "RESULT worst_slack_ns=[get_property SLACK [lindex $paths 0]]"
}
puts "RESULT bitstream=$bitstream"
exit 0

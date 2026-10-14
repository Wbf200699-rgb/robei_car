set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set build_dir [file join $repo_dir little_car build vivado_main]

open_checkpoint [file join $build_dir post_route.dcp]
phys_opt_design -directive AggressiveExplore
route_design -directive Explore
write_checkpoint -force [file join $build_dir post_route_physopt.dcp]
report_route_status -file [file join $build_dir route_status_physopt.rpt]
report_utilization -file [file join $build_dir utilization_route_physopt.rpt]
report_timing_summary -file [file join $build_dir timing_route_physopt.rpt]

set paths [get_timing_paths -max_paths 1 -nworst 1]
if {[llength $paths] > 0} {
  puts "RESULT worst_slack_ns=[get_property SLACK [lindex $paths 0]]"
}

write_bitstream -force [file join $build_dir sys_route_gui_fsm_top.bit]
exit 0

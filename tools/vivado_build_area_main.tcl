set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set source_dir [file join $repo_dir little_car build]
set build_dir [file join $source_dir vivado_main]
set filelist_path [file join $source_dir filelist.vf]
set xdc_path [file join $source_dir top.xdc]
set bitstream_path [file join $build_dir sys_route_gui_fsm_top.bit]
file mkdir $build_dir

if {[file exists $bitstream_path]} {
  file delete -force $bitstream_path
}

set fh [open $filelist_path r]
set rtl_files {}
while {[gets $fh line] >= 0} {
  set line [string trim $line]
  if {$line eq "" || [string match "#*" $line]} {
    continue
  }
  set source [file normalize $line]
  if {![file exists $source]} {
    puts stderr "ERROR: RTL source does not exist: $source"
    close $fh
    exit 2
  }
  lappend rtl_files $source
}
close $fh

read_verilog $rtl_files
read_xdc $xdc_path
synth_design -top sys_route_gui_fsm_top -part xc7a50tfgg484-1 \
  -flatten_hierarchy full -directive AreaOptimized_high

set hdmi_ports [get_ports {
  hdmi_clk_p hdmi_clk_n hdmi_d0_p hdmi_d0_n
  hdmi_d1_p hdmi_d1_n hdmi_d2_p hdmi_d2_n
}]
set_property IOSTANDARD TMDS_33 $hdmi_ports

write_checkpoint -force [file join $build_dir post_synth_area.dcp]
report_utilization -file [file join $build_dir utilization_synth_area.rpt]
opt_design -directive ExploreArea
place_design -directive Explore
phys_opt_design -directive AggressiveExplore
route_design -directive Explore

write_checkpoint -force [file join $build_dir post_route_area.dcp]
report_route_status -file [file join $build_dir route_status_area.rpt]
report_utilization -file [file join $build_dir utilization_route_area.rpt]
report_timing_summary -file [file join $build_dir timing_route_area.rpt]
report_drc -file [file join $build_dir drc_route_area.rpt]
write_bitstream -force $bitstream_path

set timing_paths [get_timing_paths -max_paths 1 -nworst 1]
if {[llength $timing_paths] > 0} {
  puts "RESULT worst_slack_ns=[get_property SLACK [lindex $timing_paths 0]]"
}
puts "RESULT rtl_file_count=[llength $rtl_files]"
puts "RESULT bitstream=$bitstream_path"
exit 0

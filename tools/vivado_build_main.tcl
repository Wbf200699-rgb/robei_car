set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set car_dir [file join $repo_dir little_car]
set source_dir [file join $car_dir build]
set build_dir [file join $source_dir vivado_main]
set filelist_path [file join $source_dir filelist.vf]
set xdc_path [file join $source_dir top.xdc]
set bitstream_path [file join $build_dir sys_route_gui_fsm_top.bit]
file mkdir $build_dir

set synthesis_only [expr {
  [info exists ::env(VIVADO_SYNTH_ONLY)] &&
  $::env(VIVADO_SYNTH_ONLY) eq "1"
}]

# Never leave an older bitstream looking valid after a later build failure.
if {!$synthesis_only && [file exists $bitstream_path]} {
  file delete -force $bitstream_path
}

set fh [open $filelist_path r]
set rtl_files {}
while {[gets $fh line] >= 0} {
  set line [string trim $line]
  if {$line eq "" || [string match "#*" $line]} {
    continue
  }
  # New exports use relative paths; preserve old absolute filelists as well.
  if {[file pathtype $line] eq "absolute"} {
    set source [file normalize $line]
  } else {
    set source [file normalize [file join $source_dir $line]]
  }
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

# This design uses only RTL and explicit Xilinx primitives already present in
# the Robei project. No XCI/DCP IP products are read or generated here.
synth_design -top sys_route_gui_fsm_top -part xc7a50tfgg484-1

# The Robei constraint model records connector pins as LVCMOS33. Override the
# differential HDMI ports only after synthesis, when get_ports has an open
# design to query.
set hdmi_ports [get_ports {
  hdmi_clk_p hdmi_clk_n hdmi_d0_p hdmi_d0_n
  hdmi_d1_p hdmi_d1_n hdmi_d2_p hdmi_d2_n
}]
set_property IOSTANDARD TMDS_33 $hdmi_ports

write_checkpoint -force [file join $build_dir post_synth.dcp]
report_utilization -file [file join $build_dir utilization_synth.rpt]
report_utilization -hierarchical -hierarchical_depth 4 \
  -file [file join $build_dir utilization_synth_hierarchical.rpt]
report_timing_summary -file [file join $build_dir timing_synth.rpt]

if {$synthesis_only} {
  puts "RESULT rtl_file_count=[llength $rtl_files]"
  puts "RESULT synthesis_checkpoint=[file join $build_dir post_synth.dcp]"
  puts "RESULT synthesis_only=1"
  exit 0
}

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
puts "RESULT rtl_file_count=[llength $rtl_files]"
puts "RESULT bitstream=$bitstream_path"
exit 0

set repo_root [file normalize [file join [file dirname [info script]] ..]]
set source_file [file join $repo_root little_car diagnostics uwb_uart_runmode_probe_top.v]
set constraint_file [file join $repo_root little_car diagnostics uwb_uart_runmode_probe_top.xdc]
set output_dir [file join $repo_root little_car build uwb_uart_runmode_probe]
set bitstream [file join $output_dir top.bit]

file mkdir $output_dir
read_verilog $source_file
read_xdc $constraint_file
synth_design -top uwb_uart_runmode_probe_top -part xc7a50tfgg484-1
opt_design
place_design
route_design
report_utilization -file [file join $output_dir utilization.rpt]
report_timing_summary -file [file join $output_dir timing_summary.rpt]
write_bitstream -force $bitstream

puts "RESULT bitstream=$bitstream"
puts "RESULT build_complete=1"
exit 0

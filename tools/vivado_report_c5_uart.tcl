set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set build_dir [file join $repo_dir little_car build vivado_main]
set checkpoint [file join $build_dir post_route_physopt.dcp]
if {![file exists $checkpoint]} {
    set checkpoint [file join $build_dir post_route.dcp]
}

open_checkpoint $checkpoint

set report_file [file join $build_dir c5_uart_physical_report.txt]
set fp [open $report_file w]
set port [get_ports -quiet c5_uart_tx]
puts $fp "checkpoint=$checkpoint"
puts $fp "port=$port"
foreach property {PACKAGE_PIN IOSTANDARD DRIVE SLEW PULLTYPE} {
    puts $fp "$property=[get_property $property $port]"
}

set net [get_nets -quiet -of_objects $port]
puts $fp "net=$net"
set drivers [get_pins -quiet -of_objects $net -filter {DIRECTION == OUT}]
puts $fp "drivers=$drivers"
foreach pin $drivers {
    set cell [get_cells -quiet -of_objects $pin]
    puts $fp "driver_pin=$pin"
    puts $fp "driver_cell=$cell"
    puts $fp "driver_ref=[get_property REF_NAME $cell]"
    puts $fp "driver_site=[get_property SITE $cell]"
    puts $fp "driver_bel=[get_property BEL $cell]"
    puts $fp "driver_loc=[get_property LOC $cell]"
    puts $fp "driver_iob=[get_property IOB $cell]"
}

puts $fp "route_status=[get_property ROUTE_STATUS $net]"
puts $fp "is_route_fixed=[get_property IS_ROUTE_FIXED $net]"
puts $fp "route=[get_property ROUTE $net]"
close $fp

report_timing -to $port -max_paths 10 -path_type full_clock_expanded \
    -file [file join $build_dir c5_uart_timing.rpt]
puts "C5_UART_REPORT=$report_file"
exit

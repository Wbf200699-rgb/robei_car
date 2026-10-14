set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set build_dir [file join $repo_dir little_car build vivado_main]
set checkpoint [file join $build_dir post_route.dcp]

if {![file exists $checkpoint]} {
  puts stderr "ERROR: routed checkpoint is missing: $checkpoint"
  exit 2
}

open_checkpoint $checkpoint

set flash_ports {
  flash_cs_n flash_clk flash_mosi flash_miso flash_wp_n flash_hold_n
}
foreach port_name $flash_ports {
  set port [get_ports -quiet $port_name]
  if {[llength $port] != 1} {
    puts stderr "ERROR: expected one port named $port_name"
    exit 3
  }
  puts "RESULT port=$port_name package_pin=[get_property PACKAGE_PIN $port] iostandard=[get_property IOSTANDARD $port] direction=[get_property DIRECTION $port]"
  set package_pin [get_package_pins -quiet [get_property PACKAGE_PIN $port]]
  if {[llength $package_pin] == 1} {
    puts "RESULT package_pin=$package_pin functions=[get_property PIN_FUNC $package_pin]"
  }
}

set startup_cells [get_cells -hier -quiet -filter {REF_NAME == STARTUPE2}]
puts "RESULT startupe2_count=[llength $startup_cells]"
if {[llength $startup_cells] != 0} {
  puts stderr "ERROR: user Flash clock must use GPIO D16, not STARTUPE2/CCLK_0"
  exit 4
}
foreach cell $startup_cells {
  puts "RESULT startupe2=$cell site=[get_property SITE $cell]"
  foreach pin_name {USRCCLKO USRCCLKTS EOS} {
    set pin [get_pins -quiet $cell/$pin_name]
    if {[llength $pin] == 1} {
      set net [get_nets -quiet -of_objects $pin]
      puts "RESULT startupe2_pin=$pin net=$net"
    }
  }
}

report_io -file [file join $build_dir flash_interface_io.rpt]
exit 0

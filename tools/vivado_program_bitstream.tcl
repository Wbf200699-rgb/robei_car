if {$argc != 1} {
  puts stderr "ERROR: expected one bitstream path"
  exit 2
}

set bitstream [file normalize [lindex $argv 0]]
if {![file exists $bitstream]} {
  puts stderr "ERROR: bitstream does not exist: $bitstream"
  exit 2
}

open_hw_manager
connect_hw_server -allow_non_jtag

set opened 0
foreach target [get_hw_targets] {
  if {[catch {
    current_hw_target $target
    open_hw_target
    set opened 1
  } message]} {
    puts "INFO: unable to open target $target: $message"
  } else {
    break
  }
}

if {!$opened} {
  puts stderr "ERROR: no JTAG hardware target could be opened"
  close_hw_manager
  exit 3
}

set candidates {}
foreach device [get_hw_devices] {
  set part [string tolower [get_property PART $device]]
  if {[string match "xc7a50t*" $part]} {
    lappend candidates $device
  }
}

if {[llength $candidates] != 1} {
  puts stderr "ERROR: expected exactly one XC7A50T, found [llength $candidates]"
  foreach device [get_hw_devices] {
    puts stderr "DEVICE: $device part=[get_property PART $device]"
  }
  close_hw_target
  disconnect_hw_server
  close_hw_manager
  exit 4
}

set device [lindex $candidates 0]
current_hw_device $device
refresh_hw_device -update_hw_probes false $device
puts "RESULT target=[current_hw_target]"
puts "RESULT device=$device"
puts "RESULT part=[get_property PART $device]"
puts "RESULT bitstream=$bitstream"

set_property PROGRAM.FILE $bitstream $device
program_hw_devices $device
refresh_hw_device -update_hw_probes false $device

set program_file [get_property PROGRAM.FILE $device]
if {[file normalize $program_file] ne $bitstream} {
  puts stderr "ERROR: programmed file does not match requested bitstream"
  close_hw_target
  disconnect_hw_server
  close_hw_manager
  exit 5
}

puts "RESULT program_complete=1"
close_hw_target
disconnect_hw_server
close_hw_manager
exit 0

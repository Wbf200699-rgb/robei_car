if {$argc != 1} {
  puts stderr "ERROR: expected one user bitstream path"
  exit 2
}

set user_bitstream [file normalize [lindex $argv 0]]
if {![file exists $user_bitstream]} {
  puts stderr "ERROR: bitstream does not exist: $user_bitstream"
  exit 2
}

set output_dir [file join [file dirname $user_bitstream] cfgmem]
file mkdir $output_dir
set readback_file [file join $output_dir cfgmem_readonly_probe_256.bin]
set cfgmem_part_name "mt25ql128-spi-x1_x2_x4"
set hw_open 0
set target_open 0
set device ""
set exit_code 0

if {[catch {
  open_hw_manager
  set hw_open 1
  connect_hw_server -allow_non_jtag

  set opened 0
  foreach target [get_hw_targets] {
    if {![catch {
      current_hw_target $target
      open_hw_target
      set target_open 1
      set opened 1
    } message]} {
      break
    }
    puts "INFO: unable to open target $target: $message"
  }
  if {!$opened} {
    error "no JTAG hardware target could be opened"
  }

  set candidates {}
  foreach candidate [get_hw_devices] {
    if {[string match "xc7a50t*" [string tolower [get_property PART $candidate]]]} {
      lappend candidates $candidate
    }
  }
  if {[llength $candidates] != 1} {
    error "expected exactly one XC7A50T, found [llength $candidates]"
  }

  set device [lindex $candidates 0]
  current_hw_device $device
  refresh_hw_device -update_hw_probes false $device

  set parts [get_cfgmem_parts $cfgmem_part_name]
  if {[llength $parts] == 0} {
    error "unsupported cfgmem part: $cfgmem_part_name"
  }
  set cfgmem [create_hw_cfgmem -hw_device $device [lindex $parts 0]]
  set access_bit [get_property PROGRAM.HW_CFGMEM_BITFILE $device]
  create_hw_bitstream -hw_device $device $access_bit
  program_hw_devices $device
  refresh_hw_device -update_hw_probes false $device

  # Read only the first 256 bytes. No erase, program, blank-check, or boot
  # operation is issued by this diagnostic.
  readback_hw_cfgmem -force -offset 0x0 -datacount 0x100 \
      -format bin -file $readback_file $cfgmem
  if {![file exists $readback_file]} {
    error "configuration Flash readback file was not created"
  }
  puts "RESULT cfgmem_readback_complete=1"
  puts "RESULT cfgmem_readback_bytes=[file size $readback_file]"
  puts "RESULT cfgmem_readback_file=$readback_file"
} message options]} {
  puts stderr "ERROR: read-only cfgmem probe failed: $message"
  if {[dict exists $options -errorinfo]} {
    puts stderr [dict get $options -errorinfo]
  }
  set exit_code 1
}

# The cfgmem access bitstream is volatile. Always restore the requested user
# design before releasing the cable, including after a readback failure.
if {$device ne ""} {
  if {[catch {
    set_property PROGRAM.FILE $user_bitstream $device
    program_hw_devices $device
    refresh_hw_device -update_hw_probes false $device
    puts "RESULT user_bitstream_restored=1"
  } restore_message]} {
    puts stderr "ERROR: failed to restore user bitstream: $restore_message"
    set exit_code 1
  }
}

if {$target_open} {
  catch {close_hw_target}
}
if {$hw_open} {
  catch {disconnect_hw_server}
  catch {close_hw_manager}
}
exit $exit_code

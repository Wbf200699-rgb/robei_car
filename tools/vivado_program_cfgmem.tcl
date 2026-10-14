if {$argc < 1 || $argc > 2} {
  puts stderr "ERROR: expected bitstream path and optional cfgmem part"
  exit 2
}

set bitstream [file normalize [lindex $argv 0]]
set cfgmem_part_name "mt25ql128-spi-x1_x2_x4"
if {$argc == 2 && [string length [lindex $argv 1]] > 0} {
  set cfgmem_part_name [lindex $argv 1]
}
set skip_backup [expr {
  [info exists ::env(VIVADO_CFGMEM_SKIP_BACKUP)] &&
  $::env(VIVADO_CFGMEM_SKIP_BACKUP) eq "1"
}]

if {![file exists $bitstream]} {
  puts stderr "ERROR: bitstream does not exist: $bitstream"
  exit 2
}
if {[file size $bitstream] > 16777216} {
  puts stderr "ERROR: bitstream exceeds the 16 MiB MT25QL128 capacity"
  exit 2
}

set output_dir [file join [file dirname $bitstream] cfgmem]
file mkdir $output_dir
set stem [file rootname [file tail $bitstream]]
set cfgmem_base [file join $output_dir "${stem}_mt25ql128_spi_x1"]
set cfgmem_file "${cfgmem_base}.mcs"
set backup_stamp [clock format [clock seconds] -format "%Y%m%d_%H%M%S"]
set backup_file [file join $output_dir "mt25ql128_before_${backup_stamp}.bin"]

write_cfgmem -force -format MCS -size 128 -interface SPIx1 \
  -loadbit "up 0x00000000 $bitstream" $cfgmem_base
if {![file exists $cfgmem_file]} {
  puts stderr "ERROR: write_cfgmem did not create $cfgmem_file"
  exit 3
}

set part_matches [get_cfgmem_parts $cfgmem_part_name]
if {[llength $part_matches] == 0} {
  puts stderr "ERROR: unsupported cfgmem part: $cfgmem_part_name"
  exit 3
}
set cfgmem_part [lindex $part_matches 0]

set hw_open 0
set target_open 0
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
      set opened 1
      set target_open 1
    } message]} {
      break
    }
    puts "INFO: unable to open target $target: $message"
  }
  if {!$opened} {
    error "no JTAG hardware target could be opened"
  }

  set candidates {}
  foreach device [get_hw_devices] {
    set part [string tolower [get_property PART $device]]
    if {[string match "xc7a50t*" $part]} {
      lappend candidates $device
    }
  }
  if {[llength $candidates] != 1} {
    error "expected exactly one XC7A50T, found [llength $candidates]"
  }

  set device [lindex $candidates 0]
  current_hw_device $device
  refresh_hw_device -update_hw_probes false $device

  set cfgmem [create_hw_cfgmem -hw_device $device $cfgmem_part]
  set access_bit [get_property PROGRAM.HW_CFGMEM_BITFILE $device]
  create_hw_bitstream -hw_device $device $access_bit
  program_hw_devices $device
  refresh_hw_device -update_hw_probes false $device

  if {$skip_backup} {
    puts "RESULT backup_skipped=1"
  } else {
    puts "RESULT backup_started=$backup_file"
    readback_hw_cfgmem -force -all -format bin -file $backup_file $cfgmem
    if {![file exists $backup_file] || [file size $backup_file] != 16777216} {
      error "configuration Flash backup is missing or not exactly 16 MiB"
    }
    puts "RESULT backup_complete=1"
    puts "RESULT backup_bytes=[file size $backup_file]"
  }

  set_property PROGRAM.FILES [list $cfgmem_file] $cfgmem
  set_property PROGRAM.ADDRESS_RANGE {use_file} $cfgmem
  set_property PROGRAM.BLANK_CHECK 1 $cfgmem
  set_property PROGRAM.ERASE 1 $cfgmem
  set_property PROGRAM.CFG_PROGRAM 1 $cfgmem
  set_property PROGRAM.VERIFY 1 $cfgmem

  puts "RESULT cfgmem_program_started=1"
  program_hw_cfgmem $cfgmem
  puts "RESULT cfgmem_program_complete=1"
  puts "RESULT cfgmem_verify_complete=1"

  boot_hw_device $device
  after 1500
  refresh_hw_device -update_hw_probes false $device
  puts "RESULT flash_boot_requested=1"
  puts "RESULT target=[current_hw_target]"
  puts "RESULT device=$device"
  puts "RESULT fpga_part=[get_property PART $device]"
  puts "RESULT cfgmem_part=$cfgmem_part_name"
  puts "RESULT cfgmem_file=$cfgmem_file"
  puts "RESULT bitstream=$bitstream"
} message options]} {
  puts stderr "ERROR: cfgmem operation failed: $message"
  if {[dict exists $options -errorinfo]} {
    puts stderr [dict get $options -errorinfo]
  }
  set exit_code 1
}

if {$target_open} {
  catch {close_hw_target}
}
if {$hw_open} {
  catch {disconnect_hw_server}
  catch {close_hw_manager}
}
exit $exit_code

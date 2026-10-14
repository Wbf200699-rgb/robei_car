open_hw_manager
connect_hw_server -allow_non_jtag

set opened 0
foreach target [get_hw_targets] {
  if {![catch {
    current_hw_target $target
    open_hw_target
    set opened 1
  } message]} {
    break
  }
  puts "INFO unable_to_open_target=$target message=$message"
}

if {!$opened} {
  puts stderr "ERROR no JTAG hardware target could be opened"
  close_hw_manager
  exit 2
}

foreach device [get_hw_devices] {
  current_hw_device $device
  refresh_hw_device -update_hw_probes true $device
  puts "RESULT device=$device"
  puts "RESULT part=[get_property PART $device]"
  foreach property {
    PROGRAM.FILE
    REGISTER.IR.BOOTSTS
    REGISTER.IR.STATUS
    REGISTER.EFUSE.FUSE_CNTL
    REGISTER.USR_ACCESS
    REGISTER.USER_ACCESS
  } {
    if {![catch {set value [get_property $property $device]}]} {
      puts "RESULT $property=$value"
    }
  }
  set ilas [get_hw_ilas -quiet -of_objects $device]
  set vios [get_hw_vios -quiet -of_objects $device]
  set probes [get_hw_probes -quiet -of_objects $device]
  puts "RESULT ila_count=[llength $ilas]"
  puts "RESULT vio_count=[llength $vios]"
  puts "RESULT probe_count=[llength $probes]"
  foreach ila $ilas {
    puts "RESULT ila=$ila"
  }
  foreach vio $vios {
    puts "RESULT vio=$vio"
  }
  foreach property [lsort [list_property $device]] {
    if {[string match "REGISTER.*" $property]} {
      if {![catch {set value [get_property $property $device]}]} {
        puts "RESULT property=$property value=$value"
      }
    }
  }
}

close_hw_target
disconnect_hw_server
close_hw_manager
exit 0

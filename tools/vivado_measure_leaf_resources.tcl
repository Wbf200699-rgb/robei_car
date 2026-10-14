set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set build_dir [file join $repo_dir little_car build]
set report_dir [file join $build_dir vivado_leaf_resources]
file mkdir $report_dir

set targets {
  {car_param_controller car_param_controller.v}
  {route_contol_core route_contol_core.v}
  {hdmi_timing_gen hdmi_timing_gen.v}
  {c5_motor_frame_driver c5_motor_frame_driver.v}
  {gui_route_renderer gui_route_renderer.v}
  {bt_route_uart bt_route_uart.v}
  {rplidar_c1_uart rplidar_c1_uart.v}
  {lidar_map_sampler lidar_map_sampler.v}
  {onboard_occupancy_map onboard_occupancy_map.v}
  {map_flash_store map_flash_store.v}
  {vl53l1x_array5 ../hdmi_rgb_input/vl53l1x_array5.v}
}

foreach target $targets {
  lassign $target top source_names
  if {[info exists ::env(LEAF_TOP)] && $::env(LEAF_TOP) ne "" &&
      $::env(LEAF_TOP) ne $top} {
    continue
  }
  set source_paths {}
  foreach source_name $source_names {
    set source_path [file normalize [file join $build_dir $source_name]]
    if {![file exists $source_path]} {
      puts stderr "ERROR: missing RTL source $source_path"
      exit 2
    }
    lappend source_paths $source_path
  }

  create_project -in_memory -part xc7a50tfgg484-1
  read_verilog $source_paths
  synth_design -top $top -part xc7a50tfgg484-1
  report_utilization -file [file join $report_dir ${top}.rpt]

  puts "RESULT leaf=$top report=[file join $report_dir ${top}.rpt]"
  close_project
}

exit 0

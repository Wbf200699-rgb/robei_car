set script_dir [file dirname [file normalize [info script]]]
set repo_dir [file dirname $script_dir]
set build_dir [file join $repo_dir little_car build vivado_main]
set checkpoint [file join $build_dir post_synth.dcp]

if {![file exists $checkpoint]} {
  puts stderr "ERROR: synthesis checkpoint does not exist: $checkpoint"
  exit 2
}

open_checkpoint $checkpoint
report_utilization -hierarchical -hierarchical_depth 4 \
  -file [file join $build_dir utilization_synth_hierarchical.rpt]
puts "RESULT report=[file join $build_dir utilization_synth_hierarchical.rpt]"
exit 0

set script_dir [file dirname [file normalize [info script]]]
set rtl_files [glob -nocomplain [file join $script_dir "verilog" "*.v"]]

if {[llength $rtl_files] == 0} {
    error "No exported Verilog files found"
}

read_verilog $rtl_files
synth_design -rtl -top sys_route_gui_fsm_top -part xc7a50tfgg484-1
report_compile_order -used_in synthesis

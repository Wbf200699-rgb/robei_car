set script_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $script_dir ".."]]
set filelist_path [file join $repo_root "little_car" "build" "filelist.vf"]
set xdc_path [file join $repo_root "little_car" "build" "top.xdc"]

set fh [open $filelist_path r]
set rtl_files {}
while {[gets $fh line] >= 0} {
    set line [string trim $line]
    if {$line eq "" || [string match "#*" $line]} {
        continue
    }
    lappend rtl_files [file normalize $line]
}
close $fh

read_verilog $rtl_files
read_xdc $xdc_path
synth_design -rtl -top sys_route_gui_fsm_top -part xc7a50tfgg484-1
report_compile_order -used_in synthesis

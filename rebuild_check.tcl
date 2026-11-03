# 注意：open_project 不会自动导入器件，必须先 import_device，
# 否则 settings.cfg 里 device_name/package_name 为空，子进程 run.exe 会立刻退出，
# 父进程则卡在 wait_run 上无限等待。（见 SWUG105 手册 3.3 节 single run 示例）
if {[catch {
    set script_dir [file dirname [file normalize [info script]]]
    # 路径必须用字符串拼接构造：file join 返回的对象带 Tcl list 内部表示，
    # 直接传给 open_project 会被 TD 6.2.168116 误解析成工程目录（PRJ-8601）。
    set project_dir "$script_dir/td_project"
    set project_file "$project_dir/camera_to_dsi_display.al"
    import_device ph1_35p.db -package PH1P35MDG324 -speed 3
    cd $project_dir
    open_project $project_file
    set build_started [clock seconds]
    reset_runs syn_1
    launch_runs {syn_1 phy_1} -jobs 2
    wait_run syn_1 -quiet
    wait_run phy_1 -quiet
    # TD can report a successful run and produce a bit even with negative slack.
    # Require a fresh final report and nonnegative setup/hold slack explicitly.
    set timing_file camera_to_dsi_display_Runs/phy_1/final_timing.rpt
    if {![file exists $timing_file] || [file mtime $timing_file] < $build_started} {
        error "Missing fresh final timing report"
    }
    set timing_handle [open $timing_file r]
    set timing_text [read $timing_handle]
    close $timing_handle
    if {![regexp {SWNS: *(-?[0-9.]+)ns} $timing_text unused setup_wns] ||
        ![regexp {HWNS: *(-?[0-9.]+)ns} $timing_text unused hold_wns]} {
        error "Cannot parse final setup/hold slack"
    }
    # The dual-axis commissioning wrapper explicitly allows generation with
    # reported timing warnings, per the user's delivery preference.
    set allow_timing_warning [expr {[info exists ::tracking_commissioning] && $::tracking_commissioning}]
    if {$setup_wns < 0 && !$allow_timing_warning} {
        error "Timing failed: setup=$setup_wns ns"
    }
    if {$hold_wns < 0 && !$allow_timing_warning} {
        error "Timing failed: hold=$hold_wns ns"
    }
    # TD's automatic V2 fractional-PLL derivation is inaccurate on this release.
    # Validate the effective clocks after the explicit SDC override, not its log
    # of the discarded auto clocks. STA quantizes the serial period to 1 ps.
    if {![regexp {clk[0-9]+: +([0-9.]+) +[0-9]+ +HDMI_PIXEL_CLK} $timing_text unused pixel_mhz] ||
        ![regexp {clk[0-9]+: +([0-9.]+) +[0-9]+ +HDMI_SERIAL_CLK} $timing_text unused serial_mhz] ||
        abs($pixel_mhz-74.25) > 0.01 || abs($serial_mhz-371.25) > 0.10} {
        error "Final HDMI clock constraints do not match 720p PLL"
    }
    save_best_bits
    close_project
    if {$setup_wns < 0 || $hold_wns < 0} {
        puts "BUILD CANDIDATE: synthesis, place/route, bitgen completed; TIMING NOT MET: setup=$setup_wns ns hold=$hold_wns ns"
    } else {
        puts "BUILD PASS: synthesis, place/route, bitgen; setup=$setup_wns ns hold=$hold_wns ns"
    }
} build_error]} {
    puts "BUILD FAILED: $build_error"
    exit 1
}
exit

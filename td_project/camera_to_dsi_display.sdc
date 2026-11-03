create_clock -name {sys_clk_50m} -period 20.000 -waveform {0.000 10.000} [get_ports {I_sys_clk}]
# External keys feed only first-stage input synchronizers. Internal stages remain timed.
set_false_path -from [get_ports {I_keys_n[*]}]
# PWM is a 50 Hz position signal, with no receiving clock or synchronous setup.
set_output_delay -clock [get_clocks {sys_clk_50m}] -max 0.000 [get_ports {O_servo_pan_pwm O_servo_tilt_pwm}]
set_output_delay -clock [get_clocks {sys_clk_50m}] -min 0.000 [get_ports {O_servo_pan_pwm O_servo_tilt_pwm}]
create_clock -name {mipi_rx_ck_pad} -period 2.222 -waveform {0.000 1.111} [get_nets {IO_rx_clk_pad_p}]

derive_clocks

rename_clock -name {pll_clk_100m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
# Buzzer is registered in the 100 MHz keypad domain; no external receiver clock.
set_output_delay -clock [get_clocks {pll_clk_100m}] -max 0.000 [get_ports {O_buzzer}]
set_output_delay -clock [get_clocks {pll_clk_100m}] -min 0.000 [get_ports {O_buzzer}]
rename_clock -name {pll_clk_24m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}]
# TD 6.2.168116 auto-derives this V2 fractional PLL incorrectly as 74.666/373.333
# MHz, and independently rounds the ratios, breaking their exact 5:1 relation.
# The mapped primitive retains N=3, M=89, FRAC=26843546/2^28; vendor calculator
# and V2 simulation both confirm 74.25/371.25 MHz. Constrain that actual clock.
remove_clock -name {u_HDMI_PLL/u_pll/u_PH1P_PHY_PLL.clkc[4]}
remove_clock -name {u_HDMI_PLL/u_pll/u_PH1P_PHY_PLL.clkc[5]}
create_generated_clock -name {HDMI_PIXEL_CLK} -source [get_ports {I_sys_clk}] -master_clock {sys_clk_50m} -multiply_by 1.485 -duty_cycle 50 [get_pins {u_HDMI_PLL/u_pll/u_PH1P_PHY_PLL.clkc[4]}]
create_generated_clock -name {HDMI_SERIAL_CLK} -source [get_ports {I_sys_clk}] -master_clock {sys_clk_50m} -multiply_by 7.425 -duty_cycle 50 [get_pins {u_HDMI_PLL/u_pll/u_PH1P_PHY_PLL.clkc[5]}]

rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]

set_clock_groups -asynchronous \
    -group [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] \
    -group [get_clocks {sys_clk_50m pll_clk_100m pll_clk_24m}] \
    -group [get_clocks {HDMI_PIXEL_CLK HDMI_SERIAL_CLK}] \
    -group [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/ddr_clk u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]

# 100MHz 控制状态域到 24MHz 摄像头配置域、以及到 CSI/HDMI 像素域的三个控制开关
# 都是"两拍同步 + 握手"的异步交接，本就不存在同频时序关系。TD 会把同一 PLL
# 派生的 pll_clk_100m 与 pll_clk_24m 当作同组同步时钟去算 hold，而两个 PLL 输出
# 之间约 1ns 的 skew 无法用 RTL 消除，于是产生与设计意图无关的 hold 违例。
# 用 datapath_only 保留数据路径上界，同时去掉这类跨时钟 hold 检查。
set_max_delay -datapath_only 20.000 \
    -from [get_clocks {pll_clk_100m}] -to [get_clocks {pll_clk_24m}]

# Async assertion is intentional ONLY at these two-stage reset synchronizers.
# TD reads this SDC before packing: gate FF reset pins are .sr, while packed
# FF reset pins are .asr. Matching only .asr silently lost the exception at
# the gate-to-place handoff. Handle both representations, never the D pins.
# Keep stage-to-stage D paths and downstream reset recovery/removal timed.
# Use literal SDC commands: TD's deferred SDC reader does not apply Tcl loops.
set_false_path -to [get_pins -quiet -hier {u_camera_cfg_reset/release_sync_reg*.sr u_camera_cfg_reset/release_sync_reg*.asr}]
set_false_path -to [get_pins -quiet -hier {u_key_tones/u_feedback_reset/release_sync_reg*.sr u_key_tones/u_feedback_reset/release_sync_reg*.asr}]
set_false_path -to [get_pins -quiet -hier {u_security_status_hdmi/g_local_resets$u_src/release_sync_reg*.sr u_security_status_hdmi/g_local_resets$u_src/release_sync_reg*.asr}]
set_false_path -to [get_pins -quiet -hier {u_security_status_hdmi/g_local_resets$u_dst/release_sync_reg*.sr u_security_status_hdmi/g_local_resets$u_dst/release_sync_reg*.asr}]
set_false_path -to [get_pins -quiet -hier {u_security_status_tone/g_local_resets$u_src/release_sync_reg*.sr u_security_status_tone/g_local_resets$u_src/release_sync_reg*.asr}]
set_false_path -to [get_pins -quiet -hier {u_security_status_tone/g_local_resets$u_dst/release_sync_reg*.sr u_security_status_tone/g_local_resets$u_dst/release_sync_reg*.asr}]

# Tracking mode resets assert asynchronously only at these release chains.
# Epoch/filter/delivered-frame data registers now use the local released reset;
# their recovery/removal remains timed. The 100 MHz source restriction below
# also survives gate SR -> packed ASR renaming. Both D chains are local 50 MHz
# (stage-0 D is constant), so their stage-to-stage setup/hold is not excluded.
set_false_path -to [get_pins -quiet -hier {u_tracking_system/mode_sys_reset_reg*.sr u_tracking_system/mode_sys_reset_reg*.asr u_tracking_system/servo_sys_reset_reg*.sr u_tracking_system/servo_sys_reset_reg*.asr u_tracking_system/mode_camera_reset_reg*.sr u_tracking_system/mode_camera_reset_reg*.asr}]
set_false_path -from [get_clocks {pll_clk_100m}] -to [get_regs -hier {u_tracking_system/mode_sys_reset* u_tracking_system/servo_sys_reset*}]

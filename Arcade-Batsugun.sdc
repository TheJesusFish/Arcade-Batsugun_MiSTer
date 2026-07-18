derive_pll_clocks
derive_clock_uncertainty

create_generated_clock -name SDRAM_CLK -source \
    [get_pins {emu|pll|raizingpll_inst|altera_pll_i|general[5].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -divide_by 1 \
    [get_ports SDRAM_CLK]

set_multicycle_path -from [get_clocks {SDRAM_CLK}] -to [get_clocks {emu|pll|raizingpll_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}] -setup -end 2

set_multicycle_path -from [get_clocks {SDRAM_CLK}] -to [get_clocks {emu|pll|raizingpll_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}] -hold -end 2

# The drop-in Template SDC groups *|pll|pll_inst, but this core's PLL
# instance is raizingpll_inst. Keep SDRAM/game clocks related and cut
# unrelated framework/audio/video/HPS clock domains.
set_clock_groups -exclusive \
    -group [get_clocks {emu|pll|raizingpll_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk emu|pll|raizingpll_inst|altera_pll_i|general[5].gpll~PLL_OUTPUT_COUNTER|divclk SDRAM_CLK}] \
    -group [get_clocks {pll_hdmi|pll_hdmi_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] \
    -group [get_clocks {pll_audio|pll_audio_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -group [get_clocks {spi_sck}] \
    -group [get_clocks {hdmi_sck}] \
    -group [get_clocks {sysmem|fpga_interfaces|clocks_resets|h2f_user0_clk}] \
    -group [get_clocks {FPGA_CLK1_50}] \
    -group [get_clocks {FPGA_CLK2_50}] \
    -group [get_clocks {FPGA_CLK3_50}]

#sdram timing
# set_input_delay -max -clock SDRAM_CLK 6.4ns [get_ports SDRAM_DQ[*]]
# set_input_delay -min -clock SDRAM_CLK 3.7ns [get_ports SDRAM_DQ[*]]

# SDRAM timing constraints for the inlined Raizing hierarchy. Upstream
# jtframe/target/mister/syn/sdram_clk96.sdc targets jtframe_mister:u_frame
# and the bank-core controller, which this core does not instantiate.
set_multicycle_path -setup -end -from [get_keepers {SDRAM_DQ[*]}] -to [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|dout[*]}] 2
set_multicycle_path -hold  -end -from [get_keepers {SDRAM_DQ[*]}] -to [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|dout[*]}] 2

set_multicycle_path -setup -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|dq_pad[*]}] -to [get_keepers {SDRAM_DQ[*]}] 2
set_multicycle_path -hold  -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|dq_pad[*]}] -to [get_keepers {SDRAM_DQ[*]}] 2

set_multicycle_path -setup -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|sdram_a[12]}] -to [get_keepers {SDRAM_DQMH}] 2
set_multicycle_path -hold  -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|sdram_a[12]}] -to [get_keepers {SDRAM_DQMH}] 2
set_multicycle_path -setup -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|sdram_a[11]}] -to [get_keepers {SDRAM_DQML}] 2
set_multicycle_path -hold  -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|sdram_a[11]}] -to [get_keepers {SDRAM_DQML}] 2

# The fx68k core documents instruction-register to micro/nano address decode
# as a 2-cycle path; the microcode output is not needed immediately.
set_multicycle_path -start -setup -from [get_keepers {emu:emu|batsugun_game:u_game|fx68k:u_main68k|Ir[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|fx68k:u_main68k|microAddr[*]}] 2
set_multicycle_path -start -hold -from [get_keepers {emu:emu|batsugun_game:u_game|fx68k:u_main68k|Ir[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|fx68k:u_main68k|microAddr[*]}] 1
set_multicycle_path -start -setup -from [get_keepers {emu:emu|batsugun_game:u_game|fx68k:u_main68k|Ir[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|fx68k:u_main68k|nanoAddr[*]}] 2
set_multicycle_path -start -hold -from [get_keepers {emu:emu|batsugun_game:u_game|fx68k:u_main68k|Ir[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|fx68k:u_main68k|nanoAddr[*]}] 1

# The compact V25 advances only when its 16 MHz clock enable is asserted. The
# NCO guarantees at least five 94.5 MHz clocks between enables, and every
# sequential element inside u_cpu is gated by that enable. Keep the exception
# inside the CPU engine so wrapper buses and shared/audio interfaces remain
# single-cycle constrained.
set v25_cpu_keepers [get_keepers {emu:emu|batsugun_game:u_game|batsugun_sound:u_sound|batsugun_v25_cpu:u_v25|batsugun_z8086:u_cpu|*}]
set_multicycle_path -setup -from $v25_cpu_keepers -to $v25_cpu_keepers 5
set_multicycle_path -hold  -from $v25_cpu_keepers -to $v25_cpu_keepers 4

# The visible pixel outputs and debug pixel snapshots are clock-enable registers.
# hcnt/vcnt and the renderer settle across the 14-cycle pixel cadence, and these
# registers only sample on clkdiv==13. Keep this exception narrow so CPU/SDRAM
# paths remain honest.
set_multicycle_path -setup -end -from [get_clocks {emu|pll|raizingpll_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|red[*] emu:emu|batsugun_game:u_game|green[*] emu:emu|batsugun_game:u_game|blue[*] emu:emu|batsugun_game:u_game|HS emu:emu|batsugun_game:u_game|VS emu:emu|batsugun_game:u_game|debug_live0_px[*] emu:emu|batsugun_game:u_game|debug_live1_px[*] emu:emu|batsugun_game:u_game|debug_live2_px[*] emu:emu|batsugun_game:u_game|debug_live3_px[*] emu:emu|batsugun_game:u_game|debug_live4_px[*] emu:emu|batsugun_game:u_game|debug_live5_px[*] emu:emu|batsugun_game:u_game|debug_live6_px[*] emu:emu|batsugun_game:u_game|debug_live7_px[*]}] 14
set_multicycle_path -hold -end -from [get_clocks {emu|pll|raizingpll_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|red[*] emu:emu|batsugun_game:u_game|green[*] emu:emu|batsugun_game:u_game|blue[*] emu:emu|batsugun_game:u_game|HS emu:emu|batsugun_game:u_game|VS emu:emu|batsugun_game:u_game|debug_live0_px[*] emu:emu|batsugun_game:u_game|debug_live1_px[*] emu:emu|batsugun_game:u_game|debug_live2_px[*] emu:emu|batsugun_game:u_game|debug_live3_px[*] emu:emu|batsugun_game:u_game|debug_live4_px[*] emu:emu|batsugun_game:u_game|debug_live5_px[*] emu:emu|batsugun_game:u_game|debug_live6_px[*] emu:emu|batsugun_game:u_game|debug_live7_px[*]}] 13

# Palette RAM lookup address is sampled earlier in the pixel pipeline, on
# clkdiv==8. hcnt/vcnt only advance on clkdiv==13, so coordinate-driven palette
# address paths get the 9-cycle h/v counter to palette-address cadence.
set_multicycle_path -setup -end -from [get_keepers {emu:emu|batsugun_game:u_game|hcnt[*] emu:emu|batsugun_game:u_game|vcnt[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|pal_scan_addr[*]}] 9
set_multicycle_path -hold -end -from [get_keepers {emu:emu|batsugun_game:u_game|hcnt[*] emu:emu|batsugun_game:u_game|vcnt[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|pal_scan_addr[*]}] 8

# GP scroll values are snapshotted into pixel-cadence registers on clkdiv==13
# before renderer use. The palette address samples those renderer results on
# clkdiv==8, so the scroll snapshot to palette-address path has the same
# 9-cycle cadence as hcnt/vcnt.
set_multicycle_path -setup -end -from [get_keepers {emu:emu|batsugun_game:u_game|gp0_scroll*_px[*] emu:emu|batsugun_game:u_game|gp1_scroll*_px[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|pal_scan_addr[*]}] 9
set_multicycle_path -hold -end -from [get_keepers {emu:emu|batsugun_game:u_game|gp0_scroll*_px[*] emu:emu|batsugun_game:u_game|gp1_scroll*_px[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|pal_scan_addr[*]}] 8

# Composite layer source pixels latch on clkdiv==0 and the palette lookup
# address samples the resolved display pixel on clkdiv==8.
set_multicycle_path -setup -end -from [get_keepers {emu:emu|batsugun_game:u_game|comp_latched_lo[*][*] emu:emu|batsugun_game:u_game|comp_latched_hi[*][*] emu:emu|batsugun_game:u_game|comp_latched_color[*][*] emu:emu|batsugun_game:u_game|comp_latched_pri[*][*] emu:emu|batsugun_game:u_game|comp_latched_valid[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|pal_scan_addr[*]}] 8
set_multicycle_path -hold -end -from [get_keepers {emu:emu|batsugun_game:u_game|comp_latched_lo[*][*] emu:emu|batsugun_game:u_game|comp_latched_hi[*][*] emu:emu|batsugun_game:u_game|comp_latched_color[*][*] emu:emu|batsugun_game:u_game|comp_latched_pri[*][*] emu:emu|batsugun_game:u_game|comp_latched_valid[*]}] \
    -to [get_keepers {emu:emu|batsugun_game:u_game|pal_scan_addr[*]}] 7

# JTFRAME
set_false_path -to [get_keepers {audio_out:audio_out|cl1[*]}]
set_false_path -to [get_keepers {audio_out:audio_out|cr1[*]}]

# Reset synchronization signal
set_false_path -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_reset:u_reset|rst_rom[0]}] -to [get_keepers {emu:emu|jtframe_board:u_board|jtframe_reset:u_reset|rst_rom_sync}]
set_false_path -to emu:emu|jtframe_board:u_board|jtframe_reset:u_reset|rst_req_sync[0]
# static signals
set_false_path -from FB_EN
set_false_path -to deb_osd[0]
set_false_path -from emu:emu|jtframe_board:u_board|jtframe_led:u_led|led

set_false_path -to [get_keepers {*altera_std_synchronizer:*|din_s1}]

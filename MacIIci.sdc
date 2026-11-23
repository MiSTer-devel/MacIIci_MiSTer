derive_pll_clocks
derive_clock_uncertainty

# The IIci's constraints.  rtl/pll_iici, one 1200 MHz VCO:
#   general[0] clk_sys 50 MHz      general[1] clk_mem 100 MHz
#   general[2] clk_sdc 100 +2.50   general[3] clk_capa 100 +1.667 (capture A)
#   general[4] clk_capb 100 +0.3125 (capture B)
#   general[5] clk_k 25 MHz (the CPU)
#   general[6] clk_vid 57.14 MHz (the RBV's video)
# clk_sys, clk_k and clk_mem are related and timed exactly; nothing between
# them is relaxed except the SDRAM controller's inputs below.  Timing must be met
# at every corner (TIMEQUEST_MULTICORNER_ANALYSIS).

set pll_out {emu|pll|pll_inst|altera_pll_i|general[%d].gpll~PLL_OUTPUT_COUNTER|divclk}

# ----------------------------------------------------------------------------
# clk_sys -> clk_mem: rtl/iici_sdram.v takes every input from clk_sys (and clk_k)
# only at sample_en, the clk_mem edge that is the next clk_sys edge, 20 ns after
# the launch; phi itself, which finds that edge, has one clk_mem period.
# ----------------------------------------------------------------------------
set iici_in [remove_from_collection [get_keepers {*|iici_sdram:sdram|*}] [get_keepers {*|iici_sdram:sdram|phi_q}]]
set_multicycle_path -setup -end 2 -from [get_clocks [list [format $pll_out 0] [format $pll_out 5]]] -to $iici_in
set_multicycle_path -hold  -end 1 -from [get_clocks [list [format $pll_out 0] [format $pll_out 5]]] -to $iici_in

# ----------------------------------------------------------------------------
# The SDRAM's pins at 100 MHz, CL2.
# SDRAM_CLK is the inverted clk_sdc.  Read data: tAC(CL2) 6.0 ns + 0.5 ns of
# trace max; tOH 2.5 min.  Commands and write data:
# tIS/tDS 1.5 + 0.5, tIH/tDH 0.8.
# ----------------------------------------------------------------------------
create_generated_clock -name sdram_clk -invert \
  -source [get_pins [format $pll_out 2]] \
  [get_ports {SDRAM_CLK}]

set_input_delay  -clock sdram_clk -max 6.5 [get_ports {SDRAM_DQ[*]}]
set_input_delay  -clock sdram_clk -min 2.5 [get_ports {SDRAM_DQ[*]}]

# The read capture (dq_q in the I/O cell, on capture A or B, chosen by the
# training): cut for the compile, timed separately with iici_time_capture set -
# at every corner at least one of the two captures must meet.  Both take the
# word on their second edge after sdram_clk's launch.
if {[info exists ::iici_time_capture]} {
  # (by destination CLOCK: set_multicycle_path has no -to_clock, and the
  # only paths sdram_clk launches into these two clocks are the captures)
  set_multicycle_path -setup -end 2 -from [get_clocks {sdram_clk}] -to [get_clocks [format $pll_out 3]]
  set_multicycle_path -setup -end 2 -from [get_clocks {sdram_clk}] -to [get_clocks [format $pll_out 4]]
} else {
  set_false_path -from [get_clocks {sdram_clk}] -to [get_keepers {*|iici_sdram:sdram|dq_q[*]}]
}
set_clock_groups -exclusive \
  -group [get_clocks [format $pll_out 3]] \
  -group [get_clocks [format $pll_out 4]]

# Fitter-only guides (the analysis keeps the true relationships): dq_q -> dq_r
# placed by the pins' side, and minimum delays on the clk_sys <-> clk_k crossings
# that share an edge with no logic between.
if {[info exists ::TimeQuestInfo(nameofexecutable)] && $::TimeQuestInfo(nameofexecutable) eq "quartus_fit"} {
  set_max_delay -from [get_keepers {*|iici_sdram:sdram|dq_q[*]}] -to [get_keepers {*|iici_sdram:sdram|dq_r[*]}] 7.0
  set CK_SYS [get_clocks {emu|pll|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}]
  set_min_delay -from $CK_SYS -to [get_registers {*|tg68k:cpu|kk_*}] 0.3
  set_min_delay -from [get_registers {*|tg68k:cpu|kk_*}] -to $CK_SYS 0.3
  set_min_delay -from [get_registers {*|tg68k:cpu|TG68KdotC_Kernel:kernel|opcode_pc[*]}] -to $CK_SYS 0.3
  set_min_delay -from $CK_SYS -to [get_registers {*|iici_sdram:sdram|x*}] 0.3
}

# Crossings with no hold relationship (each register pair never moves on a shared edge):
# sdram v_rdata / v_ack -> mdu vid_data / vid_ld: the video's long leaves only between clk_sys edges
set_false_path -hold -from [get_registers {*|iici_sdram:sdram|v_rdata[*] *|iici_sdram:sdram|v_ack}] -to [get_registers {*|iici_mdu:mdu|vid_data[*] *|iici_mdu:mdu|vid_ld}]

# mdu v_base -> sdram xv_addr: xv_addr loads a clk_mem clock after v_base moves
set_false_path -hold -from [get_registers {*|iici_mdu:mdu|v_base[*]}] -to [get_registers {*|iici_sdram:sdram|xv_addr[*]}]

# mdu q_* -> sdram xm_*: xm_* take q_* a clk_sys clock after they move
set_false_path -hold -from [get_registers {*|iici_mdu:mdu|q_*}] -to [get_registers {*|iici_sdram:sdram|xm_*}]

# the wrapper's kk_din -> din_r: kk_din moves at phi1 edges, din_r takes it at phi2 edges
set_false_path -hold -from [get_registers {*|tg68k:cpu|kk_din[*]}] -to [get_registers {*|tg68k:cpu|din_r[*]}]

# the bus registers kb_* -> the MDU's held request q_*: kb_* move at phi1 edges, q_* load at phi2 edges
set_false_path -hold -from [get_registers {*|tg68k:cpu|kb_*}] -to [get_registers {*|iici_mdu:mdu|q_*}]

set SDRAM_OUT [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_DQ[*] SDRAM_DQMH SDRAM_DQML SDRAM_nCAS SDRAM_nRAS SDRAM_nWE SDRAM_nCS SDRAM_CKE}]
set_output_delay -clock sdram_clk -max  2.0 $SDRAM_OUT
set_output_delay -clock sdram_clk -min -0.8 $SDRAM_OUT

# ----------------------------------------------------------------------------
# The 68882's APU: two clk_sys for its datapath (one microinstruction every two).
# ----------------------------------------------------------------------------
set fpu_apu  [get_keepers {*|se30_fpu_apu:apu|*}]
set fpu_p0   [get_keepers {*|se30_fpu_apu:apu|altsyncram:urom_rtl_0|* *|se30_fpu_apu:apu|altsyncram:entry_rtl_0|* *|se30_fpu_apu:apu|ua[*] *|se30_fpu_apu:apu|abort_l}]
set fpu_apu1 [remove_from_collection $fpu_apu $fpu_p0]
set_multicycle_path -setup -end 2 -from $fpu_apu1 -to $fpu_apu1
set_multicycle_path -hold  -end 1 -from $fpu_apu1 -to $fpu_apu1

# ----------------------------------------------------------------------------
# clk_vid: the RBV's video side and CLK_VIDEO, asynchronous to every other clock
# (every crossing is built for it: Gray-coded FIFO, handshakes, dual-clock CLUT).
# ----------------------------------------------------------------------------
set_clock_groups -asynchronous -group [get_clocks [format $pll_out 6]]

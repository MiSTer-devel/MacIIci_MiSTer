// tg68k.v - the 68030: the TG68K.C kernel on the 68030 bus.  Runs the kernel's beats as 68030
// bus cycles (S0-S5, dynamic bus sizing, DSACK and BERR), serves the PMMU's table walker, posts
// writes to RAM (the write pending buffer), terminates interrupt acknowledges (autovector) and
// bus-errors CPU-space cycles to absent coprocessors, holds each instruction to the 68030's
// instruction time (se30_pace030.v), and holds the on-chip caches (se30_cache030.v).
// KCLK = 1 (the IIci): the kernel on clk_k, the CPU clock; every kernel input a kernel output or
// a clk_k register, the bus leaving through clk_k registers.  BURST = 1: burst fills on CBACK*.
// XCACHE = 1: the IIci's RAM cache card.

`timescale 1ns/1ps

module tg68k #(
  parameter KCLK = 0,                  // 1: the kernel on clk_k, the CPU clock
  parameter BURST = 0,                 // 1 (with KCLK): burst fills on CBACK*
  parameter XCACHE = 0                 // 1 (with KCLK): the Macintosh IIci's RAM cache card
) (
  input         clk,                   // 2 x C16M
  input         clk_k,                 // KCLK: the CPU clock, clk / 2, rising at the end of each phi1 clk
  input         phi1,                  // C16M rising edge
  input         phi2,                  // C16M falling edge
  input         reset_n,

  output        ecs,                   // ECS*, active high here: S0 of a cycle that may follow, the new address on the bus
  output [31:0] cpu_addr,
  output        cpu_as_n,
  output        cpu_ds_n,
  output        cpu_rw_n,              // 1 = read
  output  [2:0] cpu_fc,
  output  [1:0] cpu_siz,
  output [31:0] cpu_dout,
  input  [31:0] cpu_din,
  input   [1:0] dsack_n,               // {DSACK1*, DSACK0*}
  input         berr,
  output        cbreq,                 // CBREQ* asserted (active high here)
  input         cback,                 // CBACK* asserted, with a 32-bit termination (BURST only)
  input   [2:0] ipl_n,
  input         cdis,                  // CDIS* asserted (VIA2 PB0 low): the caches disabled
  input   [1:0] coff,                  // {instruction cache off, data cache off}: a test switch
  input         pace_en,               // hold each instruction to the 68030's time; low: unpaced
  input         post_en,               // the write pending buffer; low: every write waited for
  input         xc_en,                 // XCACHE: the card enabled (/CENABLE asserted, a card fitted)
  input         xc_flush,              // XCACHE: /CFLUSH asserted
  input         xc_big,                // XCACHE: 128 KB of data (32 KB when low)
  output [47:0] dbg_xc,                // XCACHE: {hits[23:0], misses[23:0]}, the counts wrap

  output        reset_out_n,           // the RESET instruction
  output        halted,                // double bus fault
  output [31:0] dbg_d6,                // D6 and D7, for debugging: the ROM's test manager
  output [31:0] dbg_d7,                //   keeps a failed test's code there
  output [159:0] dbg_trg,              // {D1, A1, A3, A5, A6}: the bus-error trigger's snapshot
  output [56:0] dbg_exc,               // {an exception taken (one clk), its vector number, the opcode, its PC}
  output [35:0] dbg_pace,              // one instruction released by the pace: {release, decoder row, clocks, budget, fetch waits}
  output [47:0] dbg_wrap,              // the wrapper's own state, live
  output [63:0] dbg_cache,             // {CDIS*, 1'b0, CACR[13:0], instruction hits[23:0], data hits[23:0]}
  output [55:0] dbg_mmuf,              // the PMMU's last fault: {0, BERR pending, MMU BERR pending, insn, read, FC, MMUSR, address}
  output [96:0] dbg_rpr,               // {a format $A replay under way, the frame as the RTE read it}
  output [106:0] dbg_rpk,              // each kernel data write as its cycle starts
  output [160:0] dbg_jtp,              // a trace tap: {a beat taken now, its record}
  output [160:0] dbg_jte               // ... and its PMMU events
);

  // ------------------------------------------------------------- kernel
  wire        k_clkena, k_beat_valid;
  wire        k_clk, k_clkena_in, k_go;
  generate if (KCLK) begin : g_kclk
    assign k_clk = clk_k; assign k_clkena_in = k_go;
  end else begin : g_kclk
    assign k_clk = clk;   assign k_clkena_in = k_clkena;
  end endgenerate
  wire [31:0] k_din, k_dout, k_addr, k_addr_log, k_cacr, k_cache_op_addr;
  wire        k_ci, k_rmc, k_wronly, k_dibsub;
  wire  [1:0] k_dsack, k_busstate, k_siz;
  wire  [1:0] k_state;                 // the kernel's state register (busstate before its PMMU and squash terms)
  wire        k_nwr, k_nreset_out, k_clr_berr, k_cp_berr_ack;
  wire  [2:0] k_fc;
  wire        k_pmmu_busy, k_pmmu_fault, k_make_berr, k_trap_berr, k_halted;
  wire        k_exc_take;
  wire [31:0] k_f_addr;
  wire [15:0] k_f_mmusr;
  wire  [2:0] k_f_fc;
  wire        k_f_rw, k_f_insn, k_trap_mmu_berr;
  wire        k_decode;                // the kernel's decode beat is next (decodeOPC)
  wire        pace_stall;              // ... and the pace withholds it (below)
  wire [31:0] k_trap_vector;
  wire [15:0] k_opcode;
  wire [31:0] k_opcode_pc;
  wire [41:0] k_pflush;                // {PFLUSH request, MODE, MASK, FC, EA}
  wire        w_req, w_we, w_berr;
  wire [31:0] w_addr, w_wdat;
  wire        w_ack;                   // the kernel's: w_ack_s, or under KCLK its clk_k copy
  wire [31:0] w_data;
  reg         w_ack_s;                 // the walker's descriptor is in (set at phi1 edges only)
  reg  [31:0] w_data_s;
  wire        k_berr;
  reg   [2:0] kk_ipl = 3'b111;
  always @(posedge clk_k) kk_ipl <= ipl_n;
  wire  [2:0] k_ipl = KCLK ? kk_ipl : ipl_n;

  TG68KdotC_Kernel #(
    .SR_Read(2), .VBR_Stackframe(2), .extAddr_Mode(2), .MUL_Mode(2), .DIV_Mode(2),
    .BitField(2), .BarrelShifter(2), .MUL_Hardware(1), .DATA_WIDTH(32)
  ) kernel (
    .clk(k_clk), .nReset(reset_n), .clkena_in(k_clkena_in), .beat_valid(k_beat_valid),
    .data_in(k_din), .dsack(k_dsack), .IPL(k_ipl), .IPL_autovector(1'b1), .berr(k_berr), .CPU(2'b10),
    .addr_out(k_addr), .addr_log_out(k_addr_log), .data_write(k_dout), .siz(k_siz), .nWr(k_nwr),
    .CACR_out(k_cacr), .cache_op_addr(k_cache_op_addr), .pmmu_cache_inhibit(k_ci), .rmc_out(k_rmc), .wronly_rd_out(k_wronly), .dib_sub_out(k_dibsub),
    .busstate(k_busstate), .FC(k_fc), .nResetOut(k_nreset_out), .clr_berr(k_clr_berr), .cp_berr_ack(k_cp_berr_ack),
    .pmmu_walker_req(w_req), .pmmu_walker_we(w_we), .pmmu_walker_addr(w_addr), .pmmu_walker_wdat(w_wdat),
    .pmmu_walker_ack(w_ack), .pmmu_walker_data(w_data), .pmmu_walker_berr(w_berr),
    .debug_pmmu_busy(k_pmmu_busy), .debug_pmmu_fault(k_pmmu_fault),
    .debug_make_berr(k_make_berr), .debug_trap_berr(k_trap_berr), .debug_cpu_halted(k_halted),
    .debug_regfile_d6(dbg_d6), .debug_regfile_d7(dbg_d7),
    .debug_regfile_d1(dbg_trg[159:128]), .debug_regfile_a1(dbg_trg[127:96]), .debug_regfile_a3(dbg_trg[95:64]),
    .debug_regfile_a5(dbg_trg[63:32]), .debug_regfile_a6(dbg_trg[31:0]),
    .debug_exc_take(k_exc_take), .debug_trap_vector(k_trap_vector), .debug_opcode(k_opcode), .debug_opcode_pc(k_opcode_pc),
    .debug_decodeOPC(k_decode), .debug_state(k_state),
    .debug_pmmu_fault_addr(k_f_addr), .debug_pmmu_fault_mmusr(k_f_mmusr), .debug_pmmu_fault_fc(k_f_fc),
    .debug_pmmu_fault_rw(k_f_rw), .debug_pmmu_fault_is_insn(k_f_insn), .debug_trap_mmu_berr(k_trap_mmu_berr),
    .debug_rte_fmt_a_replay(dbg_rpr[96]), .debug_rte_fmt_a_state1(dbg_rpr[95:80]), .debug_rte_fmt_a_ssw(dbg_rpr[79:64]),
    .debug_rte_fmt_a_fault_addr(dbg_rpr[63:32]), .debug_rte_fmt_a_data_out(dbg_rpr[31:0]),
    .debug_pflush(k_pflush)
  );
  reg  [52:0] mmuf_q = 53'd0;
  always @(posedge clk) if (k_pmmu_fault) mmuf_q <= {k_f_insn, k_f_rw, k_f_fc, k_f_mmusr, k_f_addr};
  assign dbg_mmuf = {1'b0, k_trap_berr, k_trap_mmu_berr, mmuf_q};
  assign dbg_exc = {k_exc_take, k_trap_vector[9:2], k_opcode, k_opcode_pc};
  assign reset_out_n = k_nreset_out;
  assign halted = k_halted;

  // ----------------------------------------------------- the bus cycle
  reg         walk;                    // a walker cycle is in progress (a long, in beats)
  reg   [2:0] walk_done;               // bytes of the walker's long moved so far
  reg  [31:0] walk_buf;
  wire        k_req  = (k_busstate != 2'b01);
  wire        park   = k_pmmu_busy || k_pmmu_fault || w_req;
  reg         post;                    // a posted operand: from its first beat's S1 to its last beat's S5
  reg  [31:0] pb_addr;                 // the beat's address
  reg  [31:0] pb_dout;                 // ... its data
  reg   [2:0] pb_rem;                  // the operand's bytes from this beat on: the beat's SIZ
  reg   [2:0] pb_fc;
  wire        wsel   = !post && (walk || (w_req && !w_ack_s)); // the bus is the walker's
  wire        eff_req  = post || wsel || (k_req && !park);
  wire [31:0] eff_addr = post ? pb_addr : wsel ? {w_addr[31:2], walk_done[1:0]} : k_addr;
  wire        eff_rw   = post ? 1'b0 : wsel ? !w_we : k_nwr;
  wire  [2:0] eff_fc   = post ? pb_fc : wsel ? 3'd5 : k_fc;
  wire  [1:0] eff_siz  = post ? pb_rem[1:0] :
                         wsel ? (walk_done == 3'd3 ? 2'b01 : walk_done == 3'd2 ? 2'b10 : walk_done == 3'd1 ? 2'b11 : 2'b00) : k_siz;
  wire  [7:0] w_r0 = walk_done == 0 ? w_wdat[31:24] : walk_done == 1 ? w_wdat[23:16] : walk_done == 2 ? w_wdat[15:8] : w_wdat[7:0];
  wire  [7:0] w_r1 = walk_done == 0 ? w_wdat[23:16] : walk_done == 1 ? w_wdat[15:8]  : w_wdat[7:0];
  wire  [7:0] w_r2 = walk_done == 0 ? w_wdat[15:8]  : w_wdat[7:0];
  wire  [7:0] w_r3 = w_wdat[7:0];
  wire [31:0] w_dout = (walk_done == 0) ? {w_r0, w_r1, w_r2, w_r3} :
                       (walk_done == 1) ? {w_r0, w_r0, w_r1, w_r2} :
                       (walk_done == 2) ? {w_r0, w_r1, w_r0, w_r1} : {w_r0, w_r0, w_r1, w_r0};
  wire [31:0] eff_dout = post ? pb_dout : wsel ? w_dout : k_dout;

  wire        cpu_space = (eff_fc == 3'd7);
  wire        iack      = cpu_space && (eff_addr[19:16] == 4'hF);
  wire        cp_fpu    = cpu_space && (eff_addr[19:13] == 7'b0010_001);
  wire        cp_local  = cpu_space && !cp_fpu;                  // answered here
  reg   [2:0] s;                       // 0 idle (S0), 2 after S1, 3 S3/wait, 4 after S4, 5 after S5
  wire        xc_hit, xc_hold, xc_rel; // the cache card: it answers this cycle; it holds AS* back; the retry starts
  reg         as_n_r, ds_n_r;
  reg  [31:0] din_r;
  reg   [1:0] dsack_r;
  reg         ack_pending, ack_berr;
  reg         berr_hold;
  reg         kk_bclr;                 // the kernel released a held bus error at this edge (berr_hold follows at the phi2 edge)
  reg  [31:0] kk_dout;                 // k_dout at the kernel's last edge (a posted write's data, for pb_dout)
  reg         pb_ld;                   // KCLK: the posted write just taken has its data still to come from kk_dout
  wire        kc_end0  = ((s == 3'd3) && (berr || dsack_n != 2'b11)) || (s == 3'd4);
  wire        kc_end   = kc_end0 || xc_hit;                                            // (xc_hit: the cache card, s == 2)
  reg         e_berr;                  // BERR and DSACK* as that edge took them, for S5
  reg   [1:0] e_dsack;
  wire        berr5    = KCLK ? e_berr : berr;
  wire  [1:0] dsack5   = KCLK ? e_dsack : dsack_n;
  wire        pace_stall_b;

  // ------------------------------------------------------------ the caches
  // a fetch or data read that hits is answered from the cache with no bus cycle; cachable cycles fill
  wire        k_fetch  = !wsel && (k_busstate == 2'b00) && !park;
  wire        k_dread  = !wsel && (k_busstate == 2'b10) && !park;
  wire        k_dwrite = !wsel && (k_busstate == 2'b11) && !park;
  wire        i_hit, d_hit;
  wire        i_lnew, d_lnew;          // the line holds nothing of the request (BURST)
  wire [31:0] i_q, d_q;
  localparam  BR = (BURST != 0) && (KCLK != 0);
  reg         cyc_burst;               // this cycle asked for a burst (CBREQ*)
  reg   [1:0] b_n;                     // longs the burst has brought
  reg         b_done;                  // the last beat has ended: AS* rises at the next phi2 edge
  reg         b_adv;                   // a beat ended: the next entry at the next phi2 edge
  reg         e_cback;                 // CBACK* as the first beat's end took it
  wire        b_beat   = BR && (s == 3'd6) && !b_done;                    // a fill beat may end at this phi1 edge
  wire        f_hit    = k_fetch && i_hit;
  wire        r_hit    = k_dread && !k_rmc && d_hit && (k_fc != 3'd7);
  wire        z_hit    = k_dread && k_wronly;
  wire        s_hit    = (k_dread || k_dwrite) && k_dibsub;
  wire        any_hit  = f_hit || r_hit || z_hit || s_hit;
  reg         hit_ack;                 // a hit taken: the kernel is acknowledged at the next phi1
  reg         hit_d;                   // ... from the data cache
  reg         hit_z;                   // ... for a write-only instruction's read (z_hit)
  reg         cyc_fill;                // this cycle's long fills the instruction cache
  reg         cyc_dfill;               // ... the data cache
  reg  [31:2] cyc_la;
  reg   [2:0] cyc_fc;
  wire        fill_ok  = ((KCLK ? kc_end : (phi2 && (s == 3'd5))) || b_beat) && !berr && ((dsack_n == 2'b00) || xc_hit);
  wire        i_fill   = fill_ok && cyc_fill;
  wire        d_fill   = fill_ok && cyc_dfill;
  wire  [2:0] w_size   = (k_siz == 2'b00) ? 3'd4 : {1'b0, k_siz};
  wire  [2:0] w_nb     = (w_size > 3'd4 - k_addr[1:0]) ? 3'd4 - k_addr[1:0] : w_size;
  wire  [3:0] w_mask   = (w_nb == 3'd1) ? 4'b1000 : (w_nb == 3'd2) ? 4'b1100 : (w_nb == 3'd3) ? 4'b1110 : 4'b1111;
  wire  [3:0] d_wbe    = w_mask >> k_addr[1:0];
  wire        s1_take  = phi2 && (s == 3'd0) && !post && !any_hit && !ack_pending && !hit_ack && eff_req && !pace_stall_b;
  wire        pb_take  = phi2 && (s == 3'd0) && post && !ack_pending;
  wire        d_wr     = s1_take && k_dwrite && !k_ci && (k_fc != 3'd7);
  reg         d_wr_q   = 1'b0;
  always @(posedge clk) d_wr_q <= d_wr;
  reg         kk_cdis  = 1'b0;
  always @(posedge clk_k) kk_cdis <= cdis;
  reg  [31:0] kk_din;
  reg         kk_ifill = 1'b0, kk_dfill = 1'b0;
  reg  [31:2] kk_fla;
  reg   [2:0] kk_ffc;
  always @(posedge clk_k) begin kk_ifill <= i_fill; kk_dfill <= d_fill; kk_fla <= cyc_la; kk_ffc <= cyc_fc; end
  wire        d_inv    = phi1 && k_pmmu_fault && !walk && (s == 3'd0) && !ack_pending && !post && !k_nwr && (k_fc != 3'd7);   // k_force's terms (declared below)

  // ------------------------------------------- the write pending buffer
  // a write to RAM or the ROM's overlay is posted: acknowledged at once, run from the buffer while the
  // kernel goes on with internal beats and cache hits; any other cycle waits for it
  wire        pb_vid   = (k_addr[31:24] == 8'hFE);
  wire        k_post   = post_en && k_dwrite && !k_rmc && (k_fc != 3'd7) && ((k_addr[31:30] == 2'b00) || pb_vid);
  assign      dbg_rpk  = {s1_take && k_dwrite, k_addr_log, k_addr, k_dout, k_siz, k_fc, k_post, d_wbe};
  reg         post_ack;                // the kernel's acknowledge for its posted write, at the next phi1
  reg  [31:0] pb_lanes;                // the long as the kernel drove it: each byte on its own address's lane
  reg   [1:0] pb_lane;                 // the beat's lane in it
  reg   [2:0] pb_n;                    // beats left, this one's included
  wire  [1:0] pb_ln    = pb_lane + 2'd1;
  wire  [7:0] pb_nbyte = pb_lanes[31 - 8 * pb_ln -: 8];
  wire        c_clk    = KCLK ? clk_k : clk;
  se30_cache030 #(.ASYNC(KCLK)) cache (
    .clk(c_clk), .reset_n(reset_n),
    .cacr(k_cacr[13:0] & ~{5'b0, coff[0], 7'b0, coff[1]}), .caar(k_cache_op_addr[7:2]), .cdis(KCLK ? kk_cdis : cdis),   // coff: EI/ED cleared
    .la(k_addr_log[31:2]), .fc(k_fc), .i_hit(i_hit), .i_lnew(i_lnew), .i_q(i_q),
    .i_fill(KCLK ? kk_ifill : i_fill), .fill_la(KCLK ? kk_fla : cyc_la), .fill_fc(KCLK ? kk_ffc : cyc_fc), .fill_data(KCLK ? kk_din : cpu_din),
    .d_hit(d_hit), .d_lnew(d_lnew), .d_q(d_q), .d_fill(KCLK ? kk_dfill : d_fill),
    .d_wr(KCLK ? d_wr_q : d_wr), .d_wbe(d_wbe), .d_wdata(k_dout), .d_wlong(k_siz == 2'b00 && k_addr[1:0] == 2'b00),
    .d_inv(d_inv));
  wire [13:0] b_cacr   = k_cacr[13:0] & ~{5'b0, coff[0], 7'b0, coff[1]};
  wire        b_cdis   = KCLK ? kk_cdis : cdis;
  wire  [4:0] b_reach  = {1'b0, k_addr[3:0]} + {2'b0, w_size};            // an operand's end in its 16-byte line
  wire        b_icond  = b_cacr[0] && b_cacr[4] && !b_cacr[1] && !b_cdis && i_lnew;
  wire        b_dcond  = b_cacr[8] && b_cacr[12] && !b_cacr[9] && !b_cdis && d_lnew && !k_rmc && (b_reach <= 5'd16);
  wire        b_req    = BR && !k_ci && (k_fc != 3'd7) && ((k_fetch && b_icond) || (k_dread && b_dcond));

  wire        ecs_s    = reset_n && (s == 3'd0) && !ack_pending &&
                         (post || (eff_req && !hit_ack && !any_hit && !pace_stall_b));
  reg  [31:0] kb_addr, kb_dout;
  reg   [2:0] kb_fc;
  reg   [1:0] kb_siz;
  reg         kb_rw, kb_as_n, kb_ds_n, kb_ecs;
  reg         kb_cbreq;                // BURST: CBREQ*, negated after the third long
  always @(posedge clk_k or negedge reset_n)
    if (!reset_n) begin
      kb_addr <= 0; kb_dout <= 0; kb_fc <= 0; kb_siz <= 0; kb_rw <= 1; kb_as_n <= 1; kb_ds_n <= 1; kb_ecs <= 0; kb_cbreq <= 0;
    end else begin
      kb_as_n <= as_n_r || xc_hold; kb_ds_n <= ds_n_r || xc_hold; kb_ecs <= ((s == 3'd2) && !xc_hold) || xc_rel;
      kb_cbreq <= BR && cyc_burst && (s != 3'd0) && !(s == 3'd6 && b_n == 2'd3);
      if (s == 3'd2) begin kb_addr <= eff_addr; kb_rw <= eff_rw; kb_fc <= eff_fc; kb_siz <= eff_siz; kb_dout <= (post && post_ack) ? k_dout : eff_dout; end   // (a posted write's first beat: pb_dout is loaded at this same edge)
    end
  assign ecs      = KCLK ? (kb_ecs && phi2) : ecs_s;
  assign cpu_addr = KCLK ? kb_addr : eff_addr;
  assign cpu_as_n = KCLK ? kb_as_n : as_n_r;
  assign cpu_ds_n = KCLK ? kb_ds_n : ds_n_r;
  assign cpu_rw_n = KCLK ? kb_rw   : eff_rw;
  assign cpu_fc   = KCLK ? kb_fc   : eff_fc;
  assign cpu_siz  = KCLK ? kb_siz  : eff_siz;
  assign cpu_dout = KCLK ? kb_dout : eff_dout;
  assign cbreq    = BR ? kb_cbreq : 1'b0;

  // ------------------------------------------------------ the cache card
  // the IIci's RAM cache card (Guide 2e ch. 5): 32 or 128 KB direct-mapped, write-through, on RAM and
  // ROM reads; a hit answers in 2 clocks, a miss holds AS* back and retries the cycle from memory
  localparam        XC      = (XCACHE != 0) && (KCLK != 0);
  localparam  [2:0] XC_MISS = 3'd4;
  localparam        XC_WUPD = 0;   // a write to a long the card holds: 0 turns its valid bit off
  wire              xc_ram  = (eff_addr[31:27] == 5'b00000);
  wire              xc_rom  = (eff_addr[31:25] == 7'b0100_000);
  wire       [14:0] xc_ridx = {xc_big ? eff_addr[16:15] : 2'b00, eff_addr[14:2]};
  wire       [12:0] xc_rtag = {xc_rom, eff_addr[26:15]};
  wire              xc_ci   = !post && !wsel && k_ci;
  reg               xc_act;                  // enabled, swept, not being flushed
  reg               xc_relig, xc_welig;      // the request at this clk's start: a read / a write the card looks at
  reg        [12:0] xc_tagr;                 //   ... and its tag
  reg        [13:0] xc_tq;                   // the tag RAM's entry for it
  reg        [31:0] xc_dq;                   // ... and the data RAM's
  wire              xc_match = XC && xc_tq[13] && (xc_tq[12:0] == xc_tagr);
  assign            xc_hit   = xc_match && xc_relig && (s == 3'd2);          // at P1: the card answers
  wire              xc_miss  = XC && !xc_match && xc_relig && (s == 3'd2);   // at P1: the retry
  reg         [2:0] xc_w;                    // CPU clocks AS* is still held back
  assign            xc_hold  = XC && (((s == 3'd2) && xc_relig) || (xc_w > 3'd1));   // AS*, DS* stay high at this P
  assign            xc_rel   = XC && (xc_w == 3'd1);                         // the retried cycle's first clk: ECS
  reg               xc_fillc;                // this cycle's longs go into the card
  reg               xc_upd;                  // this write's long is in the card: its bytes at Q1
  reg        [14:0] xc_sw;                   // the sweep
  reg               xc_swreq, xc_swrun;
  reg        [23:0] xc_nh, xc_nm;
  assign            dbg_xc = {xc_nh, xc_nm};
  generate if (XC) begin : g_xc
    (* ramstyle = "no_rw_check, M10K" *) reg [13:0] xc_tag [0:32767];
    (* ramstyle = "no_rw_check, M10K" *) reg  [7:0] xc_d3 [0:32767];
    (* ramstyle = "no_rw_check, M10K" *) reg  [7:0] xc_d2 [0:32767];
    (* ramstyle = "no_rw_check, M10K" *) reg  [7:0] xc_d1 [0:32767];
    (* ramstyle = "no_rw_check, M10K" *) reg  [7:0] xc_d0 [0:32767];
    wire        xc_fill = phi1 && xc_fillc && (((s == 3'd3) && kc_end0) || b_beat) && !berr && (dsack_n == 2'b00);
    wire [31:2] xc_fa   = {kb_addr[31:4], (s == 3'd6) ? cyc_la[3:2] : kb_addr[3:2]};
    wire  [2:0] xc_sz   = (kb_siz == 2'b00) ? 3'd4 : {1'b0, kb_siz};
    wire  [2:0] xc_a10  = {1'b0, kb_addr[1:0]};
    wire  [3:0] xc_be   = { (xc_a10 == 3'd0), (xc_a10 <= 3'd1) && (xc_a10 + xc_sz > 3'd1),
                            (xc_a10 <= 3'd2) && (xc_a10 + xc_sz > 3'd2), (xc_a10 + xc_sz > 3'd3) };   // UM Table 7-7
    wire        xc_wr   = phi2 && xc_upd;
    wire [31:2] xc_wa   = xc_fill ? xc_fa : kb_addr[31:2];
    wire [14:0] xc_widx = xc_swrun ? xc_sw : {xc_big ? xc_wa[16:15] : 2'b00, xc_wa[14:2]};
    wire [31:0] xc_wd   = (XC_WUPD && !xc_fill) ? kb_dout : cpu_din;                  // (XC_WUPD 0: only a fill writes data)
    wire  [3:0] xc_dwe  = xc_fill ? 4'b1111 : (xc_wr && XC_WUPD) ? xc_be : 4'b0000;
    wire        xc_twe  = xc_swrun || xc_fill || (xc_wr && !XC_WUPD);                 // (a write: the valid bit off)
    wire [13:0] xc_twd  = (xc_swrun || !xc_fill) ? 14'd0 : {1'b1, (xc_wa[31:25] == 7'b0100_000), xc_wa[26:15]};
    reg  [14:0] xc_widx_q;
    reg  [31:0] xc_wd_q;
    reg   [3:0] xc_dwe_q;
    reg         xc_twe_q;
    reg  [13:0] xc_twd_q;
    initial begin xc_widx_q = 0; xc_wd_q = 0; xc_dwe_q = 0; xc_twe_q = 0; xc_twd_q = 0; end
    // simulation: a check that a card lookup never reads an entry a write is landing on
    // synthesis translate_off
    reg         xc_rdw = 0;            // the read at the last edge met a write landing on its entry
    reg         xc_plant = 0;          // +XC_RDW_PLANT: tests the check
    initial     xc_plant = $test$plusargs("XC_RDW_PLANT");
    always @(posedge clk) begin
      xc_rdw <= (xc_twe_q || (xc_dwe_q != 4'b0000)) && (xc_widx_q == xc_ridx);
      if (reset_n && xc_rdw && ((phi1 && (s == 3'd2) && (xc_relig || xc_welig)) || (xc_plant && xc_act)))
        $display("==== XC FAIL: a card lookup read the entry a write was landing on, %0t", $time);
    end
    // synthesis translate_on
    always @(posedge clk) begin
      xc_tq <= xc_tag[xc_ridx];
      xc_dq <= {xc_d3[xc_ridx], xc_d2[xc_ridx], xc_d1[xc_ridx], xc_d0[xc_ridx]};
      if (xc_twe_q) xc_tag[xc_widx_q] <= xc_twd_q;
      if (xc_dwe_q[3]) xc_d3[xc_widx_q] <= xc_wd_q[31:24];
      if (xc_dwe_q[2]) xc_d2[xc_widx_q] <= xc_wd_q[23:16];
      if (xc_dwe_q[1]) xc_d1[xc_widx_q] <= xc_wd_q[15:8];
      if (xc_dwe_q[0]) xc_d0[xc_widx_q] <= xc_wd_q[7:0];
      xc_widx_q <= xc_widx; xc_wd_q <= xc_wd; xc_dwe_q <= xc_dwe; xc_twe_q <= xc_twe; xc_twd_q <= xc_twd;
    end
    always @(posedge clk or negedge reset_n)
      if (!reset_n) begin
        xc_act <= 0; xc_relig <= 0; xc_welig <= 0; xc_tagr <= 0; xc_w <= 0; xc_fillc <= 0; xc_upd <= 0;
        xc_sw <= 0; xc_swreq <= 1; xc_swrun <= 0; xc_nh <= 0; xc_nm <= 0;
      end else begin
        xc_relig <= xc_act &&  eff_rw && (eff_fc != 3'd7) && (xc_ram || xc_rom) && !xc_ci;
        xc_welig <= xc_act && !eff_rw && (eff_fc != 3'd7) && xc_ram;
        xc_tagr  <= xc_rtag;
        if (phi1) begin
          if (xc_w != 0) xc_w <= xc_w - 1'b1;
          if (s == 3'd2) begin
            xc_fillc <= xc_miss;
            xc_upd   <= xc_match && xc_welig;
            if (xc_miss) begin xc_w <= XC_MISS; xc_nm <= xc_nm + 1'b1; end
            if (xc_hit) xc_nh <= xc_nh + 1'b1;
          end
        end
        if (phi2) xc_upd <= 0;
        if (!xc_en || xc_flush) xc_swreq <= 1;
        if (xc_swrun) begin
          xc_sw <= xc_sw + 1'b1;
          if (xc_sw == 15'h7FFF) xc_swrun <= 0;
        end else if (xc_swreq && !xc_act) begin xc_swreq <= 0; xc_swrun <= 1; xc_sw <= 0; end
        if (!xc_en || xc_flush || xc_swreq || xc_swrun) begin if (s == 3'd0 || !xc_en) xc_act <= 0; end
        else if (s == 3'd0 && phi1) xc_act <= 1;
      end
  end else begin : g_noxc
    always @* begin
      xc_act = 0; xc_relig = 0; xc_welig = 0; xc_tagr = 0; xc_tq = 0; xc_dq = 0; xc_w = 0; xc_fillc = 0; xc_upd = 0;
      xc_sw = 0; xc_swreq = 0; xc_swrun = 0; xc_nh = 0; xc_nm = 0;
    end
  end endgenerate

  wire  [2:0] w_room = (dsack_r == 2'b10) ? 3'd1 : (dsack_r == 2'b01) ? 3'd2 - {2'b0, walk_done[0]} : 3'd4 - walk_done;
  wire  [2:0] w_n    = (w_room > 3'd4 - walk_done) ? 3'd4 - walk_done : w_room;
  wire  [1:0] w_first = (dsack_r == 2'b10) ? 2'd0 : (dsack_r == 2'b01) ? {1'b0, walk_done[0]} : walk_done[1:0];
  wire [31:0] w_lane = (w_first == 0) ? din_r : (w_first == 1) ? {din_r[23:0], 8'h0} : (w_first == 2) ? {din_r[15:0], 16'h0} : {din_r[7:0], 24'h0};
  integer k;
  reg  [31:0] w_long;
  always @* begin
    w_long = walk_buf;
    for (k = 0; k < 4; k = k + 1)
      if (k >= walk_done && k < walk_done + w_n)
        w_long[31-8*k -: 8] = w_lane[31-8*(k-walk_done) -: 8];
  end

  always @(posedge clk or negedge reset_n)
    if (!reset_n) begin
      s <= 0; as_n_r <= 1; ds_n_r <= 1; din_r <= 0; dsack_r <= 2'b11;
      ack_pending <= 0; ack_berr <= 0; walk <= 0; walk_done <= 0; walk_buf <= 0; e_berr <= 0; e_dsack <= 2'b11;
      w_ack_s <= 0; w_data_s <= 0; berr_hold <= 0;
      hit_ack <= 0; hit_d <= 0; hit_z <= 0; cyc_fill <= 0; cyc_dfill <= 0; cyc_la <= 0; cyc_fc <= 0;
      cyc_burst <= 0; b_n <= 0; b_done <= 0; b_adv <= 0; e_cback <= 0;
      post <= 0; post_ack <= 0; pb_addr <= 0; pb_dout <= 0; pb_lanes <= 0; pb_lane <= 0; pb_n <= 0; pb_rem <= 0; pb_fc <= 0; pb_ld <= 0;
    end else begin
      if (!KCLK || phi1) w_ack_s <= 0;                                 // KCLK: held to the kernel's edge
      if (phi1) begin
        ack_pending <= 0; ack_berr <= 0; hit_ack <= 0; post_ack <= 0;
        if (ack_pending && walk) begin
          if (ack_berr) begin walk <= 0; walk_done <= 0; w_ack_s <= 1; w_data_s <= w_long; end
          else if (walk_done + w_n >= 3'd4) begin walk <= 0; walk_done <= 0; w_ack_s <= 1; w_data_s <= w_long; end
          else begin walk_done <= walk_done + w_n; walk_buf <= w_long; end
        end
        if (s == 3'd2) s <= 3'd3;
        if (KCLK ? kc_end : (s == 3'd4)) begin s <= 3'd5; e_berr <= berr; e_dsack <= xc_hit ? 2'b00 : dsack_n; e_cback <= cback && !xc_hit; end   // KCLK: the cycle's end, sampled here
        if (b_beat && (berr || dsack_n != 2'b11)) begin
          b_n <= b_n + 2'd1; b_adv <= 1;
          if (berr || dsack_n != 2'b00 || !cback || b_n == 2'd3) b_done <= 1;
        end
        if (!KCLK && (k_make_berr || k_trap_berr || k_cp_berr_ack)) berr_hold <= 0;   // (taken as no coprocessor)
      end
      if (phi2) begin
        if (KCLK && kk_bclr) berr_hold <= 0;
        if (KCLK && pb_ld) begin pb_dout <= kk_dout; pb_lanes <= kk_dout; pb_ld <= 0; end
        if (post && any_hit && !hit_ack && !post_ack && !pace_stall_b) begin
          hit_ack <= 1; hit_d <= !f_hit; hit_z <= z_hit || s_hit; dsack_r <= 2'b00;
        end
        case (s)
          3'd0: if (post) begin                                            // the buffer's next beat: S1
                  as_n_r <= 0; ds_n_r <= 1'b1; s <= 3'd2;
                end else if (any_hit && !ack_pending && !hit_ack && !pace_stall_b) begin   // a hit: no cycle
                  hit_ack <= 1; hit_d <= !f_hit; hit_z <= z_hit || s_hit; dsack_r <= 2'b00;
                end else if (eff_req && !hit_ack && !pace_stall_b) begin     // S1
                  if (w_req && !walk) begin walk <= 1; walk_done <= 0; walk_buf <= 0; end
                  as_n_r <= 0; ds_n_r <= !eff_rw ? 1'b1 : 1'b0; s <= 3'd2;
                  cyc_fill  <= k_fetch && !k_ci && (k_fc != 3'd7);
                  cyc_dfill <= k_dread && !k_ci && (k_fc != 3'd7);
                  cyc_la <= k_addr_log[31:2]; cyc_fc <= k_fc; cyc_burst <= b_req;
                  if (k_post) begin                                          // posted: the buffer takes it from here
                    post <= 1; post_ack <= 1; dsack_r <= 2'b00;
                    pb_addr <= k_addr; pb_lane <= k_addr[1:0];
                    if (!KCLK) begin pb_dout <= k_dout; pb_lanes <= k_dout; end                 // (KCLK: through kk_dout, above)
                    else pb_ld <= 1;
                    pb_n <= pb_vid ? w_nb : 3'd1; pb_rem <= w_size; pb_fc <= k_fc;
                  end
                end
          3'd3: begin                                                      // S3 and the wait states
                  ds_n_r <= 0;
                  if (cp_local) begin                                        // AVEC, or no coprocessor
                    if (iack) begin dsack_r <= 2'b00; s <= 3'd4; end
                    else begin
                      as_n_r <= 1; ds_n_r <= 1; s <= 3'd0; ack_pending <= 1; ack_berr <= 1;
                      if (!walk) berr_hold <= 1;
                    end
                  end else if (KCLK) begin
                  end else if (berr && post) begin
                    as_n_r <= 1; ds_n_r <= 1; s <= 3'd0; post <= 0; berr_hold <= 1;
                  end else if (berr) begin
                    as_n_r <= 1; ds_n_r <= 1; s <= 3'd0; ack_pending <= 1; ack_berr <= 1;
                    dsack_r <= dsack_n; if (!walk) berr_hold <= 1;
                  end else if (dsack_n != 2'b11) s <= 3'd4;
                end
          3'd5: if (post) begin                                            // S5 of a posted beat: no acknowledge
                  as_n_r <= 1; ds_n_r <= 1; s <= 3'd0;
                  if (berr5 || pb_n == 3'd1) begin post <= 0; if (berr5) berr_hold <= 1; end
                  else begin                                                 // the next byte, at the next address
                    pb_n <= pb_n - 3'd1; pb_rem <= pb_rem - 3'd1; pb_addr <= pb_addr + 32'd1;
                    pb_lane <= pb_ln; pb_dout <= {4{pb_nbyte}};
                  end
                end else if (KCLK && e_berr) begin                         // KCLK: a bus error ended it - no data
                  as_n_r <= 1; ds_n_r <= 1; s <= 3'd0; ack_pending <= 1; ack_berr <= 1;
                  dsack_r <= e_dsack; if (!walk) berr_hold <= 1;
                end else begin                                             // S5: latch at the end of S4
                  din_r <= KCLK ? kk_din : cpu_din; if (!cp_local) dsack_r <= dsack5;
                  ack_pending <= 1;
                  if (BR && cyc_burst && e_cback && e_dsack == 2'b00 && !berr5) begin   // BURST: on to the fill beats
                    s <= 3'd6; b_n <= 2'd1; b_done <= 0; b_adv <= 0; cyc_la[3:2] <= cyc_la[3:2] + 2'd1;
                  end else begin as_n_r <= 1; ds_n_r <= 1; s <= 3'd0; end
                  if (berr5) begin ack_berr <= 1; if (!walk) berr_hold <= 1; end
                end
          3'd6: if (b_done) begin                                          // BURST: the last beat has ended
                  as_n_r <= 1; ds_n_r <= 1; s <= 3'd0; b_done <= 0; b_adv <= 0; cyc_burst <= 0;
                end else if (b_adv) begin                                    // the next entry, modulo 16
                  b_adv <= 0; cyc_la[3:2] <= cyc_la[3:2] + 2'd1;
                end
          default: ;
        endcase
      end
    end

  wire k_cycle_ack = (ack_pending || hit_ack || post_ack) && !walk;
  wire k_internal  = (k_busstate == 2'b01) && !k_pmmu_busy && !w_req && !walk && !pace_stall;
  wire k_force     = k_pmmu_fault && !walk && (s == 3'd0) && !ack_pending && !post;

  reg         kk_s0, kk_post, kk_walk, kk_wack, kk_cyc, kk_bh, kk_wberr, kk_fs;
  reg  [31:0] kk_wdata;
  reg   [1:0] kk_dsack;
  reg         kk_brst, kk_bi, kk_bd;   // BURST: a burst runs, into the instruction / data cache ...
  reg   [3:0] kk_bline;                //   ... at this line
  always @(posedge clk_k or negedge reset_n)
    if (!reset_n) begin kk_brst <= 0; kk_bi <= 0; kk_bd <= 0; kk_bline <= 0; end
    else begin kk_brst <= BR && (s == 3'd6); kk_bi <= cyc_fill; kk_bd <= cyc_dfill; kk_bline <= cyc_la[7:4]; end
  reg         kk_pgo_q, kk_pgo_p;      // the pace's p_go for the decisions at Q, and at the next P (the pace, below)
  wire        kc_s4    = kc_end && !post;                                              // acknowledged at Q
  wire        kc_cpn   = (s == 3'd2) && cp_local && !iack;                             // no coprocessor: BERR at S3
  wire        kc_wdone = ack_pending && walk && (ack_berr || walk_done + w_n >= 3'd4);   // the walker's long, at P
  always @(posedge clk_k or negedge reset_n)
    if (!reset_n) begin
      kk_s0 <= 1; kk_post <= 0; kk_walk <= 0; kk_wack <= 0; kk_cyc <= 0; kk_bh <= 0; kk_wberr <= 0; kk_fs <= 0; kk_bclr <= 0; kk_dout <= 0;
      kk_din <= 0; kk_wdata <= 0; kk_dsack <= 2'b11;
    end else begin
      kk_s0    <= (s == 3'd0);
      kk_post  <= post;
      kk_walk  <= walk && !kc_wdone;
      kk_wack  <= kc_wdone;
      if (kc_wdone) kk_wdata <= w_long;
      kk_cyc   <= (kc_s4 || kc_cpn) && !walk;
      if ((kc_s4 || b_beat) && !berr) kk_din <= xc_hit ? xc_dq : cpu_din;
      kk_dsack <= kc_s4 ? (cp_local ? dsack_r : xc_hit ? 2'b00 : dsack_n) : (s == 3'd2 && cp_local && iack) ? 2'b00 : dsack_r;
      kk_bh    <= (berr_hold && !(k_make_berr || k_trap_berr || k_cp_berr_ack)) ||
                  (kc_s4 && berr && !walk) || (kc_end0 && post && berr) || (kc_cpn && !walk);
      kk_wberr <= walk && kc_s4 && berr;
      kk_bclr  <= k_make_berr || k_trap_berr || k_cp_berr_ack;
      kk_dout  <= k_dout;
      kk_fs    <= !(walk && !kc_wdone) && (((s == 3'd0) && !post) || (kc_end0 && post && (berr || pb_n == 3'd1)));
    end
  wire        kc_wsel   = !kk_post && (kk_walk || (w_req && !kk_wack));
  wire        kc_fetch  = !kc_wsel && (k_busstate == 2'b00) && !park;
  wire        kc_dread  = !kc_wsel && (k_busstate == 2'b10) && !park;
  wire        kc_dwrite = !kc_wsel && (k_busstate == 2'b11) && !park;
  wire        kc_fhit   = kc_fetch && i_hit;
  wire        kc_zs     = (kc_dread && k_wronly) || ((kc_dread || kc_dwrite) && k_dibsub);   // z_hit, s_hit
  wire        kc_anyhit = kc_fhit || (kc_dread && !k_rmc && d_hit && (k_fc != 3'd7)) || kc_zs;
  wire        kc_stallq = k_decode && !kk_pgo_q;
  wire        kc_stallp = k_decode && !kk_pgo_p;
  wire        kc_pkok   = post_en && kc_dwrite && !k_rmc && (k_fc != 3'd7) && ((k_addr[31:30] == 2'b00) || pb_vid);
  wire        kc_bblk   = kk_brst && (k_addr_log[7:4] == kk_bline) && ((kk_bi && kc_fetch) || (kk_bd && kc_dread && !kc_zs));
  wire        kc_hit    = kc_anyhit && !kc_stallp && (kk_post || kk_s0 || kk_brst) && !kc_bblk;
  wire        kc_take   = kk_s0 && !kk_post && !kc_anyhit && (kc_wsel || (k_req && !park)) && !kc_stallq && !(kc_pkok && kc_stallp);
  wire        kc_pack   = kc_take && kc_pkok;
  wire        kc_ack    = kk_cyc || kc_hit || kc_pack;
  wire        kc_int    = (k_busstate == 2'b01) && !k_pmmu_busy && !w_req && !kk_walk && !(k_decode && !kk_pgo_p);
  wire        kc_force  = k_pmmu_fault && kk_fs && !(kc_take && kc_wsel);

  generate if (KCLK) begin : g_kin
    assign k_go         = kc_ack || kc_int || kc_force;
    assign k_beat_valid = kc_ack || kc_int;
    wire        kd_z    = k_state[1] && (k_wronly || k_dibsub);
    assign k_din        = kk_cyc ? kk_din : kd_z ? 32'h0 : k_state[1] ? d_q : i_q;
    assign k_dsack      = kk_cyc ? kk_dsack : 2'b00;
    assign k_berr       = kk_bh && !(k_make_berr || k_trap_berr);
    assign w_ack = kk_wack; assign w_data = kk_wdata; assign w_berr = kk_wberr;
  end else begin : g_kin
    assign k_go         = k_cycle_ack || k_internal || k_force;
    assign k_beat_valid = k_cycle_ack || k_internal;
    assign k_din        = !hit_ack ? din_r : hit_z ? 32'h0 : hit_d ? d_q : i_q;
    assign k_dsack      = dsack_r;
    assign k_berr       = berr_hold && !(k_make_berr || k_trap_berr);
    assign w_ack = w_ack_s; assign w_data = w_data_s; assign w_berr = ack_berr && walk;
  end endgenerate
  assign k_clkena     = phi1 && k_go;
  assign dbg_jtp = {k_clkena && (k_busstate != 2'b01),
                    k_busstate, k_fc, k_siz, k_addr_log, k_addr, (k_busstate == 2'b11) ? k_dout : k_din,
                    k_beat_valid, k_force, hit_ack, hit_d, hit_z, post_ack, k_pmmu_fault, ack_berr,
                    k_decode, k_opcode, k_opcode_pc};
  reg         jt_pf_q = 1'b0;
  always @(posedge clk) jt_pf_q <= k_pflush[41];
  wire        jt_pf   = k_pflush[41] && !jt_pf_q;
  assign dbg_jte = jt_pf ? {1'b1, 2'b01, 3'd2, 2'b00, k_pflush[31:0], k_addr_log, 32'h0,
                            k_pflush[40:38], k_pflush[37:35], k_pflush[34:32], k_opcode, k_opcode_pc} :
                           {w_ack, 2'b01, {2'b00, w_we}, 2'b00, w_addr, k_addr_log, w_we ? w_wdat : w_data,
                            k_fc, 5'b0, 1'b0, k_opcode, k_opcode_pc};

  // ------------------------------------------------------------ the pace
  // each instruction held at its decode beat to its MC68030 UM Section 11 time (se30_pace030.v), the
  // bus cycles' wait states added; only the step into the next instruction waits
  reg  [15:0] p_op;      // the instruction running: its opcode ...
  reg  [31:0] p_pc;      //   ... and address, latched at its release
  reg   [5:0] p_tail;    // the tail of the instruction before it, its write cycles' wait states included (saturating)
  reg   [5:0] p_tail_q;  // KCLK: the tail this instruction leaves, a phi2 edge ahead (the pace's kk_ registers, below)
  reg   [8:0] p_cnt;     // C16M clocks since the release (saturating)
  reg   [7:0] p_rcred;   // the wait-state clocks of its data read cycles so far (saturating) ...
  reg   [7:0] p_wcred;   //   ... of its write cycles, MOVEM's register clocks with them ...
  reg   [7:0] p_fcred;   //   ... and of its instruction fetches (the no-cache case's W: budget only, no tail)
  reg   [7:0] p_xcred;   //   ... and of the instruction before's posted write, completed after the release
  reg         pb_old;    // the posted operand's instruction has been released
  reg         p_rel;     // released: the decode beat may step
  reg   [6:0] p_cyc;     // clk edges of the cycle in progress (saturating at 127: a cycle's credit is 63 clocks
  reg         p_cont;    // the cycle in progress continues an operand through a narrow port
  reg  [29:0] p_last_la; // the last data cycle's long address ...
  reg   [1:0] p_last_ds; //   ... and port size
  reg         p_last_d;  //   ... and that it was a data cycle
  wire        d_ea_on, d_ea_ophead;
  wire  [4:0] d_ea_h, d_op_h;
  wire  [1:0] d_ea_t, d_op_t, d_br, d_mvm;
  wire  [7:0] d_row;
  wire  [5:0] d_ea_cc;
  wire  [6:0] d_op_cc, d_op_cc_t;
  wire [15:0] p_op_n;
  se30_pace030 pace (
    .op(KCLK ? p_op_n : p_op), .ea_on(d_ea_on), .ea_ophead(d_ea_ophead), .ea_h(d_ea_h), .ea_t(d_ea_t), .ea_cc(d_ea_cc),
    .op_h(d_op_h), .op_t(d_op_t), .op_cc(d_op_cc), .op_cc_t(d_op_cc_t), .br(d_br), .mvm(d_mvm), .row(d_row));
  reg         r_ea_on;
  reg   [5:0] r_ea_h;    // the EA's head, the op's included where the manual says "n + op head"
  reg   [1:0] r_ea_t, r_op_t, r_br, r_mvm;
  reg   [4:0] r_op_h;
  reg   [5:0] r_ea_cc;
  reg   [6:0] r_op_cc, r_op_cc_t;
  reg         r_taken;   // a branch: the next instruction is not the one that follows
  reg   [7:0] r_row;     // the decoder's row, for the profile (dbg_pace)
  reg   [1:0] r_mvm_q;   // KCLK: r_mvm as it stood before the last phi1 edge
  wire  [2:0] p_ilen  = (d_br == 2'd2) ? 3'd4 : (p_op[7:0] == 8'h00) ? 3'd4 : (p_op[7:0] == 8'hFF) ? 3'd6 : 3'd2;
  wire        r_clk   = KCLK ? clk_k : clk;
  always @(posedge r_clk) begin
    r_ea_on  <= d_ea_on && (d_ea_cc != 6'd0);
    r_ea_h   <= {1'b0, d_ea_h} + (d_ea_ophead ? {1'b0, d_op_h} : 6'd0);
    r_ea_t   <= d_ea_t; r_ea_cc <= d_ea_cc;
    r_op_h   <= d_op_h; r_op_t <= d_op_t; r_op_cc <= d_op_cc; r_op_cc_t <= d_op_cc_t;
    r_br     <= d_br; r_mvm <= d_mvm; r_row <= d_row; r_mvm_q <= r_mvm;
    r_taken  <= (d_br != 2'd0) && (k_opcode_pc != p_pc + {29'd0, p_ilen});
  end
  reg  [31:0] kk_pcl;
  reg         kk_br;
  wire        kc_taken = kk_br && (k_opcode_pc != kk_pcl);
  wire        p_taken  = KCLK ? kc_taken : r_taken;
  function [9:0] f_budget(input ea_on, input [5:0] ea_h, input [1:0] ea_t, input [5:0] ea_cc, input [4:0] op_h,
                          input [6:0] op_clk, input [5:0] tail, input [7:0] rc, input [7:0] wc, input [7:0] fc, input [7:0] xc);
    reg [5:0] ov_ea, ov_ea_c; reg [8:0] ea_tl, t_mid, ov_op, ov_op_c; reg [9:0] credit;
    begin
      ov_ea   = (ea_h < tail) ? ea_h : tail;                                             // min(head, tail before)
      ea_tl   = {7'd0, ea_t} + {1'b0, rc};                                               // the EA part's tail, its reads' wait states in
      t_mid   = ea_on ? ea_tl : {3'd0, tail};
      ov_op   = ({4'd0, op_h} < t_mid) ? {4'd0, op_h} : t_mid;
      credit  = {2'd0, rc} + {2'd0, wc} + {2'd0, fc} + {2'd0, xc};
      ov_ea_c = (ov_ea > ea_cc) ? ea_cc : ov_ea;
      ov_op_c = (ov_op > {2'd0, op_clk}) ? {2'd0, op_clk} : ov_op;
      f_budget = (ea_on ? {4'd0, ea_cc} - {4'd0, ov_ea_c} : 10'd0) + ({3'd0, op_clk} - {1'b0, ov_op_c}) + credit;
    end
  endfunction
  wire  [6:0] p_op_clk = p_taken ? r_op_cc_t : r_op_cc;
  wire  [9:0] p_budget = f_budget(r_ea_on, r_ea_h, r_ea_t, r_ea_cc, r_op_h, p_op_clk, p_tail, p_rcred, p_wcred, p_fcred, p_xcred);
  wire  [9:0] p_tail_n = {8'd0, r_op_t} + {2'd0, p_wcred} + (r_ea_on ? 10'd0 : {2'd0, p_rcred});
  wire  [5:0] p_tail_s = (p_tail_n > 10'd63) ? 6'd63 : p_tail_n[5:0];
  wire        p_due    = ({1'b0, p_cnt} >= p_budget) || (p_cnt == 9'h1FF);
  wire        p_go     = p_rel || p_due || !pace_en;
  assign      pace_stall = k_decode && !p_go;
  reg         kk_pgo_pt, kk_pgo_pn;
  wire        kc_pgo_p = kc_taken ? kk_pgo_pt : kk_pgo_pn;
  always @* kk_pgo_p = kc_pgo_p;
  wire        p_go_rel = KCLK ? kk_pgo_p : p_go;                          // the release's p_go
  assign      pace_stall_b = KCLK ? (k_decode && !kk_pgo_q) : pace_stall;
  reg  [15:0] kk_op;                   // p_op, on clk_k
  reg         kk_rel;                  // p_rel, on clk_k (it moves at phi1 edges only)
  wire        pk_rel   = k_decode && !kk_rel && p_go_rel;                 // a release at this P
  assign      p_op_n   = pk_rel ? k_opcode : kk_op;
  wire [31:0] pk_pc_n  = pk_rel ? k_opcode_pc : p_pc;
  wire  [2:0] pk_ilen  = (d_br == 2'd2) ? 3'd4 : (p_op_n[7:0] == 8'h00) ? 3'd4 : (p_op_n[7:0] == 8'hFF) ? 3'd6 : 3'd2;
  wire        pk_rel_n = pk_rel ? !k_go : (k_go && k_decode) ? 1'b0 : kk_rel;
  wire  [8:0] pk_cnt_n = pk_rel ? 9'd1 : (p_cnt != 9'h1FF) ? p_cnt + 9'd1 : p_cnt;
  wire        pk_sat   = !pk_rel && (p_cnt >= 9'h1FE);                    // pk_cnt_n at its ceiling: due (p_due, above)
  wire  [5:0] pk_tail  = pk_rel ? p_tail_q : p_tail;
  wire  [7:0] pk_rc    = pk_rel ? 8'd0 : p_rcred;
  wire  [7:0] pk_wc    = pk_rel ? 8'd0 : p_wcred;
  wire  [7:0] pk_fc    = pk_rel ? 8'd0 : p_fcred;
  wire  [7:0] pk_xc    = pk_rel ? 8'd0 : p_xcred;
  wire        pk_old   = pk_rel ? (post || pb_old) : pb_old;
  reg   [5:0] px_addw  = 6'd0;         // p_addw for a cycle whose S5 is two clk edges on
  wire  [7:0] px_edges = {1'b0, p_cyc} + 8'd3;                                     // (p_cyc + 2) + 1, at most 130
  wire  [6:0] px_len   = px_edges[7:1] + 7'd1;
  wire  [6:0] px_a     = p_cont ? px_len : px_len - 7'd2;
  always @(posedge clk) px_addw <= (px_a > 7'd63) ? 6'd63 : px_a[5:0];
  wire        px_s5    = kc_end0 && post;
  wire  [5:0] px_add   = px_addw + {4'd0, (cyc_fc != 3'd6) ? r_mvm : 2'd0};
  wire  [8:0] px_wsum  = {1'b0, pk_wc} + {3'd0, px_add};
  wire  [8:0] px_xsum  = {1'b0, pk_xc} + {3'd0, px_addw};
  wire        px_w     = px_s5 && !pk_old;
  wire        px_x     = px_s5 && pk_old;
  wire  [7:0] pq_wc    = !px_w ? pk_wc : (px_wsum < 9'd255) ? px_wsum[7:0] : 8'hFF;
  wire  [7:0] pq_xc    = !px_x ? pk_xc : (px_xsum < 9'd255) ? px_xsum[7:0] : 8'hFF;
  wire  [6:0] px_tsum  = {1'b0, pk_tail} + {1'b0, px_addw};
  wire  [5:0] pq_tail  = !px_x ? pk_tail : (px_tsum > 7'd63) ? 6'd63 : px_tsum[5:0];
  wire        dn_ea_on = d_ea_on && (d_ea_cc != 6'd0);
  wire  [5:0] dn_ea_h  = {1'b0, d_ea_h} + (d_ea_ophead ? {1'b0, d_op_h} : 6'd0);
  wire  [9:0] pk_bq    = f_budget(r_ea_on, r_ea_h, r_ea_t, r_ea_cc, r_op_h, p_op_clk, pk_tail, pk_rc, pk_wc, pk_fc, pk_xc);
  wire  [9:0] pk_bpt   = f_budget(dn_ea_on, dn_ea_h, d_ea_t, d_ea_cc, d_op_h, d_op_cc_t, pq_tail, pk_rc, pq_wc, pk_fc, pq_xc);
  wire  [9:0] pk_bpn   = f_budget(dn_ea_on, dn_ea_h, d_ea_t, d_ea_cc, d_op_h, d_op_cc,   pq_tail, pk_rc, pq_wc, pk_fc, pq_xc);
  always @(posedge clk_k or negedge reset_n)
    if (!reset_n) begin
      kk_pgo_q <= 1'b1; kk_pgo_pt <= 1'b1; kk_pgo_pn <= 1'b1;            // p_cnt resets to 1FF: due
      kk_op <= 16'h0; kk_pcl <= 32'd2; kk_br <= 1'b0; kk_rel <= 1'b0;
    end else begin
      kk_pgo_q  <= pk_rel_n || ({1'b0, pk_cnt_n} >= pk_bq)  || pk_sat || !pace_en;
      kk_pgo_pt <= pk_rel_n || ({1'b0, pk_cnt_n} >= pk_bpt) || pk_sat || !pace_en;
      kk_pgo_pn <= pk_rel_n || ({1'b0, pk_cnt_n} >= pk_bpn) || pk_sat || !pace_en;
      kk_op <= p_op_n; kk_pcl <= pk_pc_n + {29'd0, pk_ilen}; kk_br <= (d_br != 2'd0); kk_rel <= pk_rel_n;
    end
  wire        p_release = phi1 && k_decode && !p_rel && p_go_rel;
  assign      dbg_pace  = {p_release, r_row, p_cnt, p_budget, p_fcred};
  wire  [7:0] p_edges  = {1'b0, p_cyc} + 8'd1;
  wire  [6:0] p_len    = p_edges[7:1] + 7'd1;                                           // C16M clocks
  wire  [6:0] p_a      = p_cont ? p_len : p_len - 7'd2;
  wire  [5:0] p_addw   = (p_a > 7'd63) ? 6'd63 : p_a[5:0];
  wire  [5:0] p_add    = p_addw + {4'd0, (cyc_fc != 3'd6) ? (KCLK ? r_mvm_q : r_mvm) : 2'd0};   // KCLK: r_mvm as it stands at this phi2 edge
  wire  [6:0] p_tail_x = {1'b0, p_tail} + {1'b0, p_addw};
  wire        cr_s5    = (s == 3'd5) && !(KCLK && e_berr && !post) && !(post && pb_old) && (cyc_fc != 3'd6);
  wire  [8:0] cr_wsum  = {1'b0, p_wcred} + {3'd0, p_add};
  wire  [8:0] cr_rsum  = {1'b0, p_rcred} + {3'd0, p_add};
  wire        cr_rw    = KCLK ? kb_rw : eff_rw;   // KCLK: this cycle's direction, held since S2
  wire  [7:0] cr_wc    = !(cr_s5 && !cr_rw) ? p_wcred : (cr_wsum < 9'd255) ? cr_wsum[7:0] : 8'hFF;
  wire  [7:0] cr_rc    = !(cr_s5 &&  cr_rw) ? p_rcred : (cr_rsum < 9'd255) ? cr_rsum[7:0] : 8'hFF;
  wire  [9:0] cr_tail  = {8'd0, r_op_t} + {2'd0, cr_wc} + (r_ea_on ? 10'd0 : {2'd0, cr_rc});
  always @(posedge clk or negedge reset_n)
    if (!reset_n) p_tail_q <= 6'd0;
    else if (phi2) p_tail_q <= (cr_tail > 10'd63) ? 6'd63 : cr_tail[5:0];
  always @(posedge clk or negedge reset_n)
    if (!reset_n) begin
      p_op <= 16'h0; p_pc <= 32'h0; p_tail <= 6'd0; p_cnt <= 9'h1FF; p_rcred <= 8'd0; p_wcred <= 8'd0; p_fcred <= 8'd0; p_rel <= 1'b0;
      p_xcred <= 8'd0; pb_old <= 1'b0;
      p_cyc <= 7'd0; p_cont <= 1'b0; p_last_la <= 30'd0; p_last_ds <= 2'b00; p_last_d <= 1'b0;
    end else begin
      if (s != 3'd0 && p_cyc != 7'h7F) p_cyc <= p_cyc + 7'd1;
      if (phi1) begin
        if (p_cnt != 9'h1FF) p_cnt <= p_cnt + 9'd1;
        if (k_decode && !p_rel && p_go_rel) begin                         // the release
          p_op <= k_opcode; p_pc <= k_opcode_pc; p_tail <= (p_tail_n > 10'd63) ? 6'd63 : p_tail_n[5:0];
          p_cnt <= 9'd1; p_rcred <= 8'd0; p_wcred <= 8'd0; p_fcred <= 8'd0;     // this clock is the instruction's first
          p_xcred <= 8'd0; if (post) pb_old <= 1'b1;                       // a write still posted is the instruction before's
          p_rel <= !k_clkena;                                              // an internal beat steps now
        end else if (k_clkena && k_decode) p_rel <= 1'b0;                  // the decode beat taken
      end
      if (phi2) begin
        if (s1_take) begin
          p_cyc <= 7'd0;
          p_cont <= !wsel && (k_fc != 3'd6) && p_last_d && (p_last_ds != 2'b00) && (k_addr[31:2] == p_last_la);
          if (k_post) pb_old <= 1'b0;
        end
        if (pb_take) begin p_cyc <= 7'd0; p_cont <= 1'b1; end              // the buffer's later beats continue its operand
        if (s == 3'd5 && !(KCLK && e_berr && !post)) begin                // (KCLK: a bus-errored cycle gives no credit)
          if (post && pb_old) begin                                         // a posted beat after its instruction's release
            if (p_xcred + {2'd0, p_addw} < 9'd255) p_xcred <= p_xcred + {2'd0, p_addw}; else p_xcred <= 8'hFF;
            p_tail <= (p_tail_x > 7'd63) ? 6'd63 : p_tail_x[5:0];
          end else if (cyc_fc == 3'd6) begin                                // a fetch
            if (p_fcred + {2'd0, p_add} < 9'd255) p_fcred <= p_fcred + {2'd0, p_add}; else p_fcred <= 8'hFF;
          end else if (!eff_rw) begin                                        // a write: the operation's
            if (p_wcred + {2'd0, p_add} < 9'd255) p_wcred <= p_wcred + {2'd0, p_add}; else p_wcred <= 8'hFF;
          end else begin                                                     // a data read: the EA's
            if (p_rcred + {2'd0, p_add} < 9'd255) p_rcred <= p_rcred + {2'd0, p_add}; else p_rcred <= 8'hFF;
          end
          p_last_d <= (cyc_fc != 3'd6); p_last_ds <= dsack5; p_last_la <= cyc_la;
        end
      end
    end

  reg [23:0] n_ihit, n_dhit;
  always @(posedge clk or negedge reset_n)
    if (!reset_n) begin n_ihit <= 0; n_dhit <= 0; end
    else if (phi1 && hit_ack && !hit_z) begin
      if (hit_d) n_dhit <= n_dhit + 1'd1; else n_ihit <= n_ihit + 1'd1;
    end
  assign dbg_cache = {cdis, 1'b0, k_cacr[13:0], n_ihit, n_dhit};
  assign dbg_wrap = {6'd0, s, post, park, walk, w_req, w_ack_s, ack_pending, b_done, b_adv, cyc_burst, b_n,
                     kk_s0, kk_post, kk_brst, kk_walk, kk_wack, kk_cyc, kk_bh, kk_fs, kk_pgo_q, kk_pgo_p,
                     k_decode, k_busstate, k_state, k_pmmu_busy, k_pmmu_fault,
                     kc_take, kc_anyhit, kc_bblk, kc_stallq, kc_wsel, kc_int, kc_force, berr_hold, pb_n};

endmodule

// pokey_cx: Atari POKEY (C012294), cycle-exact against MAME 0.289 pokey.cpp.
//
// One `ce` = one POKEY clock (MAME step_one_clock).  A write (`wr`) is applied
// BEFORE the step on the same edge, so "write after step k" (MAME sync_write)
// and write-then-step on one edge give the same result.  Everything MAME does
// "instantly" (STIMER, SKCTL init, POTGO) completes inside that one edge.
// State representation = MAME's: up-counting channel counters reloaded with
// AUDF^FF, borrow_cnt, filter_sample, poly registers holding the LFSR value
// (table[index]); `dbg_state` exports it for the lockstep rig.
//
// Scope (trace histogram, plan M2): arcade use.  Not implemented: keyboard scan
// (KBCODE reads 09, MAME with no keyboard callback), serial I/O (SEROUT/SERIN
// shifting, SKCTL bits 3/5/6: two-tone and serial clock modes, SOD/OCLK pins).
// SKCTL bit 4 (async) is implemented for its audio effect (CH3/CH4 hold/reset).
//
// Parameters (per chip, from the MAME machine config):
//   POT_CB    pots with a pot_r callback (warlords 0F); value = pot_val, sampled at POTGO
//   ALLPOT_CB chip has an allpot_r callback (milliped/mazeinv/multiped, ccastles pokey2):
//             ALLPOT reads return allpot_ext and an `rd` of ALLPOT stores it (pokey.cpp:944)
//   POT_EXT   pots measured by an external comparator instead (old board wiring, pokey_glue):
//             pot_in[n] = (pot_count_step > pot value n).  The latch then happens at the same
//             step as MAME's (POTx < counter); POTn holds the counter value at the latch
//             (= the pot value).  Differences to MAME: value 0 latches at the first count
//             instead of at POTGO, and POTx reads 228 until latched (not observable: POTn
//             reads return the counter until the ALLPOT bit is set).  Exclusive with POT_CB.
`default_nettype none
module pokey_cx #(
    parameter [7:0] POT_CB    = 8'h00,
    parameter [0:0] ALLPOT_CB = 1'b0,
    parameter [7:0] POT_EXT   = 8'h00
)(
    input  wire        clk,
    input  wire        reset_n,      // synchronous; MAME device_start + device_reset state
    input  wire        ce,           // one POKEY clock
    input  wire [3:0]  addr,
    input  wire [7:0]  din,
    input  wire        wr,           // any clk; applied before a step on the same edge
    input  wire        rd,           // read strobe (only ALLPOT with ALLPOT_CB has a side effect)
    output reg  [7:0]  dout,         // combinational from the current state
    input  wire [63:0] pot_val,      // pot n = pot_val[8n+7:8n], sampled at POTGO / reset
    input  wire [7:0]  allpot_ext,   // allpot_r callback value
    output wire        irq_n,
    output wire [15:0] out_raw,      // MAME m_out_raw {ch4,ch3,ch2,ch1} volume nibbles
    output wire [3:0]  channel_0_out,
    output wire [3:0]  channel_1_out,
    output wire [3:0]  channel_2_out,
    output wire [3:0]  channel_3_out,
    output wire [7:0]  pot_count,
    input  wire [7:0]  pot_in,       // POT_EXT comparator results (see above)
    output reg  [7:0]  pot_count_step // pot counter after a step on this edge (incl. this edge's write)
`ifdef POKEY_DBG
  , output wire [341:0] dbg_state
`endif
);

// ------------------------------------------------------------------ constants
localparam [3:0]  P4_INIT  = 4'h1;       // LFSR value at index 0 (verilator/lfsr_test)
localparam [4:0]  P5_INIT  = 5'h01;
localparam [8:0]  P9_INIT  = 9'h0FF;
localparam [16:0] P17_INIT = 17'h1FF7F;
localparam [7:0]  POT_MAX  = 8'd228;

// ------------------------------------------------------------------ state
// small register arrays (parallel access): keep them out of RAM inference
(* ramstyle = "logic" *) reg  [7:0]  audf [0:3];
(* ramstyle = "logic" *) reg  [7:0]  audc [0:3];
(* ramstyle = "logic" *) reg  [7:0]  cnt  [0:3];
(* ramstyle = "logic" *) reg  [2:0]  bcnt [0:3];
reg  [3:0]  outp, fsmp;
reg  [7:0]  audctl, skctl, irqen, irqst, allpot;
reg  [4:0]  clk28;
reg  [6:0]  clk114;
reg  [3:0]  p4;
reg  [4:0]  p5;
reg  [8:0]  p9;
reg  [16:0] p17;
reg  [7:0]  pot_cnt;
(* ramstyle = "logic" *) reg  [7:0]  potx [0:7];
reg  [15:0] oraw;
reg         irq_lvl;
wire [7:0]  skstat = 8'h00;              // only serial/keyboard/SID set SKSTAT bits

// ------------------------------------------------------------------ helpers
// inc_chan: count up; arm the borrow when the counter wraps with no borrow pending.
// returns {arm, borrow_cnt, counter}
function automatic [11:0] f_inc(input [7:0] c, input [2:0] b, input [2:0] cyc);
    reg [7:0] cn;
    begin
        cn = c + 8'd1;
        if (cn == 8'd0 && b == 3'd0) f_inc = {1'b1, cyc, cn};
        else                         f_inc = {1'b0, b, cn};
    end
endfunction

function automatic [8:0] f_pot_clamp(input [7:0] v);   // {is_zero, clamped}
    begin
        f_pot_clamp = {(v == 8'd0), (v >= POT_MAX) ? POT_MAX : v};
    end
endfunction

// channel output on a borrow (MAME process_channel), polys = this step's values
// c = AUDC[7:5] {NOTPOLY5, POLY4, PURE}
function automatic f_proc(input o, input [2:0] c, input p4b, input p5b, input p9b, input p17b, input poly9);
    begin
        if (c[2] || p5b) begin
            if (c[0])      f_proc = ~o;
            else if (c[1]) f_proc = p4b;
            else if (poly9) f_proc = p9b;
            else           f_proc = p17b;
        end else
            f_proc = o;
    end
endfunction

// ------------------------------------------------------------------ next state
reg  [7:0]  n_audf [0:3];
reg  [7:0]  n_audc [0:3];
reg  [7:0]  n_cnt  [0:3];
reg  [2:0]  n_bcnt [0:3];
reg  [3:0]  n_outp, n_fsmp;
reg  [7:0]  n_audctl, n_skctl, n_irqen, n_irqst, n_allpot;
reg  [4:0]  n_clk28;
reg  [6:0]  n_clk114;
reg  [3:0]  n_p4;
reg  [4:0]  n_p5;
reg  [8:0]  n_p9;
reg  [16:0] n_p17;
reg  [7:0]  n_pot_cnt;
reg  [7:0]  n_potx [0:7];
reg  [15:0] n_oraw;
reg         n_irq;

reg  [11:0] t;
reg  [8:0]  pc;
reg  [7:0]  upd;
reg  [2:0]  arm;          // inc_chan armed a borrow this step: {ch4, ch2, ch1} = IRQ bits 2,1,0
reg         run, t28, t114, tb, async_rst, j12, j34, fire, poly9;
integer     i;

always @* begin
    for (i = 0; i < 4; i = i + 1) begin
        n_audf[i] = audf[i]; n_audc[i] = audc[i]; n_cnt[i] = cnt[i]; n_bcnt[i] = bcnt[i];
    end
    for (i = 0; i < 8; i = i + 1) n_potx[i] = potx[i];
    n_outp = outp; n_fsmp = fsmp;
    n_audctl = audctl; n_skctl = skctl; n_irqen = irqen; n_irqst = irqst; n_allpot = allpot;
    n_clk28 = clk28; n_clk114 = clk114;
    n_p4 = p4; n_p5 = p5; n_p9 = p9; n_p17 = p17;
    n_pot_cnt = pot_cnt; n_oraw = oraw; n_irq = irq_lvl;
    t = 12'd0; pc = 9'd0; upd = 8'd0; arm = 3'd0;
    run = 1'b0; t28 = 1'b0; t114 = 1'b0; tb = 1'b0; async_rst = 1'b0; j12 = 1'b0; j34 = 1'b0; fire = 1'b0; poly9 = 1'b0;

    // ------------------------------------------------ bus write (pokey.cpp write_internal)
    if (wr) begin
        case (addr)
            4'h0: n_audf[0] = din;
            4'h1: n_audc[0] = din;
            4'h2: n_audf[1] = din;
            4'h3: n_audc[1] = din;
            4'h4: n_audf[2] = din;
            4'h5: n_audc[2] = din;
            4'h6: n_audf[3] = din;
            4'h7: n_audc[3] = din;
            4'h8: n_audctl = din;
            4'h9: begin                                   // STIMER: all channels reload, outputs low
                for (i = 0; i < 4; i = i + 1) begin
                    n_cnt[i] = ~n_audf[i]; n_bcnt[i] = 3'd0;
                end
                n_outp = 4'b0000; n_fsmp = 4'b0011;
            end
            4'hB: if (skctl[1:0] != 2'b00) begin          // POTGO (ignored in init)
                n_allpot = 8'h00; n_pot_cnt = 8'd0;
                for (i = 0; i < 8; i = i + 1) begin
                    n_potx[i] = POT_MAX;
                    if (POT_CB[i]) begin
                        pc = f_pot_clamp(pot_val[8*i +: 8]);
                        n_potx[i] = pc[7:0];
                        if (pc[8]) n_allpot[i] = 1'b1;
                    end
                end
            end
            4'hE: begin                                   // IRQEN: ack masked bits (SEROC kept)
                n_irqst = n_irqst & (din | 8'h08);
                n_irqen = din;
                if ((din & n_irqst & 8'h08) != 8'h00) n_irq = 1'b1;
                else if ((din & n_irqst) == 8'h00)    n_irq = 1'b0;
            end
            4'hF: if (din != skctl) begin                 // SKCTL (same value: no effect)
                n_skctl = din;
                if (din[1:0] == 2'b00) begin              // init: IRQEN=0, polys/prescalers to 0
                    n_irqen = 8'h00; n_irqst = 8'h08;     // IRQEN(0) keeps SEROC; serout_complete sets it
                    n_irq = 1'b0;
                    n_p4 = P4_INIT; n_p5 = P5_INIT; n_p9 = P9_INIT; n_p17 = P17_INIT;
                    n_clk28 = 5'd0; n_clk114 = 7'd0;
                end
                if (din[4]) begin                         // async receive: CH3/CH4 reload
                    n_cnt[2] = ~n_audf[2]; n_bcnt[2] = 3'd0;
                    n_cnt[3] = ~n_audf[3]; n_bcnt[3] = 3'd0;
                end
            end
            default: ;                                    // SKREST (SKSTAT is 0), POTGO, SEROUT, 0C
        endcase
    end

    // ------------------------------------------------ ALLPOT read with callback stores the value
    if (rd && ALLPOT_CB && addr == 4'h8 && n_skctl[1:0] != 2'b00)
        n_allpot = allpot_ext;

    // ------------------------------------------------ one POKEY clock (pokey.cpp step_one_clock order)
    if (ce) begin
        run   = n_skctl[1:0] != 2'b00;
        j12   = n_audctl[4];
        j34   = n_audctl[3];
        poly9 = n_audctl[7];
        async_rst = n_skctl[4];                           // && !(SKSTAT & (BUSY|SERIN)), SKSTAT = 0
        if (run) begin
            // 1. polys
            n_p4  = {n_p4[2:0], ~(n_p4[2] ^ n_p4[3])};
            n_p5  = {n_p5[3:0], ~(n_p5[2] ^ n_p5[4])};
            n_p9  = {n_p9[0] ^ n_p9[5], n_p9[8:1]};
            n_p17 = {n_p17[0], n_p17[16:9], n_p17[8] ^ n_p17[13], n_p17[7:1]};
            // 2. prescalers
            if (n_clk28 == 5'd27) begin n_clk28 = 5'd0; t28 = 1'b1; end
            else n_clk28 = n_clk28 + 5'd1;
            if (n_clk114 == 7'd113) begin n_clk114 = 7'd0; t114 = 1'b1; end
            else n_clk114 = n_clk114 + 7'd1;
            tb = n_audctl[0] ? t114 : t28;
            // 3. CH1
            if (n_audctl[6]) begin
                t = f_inc(n_cnt[0], n_bcnt[0], j12 ? 3'd7 : 3'd4);
                n_cnt[0] = t[7:0]; n_bcnt[0] = t[10:8]; arm[0] = t[11];
            end else if (tb) begin
                t = f_inc(n_cnt[0], n_bcnt[0], 3'd1);
                n_cnt[0] = t[7:0]; n_bcnt[0] = t[10:8]; arm[0] = t[11];
            end
            // 5. CH3 (held while async reset)
            if (!async_rst) begin
                if (n_audctl[5]) begin
                    t = f_inc(n_cnt[2], n_bcnt[2], j34 ? 3'd7 : 3'd4);
                    n_cnt[2] = t[7:0]; n_bcnt[2] = t[10:8];   // CH3 has no IRQ
                end else if (tb) begin
                    t = f_inc(n_cnt[2], n_bcnt[2], 3'd1);
                    n_cnt[2] = t[7:0]; n_bcnt[2] = t[10:8];   // CH3 has no IRQ
                end
            end
            // 6. CH2 / CH4 on the base clock when not joined
            if (tb && !j12) begin
                t = f_inc(n_cnt[1], n_bcnt[1], 3'd1);
                n_cnt[1] = t[7:0]; n_bcnt[1] = t[10:8]; arm[1] = t[11];
            end
            if (tb && !j34 && !async_rst) begin
                t = f_inc(n_cnt[3], n_bcnt[3], 3'd1);
                n_cnt[3] = t[7:0]; n_bcnt[3] = t[10:8]; arm[2] = t[11];
            end
            // 7. pot scan
            if ((t114 || n_skctl[2]) && n_pot_cnt < POT_MAX) begin
                n_pot_cnt = n_pot_cnt + 8'd1;
                for (i = 0; i < 8; i = i + 1)
                    if (POT_EXT[i]) begin
                        upd[i] = pot_in[i] || (n_pot_cnt == POT_MAX);
                        if (upd[i] && !n_allpot[i]) n_potx[i] = pot_in[i] ? n_pot_cnt - 8'd1 : POT_MAX;
                    end else
                        upd[i] = (n_potx[i] < n_pot_cnt) || (n_pot_cnt == POT_MAX);
            end
        end

        // 9. CH3 borrow (runs in init mode too)
        fire = (n_bcnt[2] == 3'd1);
        if (n_bcnt[2] != 3'd0) n_bcnt[2] = n_bcnt[2] - 3'd1;
        if (fire) begin
            if (j34) begin
                t = f_inc(n_cnt[3], n_bcnt[3], 3'd1);
                n_cnt[3] = t[7:0]; n_bcnt[3] = t[10:8]; arm[2] = arm[2] | t[11];
            end else begin
                n_cnt[2] = ~n_audf[2]; n_bcnt[2] = 3'd0;
            end
            n_outp[2] = f_proc(n_outp[2], n_audc[2][7:5], n_p4[0], n_p5[0], n_p9[0], n_p17[0], poly9);
            n_fsmp[0] = n_audctl[2] ? n_outp[0] : 1'b1;
        end
        // 10. CH4 borrow
        fire = (n_bcnt[3] == 3'd1);
        if (n_bcnt[3] != 3'd0) n_bcnt[3] = n_bcnt[3] - 3'd1;
        if (fire) begin
            if (j34) begin n_cnt[2] = ~n_audf[2]; n_bcnt[2] = 3'd0; end
            n_cnt[3] = ~n_audf[3]; n_bcnt[3] = 3'd0;
            n_outp[3] = f_proc(n_outp[3], n_audc[3][7:5], n_p4[0], n_p5[0], n_p9[0], n_p17[0], poly9);
            n_fsmp[1] = n_audctl[1] ? n_outp[1] : 1'b1;
        end
        // 11. two-tone: not implemented (SKCTL bit 3)
        // 12. CH1 borrow
        fire = (n_bcnt[0] == 3'd1);
        if (n_bcnt[0] != 3'd0) n_bcnt[0] = n_bcnt[0] - 3'd1;
        if (fire) begin
            if (j12) begin
                t = f_inc(n_cnt[1], n_bcnt[1], 3'd1);
                n_cnt[1] = t[7:0]; n_bcnt[1] = t[10:8]; arm[1] = arm[1] | t[11];
            end else begin
                n_cnt[0] = ~n_audf[0]; n_bcnt[0] = 3'd0;
            end
            n_outp[0] = f_proc(n_outp[0], n_audc[0][7:5], n_p4[0], n_p5[0], n_p9[0], n_p17[0], poly9);
        end
        // 13. CH2 borrow
        fire = (n_bcnt[1] == 3'd1);
        if (n_bcnt[1] != 3'd0) n_bcnt[1] = n_bcnt[1] - 3'd1;
        if (fire) begin
            if (j12) begin n_cnt[0] = ~n_audf[0]; n_bcnt[0] = 3'd0; end
            n_cnt[1] = ~n_audf[1]; n_bcnt[1] = 3'd0;
            n_outp[1] = f_proc(n_outp[1], n_audc[1][7:5], n_p4[0], n_p5[0], n_p9[0], n_p17[0], poly9);
        end
        // 14. audio output
        for (i = 0; i < 4; i = i + 1)
            n_oraw[4*i +: 4] = ((n_outp[i] ^ n_fsmp[i]) || n_audc[i][4]) ? n_audc[i][3:0] : 4'd0;
        // deferred (MAME zero-delay timers, applied after the step): pot bits, timer IRQs
        n_allpot = n_allpot | upd;
        if ((arm & n_irqen[2:0]) != 3'd0) begin
            n_irqst[2:0] = n_irqst[2:0] | (arm & n_irqen[2:0]);
            n_irq = 1'b1;
        end
    end
end

// ------------------------------------------------------------------ pot counter if this edge steps
// (POT_EXT comparators).  From the registers + this edge's write only, so the external
// comparator -> pot_in path has no combinational dependence on the step logic.
reg  [7:0] ps_cnt;
reg  [2:0] ps_sk;
always @* begin
    ps_sk  = (wr && addr == 4'hF) ? din[2:0] : skctl[2:0];
    ps_cnt = (wr && addr == 4'hB && skctl[1:0] != 2'b00) ? 8'd0 : pot_cnt;
    pot_count_step = ps_cnt;
    if (ps_sk[1:0] != 2'b00 && (clk114 == 7'd113 || ps_sk[2]) && ps_cnt < POT_MAX)
        pot_count_step = ps_cnt + 8'd1;
end

// ------------------------------------------------------------------ registers
always @(posedge clk) begin
    if (!reset_n) begin
        for (i = 0; i < 4; i = i + 1) begin
            audf[i] <= 8'h00; audc[i] <= 8'hB0; cnt[i] <= 8'h00; bcnt[i] <= 3'd0;
        end
        outp <= 4'b0000; fsmp <= 4'b0000;
        audctl <= 8'h00; skctl <= 8'h00; irqen <= 8'h00; irqst <= 8'h08;
        clk28 <= 5'd0; clk114 <= 7'd0;
        p4 <= P4_INIT; p5 <= P5_INIT; p9 <= P9_INIT; p17 <= P17_INIT;
        oraw <= 16'h0000; irq_lvl <= 1'b0;
        // device_reset -> pokey_potgo (pokey.cpp:380)
        pot_cnt <= 8'd0;
        for (i = 0; i < 8; i = i + 1) begin
            if (POT_CB[i]) begin
                potx[i]   <= (pot_val[8*i +: 8] >= POT_MAX) ? POT_MAX : pot_val[8*i +: 8];
                allpot[i] <= (pot_val[8*i +: 8] == 8'd0);
            end else begin
                potx[i]   <= POT_MAX;
                allpot[i] <= 1'b0;
            end
        end
    end else begin
        for (i = 0; i < 4; i = i + 1) begin
            audf[i] <= n_audf[i]; audc[i] <= n_audc[i]; cnt[i] <= n_cnt[i]; bcnt[i] <= n_bcnt[i];
        end
        for (i = 0; i < 8; i = i + 1) potx[i] <= n_potx[i];
        outp <= n_outp; fsmp <= n_fsmp;
        audctl <= n_audctl; skctl <= n_skctl; irqen <= n_irqen; irqst <= n_irqst; allpot <= n_allpot;
        clk28 <= n_clk28; clk114 <= n_clk114;
        p4 <= n_p4; p5 <= n_p5; p9 <= n_p9; p17 <= n_p17;
        pot_cnt <= n_pot_cnt; oraw <= n_oraw; irq_lvl <= n_irq;
    end
end

// ------------------------------------------------------------------ read port (pokey.cpp read)
always @* begin
    case (addr)
        4'h0, 4'h1, 4'h2, 4'h3, 4'h4, 4'h5, 4'h6, 4'h7:
            dout = allpot[addr[2:0]] ? potx[addr[2:0]] : pot_cnt;
        4'h8: dout = (skctl[1:0] == 2'b00) ? allpot : (ALLPOT_CB ? allpot_ext : ~allpot);
        4'h9: dout = 8'h09;                                   // KBCODE, no keyboard
        4'hA: dout = audctl[7] ? p9[7:0] : p17[15:8];         // RANDOM
        4'hD: dout = 8'h00;                                   // SERIN (never received)
        4'hE: dout = ~irqst;
        4'hF: dout = ~skstat;
        default: dout = 8'hFF;
    endcase
end

assign irq_n = ~irq_lvl;
assign out_raw = oraw;
assign channel_0_out = oraw[3:0];
assign channel_1_out = oraw[7:4];
assign channel_2_out = oraw[11:8];
assign channel_3_out = oraw[15:12];
assign pot_count = pot_cnt;

`ifdef POKEY_DBG
// serial-group fields are constants (not implemented), matching MAME's values only
// while SKCTL bits 3/5/6 stay 0 and SEROUT is unused; the rig does not compare them
wire [7:0] serin = 8'h00, serout = 8'h00;
wire [9:0] serin_shift = 10'd1, serout_shift = 10'd1;
wire serout_full = 1'b0, ser_iclk = 1'b1, ser_oclk = 1'b0, sod_twotone = 1'b0, sod_lvl = 1'b1, oclk_lvl = 1'b0;
`include "pokey_cx_dbg.svh"
`endif

endmodule
`default_nettype wire

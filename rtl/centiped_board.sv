//============================================================================
//
//  Atari Centipede hardware (MAME centiped.cpp boards) for MiSTer
//  Copyright (C) 2026 Rodimus
//
//  Timing and video pipeline derived from "Atari Centipede, from the schematics"
//  by Brad Parker (lisper) and the MiSTer port by Alan Steremberg; variants from MAME.
//
//  Variants (MRA index 1 byte 0):
//    0 centiped    Atari Centipede: POKEY, trackball
//    1 centipedj   same board, no trackball handler (joystick only)
//    2 centipdb    bootleg: AY-3-8910 (1000 data / 1001 address), 4000 mirror, extra ROM at 6000
//    3 caterplr    bootleg: AY-3-8910 registers at 1000-100F, 1780 reads random
//    4 magworm     bootleg: AY-3-8912 (1001 address / 1003 data), RAM cleared at reset
//    5 milliped    Atari Millipede: 2 POKEYs, palette RAM, DIP switches on POKEY ALLPOT
//    6 multiped    Braze Multipede kit: Millipede board, 64K banked scrambled ROM, 93C46
//    7 warlords    Atari Warlords: color PROM, quadrant colors, 8x8 sprites, 4 paddles on POKEY pots
//    8 mazeinv     Atari Maze Invaders: Millipede board, palette through a PROM, 4 analog stick channels
//    9 bullsdrt    Bulls Eye Darts: S2650 daughterboard, SN76496, per-column tile banks
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//
//============================================================================

// ioctl index 0 layout: CPU ROM 0x00000 (64K), gfx low plane 0x10000 (8K), gfx high plane 0x12000 (8K),
// sync PROM 0x14000 (256), color PROM 0x14100 (256)

module centiped_board
(
    input               clk,            // 48.384 MHz = 4 x 12.096 MHz
    input               reset,
    input               pause,
    input         [7:0] variant,

    input         [7:0] in0, in1, in2, in3, dsw1, dsw2,
    input         [7:0] tb1_x, tb1_y,   // trackball counters, player 1 / 2 (cocktail)
    input         [7:0] tb2_x, tb2_y,
    input         [3:0] tb_sign,        // direction of the last count: {P2 Y, P2 X, P1 Y, P1 X}, 1 = down
    input         [7:0] an0, an1, an2, an3,   // Warlords paddles P1-P4 / Maze Invaders STICK0-3

    input        [24:0] ioctl_addr,
    input         [7:0] ioctl_dout,
    input               ioctl_wr0,

    input               crt_flip,
    input  signed [3:0] h_adj,          // CRT position: HSYNC moved 2 pixels per step
    input  signed [3:0] v_adj,          //               VSYNC moved 1 line per step

    output              ce_pix,
    output reg    [7:0] video_r = 8'd0,
    output reg    [7:0] video_g = 8'd0,
    output reg    [7:0] video_b = 8'd0,
    output reg          video_hs = 1'b0,
    output reg          video_vs = 1'b0,
    output reg          video_hblank = 1'b1,
    output reg          video_vblank = 1'b1,

    output signed [15:0] audio,

    // save chip (EAROM 64 bytes / Multipede 93C46 128 bytes) for the NVRAM file, while the CPU is held or paused
    input               nv_sel,
    input         [7:0] nv_addr,
    input         [7:0] nv_din,
    input               nv_we,
    output        [7:0] nv_dout,

    // work RAM second port for hiscore.dat restore / save (Magic Worm, Warlords)
    input         [9:0] hs_addr,
    input         [7:0] hs_din,
    input               hs_we,
    output        [7:0] hs_dout
);

// ---------------------------------------------------------------- variant

wire map_c   = variant <= 8'd4;                     // Centipede memory map
wire v_tb    = variant != 8'd1;                     // trackball handler on IN0 / IN2 (IN0 / IN1 on Millipede)
wire v_cdb   = variant == 8'd2;
wire v_cat   = variant == 8'd3;
wire v_mag   = variant == 8'd4;
wire v_ay    = v_cdb | v_cat | v_mag;
wire v_multi = variant == 8'd6;
wire v_war   = variant == 8'd7;
wire v_maze  = variant == 8'd8;
wire v_bull  = variant == 8'd9;
wire map_m   = variant == 8'd5 || v_multi || v_maze; // Millipede memory map
wire v_mtile = map_m;                                // Millipede tiles: bank + color in D6/D7, flipped by the latch
wire v_mpal  = map_m;                                // palette RAM (Maze Invaders through its PROM)

// ---------------------------------------------------------------- timebase

// 8 clocks per pixel (6.048 MHz); H counts 0x080-0x1FF (256H high = active), V from the sync PROM
reg  [2:0] ph = 3'd0;
always @(posedge clk) ph <= ph + 3'd1;
assign ce_pix = (ph == 3'd7);

reg  [8:0] hcnt = 9'h080;
reg  [7:0] vcnt = 8'd0;
reg  [3:0] vprom_reg = 4'd0;
wire [7:0] vprom_q;
wire       vsync_p = vprom_reg[0];
wire       vreset  = vprom_reg[2];
wire       vblank  = vprom_reg[3];
wire       line_end = ce_pix && hcnt == 9'h1FF;
wire [7:0] vcnt_next = vreset ? 8'd0 : vcnt + 8'd1;

always @(posedge clk) begin
    if (ce_pix) hcnt <= (hcnt == 9'h1FF) ? 9'h080 : hcnt + 9'd1;
    if (line_end) begin
        vprom_reg <= vprom_q[3:0];
        vcnt      <= vcnt_next;
    end
end

dpram_dc #(.widthad_a(8)) sync_prom
(
    .clock_a(clk), .address_a(ioctl_addr[7:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:8] == 17'h00140), .q_a(),
    .clock_b(clk), .address_b({vblank, vcnt[7:5], vcnt[3:0]}), .q_b(vprom_q)
);

// ---------------------------------------------------------------- reset, watchdog

reg  [7:0] rst_cnt = 8'd0;
reg        wd_fire = 1'b0;
wire       sys_reset = reset | wd_fire;
always @(posedge clk) begin
    if (sys_reset)                  rst_cnt <= 8'd0;
    else if (ce_pix && !rst_cnt[4]) rst_cnt <= rst_cnt + 8'd1;
end
wire cpu_reset_n = rst_cnt[4];                      // the CPU starts 16 pixel clocks after reset

// ---------------------------------------------------------------- 6502

wire [23:0] cpu_a24;
wire  [7:0] cpu_dout;
reg   [7:0] cpu_din;
wire        cpu_rw_n;
reg         irq_n = 1'b1;

// phi0: 1.512 MHz, stretched while the CPU addresses the playfield RAM in the video half of the char (4H low)
reg  phi0a_temp = 1'b0, phi0a = 1'b0;
wire pf_access;
wire h1_rise = ce_pix && !hcnt[0];                  // 1H goes high
wire h1_fall = ce_pix &&  hcnt[0];                  // 1H goes low
always @(posedge clk) begin
    if (h1_rise)
        case ({~pf_access | hcnt[2], hcnt[1]})
            2'b00: phi0a_temp <= phi0a_temp;
            2'b01: phi0a_temp <= 1'b0;
            2'b10: phi0a_temp <= 1'b1;
            2'b11: phi0a_temp <= ~phi0a_temp;
        endcase
    if (h1_fall) phi0a <= phi0a_temp;
end
wire phi0_rise = h1_fall && phi0a && !phi0a_temp;   // one CPU cycle ends
wire cpu_ce    = phi0_rise && !pause && !v_bull;

T65 cpu
(
    .Mode(2'b00),
    .Res_n(cpu_reset_n & ~v_bull),
    .Enable(cpu_ce),
    .Clk(clk),
    .Rdy(1'b1),
    .Abort_n(1'b1),
    .IRQ_n(irq_n),
    .NMI_n(1'b1),
    .SO_n(1'b1),
    .R_W_n(cpu_rw_n),
    .A(cpu_a24),
    .DI(cpu_din),
    .DO(cpu_dout)
);

// ---------------------------------------------------------------- S2650 (Bulls Eye Darts)

// one bus access per 12 pixels = 504 kHz (2650 at 1.512 MHz, 3 clocks per access); read data one clock after ack
wire        s26_req, s26_wr, s26_mio, s26_ene, s26_dc, s26_intack;
wire [14:0] s26_ad;
wire  [7:0] s26_dw;
wire  [1:0] s26_ph;
wire  [7:0] s26_dr;
reg   [1:0] s26_div = 2'd0;
reg  [14:0] s26_lastcode = 15'd0;
always @(posedge clk) if (ce_pix && hcnt[1:0] == 2'b00) s26_div <= (s26_div == 2'd2) ? 2'd0 : s26_div + 2'd1;
wire s26_ack = v_bull & ce_pix & ~pause & hcnt[1:0] == 2'b00 & s26_div == 2'd0;
wire s26_acc = s26_ack & s26_req;
always @(posedge clk) if (s26_acc && s26_mio && s26_ph == 2'b00) s26_lastcode <= s26_ad;

s2650_cpu s2650
(
    .clk(clk),
    .reset(~cpu_reset_n | ~v_bull),
    .req(s26_req),
    .ack(s26_ack),
    .ad(s26_ad),
    .wr(s26_wr),
    .dw(s26_dw),
    .dr(s26_dr),
    .mio(s26_mio),
    .ene(s26_ene),
    .dc(s26_dc),
    .ph(s26_ph),
    .irq(1'b0),
    .intack(s26_intack),
    .ivec(8'h00),
    .sense(1'b0),
    .flag()
);

// ---------------------------------------------------------------- common bus

wire [15:0] bus_a    = v_bull ? {1'b0, s26_ad} : cpu_a24[15:0];
wire  [7:0] bus_dout = v_bull ? s26_dw : cpu_dout;
wire        bus_mem  = v_bull ? s26_mio : 1'b1;
wire        bus_we   = v_bull ? s26_acc & s26_wr & s26_mio : cpu_ce & ~cpu_rw_n;
wire        bus_rd   = v_bull ? ~s26_wr : cpu_rw_n;

// ---------------------------------------------------------------- address decode

// Centipede (mask 3FFF): 0000 RAM, 0400 playfield, 0800 DSW, 0C00 inputs, 1000 sound, 1400 colors / EAROM,
// 1800 IRQ ack, 1C00 latch, 2000 watchdog, 2000-3FFF ROM
// Warlords  (mask 7FFF): same low map without colors / EAROM; 4000 watchdog, 5000-7FFF ROM
wire [3:0] c_blk = bus_a[13:10];
wire       w_lo  = v_war & ~bus_a[14];
// Millipede (mask 7FFF): 0000 RAM, 0400 POKEY 1, 0800 POKEY 2, 1000 playfield, 2000 inputs,
// 2480 palette, 2500 latch, 2580 Maze Invaders stick select, 2600 IRQ ack, 2680 watchdog, 2700 / 2780 EAROM,
// 4000-7FFF ROM (Maze Invaders 3000-7FFF)
wire [4:0] m_blk = bus_a[14:10];
// Bulls Eye Darts (15-bit, 1xxx mirrored at 3xxx / 5xxx / 7xxx): 1000 DSW1, 1080 inputs, 1200 EAROM, 1280 EAROM
// control, 1300 DSW2, 1400 colors, 1480 latch, 1500 watchdog, 1800 playfield, 1C00 RAM; ROM elsewhere
wire       b_io  = v_bull & bus_a[12];
wire [4:0] b_blk = bus_a[11:7];

reg        prg_bank = 1'b0;                         // Multipede: 0 Millipede, 1 Centipede
reg        gfx_hi = 1'b0;
wire [1:0] gfx_bank = v_multi ? {gfx_hi, prg_bank} : 2'b00;

// Multipede: 0400-07FF is a window on 1400 (bank 0) or 1000 (bank 1); 2000-3FFF reads A000 (bank 0) or ROM
wire mp_win   = v_multi && bus_a[15:10] == 6'b000001;
wire mp_io2   = v_multi && !prg_bank && bus_a[15:13] == 3'b001;
wire mp_ioA   = v_multi && bus_a[15:13] == 3'b101;
wire mp_io    = (mp_io2 | mp_ioA) && bus_a[12:6] == 7'd0;

wire m_in_grp = v_multi ? mp_io : map_m && m_blk == 5'd8;            // Millipede input group (2000-203F)
wire [2:0] m_in_sel = {bus_a[5:4], bus_a[0]};

wire sel_ram    = bus_mem & (map_c ? c_blk == 4'd0 : w_lo ? c_blk == 4'd0 :
                             v_multi ? bus_a[15:10] == 6'd0 : map_m ? m_blk == 5'd0 :
                             b_io && bus_a[11:10] == 2'b11);
wire sel_pf     = bus_mem & (map_c ? c_blk == 4'd1 : w_lo ? c_blk == 4'd1 :
                             v_multi ? (bus_a[15:10] == 6'b000100 || (mp_win && prg_bank)) :
                             map_m ? m_blk == 5'd4 : b_io && bus_a[11:10] == 2'b10);
wire sel_pokey1 = v_multi ? (bus_a[15:4] == 12'h140 || (mp_win && !prg_bank && bus_a[9:4] == 6'd0)) :
                  map_m   ? m_blk == 5'd1 : (map_c && !v_ay && c_blk == 4'd4) || (w_lo && c_blk == 4'd4);
wire sel_pokey2 = v_multi ? bus_a[15:4] == 12'h080 : map_m && m_blk == 5'd2;
wire sel_dsw    = ((map_c | w_lo) && c_blk == 4'd2) || (b_io && (b_blk == 5'd0 || b_blk == 5'd6));
wire dsw_hi     = v_bull ? b_blk == 5'd6 : bus_a[0];
wire sel_in     = m_in_grp ? (m_in_sel[2] == 1'b0) : ((map_c | w_lo) && c_blk == 4'd3) || (b_io && b_blk == 5'd1);
wire [1:0] in_idx = (map_m) ? m_in_sel[1:0] : bus_a[1:0];
wire sel_stick  = v_maze && m_in_grp && m_in_sel == 3'b100;         // 2020
wire sel_ear    = (map_m && !v_multi && m_in_grp && m_in_sel == 3'b110) ||
                  (map_c && c_blk == 4'd5 && bus_a[9] && bus_a[8:7] == 2'b10) || (b_io && b_blk == 5'd4);
wire sel_rand   = v_cat && c_blk == 4'd5 && bus_a[9] && bus_a[8:7] == 2'b11;
wire sel_ay     = v_ay && c_blk == 4'd4;
wire sel_eep    = v_multi && bus_a[15:10] == 6'b110110;             // D800-DBFF
wire sel_rom    = v_multi ? 1'b1 : v_maze ? (bus_a[14] || bus_a[13:12] == 2'b11) : map_m ? bus_a[14] :
                  v_war ? bus_a[14] : v_bull ? !bus_a[12] : bus_a[13];

// writes (Millipede group at 2400-27FF, mirrored at A400 on Multipede)
wire m_wr_grp  = map_m && bus_a[14:10] == 5'd9;
wire wr_color  = (map_c && c_blk == 4'd5 && !bus_a[9]) || (b_io && b_blk == 5'd8);
wire wr_pal    = m_wr_grp && bus_a[9:7] == 3'b001;
wire wr_latch  = ((map_c | w_lo) && c_blk == 4'd7) || (m_wr_grp && bus_a[9:7] == 3'b010) || (b_io && b_blk == 5'd9);
wire wr_ssel   = v_maze && m_wr_grp && bus_a[9:7] == 3'b011;        // 2580-2583
wire wr_irqack = ((map_c | w_lo) && c_blk == 4'd6) || (m_wr_grp && bus_a[9:7] == 3'b100);
wire wr_wdog   = (map_c && c_blk == 4'd8) || (v_war && bus_a[14]) || (m_wr_grp && bus_a[9:7] == 3'b101);
wire wr_eactl  = (map_c && c_blk == 4'd5 && bus_a[9] && bus_a[8:7] == 2'b01) ||
                 (map_m && !v_multi && m_wr_grp && bus_a[9:7] == 3'b110) || (b_io && b_blk == 5'd5);
wire wr_eaadr  = (map_c && c_blk == 4'd5 && bus_a[9] && bus_a[8:7] == 2'b00) ||
                 (map_m && !v_multi && m_wr_grp && bus_a[9:7] == 3'b111) || (b_io && b_blk == 5'd4);
wire wr_eepw   = v_multi && bus_a[15:11] == 5'b11010;               // D000-D7FF
wire wr_prgb   = sel_eep;
wire wr_gfxb   = v_multi && bus_a[15:10] == 6'b110111;              // DC00-DFFF

assign pf_access = sel_pf & ~v_bull;

// ---------------------------------------------------------------- ROM / RAM

// Multipede: MAME init_multiped descramble, applied on read: CPU address j -> file address, data bits swapped
wire [15:0] mp_j = {prg_bank, (mp_win && !prg_bank) ? {5'b00101, bus_a[9:0]} : bus_a[14:0]};
wire [15:0] mp_i = {mp_j[15:13], mp_j[12] ^ ~mp_j[8], mp_j[11], mp_j[10] ^ ~mp_j[13], mp_j[9:0]};
wire [15:0] mp_src = {mp_i[15], mp_i[14], mp_i[13], mp_i[1], mp_i[8], mp_i[11], mp_i[4], mp_i[7],
                      mp_i[10], mp_i[5], mp_i[6], mp_i[9], mp_i[12], mp_i[0], mp_i[3], mp_i[2]};

wire [15:0] rom_addr = v_multi ? mp_src :
                       (map_m | v_war | v_bull) ? {1'b0, bus_a[14:0]} :
                       (v_cdb && bus_a[14] && bus_a[13:11] == 3'b100) ? {1'b0, bus_a[14:0]} :
                       {2'b00, bus_a[13:0]};
wire  [7:0] rom_q_raw;
wire  [7:0] rom_q = v_multi ? {rom_q_raw[0], rom_q_raw[2], rom_q_raw[1], rom_q_raw[3],
                               rom_q_raw[4], rom_q_raw[5], rom_q_raw[6], rom_q_raw[7]} : rom_q_raw;

dpram_dc #(.widthad_a(16)) prog_rom
(
    .clock_a(clk), .address_a(ioctl_addr[15:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:16] == 9'd0), .q_a(),
    .clock_b(clk), .address_b(rom_addr), .q_b(rom_q_raw)
);

// work RAM, zeroed while in reset (MAME clears it for Magic Worm; every set initialises it anyway)
reg  [9:0] clr_addr = 10'd0;
always @(posedge clk) clr_addr <= reset ? clr_addr + 10'd1 : 10'd0;
wire [7:0] ram_q;
dpram_dc #(.widthad_a(10)) work_ram
(
    .clock_a(clk), .address_a(reset ? clr_addr : bus_a[9:0]), .data_a(reset ? 8'd0 : bus_dout),
    .wren_a(reset | (bus_we & sel_ram)), .q_a(ram_q),
    .clock_b(clk), .address_b(hs_addr), .data_b(hs_din), .wren_b(hs_we), .q_b(hs_dout)
);

// ---------------------------------------------------------------- playfield RAM

// 0000-03BF tile codes (32 x 30), 03C0 sprite picture, 03D0 vertical, 03E0 horizontal, 03F0 color
reg  [9:0] pf_vaddr;
wire [7:0] pf_q, pf_vq;
dpram_dc #(.widthad_a(10)) pf_ram
(
    .clock_a(clk), .address_a(bus_a[9:0]), .data_a(bus_dout), .wren_a(bus_we & sel_pf), .q_a(pf_q),
    .clock_b(clk), .address_b(pf_vaddr), .q_b(pf_vq)
);

// ---------------------------------------------------------------- output latch, flip, IRQ, watchdog

reg  [7:0] outlatch = 8'd0;
always @(posedge clk) begin
    if (sys_reset) outlatch <= 8'd0;
    else if (bus_we & wr_latch) outlatch[bus_a[2:0]] <= bus_dout[7];
end
wire flip     = map_m ? outlatch[6] : v_war ? 1'b0 : outlatch[7];   // trackball player, Millipede tile / sprite flip
wire tben     = (map_m && !v_maze) ? outlatch[5] : 1'b1;            // Millipede: 0 = IN0 / IN1 low nibble reads DIPs
wire cntrlsel = map_m && !v_maze && outlatch[7];                    // Millipede: player 2 joystick replaces player 1's
wire upright  = v_war & in0[7];                                     // Warlords: overlay cabinet, mirrored, grey

// IRQ on 16V rising, asserted when 32V is high (4 per frame); acknowledged by a write (none on Bulls Eye Darts)
always @(posedge clk) begin
    if (sys_reset || v_bull)               irq_n <= 1'b1;
    else if (bus_we & wr_irqack)           irq_n <= 1'b1;
    else if (line_end && !vcnt[4] && vcnt_next[4]) irq_n <= ~vcnt_next[5];
end

// watchdog: 8 frames without a write resets the board (MAME set_vblank_count 8; Bulls Eye Darts has none)
reg  [3:0] wd_cnt = 4'd0;
reg        vbl_d = 1'b0;
reg  [7:0] wd_hold = 8'd0;
always @(posedge clk) begin
    vbl_d <= vblank;
    if (reset || v_bull || (bus_we & wr_wdog)) wd_cnt <= 4'd0;
    else if (vblank & ~vbl_d)                  wd_cnt <= wd_cnt + 4'd1;
    if (wd_cnt == 4'd8 && !wd_fire) begin wd_fire <= 1'b1; wd_hold <= 8'd0; wd_cnt <= 4'd0; end
    else if (wd_fire) begin
        wd_hold <= wd_hold + 8'd1;
        if (&wd_hold) wd_fire <= 1'b0;
    end
end

// Multipede banks; Maze Invaders stick select; Bulls Eye Darts sprite / tile banks (extended I/O 00, 20-3F)
reg  [1:0] stick_sel = 2'd0;
reg  [7:0] b_sbank = 8'd0;
reg  [3:0] b_tbank[32];
always @(posedge clk) begin
    if (reset) begin prg_bank <= 1'b0; gfx_hi <= 1'b0; end
    else if (bus_we & wr_prgb) prg_bank <= bus_dout[7];
    else if (bus_we & wr_gfxb) gfx_hi   <= bus_dout[7];
    if (bus_we & wr_ssel) stick_sel <= bus_a[1:0];
    if (s26_acc && !s26_mio && s26_wr && s26_ene) begin
        if (s26_ad[7:0] == 8'h00)      b_sbank <= s26_dw;
        if (s26_ad[7:5] == 3'b001)     b_tbank[s26_ad[4:0]] <= s26_dw[3:0];
    end
end

// ---------------------------------------------------------------- EAROM (ER2055, 64 bytes)

reg  [5:0] ea_addr = 6'd0;
reg  [7:0] ea_in = 8'd0;
reg  [3:0] ea_ctrl = 4'b0010;
wire [7:0] ea_q;
always @(posedge clk) begin
    if (sys_reset) ea_ctrl <= 4'b0010;
    else if (bus_we & wr_eactl) ea_ctrl <= {bus_dout[3:2], ~bus_dout[1], bus_dout[0]};
    if (bus_we & wr_eaadr) begin ea_addr <= bus_a[5:0]; ea_in <= bus_dout; end
end
wire [7:0] ea_nv_q, eep_nv_q;
assign nv_dout = v_multi ? eep_nv_q : ea_nv_q;
dpram_dc #(.widthad_a(6)) earom
(
    .clock_a(clk), .address_a(ea_addr), .data_a(ea_ctrl[2] ? 8'h00 : ea_in),
    .wren_a(ce_pix & ea_ctrl[3] & ~ea_ctrl[1]), .q_a(ea_q),
    .clock_b(clk), .address_b(nv_addr[5:0]), .data_b(nv_din), .wren_b(nv_we & nv_addr[7:6] == 2'b00 & ~v_multi),
    .q_b(ea_nv_q)
);

// ---------------------------------------------------------------- Multipede 93C46

reg  eep_di = 1'b0, eep_sk = 1'b0, eep_cs = 1'b0, eep_wr = 1'b0;
wire eep_do;
always @(posedge clk) begin
    eep_wr <= 1'b0;
    if (bus_we & wr_eepw) begin
        if (!bus_a[1]) eep_di <= bus_dout[7];
        if (!bus_a[2]) eep_sk <= ~bus_dout[7];
        else if (bus_a[1]) eep_cs <= bus_dout[7];
        eep_wr <= 1'b1;
    end
end
eeprom_93c46 eeprom
(
    .clk(clk), .reset(reset), .clear(1'b0),
    .wr(eep_wr), .di(eep_di), .sk(eep_sk), .cs(eep_cs), .dout(eep_do),
    .ext_sel(nv_sel & v_multi), .ext_addr(nv_addr[6:0]), .ext_din(nv_din), .ext_we(nv_we & ~nv_addr[7] & v_multi),
    .ext_q(eep_nv_q)
);

// ---------------------------------------------------------------- inputs

// trackball: counter low nibble + direction of the last count; player 2's when flipped (cocktail)
wire [7:0] tbx = flip ? tb2_x : tb1_x;
wire [7:0] tby = flip ? tb2_y : tb1_y;
wire       tbsx = flip ? tb_sign[2] : tb_sign[0];
wire       tbsy = flip ? tb_sign[3] : tb_sign[1];

wire [7:0] in0_v = v_maze ? {vblank, in0[6:0]} : {in0[7], vblank, in0[5:0]};   // VBLANK: bit 7 on Maze Invaders
wire [7:0] c_in0 = (v_tb && !v_war) ? {tbsx, in0_v[6:4], tbx[3:0]} : in0_v;
wire [7:0] c_in2 = (v_tb && !v_war) ? {tbsy, in2[6:4], tby[3:0]} : in2;
wire [7:0] m_in0 = v_maze ? in0_v : tben ? {tbsx, in0_v[6:4], tbx[3:0]} : {tbsx, in0_v[6:0]};
wire [7:0] m_in1 = v_maze ? in1 : tben ? {tbsy, in1[6:4], tby[3:0]} : {tbsy, in1[6:0]};
wire [7:0] m_in2 = cntrlsel ? {in2[7:4], in3[2], in3[3], in3[0], in3[1]} : in2;   // MAME: P2 stick, reversed

reg  [7:0] in_q;
always @* begin
    if (map_m)
        case (in_idx)
            2'd0: in_q = m_in0;
            2'd1: in_q = m_in1;
            2'd2: in_q = m_in2;
            2'd3: in_q = in3;
        endcase
    else
        case (in_idx)
            2'd0: in_q = c_in0;
            2'd1: in_q = in1;
            2'd2: in_q = c_in2;
            2'd3: in_q = in3;
        endcase
end

wire [7:0] stick_q = (stick_sel == 2'd0) ? an0 : (stick_sel == 2'd1) ? an1 : (stick_sel == 2'd2) ? an2 : an3;

// random byte: caterplr 1780, AY port A on centipdb
reg [15:0] lfsr = 16'hACE1;
always @(posedge clk) lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

// ---------------------------------------------------------------- sound

wire [3:0] p1c0, p1c1, p1c2, p1c3, p2c0, p2c1, p2c2, p2c3;
wire [7:0] pokey1_q, pokey2_q;

// POKEY clock: a steady 12.096 MHz / 8 = 1.512 MHz (MAME), not the CPU's stretched phi0. Phase ph 3 of every 4th
// pixel, so it never coincides with cpu_ce (ph 7). A CPU write is held until the next POKEY clock.
reg  [1:0] pk_hi = 2'd0;
always @(posedge clk) if (ce_pix) pk_hi <= pk_hi + 2'd1;
wire       pk_ce = ph == 3'd3 && pk_hi == 2'd0 && !pause;
reg        pk1_pend = 1'b0, pk2_pend = 1'b0;
reg  [3:0] pk1_wa = 4'd0, pk2_wa = 4'd0;
reg  [7:0] pk1_wd = 8'd0, pk2_wd = 8'd0;
always @(posedge clk) begin
    if (pk_ce) begin pk1_pend <= 1'b0; pk2_pend <= 1'b0; end
    if (cpu_ce && !cpu_rw_n && sel_pokey1) begin pk1_pend <= 1'b1; pk1_wa <= bus_a[3:0]; pk1_wd <= bus_dout; end
    if (cpu_ce && !cpu_rw_n && sel_pokey2) begin pk2_pend <= 1'b1; pk2_wa <= bus_a[3:0]; pk2_wd <= bus_dout; end
end
wire       pk1_w = pk_ce & pk1_pend, pk2_w = pk_ce & pk2_pend;

// Warlords paddles on POT0-3 as the real pot scan (MAME pokey_potgo / step_pot): after POTGO the counter runs at
// clock / 114 (fast mode: every clock) to 228; a pot latches once the counter passes its value, ALLPOT reads 1 while
// it is still counting and POTn reads the running counter until then
wire [7:0] pot_count1;
wire [7:0] pot_cmp = v_war ? {4'h0, pot_count1 > an3, pot_count1 > an2, pot_count1 > an1, pot_count1 > an0} : 8'h00;

pokey pokey1
(
    .clk(clk), .ce(pk_ce), .enable_179(1'b1), .addr(pk1_w ? pk1_wa : bus_a[3:0]), .data_in(pk1_wd),
    .wr_en(pk1_w), .reset_n(cpu_reset_n),
    .keyboard_scan_enable(1'b0), .keyboard_scan(), .keyboard_response(2'b11),
    .pot_in(pot_cmp), .sio_in1(1'b1), .sio_in2(1'b1), .sio_in3(1'b1),
    .data_out(pokey1_q),
    .channel_0_out(p1c0), .channel_1_out(p1c1), .channel_2_out(p1c2), .channel_3_out(p1c3),
    .irq_n_out(), .sio_out1(), .sio_out2(), .sio_out3(),
    .sio_clockin_in(1'b1), .sio_clockin_out(), .sio_clockin_oe(), .sio_clockout(), .pot_reset(), .pot_count(pot_count1)
);

pokey pokey2
(
    .clk(clk), .ce(pk_ce & map_m), .enable_179(1'b1), .addr(pk2_w ? pk2_wa : bus_a[3:0]), .data_in(pk2_wd),
    .wr_en(pk2_w), .reset_n(cpu_reset_n),
    .keyboard_scan_enable(1'b0), .keyboard_scan(), .keyboard_response(2'b11),
    .pot_in(8'h00), .sio_in1(1'b1), .sio_in2(1'b1), .sio_in3(1'b1),
    .data_out(pokey2_q),
    .channel_0_out(p2c0), .channel_1_out(p2c1), .channel_2_out(p2c2), .channel_3_out(p2c3),
    .irq_n_out(), .sio_out1(), .sio_out2(), .sio_out3(),
    .sio_clockin_in(1'b1), .sio_clockin_out(), .sio_clockin_oe(), .sio_clockout(), .pot_reset(), .pot_count()
);

// Millipede board: DIP switches are POKEY ALLPOT (MAME allpot_r)
wire [7:0] pokey1_rd = (map_m && bus_a[3:0] == 4'h8) ? dsw1 : pokey1_q;
wire [7:0] pokey2_rd = (map_m && bus_a[3:0] == 4'h8) ? dsw2 : pokey2_q;

// AY-3-8910 / SN76496 at 12.096 MHz / 8
reg  [4:0] ay_div = 5'd0;
always @(posedge clk) ay_div <= ay_div + 5'd1;
wire ay_cen = (ay_div == 5'd0) && !pause;

// bus phases: caterplr latches its register number from A0-A3 at the start of every access
reg  cyc_start = 1'b0;
always @(posedge clk) cyc_start <= cpu_ce;
wire ay_cat_latch = v_cat & sel_ay & cyc_start;
wire ay_is_addr   = v_cdb ? bus_a[0] : v_mag ? bus_a[1:0] == 2'b01 : 1'b0;
wire ay_is_data   = v_cdb ? ~bus_a[0] : v_mag ? bus_a[1:0] == 2'b11 : 1'b1;
wire ay_wr_ev     = bus_we & sel_ay & (ay_is_addr | ay_is_data);
wire ay_bdir      = ay_cat_latch | ay_wr_ev;
wire ay_bc1       = ay_cat_latch ? 1'b1 : ay_wr_ev ? ay_is_addr : 1'b1;
wire [7:0] ay_din = ay_cat_latch ? {4'd0, bus_a[3:0]} : bus_dout;
wire [7:0] ay_q;
wire [9:0] ay_sound;

jt49_bus #(.COMP(3'b010)) ay
(
    .rst_n(cpu_reset_n),
    .clk(clk),
    .clk_en(ay_cen),
    .bdir(ay_bdir),
    .bc1(ay_bc1),
    .din(ay_din),
    .sel(1'b1),
    .dout(ay_q),
    .sound(ay_sound),
    .A(), .B(), .C(),
    .sample(),
    .IOA_in(lfsr[7:0]),
    .IOA_out(),
    .IOB_in(8'hFF),
    .IOB_out()
);

wire signed [15:0] ay_audio;
jt49_dcrm2 #(.sw(16)) ay_dcrm
(
    .clk(clk), .cen(dc_cen), .rst(~cpu_reset_n),
    .din({1'b0, ay_sound, 5'd0}), .dout(ay_audio)
);

// Bulls Eye Darts: SN76496 on the 2650 data port (WRTD)
wire signed [10:0] sn_sound;
jt89 sn
(
    .rst(~cpu_reset_n),
    .clk(clk),
    .clk_en(ay_cen),
    .wr_n(~(s26_acc & ~s26_mio & s26_wr & ~s26_ene & s26_dc)),
    .cs_n(1'b0),
    .din(s26_dw),
    .sound(sn_sound),
    .ready()
);

// POKEY output: each channel's volume drives the chip's resistor DAC (90k / 26.5k / 8.05k / 3.4k, MAME vol_init);
// the output follows the summed conductance, normalised here to 0-255 per channel
function automatic [7:0] pk_dac(input [3:0] v);
    case (v)
        4'd0:  pk_dac = 8'd0;   4'd1:  pk_dac = 8'd6;   4'd2:  pk_dac = 8'd21;  4'd3:  pk_dac = 8'd27;
        4'd4:  pk_dac = 8'd68;  4'd5:  pk_dac = 8'd74;  4'd6:  pk_dac = 8'd88;  4'd7:  pk_dac = 8'd94;
        4'd8:  pk_dac = 8'd161; 4'd9:  pk_dac = 8'd167; 4'd10: pk_dac = 8'd181; 4'd11: pk_dac = 8'd187;
        4'd12: pk_dac = 8'd228; 4'd13: pk_dac = 8'd234; 4'd14: pk_dac = 8'd249; default: pk_dac = 8'd255;
    endcase
endfunction
wire [9:0] pk1 = pk_dac(p1c0) + pk_dac(p1c1) + pk_dac(p1c2) + pk_dac(p1c3);
wire [9:0] pk2 = pk_dac(p2c0) + pk_dac(p2c1) + pk_dac(p2c2) + pk_dac(p2c3);
wire [15:0] pk_raw = map_m ? {1'b0, {1'b0, pk1} + {1'b0, pk2}, 4'd0} : {1'b0, pk1, 5'd0};   // full scale 32640

// output stage: DC removal (~7 Hz), then Centipede's op-amp low-pass (MAME set_output_opamp_low_pass 3.3k / 0.01 uF,
// fc 4.8 kHz, 1-pole at 756 kHz; a = 1 - exp(-2 pi fc / fs) = 2575 / 65536)
reg  [9:0] dc_div = 10'd0;
always @(posedge clk) dc_div <= dc_div + 10'd1;
wire dc_cen = dc_div == 10'd0;                      // 47.25 kHz
wire signed [15:0] pk_ac;
jt49_dcrm2 #(.sw(16)) pk_dcrm(.clk(clk), .cen(dc_cen), .rst(~cpu_reset_n), .din(pk_raw), .dout(pk_ac));

reg  signed [15:0] pk_lp = 16'sd0;
wire signed [31:0] pk_step = ($signed({pk_ac[15], pk_ac}) - $signed({pk_lp[15], pk_lp})) * 32'sd2575;
always @(posedge clk) if (dc_div[5:0] == 6'd0) pk_lp <= pk_lp + 16'(pk_step >>> 16);
wire signed [15:0] pokey_audio = (variant <= 8'd1) ? pk_lp : pk_ac;

// levels (sim, coin + start, 2026-10-13): POKEY x2, AY x4 put every board at RMS ~1000-2200, peaks < 20000
function automatic signed [15:0] sat_shl(input signed [15:0] x, input [1:0] k);
    reg signed [18:0] y;
    begin
        y = 19'(x) <<< k;
        sat_shl = (y > 19'sd32767) ? 16'sd32767 : (y < -19'sd32768) ? -16'sd32768 : y[15:0];
    end
endfunction
assign audio = v_ay ? sat_shl(ay_audio, 2'd2) : v_bull ? {sn_sound, 5'd0} : sat_shl(pokey_audio, 2'd1);

// ---------------------------------------------------------------- CPU data in

always @* begin
    cpu_din = 8'hFF;
    if      (sel_ram)               cpu_din = ram_q;
    else if (sel_pf)                cpu_din = pf_q;
    else if (sel_pokey1)            cpu_din = pokey1_rd;
    else if (sel_pokey2)            cpu_din = pokey2_rd;
    else if (sel_dsw)               cpu_din = dsw_hi ? dsw2 : dsw1;
    else if (sel_in)                cpu_din = in_q;
    else if (sel_stick)             cpu_din = stick_q;
    else if (sel_ear)               cpu_din = ea_q;
    else if (sel_rand)              cpu_din = lfsr[7:0];
    else if (sel_ay)                cpu_din = ay_q;
    else if (sel_eep)               cpu_din = {eep_do, 7'd0};
    else if (mp_io)                 cpu_din = 8'h00;                 // Multipede A030: nop read
    else if (sel_rom)               cpu_din = rom_q;
end

// 2650: RAM / playfield / ROM come straight from the BRAMs (addressed during ack); registers are latched at ack.
// Data port read = MAME's protection answer: 1 at PC 0033 / 6B19 (pc = last code fetch + 1)
wire [14:0] s26_pc = s26_lastcode + 15'd1;
reg  [1:0] s26_src = 2'd0;                          // 0 register, 1 RAM, 2 playfield, 3 ROM
reg  [7:0] s26_reg = 8'd0;
always @(posedge clk) if (s26_acc) begin
    s26_src <= !s26_mio ? 2'd0 : sel_ram ? 2'd1 : sel_pf ? 2'd2 : sel_rom ? 2'd3 : 2'd0;
    s26_reg <= !s26_mio ? {7'd0, ~s26_ene & s26_dc & (s26_pc == 15'h0033 || s26_pc == 15'h6B19)} :
               sel_dsw ? (dsw_hi ? dsw2 : dsw1) : sel_in ? in_q : sel_ear ? ea_q : 8'hFF;
end
assign s26_dr = (s26_src == 2'd1) ? ram_q : (s26_src == 2'd2) ? pf_q : (s26_src == 2'd3) ? rom_q : s26_reg;

// ---------------------------------------------------------------- palette

// Centipede / Bulls Eye Darts: 16 x 4-bit color RAM (1400-140F): 4-7 playfield, C-F sprites; inverted R, G, B, alt
// Millipede: 32 x 8-bit palette RAM (2480-249F): 00-0F playfield (4 x 4), 10-1F sprites (4 banks x 4)
reg [3:0] cram[16];
reg [7:0] mpal[32];
always @(posedge clk) begin
    if (bus_we & wr_color) cram[bus_a[3:0]] <= bus_dout[3:0];
    if (bus_we & wr_pal)   mpal[bus_a[4:0]] <= bus_dout;
end

// ---------------------------------------------------------------- video: fetch

// Each 8-pixel char: during the stamp half (256H low) sprite H[6:3] is read, otherwise the tile under H.
// Data fetched during char k is shifted out during char k+1 (tiles to the screen, sprites into the line buffer).
// mx / my: mirror the picture (CRT flip; Warlords' upright cabinet mirrors X)
wire       mx     = crt_flip ^ upright;
wire       my     = crt_flip;
wire [7:0] flip_v = (vcnt < 8'd240) ? 8'd239 - vcnt : vcnt;
wire [7:0] vline  = my ? flip_v : vcnt;
wire [4:0] tcol   = mx ? ~hcnt[7:3] : hcnt[7:3];
wire       stamp  = !hcnt[8];
wire [5:0] cc     = {hcnt[2:0], ph};                // clock within the char

always @* begin
    if (stamp)
        case (cc[3:1])
            3'd0:    pf_vaddr = {6'b111100, hcnt[6:3]};   // picture
            3'd1:    pf_vaddr = {6'b111101, hcnt[6:3]};   // vertical
            3'd2:    pf_vaddr = {6'b111110, hcnt[6:3]};   // horizontal
            default: pf_vaddr = {6'b111111, hcnt[6:3]};   // color
        endcase
    else
        pf_vaddr = {vline[7:3], tcol};
end

reg  [7:0] f_pic = 8'd0, f_vert = 8'd0, f_horiz = 8'd0, f_color = 8'd0;
reg  [2:0] f_wcol = 3'd0;                           // Warlords tile color: {upright, bottom half, right half}
reg  [3:0] f_tbank = 4'd0;                          // Bulls Eye Darts tile bank of this column
always @(posedge clk) begin
    if (cc == 6'd2) begin
        f_pic   <= pf_vq;
        f_wcol  <= {upright, pf_vaddr[9], pf_vaddr[4]};
        f_tbank <= b_tbank[pf_vaddr[4:0]];
    end
    if (cc == 6'd4) f_vert  <= pf_vq;
    if (cc == 6'd6) f_horiz <= pf_vq;
    if (cc == 6'd8) f_color <= pf_vq;
end

wire [7:0] msum    = vline + f_vert;
wire       s_match = v_war ? msum[7:3] == 5'h1F : msum[7:4] == 4'hF;   // Warlords sprites are 8 lines
wire       s_flipy = (map_m && !v_maze) ? f_pic[7] ^ flip : f_pic[7];
wire       t_flipy = v_mtile ? flip : f_pic[7];
wire [3:0] s_row   = msum[3:0] ^ {4{s_flipy}};
wire [2:0] t_row   = vline[2:0] ^ {3{t_flipy}};

// gfx ROM address (8K per plane):
//   Centipede / Millipede tiles {bank, A10, 1, code, row}, sprites {bank, pic0, 0, pic[5:1], row[3:0]}
//   Warlords tiles {000, code, row}, sprites (8x8) {001, code, row}
//   Bulls Eye Darts tiles {column bank, code, row}, sprites (code + 32 x bank) x 16 + row
wire        t_a10   = v_mtile ? f_pic[6] : 1'b0;
wire  [8:0] b_scode = {2'b00, f_pic[0], 1'b0, f_pic[5:1]} + {b_sbank[3:0], 5'd0};
reg  [12:0] gfx_addr;
always @* begin
    if (v_war)       gfx_addr = stamp ? {4'b0001, f_pic[5:0], s_row[2:0]} : {4'b0000, f_pic[5:0], t_row};
    else if (v_bull) gfx_addr = stamp ? {b_scode, s_row} : {f_tbank, f_pic[5:0], t_row};
    else             gfx_addr = stamp ? {gfx_bank, f_pic[0], 1'b0, f_pic[5:1], s_row}
                                      : {gfx_bank, t_a10, 1'b1, f_pic[5:0], t_row};
end
wire [7:0] gfx_lo_q, gfx_hi_q;

dpram_dc #(.widthad_a(13)) gfx_rom_lo
(
    .clock_a(clk), .address_a(ioctl_addr[12:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:13] == 12'h008), .q_a(),
    .clock_b(clk), .address_b(gfx_addr), .q_b(gfx_lo_q)
);
dpram_dc #(.widthad_a(13)) gfx_rom_hi
(
    .clock_a(clk), .address_a(ioctl_addr[12:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:13] == 12'h009), .q_a(),
    .clock_b(clk), .address_b(gfx_addr), .q_b(gfx_hi_q)
);

function automatic [7:0] rev8(input [7:0] d);
    rev8 = {d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]};
endfunction

// ---------------------------------------------------------------- video: shifter, line buffer

wire       char_end = ce_pix && hcnt[2:0] == 3'd7;
wire       s_flipx  = (map_m && !v_maze) ? flip : v_bull ? 1'b1 : f_pic[6];
wire       t_flipx  = v_mtile ? flip : f_pic[6];
wire       f_flipx  = (stamp ? s_flipx : t_flipx) ^ mx;
wire       f_blank  = stamp && !s_match;            // sprite not on this line

reg  [7:0] sh_lo = 8'd0, sh_hi = 8'd0;
reg        sh_spr = 1'b0;                           // the char in the shifter is a sprite
reg  [7:0] sh_color = 8'd0;                         // sprite color byte
reg  [1:0] sh_tcol = 2'd0;                          // Millipede tile color
reg  [2:0] sh_wcol = 3'd0;                          // Warlords tile color
reg  [7:0] lb_wa = 8'd0;                            // sprite: line buffer address of the current pixel
reg  [7:0] lb_x = 8'd0;                             // display: line buffer address of the current pixel

always @(posedge clk) begin
    if (char_end) begin
        sh_lo    <= f_blank ? 8'd0 : (f_flipx ? rev8(gfx_lo_q) : gfx_lo_q);
        sh_hi    <= f_blank ? 8'd0 : (f_flipx ? rev8(gfx_hi_q) : gfx_hi_q);
        sh_spr   <= stamp;
        sh_color <= f_color;
        sh_tcol  <= f_pic[7:6];
        sh_wcol  <= f_wcol;
        lb_wa    <= mx ? 8'd248 - f_horiz : f_horiz;
    end
    else if (ce_pix) begin
        sh_lo <= {sh_lo[6:0], 1'b0};
        sh_hi <= {sh_hi[6:0], 1'b0};
        lb_wa <= lb_wa + 8'd1;
    end
    if (char_end && hcnt == 9'h107) lb_x <= 8'd0;    // first visible pixel = line buffer 0
    else if (ce_pix)                lb_x <= lb_x + 8'd1;
end

wire [1:0] pen = {sh_hi[7], sh_lo[7]};

// visible: line buffer pixels 0-255 = H 108-1FF and 080-087 of the next line period
reg        vis = 1'b0;
reg        disp_vbl = 1'b1;
always @(posedge clk) if (ce_pix) begin
    if (hcnt == 9'h107) begin vis <= 1'b1; disp_vbl <= vblank; end
    else if (hcnt == 9'h087) vis <= 1'b0;
end

// sprite pens through the color byte: [1:0] pen 1, [3:2] pen 2, [5:4] pen 3, [7:6] Millipede palette bank;
// Warlords keeps the pen (colors come from the screen quadrant)
wire [1:0] s_col = (pen == 2'd1) ? sh_color[1:0] : (pen == 2'd2) ? sh_color[3:2] : sh_color[5:4];
wire [7:0] lb_wd = v_war ? {6'd0, pen} : {4'd0, sh_color[7:6], s_col};
wire       lb_we_spr = sh_spr && pen != 2'd0 && ph == 3'd4;
wire       lb_we_clr = vis && ph == 3'd5;
wire [7:0] lb_q;

dpram_dc #(.widthad_a(8)) line_buf
(
    .clock_a(clk), .address_a(lb_we_spr ? lb_wa : lb_x), .data_a(lb_we_spr ? lb_wd : 8'd0),
    .wren_a(lb_we_spr | lb_we_clr), .q_a(),
    .clock_b(clk), .address_b(lb_x), .q_b(lb_q)
);

// ---------------------------------------------------------------- video: color out

// the pixel is read at ph 3, before the clear at ph 5
reg  [3:0] lb_pix = 4'd0;
always @(posedge clk) if (ph == 3'd3) lb_pix <= lb_q[3:0];

// sprites hidden on line buffer pixels 0-7 and 248-255 (lisper's MOB gating: first char after 256H falls and
// the first char of 256HD; MAME clips 248-255)
wire       spr_on = vis && lb_pix[1:0] != 2'd0 && lb_x[7:3] != 5'd0 && lb_x[7:3] != 5'd31;
wire [3:0] c_idx  = spr_on ? {2'b11, lb_pix[1:0]} : {2'b01, pen};
wire [3:0] c_val  = cram[c_idx];
wire [4:0] m_idx  = spr_on ? {1'b1, lb_pix} : {1'b0, sh_tcol, pen};
wire [7:0] m_val  = mpal[m_idx];

// color PROM: Warlords palette (sprite color from the screen quadrant of the pixel), Maze Invaders palette lookup
wire       q_x    = crt_flip ? ~lb_x[7] : lb_x[7];
wire [2:0] w_scol = {upright, vline[7], q_x};
wire [6:0] w_idx  = spr_on ? {w_scol, lb_pix[1:0], 2'b00} : {sh_wcol, 2'b00, pen};
wire [7:0] cprom_q;
dpram_dc #(.widthad_a(8)) color_prom
(
    .clock_a(clk), .address_a(ioctl_addr[7:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:8] == 17'h00141), .q_a(),
    .clock_b(clk), .address_b(v_maze ? {4'd0, ~m_val[3:0]} : {1'b0, w_idx}), .q_b(cprom_q)
);

function automatic [7:0] w3(input [2:0] b);          // Millipede resistor weights 0x21 / 0x47 / 0x97
    w3 = (b[0] ? 8'h21 : 8'h00) + (b[1] ? 8'h47 : 8'h00) + (b[2] ? 8'h97 : 8'h00);
endfunction

wire       c_r = ~c_val[0], c_g = ~c_val[1], c_b = ~c_val[2], c_alt = ~c_val[3];
wire [7:0] cr8 = c_r ? 8'hFF : 8'h00;
wire [7:0] cg8 = c_g ? ((c_alt && !c_b) ? 8'hC0 : 8'hFF) : 8'h00;
wire [7:0] cb8 = c_b ? (c_alt ? 8'hC0 : 8'hFF) : 8'h00;

// Warlords: PROM bit 2 R, 1 G, 0 B; entries 40-7F (upright, overlay) in grey (MAME 30/59/11 %)
wire [7:0] w_grey = (cprom_q[2] ? 8'h4D : 8'h00) + (cprom_q[1] ? 8'h96 : 8'h00) + (cprom_q[0] ? 8'h1C : 8'h00);

// hsync: 256H low, from H 0A8 for 32 pixels (lisper's hs / hsync flip-flops), moved by h_adj
wire [8:0] hs_on = 9'h0A8 + {{4{h_adj[3]}}, h_adj, 1'b0};

// vsync: the sync PROM's, moved by v_adj lines (frame length and pulse width measured from the PROM)
reg  [8:0] lc = 9'd0, frame_lines = 9'd262, vs_len = 9'd3, vs_cnt = 9'd0;
reg        vs_p_d = 1'b0;
wire [8:0] vs_start = v_adj[3] ? frame_lines + {{5{1'b1}}, v_adj} : {5'd0, v_adj};
always @(posedge clk) if (line_end) begin
    vs_p_d <= vsync_p;
    if (vsync_p && !vs_p_d) begin frame_lines <= lc + 9'd1; lc <= 9'd0; vs_cnt <= 9'd1; end
    else begin
        lc <= lc + 9'd1;
        if (vsync_p) vs_cnt <= vs_cnt + 9'd1;
    end
    if (!vsync_p && vs_p_d) vs_len <= vs_cnt;
end

always @(posedge clk) if (ce_pix) begin
    if (!vis || disp_vbl) begin
        video_r <= 8'd0; video_g <= 8'd0; video_b <= 8'd0;
    end else if (v_war) begin
        if (upright) begin video_r <= w_grey; video_g <= w_grey; video_b <= w_grey; end
        else begin
            video_r <= {8{cprom_q[2]}}; video_g <= {8{cprom_q[1]}}; video_b <= {8{cprom_q[0]}};
        end
    end else if (v_maze) begin                       // ~PROM[~data & 0F] through milliped_set_color
        video_r <= w3(cprom_q[7:5]);
        video_g <= w3({cprom_q[4:3], 1'b0});
        video_b <= w3(cprom_q[2:0]);
    end else if (v_mpal) begin
        video_r <= w3(~m_val[7:5]);
        video_g <= w3({~m_val[4:3], 1'b0});
        video_b <= w3(~m_val[2:0]);
    end else begin
        video_r <= cr8; video_g <= cg8; video_b <= cb8;
    end
    video_hblank <= !vis;
    video_vblank <= disp_vbl;
    if      (hcnt == hs_on)          video_hs <= 1'b1;
    else if (hcnt == hs_on + 9'h020) video_hs <= 1'b0;
    video_vs <= (lc >= vs_start && lc < vs_start + vs_len) || (vs_start + vs_len > frame_lines && lc < vs_start + vs_len - frame_lines);
end

endmodule

//============================================================================
//
//  Atari Crystal Castles hardware (MAME ccastles.cpp) for MiSTer
//  Copyright (C) 2026 Rodimus
//
//  Derived from the MiSTer Crystal Castles core by Enceladus (bus decode, auto-increment, POKEY with
//  clock enable) and MAME ccastles.cpp by Aaron Giles (sync / write-protect / priority PROMs, sprite
//  list, scroll, palette weights).
//
//  Variants (MRA index 1 byte 0): 10 ccastles (trackball), 11 ccastlesj (joystick on LETA 0)
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

// ioctl index 0 layout (shared with centiped_board): CPU ROM 0x00000 (A000-FFFF at their addresses, the
// banked A000-DFFF half at 0x2000-0x5FFF), sprite ROM 8D 0x10000, 8B 0x12000, sync PROM 0x14000,
// write-protect PROM 0x14100, priority PROM 0x14200

module ccastles_board
(
    input               clk,            // 50 MHz = 5 x 10 MHz (the PLL is retuned for this board)
    input               reset,
    input               pause,
    input               joy_ver,        // ccastlesj: LETA 0 is a joystick (in2)

    input         [7:0] in0, in1, in2,  // IN0, IN1 (POKEY 2 ALLPOT: DIPs, cocktail starts), LETA 0 (ccastlesj)
    input         [7:0] tb1_x, tb1_y,   // trackball counters, player 1 / 2
    input         [7:0] tb2_x, tb2_y,

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

    // X2212 EEPROM shadow (256 bytes) for the NVRAM file, while the CPU is held or paused
    input               nv_sel,
    input         [7:0] nv_addr,
    input         [7:0] nv_din,
    input               nv_we,
    output        [7:0] nv_dout
);

// ---------------------------------------------------------------- timebase

// 10 clocks per pixel (5 MHz); H 0-319 (active 0-255), V 0-255; the CPU runs at 10 MHz / 8 = 1.25 MHz
reg  [3:0] pc = 4'd0;
always @(posedge clk) pc <= (pc == 4'd9) ? 4'd0 : pc + 4'd1;
assign ce_pix = (pc == 4'd9);

reg  [5:0] cdiv = 6'd0;
always @(posedge clk) cdiv <= (cdiv == 6'd39) ? 6'd0 : cdiv + 6'd1;
wire cpu_ce = (cdiv == 6'd39) && !pause;

reg  [8:0] hcnt = 9'd0;
reg  [7:0] vcnt = 8'd0;
wire       line_start = ce_pix && hcnt == 9'd319;
always @(posedge clk) if (ce_pix) begin
    hcnt <= line_start ? 9'd0 : hcnt + 9'd1;
    if (line_start) vcnt <= vcnt + 8'd1;
end

// sync PROM 7K at V: bit 0 VBLANK, bit 3 IRQCK (IRQ on its rising edge)
wire [7:0] sync_q;
dpram_dc #(.widthad_a(8)) sync_prom
(
    .clock_a(clk), .address_a(ioctl_addr[7:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:8] == 17'h00140), .q_a(),
    .clock_b(clk), .address_b(vcnt), .q_b(sync_q)
);
wire vblank = sync_q[0];

reg  [7:0] vb_end = 8'd24;                          // first visible line (MAME finds it in the PROM)
reg        vb_d = 1'b1, irqck_d = 1'b0;
always @(posedge clk) if (ce_pix && hcnt == 9'd2) begin
    vb_d    <= vblank;
    irqck_d <= sync_q[3];
    if (vb_d && !vblank) vb_end <= vcnt;
end
wire irq_edge = ce_pix && hcnt == 9'd2 && sync_q[3] && !irqck_d;

// ---------------------------------------------------------------- reset, watchdog, CPU

reg  [3:0] wd_cnt = 4'd0;
reg  [7:0] wd_hold = 8'd0;
reg        wd_fire = 1'b0;
wire       sys_reset = reset | wd_fire;
reg  [7:0] rst_cnt = 8'd0;
always @(posedge clk) begin
    if (sys_reset)                  rst_cnt <= 8'd0;
    else if (ce_pix && !rst_cnt[4]) rst_cnt <= rst_cnt + 8'd1;
end
wire cpu_reset_n = rst_cnt[4];

wire [23:0] cpu_a24;
wire [15:0] a = cpu_a24[15:0];
wire  [7:0] cpu_dout;
reg   [7:0] cpu_din;
wire        cpu_rw_n;
reg         irq_n = 1'b1;
wire        we = cpu_ce & ~cpu_rw_n;

T65 cpu
(
    .Mode(2'b00),
    .Res_n(cpu_reset_n),
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

// ---------------------------------------------------------------- address decode

// 0000-7FFF bitmap DRAM (0000/0001 also X/Y, 0002 bit mode), 8000-8FFF static RAM (8E00 / 8F00 sprite buffers),
// 9000 NVRAM, 9400 trackballs, 9600 IN0, 9800 POKEY 1, 9A00 POKEY 2, 9C00 recall, 9C80 H scroll, 9D00 V scroll,
// 9D80 IRQ ack, 9E00 watchdog, 9E80 OUT0 (D0), 9F00 OUT1 (D3), 9F80 colors, A000-FFFF ROM (A000-DFFF banked)
wire sel_dram  = !a[15];
wire sel_bitmd = a == 16'h0002;
wire sel_xy    = a[15:1] == 15'd0;
wire sel_sram  = a[15:12] == 4'h8;
wire io        = a[15:12] == 4'h9;
wire sel_nv    = io && a[11:10] == 2'b00;
wire sel_leta  = io && a[11:9] == 3'b010;
wire sel_in0   = io && a[11:9] == 3'b011;
wire sel_pk1   = io && a[11:9] == 3'b100;
wire sel_pk2   = io && a[11:9] == 3'b101;
wire w_grp     = io && a[11:10] == 2'b11;
wire wr_hscr   = we && w_grp && a[9:7] == 3'b001;
wire wr_vscr   = we && w_grp && a[9:7] == 3'b010;
wire wr_iack   = we && w_grp && a[9:7] == 3'b011;
wire wr_wdog   = we && w_grp && a[9:7] == 3'b100;
wire wr_out0   = we && w_grp && a[9:7] == 3'b101;
wire wr_out1   = we && w_grp && a[9:7] == 3'b110;
wire wr_color  = we && w_grp && a[9:7] == 3'b111;
wire sel_rom   = a[15:13] >= 3'b101;

reg  [7:0] out0 = 8'd0, out1 = 8'd0;
always @(posedge clk) begin
    if (sys_reset) begin out0 <= 8'd0; out1 <= 8'd0; end
    else begin
        if (wr_out0) out0[a[2:0]] <= cpu_dout[0];
        if (wr_out1) out1[a[2:0]] <= cpu_dout[3];
    end
end
wire bank1   = out0[7];
wire n_ax    = out1[0], n_ay = out1[1], n_xinc = out1[2], n_yinc = out1[3];
wire player2 = out1[4];                             // cocktail flip
wire buf1    = out1[7];                             // sprite list: 1 = 8F00, 0 = 8E00

reg  [7:0] hscroll = 8'd0, vscroll = 8'd0;
always @(posedge clk) begin
    if (wr_hscr) hscroll <= cpu_dout;
    if (wr_vscr) vscroll <= cpu_dout;
end

always @(posedge clk) begin
    if (sys_reset || wr_iack) irq_n <= 1'b1;
    else if (irq_edge)        irq_n <= 1'b0;
end

// watchdog: 8 frames (MAME set_vblank_count 8)
reg vbw_d = 1'b0;
always @(posedge clk) begin
    vbw_d <= vblank;
    if (reset || wr_wdog)    wd_cnt <= 4'd0;
    else if (vblank && !vbw_d) wd_cnt <= wd_cnt + 4'd1;
    if (wd_cnt == 4'd8 && !wd_fire) begin wd_fire <= 1'b1; wd_hold <= 8'd0; wd_cnt <= 4'd0; end
    else if (wd_fire) begin
        wd_hold <= wd_hold + 8'd1;
        if (&wd_hold) wd_fire <= 1'b0;
    end
end

// ---------------------------------------------------------------- ROM

wire [15:0] rom_addr = (bank1 && a[15:13] != 3'b111) ? a - 16'h8000 : a;   // bank 1: region 10000 at 2000
wire  [7:0] rom_q;
dpram_dc #(.widthad_a(16)) prog_rom
(
    .clock_a(clk), .address_a(ioctl_addr[15:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:16] == 9'd0), .q_a(),
    .clock_b(clk), .address_b(rom_addr), .q_b(rom_q)
);

// ---------------------------------------------------------------- bitmap DRAM

// 16K words of 4 pixels; a CPU write goes through the write-protect PROM (MAME write_vram) one nibble at a time.
// Bit mode: address from the X/Y latches ({Y, X[7:1]}), pixel = X[0], auto-increment after each access.
reg  [7:0] bx = 8'd0, by = 8'd0;
wire [14:0] ea = sel_bitmd ? {by, bx[7:1]} : a[14:0];
wire  [7:0] ev_q, od_q;                             // even / odd byte of the word at ea
wire  [7:0] wp_q;
wire  [7:0] wp_idx = {ea[14:12] == 3'd0, ea[11:10], ~sel_bitmd, 1'b0, ea[0], sel_bitmd ? bx[1:0] : 2'b00};
wire  [7:0] wdat = sel_bitmd ? {cpu_dout[7:4], cpu_dout[7:4]} : cpu_dout;
wire  [7:0] ev_w = {wp_q[1] ? ev_q[7:4] : wdat[7:4], wp_q[0] ? ev_q[3:0] : wdat[3:0]};
wire  [7:0] od_w = {wp_q[3] ? od_q[7:4] : wdat[7:4], wp_q[2] ? od_q[3:0] : wdat[3:0]};
wire        dram_we = we & sel_dram;
wire  [7:0] dram_byte = ea[0] ? od_q : ev_q;
wire  [7:0] dram_rd = sel_bitmd ? {bx[0] ? dram_byte[7:4] : dram_byte[3:0], 4'hF} : dram_byte;

dpram_dc #(.widthad_a(8)) wp_prom
(
    .clock_a(clk), .address_a(ioctl_addr[7:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:8] == 17'h00141), .q_a(),
    .clock_b(clk), .address_b(wp_idx), .q_b(wp_q)
);

always @(posedge clk) begin
    if (sys_reset) begin bx <= 8'd0; by <= 8'd0; end
    else if (we && sel_xy && !a[0]) bx <= cpu_dout;
    else if (we && sel_xy &&  a[0]) by <= cpu_dout;
    else if (cpu_ce && sel_bitmd) begin             // read or write of 0002
        if (!n_ax) bx <= n_xinc ? bx - 8'd1 : bx + 8'd1;
        if (!n_ay) by <= n_yinc ? by - 8'd1 : by + 8'd1;
    end
end

// video side: the byte of the line / pixel being fetched
reg  [13:0] v_waddr;
wire  [7:0] v_ev, v_od;
dpram_dc #(.widthad_a(14)) dram_even
(
    .clock_a(clk), .address_a(ea[14:1]), .data_a(ev_w), .wren_a(dram_we), .q_a(ev_q),
    .clock_b(clk), .address_b(v_waddr), .q_b(v_ev)
);
dpram_dc #(.widthad_a(14)) dram_odd
(
    .clock_a(clk), .address_a(ea[14:1]), .data_a(od_w), .wren_a(dram_we), .q_a(od_q),
    .clock_b(clk), .address_b(v_waddr), .q_b(v_od)
);

// ---------------------------------------------------------------- static RAM, NVRAM, colors

reg  [11:0] spr_addr;
wire  [7:0] sram_q, spr_q;
dpram_dc #(.widthad_a(12)) sram
(
    .clock_a(clk), .address_a(a[11:0]), .data_a(cpu_dout), .wren_a(we & sel_sram), .q_a(sram_q),
    .clock_b(clk), .address_b(spr_addr), .q_b(spr_q)
);

// X2212 pair (4B low nibble, 4A high nibble): static RAM the CPU sees + EEPROM shadow. RECALL (9C00 write) copies
// shadow -> RAM, STORE (OUT0 Q3 high with Q2 low, rising) copies RAM -> shadow (MAME nvram_recall_w / nvram_store_w).
// The NVRAM file is the shadow; it is also recalled when reset ends.
wire [7:0] nv_q, nvr_q, nve_q;
reg  [8:0] cp_cnt = 9'd0;                           // 0-511: address = cnt[8:1], cnt[0] = write phase
reg        cp_busy = 1'b0, cp_store = 1'b0, store_d = 1'b0, rst_d = 1'b1;
wire       store_now = ~out0[2] & out0[3];
wire       wr_recall = we && w_grp && a[9:7] == 3'b000;
always @(posedge clk) begin
    store_d <= store_now;
    rst_d   <= sys_reset;
    if (cp_busy) begin
        cp_cnt <= cp_cnt + 9'd1;
        if (&cp_cnt) cp_busy <= 1'b0;
    end
    else if ((rst_d && !sys_reset) || wr_recall) begin cp_busy <= 1'b1; cp_store <= 1'b0; cp_cnt <= 9'd0; end
    else if (store_now && !store_d)              begin cp_busy <= 1'b1; cp_store <= 1'b1; cp_cnt <= 9'd0; end
end
dpram_dc #(.widthad_a(8)) nv_ram
(
    .clock_a(clk), .address_a(a[7:0]), .data_a(cpu_dout), .wren_a(we & sel_nv), .q_a(nv_q),
    .clock_b(clk), .address_b(cp_cnt[8:1]), .data_b(nve_q), .wren_b(cp_busy & ~cp_store & cp_cnt[0]), .q_b(nvr_q)
);
dpram_dc #(.widthad_a(8)) nv_shadow
(
    .clock_a(clk), .address_a(nv_we ? nv_addr : cp_cnt[8:1]), .data_a(nv_we ? nv_din : nvr_q),
    .wren_a(nv_we | (cp_busy & cp_store & cp_cnt[0])), .q_a(),
    .clock_b(clk), .address_b(cp_busy ? cp_cnt[8:1] : nv_addr), .q_b(nve_q)
);
assign nv_dout = nve_q;

// 32 colors: R = {A5, D7, D6}, B = D5-D3, G = D2-D0, inverted, 22k / 10k / 4.7k with 1k pull-down (MAME weights)
reg  [8:0] cram[32];
initial for (int i = 0; i < 32; i++) cram[i] = 9'h1FF;   // black until written (MAME palette)
always @(posedge clk) if (wr_color) cram[a[4:0]] <= {a[5], cpu_dout};

function automatic [7:0] lvl(input [2:0] b);
    case (b)
        3'd0: lvl = 8'd0;   3'd1: lvl = 8'd32;  3'd2: lvl = 8'd71;  3'd3: lvl = 8'd104;
        3'd4: lvl = 8'd151; 3'd5: lvl = 8'd184; 3'd6: lvl = 8'd223; default: lvl = 8'd255;
    endcase
endfunction

// ---------------------------------------------------------------- inputs, sound

// LETA: 0 = P1 Y (reversed), 1 = P1 X, 2 = P2 Y (reversed), 3 = P2 X; Crystal Castles (joystick) has the stick on 0
reg [7:0] leta_q;
always @* begin
    case (a[1:0])
        2'd0: leta_q = joy_ver ? in2 : ~tb1_y;
        2'd1: leta_q = joy_ver ? 8'hFF : ~tb1_x;
        2'd2: leta_q = joy_ver ? 8'hFF : tb2_y;
        2'd3: leta_q = joy_ver ? 8'hFF : tb2_x;
    endcase
end
wire [7:0] in0_v = {in0[7:6], vblank, in0[4:0]};

wire [3:0] p1c0, p1c1, p1c2, p1c3, p2c0, p2c1, p2c2, p2c3;
wire [7:0] pk1_q, pk2_q;
pokey pokey1
(
    .clk(clk), .ce(cpu_ce), .enable_179(1'b1), .addr(a[3:0]), .data_in(cpu_dout),
    .wr_en(~cpu_rw_n & sel_pk1), .reset_n(cpu_reset_n),
    .keyboard_scan_enable(1'b0), .keyboard_scan(), .keyboard_response(2'b11),
    .pot_in(8'h00), .sio_in1(1'b1), .sio_in2(1'b1), .sio_in3(1'b1),
    .data_out(pk1_q),
    .channel_0_out(p1c0), .channel_1_out(p1c1), .channel_2_out(p1c2), .channel_3_out(p1c3),
    .irq_n_out(), .sio_out1(), .sio_out2(), .sio_out3(),
    .sio_clockin_in(1'b1), .sio_clockin_out(), .sio_clockin_oe(), .sio_clockout(), .pot_reset(), .pot_count()
);
pokey pokey2
(
    .clk(clk), .ce(cpu_ce), .enable_179(1'b1), .addr(a[3:0]), .data_in(cpu_dout),
    .wr_en(~cpu_rw_n & sel_pk2), .reset_n(cpu_reset_n),
    .keyboard_scan_enable(1'b0), .keyboard_scan(), .keyboard_response(2'b11),
    .pot_in(8'h00), .sio_in1(1'b1), .sio_in2(1'b1), .sio_in3(1'b1),
    .data_out(pk2_q),
    .channel_0_out(p2c0), .channel_1_out(p2c1), .channel_2_out(p2c2), .channel_3_out(p2c3),
    .irq_n_out(), .sio_out1(), .sio_out2(), .sio_out3(),
    .sio_clockin_in(1'b1), .sio_clockin_out(), .sio_clockin_oe(), .sio_clockout(), .pot_reset(), .pot_count()
);
wire [7:0] pk2_rd = (a[3:0] == 4'h8) ? in1 : pk2_q;   // MAME pokey2 allpot_r = IN1

// POKEY resistor DAC (MAME vol_init, see centiped_board.sv); MAME's OPAMP_C_TO_GROUND stage has no low-pass, only the
// high-pass after it (DC removal here, ~7 Hz)
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
reg  [9:0] dc_div = 10'd0;
always @(posedge clk) dc_div <= dc_div + 10'd1;
wire signed [15:0] pk_ac;
jt49_dcrm2 #(.sw(16)) pk_dcrm
(
    .clk(clk), .cen(dc_div == 10'd0), .rst(~cpu_reset_n),
    .din({1'b0, {1'b0, pk1} + {1'b0, pk2}, 4'd0}), .dout(pk_ac)
);
wire signed [16:0] pk_x2 = {pk_ac, 1'b0};             // x2 like the Centipede boards, saturated
assign audio = (pk_x2 > 17'sd32767) ? 16'sd32767 : (pk_x2 < -17'sd32768) ? -16'sd32768 : pk_x2[15:0];

always @* begin
    cpu_din = 8'hFF;
    if      (sel_dram) cpu_din = dram_rd;
    else if (sel_sram) cpu_din = sram_q;
    else if (sel_nv)   cpu_din = nv_q;
    else if (sel_leta) cpu_din = leta_q;
    else if (sel_in0)  cpu_din = in0_v;
    else if (sel_pk1)  cpu_din = pk1_q;
    else if (sel_pk2)  cpu_din = pk2_rd;
    else if (sel_rom)  cpu_din = rom_q;
end

// ---------------------------------------------------------------- sprites

// During line V the engine fills the line buffer for line V+1: clear to 0F, then the 40 entries of the selected
// list (picture, vertical, priority, horizontal), drawn in order with pen 7 transparent (MAME screen_update).
// 8x16, 3 planes: 8D low nibble = plane 2, 8B high / low nibble = planes 1 / 0; two bytes per row.
// mx / my mirror the picture for CRT flip; PLAYER2 flips the sprite images (positions are the game's).
wire       mx = crt_flip, my = crt_flip;
wire [7:0] t_line = vcnt + 8'd1;
wire [7:0] s_t    = my ? 8'd23 - t_line : t_line;   // 279 - t: lines 24-255 mirrored

localparam S_IDLE = 3'd0, S_CLR = 3'd1, S_RD = 3'd2, S_GFX = 3'd3, S_DRAW = 3'd4;
reg  [2:0] st = S_IDLE;
reg  [8:0] cnt = 9'd0;                              // clear address / pixel index
reg  [5:0] sn = 6'd0;                               // sprite number 0-39
reg  [3:0] sstep = 4'd0;
reg  [7:0] s_pic = 8'd0, s_vert = 8'd0, s_pri = 8'd0, s_x = 8'd0;
reg  [7:0] g_lo0 = 8'd0, g_lo1 = 8'd0, g_hi0 = 8'd0, g_hi1 = 8'd0;
reg        lb_we = 1'b0;
reg  [8:0] lb_wa = 9'd0;
reg  [3:0] lb_wd = 4'd0;
reg  [12:0] g_addr = 13'd0;
wire [7:0] gfx_lo_q, gfx_hi_q;

wire [7:0] ssum   = s_t + s_vert;
wire       s_hit  = ssum[7:4] == 4'hF;
wire [3:0] s_row  = ssum[3:0] ^ {4{player2}};

always @* spr_addr = {3'b111, buf1, sn, sstep[2:1]};  // 8E00 / 8F00 + 4n + field

always @(posedge clk) begin
    lb_we <= 1'b0;
    if (line_start) begin st <= S_CLR; cnt <= 9'd0; end
    else case (st)
        S_CLR: begin
            lb_we <= 1'b1; lb_wa <= {t_line[0], cnt[7:0]}; lb_wd <= 4'hF;
            cnt <= cnt + 9'd1;
            if (cnt == 9'd255) begin st <= S_RD; sn <= 6'd0; sstep <= 4'd0; end
        end
        S_RD: begin                                 // field k addressed at step 2k, read at step 2k+2
            sstep <= sstep + 4'd1;
            case (sstep)
                4'd2: s_pic  <= spr_q;
                4'd4: s_vert <= spr_q;
                4'd6: s_pri  <= spr_q;
                4'd8: begin s_x <= spr_q; sstep <= 4'd0; st <= S_GFX; end
                default: ;
            endcase
        end
        S_GFX: begin
            sstep <= sstep + 4'd1;
            case (sstep)
                4'd0: begin
                    if (!s_hit) begin                // not on this line: next sprite
                        sstep <= 4'd0;
                        if (sn == 6'd39) st <= S_IDLE; else begin sn <= sn + 6'd1; st <= S_RD; end
                    end
                    g_addr <= {s_pic, s_row, 1'b0};
                end
                4'd1: g_addr <= {s_pic, s_row, 1'b1};
                4'd2: begin g_lo0 <= gfx_lo_q; g_hi0 <= gfx_hi_q; end
                4'd3: begin g_lo1 <= gfx_lo_q; g_hi1 <= gfx_hi_q; cnt <= 9'd0; st <= S_DRAW; end
                default: ;
            endcase
        end
        S_DRAW: begin
            begin : draw
                reg [2:0] px, sx;
                reg [8:0] gx;
                reg [2:0] pen;
                px  = cnt[2:0];
                sx  = player2 ? ~px : px;           // source pixel within the sprite
                gx  = {1'b0, s_x} + {6'd0, px};
                pen = sx[2] ? {g_lo1[~sx[1:0]], g_hi1[{1'b1, ~sx[1:0]}], g_hi1[{1'b0, ~sx[1:0]}]}
                            : {g_lo0[~sx[1:0]], g_hi0[{1'b1, ~sx[1:0]}], g_hi0[{1'b0, ~sx[1:0]}]};
                if (pen != 3'd7 && !gx[8]) begin
                    lb_we <= 1'b1;
                    lb_wa <= {t_line[0], mx ? ~gx[7:0] : gx[7:0]};
                    lb_wd <= {s_pri[7], pen};
                end
            end
            cnt <= cnt + 9'd1;
            if (cnt[2:0] == 3'd7) begin
                sstep <= 4'd0;
                if (sn == 6'd39) st <= S_IDLE; else begin sn <= sn + 6'd1; st <= S_RD; end
            end
        end
        default: ;
    endcase
end

dpram_dc #(.widthad_a(13)) gfx_8d
(
    .clock_a(clk), .address_a(ioctl_addr[12:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:13] == 12'h008), .q_a(),
    .clock_b(clk), .address_b(g_addr), .q_b(gfx_lo_q)
);
dpram_dc #(.widthad_a(13)) gfx_8b
(
    .clock_a(clk), .address_a(ioctl_addr[12:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:13] == 12'h009), .q_a(),
    .clock_b(clk), .address_b(g_addr), .q_b(gfx_hi_q)
);

wire [7:0] lb_q;
dpram_dc #(.widthad_a(9)) line_buf
(
    .clock_a(clk), .address_a(lb_wa), .data_a({4'd0, lb_wd}), .wren_a(lb_we), .q_a(),
    .clock_b(clk), .address_b({vcnt[0], hcnt[7:0]}), .q_b(lb_q)
);

// ---------------------------------------------------------------- bitmap, priority, color

// MAME: effy = ((y - vblank_end) + (flip ? 0 : vscroll)) ^ flip, at least 24; effx = (hscroll + (x ^ flip))
wire [7:0] s_y   = my ? 8'd23 - vcnt : vcnt;
wire [7:0] xg    = mx ? ~hcnt[7:0] : hcnt[7:0];
wire [7:0] ey0   = (s_y - vb_end) + (player2 ? 8'd0 : vscroll);
wire [7:0] ey1   = player2 ? ~ey0 : ey0;
wire [7:0] effy  = (ey1 < 8'd24) ? 8'd24 : ey1;
wire [7:0] effx  = hscroll + (player2 ? ~xg : xg);
always @* v_waddr = {effy, effx[7:2]};

reg  [3:0] pix = 4'd0, mopix = 4'd0;
wire [7:0] pri_q;
always @(posedge clk) if (pc == 4'd2) begin
    pix   <= effx[1] ? (effx[0] ? v_od[7:4] : v_od[3:0]) : (effx[0] ? v_ev[7:4] : v_ev[3:0]);
    mopix <= lb_q[3:0];
end

dpram_dc #(.widthad_a(8)) pri_prom
(
    .clock_a(clk), .address_a(ioctl_addr[7:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:8] == 17'h00142), .q_a(),
    .clock_b(clk), .address_b({3'b010, mopix[2:0], mopix[3], pix[3]}), .q_b(pri_q)
);

wire [4:0] cidx = {pri_q[0], pri_q[1] ? mopix : pix};
wire [8:0] cval = cram[cidx];


// HSYNC H 304-319 (MAME), VSYNC lines 4-6, moved by h_adj / v_adj. Windows are taken modulo the line (320) and
// frame (256): an end compare of 304 + 16 = 320 never matched H 0-319, so HSYNC stuck high (HW 2026-10-13: no picture,
// screenshots failed)
wire [8:0] hs_on  = 9'd304 + {{4{h_adj[3]}}, h_adj, 1'b0};             // 290-318
wire [8:0] hs_rel = (hcnt >= hs_on) ? hcnt - hs_on : hcnt + 9'd320 - hs_on;
wire [7:0] vs_on  = 8'd4 + {{4{v_adj[3]}}, v_adj};
wire [7:0] vs_rel = (vcnt + 8'd1) - vs_on;                               // next line, modulo 256

always @(posedge clk) if (ce_pix) begin
    if (hcnt[8] || vblank) begin
        video_r <= 8'd0; video_g <= 8'd0; video_b <= 8'd0;
    end else begin
        video_r <= lvl(~cval[8:6]);
        video_g <= lvl(~cval[2:0]);
        video_b <= lvl(~cval[5:3]);
    end
    video_hblank <= hcnt[8];
    video_vblank <= vblank;
    video_hs <= hs_rel < 9'd16;
    if (line_start) video_vs <= vs_rel < 8'd3;
end

endmodule

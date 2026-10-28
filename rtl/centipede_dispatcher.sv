//============================================================================
//
//  Centipede dispatcher: shared CPU / sound chip / ROM / RAM pools + the two boards (rtl/boards/)
//  Copyright (C) 2026 Rodimus
//
//  Only one board runs at a time (v_cc = Crystal Castles), so every CPU, sound chip, ROM store and board RAM lives
//  here once and the active board drives it; the other board is held in reset and its pool inputs are ignored.
//  Both boards load the same index-0 windows (program 0x00000, graphics 0x10000 / 0x12000, PROMs 0x14000-0x14200),
//  so each ROM block holds the same bytes whichever game loads.
//
//  Redistribution and use in source and synthesized forms, with or without modification, are permitted
//  provided that the above copyright notice is retained. THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY.
//
//============================================================================

module centipede_dispatcher
(
    input               clk,
    input               reset,
    input               v_cc,           // Crystal Castles board (variants 10 / 11)
    input               pause,
    input         [7:0] variant,

    input         [7:0] in0, in1, in2, in3, dsw1, dsw2,
    input         [7:0] tb1_x, tb1_y,
    input         [7:0] tb2_x, tb2_y,
    input         [3:0] tb_sign,
    input         [7:0] an0, an1, an2, an3,

    input        [24:0] ioctl_addr,
    input         [7:0] ioctl_dout,
    input               ioctl_wr0,

    input               crt_flip,
    input  signed [3:0] h_adj,
    input  signed [3:0] v_adj,

    output              c_ce, c_hs, c_vs, c_hbl, c_vbl,
    output        [7:0] c_r, c_g, c_b,
    output signed [15:0] c_audio,
    output              k_ce, k_hs, k_vs, k_hbl, k_vbl,
    output        [7:0] k_r, k_g, k_b,
    output signed [15:0] k_audio,

    input               nv_sel,
    input         [7:0] nv_addr,
    input         [7:0] nv_din,
    input               nv_we,
    output        [7:0] c_nv_dout,
    output        [7:0] k_nv_dout,

    input         [9:0] hs_addr,
    input         [7:0] hs_din,
    input               hs_we,
    output        [7:0] hs_dout
);

/////////////////////////////  Shared ROM blocks  /////////////////////////////
// port A = the index-0 download, port B = the active board's read

wire [15:0] c_prog_a, k_prog_a;
wire [12:0] c_gfx_a, k_gfx_a;
wire  [7:0] c_sync_a, k_sync_a, c_p1_a, k_p1_a, k_p2_a;
wire  [7:0] prog_q, gfx_lo_q, gfx_hi_q, sync_q, p1_q, p2_q;

dpram_dc #(.widthad_a(16)) rom_prog
(
    .clock_a(clk), .address_a(ioctl_addr[15:0]), .data_a(ioctl_dout), .wren_a(ioctl_wr0 && ioctl_addr[24:16] == 9'd0), .q_a(),
    .clock_b(clk), .address_b(v_cc ? k_prog_a : c_prog_a), .data_b(8'd0), .wren_b(1'b0), .q_b(prog_q)
);
dpram_dc #(.widthad_a(13)) rom_gfx_lo
(
    .clock_a(clk), .address_a(ioctl_addr[12:0]), .data_a(ioctl_dout), .wren_a(ioctl_wr0 && ioctl_addr[24:13] == 12'h008), .q_a(),
    .clock_b(clk), .address_b(v_cc ? k_gfx_a : c_gfx_a), .data_b(8'd0), .wren_b(1'b0), .q_b(gfx_lo_q)
);
dpram_dc #(.widthad_a(13)) rom_gfx_hi
(
    .clock_a(clk), .address_a(ioctl_addr[12:0]), .data_a(ioctl_dout), .wren_a(ioctl_wr0 && ioctl_addr[24:13] == 12'h009), .q_a(),
    .clock_b(clk), .address_b(v_cc ? k_gfx_a : c_gfx_a), .data_b(8'd0), .wren_b(1'b0), .q_b(gfx_hi_q)
);
dpram_dc #(.widthad_a(8)) rom_sync           // sync PROM, 0x14000
(
    .clock_a(clk), .address_a(ioctl_addr[7:0]), .data_a(ioctl_dout), .wren_a(ioctl_wr0 && ioctl_addr[24:8] == 17'h00140), .q_a(),
    .clock_b(clk), .address_b(v_cc ? k_sync_a : c_sync_a), .data_b(8'd0), .wren_b(1'b0), .q_b(sync_q)
);
dpram_dc #(.widthad_a(8)) rom_p1             // colour PROM / write-protect PROM, 0x14100
(
    .clock_a(clk), .address_a(ioctl_addr[7:0]), .data_a(ioctl_dout), .wren_a(ioctl_wr0 && ioctl_addr[24:8] == 17'h00141), .q_a(),
    .clock_b(clk), .address_b(v_cc ? k_p1_a : c_p1_a), .data_b(8'd0), .wren_b(1'b0), .q_b(p1_q)
);
dpram_dc #(.widthad_a(8)) rom_p2             // priority PROM (Crystal Castles), 0x14200
(
    .clock_a(clk), .address_a(ioctl_addr[7:0]), .data_a(ioctl_dout), .wren_a(ioctl_wr0 && ioctl_addr[24:8] == 17'h00142), .q_a(),
    .clock_b(clk), .address_b(k_p2_a), .data_b(8'd0), .wren_b(1'b0), .q_b(p2_q)
);

/////////////////////////////  Chip pool  /////////////////////////////
// t65_0 = main 6502 (both boards); pokey_0 = Centipede POKEY 1 (POT_CMP 0F), pokey_1 = Centipede POKEY 2 | Crystal
// Castles POKEY 1, pokey_2 = Crystal Castles POKEY 2; s2650_0, ay_0, sn_0, eeprom_0 = Centipede board only

chip_pool_pkg::t65_i_t          c_t65_i, k_t65_i, t65_0_i;
chip_pool_pkg::t65_o_t          t65_0_o;
chip_pool_pkg::pokey_i_t  [1:0] c_pokey_i, k_pokey_i;
chip_pool_pkg::pokey_i_t  [2:0] pokey_i;
chip_pool_pkg::pokey_o_t  [2:0] pokey_o;
chip_pool_pkg::s2650_i_t        s2650_0_i;
chip_pool_pkg::s2650_o_t        s2650_0_o;
chip_pool_pkg::ayr_i_t          ay_0_i;
chip_pool_pkg::ay_o_t           ay_0_o;
chip_pool_pkg::sn76489_i_t      sn_0_i;
chip_pool_pkg::sn76489_o_t      sn_0_o;
chip_pool_pkg::eeprom93c46_i_t  eeprom_0_i;
chip_pool_pkg::eeprom93c46_o_t  eeprom_0_o;

assign t65_0_i    = v_cc ? k_t65_i : c_t65_i;
assign pokey_i[0] = c_pokey_i[0];
assign pokey_i[1] = v_cc ? k_pokey_i[0] : c_pokey_i[1];
assign pokey_i[2] = k_pokey_i[1];

wire [23:0] t65_a;
assign t65_0_o.a = t65_a[15:0];
T65 t65_0
(
    .Mode(2'b00), .Res_n(t65_0_i.res_n), .Enable(t65_0_i.enable), .Clk(clk), .Rdy(t65_0_i.rdy), .Abort_n(1'b1),
    .IRQ_n(t65_0_i.irq_n), .NMI_n(t65_0_i.nmi_n), .SO_n(1'b1), .R_W_n(t65_0_o.rw), .A(t65_a), .DI(t65_0_i.di),
    .DO(t65_0_o.dout)
);

genvar pk;
generate
    for (pk = 0; pk < 3; pk = pk + 1) begin : g_pokey
        pokey_glue #(.POT_CMP(pk == 0 ? 8'h0F : 8'h00)) pokey
        (
            .clk(clk), .ce(pokey_i[pk].ce), .enable_179(1'b1), .addr(pokey_i[pk].addr), .data_in(pokey_i[pk].din),
            .wr_en(pokey_i[pk].wr_en), .reset_n(pokey_i[pk].reset_n),
            .keyboard_scan_enable(1'b0), .keyboard_scan(), .keyboard_response(2'b11),
            .pot_in(pokey_i[pk].pot_in), .sio_in1(1'b1), .sio_in2(1'b1), .sio_in3(1'b1),
            .data_out(pokey_o[pk].dout),
            .channel_0_out(pokey_o[pk].ch0), .channel_1_out(pokey_o[pk].ch1),
            .channel_2_out(pokey_o[pk].ch2), .channel_3_out(pokey_o[pk].ch3),
            .irq_n_out(), .sio_out1(), .sio_out2(), .sio_out3(),
            .sio_clockin_in(1'b1), .sio_clockin_out(), .sio_clockin_oe(), .sio_clockout(), .pot_reset(),
            .pot_count(pokey_o[pk].pot_count)
        );
    end
endgenerate

s2650_cpu s2650_0
(
    .clk(clk), .reset(s2650_0_i.reset), .req(s2650_0_o.req), .ack(s2650_0_i.ack), .ad(s2650_0_o.ad), .wr(s2650_0_o.wr),
    .dw(s2650_0_o.dw), .dr(s2650_0_i.dr), .mio(s2650_0_o.mio), .ene(s2650_0_o.ene), .dc(s2650_0_o.dc), .ph(s2650_0_o.ph),
    .irq(s2650_0_i.irq), .intack(s2650_0_o.intack), .ivec(8'h00), .sense(s2650_0_i.sense), .flag()
);

jt49_bus #(.COMP(3'b010)) ay_0
(
    .rst_n(ay_0_i.rst_n), .clk(clk), .clk_en(ay_0_i.ce), .bdir(ay_0_i.bdir), .bc1(ay_0_i.bc1), .din(ay_0_i.din), .sel(1'b1),
    .dout(ay_0_o.dout), .sound(ay_0_o.sound), .A(ay_0_o.a), .B(ay_0_o.b), .C(ay_0_o.c), .sample(),
    .IOA_in(ay_0_i.ioa_in), .IOA_out(), .IOB_in(ay_0_i.iob_in), .IOB_out(ay_0_o.iob_out)
);

jt89 sn_0
(
    .rst(sn_0_i.rst), .clk(clk), .clk_en(sn_0_i.ce), .wr_n(sn_0_i.wr_n), .cs_n(sn_0_i.cs_n), .din(sn_0_i.din),
    .sound(sn_0_o.sound), .ready(sn_0_o.ready)
);

eeprom_93c46 eeprom_0
(
    .clk(clk), .reset(eeprom_0_i.reset), .clear(eeprom_0_i.clear),
    .wr(eeprom_0_i.wr), .di(eeprom_0_i.di), .sk(eeprom_0_i.sk), .cs(eeprom_0_i.cs), .dout(eeprom_0_o.dout),
    .ext_sel(eeprom_0_i.ext_sel), .ext_addr(eeprom_0_i.ext_addr), .ext_din(eeprom_0_i.ext_din), .ext_we(eeprom_0_i.ext_we),
    .ext_q(eeprom_0_o.ext_q)
);

/////////////////////////////  Board RAM pool  /////////////////////////////
// block      Centipede board              Crystal Castles board
//   0  16K   work RAM 1K (port B hiscore)  bitmap even
//   1  16K   playfield 1K                  bitmap odd
//   2   4K   line buffer 256               sprite RAM
//   3  512   EAROM 64 (port B NVRAM)       line buffer
//   4  256   -                             NVRAM
//   5  256   -                             NVRAM shadow

ram_pool_pkg::pram_i_t [3:0] c_pr_i;
ram_pool_pkg::pram_i_t [5:0] k_pr_i, pool_i;
ram_pool_pkg::pram_o_t [5:0] pool_o;
assign pool_i = v_cc ? k_pr_i : {96'd0, c_pr_i};      // blocks 4-5 idle on the Centipede board (2 x 48-bit pram_i_t)

dpram_dc #(.widthad_a(14)) ram_0
(
    .clock_a(clk), .wren_a(pool_i[0].a_we), .address_a(pool_i[0].a_addr[13:0]), .data_a(pool_i[0].a_d), .q_a(pool_o[0].a_q),
    .clock_b(clk), .wren_b(pool_i[0].b_we), .address_b(pool_i[0].b_addr[13:0]), .data_b(pool_i[0].b_d), .q_b(pool_o[0].b_q)
);
dpram_dc #(.widthad_a(14)) ram_1
(
    .clock_a(clk), .wren_a(pool_i[1].a_we), .address_a(pool_i[1].a_addr[13:0]), .data_a(pool_i[1].a_d), .q_a(pool_o[1].a_q),
    .clock_b(clk), .wren_b(pool_i[1].b_we), .address_b(pool_i[1].b_addr[13:0]), .data_b(pool_i[1].b_d), .q_b(pool_o[1].b_q)
);
dpram_dc #(.widthad_a(12)) ram_2
(
    .clock_a(clk), .wren_a(pool_i[2].a_we), .address_a(pool_i[2].a_addr[11:0]), .data_a(pool_i[2].a_d), .q_a(pool_o[2].a_q),
    .clock_b(clk), .wren_b(pool_i[2].b_we), .address_b(pool_i[2].b_addr[11:0]), .data_b(pool_i[2].b_d), .q_b(pool_o[2].b_q)
);
dpram_dc #(.widthad_a(9)) ram_3
(
    .clock_a(clk), .wren_a(pool_i[3].a_we), .address_a(pool_i[3].a_addr[8:0]), .data_a(pool_i[3].a_d), .q_a(pool_o[3].a_q),
    .clock_b(clk), .wren_b(pool_i[3].b_we), .address_b(pool_i[3].b_addr[8:0]), .data_b(pool_i[3].b_d), .q_b(pool_o[3].b_q)
);
dpram_dc #(.widthad_a(8)) ram_4
(
    .clock_a(clk), .wren_a(pool_i[4].a_we), .address_a(pool_i[4].a_addr[7:0]), .data_a(pool_i[4].a_d), .q_a(pool_o[4].a_q),
    .clock_b(clk), .wren_b(pool_i[4].b_we), .address_b(pool_i[4].b_addr[7:0]), .data_b(pool_i[4].b_d), .q_b(pool_o[4].b_q)
);
dpram_dc #(.widthad_a(8)) ram_5
(
    .clock_a(clk), .wren_a(pool_i[5].a_we), .address_a(pool_i[5].a_addr[7:0]), .data_a(pool_i[5].a_d), .q_a(pool_o[5].a_q),
    .clock_b(clk), .wren_b(pool_i[5].b_we), .address_b(pool_i[5].b_addr[7:0]), .data_b(pool_i[5].b_d), .q_b(pool_o[5].b_q)
);

/////////////////////////////  Boards  /////////////////////////////

centiped_board board
(
    .clk(clk), .reset(reset | v_cc), .pause(pause), .variant(variant),
    .in0(in0), .in1(in1), .in2(in2), .in3(in3), .dsw1(dsw1), .dsw2(dsw2),
    .tb1_x(tb1_x), .tb1_y(tb1_y), .tb2_x(tb2_x), .tb2_y(tb2_y), .tb_sign(tb_sign),
    .an0(an0), .an1(an1), .an2(an2), .an3(an3),
    .t65_0_i(c_t65_i), .t65_0_o(t65_0_o), .pokey_i(c_pokey_i), .pokey_o(pokey_o[1:0]),
    .s2650_0_i(s2650_0_i), .s2650_0_o(s2650_0_o), .ay_0_i(ay_0_i), .ay_0_o(ay_0_o),
    .sn_0_i(sn_0_i), .sn_0_o(sn_0_o), .eeprom_0_i(eeprom_0_i), .eeprom_0_o(eeprom_0_o),
    .rom_prog_a(c_prog_a), .rom_prog_q(prog_q), .rom_gfx_a(c_gfx_a), .rom_gfx_lo_q(gfx_lo_q), .rom_gfx_hi_q(gfx_hi_q),
    .rom_sync_a(c_sync_a), .rom_sync_q(sync_q), .rom_p1_a(c_p1_a), .rom_p1_q(p1_q),
    .pr_i(c_pr_i), .pr_o(pool_o[3:0]),
    .crt_flip(crt_flip), .h_adj(h_adj), .v_adj(v_adj),
    .ce_pix(c_ce), .video_r(c_r), .video_g(c_g), .video_b(c_b),
    .video_hs(c_hs), .video_vs(c_vs), .video_hblank(c_hbl), .video_vblank(c_vbl),
    .audio(c_audio),
    .nv_sel(nv_sel), .nv_addr(nv_addr), .nv_din(nv_din), .nv_we(nv_we), .nv_dout(c_nv_dout),
    .hs_addr(hs_addr), .hs_din(hs_din), .hs_we(hs_we), .hs_dout(hs_dout)
);

ccastles_board ccastles
(
    .clk(clk), .reset(reset | ~v_cc), .pause(pause), .joy_ver(variant == 8'd11),
    .in0(in0), .in1(in1), .in2(in2),
    .tb1_x(tb1_x), .tb1_y(tb1_y), .tb2_x(tb2_x), .tb2_y(tb2_y),
    .t65_0_i(k_t65_i), .t65_0_o(t65_0_o), .pokey_i(k_pokey_i), .pokey_o(pokey_o[2:1]),
    .rom_prog_a(k_prog_a), .rom_prog_q(prog_q), .rom_gfx_a(k_gfx_a), .rom_gfx_lo_q(gfx_lo_q), .rom_gfx_hi_q(gfx_hi_q),
    .rom_sync_a(k_sync_a), .rom_sync_q(sync_q), .rom_p1_a(k_p1_a), .rom_p1_q(p1_q), .rom_p2_a(k_p2_a), .rom_p2_q(p2_q),
    .pr_i(k_pr_i), .pr_o(pool_o),
    .crt_flip(crt_flip), .h_adj(h_adj), .v_adj(v_adj),
    .ce_pix(k_ce), .video_r(k_r), .video_g(k_g), .video_b(k_b),
    .video_hs(k_hs), .video_vs(k_vs), .video_hblank(k_hbl), .video_vblank(k_vbl),
    .audio(k_audio),
    .nv_sel(nv_sel), .nv_addr(nv_addr), .nv_din(nv_din), .nv_we(nv_we), .nv_dout(k_nv_dout)
);

endmodule

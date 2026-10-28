//============================================================================
//  board_template.sv - start a new Centipede-core board by copying this file (NOT in files.qip, never compiled).
//  Copyright (C) 2026 Rodimus
//
//  Boards never instantiate a CPU, sound chip, ROM store or board RAM. They live ONCE in the dispatcher
//  rtl/centipede_board.sv: 6502 t65_0, POKEYs pokey_0-2 (POT_CMP is per slot: 0 = 0F, 1-2 = 00), S2650 s2650_0,
//  AY-3-8910 ay_0, SN76489 sn_0, 93C46 eeprom_0 (chip_pool_pkg buses), the shared ROM blocks (index 0: program
//  0x00000, graphics 0x10000 / 0x12000, PROMs 0x14000 / 0x14100 / 0x14200) and the 6 board RAM pool blocks
//  (ram_pool_pkg: 0-1 = 16K, 2 = 4K, 3 = 512, 4-5 = 256).
//  To add a board: copy this file to rtl/boards/<board>/<board>_board.sv, add it to files.qip + verilator/Makefile,
//  instance it in the dispatcher, put its buses on the board muxes (t65_0_i, pokey_i, pool_i, ROM read addresses) and
//  tie the pool blocks it doesn't use to '0. A RAM bigger than its block = grow the block, never a private dpram_dc.
//  ROM images: an index-0 window + a read port (load decode by the download address only, NEVER by variant: the
//  variant byte arrives after the ROMs). A RAM / chip that truly can't pool = "// POOL-EXCEPTION: <reason>" on its
//  instance line + a Pool contract row (Claude/ vault note). verilator/common/pool_check.sh (POOL_MODS) enforces it.
//============================================================================

module board_template
(
    input  logic        clk,            // 48.384 MHz
    input  logic        reset,          // core reset | this board inactive
    input  logic        pause,

    output chip_pool_pkg::t65_i_t          t65_0_i,
    input  chip_pool_pkg::t65_o_t          t65_0_o,
    output chip_pool_pkg::pokey_i_t  [1:0] pokey_i,
    input  chip_pool_pkg::pokey_o_t  [1:0] pokey_o,

    output logic [15:0] rom_prog_a,     // shared ROM blocks (read ports)
    input  logic  [7:0] rom_prog_q,

    output ram_pool_pkg::pram_i_t [5:0] pr_i,
    input  ram_pool_pkg::pram_o_t [5:0] pr_o,

    input  logic  [7:0] in0,

    output logic        ce_pix,
    output logic  [7:0] video_r,
    output logic  [7:0] video_g,
    output logic  [7:0] video_b,
    output logic        video_hs,
    output logic        video_vs,
    output logic        video_hblank,
    output logic        video_vblank,
    output logic signed [15:0] audio
);

endmodule

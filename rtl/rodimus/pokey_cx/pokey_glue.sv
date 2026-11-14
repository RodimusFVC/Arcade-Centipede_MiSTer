// pokey_glue: pokey_cx behind the port list of the Mark Watson `pokey` module
// (Arcade-Centipede_MiSTer rtl/sound/pokey/pokey.v) so the existing board instances
// swap by changing the module name and adding parameters.
//
// Old-port semantics kept:
//   * a write happens on a clk edge with ce=1 and wr_en=1 (old: registers update only
//     under ce); it is applied before that edge's POKEY step (= MAME "after step k").
//   * data_out is combinational from addr (no read strobe in the old port list).
//   * channel_N_out = 4-bit volume of channel N+1 while its output is high (MAME m_out_raw nibbles).
//   * pot_in: old board comparator inputs (centiped_board.sv pot_cmp = pot_count > paddle).
//     pot_count here is the counter value AFTER this edge's step, so the comparator result
//     sampled on the edge matches MAME's latch condition (pot value < counter) on that step.
//     Pots in POT_CMP use it; MAME-exact while the paddle value is stable during a scan and
//     not 0 (warlords paddles are 0x1d-0xcb, centiped.cpp:1514).
//   * reset_n: now synchronous (boards hold cpu_reset_n for many clocks).
// Ignored / constant (not used by any board instance): enable_179 (must be 1), keyboard
// and serial ports.  ALLPOT callbacks: the boards mux the DIP switches over ALLPOT reads
// (centiped_board.sv pokey1_rd/pokey2_rd, ccastles_board.sv pk2_rd), so the glue has no
// ALLPOT_CB; MAME's store of the callback value into ALLPOT on a read (pokey.cpp:944)
// is therefore not reproduced (only visible through POTn reads after an ALLPOT read).
// For full MAME exactness use pokey_cx directly (ALLPOT_CB, rd, allpot_ext).
`default_nettype none
module pokey_glue #(
    parameter [7:0] POT_CMP = 8'h00     // pots driven by the board comparator on pot_in (warlords: 8'h0F)
)(
    input  wire       clk,
    input  wire       ce,
    input  wire       enable_179,
    input  wire [3:0] addr,
    input  wire [7:0] data_in,
    input  wire       wr_en,
    input  wire       reset_n,
    input  wire       keyboard_scan_enable,
    output wire [5:0] keyboard_scan,
    input  wire [1:0] keyboard_response,
    input  wire [7:0] pot_in,
    input  wire       sio_in1,
    input  wire       sio_in2,
    input  wire       sio_in3,
    output wire [7:0] data_out,
    output wire [3:0] channel_0_out,
    output wire [3:0] channel_1_out,
    output wire [3:0] channel_2_out,
    output wire [3:0] channel_3_out,
    output wire       irq_n_out,
    output wire       sio_out1,
    output wire       sio_out2,
    output wire       sio_out3,
    input  wire       sio_clockin_in,
    output wire       sio_clockin_out,
    output wire       sio_clockin_oe,
    output wire       sio_clockout,
    output wire       pot_reset,
    output wire [7:0] pot_count
`ifdef POKEY_DBG
  , output wire [341:0] dbg_state
`endif
);

wire [7:0] cnt_now;
wire [15:0] out_raw_unused;

pokey_cx #(.POT_CB(8'h00), .ALLPOT_CB(1'b0), .POT_EXT(POT_CMP)) core (
    .clk(clk), .reset_n(reset_n), .ce(ce),
    .addr(addr), .din(data_in), .wr(wr_en & ce), .rd(1'b0), .dout(data_out),
    .pot_val(64'd0), .allpot_ext(8'h00),
    .irq_n(irq_n_out), .out_raw(out_raw_unused),
    .channel_0_out(channel_0_out), .channel_1_out(channel_1_out),
    .channel_2_out(channel_2_out), .channel_3_out(channel_3_out),
    .pot_count(cnt_now), .pot_in(pot_in & POT_CMP), .pot_count_step(pot_count)
`ifdef POKEY_DBG
  , .dbg_state(dbg_state)
`endif
);

assign keyboard_scan   = 6'd0;
assign sio_out1        = 1'b1;
assign sio_out2        = 1'b1;
assign sio_out3        = 1'b1;
assign sio_clockin_out = 1'b1;
assign sio_clockin_oe  = 1'b0;
assign sio_clockout    = 1'b0;
assign pot_reset       = (cnt_now == 8'd228);   // scan finished (dump transistors on)

// unused inputs of the old port list
wire unused_ok = &{1'b0, enable_179, keyboard_scan_enable, keyboard_response, sio_in1, sio_in2, sio_in3,
                   sio_clockin_in, out_raw_unused};

endmodule
`default_nettype wire

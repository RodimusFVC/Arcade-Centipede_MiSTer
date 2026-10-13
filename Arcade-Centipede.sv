//============================================================================
//
//  Atari Centipede / Millipede hardware for MiSTer
//  Copyright (C) 2026 Rodimus
//
//  Board model derived from "Atari Centipede, from the schematics" by Brad Parker (lisper)
//  and the MiSTer Centipede port by Alan Steremberg; SNAC trackball wiring from the
//  MiSTer Crystal Castles core by Enceladus
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

module emu
(
    `include "sys/emu_ports.vh"
);

wire        CLK_48M;
wire        locked;
wire [127:0] status;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire [10:0] ps2_key;
wire [24:0] ps2_mouse;
wire        ioctl_download;
wire        ioctl_upload;
wire        ioctl_upload_req;
wire  [7:0] ioctl_din;
wire        ioctl_wr;
wire  [7:0] ioctl_index;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] joystick_0, joystick_1, joystick_2, joystick_3;
wire [15:0] joystick_l_analog_0, joystick_l_analog_1, joystick_l_analog_2, joystick_l_analog_3;   // [15:8] Y, [7:0] X, signed
wire  [7:0] paddle_0, paddle_1, paddle_2, paddle_3;
wire  [8:0] spinner_0, spinner_1, spinner_2, spinner_3;
wire [21:0] gamma_bus;
wire        direct_video;
wire        video_rotated;
wire        pause_cpu;
wire        hblank, vblank;

assign ADC_BUS  = 'Z;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign VGA_F1 = 0;
assign VGA_SCALER = 0;
assign VGA_DISABLE = 0;
assign FB_FORCE_BLANK = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

wire signed [15:0] audio;
assign AUDIO_L = pause_cpu ? 16'd0 : audio;
assign AUDIO_R = pause_cpu ? 16'd0 : audio;
assign AUDIO_S = 1;   // signed
assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign LED_USER  = ioctl_download;
assign BUTTONS = 0;

///////////////////////////////////////////////////

// MRA index 1:
//   byte 0      board variant (see rtl/centiped_board.sv)
//   byte 1      flags: [0] trackball game, [1] paddle game, [2] analog stick game, [3] no joystick inputs,
//               [4] vertical, [7] vertical is ROT90
//   bytes 4-9   idle level of every bit of IN0, IN1, IN2, IN3, DSW1, DSW2
//   bytes 16-63 input map, one byte per port bit (bit 0 first): 0x80 | n = DIP switch bit n,
//               1-63 = control id inverting the idle level, 0 = idle level
reg [7:0] game_var   = 8'd0;
reg [7:0] game_flags = 8'h11;
reg [7:0] port_idle[6] = '{8'hFF,8'hFF,8'hFF,8'hFF,8'hFF,8'hFF};
reg [7:0] in_map[48];

wire [2:0] idle_idx = ioctl_addr[2:0] - 3'd4;      // bytes 4-9  -> 0-5
wire [5:0] map_idx  = ioctl_addr[5:0] - 6'd16;     // bytes 16-63 -> 0-47

always @(posedge CLK_48M) begin
    if (ioctl_wr && ioctl_index == 8'd1) begin
        if (ioctl_addr == 25'd0) game_var   <= ioctl_dout;
        if (ioctl_addr == 25'd1) game_flags <= ioctl_dout;
        if (ioctl_addr >= 25'd4  && ioctl_addr < 25'd10) port_idle[idle_idx] <= ioctl_dout;
        if (ioctl_addr >= 25'd16 && ioctl_addr < 25'd64) in_map[map_idx] <= ioctl_dout;
    end
end

wire tb_game   = game_flags[0];
wire pad_game  = game_flags[1];
wire stk_game  = game_flags[2];
wire game_vert = game_flags[4];
wire vert_view = game_vert & ~status[8];

wire [1:0] ar = status[7:6];

assign VIDEO_ARX = (!ar) ? (vert_view ? 12'd3 : 12'd4) : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? (vert_view ? 12'd4 : 12'd3) : 12'd0;

// Status bits avoid the old Centipede core's 2-5, 11-16, 19-20 and 25-27 so its saved settings can't leak in
`include "build_id.v"
localparam CONF_STR = {
	"CENTIPEDE;;",
	"P1,Video Options;",
	"P1O[7:6],Aspect Ratio,Original,Full screen,[ARC1],[ARC2];",
	"P1O[8],Orientation,Vert,Horz;",
	"P1O[9],HDMI Flip,Off,On;",
	"P1O[10],CRT Flip,Off,On;",
	"P1O[37:34],H Position (CRT),0,+1,+2,+3,+4,+5,+6,+7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[41:38],V Position (CRT),0,+1,+2,+3,+4,+5,+6,+7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[23:21],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
	"P2,Game Options;",
	"H1P2O[29:28],Trackball Speed,Normal,Fast,Slow;",
	"H1P2O[30],Stick/D-pad Drives,Joystick,Trackball;",
	"P2O[31],SNAC Trackball,Off,On;",
	"H2P2O[44:42],Paddle Sensitivity,1.0x,0.75x,0.5x,0.25x,0.125x,1.25x,1.5x,2.0x;",
	"H2P2O[45],Paddle Direction,Normal,Reversed;",
	"H2P2-,Paddle / spinner / mouse / stick;",
	"-;",
	"P3,Pause Options;",
	"P3O[17],Pause when OSD is open,On,Off;",
	"P3O[18],Dim video after 10s,On,Off;",
	"-;",
	"DIP;",
	"-;",
	"R0,Reset;",
	"J1,Btn 1,Btn 2,Btn 3,Btn 4,Coin,Start 1P,Start 2P,Pause;",
	"jn,A,B,X,Y,Select,Start,R,L;",
	"V,v",`BUILD_DATE
};

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(CLK_48M),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),
	.direct_video(direct_video),
	.video_rotated(video_rotated),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({~pad_game, ~tb_game, direct_video}),

	.ioctl_download(ioctl_download),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_din(ioctl_din),
	.ioctl_index(ioctl_index),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_2(joystick_2),
	.joystick_3(joystick_3),
	.joystick_l_analog_0(joystick_l_analog_0),
	.joystick_l_analog_1(joystick_l_analog_1),
	.joystick_l_analog_2(joystick_l_analog_2),
	.joystick_l_analog_3(joystick_l_analog_3),
	.paddle_0(paddle_0), .paddle_1(paddle_1), .paddle_2(paddle_2), .paddle_3(paddle_3),
	.spinner_0(spinner_0), .spinner_1(spinner_1), .spinner_2(spinner_2), .spinner_3(spinner_3),
	.ps2_key(ps2_key),
	.ps2_mouse(ps2_mouse)
);

////////////////////   CLOCKS   ///////////////////

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(CLK_48M),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll),
	.locked(locked)
);

wire [63:0] reconfig_to_pll;
wire [63:0] reconfig_from_pll;
wire        cfg_waitrequest;

pll_cfg pll_cfg
(
	.mgmt_clk(CLK_50M),
	.mgmt_reset(0),
	.mgmt_waitrequest(cfg_waitrequest),
	.mgmt_read(0),
	.mgmt_readdata(),
	.mgmt_write(0),
	.mgmt_address(0),
	.mgmt_writedata(0),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

// Hold the board in reset until the PLL is locked and the ROM download has finished
wire reset = RESET | status[0] | buttons[1] | ioctl_download | ~locked;

///////////////////         Keyboard           //////////////////

reg kb_up = 0, kb_down = 0, kb_left = 0, kb_right = 0;
reg kb_b1 = 0, kb_b2 = 0;
reg kb_coin1 = 0, kb_coin2 = 0, kb_start1 = 0, kb_start2 = 0, kb_pause = 0, kb_service = 0, kb_tilt = 0;

wire       pressed = ~ps2_key[9];
wire [7:0] code    = ps2_key[7:0];

always @(posedge CLK_48M) begin
	reg old_state;
	old_state <= ps2_key[10];
	if (old_state != ps2_key[10]) begin
		case (code)
			'h16: kb_start1  <= pressed; // 1
			'h1E: kb_start2  <= pressed; // 2
			'h2E: kb_coin1   <= pressed; // 5
			'h36: kb_coin2   <= pressed; // 6
			'h46: kb_service <= pressed; // 9
			'h2C: kb_tilt    <= pressed; // T
			'h4D: kb_pause   <= pressed; // P

			'h75: kb_up      <= pressed; // up
			'h72: kb_down    <= pressed; // down
			'h6B: kb_left    <= pressed; // left
			'h74: kb_right   <= pressed; // right
			'h14: kb_b1      <= pressed; // ctrl
			'h11: kb_b2      <= pressed; // alt
		endcase
	end
end

//////////////////  Arcade Buttons/Interfaces   ///////////////////////////

// SNAC trackball (Crystal Castles core wiring): USER_IN 0 fire / start, 1 coin (both to ground),
// 2/3 vertical and 4/5 horizontal quadrature A/B
wire snac_en = status[31];
assign USER_OUT = snac_en ? 7'b0111111 : 7'b1111111;
reg  [6:0] user_s1 = 7'h7F, user_s = 7'h7F;
always @(posedge CLK_48M) begin user_s1 <= USER_IN; user_s <= user_s1; end
wire snac_fire = snac_en & ~user_s[0];
wire snac_coin = snac_en & ~user_s[1];

// stick / d-pad drive the joystick bits, or the trackball when "Stick/D-pad Drives" is Trackball or the game
// has no joystick (Bulls Eye Darts)
wire stick_tb = (status[30] | game_flags[3]) & tb_game;

// Joystick bits: 0 R, 1 L, 2 D, 3 U, 4-7 Btn 1-4, 8 Coin, 9 Start 1P, 10 Start 2P, 11 Pause
wire [3:0] dir1_raw = {joystick_0[3] | kb_up, joystick_0[2] | kb_down, joystick_0[1] | kb_left, joystick_0[0] | kb_right};
wire [3:0] dir2_raw = joystick_1[3:0];
wire [3:0] dir1 = stick_tb ? 4'd0 : dir1_raw;      // {U, D, L, R}
wire [3:0] dir2 = stick_tb ? 4'd0 : dir2_raw;

wire m_pause = joystick_0[11] | kb_pause;

// Trackball: fine counters from mouse, SNAC quadrature edges, and (Trackball mode) left stick / d-pad once a frame.
// Speed: the board sees the counter >> 1 (Normal, MAME's 50%), >> 0 (Fast) or >> 2 (Slow).
// MAME: player 1 X is reversed; player 2 (cocktail) Y is reversed.
reg  [9:0] tb1x = 10'd0, tb1y = 10'd0, tb2x = 10'd0, tb2y = 10'd0;
reg        mouse_tog = 1'b0, tb_vbl = 1'b0;
reg  [2:0] qa_h = 3'd0, qb_h = 3'd0, qa_v = 3'd0, qb_v = 3'd0;

function automatic [9:0] stick_d(input [7:0] a, input pos, input neg);
	stick_d = ($signed(a) > 8'sd12 || $signed(a) < -8'sd12) ? {{5{a[7]}}, a[7:3]} : pos ? 10'd6 : neg ? -10'd6 : 10'd0;
endfunction

wire [9:0] mouse_dx = {{2{ps2_mouse[4]}}, ps2_mouse[15:8]};
wire [9:0] mouse_dy = {{2{ps2_mouse[5]}}, ps2_mouse[23:16]};     // PS/2 Y is positive upward
wire [9:0] s1x = stick_d(joystick_l_analog_0[7:0],  dir1_raw[0], dir1_raw[1]);
wire [9:0] s1y = stick_d(joystick_l_analog_0[15:8], dir1_raw[2], dir1_raw[3]);
wire [9:0] s2x = stick_d(joystick_l_analog_1[7:0],  dir2_raw[0], dir2_raw[1]);
wire [9:0] s2y = stick_d(joystick_l_analog_1[15:8], dir2_raw[2], dir2_raw[3]);

// SNAC quadrature (both edges of A and B, as the Crystal Castles core's LETA decoder)
wire q_h_ce  = qa_h[1] ^ qa_h[2] ^ qb_h[1] ^ qb_h[2];
wire q_h_dir = qa_h[1] ^ qb_h[2];
wire q_v_ce  = qa_v[1] ^ qa_v[2] ^ qb_v[1] ^ qb_v[2];
wire q_v_dir = qa_v[1] ^ qb_v[2];

always @(posedge CLK_48M) begin
	mouse_tog <= ps2_mouse[24];
	tb_vbl    <= vblank;
	qa_v <= {qa_v[1:0], user_s[2]}; qb_v <= {qb_v[1:0], user_s[3]};
	qa_h <= {qa_h[1:0], user_s[4]}; qb_h <= {qb_h[1:0], user_s[5]};
	if (mouse_tog != ps2_mouse[24]) begin
		tb1x <= tb1x - mouse_dx;
		tb1y <= tb1y - mouse_dy;
	end
	else if (snac_en && (q_h_ce || q_v_ce)) begin
		if (q_h_ce) tb1x <= q_h_dir ? tb1x + 10'd1 : tb1x - 10'd1;
		if (q_v_ce) tb1y <= q_v_dir ? tb1y + 10'd1 : tb1y - 10'd1;
	end
	else if (stick_tb && vblank && !tb_vbl) begin
		tb1x <= tb1x - s1x;
		tb1y <= tb1y + s1y;
		tb2x <= tb2x + s2x;
		tb2y <= tb2y - s2y;
	end
end

function automatic [7:0] tb_scale(input [9:0] c, input [1:0] spd);
	tb_scale = (spd == 2'd1) ? c[7:0] : (spd == 2'd2) ? c[9:2] : c[8:1];
endfunction
wire [7:0] tb1x_o = tb_scale(tb1x, status[29:28]), tb1y_o = tb_scale(tb1y, status[29:28]);
wire [7:0] tb2x_o = tb_scale(tb2x, status[29:28]), tb2y_o = tb_scale(tb2y, status[29:28]);

// direction latched on every count (Atari trackball DIR flip-flop / MAME read_trackball sign)
reg  [7:0] tb1x_d = 0, tb1y_d = 0, tb2x_d = 0, tb2y_d = 0;
reg  [3:0] tb_sign = 4'd0;
wire [7:0] d1x = tb1x_o - tb1x_d, d1y = tb1y_o - tb1y_d, d2x = tb2x_o - tb2x_d, d2y = tb2y_o - tb2y_d;
always @(posedge CLK_48M) begin
	tb1x_d <= tb1x_o; tb1y_d <= tb1y_o; tb2x_d <= tb2x_o; tb2y_d <= tb2y_o;
	if (d1x != 8'd0) tb_sign[0] <= d1x[7];
	if (d1y != 8'd0) tb_sign[1] <= d1y[7];
	if (d2x != 8'd0) tb_sign[2] <= d2x[7];
	if (d2y != 8'd0) tb_sign[3] <= d2y[7];
end

// control ids used by the MRA input map (tools/gen_mra.py CTL)
wire [63:0] ctl =
{
	36'd0,
	joystick_3[9] | joystick_3[10],                 // 27 start 4
	joystick_2[9] | joystick_2[10],                 // 26 start 3
	joystick_3[4],                                  // 25 P4 Btn 1
	joystick_2[4],                                  // 24 P3 Btn 1
	joystick_2[8] | joystick_3[8],                  // 23 coin 3
	kb_service,                                     // 22 service
	kb_tilt,                                        // 21 tilt
	joystick_0[10] | joystick_1[10] | kb_start2,    // 20 start 2
	joystick_0[9]  | joystick_1[9]  | kb_start1,    // 19 start 1
	joystick_1[8] | kb_coin2,                       // 18 coin 2
	joystick_0[8] | kb_coin1 | snac_coin,           // 17 coin 1
	joystick_1[7:4],                                // 16-13 P2 Btn 4-1
	dir2[0], dir2[1], dir2[2], dir2[3],             // 12 R, 11 L, 10 D, 9 U
	joystick_0[7:6], joystick_0[5] | kb_b2, joystick_0[4] | kb_b1 | ps2_mouse[0] | snac_fire,   // 8-5 P1 Btn 4-1
	dir1[0], dir1[1], dir1[2], dir1[3],             // 4 R, 3 L, 2 D, 1 U
	1'b0                                            // 0 none
};

// DIP switches arrive from the OSD via ioctl index 254 (3 bytes of packed DIP fields)
reg [7:0] dip_sw[8] = '{8'hFF,8'hFF,8'hFF,8'hFF,8'h00,8'h00,8'h00,8'h00};
always @(posedge CLK_48M) begin
	if (ioctl_wr && (ioctl_index == 8'd254) && !ioctl_addr[24:3])
		dip_sw[ioctl_addr[2:0]] <= ioctl_dout;
end
wire [23:0] dips = {dip_sw[2], dip_sw[1], dip_sw[0]};

reg [7:0] in_port[6];
always @(posedge CLK_48M) begin
	for (int p = 0; p < 6; p++)
		for (int b = 0; b < 8; b++)
			in_port[p][b] <= in_map[p*8 + b][7] ? dips[in_map[p*8 + b][4:0]]
			                                    : port_idle[p][b] ^ ctl[in_map[p*8 + b][5:0]];
end

// Warlords paddles P1-P4 (P1 also takes the mouse)
wire [7:0] pad_pos[4];
paddle_ctl pad1(.clk(CLK_48M), .reset(reset), .vblank(vblank), .sensitivity(status[44:42]), .reverse(status[45]),
	.paddle(paddle_0), .spinner(spinner_0), .use_mouse(1'b1), .mouse(ps2_mouse), .stick_x(joystick_l_analog_0[7:0]),
	.left(joystick_0[1] | kb_left), .right(joystick_0[0] | kb_right), .pos(pad_pos[0]));
paddle_ctl pad2(.clk(CLK_48M), .reset(reset), .vblank(vblank), .sensitivity(status[44:42]), .reverse(status[45]),
	.paddle(paddle_1), .spinner(spinner_1), .use_mouse(1'b0), .mouse(ps2_mouse), .stick_x(joystick_l_analog_1[7:0]),
	.left(joystick_1[1]), .right(joystick_1[0]), .pos(pad_pos[1]));
paddle_ctl pad3(.clk(CLK_48M), .reset(reset), .vblank(vblank), .sensitivity(status[44:42]), .reverse(status[45]),
	.paddle(paddle_2), .spinner(spinner_2), .use_mouse(1'b0), .mouse(ps2_mouse), .stick_x(joystick_l_analog_2[7:0]),
	.left(joystick_2[1]), .right(joystick_2[0]), .pos(pad_pos[2]));
paddle_ctl pad4(.clk(CLK_48M), .reset(reset), .vblank(vblank), .sensitivity(status[44:42]), .reverse(status[45]),
	.paddle(paddle_3), .spinner(spinner_3), .use_mouse(1'b0), .mouse(ps2_mouse), .stick_x(joystick_l_analog_3[7:0]),
	.left(joystick_3[1]), .right(joystick_3[0]), .pos(pad_pos[3]));

// Maze Invaders analog sticks (MAME STICK0-3, 40-BF, centre 7F): P1 Y reversed, P2 Y, P1 X, P2 X reversed.
// Left stick, or the D-pad at full deflection.
function automatic [7:0] stick_val(input [7:0] a, input pos, input neg);
	reg [8:0] v;
	begin
		v = 9'h07F + {{2{a[7]}}, a[7:1]};
		if (pos) v = 9'h0BF;
		else if (neg) v = 9'h040;
		stick_val = (v < 9'h040) ? 8'h40 : (v > 9'h0BF) ? 8'hBF : v[7:0];
	end
endfunction
wire [7:0] stk_p1y = stick_val(joystick_l_analog_0[15:8], dir1_raw[2], dir1_raw[3]);
wire [7:0] stk_p2y = stick_val(joystick_l_analog_1[15:8], dir2_raw[2], dir2_raw[3]);
wire [7:0] stk_p1x = stick_val(joystick_l_analog_0[7:0],  dir1_raw[0], dir1_raw[1]);
wire [7:0] stk_p2x = stick_val(joystick_l_analog_1[7:0],  dir2_raw[0], dir2_raw[1]);

wire [7:0] an0 = stk_game ? 8'hFF - stk_p1y : pad_pos[0];
wire [7:0] an1 = stk_game ? stk_p2y         : pad_pos[1];
wire [7:0] an2 = stk_game ? stk_p1x         : pad_pos[2];
wire [7:0] an3 = stk_game ? 8'hFF - stk_p2x : pad_pos[3];

// PAUSE SYSTEM
wire [23:0] rgb_out;
wire  [7:0] r, g, b;
pause #(8,8,8,48) pause
(
	.*,
	.clk_sys(CLK_48M),
	.user_button(m_pause),
	.pause_request(1'b0),
	.options(~status[18:17])
);

///////////////                 Video                  ////////////////

wire hs, vs;
wire ce_pix;

wire rotate_ccw = ~game_flags[7];  // ROT270 sets rotate CCW, ROT90 sets CW
wire no_rotate  = ~game_vert | status[8] | direct_video;
wire flip       = status[9];
screen_rotate screen_rotate(.*);

arcade_video #(256,24) arcade_video
(
	.*,

	.clk_video(CLK_48M),

	.RGB_in(rgb_out),
	.HBlank(hblank),
	.VBlank(vblank),
	.HSync(hs),
	.VSync(vs),

	.fx(status[23:21])
);

///////////////                 Board                  ////////////////

centiped_board board
(
	.clk(CLK_48M),
	.reset(reset),
	.pause(pause_cpu),
	.variant(game_var),

	.in0(in_port[0]),
	.in1(in_port[1]),
	.in2(in_port[2]),
	.in3(in_port[3]),
	.dsw1(in_port[4]),
	.dsw2(in_port[5]),
	.tb1_x(tb1x_o), .tb1_y(tb1y_o),
	.tb2_x(tb2x_o), .tb2_y(tb2y_o),
	.tb_sign(tb_sign),
	.an0(an0), .an1(an1), .an2(an2), .an3(an3),

	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wr0(ioctl_wr & (ioctl_index == 8'd0)),

	.crt_flip(status[10]),
	.h_adj(status[37:34]),
	.v_adj(status[41:38]),

	.ce_pix(ce_pix),
	.video_r(r),
	.video_g(g),
	.video_b(b),
	.video_hs(hs),
	.video_vs(vs),
	.video_hblank(hblank),
	.video_vblank(vblank),

	.audio(audio),

	.earom_hs_addr(6'd0),
	.earom_hs_din(8'd0),
	.earom_hs_dout(),
	.earom_hs_we(1'b0)
);

endmodule

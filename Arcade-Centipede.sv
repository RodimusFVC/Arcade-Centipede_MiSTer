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
wire [26:0] ioctl_addr27;                     // hps_io's full address: the artwork download can pass 32 MB
wire [24:0] ioctl_addr = ioctl_addr27[24:0];  // every other ioctl user
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
//   byte 0      board variant (see rtl/boards/centiped/centiped_board.sv)
//   byte 2      saves: 0 none, 1 hiscore.dat (index 3 config), 2 the board's EAROM / 93C46 / X2212 (index 4 = chip image)
//   byte 1      flags: [0] trackball game, [1] paddle game, [2] analog stick game, [3] no joystick inputs,
//               [4] vertical, [7] vertical is ROT90
//   bytes 4-9   idle level of every bit of IN0, IN1, IN2, IN3, DSW1, DSW2
//   bytes 16-63 input map, one byte per port bit (bit 0 first): 0x80 | n = DIP switch bit n,
//               1-63 = control id inverting the idle level, 0 = idle level
reg [7:0] game_var   = 8'd0;
reg [7:0] game_flags = 8'h11;
reg [1:0] save_mode  = 2'd0;       // 0 none, 1 hiscore.dat RAM ranges (index 3), 2 board save chip (EAROM / EEPROM / X2212)
reg [7:0] port_idle[6] = '{8'hFF,8'hFF,8'hFF,8'hFF,8'hFF,8'hFF};
reg [7:0] in_map[48];

wire [2:0] idle_idx = ioctl_addr[2:0] - 3'd4;      // bytes 4-9  -> 0-5
wire [5:0] map_idx  = ioctl_addr[5:0] - 6'd16;     // bytes 16-63 -> 0-47

always @(posedge CLK_48M) begin
    if (ioctl_wr && ioctl_index == 8'd1) begin
        if (ioctl_addr == 25'd0) game_var   <= ioctl_dout;
        if (ioctl_addr == 25'd1) game_flags <= ioctl_dout;
        if (ioctl_addr == 25'd2) save_mode  <= ioctl_dout[1:0];
        if (ioctl_addr >= 25'd4  && ioctl_addr < 25'd10) port_idle[idle_idx] <= ioctl_dout;
        if (ioctl_addr >= 25'd16 && ioctl_addr < 25'd64) in_map[map_idx] <= ioctl_dout;
    end
end

wire v_cc      = game_var == 8'd10 || game_var == 8'd11;   // Crystal Castles board
wire tb_game   = game_flags[0];
wire pad_game  = game_flags[1];
wire stk_game  = game_flags[2];
wire game_vert = game_flags[4];
wire vert_view = game_vert & ~status[8];

wire [1:0] ar = status[7:6];

wire        art_fb;                // artwork framebuffer on HDMI: square canvas pixels
wire        art_wait, art_avail;
wire [11:0] art_cw, art_ch;
assign VIDEO_ARX = (!ar) ? (art_fb ? art_cw : vert_view ? 12'd3 : 12'd4) : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? (art_fb ? art_ch : vert_view ? 12'd4 : 12'd3) : 12'd0;

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
	"H3P2-;",
	"H3P2-,Artwork (MAME layout views);",
	"H3P2O[49:47],View,Off,View 1,View 2,View 3,View 4,View 5,View 6,View 7;",
	"H3P2O[50],Backdrop,On,Off;",
	"H3P2O[51],Overlay,On,Off;",
	"H3P2O[52],Bezel,On,Off;",
	"H3P2O[55],Control Panel,On,Off;",
	"H3P2O[54],Side Art,On,Off;",
	"H3P2O[53],Zoom to Screen,Off,On;",
	"-;",
	"P3,Pause Options;",
	"P3O[17],Pause when OSD is open,On,Off;",
	"P3O[18],Dim video after 10s,On,Off;",
	"-;",
	"P4,High Score Options;",
	"P4O[24],Autosave Hiscores,Off,On;",
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
	.status_menumask({~art_avail, ~pad_game, ~tb_game, direct_video}),

	.ioctl_download(ioctl_download),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr27),
	.ioctl_dout(ioctl_dout),
	.ioctl_din(ioctl_din),
	.ioctl_index(ioctl_index),
	.ioctl_wait(art_wait),

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
	.mgmt_write(cfg_write),
	.mgmt_address(cfg_address),
	.mgmt_writedata(cfg_data),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

// Crystal Castles runs at 50 MHz (10 MHz x 5), the Centipede boards at 48.384 MHz (12.096 MHz x 4). The PLL's M
// counter (register 4) and fractional K (register 7) are rewritten, C stays 9: M 9 / K 1 = 50 MHz exactly, M 8 /
// K 3045647209 = 48.384 MHz. K must stay inside 0.05-0.95 or be ~0 (Main_MiSTer's PLL encoding;
// K FFFFFFFF breaks the picture). Written after the ROM download, so the download never runs through
// a relock (JunoFirst / Tutankham sequence).
reg         cfg_write = 1'b0;
reg   [5:0] cfg_address = 6'd0;
reg  [31:0] cfg_data = 32'd0;
always @(posedge CLK_50M) begin
	reg [1:0] want_s = 2'b00, dl_s = 2'b00;
	reg       want_cc = 1'b0, have_cc = 1'b0;
	reg [2:0] state = 3'd0;

	want_s <= {want_s[0], v_cc};
	dl_s   <= {dl_s[0], ioctl_download};
	want_cc <= want_s[1];

	cfg_write <= 1'b0;
	if (!dl_s[1] && state == 3'd0 && want_cc != have_cc) begin
		state   <= 3'd1;
		have_cc <= want_cc;
	end

	if (!cfg_waitrequest) begin
		if (state) state <= state + 3'd1;
		case (state)
			1: begin cfg_address <= 6'd0; cfg_data <= 32'd0; cfg_write <= 1'b1; end
			3: begin cfg_address <= 6'd4; cfg_data <= have_cc ? 32'h00020504 : 32'h00000404; cfg_write <= 1'b1; end
			5: begin cfg_address <= 6'd7; cfg_data <= have_cc ? 32'd1 : 32'd3045647209; cfg_write <= 1'b1; end
			7: begin cfg_address <= 6'd2; cfg_data <= 32'd0; cfg_write <= 1'b1; end
			default: ;
		endcase
	end
end

// Hold the board in reset until the PLL is locked and the ROM download has finished
wire reset = RESET | status[0] | buttons[1] | ioctl_download | ~locked;

///////////////////         Keyboard           //////////////////

wire [12:0] kb;
/* verilator lint_off UNUSED */
wire kbx_none;                                     // v2 extra-key output: no extra keys here
/* verilator lint_on UNUSED */
// kb: [0] up [1] down [2] left [3] right [4] ctrl [5] alt [6] 1 [7] 2 [8] 5 [9] 6 [10] 9 [11] T [12] P
ctl_keyboard u_kb(.clk(CLK_48M), .ps2_key(ps2_key), .pause(pause_cpu), .kb(kb), .kbx(kbx_none));
wire kb_up    = kb[0];
wire kb_down  = kb[1];
wire kb_left  = kb[2];
wire kb_right = kb[3];
wire kb_pause = kb[12];

//////////////////  Arcade Buttons/Interfaces   ///////////////////////////

// SNAC trackball (Crystal Castles core wiring): USER_IN 0 fire / start, 1 coin (both to ground),
// 2/3 vertical and 4/5 horizontal quadrature A/B
wire snac_en = status[31];
assign USER_OUT = snac_en ? 7'b0111111 : 7'b1111111;
/* verilator lint_off UNUSED */   // user_s[5:2] unused (the SNAC decoders sync USER_IN themselves)
reg  [6:0] user_s1 = 7'h7F, user_s = 7'h7F;
/* verilator lint_on UNUSED */
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

wire       tb_mouse_ev;
wire [9:0] tb_mouse_dx, tb_mouse_dy;
/* verilator lint_off UNUSED */
wire [2:0] tb_mouse_btn;
/* verilator lint_on UNUSED */
// trackball: 12-bit counters; the board sees counter >> base (Centipede boards 1, Crystal Castles 3), -1 Fast / +1 Slow;
// SNAC and the stick add 2^(base-1) per count; P1 X and P2 Y reversed; DIR latch per axis. Sources on one clock sum.
ctl_mouse u_mouse(.clk(CLK_48M), .ps2_mouse(ps2_mouse), .ev(tb_mouse_ev), .dx(tb_mouse_dx), .dy(tb_mouse_dy), .btn(tb_mouse_btn));
wire frame_tick;
ctl_tick #(.MODE(0)) u_frame_tick(.clk(CLK_48M), .pause(pause_cpu), .vblank(vblank), .period(18'd0), .tick(frame_tick));
wire q_h_ce, q_h_dir, q_v_ce, q_v_dir;
ctl_quad_decoder u_quad_h(.clk(CLK_48M), .a(USER_IN[4]), .b(USER_IN[5]), .ce(q_h_ce), .dir(q_h_dir));
ctl_quad_decoder u_quad_v(.clk(CLK_48M), .a(USER_IN[2]), .b(USER_IN[3]), .ce(q_v_ce), .dir(q_v_dir));
wire [7:0] tb1x_o, tb1y_o, tb2x_o, tb2y_o;
wire [1:0] tb1_sign, tb2_sign;
ctl_trackball #(.CW(12), .OUT_W(8), .MOUSE_SH(0), .SPIN_SH(0), .STICK_RSH(3), .DPAD_STEP(6), .DZ(12), .LIVE_ANY(0)) u_tb1
(
	.clk(CLK_48M), .reset(1'b0), .pause(pause_cpu), .rev(2'b01), .unit_sh(v_cc ? 2'd2 : 2'd0), .base_sh(v_cc ? 4'd3 : 4'd1),
	.speed(status[29:28]), .mouse_ev(tb_mouse_ev), .mouse_dx(tb_mouse_dx), .mouse_dy(tb_mouse_dy), .spinner(9'd0),
	.quad_ce({q_v_ce & snac_en, q_h_ce & snac_en}), .quad_dir({q_v_dir, q_h_dir}),
	.tick(frame_tick), .stick_en(stick_tb), .stick(joystick_l_analog_0), .dpad(dir1_raw),
	.x(tb1x_o), .y(tb1y_o), .dir(tb1_sign)
);
ctl_trackball #(.CW(12), .OUT_W(8), .MOUSE_SH(0), .SPIN_SH(0), .STICK_RSH(3), .DPAD_STEP(6), .DZ(12), .LIVE_ANY(0)) u_tb2
(
	.clk(CLK_48M), .reset(1'b0), .pause(pause_cpu), .rev(2'b10), .unit_sh(v_cc ? 2'd2 : 2'd0), .base_sh(v_cc ? 4'd3 : 4'd1),
	.speed(status[29:28]), .mouse_ev(1'b0), .mouse_dx(10'd0), .mouse_dy(10'd0), .spinner(9'd0),
	.quad_ce(2'b00), .quad_dir(2'b00),
	.tick(frame_tick), .stick_en(stick_tb), .stick(joystick_l_analog_1), .dpad(dir2_raw),
	.x(tb2x_o), .y(tb2y_o), .dir(tb2_sign)
);
wire [3:0] tb_sign = {tb2_sign, tb1_sign};   // [0] P1 X, [1] P1 Y, [2] P2 X, [3] P2 Y

/* verilator lint_off UNUSED */
wire [35:0] ids;
/* verilator lint_on UNUSED */
ctl_ids #(.BTN56(0)) u_ids(.joy0(joystick_0), .joy1(joystick_1), .kb(kb), .dir1(dir1), .dir2(dir2),
	.coin1(joystick_0[8] | kb[8] | snac_coin), .coin2(joystick_1[8] | kb[9]), .coin3(joystick_2[8] | joystick_3[8]), .ctl(ids));
wire [63:0] ctl =
{
	36'd0,
	joystick_3[9] | joystick_3[10],                 // 27 start 4
	joystick_2[9] | joystick_2[10],                 // 26 start 3
	joystick_3[4],                                  // 25 P4 Btn 1
	joystick_2[4],                                  // 24 P3 Btn 1
	ids[23:6],                                      // 23-6 (coin 3 ... P1 Btn 2)
	ids[5] | ps2_mouse[0] | snac_fire,              // 5 P1 Btn 1 (+ mouse left, SNAC fire)
	ids[4:0]                                        // 4-1 P1 R L D U, 0 none
};

// DIP switches arrive from the OSD via ioctl index 254 (3 bytes of packed DIP fields)
reg [7:0] dip_sw[8] = '{8'hFF,8'hFF,8'hFF,8'hFF,8'h00,8'h00,8'h00,8'h00};
always @(posedge CLK_48M) begin
	if (ioctl_wr && (ioctl_index == 8'd254) && !ioctl_addr[24:3])
		dip_sw[ioctl_addr[2:0]] <= ioctl_dout;
end
wire [23:0] dips = {dip_sw[2], dip_sw[1], dip_sw[0]};

wire [47:0]  port_idle_p = {port_idle[5], port_idle[4], port_idle[3], port_idle[2], port_idle[1], port_idle[0]};
wire [383:0] in_map_p;
wire [47:0]  in_ports_p;
genvar gm;
generate
	for (gm = 0; gm < 48; gm = gm + 1) begin : g_in_map
		assign in_map_p[gm*8 +: 8] = in_map[gm];
	end
endgenerate
ctl_port_map #(.NP(6), .IDW(6), .DIP_MODE(1)) u_ports(.clk(CLK_48M), .idle(port_idle_p), .in_map(in_map_p),
	.dips({8'd0, dips}), .port_sel(48'd0), .dip_bytes(64'd0), .dip_bit(144'd0), .ctl(ctl), .ovr_mask(48'd0), .ovr_val(48'd0),
	.ports(in_ports_p));
wire [7:0] in_port[6];
generate
	for (gm = 0; gm < 6; gm = gm + 1) begin : g_in_port
		assign in_port[gm] = in_ports_p[gm*8 +: 8];
	end
endgenerate

wire [7:0] pad_pos[4];
ctl_paddle #(.PAUSE_MODE(1)) u_pad1(.clk(CLK_48M), .reset(reset), .pause(pause_cpu), .tick(frame_tick), .sensitivity(status[44:42]),
	.reverse(status[45]), .paddle(paddle_0), .spinner(spinner_0), .mouse_ev(tb_mouse_ev), .mouse_dx(tb_mouse_dx),
	.stick_x(joystick_l_analog_0[7:0]), .left(joystick_0[1] | kb_left), .right(joystick_0[0] | kb_right), .pos(pad_pos[0]));
ctl_paddle #(.PAUSE_MODE(1)) u_pad2(.clk(CLK_48M), .reset(reset), .pause(pause_cpu), .tick(frame_tick), .sensitivity(status[44:42]),
	.reverse(status[45]), .paddle(paddle_1), .spinner(spinner_1), .mouse_ev(1'b0), .mouse_dx(10'd0),
	.stick_x(joystick_l_analog_1[7:0]), .left(joystick_1[1]), .right(joystick_1[0]), .pos(pad_pos[1]));
ctl_paddle #(.PAUSE_MODE(1)) u_pad3(.clk(CLK_48M), .reset(reset), .pause(pause_cpu), .tick(frame_tick), .sensitivity(status[44:42]),
	.reverse(status[45]), .paddle(paddle_2), .spinner(spinner_2), .mouse_ev(1'b0), .mouse_dx(10'd0),
	.stick_x(joystick_l_analog_2[7:0]), .left(joystick_2[1]), .right(joystick_2[0]), .pos(pad_pos[2]));
ctl_paddle #(.PAUSE_MODE(1)) u_pad4(.clk(CLK_48M), .reset(reset), .pause(pause_cpu), .tick(frame_tick), .sensitivity(status[44:42]),
	.reverse(status[45]), .paddle(paddle_3), .spinner(spinner_3), .mouse_ev(1'b0), .mouse_dx(10'd0),
	.stick_x(joystick_l_analog_3[7:0]), .left(joystick_3[1]), .right(joystick_3[0]), .pos(pad_pos[3]));

// Maze Invaders analog sticks (STICK0-3, 40-BF, centre 7F): P1 Y reversed, P2 Y, P1 X, P2 X reversed.
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

// save system nets (instances after the boards)
wire        nv_dl = ioctl_download && ioctl_index == 8'd4;
wire  [7:0] nv_address, c_nv_q, k_nv_q;
wire  [7:0] nv_din_hps, hs_din_hps;
wire        nv_upload_req, hs_upload_req, nv_pause, hs_pause;
wire  [7:0] nv_port_addr = nv_dl ? ioctl_addr[7:0] : nv_address;
wire        nv_port_we   = nv_dl && ioctl_wr && save_mode == 2'd2 && ioctl_addr[24:8] == 17'd0;
wire        nv_sel       = nv_dl || nv_pause;
wire [15:0] hs_address;
wire  [7:0] hs_data_in;
wire  [7:0] hs_data_out;
wire        hs_write_enable;

// PAUSE SYSTEM
wire [23:0] rgb_out;
wire  [7:0] r, g, b;
pause #(8,8,8,48) pause
(
	.*,
	.clk_sys(CLK_48M),
	.user_button(m_pause),
	.pause_request((save_mode == 2'd1 && hs_pause) || (save_mode == 2'd2 && nv_pause)),
	.options(~status[18:17])
);

///////////////                 Video                  ////////////////

wire hs, vs;
wire ce_pix;
wire        c_ce, c_hs, c_vs, c_hbl, c_vbl, k_ce, k_hs, k_vs, k_hbl, k_vbl;
wire  [7:0] c_r, c_g, c_b, k_r, k_g, k_b;
wire signed [15:0] c_audio, k_audio;
wire        bd_hs, bd_vs, bd_hbl, bd_vbl;
assign {ce_pix, bd_hs, bd_vs, bd_hbl, bd_vbl} = v_cc ? {k_ce, k_hs, k_vs, k_hbl, k_vbl} : {c_ce, c_hs, c_vs, c_hbl, c_vbl};
wire [23:0] bd_rgb = v_cc ? {k_r, k_g, k_b} : {c_r, c_g, c_b};

// MAME artwork (ioctl index 5, OSD Artwork page): native composite on the core video (one pixel late),
// full view through the DDR3 framebuffer on HDMI
wire        art_run, art_ddr;
wire [31:0] art_fb_base;
wire  [7:0] art_burstcnt, art_be;
wire [28:0] art_addr;
wire [63:0] art_din;
wire        art_rd, art_we, art_clk;
artwork #(.IDX(8'd5)) artwork
(
	.clk(CLK_48M),
	.sel({1'b0, status[49:47]}), .layer_off({status[55], status[52:50]}), .side_off(status[54]), .zoom(status[53]),
	.crt_flip(status[10]), .hdmi_flip(status[9]), .turn(status[8]), .avail(art_avail),
	.hdmi_w(HDMI_WIDTH), .hdmi_h(HDMI_HEIGHT),
	.ioctl_download(ioctl_download), .ioctl_index(ioctl_index), .ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr27), .ioctl_dout(ioctl_dout), .ioctl_wait(art_wait),
	.ce_pix(ce_pix),
	.hs_in(bd_hs), .vs_in(bd_vs), .hblank_in(bd_hbl), .vblank_in(bd_vbl), .rgb_in(bd_rgb),
	.hs_out(hs), .vs_out(vs), .hblank_out(hblank), .vblank_out(vblank), .rgb_out({r, g, b}),
	.running(art_run), .ddr_owner(art_ddr), .canvas_w(art_cw), .canvas_h(art_ch),
	.fb_base(art_fb_base), .fb_vbl(FB_VBL),
	.DDRAM_CLK(art_clk), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(art_burstcnt), .DDRAM_ADDR(art_addr),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(art_rd),
	.DDRAM_DIN(art_din), .DDRAM_BE(art_be), .DDRAM_WE(art_we)
);
// Orientation Horz (rotated monitor): the artwork turns its own view, so it keeps the framebuffer
assign art_fb = art_run & ~direct_video;
assign audio = v_cc ? k_audio : c_audio;

wire rotate_ccw = ~game_flags[7];  // ROT270 sets rotate CCW, ROT90 sets CW
wire no_rotate  = ~game_vert | status[8] | direct_video;
wire flip       = status[9];
wire        sr_fb_en, sr_clk, sr_rd, sr_we;
wire  [4:0] sr_fb_format;
wire [11:0] sr_fb_w, sr_fb_h;
wire [31:0] sr_fb_base;
wire [13:0] sr_fb_stride;
wire  [7:0] sr_burstcnt, sr_be;
wire [28:0] sr_addr;
wire [63:0] sr_din;
screen_rotate screen_rotate
(
	.*,
	.FB_EN(sr_fb_en), .FB_FORMAT(sr_fb_format), .FB_WIDTH(sr_fb_w), .FB_HEIGHT(sr_fb_h),
	.FB_BASE(sr_fb_base), .FB_STRIDE(sr_fb_stride),
	.DDRAM_CLK(sr_clk), .DDRAM_BURSTCNT(sr_burstcnt), .DDRAM_ADDR(sr_addr), .DDRAM_DIN(sr_din),
	.DDRAM_BE(sr_be), .DDRAM_WE(sr_we), .DDRAM_RD(sr_rd)
);

// the artwork takes the framebuffer while it draws, and DDR3 while it loads or draws
assign FB_EN     = art_fb | sr_fb_en;
assign FB_FORMAT = art_fb ? 5'b00110  : sr_fb_format;
assign FB_WIDTH  = art_fb ? art_cw    : sr_fb_w;
assign FB_HEIGHT = art_fb ? art_ch    : sr_fb_h;
assign FB_BASE   = art_fb ? art_fb_base : sr_fb_base;
assign FB_STRIDE = art_fb ? 14'd8192  : sr_fb_stride;
assign DDRAM_CLK      = CLK_48M;
assign DDRAM_BURSTCNT = art_ddr ? art_burstcnt : sr_burstcnt;
assign DDRAM_ADDR     = art_ddr ? art_addr : sr_addr;
assign DDRAM_DIN      = art_ddr ? art_din : sr_din;
assign DDRAM_BE       = art_ddr ? art_be : sr_be;
assign DDRAM_WE       = art_ddr ? art_we : sr_we;
assign DDRAM_RD       = art_ddr ? art_rd : sr_rd;

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

centipede_dispatcher boards
(
	.clk(CLK_48M), .reset(reset), .v_cc(v_cc), .pause(pause_cpu), .variant(game_var),
	.in0(in_port[0]), .in1(in_port[1]), .in2(in_port[2]), .in3(in_port[3]), .dsw1(in_port[4]), .dsw2(in_port[5]),
	.tb1_x(tb1x_o), .tb1_y(tb1y_o), .tb2_x(tb2x_o), .tb2_y(tb2y_o), .tb_sign(tb_sign),
	.an0(an0), .an1(an1), .an2(an2), .an3(an3),
	.ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout), .ioctl_wr0(ioctl_wr & (ioctl_index == 8'd0)),
	.crt_flip(status[10]), .h_adj(status[37:34]), .v_adj(status[41:38]),
	.c_ce(c_ce), .c_r(c_r), .c_g(c_g), .c_b(c_b), .c_hs(c_hs), .c_vs(c_vs), .c_hbl(c_hbl), .c_vbl(c_vbl), .c_audio(c_audio),
	.k_ce(k_ce), .k_r(k_r), .k_g(k_g), .k_b(k_b), .k_hs(k_hs), .k_vs(k_vs), .k_hbl(k_hbl), .k_vbl(k_vbl), .k_audio(k_audio),
	.nv_sel(nv_sel), .nv_addr(nv_port_addr), .nv_din(ioctl_dout), .nv_we(nv_port_we), .c_nv_dout(c_nv_q), .k_nv_dout(k_nv_q),
	.hs_addr(hs_address[9:0]), .hs_din(hs_data_in), .hs_we(hs_write_enable & save_mode == 2'd1), .hs_dout(hs_data_out)
);

// Saves. Chip boards: the NVRAM file is the save chip's contents, written into the chip while the MRA loads (the
// board is in reset) and read back by nvram.v; the game restores its own scores as on the real board.
// Magic Worm / Warlords: hiscore.dat ranges in work RAM through hiscore.v. Each is inert outside its mode.

nvram #(
	.DUMPWIDTH(8),
	.DUMPINDEX(4),
	.PAUSEPAD(2)
) nv (
	.clk(CLK_48M),
	.paused(pause_cpu),
	.reset(reset),
	.autosave(status[24]),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(nv_upload_req),
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_index(ioctl_index),
	.ioctl_din(nv_din_hps),
	.ioctl_dout(ioctl_dout),
	.OSD_STATUS(OSD_STATUS),
	.nvram_address(nv_address),
	.nvram_data_out(v_cc ? k_nv_q : c_nv_q),
	.pause_cpu(nv_pause)
);

hiscore #(
	.HS_ADDRESSWIDTH(16),
	.CFG_ADDRESSWIDTH(4),
	.CFG_LENGTHWIDTH(2)
) hi (
	.clk(CLK_48M),
	.paused(pause_cpu),
	.reset(reset),
	.autosave(status[24]),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(hs_upload_req),
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_index(ioctl_index),
	.OSD_STATUS(OSD_STATUS),
	.data_from_hps(ioctl_dout),
	.data_from_ram(hs_data_out),
	.ram_address(hs_address),
	.data_to_hps(hs_din_hps),
	.data_to_ram(hs_data_in),
	.ram_write(hs_write_enable),
	.ram_intent_read(),
	.ram_intent_write(),
	.pause_cpu(hs_pause),
	.configured()
);

assign ioctl_din        = (save_mode == 2'd1) ? hs_din_hps : nv_din_hps;
assign ioctl_upload_req = (save_mode == 2'd1) ? hs_upload_req : (save_mode == 2'd2) ? nv_upload_req : 1'b0;

endmodule

//============================================================================
//
//  Paddle position from MiSTer controllers (Warlords: MAME IPT_PADDLE, MINMAX 1D-CB, sensitivity 50)
//  Copyright (C) 2026 Rodimus
//
//  - paddle  (hps_io paddle_N): absolute, the position follows the knob over the full range
//  - spinner (hps_io spinner_N) and mouse X: relative, half a count per count at 1.0x (MAME sensitivity 50)
//  - left stick X: relative, up to 8 counts per frame; D-pad: 5 counts per frame (MAME key delta 10 at 50 %)
//  Sensitivity and direction follow Pole Position's OSD options (1.0 .75 .5 .25 .125 1.25 1.5 2.0 x).
//
//  Redistribution and use in source and synthesized forms, with or without modification, are permitted
//  provided that the above copyright notice is retained. THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY.
//
//============================================================================

module paddle_ctl
(
    input             clk,
    input             reset,
    input             vblank,
    input       [2:0] sensitivity,
    input             reverse,

    input       [7:0] paddle,
    input       [8:0] spinner,
    input             use_mouse,
    input      [24:0] mouse,
    input       [7:0] stick_x,          // signed
    input             left,
    input             right,

    output      [7:0] pos
);

localparam [10:0] LO = 11'h1D << 3, HI = 11'hCB << 3, MID = 11'h80 << 3;

// value * sensitivity in 1/8 counts
function automatic signed [15:0] scale_q3(input signed [9:0] v, input [2:0] s);
    reg signed [15:0] e;
    begin
        e = {{6{v[9]}}, v};
        case (s)
            3'd0:    scale_q3 = e <<< 3;
            3'd1:    scale_q3 = (e <<< 2) + (e <<< 1);
            3'd2:    scale_q3 = e <<< 2;
            3'd3:    scale_q3 = e <<< 1;
            3'd4:    scale_q3 = e;
            3'd5:    scale_q3 = (e <<< 3) + (e <<< 1);
            3'd6:    scale_q3 = (e <<< 3) + (e <<< 2);
            default: scale_q3 = e <<< 4;
        endcase
    end
endfunction

reg        spin_seen = 1'b0, mouse_seen = 1'b0, vbl_d = 1'b0;
reg  [7:0] pad_d = 8'h80;
reg [10:0] posq = MID;                               // position in 1/8 counts

wire signed [9:0] spin_d  = (spinner[8] != spin_seen) ? $signed({{2{spinner[7]}}, spinner[7:0]}) : 10'sd0;
wire signed [9:0] mouse_d = (use_mouse && mouse[24] != mouse_seen) ? $signed({mouse[4], mouse[4], mouse[15:8]}) : 10'sd0;
wire signed [7:0] sx      = stick_x;
wire signed [9:0] frame_d = (sx > 8'sd12 || sx < -8'sd12) ? (10'(sx) >>> 4) : right ? 10'sd5 : left ? -10'sd5 : 10'sd0;
wire              frame   = vblank & ~vbl_d;

wire signed [15:0] d_q3 = (scale_q3(spin_d, sensitivity) >>> 1) + (scale_q3(mouse_d, sensitivity) >>> 1) +
                          (frame ? scale_q3(frame_d, sensitivity) : 16'sd0);
wire signed [15:0] nxt  = $signed({5'd0, posq}) + (reverse ? -d_q3 : d_q3);

// absolute knob: 00-FF -> 1D-CB
wire [15:0] knob  = {8'd0, reverse ? ~paddle : paddle} * 16'd174;
wire [10:0] abs_q = {(8'h1D + knob[15:8]), 3'd0};

always @(posedge clk) begin
    spin_seen  <= spinner[8];
    mouse_seen <= mouse[24];
    vbl_d      <= vblank;
    pad_d      <= paddle;
    if (reset)                 posq <= MID;
    else if (paddle != pad_d)  posq <= abs_q;
    else if (nxt < $signed({5'd0, LO})) posq <= LO;
    else if (nxt > $signed({5'd0, HI})) posq <= HI;
    else                       posq <= nxt[10:0];
end

assign pos = posq[10:3];

endmodule

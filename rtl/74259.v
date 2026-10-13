module ttl_74259 (
    input  wire       mr_n,  // Master Reset (Clear) - Active Low
    input  wire       le_n,  // Latch Enable - Active Low
    input  wire [2:0] a,     // 3-Bit Address Select (A0-A2)
    input  wire       d,     // Data Input
    output reg  [7:0] q      // 8-Bit Parallel Outputs (Q0-Q7)
);

always @(*) begin
    if (!mr_n) begin
        q = 8'b0;          // Clear mode: all outputs forced low
    } else if (!le_n) begin
        q[a] = d;          // Addressable latch mode: selected bit follows D
    end
    // When both mr_n and le_n are high, it enters "Hold" mode 
    // and retains its previous state natively.
end

endmodule
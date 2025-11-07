// ============================================================
// File: miso_mux_8to1.v
// 8:1 MISO selector for the chosen slave
// ============================================================
`default_nettype none
module miso_mux_8to1(
    input  wire [7:0] slave_miso_lines_in,
    input  wire [2:0] select,
    output wire       master_miso_line_out
);
    assign master_miso_line_out = slave_miso_lines_in[select];
endmodule
`default_nettype wire

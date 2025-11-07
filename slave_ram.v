// ============================================================
// File: slave_ram.v
// 256 x 16 single-clock, simple R/W RAM (1-cycle read)
// Exposes "mem" for simulation visibility
// ============================================================
`default_nettype none
module slave_ram(
    input  wire        clk,
    input  wire        we,
    input  wire        re,
    input  wire [7:0]  write_addr,
    input  wire [7:0]  read_addr,
    input  wire [15:0] data_in,
    output reg  [15:0] data_out
);
    reg [15:0] mem [0:255];

    always @(posedge clk) begin
        if (we)
            mem[write_addr] <= data_in;
        if (re)
            data_out <= mem[read_addr];
    end
endmodule
`default_nettype wire

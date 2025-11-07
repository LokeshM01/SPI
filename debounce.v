// ============================================================
// File: debounce.v
// Simple debouncer for active-high push buttons
// ============================================================
`default_nettype none
module debounce #(
    parameter integer COUNT_MAX = 250_000  // ~2.5ms @ 100MHz
)(
    input  wire clk,
    input  wire rst,
    input  wire noisy_in,
    output reg  debounced_out
);
    reg        sync0, sync1;
    reg [31:0] cnt;
    reg        stable;

    // 2FF sync
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sync0 <= 1'b0;
            sync1 <= 1'b0;
        end else begin
            sync0 <= noisy_in;
            sync1 <= sync0;
        end
    end

    // counter-based debounce
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt           <= 32'd0;
            stable        <= 1'b0;
            debounced_out <= 1'b0;
        end else begin
            if (sync1 != stable) begin
                // changed -> count until stable for COUNT_MAX cycles
                cnt <= cnt + 1;
                if (cnt >= COUNT_MAX) begin
                    stable        <= sync1;
                    debounced_out <= sync1;
                    cnt           <= 32'd0;
                end
            end else begin
                cnt <= 32'd0;
            end
        end
    end
endmodule
`default_nettype wire

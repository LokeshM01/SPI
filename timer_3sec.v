// ============================================================
// File: timer_3sec.v
// 3-second one-shot timer (start pulse -> done pulse)
// ============================================================
`default_nettype none
module timer_3sec #(
    parameter integer CLK_HZ  = 100_000_000,
    parameter integer SECONDS = 3
)(
    input  wire clk,
    input  wire rst,
    input  wire start,
    output reg  done
);
    localparam integer TERMINAL = CLK_HZ * SECONDS;
    reg        running;
    reg [31:0] cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            running <= 1'b0;
            cnt     <= 32'd0;
            done    <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start && !running) begin
                running <= 1'b1;
                cnt     <= 32'd0;
            end else if (running) begin
                cnt <= cnt + 1;
                if (cnt >= TERMINAL-1) begin
                    running <= 1'b0;
                    done    <= 1'b1;  // one-cycle pulse
                end
            end
        end
    end
endmodule
`default_nettype wire

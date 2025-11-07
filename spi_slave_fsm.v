// ============================================================
// File: spi_slave_fsm.v  (READ bit-slip fixed + concat index fix)
// ============================================================
`default_nettype none
module spi_slave_fsm #(
    parameter [2:0] MY_ID = 3'd0
)(
    input  wire clk,
    input  wire rst,
    input  wire sclk,
    input  wire mosi,
    output reg  miso,
    input  wire cs_n
);
    // Sync external SCLK and CS_N
    reg sclk_d0, sclk_d1;
    reg cs_d0,   cs_d1;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sclk_d0 <= 1'b0; sclk_d1 <= 1'b0;
            cs_d0   <= 1'b1; cs_d1   <= 1'b1;
        end else begin
            sclk_d0 <= sclk;  sclk_d1 <= sclk_d0;
            cs_d0   <= cs_n;  cs_d1   <= cs_d0;
        end
    end
    wire cs_active = ~cs_d1;
    wire sclk_rise = (sclk_d1==1'b0) && (sclk_d0==1'b1);
    wire sclk_fall = (sclk_d1==1'b1) && (sclk_d0==1'b0);

    // RAM
    reg        ram_we, ram_re;
    reg [7:0]  ram_waddr, ram_raddr;
    reg [15:0] ram_din;
    wire[15:0] ram_dout;

    slave_ram u_ram (
        .clk        (clk),
        .we         (ram_we),
        .re         (ram_re),
        .write_addr (ram_waddr),
        .read_addr  (ram_raddr),
        .data_in    (ram_din),
        .data_out   (ram_dout)
    );

    // Debug aliases
    (* keep="true", mark_debug="true" *) wire        ram_we_dbg    = ram_we;
    (* keep="true", mark_debug="true" *) wire [7:0]  ram_addr_dbg  = ram_waddr;
    (* keep="true", mark_debug="true" *) wire [15:0] mosi_data_dbg = ram_din;

    // Shifters/state
    reg [15:0] cmd_shift;
    reg [15:0] data_shift_in;
    reg [15:0] data_shift_out;
    reg [5:0]  bitcnt;
    reg        is_read;

    // NEW: temporary to hold "next command word" (fix for concat indexing)
    reg [15:0] next_cmd;

    localparam [1:0]
        S_IDLE      = 2'd0,
        S_CMD       = 2'd1,
        S_DATA      = 2'd2,
        S_DATA_PREP = 2'd3; // wait 1 clk for RAM + preload MISO

    reg [1:0] state;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state         <= S_IDLE;
            bitcnt        <= 6'd0;
            miso          <= 1'b0;
            ram_we        <= 1'b0;
            ram_re        <= 1'b0;
            ram_waddr     <= 8'h00;
            ram_raddr     <= 8'h00;
            ram_din       <= 16'h0000;
            cmd_shift     <= 16'h0000;
            data_shift_in <= 16'h0000;
            data_shift_out<= 16'h0000;
            is_read       <= 1'b0;
            next_cmd      <= 16'h0000;
        end else begin
            ram_we <= 1'b0;
            ram_re <= 1'b0;

            case (state)
            S_IDLE: begin
                miso <= 1'b0;
                if (cs_active) begin
                    state     <= S_CMD;
                    bitcnt    <= 6'd16;
                    cmd_shift <= 16'h0000;
                end
            end

            S_CMD: begin
                if (!cs_active) begin
                    state <= S_IDLE;
                end else begin
                    // Mode 0: sample MOSI on rising, change MISO on falling
                    if (sclk_rise) begin
                        cmd_shift <= {cmd_shift[14:0], mosi};
                        if (bitcnt != 0) bitcnt <= bitcnt - 1;

                        if (bitcnt == 6'd1) begin
                            // Build the just-captured command word safely
                            next_cmd   = {cmd_shift[14:0], mosi}; // blocking OK for temp
                            is_read    <= next_cmd[1];
                            ram_waddr  <= next_cmd[10:3];
                            ram_raddr  <= next_cmd[10:3];

                            if (next_cmd[1]) begin
                                // READ: assert RAM read, then wait one clk for data valid
                                ram_re <= 1'b1;
                                bitcnt <= 6'd16;
                                state  <= S_DATA_PREP;
                            end else begin
                                // WRITE: straight to data phase
                                data_shift_in <= 16'h0000;
                                bitcnt        <= 6'd16;
                                state         <= S_DATA;
                            end
                        end
                    end
                end
            end

            S_DATA_PREP: begin
                // After one clk, ram_dout is valid (sync RAM)
                data_shift_out <= ram_dout;
                miso           <= ram_dout[15]; // preload MSB before first rising
                state          <= S_DATA;
            end

            S_DATA: begin
                if (!cs_active) begin
                    miso  <= 1'b0;
                    state <= S_IDLE;
                end else if (is_read) begin
                    // READ: master samples @ rising; change MISO @ falling
                    if (sclk_fall) begin
                        miso           <= data_shift_out[15];
                        data_shift_out <= {data_shift_out[14:0], 1'b0};
                    end
                    if (sclk_rise) begin
                        if (bitcnt != 0) bitcnt <= bitcnt - 1;
                    end
                    if (bitcnt == 0) begin
                        miso  <= 1'b0;
                        state <= S_IDLE;
                    end
                end else begin
                    // WRITE: capture MOSI @ rising, write at last bit
                    if (sclk_rise) begin
                        data_shift_in <= {data_shift_in[14:0], mosi};
                        if (bitcnt != 0) bitcnt <= bitcnt - 1;
                        if (bitcnt == 1) begin
                            ram_din <= {data_shift_in[14:0], mosi};
                            ram_we  <= 1'b1;
                        end
                    end
                    if (bitcnt == 0) begin
                        miso  <= 1'b0;
                        state <= S_IDLE;
                    end
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire

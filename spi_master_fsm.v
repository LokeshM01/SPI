// ============================================================
// File: spi_master_fsm.v
// Simple SPI Mode-0 master: 16b CMD then 16b DATA (R/W)
// - cmd_packet_in format:
//   [15:14]=00, [13:11]=ID, [10:3]=ADDR, [2]=GLOBAL(0), [1]=READ, [0]=RES
// ============================================================
`default_nettype none
module spi_master_fsm #(
    parameter integer SCLK_DIV = 8  // clk/(2*SCLK_DIV) ~= SCLK freq
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start_tx,
    input  wire [15:0] cmd_packet_in,
    input  wire [15:0] data_out_in,   // data to write (when READ=0)
    output reg         spi_busy,
    output reg         tx_done,
    output reg [15:0]  data_read_out, // data read (when READ=1)

    output reg         sclk_out,
    output reg         mosi_out,
    input  wire        miso_in,
    output reg  [7:0]  cs_n_out       // active-low chip selects
);
    // Decode fields
    wire [2:0] id    = cmd_packet_in[13:11];
    wire [7:0] addr  = cmd_packet_in[10:3];
    wire       rd    = cmd_packet_in[1];

    // SCLK divider
    reg [15:0] divcnt;
    wire       tick = (divcnt==0);

    // Shift regs/counters
    reg [15:0] shifter_out;
    reg [15:0] shifter_in;
    reg [5:0]  bitcnt; // up to 32 edges
    reg [1:0]  phase;  // 0=CMD, 1=DATA
    reg        active;

    // state
    localparam ST_IDLE     = 2'd0;
    localparam ST_SEND_CMD = 2'd1;
    localparam ST_DATA     = 2'd2;
    localparam ST_DONE     = 2'd3;
    reg [1:0] state;

    // sclk edge detect (we toggle ourselves)
    wire sclk_rise = tick && (sclk_out==1'b0); // will go 0->1
    wire sclk_fall = tick && (sclk_out==1'b1); // will go 1->0

    // clock divider
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            divcnt  <= SCLK_DIV-1;
            sclk_out<= 1'b0;
        end else begin
            if (active) begin
                if (divcnt == 0) begin
                    divcnt  <= SCLK_DIV-1;
                    sclk_out<= ~sclk_out;
                end else begin
                    divcnt <= divcnt - 1;
                end
            end else begin
                divcnt  <= SCLK_DIV-1;
                sclk_out<= 1'b0; // CPOL=0 idle low
            end
        end
    end

    // main
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state         <= ST_IDLE;
            active        <= 1'b0;
            spi_busy      <= 1'b0;
            tx_done       <= 1'b0;
            cs_n_out      <= 8'hFF;
            mosi_out      <= 1'b0;
            data_read_out <= 16'h0000;
            shifter_out   <= 16'h0000;
            shifter_in    <= 16'h0000;
            bitcnt        <= 6'd0;
            phase         <= 2'd0;
        end else begin
            tx_done <= 1'b0;

            case (state)
            ST_IDLE: begin
                spi_busy <= 1'b0;
                active  <= 1'b0;
                cs_n_out<= 8'hFF;
                if (start_tx) begin
                    // assert CS for selected ID
                    cs_n_out      <= ~(8'b1 << id); // one-hot low
                    spi_busy      <= 1'b1;
                    active        <= 1'b1;
                    phase         <= 2'd0;          // CMD phase
                    shifter_out   <= cmd_packet_in; // MSB-first
                    bitcnt        <= 6'd16;         // 16 bits
                    // put first MOSI bit before first rising edge (mode 0: change on falling)
                    mosi_out      <= cmd_packet_in[15];
                    state         <= ST_SEND_CMD;
                end
            end

            ST_SEND_CMD: begin
                // Mode 0: sample @ rising, change @ falling
                if (sclk_rise) begin
                    // slave samples MOSI; master can sample MISO but we don't need during CMD
                    // count a bit sent
                    if (bitcnt != 0) bitcnt <= bitcnt - 1;
                end
                if (sclk_fall) begin
                    // shift to next MOSI bit on falling edges
                    shifter_out <= {shifter_out[14:0],1'b0};
                    mosi_out    <= shifter_out[14]; // next MSB
                    if (bitcnt == 1) begin
                        // next falling will end the 16 bits; prep data phase
                        if (rd) begin
                            // READ: next phase captures 16 MISO bits
                            shifter_in  <= 16'h0000;
                        end else begin
                            // WRITE: next phase shifts data_out_in on MOSI
                            shifter_out <= data_out_in;
                            mosi_out    <= data_out_in[15];
                        end
                        phase   <= 2'd1;
                    end
                end
                // transition to data phase when 16 bits are done (detect after a rising+falling pair)
                if (bitcnt == 0 && sclk_out==1'b0 && tick) begin
                    bitcnt <= 6'd16;
                    state  <= ST_DATA;
                end
            end

            ST_DATA: begin
                if (rd) begin
                    // READ: shift in MISO @ rising, drive don't care MOSI @ falling
                    if (sclk_rise) begin
                        shifter_in <= {shifter_in[14:0], miso_in};
                        if (bitcnt != 0) bitcnt <= bitcnt - 1;
                    end
                    if (sclk_fall) begin
                        // keep MOSI low (or 0) during read data phase
                        mosi_out <= 1'b0;
                    end
                end else begin
                    // WRITE: shift out data on MOSI @ falling
                    if (sclk_rise) begin
                        if (bitcnt != 0) bitcnt <= bitcnt - 1;
                    end
                    if (sclk_fall) begin
                        shifter_out <= {shifter_out[14:0],1'b0};
                        mosi_out    <= shifter_out[14];
                    end
                end

                if (bitcnt == 0 && sclk_out==1'b0 && tick) begin
                    // finish
                    if (rd) data_read_out <= shifter_in;
                    active   <= 1'b0;
                    cs_n_out <= 8'hFF;
                    state    <= ST_DONE;
                end
            end

            ST_DONE: begin
                spi_busy <= 1'b0;
                tx_done  <= 1'b1; // one-cycle pulse
                state    <= ST_IDLE;
            end

            default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire

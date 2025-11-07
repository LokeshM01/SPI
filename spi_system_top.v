// ============================================================
// File: spi_system_top.v
// Top-level wiring to XDC, 8 slaves, status RGBs, etc.
// ============================================================
`default_nettype none
module spi_system_top (
    input  wire        clk,   // 100 MHz

    input  wire [15:0] sw,
    input  wire [3:0]  btn,

    output wire [15:0] led,
    output wire [2:0]  RGB0,
    output wire [2:0]  RGB1
);
    wire sys_reset = btn[0];

    // Control <-> Master
    (* mark_debug="true" *) wire        master_start_tx;
    (* mark_debug="true" *) wire        master_spi_busy;
    (* mark_debug="true" *) wire        master_tx_done;
    (* mark_debug="true" *) wire [15:0] master_cmd_packet;
    (* mark_debug="true" *) wire [15:0] master_data_wr;
    (* mark_debug="true" *) wire [15:0] master_data_rd;

    // SPI bus
    wire        spi_sclk;
    wire        spi_mosi;
    wire [7:0]  spi_cs_n;

    // MISO mux
    wire [7:0]  slave_miso_bus;
    wire        spi_miso_to_master;
    (* mark_debug="true" *) wire [2:0] latched_id_from_fsm;

    // Status wires
    wire led_red_wire, led_blue_wire, led_green_wire, led_green_write_wire;

    // Control FSM
    system_control_fsm u_control_fsm (
        .clk               (clk),
        .rst               (sys_reset),
        .sw_in             (sw),

        .btn_latch         (btn[2]),
        .btn_write_single  (btn[3]),
        .btn_read_single   (btn[1]),

        .led_data_out      (led),
        .led_red_out       (led_red_wire),
        .led_blue_out      (led_blue_wire),
        .led_green_out     (led_green_wire),
        .led_write_done_g  (led_green_write_wire),

        .latched_id_out    (latched_id_from_fsm),

        .master_start_tx   (master_start_tx),
        .master_spi_busy   (master_spi_busy),
        .master_tx_done    (master_tx_done),
        .master_cmd_packet (master_cmd_packet),
        .master_data_wr    (master_data_wr),
        .master_data_rd    (master_data_rd)
    );

    // SPI Master
    spi_master_fsm #(.SCLK_DIV(8)) u_spi_master (
        .clk           (clk),
        .rst           (sys_reset),
        .start_tx      (master_start_tx),
        .cmd_packet_in (master_cmd_packet),
        .data_out_in   (master_data_wr),
        .spi_busy      (master_spi_busy),
        .tx_done       (master_tx_done),
        .data_read_out (master_data_rd),
        .sclk_out      (spi_sclk),
        .mosi_out      (spi_mosi),
        .miso_in       (spi_miso_to_master),
        .cs_n_out      (spi_cs_n)
    );

    // MISO mux: pick slave MISO based on latched ID
    miso_mux_8to1 u_miso_mux (
        .slave_miso_lines_in (slave_miso_bus),
        .select              (latched_id_from_fsm),
        .master_miso_line_out(spi_miso_to_master)
    );

    // 8 SPI slaves
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : slave_gen
            spi_slave_fsm #(.MY_ID(i[2:0])) u_spi_slave (
                .clk   (clk),
                .rst   (sys_reset),
                .sclk  (spi_sclk),
                .mosi  (spi_mosi),
                .miso  (slave_miso_bus[i]),
                .cs_n  (spi_cs_n[i])
            );
        end
    endgenerate

    // RGB status
    // If your board's RGB LEDs are active-low, invert these signals.
    assign RGB0[0] = led_red_wire;        // Red: read returned 0
    assign RGB0[1] = led_green_wire;      // Green: read returned non-zero
    assign RGB0[2] = led_blue_wire;       // Blue: write progress/done (with 3s hold)

    assign RGB1[0] = 1'b0;
    assign RGB1[1] = led_green_write_wire; // Write-done indicator
    assign RGB1[2] = 1'b0;
endmodule
`default_nettype wire

// Wishbone address decoder
// Maps address ranges to peripheral select lines

module wb_address_decoder (
    input wire clk,
    input wire rst,

    // Wishbone master interface
    input wire [7:0] wb_adr_i,
    input wire [7:0] wb_dat_i,
    output reg [7:0] wb_dat_o,
    input wire wb_we_i,
    input wire wb_cyc_i,
    input wire wb_stb_i,
    output reg wb_ack_o,

    // Slave 0 (RGB LED) - signals named to match top.v instantiation
    output reg [7:0] s0_wb_adr_o,
    output reg [7:0] s0_wb_dat_o,
    input wire [7:0] s0_wb_dat_i,
    output reg s0_wb_cyc_o,
    output reg s0_wb_stb_o,
    output reg s0_wb_we_o,
    input wire s0_wb_ack_i,

    // Slave 1 (HDMI)
    output reg [7:0] s1_wb_adr_o,
    output reg [7:0] s1_wb_dat_o,
    input wire [7:0] s1_wb_dat_i,
    output reg s1_wb_cyc_o,
    output reg s1_wb_stb_o,
    output reg s1_wb_we_o,
    input wire s1_wb_ack_i,

    // Slave 2 (USB Serial / UART) - matches top.v naming
    output reg [7:0] s2_wb_adr_o,
    output reg [7:0] s2_wb_dat_o,
    input wire [7:0] s2_wb_dat_i,
    output reg s2_wb_cyc_o,
    output reg s2_wb_stb_o,
    output reg s2_wb_we_o,
    input wire s2_wb_ack_i
);

    wire [3:0] region = wb_adr_i[7:4];

    always @(*) begin
        // Default deasserts
        s0_wb_adr_o = wb_adr_i; s0_wb_dat_o = wb_dat_i; s0_wb_cyc_o = 0; s0_wb_stb_o = 0; s0_wb_we_o = 0;
        s1_wb_adr_o = wb_adr_i; s1_wb_dat_o = wb_dat_i; s1_wb_cyc_o = 0; s1_wb_stb_o = 0; s1_wb_we_o = 0;
        s2_wb_adr_o = wb_adr_i; s2_wb_dat_o = wb_dat_i; s2_wb_cyc_o = 0; s2_wb_stb_o = 0; s2_wb_we_o = 0;
        wb_dat_o = 8'h00;
        wb_ack_o = 0;

        case (region)
            4'h0: begin
                // 0x00-0x0F -> RGB LED (slave 0)
                s0_wb_cyc_o = wb_cyc_i & wb_stb_i;
                s0_wb_stb_o = wb_cyc_i & wb_stb_i;
                s0_wb_we_o = wb_we_i;
                wb_dat_o = s0_wb_dat_i;
                wb_ack_o = s0_wb_ack_i;
            end
            4'h1: begin
                // 0x10-0x1F -> HDMI (slave 1)
                s1_wb_cyc_o = wb_cyc_i & wb_stb_i;
                s1_wb_stb_o = wb_cyc_i & wb_stb_i;
                s1_wb_we_o = wb_we_i;
                wb_dat_o = s1_wb_dat_i;
                wb_ack_o = s1_wb_ack_i;
            end
            4'h2: begin
                // 0x20-0x2F -> USB Serial (slave 2)
                s2_wb_cyc_o = wb_cyc_i & wb_stb_i;
                s2_wb_stb_o = wb_cyc_i & wb_stb_i;
                s2_wb_we_o = wb_we_i;
                wb_dat_o = s2_wb_dat_i;
                wb_ack_o = s2_wb_ack_i;
            end
            default: begin
                // No slave selected
                wb_dat_o = 8'h00;
                wb_ack_o = 1'b1;
            end
        endcase
    end

endmodule

// wb_address_decoder_4.v
// Wishbone address decoder for 4 slaves
// Extended from 3-slave version - now supports 16-bit addressing
//
// Address Map (revised for HQVGA's 19200 pixel frame buffer):
// 0x0000-0x7FFF: Slave 3 (HQVGA) - 32K addresses for frame buffer (bit 15=0)
// 0x8000-0x80FF: Slave 0 (RGB LED) - 256 addresses
// 0x8100-0x81FF: Slave 1 (HDMI Video) - 256 addresses  
// 0x8200-0x8FFF: Slave 2 (Character RAM) - 3584 addresses

(* keep = "true" *)
module wb_address_decoder_4 (
    input wire clk,
    input wire rst,
    
    // Master interface (16-bit address)
    input wire [15:0] wb_adr_i,
    input wire [7:0] wb_dat_i,
    output wire [7:0] wb_dat_o,
    input wire wb_cyc_i,
    input wire wb_stb_i,
    input wire wb_we_i,
    output wire wb_ack_o,
    
    // Slave 0 interface (0x8000-0x80FF)
    output wire [7:0] s0_wb_adr_o,
    output wire [7:0] s0_wb_dat_o,
    input wire [7:0] s0_wb_dat_i,
    output wire s0_wb_cyc_o,
    output wire s0_wb_stb_o,
    output wire s0_wb_we_o,
    input wire s0_wb_ack_i,
    
    // Slave 1 interface (0x8100-0x81FF)
    output wire [7:0] s1_wb_adr_o,
    output wire [7:0] s1_wb_dat_o,
    input wire [7:0] s1_wb_dat_i,
    output wire s1_wb_cyc_o,
    output wire s1_wb_stb_o,
    output wire s1_wb_we_o,
    input wire s1_wb_ack_i,
    
    // Slave 2 interface (0x8200-0x8FFF)
    output wire [7:0] s2_wb_adr_o,
    output wire [7:0] s2_wb_dat_o,
    input wire [7:0] s2_wb_dat_i,
    output wire s2_wb_cyc_o,
    output wire s2_wb_stb_o,
    output wire s2_wb_we_o,
    input wire s2_wb_ack_i,
    
    // Slave 3 interface (0x0000-0x7FFF) - 15-bit address for HQVGA frame buffer
    output wire [14:0] s3_wb_adr_o,
    output wire [7:0] s3_wb_dat_o,
    input wire [7:0] s3_wb_dat_i,
    output wire s3_wb_cyc_o,
    output wire s3_wb_stb_o,
    output wire s3_wb_we_o,
    input wire s3_wb_ack_i
);

    // Decode slave select - HQVGA gets lower half (bit 15=0), peripherals get upper half
    wire sel_s3 = !wb_adr_i[15];                                    // 0x0000-0x7FFF: HQVGA
    wire sel_s0 = wb_adr_i[15] && (wb_adr_i[14:8] == 7'h00);       // 0x8000-0x80FF: RGB LED
    wire sel_s1 = wb_adr_i[15] && (wb_adr_i[14:8] == 7'h01);       // 0x8100-0x81FF: HDMI
    wire sel_s2 = wb_adr_i[15] && (wb_adr_i[14:9] >= 6'h01);       // 0x8200-0xFFFF: Char RAM
    
    // Route address and data to all slaves
    assign s0_wb_adr_o = wb_adr_i[7:0];   // Pass lower 8 bits to slaves 0-2
    assign s0_wb_dat_o = wb_dat_i;
    assign s1_wb_adr_o = wb_adr_i[7:0];
    assign s1_wb_dat_o = wb_dat_i;
    assign s2_wb_adr_o = wb_adr_i[7:0];
    assign s2_wb_dat_o = wb_dat_i;
    assign s3_wb_adr_o = wb_adr_i[14:0];  // Pass 15 bits to HQVGA (0x0000-0x7FFF range)
    assign s3_wb_dat_o = wb_dat_i;
    
    // Route control signals based on address decode
    assign s0_wb_cyc_o = wb_cyc_i && sel_s0;
    assign s0_wb_stb_o = wb_stb_i && sel_s0;
    assign s0_wb_we_o = wb_we_i;
    
    assign s1_wb_cyc_o = wb_cyc_i && sel_s1;
    assign s1_wb_stb_o = wb_stb_i && sel_s1;
    assign s1_wb_we_o = wb_we_i;
    
    assign s2_wb_cyc_o = wb_cyc_i && sel_s2;
    assign s2_wb_stb_o = wb_stb_i && sel_s2;
    assign s2_wb_we_o = wb_we_i;
    
    assign s3_wb_cyc_o = wb_cyc_i && sel_s3;
    assign s3_wb_stb_o = wb_stb_i && sel_s3;
    assign s3_wb_we_o = wb_we_i;
    
    // Multiplex data back from slaves
    assign wb_dat_o = sel_s0 ? s0_wb_dat_i :
                      sel_s1 ? s1_wb_dat_i :
                      sel_s2 ? s2_wb_dat_i :
                      sel_s3 ? s3_wb_dat_i :
                      8'h00;
    
    // OR all acknowledge signals
    assign wb_ack_o = s0_wb_ack_i | s1_wb_ack_i | s2_wb_ack_i | s3_wb_ack_i;

endmodule

// Simple UART transmitter module (copied)
module uart_tx (
    input wire clk,
    input wire rst,
    input wire [7:0] data,
    input wire send,
    output reg tx,
    output reg busy
);

    // Parameter tuned for 27 MHz clock and 115200 baud
    parameter CLKS_PER_BIT = 234;

    reg [15:0] clk_count;
    reg [3:0] bit_index;
    reg [9:0] shifter;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx <= 1'b1;
            busy <= 0;
            clk_count <= 0;
            bit_index <= 0;
            shifter <= 10'b1111111111;
        end else begin
            if (send && !busy) begin
                // Load frame: start bit (0), data LSB first, stop bit (1)
                shifter <= {1'b1, data, 1'b0};
                busy <= 1;
                clk_count <= 0;
                bit_index <= 0;
            end else if (busy) begin
                if (clk_count < CLKS_PER_BIT - 1) begin
                    clk_count <= clk_count + 1;
                end else begin
                    clk_count <= 0;
                    tx <= shifter[0];
                    shifter <= {1'b1, shifter[9:1]};
                    bit_index <= bit_index + 1;
                    if (bit_index == 9) begin
                        busy <= 0;
                    end
                end
            end
        end
    end

endmodule

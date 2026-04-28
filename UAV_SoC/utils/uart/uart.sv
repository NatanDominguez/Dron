

module uart #(
    parameter int DATA_SIZE = 32,
    parameter int ADDR_SIZE = 32
    ) (
    input clk_i,
    input rst_ni,

    input logic [ADDR_SIZE-1:0] addr_i,

    input logic [DATA_SIZE-1:0] data_w_i,
    input logic we_i,

    output logic [DATA_SIZE-1:0] data_r_o,
    output logic r_valid_o,
    input logic re_i
);

    reg [DATA_SIZE-1:0] data;
    reg [7:0] status;
    reg [15:0] baudrate;

    logic [1:0] sel;

    always_comb begin

        sel = 2'b11;
        if(addr_i[3:0] == 4'h0) sel = 2'b00;    // DATA
        if(addr_i[3:0] == 4'h4) sel = 2'b01;    // STATUS
        if(addr_i[3:0] == 4'h8) sel = 2'b10;    // BAUDRATE

    end

    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            data <= '0;
            status <= '0;
            baudrate <= '0;
            data_r_o <= '0;
            r_valid_o <= 1'b0;
        end else begin

            data_r_o <= '0;
            r_valid_o <= 1'b0;
            
            case(sel)
                2'b00: 
                begin
                    if(we_i) data <= data_w_i[7:0];
                    else if(re_i) begin
                        data_r_o[7:0] <= data;
                        r_valid_o <= 1'b1;
                    end
                end
                2'b01:
                begin
                    if(re_i) begin
                        data_r_o[7:0] <= status;
                        r_valid_o <= 1'b1;
                    end
                end
                2'b10:
                begin
                    if(we_i) baudrate <= data_w_i[7:0];
                    else if(re_i) begin
                        data_r_o[7:0] <= baudrate;
                        r_valid_o <= 1'b1;
                    end
                end
            endcase
        end

    end

    logic [7:0] data_tx;
    assign data_tx = data;
/*
    uart_tx i_uart_tx (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .data_tx_i (data_tx),
        .prescale_i (baudrate)
    );*/

endmodule
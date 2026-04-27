
module ram #(
    parameter DATA_SIZE = 32,
    parameter DEPTH = 512,
    parameter ADDR_SIZE = 32
) (
    input clk_i,
    input rst_ni,

    input re_i,
    input logic [ADDR_SIZE-1:0] addr_r_i,
    output logic [DATA_SIZE-1:0] data_r_o,
    output logic data_r_valid,

    input we_i,
    input logic [ADDR_SIZE-1:0] addr_w_i,
    input logic [DATA_SIZE-1:0] data_w_i
);

    reg [DATA_SIZE-1:0] mem [DEPTH];

    integer i;
    always @(posedge clk_i) begin
        if(!rst_ni) begin
            data_r_o <= '0;
            for(i = 0; i < DEPTH; i++) begin
                mem[i] <= '0;
            end
        end else begin
            if(we_i) begin
                mem[addr_w_i] <= data_w_i;
            end
            if(re_i) begin
                data_r_o <= mem[addr_r_i];
            end
        end
    end

    always @(posedge clk_i) begin
        if(!rst_ni) begin
            data_r_valid <= 1'b0;
        end else begin
            data_r_valid <= 1'b0;
            if(re_i && !data_r_valid)   data_r_valid <= 1'b1;
        end
    end


endmodule
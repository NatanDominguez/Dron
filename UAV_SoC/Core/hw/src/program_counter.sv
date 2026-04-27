
module program_counter #(
    parameter int DEPTH = 64,
    parameter int REG_SIZE = $clog2(DEPTH)
) (
    input logic clk_i,
    input logic rst_ni,

    input logic stall_i,

    input logic [REG_SIZE-1:0] next_pc_i,

    output logic [REG_SIZE-1:0] pc_o
);

    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            pc_o <= '0;
        end else if (!stall_i) begin
            pc_o <= next_pc_i;
        end
    end
    
endmodule
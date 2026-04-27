
import core_pkg::*;

module program_counter #(
    parameter int PC_SIZE = 32
) (
    input logic clk_i,
    input logic rst_ni,

    input logic stall_i,
    input pc_sel_enum jump_i,
    input logic [PC_SIZE-1:0] jump_offset_i,
    input logic [31:0] alu_i,
    input logic eq_i,
    input logic lt_i,

    output logic [PC_SIZE-1:0] pc_o
);

    logic take_branch;

    always_comb begin
        
        case(jump_i)
            PC_INCR: take_branch = 1'b0;
            PC_JUMP: take_branch = 1'b1;
            PC_BEQ:  take_branch = eq_i;
            PC_BNE:  take_branch = ~eq_i;
            PC_BLT:  take_branch = lt_i;
            PC_BGE:  take_branch = ~lt_i;
            PC_BLTU: take_branch = lt_i;
            PC_BGEU: take_branch = ~lt_i;
            PC_JALR: take_branch = 1'b1;
            default: take_branch = 1'b0;
        endcase
    end

    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            pc_o <= '0;
        end else if (!stall_i) begin
            pc_o <= take_branch ? ((jump_i == PC_JALR) ? alu_i : pc_o + jump_offset_i) : (pc_o + 4);
            //pc_o <= take_branch ? (pc_o + ((jump_i == PC_JALR) ? alu_i : jump_offset_i)) : (pc_o + 4);
        end
    end
    
endmodule

import riscv_pkg::*;
import core_pkg::*;

module decoder #(
    parameter int DATA_SIZE = 32,
    parameter int SIZE_ALU_OP = $clog2(8)
) (
    input [31:0] inst_i,

    output logic [4:0] rs1_o,
    output logic [4:0] rs2_o,
    output logic [4:0] rd_o,
    output logic       reg_we_o,
    output w_sel_enum  reg_w_sel_o,

    output logic [DATA_SIZE-1:0] imm_o,

    output logic [3:0] alu_op_o,
    output logic       alu_b_src_o,

    output logic       ram_re_o,
    output logic       ram_we_o,

    output logic [DATA_SIZE-1:0] jump_offset_o,
    output logic       jump_o
);

    assign rs1_o = inst_i[19:15];
    assign rs2_o = inst_i[24:20];
    assign rd_o = inst_i[11:7];

    opcode_enum opcode;
    funct3_enum funct3;
    funct7_enum funct7;

    assign opcode = opcode_enum'(inst_i[6:0]);
    assign funct3 = funct3_enum'(inst_i[14:12]);
    assign funct7 = funct7_enum'(inst_i[31:25]);

    always_comb begin
        
        alu_op_o = 1'b0;
        alu_b_src_o = 1'b0;
        imm_o = '0;
        reg_we_o = 1'b1;
        reg_w_sel_o = W_ALU;
        ram_re_o = 1'b0;
        ram_we_o = 1'b0;
        jump_offset_o = '0;
        jump_o = 1'b0;

        case(opcode)
            R_TYPE:
            begin
                case(funct3)
                    F3_ADD_SUB:
                        begin
                            case(funct7)
                                F7_STANDARD: alu_op_o = SIZE_ALU_OP'(0);
                                F7_ALT: alu_op_o = SIZE_ALU_OP'(1);
                            endcase
                        end
                    F3_SLL: alu_op_o = SIZE_ALU_OP'(5);
                    /*
                    F3_SLT:
                    F3_SLTU:
                    F3_XOR:
                    F3_SRL_SRA:
                    F3_OR:
                    F3_AND:
                    */
                endcase
            end
            I_TYPE_ALU:
            begin
                imm_o = {{20{inst_i[31]}}, inst_i[31:20]};
                alu_b_src_o = 1'b1;

                case(funct3)
                    F3_ADD_SUB:
                    begin
                        
                    end
                endcase
            end
            I_TYPE_LOAD:
            begin
                imm_o = {{20{inst_i[31]}}, inst_i[31:20]};
                alu_b_src_o = 1'b1;
                reg_w_sel_o = W_RAM;
                ram_re_o = 1'b1;
/*
                case(funct3)
                    3'b000: // LOAD BYTE
                    begin
                            
                    end
                    3'b001: // LOAD HALF-WORD
                    begin
                        
                    end
                    3'b010: // LOAD WORD
                    begin
                        
                    end
                    3'b100: // LOAD 
                    begin
                        
                    end
                    3'b101:
                    begin
                        
                    end

                endcase
*/
            end
            S_TYPE:
            begin
                reg_we_o = 1'b0;
                ram_we_o = 1'b1;
                alu_b_src_o = 1'b1;

                imm_o = {{20{inst_i[31]}} ,inst_i[31:25], inst_i[11:7]};
                /*
                case(funct3)
                    3'b000:



                endcase
                */
            end
            B_TYPE:
            begin
                
            end
            J_TYPE:
            begin
                reg_w_sel_o = W_JUMP;
                jump_o = 1'b1;
                jump_offset_o = {{12{inst_i[31]}}, inst_i[19:12], inst_i[20], inst_i[30:21], 1'b0};
                
            end
            U_TYPE:
            begin
                
            end
            default: continue;
        endcase
    end



endmodule
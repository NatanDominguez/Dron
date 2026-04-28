
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

    output op_enum     alu_op_o,
    output logic       alu_b_src_o,

    output logic       re_x_o,
    output logic       we_x_o,

    output logic [DATA_SIZE-1:0] jump_offset_o,
    output pc_sel_enum       jump_o,

    output logic [DATA_SIZE-1:0] imm_u_o
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
        
        alu_op_o = ADD;
        alu_b_src_o = 1'b0;
        imm_o = '0;
        reg_we_o = 1'b1;
        reg_w_sel_o = W_ALU;
        re_x_o = 1'b0;
        we_x_o = 1'b0;
        jump_offset_o = '0;
        jump_o = PC_INCR;
        imm_u_o = '0;

        case(opcode)
            R_TYPE:
            begin
                case(funct3)
                    F3_ADD_SUB:
                        begin
                            case(funct7)
                                F7_STANDARD: alu_op_o = ADD;
                                F7_ALT: alu_op_o = SUB;
                            endcase
                        end
                    F3_SLL:     alu_op_o = SLL;
                    F3_SLT:     alu_op_o = SLT;
                    F3_SLTU:    alu_op_o = SLT;
                    F3_XOR:     alu_op_o = XOR;
                    F3_SRL_SRA: 
                        begin
                            case(funct7)
                                F7_STANDARD:    alu_op_o = SRL;
                                F7_ALT:         alu_op_o = SRA;
                            endcase
                        end
                    F3_OR:      alu_op_o = OR;
                    F3_AND:     alu_op_o = AND;
                endcase
            end
            I_TYPE_ALU:
            begin
                imm_o = {{20{inst_i[31]}}, inst_i[31:20]};
                alu_b_src_o = 1'b1;
                case(funct3)
                    F3_ADD_SUB:
                        begin
                            case(funct7)
                                F7_STANDARD: alu_op_o = ADD;
                                F7_ALT: alu_op_o = SUB;
                            endcase
                        end
                    F3_SLL:     alu_op_o = SLL;
                    F3_SLT:     alu_op_o = SLT;
                    F3_SLTU:    alu_op_o = SLT;
                    F3_XOR:     alu_op_o = XOR;
                    F3_SRL_SRA: 
                        begin
                            case(funct7)
                                F7_STANDARD:    alu_op_o = SRL;
                                F7_ALT:         alu_op_o = SRA;
                            endcase
                        end
                    F3_OR:      alu_op_o = OR;
                    F3_AND:     alu_op_o = AND;
                endcase
            end
            I_TYPE_LOAD:
            begin
                imm_o = {{20{inst_i[31]}}, inst_i[31:20]};
                alu_b_src_o = 1'b1;
                reg_w_sel_o = W_RAM;
                re_x_o = 1'b1;
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
                we_x_o = 1'b1;
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

                jump_offset_o = {{20{inst_i[31]}}, inst_i[7], inst_i[30:25], inst_i[11:8], 1'b0};
                case(funct3)
                    3'b000: begin
                        jump_o = PC_BEQ;
                        alu_op_o = SUB;
                    end 
                    3'b001: begin
                        jump_o = PC_BNE;
                        alu_op_o = SUB;
                    end
                    3'b100: begin
                        jump_o = PC_BLT;
                        alu_op_o = SLT;
                    end
                    3'b101: begin
                        jump_o = PC_BGE;
                        alu_op_o = SLT;
                    end
                    3'b110: begin
                        jump_o = PC_BLTU;
                        alu_op_o = SLT;
                    end
                    3'b111: begin
                        jump_o = PC_BGEU;
                        alu_op_o = SLT;
                    end

                endcase
                
            end
            J_TYPE:
            begin
                reg_w_sel_o = W_JUMP;
                jump_o = PC_JUMP;
                jump_offset_o = {{12{inst_i[31]}}, inst_i[19:12], inst_i[20], inst_i[30:21], 1'b0};
                
            end
            LUI:
            begin
                reg_w_sel_o = W_UPPER;
                imm_u_o = {inst_i[31:12], 12'b0};
            end
            AUIPC:
            begin
                reg_w_sel_o = W_AUIPC;
                imm_u_o = {inst_i[31:12], 12'b0};
            end
            JALR:
            begin
                jump_o = PC_JALR;
                reg_w_sel_o = W_JUMP;
                alu_b_src_o = 1'b1;
                imm_o = {{20{inst_i[31]}}, inst_i[31:20]};
            end
            default: continue;
        endcase
    end

endmodule
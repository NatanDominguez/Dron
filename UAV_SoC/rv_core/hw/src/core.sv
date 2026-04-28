

import core_pkg::*;

module core #(
    parameter int DATA_SIZE = 32,
    parameter int NUM_OP = 9,
    parameter int SIZE_OP = $clog2(NUM_OP),
    parameter int ADDR_SIZE = 5,
    parameter int PC_W = 32
) (
    input clk_i,
    input rst_ni,

    input logic [31:0] inst_i,

    output logic [PC_W-1:0] pc_o,

    output logic [31:0] addr_x_o,

    output logic re_x_o,
    input  logic [DATA_SIZE-1:0] data_rx_i,
    input  logic rx_valid_i,

    output logic we_x_o,

    output logic [DATA_SIZE-1:0] data_wx_o

);

    // SYS
    logic stall;

    // ALU
    logic [DATA_SIZE-1:0] a, b, alu_o;
    op_enum   op;
    logic                 carry, zero, lt, alu_b_src;

    // REGISTERS
    logic [ADDR_SIZE-1:0] addr_ra, addr_rb, addr_w;
    logic [DATA_SIZE-1:0] b_reg, w_reg;
    logic                 we_reg;

    w_sel_enum  reg_w_sel;

    // DECODER
    logic [DATA_SIZE-1:0] imm, jump_offset, imm_u;
    pc_sel_enum           jump;

    always_comb begin
        if(~re_x_o) stall = 1'b0;
        else stall = 1'b1;

        if(stall == 1'b1 && rx_valid_i) stall = 1'b0;
    end

    assign b = alu_b_src ? imm : b_reg;
    
    assign addr_x_o = alu_o;
    assign data_wx_o = b_reg;

    alu #(
        .DATA_SIZE (DATA_SIZE),
        .NUM_OP (NUM_OP)
    ) i_alu (
        .a_i (a),
        .b_i (b),
        .op_i (op),
        .out_o (alu_o),
        .carry_o (carry),
        .zero_o (zero),
        .lt_o (lt)
    );

    always_comb begin
        case(reg_w_sel)
            W_ALU:      w_reg = alu_o;
            W_RAM:      w_reg = data_rx_i;
            W_JUMP:     w_reg = pc_o + 4;
            W_UPPER:    w_reg = imm_u;
            W_AUIPC:    w_reg = imm_u + pc_o;
            default:    w_reg = alu_o;
        endcase 
    end

    registers #(
        .DATA_SIZE (DATA_SIZE),
        .ADDR_NUM  (32)
    ) i_registers (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .stall_i (stall),
        
        .addr_ra_i (addr_ra),
        .data_ra_o (a),

        .addr_rb_i (addr_rb),
        .data_rb_o (b_reg),

        .addr_w_i (addr_w),
        .data_w_i (w_reg),
        .we_i     (we_reg)
    );

    decoder #(
        .DATA_SIZE (DATA_SIZE)
    ) i_decoder (
        .inst_i (inst_i),
        .rs1_o (addr_ra),
        .rs2_o (addr_rb),
        .rd_o  (addr_w),
        .imm_o (imm),
        .alu_op_o (op),
        .alu_b_src_o (alu_b_src),
        .reg_we_o (we_reg),
        .reg_w_sel_o ( reg_w_sel ),
        .re_x_o (re_x_o),
        .we_x_o (we_x_o),
        .jump_offset_o (jump_offset),
        .jump_o (jump),
        .imm_u_o (imm_u)
    );

    program_counter #(
        .PC_SIZE (PC_W)
    ) i_program_counter (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .stall_i (stall),
        .jump_i  (jump),
        .jump_offset_i (jump_offset),
        .alu_i (alu_o),
        .eq_i (zero),
        .lt_i (lt),
        .pc_o (pc_o)
    );

endmodule
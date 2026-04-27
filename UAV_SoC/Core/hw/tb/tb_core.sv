
`timescale 10ns/10ps

module tb_core;

    import core_pkg::*;

    parameter int DATA_SIZE = 32;
    parameter int NUM_OP = 9;
    parameter int SIZE_OP = $clog2(NUM_OP);
    parameter int ADDR_SIZE = 5;
    parameter int INST_DEPTH = 64;
    parameter int PC_W = $clog2(INST_DEPTH);
    parameter int RAM_DEPTH = 512;
    parameter int RAM_ADDR_SIZE = $clog2(RAM_DEPTH);

    logic clk;
    logic rst_n;

    // SYS
    logic stall;

    // ALU
    logic [DATA_SIZE-1:0] a, b;
    logic [SIZE_OP-1:0] op;
    logic carry, zero, lt, alu_b_src;

    // REGISTER
    logic [DATA_SIZE-1:0] alu_o, b_reg, w_reg;
    logic [ADDR_SIZE-1:0] addr_ra, addr_rb, addr_w;
    logic ena;

    w_sel_enum reg_w_sel;

    // DECODER
    logic [31:0] inst, imm, jump_offset;
    logic jump;

    // PROGRAM COUNTER
    logic [PC_W-1:0] pc, next_pc;

    // RAM
    logic re_ram, we_ram, ram_r_valid;
    logic [RAM_ADDR_SIZE-1:0] ram_r_addr, ram_w_addr;
    logic [DATA_SIZE-1:0] ram_r_data, ram_w_data;


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

    assign b = alu_b_src ? imm : b_reg;

    always_comb begin
        if(~re_ram) stall = 1'b0;
        else stall = 1'b1;

        if(stall == 1'b1 && ram_r_valid) stall = 1'b0;
    end

    registers #(
        .DATA_SIZE (DATA_SIZE),
        .ADDR_NUM  (32)
    ) i_registers (
        .clk_i (clk),
        .rst_ni (rst_n),
        .stall_i (stall),
        
        .addr_ra_i (addr_ra),
        .data_ra_o (a),

        .addr_rb_i (addr_rb),
        .data_rb_o (b_reg),

        .addr_w_i (addr_w),
        .data_w_i (w_reg),
        .we_i     (we_reg)
    );


    always_comb begin
        case(reg_w_sel)
            W_ALU:  w_reg = alu_o;
            W_RAM:  w_reg = ram_r_data;
            W_JUMP: w_reg = pc + 4;
            default: w_reg = alu_o;
        endcase 
    end

    decoder #(
        .DATA_SIZE (DATA_SIZE)
    ) i_decoder (
        .inst_i (inst),
        .rs1_o (addr_ra),
        .rs2_o (addr_rb),
        .rd_o  (addr_w),
        .imm_o (imm),
        .alu_op_o (op),
        .alu_b_src_o (alu_b_src),
        .reg_we_o (we_reg),
        .reg_w_sel_o ( reg_w_sel ),
        .ram_re_o (re_ram),
        .ram_we_o (we_ram),
        .jump_offset_o (jump_offset),
        .jump_o (jump)
    );

    program_counter #(
        .DEPTH (INST_DEPTH)
    ) i_program_counter (
        .clk_i (clk),
        .rst_ni (rst_n),
        .stall_i (stall),
        .next_pc_i (next_pc),
        .pc_o (pc)
    );

    assign next_pc = jump ? (pc + jump_offset) : (pc + 4);

    instr_mem #(
        .DEPTH (INST_DEPTH)
    ) i_instr_mem (
        .addr_i (pc),
        .instr_o (inst)
    );

    ram i_ram (
        .clk_i (clk),
        .rst_ni (rst_n),

        .re_i (re_ram),
        .addr_r_i (ram_r_addr),
        .data_r_o (ram_r_data),
        .data_r_valid (ram_r_valid),
        
        .we_i (we_ram),
        .addr_w_i (ram_w_addr),
        .data_w_i (ram_w_data)
    );

    assign ram_r_addr = alu_o;
    assign ram_w_addr = alu_o;

    assign ram_w_data = b_reg;


    initial begin
        clk = 1'b0;
        rst_n = 1'b0;

        ena = 1'b0;
    end

    always #5 clk = ~clk;

    always begin

        repeat(5) @(posedge clk);
        rst_n = 1'b1;
        ena = 1'b1;

        repeat(64) @(posedge clk);

        $finish;
    end
/*
    function logic [31:0] rtype(
        input logic [6:0] funct7,
        input logic [4:0] rs1,
        input logic [4:0] rs2,
        input logic [2:0] funct3,
        input logic [4:0] rd
        );
        return {funct7, rs1, rs2, funct3, rd, 7'b0110011};
    endfunction


    function logic [31:0] itype_load(
        input logic [11:0] imm,
        input logic [4:0]  rs1,
        input logic [2:0]  f3,
        input logic [4:0]  rd
        );
        return {imm, rs1, f3, rd, 7'b0010011};
    endfunction
*/

endmodule
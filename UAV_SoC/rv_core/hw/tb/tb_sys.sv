
import core_pkg::*;
import riscv_pkg::*;

module tb_sys;

    parameter int DATA_SIZE = 32;
    parameter int NUM_OP = 9;
    parameter int SIZE_OP = $clog2(NUM_OP);
    parameter int ADDR_SIZE = 5;
    parameter int INST_DEPTH = 64;
    parameter int PC_W = 32;
    parameter int RAM_ADDR_SIZE = 32;


    logic clk, rst_n;

    logic [31:0] inst;
    logic [PC_W-1:0] pc;

    logic re_ram, we_ram, ram_r_valid;
    logic [RAM_ADDR_SIZE-1:0] ram_addr;
    logic [DATA_SIZE-1:0] ram_r_data, ram_w_data;

    core i_core (
        .clk_i (clk),
        .rst_ni (rst_n),

        .inst_i (inst),
        .pc_o   (pc),
        
        .ram_addr_o ( ram_addr),
        .re_ram_o (re_ram),
        .ram_r_data_i (ram_r_data),
        .ram_r_valid_i (ram_r_valid),

        .we_ram_o (we_ram),
        .ram_w_data_o (ram_w_data)
    );

    instr_mem #(
        .DEPTH (INST_DEPTH)
    ) i_instr_mem (
        .addr_i  (pc),
        .instr_o (inst)
    );

    ram i_ram (
        .clk_i          (clk),
        .rst_ni         (rst_n),
        .re_i           (re_ram),
        .addr_r_i       (ram_addr),
        .data_r_o       (ram_r_data),
        .data_r_valid   (ram_r_valid),
        .we_i           (we_ram),
        .addr_w_i       (ram_addr),
        .data_w_i       (ram_w_data)
    );


    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
    end

    always #5 clk = ~clk;

    always begin

        repeat(5) @(posedge clk);
        rst_n = 1'b1;

        repeat(255) @(posedge clk);

        $finish;
    end

endmodule
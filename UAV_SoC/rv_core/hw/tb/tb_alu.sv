
`timescale 10ns/10ps

module tb_alu;

    parameter int SIZE = 8;
    parameter int NUM_OP = 9;
    parameter int SIZE_OP = $clog2(NUM_OP);

    logic clk;
    logic rst_n;

    logic [SIZE-1:0] a, b, result;
    logic [SIZE_OP-1:0] op;
    logic carry, zero, lt;


    alu #(
        .DATA_SIZE (SIZE),
        .NUM_OP (NUM_OP)
    ) i_alu (
        .a_i (a),
        .b_i (b),
        .op_i (op),
        .out_o (result),
        .carry_o (carry),
        .zero_o (zero),
        .lt_o (lt)
    );

    registers #(
        .DATA_SIZE (SIZE),
        .ADDR_NUM  (32)
    ) (
        .clk_i (clk),
        .rst_ni (rst_n),
        .ena_i (ena),
        
        .addr_ra_i (addr_ra),
        .data_ra_o (data_ra),

        .addr_ra_i (addr_ra),
        .data_ra_o (data_ra),

        .addr_w_i (addr_w),
        .data_w_i (data_w),
    )


    initial begin
        clk = 1'b0;
        rst_n = 1'b1;
        a = '0;
        b = '0;
        op = '0;

        addr_ra = '0;
        addr_rb = '0;

        data_w = '0;
        addr_w = '0;

        ena = 1'b0;
    end

    always #5 clk = ~clk;


    always begin

        repeat(5) @(posedge clk);

        //ADD
        op = '0;
        a = SIZE'(15);
        b = SIZE'(10);

        repeat(5) @(posedge clk);

        //SUB
        op = SIZE_OP'(1);

        repeat(5) @(posedge clk);

        //ADD W OVERFLOW
        op = '0;
        a = {SIZE{1'b1}};
        b = SIZE'(1);

        repeat(5) @(posedge clk);

        //AND
        op = SIZE_OP'(2);
        a = SIZE'(3);
        b = SIZE'(9);

        repeat(5) @(posedge clk);

        //OR
        op = SIZE_OP'(3);
        a = SIZE'(7);
        b = SIZE'(9);

        repeat(5) @(posedge clk);

        //XOR
        op = SIZE_OP'(4);
        a = SIZE'(5);
        b = SIZE'(127);

        repeat(5) @(posedge clk);

        //SHIFT LEFT
        op = SIZE_OP'(5);
        a = SIZE'(16);
        b = SIZE'(3);

        repeat(5) @(posedge clk);

        //SHIFT RIGHT
        op = SIZE_OP'(6);
        a = SIZE'(16);
        b = SIZE'(3);

        repeat(5) @(posedge clk);

        //SHIFT RIGHT ARITHMETIC
        op = SIZE_OP'(7);
        a = SIZE'(200);
        b = SIZE'(20);

        repeat(5) @(posedge clk);

        //LESS THAN (POSITIVE)
        op = SIZE_OP'(8);
        a = SIZE'(20);
        b = SIZE'(30);
        
        repeat(5) @(posedge clk);

        //LESS THAN (NEGATIVE)
        op = SIZE_OP'(8);
        a = SIZE'(40);
        b = SIZE'(30);

        repeat(5) @(posedge clk);

        $finish;
    end


endmodule
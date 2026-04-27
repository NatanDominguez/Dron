
package core_pkg;

    typedef enum logic [2:0] {
        W_ALU = 0,
        W_RAM = 1,
        W_JUMP = 2,
        W_UPPER = 3,
        W_AUIPC = 4
    } w_sel_enum;

    typedef enum logic [3:0] {
        ADD = 0,
        SUB = 1,
        AND = 2,
        OR  = 3,
        XOR = 4,
        SLL = 5,
        SRL = 6,
        SRA = 7,
        SLT = 8
    } op_enum;

    typedef enum logic [3:0] {
        PC_INCR   = 4'b0000,
        PC_JUMP   = 4'b0001,
        PC_BEQ    = 4'b0010,
        PC_BNE    = 4'b0011,
        PC_BLT    = 4'b0100,
        PC_BGE    = 4'b0101,
        PC_BLTU   = 4'b0110,
        PC_BGEU   = 4'b0111,
        PC_JALR   = 4'b1000
    } pc_sel_enum;

endpackage
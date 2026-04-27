
package riscv_pkg;

    typedef enum logic [6:0] {
        R_TYPE      = 7'b0110011,
        I_TYPE_ALU  = 7'b0010011,
        I_TYPE_LOAD = 7'b0000011,
        S_TYPE      = 7'b0100011,
        B_TYPE      = 7'b1100011,
        J_TYPE      = 7'b1101111,
        LUI         = 7'b0110111,
        AUIPC       = 7'b0010111,
        JALR        = 7'b1100111
    } opcode_enum;

    typedef enum logic [2:0] {
        F3_ADD_SUB = 3'b000, // Suma o Resta
        F3_SLL     = 3'b001, // Shift Left Logical
        F3_SLT     = 3'b010, // Set Less Than (signed)
        F3_SLTU    = 3'b011, // Set Less Than (unsigned)
        F3_XOR     = 3'b100, // XOR
        F3_SRL_SRA = 3'b101, // Shift Right (Logical o Arithmetic)
        F3_OR      = 3'b110, // OR
        F3_AND     = 3'b111  // AND
    } funct3_enum;

    typedef enum logic [6:0] {
        F7_STANDARD = 7'b0000000,
        F7_ALT      = 7'b0100000
    } funct7_enum;

endpackage


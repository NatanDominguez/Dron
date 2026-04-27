

module alu #(
    parameter int DATA_SIZE = 32,
    parameter int SIZE_LOG = $clog2(DATA_SIZE),
    parameter int NUM_OP = 8,
    parameter int SIZE_OP = $clog2(NUM_OP)
    ) (
    input logic [DATA_SIZE-1:0] a_i,
    input logic [DATA_SIZE-1:0] b_i,

    input logic [SIZE_OP-1:0] op_i,

    output logic [DATA_SIZE-1:0] out_o,
    output logic carry_o,

    output logic zero_o,
    output logic lt_o

    );


    typedef enum logic [SIZE_OP-1:0] {
        ADD = 0,
        SUB = 1,

        AND = 2,
        OR  = 3,
        XOR = 4,

        SLL = 5,
        SRL = 6,
        SRA = 7,

        SLT = 8

    } op_t;

    op_t op;

    reg [DATA_SIZE:0] result;

    assign out_o = result[DATA_SIZE-1:0];
    assign carry_o = result[DATA_SIZE];
    assign zero_o = ~(|result);

    assign op = op_t'(op_i);

    always_comb begin
        
        lt_o = 1'b0;
        result = '0;

        case(op)
            ADD:    result = a_i + b_i;
            SUB:    result = a_i - b_i;

            AND:    result = {1'b0, a_i & b_i};
            OR:     result = {1'b0, a_i | b_i};
            XOR:    result = {1'b0, a_i ^ b_i};

            SLL:    result = {1'b0, a_i << b_i[SIZE_LOG-1:0]};
            SRL:    result = {1'b0, a_i >> b_i[SIZE_LOG-1:0]};
            SRA:    result = {1'b0, signed'(a_i) >>> b_i[SIZE_LOG-1:0]};

            SLT:    if(a_i < b_i) lt_o = 1'b1;

            default: result = '0;
        endcase
    end


endmodule
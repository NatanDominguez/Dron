
module instr_mem #(
    parameter int DEPTH = 64,
    parameter int ADDR_SIZE = $clog2(DEPTH)
) (
    input  logic [ADDR_SIZE-1:0] addr_i,
    output logic [31:0] instr_o
);

    logic [31:0] memory [DEPTH];

    initial begin
        $readmemh("../../hw/src/program.hex", memory);
    end

    assign instr_o = memory[addr_i >> 2];

endmodule
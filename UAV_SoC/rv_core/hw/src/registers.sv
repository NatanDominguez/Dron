
module registers #(
    parameter int DATA_SIZE = 32,
    parameter int ADDR_NUM = 32,
    parameter int ADDR_SIZE = $clog2(ADDR_NUM)

) (
    input logic clk_i,
    input logic rst_ni,

    input logic stall_i,

    input logic [ADDR_SIZE-1:0] addr_ra_i,
    output logic [DATA_SIZE-1:0] data_ra_o,

    input logic [ADDR_SIZE-1:0] addr_rb_i,
    output logic [DATA_SIZE-1:0] data_rb_o,

    input logic [ADDR_SIZE-1:0] addr_w_i,
    input logic [DATA_SIZE-1:0] data_w_i,
    input logic                 we_i

);

    logic [DATA_SIZE-1:0] registers [ADDR_NUM];

    // READ PORTS
    assign data_ra_o = (addr_ra_i == '0) ? '0 : registers[addr_ra_i];
    assign data_rb_o = (addr_rb_i == '0) ? '0 : registers[addr_rb_i];

    // WRITE PORT
    integer i;
    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            for(i = 0; i < ADDR_NUM; i++) begin
                registers[i] <= '0;
            end
        end else begin
            if((addr_w_i != '0) && ~stall_i && we_i) begin
               registers[addr_w_i] <= data_w_i; 
            end
        end
    end

endmodule

module delta_counter #(
    parameter int SIZE = 8,
    parameter int DELTA = 1,
    parameter int START = 0,
    parameter int FINISH = 1,
    parameter int FACTOR = 1
) (
    input logic clk_i,
    input logic rst_ni,
    input logic ena_i,
    output logic [SIZE-1:0]   count_o
);

    logic [31:0] scale;

    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            count_o <= START;
            scale <= '0;
        end else begin
            if(ena_i) begin
                if(count_o < FINISH) begin
                    count_o <= count_o;
                end else begin
                    if(scale == FACTOR) begin
                        count_o <= count_o + DELTA;
                        scale <= '0;
                    end else begin
                        count_o <= count_o;
                        scale <= scale + 1;
                    end
                    
                end
            end
        end
    end

endmodule
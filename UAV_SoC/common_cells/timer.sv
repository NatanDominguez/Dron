
/*
    UP COUNTER TIMER
*/

module timer #(
    parameter SIZE = 16,
) (
    input logic clk_i,
    input logic rst_ni,
    input logic ena_i,
    input logic restart_i,
    input logic [SIZE-1:0] limit_i,
    output logic interrupt_o
);

    logic [SIZE-1:0]    count;

    assign pulse_o = ((count == limit_i) && ena_i) ? 1'b1 : 1'b0;
    
    always_ff @(posedge clk_i) begin
        if(rst_ni || restart_i) begin
            count <= '0;
        end else begin
            if(pulse_o) begin
                count <= '0;
            end else begin
                if(ena_i) begin
                    count <= count + 1;
                end
            end
        end
    end

endmodule

// FACTOR DOWN THE CLK FREQUENCY

module prescaler (
    input logic clk_i,
    input logic rst_ni,
    input logic factor_i,
    output logic clk_o
);

    logic [31:0] counter;

    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            counter <= 1'b0;
            clk_o <= 1'b0;
        end
        else begin
            counter <= counter + 1;

            if(counter == factor) begin
                clk_o <= clk_o ^ 1'b1;
                counter <= 1'b0;
            end
        end

    end


endmodule
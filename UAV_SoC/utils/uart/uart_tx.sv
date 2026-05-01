

module uart_tx (
    input clk_i,
    input rst_ni,

    input data_tx_i,
    

    output tx_o
);


    typedef enum logic[2:0] {
        IDLE = 3'b000,
        START = 3'b001,
        DATA = 3'b010,
        PB = 3'b011,
        STOP = 3'b100
    } state_e;

    state_e state, next;

    always @(posedge clk_i) begin
        if(!rst_ni) begin
            state <= IDLE;
        end else begin
            state <= next;
        end
    end

    always_comb begin
        
        next = IDLE;
        tx_o = 1'b1;

        begin
            case(state)
            IDLE: 

            START:

            DATA:

            PB:

            STOP:

            endcase
        end

    end




endmodule
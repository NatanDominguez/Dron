


module bldc_controller (
    input logic clk_i,
    input logic rst_ni,
    input logic [15:0] duty_i,
    input logic [2:0] zero_crossing_i,
    output logic [5:0] hbridge_o,
);


    parameter logic [1:0]   IDLE = 2'b00,
                            OPEN_LOOP = 2'b01,
                            TRANSITION = 2'b10,
                            CLOSED_LOOP = 2'b11;


    logic [1:0]     state, next;

    logic [5:0]     hbridge_status, ol_hbridge, cl_hbridge;
    logic           ena_cl_fsm, ena_ol_fsm;

    always_ff @(posedge clk_i) begin
        if(!rst_ni) state <= IDLE;
        else        state <= next;
    end

    always_comb begin
        next = state;

        case(state)

            OPEN_LOOP: begin
                if(rampup_end) next = TRANSITION;
            end

            TRANSITION: begin
                if(transition_end) next = CLOSED_LOOP;
            end

            default: next = IDLE;

        endcase
    end

    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            hbridge_status <= 6'b0;
            ena_ol_fsm <= 1'b0;
            ena_cl_fsm <= 1'b0;
        end else begin

            hbridge_status <= '0;
            ena_ol_fsm <= 1'b0;
            ena_cl_fsm <= 1'b0;

            case(state)

                OPEN_LOOP: begin
                    hbridge_status <= ol_hbridge;
                    ena_ol_fsm <= 1'b1;
                    ena_cl_fsm <= 1'b0;
                end

                TRANSITION: begin
                    hbridge_status <= ol_hbridge;
                    ena_ol_fsm <= 1'b1;
                    ena_cl_fsm <= 1'b0;
                end

                CLOSED_LOOP: begin
                    hbridge_status <= cl_hbridge;
                    ena_ol_fsm <= 1'b0;
                    ena_cl_fsm <= 1'b1;
                end

            endcase
        end
    end

    // GENERATE H-BRIDGE SIGNALS (PWM & PULSES)

    signal_gen i_bldc_signal_gen(
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .duty_i (duty_i),
        .status_i (hbridge_status),
        .signals_o (hbridge_o)
    );


          ///////////////
         // OPEN LOOP //
        ///////////////

    // THROUGH THIS TIMER THE SPEED IS SELECTED -> FSM SYNCHRONIZED WITH TIMER INTERRUPT
    timer i_timer_ol (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .ena_i (ena_ol_fsm),
        .restart_i (1'b0),
        .limit_i (speed_i),
        .interrupt_o (timer_ol_int)
    );

    bldc_ol_fsm i_bldc_ol_fsm (
        .clk_i (timer_ol_int),
        .rst_ni (rst_ni),
        .ena_i (ena_ol_fsm),
        .status_o (ol_hbridge)
    );

          /////////////////
         // CLOSED LOOP //
        /////////////////


endmodule

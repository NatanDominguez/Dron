


module bldc_controller #(
    parameter int unsigned FREQ_CLK = 1000000,
    parameter int unsigned RAMPUP_TIME = 5*FREQ_CLK, // in cycles 
    parameter int unsigned TRANSITION_TIME = 1*FREQ_CLK,
    parameter int unsigned SPEED_START = 100000,      // cycles/revolution
    parameter int unsigned SPEED_END = 1000,
    parameter int unsigned DUTY_START = 200,
    parameter int unsigned DUTY_END = 180,
    parameter int SPEED_DELTA = -2,
    parameter int DUTY_DELTA = (DUTY_END - DUTY_START)/RAMPUP_TIME
    ) (
    input logic clk_i,
    input logic rst_ni,
    input logic start_i,
    input logic [15:0] duty_i,          // DUTY INPUT FOR CLOSED LOOP
    input logic [2:0] zero_crossing_i,  // ZERO-CROSSING SIGNALS FOR CLOSED LOOP SYNCHRONIZATION
    output logic [5:0] hbridge_o       // OUTPUT SIGNALS TO DRIVE THE HBRIDGE
);


    parameter logic [1:0]   IDLE = 2'b00,
                            OPEN_LOOP = 2'b01,
                            TRANSITION = 2'b10,
                            CLOSED_LOOP = 2'b11;


    logic [1:0]     state, next;

    logic [5:0]     hbridge_status, ol_hbridge, cl_hbridge;
    logic           ena_cl_fsm, ena_ol_fsm;

    logic           rampup_on, rampup_end, transition_end;
    logic           rampup_timer_int, transition_timer_int;


    logic [15:0] duty, duty_ol;
    logic [31:0] speed;

    always_ff @(posedge clk_i) begin
        if(!rst_ni) state <= IDLE;
        else        state <= next;
    end

    always_comb begin
        next = state;

        case(state)

            IDLE: begin
                if(start_i) next = OPEN_LOOP;
            end

            OPEN_LOOP: begin
                if(rampup_end) next = TRANSITION;
            end

            TRANSITION: begin
                if(transition_end) next = CLOSED_LOOP;
            end

        endcase
    end

    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            hbridge_status <= 6'b0;
            ena_ol_fsm <= 1'b0;
            ena_cl_fsm <= 1'b0;
            rampup_end <= 1'b0;
            transition_end <= 1'b0;
        end else begin

            hbridge_status <= '0;
            ena_ol_fsm <= 1'b0;
            ena_cl_fsm <= 1'b0;

            rampup_on <= 1'b0;
            rampup_end <= rampup_timer_int;
            transition_end <= transition_timer_int;

            case(state)

                OPEN_LOOP: begin
                    hbridge_status <= ol_hbridge;
                    ena_ol_fsm <= 1'b1;
                    ena_cl_fsm <= 1'b0;
                    rampup_on <= 1'b1;
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

    // TIMERS TO GENERATE SEQUENCE || OPEN LOOP -> TRANSITION -> CLOSED LOOP

    timer #(
        .SIZE (32) 
    ) i_timer_rampup (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .ena_i (ena_ol_fsm),
        .restart_i (1'b0),
        .limit_i (RAMPUP_TIME),
        .interrupt_o (rampup_timer_int)
    );

    timer #(
        .SIZE (32) 
    ) i_timer_transition (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .ena_i (ena_ol_fsm),        // timer active when ol fsm working (output not used in open loop)
        .restart_i (rampup_end),    // restart this timer when the open loop finishes
        .limit_i (TRANSITION_TIME),
        .interrupt_o (transition_timer_int)
    );

    // GENERATE H-BRIDGE SIGNALS (PWM & PULSES)

    
    always_comb begin
        duty = ena_ol_fsm ? duty_ol : duty_i;
    end

    signal_gen i_bldc_signal_gen(
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .duty_i (duty),
        .status_i (hbridge_status),
        .signals_o (hbridge_o)
    );


          ///////////////
         // OPEN LOOP //
        ///////////////

    // DELTA COUNTERS TO CREATE RAMP-UP

    delta_counter #(
        .SIZE ( 32),
        .DELTA (SPEED_DELTA),
        .START (SPEED_START),
        .FINISH (SPEED_END),
        .FACTOR (100)
        ) i_delta_counter_speed (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .ena_i  (rampup_on),
        .count_o (speed)
    );

    delta_counter #(
        .SIZE  ( 16          ),
        .DELTA ( DUTY_DELTA ),
        .START ( DUTY_START ),
        .FINISH (DUTY_END),
        .FACTOR (100)
    )  i_delta_counter_duty (
        .clk_i   ( clk_i     ),
        .rst_ni  ( rst_ni    ),
        .ena_i   ( rampup_on ),
        .count_o ( duty_ol   )
    );


    // THROUGH THIS TIMER THE SPEED IS SELECTED -> FSM SYNCHRONIZED WITH TIMER INTERRUPT
    timer #(
        .SIZE (32)
        ) i_timer_ol (
        .clk_i        (clk_i),
        .rst_ni       (rst_ni),
        .ena_i        (ena_ol_fsm),
        .restart_i    (1'b0),
        .limit_i      (speed),
        .interrupt_o  (timer_ol_int)
    );

    bldc_ol_fsm i_bldc_ol_fsm (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .ena_i (ena_ol_fsm),
        .phase_jump (timer_ol_int),
        .status_o (ol_hbridge)
    );

    // TIMER TO FINISH THE RAMPUP

          /////////////////
         // CLOSED LOOP //
        /////////////////

    bldc_cl_fsm i_bldc_cl_fsm (
        .clk_i (clk_i),
        .rst_ni (rst_ni),
        .ena_i (ena_cl_fsm),
        .zero_crossing_i(zero_crossing_i),
        .status_o (cl_hbridge)
    );

endmodule



/*
THIS FSM IS USED TO GENERATE THE ELECTRICAL PHASES FOR THE BLDC CONTROL IN OPEN LOOP

THE CLK SIGNAL IS DIRECTLY RELATED TO THE BLDC SPEED, SO IT HAS TO BE DRIVEN BY A
PROGRAMABLE COUNTER THAT GENERATES THE SPEED

ITS ONLY OUTPUT IS USED TO GIVE INFORMATION ON THE SIGNAL DRIVEN TO THE H-BRIDGE 
EACH BITS CORRESPOND TO A PORT -> HA : (0 -> low, 1 -> high/pwm)

            HA: --------________________
            LA: ____________--------____

            HB: ________--------________
            LB: ----________________----

            HC: ________________--------
            LC: ____--------____________

*/

module bldc_ol_fsm (
    input logic clk_i,
    input logic rst_ni,
    input logic ena_i,

    input logic phase_jump,

    output logic [5:0] status_o  // 6'b_AHS-ALS-BHS-BLS-CHS-CLS
);

    parameter [2:0]     IDLE = 3'b000,
                        P_1 = 3'b001,
                        P_2 = 3'b010,
                        P_3 = 3'b011,
                        P_4 = 3'b100,
                        P_5 = 3'b101,
                        P_6 = 3'b110;
    
    logic [2:0] phase, next;

    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            phase <= IDLE;
        end else begin
            if(phase_jump)  phase <= next;
        end     
    end

    always_comb begin
        case(phase)
            IDLE:       next = P_1;
            P_1:        next = P_2;
            P_2:        next = P_3;
            P_3:        next = P_4;
            P_4:        next = P_5;
            P_5:        next = P_6;
            P_6:        next = P_1;
        endcase
    end

    always_ff @(posedge clk_i) begin
        if(!rst_ni) begin
            status_o <= '0;
        end else begin

            status_o <= '0;

            case(phase) 
                IDLE:   status_o <= '0;

                P_1:    status_o <= 6'b100100;  // PWMA || LB
                P_2:    status_o <= 6'b100001;  // PWMA || LC
                P_3:    status_o <= 6'b001001;  // PWMB || LC
                P_4:    status_o <= 6'b011000;  // PWMB || LA
                P_5:    status_o <= 6'b010010;  // PWMC || LA
                P_6:    status_o <= 6'b000110;  // PWMC || LB
            endcase
        end
    end

endmodule





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



            DUE TO POSSIBLE NOISE IN ZERO-CROSSING SIGNALS -> ADD MODULE TO APPLY GATING GIVEN THE STATUS
*/


module bldc_cl_fsm (
    input logic clk_i,
    input logic rst_ni,
    input logic ena_i,

    input logic [2:0] zero_crossing_i, // A [2] / B[1] / C[0]

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
    logic [2:0] zero_crossing_prev;
    logic [2:0] edge_rise_detect; //RISE DETECTION ZERO CROSS      _________-----------
    logic [2:0] edge_falling_detect; //FALLING DETECTION ZERO CROSS ---------___________

    always_ff @(posedge clk_i) begin 
        if(!rst_ni) begin
            phase <= IDLE;
        end else begin
            phase <= next;
        end  
    end


    always_ff @(posedge clk_i) begin  //edge detectation UP/DOWN
        if (!rst_ni) begin
            zero_crossing_prev <= '0;
            edge_rise_detect    <= '0;
            edge_falling_detect <= '0;
        end else begin
                edge_rise_detect <= ~zero_crossing_prev & zero_crossing_i;
                edge_falling_detect <= zero_crossing_prev & ~zero_crossing_i;
                zero_crossing_prev <= zero_crossing_i;
            end
            
        end



    always_comb begin
        case(phase)
            IDLE: next = P_1;
            P_1: if (edge_falling_detect[0]) next = P_2;
            P_2: if (edge_rise_detect[1]) next = P_3;
            P_3: if (edge_falling_detect[2]) next = P_4;
            P_4: if (edge_rise_detect[0]) next = P_5;
            P_5: if (edge_falling_detect[1]) next = P_6;
            P_6: if (edge_rise_detect[2]) next = P_1;
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
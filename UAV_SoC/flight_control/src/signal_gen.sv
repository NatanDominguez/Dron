
/*
SIGNAL GENERATION FOR THE H-BRIDGES

HS -> PWM       (WHEN STATUS[i] IS ON)
LS -> PULSE         """"
*/


module signal_gen #(
    parameter int PWM_BASE = 1000,
    parameter int PWM_FREQ = 10000,
    parameter int CLK_FREQ = 1000000
    ) (
    input logic clk_i,
    input logic rst_ni,
    input logic [15:0] duty_i,
    input logic [5:0] status_i,
    output logic [5:0] signals_o
);

    logic pwm_ha,pwm_hb,pwm_hc;

    // H-BRIDGE LOWSIDE SIGNALS     ((PULSE))

    assign signals_o[4] = status_i[4];
    assign signals_o[2] = status_i[2];
    assign signals_o[0] = status_i[0];


    // H-BRIDGE HIGHSIDE SIGNALS    ((PWM))

    assign signals_o[5] = pwm_ha;
    assign signals_o[3] = pwm_hb;
    assign signals_o[1] = pwm_hc;


    pwm_gen #(
        .SIZE (16)
        ) i_pwm_gen_a (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .duty_i (duty_i),
        .ena_i  (status_i[5]),
        .pwm_o  (pwm_ha)
    );

    pwm_gen #(
        .SIZE (16)
        ) i_pwm_gen_b (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .duty_i (duty_i),
        .ena_i  (status_i[3]),
        .pwm_o  (pwm_hb)
    );

    pwm_gen #(
        .SIZE (16)
        ) i_pwm_gen_c (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .duty_i (duty_i),
        .ena_i  (status_i[1]),
        .pwm_o  (pwm_hc)
    );

endmodule
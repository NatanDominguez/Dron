




module pwm_gen #(
    parameter int BASE = 100,   // Ticks máximo (periodo)
    parameter int SIZE = 8
) (
    input  logic              clk_i,   // Reloj PWM
    input  logic              rst_ni,  // Reset activo bajo
    input  logic [SIZE-1:0]   duty_i,    // Ciclo de trabajo
    input  logic              ena_i,   // Enable
    output logic              pwm_o    // Señal PWM
);

    logic [SIZE-1:0] ticks;

    // Señal PWM: activa mientras ticks <= duty y enable activo
    assign pwm_o = (ena_i && (ticks < duty_i)) ? 1'b1 : 1'b0;

    // Contador de ticks
always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni || !ena_i) begin
        ticks <= '0;
    end else begin
        ticks <= (ticks >= BASE - 1) ? '0 : ticks + 1;
    end
end

endmodule

import core_pkg::*;
import riscv_pkg::*;

module tb_sys;

    parameter int DATA_SIZE = 32;
    parameter int NUM_OP = 9;
    parameter int SIZE_OP = $clog2(NUM_OP);
    parameter int ADDR_SIZE = 32;
    parameter int INST_DEPTH = 64;
    parameter int PC_W = 32;
    parameter int RAM_ADDR_SIZE = 32;


    logic clk, rst_n;

    logic [31:0] inst;
    logic [PC_W-1:0] pc;

    logic re_x, we_x, rx_valid;
    logic [DATA_SIZE-1:0] data_rx, data_wx;
    logic [ADDR_SIZE-1:0] addr_x;

    logic re_ram, we_ram, ram_r_valid;
    logic [ADDR_SIZE-1:0] ram_addr;
    logic [DATA_SIZE-1:0] ram_r_data, ram_w_data;

    logic re_uart, we_uart, uart_r_valid;
    logic [ADDR_SIZE-1:0] uart_addr;
    logic [DATA_SIZE-1:0] uart_r_data, uart_w_data;



    core i_core (
        .clk_i      ( clk      ),
        .rst_ni     ( rst_n    ),

        .inst_i     ( inst     ),
        .pc_o       ( pc       ),
        
        .addr_x_o    ( addr_x  ),
        .re_x_o     ( re_x     ),
        .data_rx_i  ( data_rx  ),
        .rx_valid_i ( rx_valid ),

        .we_x_o     ( we_x     ),
        .data_wx_o  ( data_wx  )
    );

    data_controller #(
        .DATA_SIZE (DATA_SIZE),
        .ADDR_SIZE (ADDR_SIZE)
        
        ) i_data_controller (
        .addr_i ( addr_x ),
        
        .re_x_i ( re_x ),
        .data_rx_o ( data_rx ),
        .rx_valid_o  ( rx_valid),

        .we_x_i (we_x),
        .data_wx_i (data_wx),

        // RAM
        .ram_addr_o (ram_addr),
        .ram_r_data_i (ram_r_data),
        .ram_r_valid_i (ram_r_valid),
        .ram_re_o (re_ram),
        .ram_w_data_o(ram_w_data),
        .ram_we_o (we_ram),

        // UART
        .uart_addr_o (uart_addr),
        .uart_r_data_i (uart_r_data),
        .uart_r_valid_i (uart_r_valid),
        .uart_re_o(re_uart),
        .uart_w_data_o(uart_w_data),
        .uart_we_o(we_uart)

    );

    instr_mem #(
        .DEPTH (INST_DEPTH)
    ) i_instr_mem (
        .addr_i  (pc),
        .instr_o (inst)
    );

    ram i_ram (
        .clk_i          (clk),
        .rst_ni         (rst_n),
        .re_i           (re_ram),
        .addr_r_i       (ram_addr),
        .data_r_o       (ram_r_data),
        .data_r_valid   (ram_r_valid),
        .we_i           (we_ram),
        .addr_w_i       (ram_addr),
        .data_w_i       (ram_w_data)
    );


    uart #(
        .DATA_SIZE(DATA_SIZE),
        .ADDR_SIZE(ADDR_SIZE)  
    ) i_uart (
        .clk_i (clk),
        .rst_ni (rst_n),
        .addr_i (uart_addr),
        .data_w_i (uart_w_data),
        .we_i (we_uart),
        .data_r_o(uart_r_data),
        .r_valid_o(uart_r_valid),
        .re_i(re_uart)
    );

    


    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
    end

    always #5 clk = ~clk;

    always begin

        repeat(5) @(posedge clk);
        rst_n = 1'b1;

        repeat(256) @(posedge clk);

        $finish;
    end


    always @(posedge clk) begin
        if(we_x && (addr_x == 32'h70000000)) begin
            $write("%c", data_wx);
        end
    end

endmodule
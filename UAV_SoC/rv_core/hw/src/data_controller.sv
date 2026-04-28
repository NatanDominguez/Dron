

module data_controller #(
    parameter int DATA_SIZE = 32,
    parameter int ADDR_SIZE = 32
) (
    input logic [ADDR_SIZE-1:0] addr_i,

    input logic re_x_i,
    output logic [DATA_SIZE-1:0] data_rx_o,
    output logic rx_valid_o,

    input logic we_x_i,
    input logic [DATA_SIZE-1:0] data_wx_i,

    output logic [ADDR_SIZE-1:0] ram_addr_o,

    output logic ram_re_o,
    input logic [DATA_SIZE-1:0] ram_r_data_i,
    input logic ram_r_valid_i,

    output logic ram_we_o,
    output logic [DATA_SIZE-1:0] ram_w_data_o,


        // UART
    output logic [ADDR_SIZE-1:0] uart_addr_o,
    input logic [DATA_SIZE-1:0] uart_r_data_i,
    input logic uart_r_valid_i,
    output logic uart_re_o,
    output logic [DATA_SIZE-1:0] uart_w_data_o,
    output logic uart_we_o
    
);

    logic sel;

    always_comb begin
        if(addr_i >= 32'h00000000 && addr_i <= 32'h3FFFFFFF) sel = 1'b0;
        else sel = 1'b1;
    end

    always_comb begin
        ram_addr_o = '0;
        ram_re_o = 1'b0;
        data_rx_o = '0;
        rx_valid_o = 1'b0;
        ram_w_data_o = '0;
        ram_we_o = 1'b0;
        uart_addr_o = '0;
        uart_re_o = 1'b0;
        uart_w_data_o = '0;
        uart_we_o = 1'b0;
        case(sel)
            1'b0:
            begin
                ram_addr_o = addr_i;
                if(re_x_i) begin
                    ram_re_o   = re_x_i;
                    data_rx_o  = ram_r_data_i;
                    rx_valid_o = ram_r_valid_i; 
                end
                if(we_x_i) begin
                    ram_we_o     = we_x_i;
                    ram_w_data_o = data_wx_i;
                end
            end
            1'b1:
            begin
                uart_addr_o = addr_i;
                if(re_x_i) begin
                    uart_re_o   = re_x_i;
                    data_rx_o  = uart_r_data_i;
                    rx_valid_o = uart_r_valid_i; 
                end
                if(we_x_i) begin
                    uart_we_o     = we_x_i;
                    uart_w_data_o = data_wx_i;
                end
            end

        endcase
    end



endmodule
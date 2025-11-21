
`timescale 1ns / 10ps


module uav_testbench();


    logic clk;
    logic rst_n;
    logic start;

    logic [15:0] duty;
    logic [2:0] zero_crossing;
    logic [5:0] hbridge;


    bldc_controller i_bldc_controller (
        .clk_i (clk),
        .rst_ni (rst_n),
        .start_i (start),
        .duty_i (duty),
        .zero_crossing_i (zero_crossing),
        .hbridge_o (hbridge)
    );


    initial begin
        clk = 1'b0;
        rst_n = 1'b1;
        start = 1'b0;
    end

    always #500 clk = ~clk;

    initial begin
        
        #10000;
        rst_n = 1'b0;

        #10000;
        rst_n = 1'b1;
        start = 1'b1;

        #100000;


    end

    initial begin
        zero_crossing = 3'b001;

        forever begin
            zero_crossing[2] = 1'b1;
            #500000;
            zero_crossing[0] = 1'b0;
            #500000;
            zero_crossing[1] = 1'b1;
            #500000;
            zero_crossing[2] = 1'b0;
            #500000;
            zero_crossing[0] = 1'b1;
            #500000;
            zero_crossing[1] = 1'b0;
            #500000;
        end
    end

    initial begin
        duty = 10;

        forever begin
            #1000000
            if(duty < 100) begin
                duty = duty + 10;
            end else begin
                duty = 10;
            end
        end
    end



endmodule
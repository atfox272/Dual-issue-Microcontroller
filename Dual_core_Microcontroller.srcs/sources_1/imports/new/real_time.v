`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/04/2023 04:55:43 PM
// Design Name: 
// Module Name: real_time
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module real_time
    #(
    parameter  MAX_COUNTER       = 10000,
    localparam MAX_COUNTER_WIDTH = $clog2(MAX_COUNTER)
    )
    (
    input                           clk,
    input                           counter_enable,
    input [MAX_COUNTER_WIDTH - 1:0] limit_counter,
    output                          limit_flag,
    
    input rst_n
    );
    
    reg [MAX_COUNTER_WIDTH - 1:0] counter;
    reg                           limit_flag_reg;
    
    assign limit_flag = limit_flag_reg;
    
    logic[MAX_COUNTER_WIDTH - 1:0]  counter_next;
    logic                           limit_flag_next;
    always_comb begin
        counter_next = counter + 1;
        limit_flag_next = 0;
        if(!counter_enable) begin
            counter_next = 0;
            limit_flag_next = 0;
        end
        else begin
            if(counter == limit_counter) begin
                counter_next = 0;
                limit_flag_next = 1;
            end
        end
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            counter <= 0;
            limit_flag_reg <= 0;
        end
        else begin
            limit_flag_reg <= limit_flag_next;
            counter <= counter_next;
        end
    end
    
    
endmodule

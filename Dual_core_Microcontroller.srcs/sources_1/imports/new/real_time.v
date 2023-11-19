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
    parameter MAX_COUNTER       = 10000,
    parameter MAX_COUNTER_WIDTH = $clog2(MAX_COUNTER)
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
    
    always @(posedge clk) begin
        if(!rst_n) begin
            counter <= 0;
            limit_flag_reg <= 0;
        end
        else if(counter_enable) begin
            limit_flag_reg <= (counter == limit_counter) ? 1'b1 : limit_flag_reg;   // Greater than => Set high
            counter <= counter + 1;
        end
        else begin
            limit_flag_reg <= 1'b0;
            counter <= 0;
        end
    end
    
    
endmodule

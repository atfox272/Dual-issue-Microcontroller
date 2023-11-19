`timescale 1ns / 1ns
module edge_detector(
    input clk,
    input sig_in,
    input rst_n,
    output reg out
    );
    reg [1:0] sig_in_counter;
    reg [1:0] rep_counter;
    reg dur_counter;
    reg rising;
    
    wire balance;
    wire [1:0] state;
    
    assign balance = (rep_counter == sig_in_counter);
    assign state = {dur_counter, !balance};
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            out <= 0;
            dur_counter <= 0;
            rep_counter <= 0;
        end
        else begin
            case (state) 
                2'b00: out <= 0;
                2'b01: begin
                    dur_counter <= dur_counter + 1;
                    rep_counter <= rep_counter + 1;
                    out <= 1;
                end
                default: begin                      // case 2'b1x
                    dur_counter <= 0;
                    out <= 0;
                end
            endcase
        end
    end
//    always @(posedge sig_in, negedge sig_in) begin
//        if(sig_in) rising <= 1'b1;
//        else rising <= 1'b0;
//    end
    always @(posedge sig_in, negedge rst_n) begin
        if(!rst_n) sig_in_counter <= 0;
        else sig_in_counter <= sig_in_counter + 1;
    end
endmodule

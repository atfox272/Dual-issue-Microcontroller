`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/09/2023 07:37:12 PM
// Design Name: 
// Module Name: timing_uart_peripheral_file
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
//`define TX_TESTING
//`define RX_TESTING
`define FULLDUPLEX_TESTING
module timing_uart_peripheral_file
    #(
    parameter INTERNAL_CLOCK   = 125000000
    )
    (
    input clk,
    
    input   RX,
    output  TX,
    
//    `ifdef FULLDUPLEX_TESTING
    output  RX_sub,
    output  RX_available_sub,
    output [2:0]  debug,              // AUX_2 in logic analyzer
//    `endif
    
    input rst
    );
    
//    localparam INTERNAL_CLOCK   = 125000000;
    localparam BD9600_8N1       = 8'b00100011;
    localparam BD115200_8N1     = 8'b11100011;
    
    localparam WAITING_TIME     = 1000000;    
    
    wire [7:0]   data_in;
    wire [7:0]   data_out;
    reg         TX_use;
    reg         RX_use;
//    wire        send_flag;
    wire        TX_available;
    wire        RX_flag;
    wire        RX_full;
    wire RX_available;
// (* dont_touch = "yes" *)    
uart_peripheral 
        #(
        .INTERNAL_CLOCK(INTERNAL_CLOCK),
        .RX_FLAG_TYPE(0)
        )uart_peripheral(
        .clk(clk),
        .RX(RX),
        .TX(TX),
        .RX_config_register(BD9600_8N1),
        .TX_config_register(BD9600_8N1),
        // TX interface 
        .data_in(data_in),
        .TX_use(TX_use),
        .TX_flag(),
        .TX_complete(),
        .TX_available(TX_available),
        // RX interface
        .data_out(data_out),
        .RX_use(RX_use),
        .RX_flag(RX_flag),
        .debug(debug),
        .RX_available(RX_available),
        
        .rst_n(~rst)
        );   
    `ifdef TX_TESTING    
//    localparam INIT_STATE = 2'd0;
//    localparam IDLE_STATE = 2'd1;
//    localparam TRANS_STATE = 2'd3;
//    localparam CONFIRM_STATE = 2'd2;
    
//    reg [1:0] state;
//    reg [7:0] count;
//    reg [7:0] buffer;
//    assign data_in = buffer;
//    always @(posedge clk) begin
//        if(rst) begin
//            count <= 0;
//            buffer <= 0;
//            TX_use <= 0;
//            state <= INIT_STATE;
//        end
//        else begin
//            case(state) 
//                INIT_STATE: begin
//                    if(RX_flag) state <= IDLE_STATE;
//                end
//                IDLE_STATE: begin
//                    if(TX_available) begin
//                        state <= TRANS_STATE;
//                        buffer <= count;
//                    end
//                end
//                TRANS_STATE: begin
//                    state <= CONFIRM_STATE;
//                    TX_use <= 1;
//                    count <= count + 1;
//                end
//                CONFIRM_STATE: begin
//                    state <= IDLE_STATE;
//                    TX_use <= 0;
//                end
//            endcase 
//        end
//    end
//    `elseif TX_TESTING
    `endif
    
    `ifdef FULLDUPLEX_TESTING
    (* keep = "true" *)reg [1:0] state;    
    (* keep = "true" *)reg [7:0] buffer;
    reg baudrate_clk_en_rx_toggle;
    localparam IDLE_STATE = 0;
    localparam INIT_STATE = 3;
    localparam SEND_STATE = 1;
    localparam CONFIRM_SEND_STATE = 2;
    assign RX_available_sub = ~RX_available;
    assign RX_sub = RX;
    assign data_in = buffer;    
    always @(posedge clk) begin
        if(rst) begin
            state <= IDLE_STATE;
            TX_use <= 0;
            RX_use <= 0;
            buffer <= 0;
        end
        else begin
            case(state) 
                INIT_STATE: begin
                    if(RX_full) state <= IDLE_STATE;
                end
                IDLE_STATE: begin
                    if(RX_flag) begin
                        state <= SEND_STATE;
                        buffer <= data_out;
                    end
                end
                SEND_STATE: begin
                    RX_use <= 1;
                    TX_use <= 1;
                    state  <= CONFIRM_SEND_STATE;
                end
                CONFIRM_SEND_STATE: begin
                    RX_use <= 0;
                    TX_use <= 0;
                    state  <= IDLE_STATE;
                end 
            endcase 
//            if(RX_flag) begin
//                TX_use <= 1;
//                RX_use <= 1;
//            end
//            else begin
//                TX_use <= 0;
//                RX_use <= 0;
//            end
        end
    end
//    always @(posedge RX_full) begin
//        if(rst) begin
//            baudrate_clk_en_rx_toggle <= 0;
//        end
//        else begin
//            baudrate_clk_en_rx_toggle <= ~baudrate_clk_en_rx_toggle;
//        end
//    end
    `endif           
endmodule

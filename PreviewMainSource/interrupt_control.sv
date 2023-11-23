module Interrupt_controller
    #(
    parameter INT_REQUEST_BUFFER = 32
    )
    (
    // Multi-processor manager
    input   wire    clk,
    
    output  wire    interrupt_flag_1,
    output  wire    interrupt_flag_2,
    output  wire    interrupt_flag_3,
    input   wire    RETI_1,
    input   wire    RETI_2,
    input   wire    RETI_3,
    input   wire    interrupt_handling_1,
    input   wire    interrupt_handling_2,
    input   wire    interrupt_handling_3,
    // Interrupt unit
    input   wire    interrupt_request_1,
    input   wire    interrupt_request_2,
    input   wire    interrupt_request_3,
    // Confuration register
    input   wire    interrupt_enable_1,
    input   wire    interrupt_enable_2,
    input   wire    interrupt_enable_3,

    input   wire    rst_n
    );
    
    wire interrupt_request_1_empty;
    wire interrupt_request_2_empty;
    wire interrupt_request_3_empty;
    
    assign interrupt_flag_1 = (interrupt_enable_1) ? ~interrupt_request_1_empty : 1'b0;
    assign interrupt_flag_2 = (interrupt_enable_2) ? ~interrupt_request_2_empty : 1'b0;
    assign interrupt_flag_3 = (interrupt_enable_3) ? ~interrupt_request_3_empty : 1'b0;
    
    sync_fifo 
        #(
        .FIFO_DEPTH(INT_REQUEST_BUFFER),
        .DATA_WIDTH(1)
        )interrupt_request_buffer_1(
        .clk(clk),
        .wr_req(interrupt_request_1),
        .rd_req(RETI_1),
        .empty(interrupt_request_1_empty),
        .data_in(),
        .data_out(),
        .full(),
        .almost_full(),
        .almost_empty(),
        .counter_threshold(),
        .counter_threshold_flag(),        
        .rst_n(rst_n)
        );
    sync_fifo 
        #(
        .FIFO_DEPTH(INT_REQUEST_BUFFER),
        .DATA_WIDTH(1)
        )interrupt_request_buffer_2(
        .clk(clk),
        .wr_req(interrupt_request_2),
        .rd_req(RETI_2),
        .empty(interrupt_request_2_empty),
        .data_in(),
        .data_out(),
        .full(),
        .almost_full(),
        .almost_empty(),
        .counter_threshold(),
        .counter_threshold_flag(),        
        .rst_n(rst_n)
        );
    sync_fifo 
        #(
        .FIFO_DEPTH(INT_REQUEST_BUFFER),
        .DATA_WIDTH(1)
        )interrupt_request_buffer_3(
        .clk(clk),
        .wr_req(interrupt_request_3),
        .rd_req(RETI_3),
        .empty(interrupt_request_3_empty),
        .data_in(),
        .data_out(),
        .full(),
        .almost_full(),
        .almost_empty(),
        .counter_threshold(),
        .counter_threshold_flag(),        
        .rst_n(rst_n)
        );
                  
endmodule

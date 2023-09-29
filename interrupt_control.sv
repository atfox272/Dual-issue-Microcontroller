module Interrupt_control
    #(
    parameter INT_REQUEST_BUFFER = 32
    )
    (
    // Multi-processor manager
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
    
    fifo_module #(
                .DEPTH(INT_REQUEST_BUFFER),
                .WIDTH(1),
                .SLEEP_MODE(1'b1)
                )interrupt_request_buffer_1(
                .write_ins(interrupt_request_1),
                .read_ins(RETI_1),
                .empty(interrupt_request_1_empty),
                .enable(interrupt_enable_1),
                .rst_n(rst_n)
                );
    
    fifo_module #(
                .DEPTH(INT_REQUEST_BUFFER),
                .WIDTH(1),
                .SLEEP_MODE(1'b1)
                )interrupt_request_buffer_2(
                .write_ins(interrupt_request_2),
                .read_ins(RETI_2),
                .empty(interrupt_request_2_empty),
                .enable(interrupt_enable_2),
                .rst_n(rst_n)
                );   
    
    fifo_module #(
                .DEPTH(INT_REQUEST_BUFFER),
                .WIDTH(1),
                .SLEEP_MODE(1'b1)
                )interrupt_request_buffer_3(
                .write_ins(interrupt_request_3),
                .read_ins(RETI_3),
                .empty(interrupt_request_3_empty),
                .enable(interrupt_enable_3),
                .rst_n(rst_n)
                );                
endmodule

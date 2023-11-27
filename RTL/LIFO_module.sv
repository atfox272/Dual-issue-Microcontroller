module LIFO_module
    #(
    parameter DATA_WIDTH = 8,
    parameter LIFO_DEPTH = 32,
    
    parameter POINTER_WIDTH = $clog2(LIFO_DEPTH)
    )
    (
    input   wire                        active_clk,
    
    input   wire    [DATA_WIDTH - 1:0]  data_bus_in,
    output  wire    [DATA_WIDTH - 1:0]  data_bus_out,
    
    input   wire                        rd_ins,
    input   wire                        wr_ins,
    
    output  wire                        full,
    output  wire                        almost_full,
    
    output  wire                        empty,
    output  wire                        almost_empty,

    input   wire                        rst_n
    
    // Debug 
    ,output wire    [DATA_WIDTH - 1:0]  stack_wire [0:LIFO_DEPTH - 1]
    );
    
    reg [DATA_WIDTH - 1:0]      stack   [0:LIFO_DEPTH - 1];
    reg [POINTER_WIDTH - 1:0]   top_stack;
    
    // Data
    assign data_bus_out = stack[top_stack - 1];
    // State
    assign empty = (top_stack == 0);
    assign almost_empty = (top_stack == 1);
    assign full = (&top_stack[POINTER_WIDTH - 1:0] == 1);
    assign almost_full = (&top_stack[POINTER_WIDTH - 1:1] == 1 & top_stack[0] == 0);
    
    always @(posedge active_clk, negedge rst_n) begin
        if(!rst_n) begin
            top_stack <= 0;
        end
        else begin
            if(rd_ins) begin
                top_stack <= (empty) ? top_stack : top_stack - 1;            
            end
            if(wr_ins) begin
                if(!full) begin
                    stack[top_stack] <= data_bus_in;
                    top_stack <= top_stack + 1;
                end
            end
        end
    end
    // Debug 
    for(genvar i = 0; i < LIFO_DEPTH; i = i + 1) begin
        assign stack_wire[i] = stack[i];
    end
endmodule    
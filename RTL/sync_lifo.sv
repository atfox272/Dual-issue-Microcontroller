module sync_lifo
    #(
    parameter DATA_WIDTH = 8,
    parameter LIFO_DEPTH = 32
    )
    (
    input                       clk,
    
    input  [DATA_WIDTH - 1:0]   data_i,
    output [DATA_WIDTH - 1:0]   data_o,
    
    input                       rd_req,
    input                       wr_req,
    
    output                      full,
    output                      empty,
    output                      almost_full,
    output                      almost_empty,
    
    input                       enable,
    
    input rst_n
    );
    localparam STACK_POINTER_WIDTH = $clog2(LIFO_DEPTH);
    
    reg   [DATA_WIDTH - 1:0]          registers [0:LIFO_DEPTH - 1];
    reg   [STACK_POINTER_WIDTH - 1:0] stack_pointer;
    logic [STACK_POINTER_WIDTH - 1:0] stack_pointer_incr;
    logic [STACK_POINTER_WIDTH - 1:0] stack_pointer_decr;
    logic [STACK_POINTER_WIDTH - 1:0] stack_pointer_next;
//    wire                              rd_sig;
//    wire                              wr_sig;
//    edgedet 
//        #(
//        .IDLE_INPUT_STATE()
//        )edgedet_rd(
//        .clk(clk),
//        .i(rd_req),
//        .o(rd_sig),
//        .rst_n(rst_n)
//        );
//    edgedet 
//        #(
//        .IDLE_INPUT_STATE()
//        )edgedet_wr(
//        .clk(clk),
//        .i(wr_req),
//        .o(wr_sig),
//        .rst_n(rst_n)
//        );
            
    assign data_o = registers[stack_pointer_decr];
    assign full  = stack_pointer == {STACK_POINTER_WIDTH{1'b1}};
    assign empty = stack_pointer == {STACK_POINTER_WIDTH{1'b0}};
    assign almost_full  = stack_pointer == {{(STACK_POINTER_WIDTH - 1){1'b1}}, 1'b0};
    assign almost_empty = stack_pointer == {{(STACK_POINTER_WIDTH - 1){1'b0}}, 1'b1};
    assign stack_pointer_incr = stack_pointer + 1'b1;
    assign stack_pointer_decr = stack_pointer - 1'b1;
    
    logic [DATA_WIDTH - 1:0] registers_next [0:LIFO_DEPTH - 1];
    always_comb begin
        for(int i = 0; i < LIFO_DEPTH; i = i + 1) begin
        registers_next[i] = registers[i];
        end
        stack_pointer_next = stack_pointer;
        case({rd_req, wr_req})
            2'b11: begin        // Read-Write
                stack_pointer_next = stack_pointer;
                registers_next[stack_pointer_decr] = data_i;
            end
            2'b01: begin        // Write
                if(!full) begin
                stack_pointer_next = stack_pointer_incr;
                registers_next[stack_pointer] = data_i;
                end
            end
            2'b10: begin        // Read 
                stack_pointer_next = (!empty) ? stack_pointer_decr : stack_pointer;
            end
        endcase
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            stack_pointer <= 0;
        end
        else if(enable) begin
            for(int i = 0; i < LIFO_DEPTH; i = i + 1) begin
            registers[i] = registers_next[i];
            end
            stack_pointer <= stack_pointer_next;
        end
    end
    
endmodule
//    sync_lifo
//        #(
//        .DATA_WIDTH(DATA_WIDTH),
//        .LIFO_DEPTH(LIFO_DEPTH)
//        )sync_lifo(
//        .clk(clk),
//        .data_i(),
//        .data_o(),
//        .rd_req(),
//        .wr_req(),
//        .full(),
//        .empty(),
//        .almost_full(),
//        .almost_empty(),
//        .enable(1'b1),
//        .rst_n(rst_n)
//        );
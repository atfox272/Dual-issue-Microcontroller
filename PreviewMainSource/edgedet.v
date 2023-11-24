module edgedet
    #(
    parameter IDLE_INPUT_STATE = 0
    )
    (
    input   clk,
    input   i,
    output  o,
    input   rst_n
    );
    reg prev_i;
    always @(posedge clk) begin
        if(!rst_n) prev_i <= IDLE_INPUT_STATE;
        else prev_i <= i;
    end
    assign o = ~prev_i & i;
endmodule
//edgedet 
//    #(
//    .IDLE_INPUT_STATE()
//    )edgedet(
//    .clk(),
//    .i(),
//    .o(),
//    .rst_n()
//    );

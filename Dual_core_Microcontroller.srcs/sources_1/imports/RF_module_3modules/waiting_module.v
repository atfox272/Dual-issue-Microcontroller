module waiting_module
    #(
    parameter END_COUNTER = 156250,
    parameter START_COUNTER = 0,
    parameter WAITING_TYPE = 0, // 0: Counting HIGH or LOW pulse
                                // 1: Counting between "start" and "stop" 
    parameter LEVEL_PULSE = 1,  // 0: count when LOW level
                                // 1: count when HIGH level                           
    parameter LIMIT_COUNTER_WIDTH = $clog2(END_COUNTER)
    )
    (
    input clk,
    input start_counting,
    input stop_counting,
    input rst_counting,
    input rst_n,
    output reg reach_limit
    // Debug 
//    ,output [LIMIT_COUNTER_WIDTH - 1:0] counter_wire
    );
    reg [LIMIT_COUNTER_WIDTH - 1:0] counter;
    reg start_counting_sync;
    reg stop_counting_sync;
    reg rst_counting_sync;    
    reg [1:0] state_counter;

    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            start_counting_sync <= 0;
            stop_counting_sync <= 0;
            rst_counting_sync <= 0;
        end
        else begin
            start_counting_sync <= start_counting;
            stop_counting_sync <= stop_counting; 
            rst_counting_sync <= rst_counting; 
        end
        
    end
    
	generate
        if(WAITING_TYPE) begin
            localparam IDLE_STATE = 0;
            localparam COUNTING_STATE = 1;
            localparam STOP_STATE = 2;
            always @(posedge clk, negedge rst_n) begin
                if(!rst_n) begin
                    counter <= START_COUNTER;
                    state_counter <= IDLE_STATE;
                    reach_limit <= 0;
                end
                else begin
                    case(state_counter)
                        IDLE_STATE: begin
                            if(start_counting_sync) begin
                                state_counter <= COUNTING_STATE;
                                counter <= counter + 1;
                            end
                            reach_limit <= 0;
                        end
                        COUNTING_STATE: begin
                            if(rst_counting_sync) begin
                                state_counter <= IDLE_STATE;
                                counter <= START_COUNTER;
                            end
                            else if(counter == END_COUNTER) begin
                                state_counter <= IDLE_STATE;
                                counter <= START_COUNTER;
                                reach_limit <= 1;
                            end
                            else if(stop_counting_sync) begin
                                state_counter <= STOP_STATE;
                            end 
                            else counter <= counter + 1; 
                        end
                        STOP_STATE: begin
                            if(rst_counting_sync) begin
                                state_counter <= IDLE_STATE;
                                counter <= START_COUNTER;
                            end
                            else if(start_counting_sync) begin
                                state_counter <= COUNTING_STATE;
                                counter <= counter + 1;
                            end
                        end
                        
                    endcase 
                end
            end
        end
        else begin
            always @(posedge clk, negedge rst_n) begin
                if(!rst_n) begin
                    counter <= START_COUNTER;
                    reach_limit <= 0;
                end
                else begin
                    if(start_counting == LEVEL_PULSE) begin
                        if(counter == END_COUNTER) begin
                            counter <= START_COUNTER;
                            reach_limit <= 1;
                        end 
                        else begin
                            counter <= counter + 1;
                            reach_limit <= 0;
                        end
                    end
                    else begin
                        counter <= START_COUNTER;
                        reach_limit <= 0;
                    end
                end
            end
        end
    endgenerate
    // debug area
//    assign counter_wire = counter;
endmodule

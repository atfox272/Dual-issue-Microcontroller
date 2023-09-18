module com_uart_trans
    #(  parameter FIRST_BIT = 0,
        parameter START_BIT = 0,
        parameter STOP_BIT = 1
    )
    (
    input [7:0] data_bus_in_TX,
    input timer_baudrate,
    input rst_n,
    output ctrl_idle_state,
    output ctrl_stop_state,
    output reg tx_port,
//    output reg TX_free,
    
    // Configuration
    input stop_bit_config,              // 1 - 2 (0/1) stop bit
    input [1:0] parity_bit_config,            // Odd / Even parity bit 
    input [1:0] data_bit_config         // 5 -> 8 data bit
    
    // debug 
//    ,output [3:0] state_counter_wire 
    );
    
    localparam START_STATE = 1;
    localparam DATA_STATE = 3;  // previous stop-state 
    localparam STOP_STATE = 2;  
    localparam IDLE_STATE = 0;
    localparam PREV_STOP_STATE = 4;
    localparam PARITY_STATE = 5;
    
    reg [2:0] data_counter;
    reg [3:0] state_counter;
    wire [2:0] data_packet_bit;
    // Data 
    reg [7:0] buffer_TX;
    assign data_packet_bit = {1'b1, data_bit_config[1], data_bit_config[0]} + 1'b1; // = data packet size + 1
    // Timer control
    assign ctrl_idle_state = state_counter == IDLE_STATE;
    assign ctrl_stop_state = state_counter == STOP_STATE;
    
    always @(posedge timer_baudrate, negedge rst_n) begin
        if(!rst_n) begin
            state_counter <= IDLE_STATE;
            data_counter <= FIRST_BIT;
//            TX_free <= 0;
            tx_port <= 1;
        end
        else begin
            case (state_counter)
                IDLE_STATE: begin
                    tx_port <= START_BIT;
                    state_counter <= START_STATE;
//                    TX_free <= 0;
                    // Data 
                    buffer_TX <= data_bus_in_TX;
                end
                START_STATE: begin
                    state_counter <= DATA_STATE;
                    tx_port <= buffer_TX[data_counter];
                    data_counter <= data_counter + 1;
                end
                PREV_STOP_STATE: state_counter <= STOP_STATE;
                STOP_STATE: begin
                    state_counter <= IDLE_STATE;
//                    TX_free <= 1;
                end
                PARITY_STATE: begin
                    // TX controller
                    tx_port <= STOP_BIT;
                    state_counter <= (stop_bit_config) ? PREV_STOP_STATE : STOP_STATE;
                end    
                DATA_STATE: begin
                    if(data_counter == data_packet_bit) begin
                        // Parity controller
                        if(parity_bit_config[1]) begin
                            tx_port <= (parity_bit_config[0]) ? !(^buffer_TX) : (^buffer_TX);
                            state_counter <= PARITY_STATE;
                        end
                        else begin
                            tx_port <= STOP_BIT;
                            state_counter <= (stop_bit_config) ? PREV_STOP_STATE : STOP_STATE;
                        end
                        // Data packet set default
                        data_counter <= FIRST_BIT;
                    end
                    else begin
                        tx_port <= buffer_TX[data_counter];
                        data_counter <= data_counter + 1;
                    end
                end
            endcase
        end
    end
    // Debug 
//    assign state_counter_wire = state_counter;
endmodule

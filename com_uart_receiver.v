module com_uart_receiver(
    input timer_baudrate, // take from timer
    input rx_port,
    input rst_n,
    output [7:0] data_in_buffer,   // buffer reg of UART module 
    output write_en,
    
    output reg valid_data_packet,
    // Configuration
    input stop_bit_config,              // 1 - 2 (0/1) stop bit
    input [1:0] parity_bit_config,            // Odd / Even parity bit 
    input [1:0] data_bit_config         // 5 -> 8 data bit
    
    // Debug 
//    ,output [2:0] state_debug
    );
    // Packet: start_bit(0) - data_bit(1:8) - stop_bit(9)
    // State encoder
    localparam INIT_STATE = 6;
    localparam IDLE_STATE = 0; // 1st state
    localparam START_STATE = 1; // 1st state
    localparam DATA_STATE = 2;
    localparam STOP_STATE = 3;  // 10th state
    localparam PREV_STOP_STATE = 4;  // 10th state
    localparam PARITY_STATE = 5;  // 10th state
    // Data 
    localparam default_index = 0;
    localparam default_data = 8'b00000000;
    // Parity 
    localparam default_valid_packet = 1'b1;
    
    reg [2:0] counter;
    reg [7:0] data_in_shifting;
    reg [2:0] state_counter;
    wire [2:0] data_packet_bit;
    
    assign data_packet_bit = {1'b1, data_bit_config[1], data_bit_config[0]};
    assign write_en = (state_counter == IDLE_STATE); // counter == 4'b1001;
    assign data_in_buffer = data_in_shifting;
    always @(negedge timer_baudrate, negedge rst_n) begin
        if(!rst_n) begin
            counter <= data_packet_bit;
            state_counter <= IDLE_STATE;
            data_in_shifting <= default_data;
            valid_data_packet <= default_valid_packet;
        end
        else begin
            case (state_counter) 
                INIT_STATE: state_counter <= START_STATE;
                IDLE_STATE: state_counter <= START_STATE;
                START_STATE: begin
                    // State contrller
                    state_counter <= DATA_STATE;
                    // Data controller
                    data_in_shifting <= data_in_shifting >> 1;
                    counter <= counter - 1;
                    data_in_shifting[data_packet_bit] <= rx_port;
                end
                PREV_STOP_STATE: begin
                    state_counter <= STOP_STATE;
                end
                STOP_STATE: begin
                    state_counter <= IDLE_STATE;
                end
                PARITY_STATE: begin
                    // State controller
//                    state_counter <= (stop_bit_config) ? PREV_STOP_STATE : stop_state;
                    state_counter <= IDLE_STATE;
                end
                DATA_STATE: begin
                    if(&counter) begin
                        // State controller 
                        if(parity_bit_config[1]) begin
                            state_counter <= PARITY_STATE;
                            // Parity checker
                            valid_data_packet <= (parity_bit_config[0]) ? 
                                                 (!(^data_in_shifting) == rx_port) :
                                                 ((^data_in_shifting) == rx_port);
                        end
                        else begin
                            state_counter <= IDLE_STATE;
                        end
                        // Counter controller
                        counter <= data_packet_bit;
                    end
                    else begin
                        data_in_shifting <= data_in_shifting >> 1;
                        counter <= counter - 1;
                        data_in_shifting[data_packet_bit] <= rx_port;
                    end
                end
                default: state_counter <= IDLE_STATE;
            endcase
        end
    end
    // Debug area
//    assign state_debug = state_counter;
endmodule 

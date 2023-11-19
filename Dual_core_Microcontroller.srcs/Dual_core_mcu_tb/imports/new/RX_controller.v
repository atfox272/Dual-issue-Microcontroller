module RX_controller
    #(
    parameter DATA_WIDTH = 8,
    // configuration 
    parameter DATA_WIDTH_OPTION     = 4,
    parameter DATA_WIDTH_OPTION_1   = 5,
    parameter DATA_WIDTH_OPTION_2   = 6,
    parameter DATA_WIDTH_OPTION_3   = 7,
    parameter DATA_WIDTH_OPTION_4   = 8,
    parameter DATA_WIDTH_OPTION_W   = $clog2(DATA_WIDTH_OPTION),    // encode width
    parameter DATA_WIDTH_OPTION_1ENC= 2'b00,    // encode 
    parameter DATA_WIDTH_OPTION_2ENC= 2'b01,    // encode 
    parameter DATA_WIDTH_OPTION_3ENC= 2'b10,    // encode 
    parameter DATA_WIDTH_OPTION_4ENC= 2'b11,    // encode 
    
    parameter PARITY_OPTION         = 3,
    parameter PARITY_NOT_ENCODE     = 0,
    parameter PARITY_ODD_ENCODE     = 1,
    parameter PARITY_EVEN_ENCODE    = 2,
    parameter PARITY_OPTION_W       = $clog2(PARITY_OPTION),
    
    parameter STOP_BIT_OPTION       = 2,
    parameter STOP_BIT_1BIT         = 0,
    parameter STOP_BIT_2BIT         = 1,
    parameter STOP_BIT_OPTION_W     = $clog2(STOP_BIT_OPTION),
    // Deep 
    parameter DATA_COUNTER_WIDTH    = $clog2(DATA_WIDTH)
    )
    (
    input                               clk,
    
    input                               RX,
    // Baudrate interconnect
    input                               baudrate_clk_en,
    output                              transaction_en,
    // FIFO interconnect
    output[DATA_WIDTH - 1:0]            data_out_rx,
    output                              fifo_wr,
    
    input [DATA_WIDTH_OPTION_W - 1:0]   data_width_option, 
    input [PARITY_OPTION_W - 1:0]       parity_option, 
    input [STOP_BIT_OPTION_W - 1:0]     stop_bit_option,
    
    output logic                        valid_data_flag,
    
    input   rst_n
    );
    
    localparam IDLE_STATE   = 0;
    // RX state
    localparam RECV_STATE   = 1;
    // Transaction state
    localparam START_BIT_STATE  = 1;
    localparam DATA_STATE       = 2;
    localparam PARITY_STATE     = 3;
    localparam STOP_STATE       = 4;
    
    // Function bit
    localparam START_BIT    = 1'b0;
    localparam STOP_BIT     = 1'b1;
    
    reg [2:0]                       rx_state;
    reg [2:0]                       transaction_state;
    reg [DATA_WIDTH - 1:0]          rx_buffer;
    reg                             parity_buffer;
    reg                             fifo_wr_reg;
    
    reg [DATA_COUNTER_WIDTH - 1:0]  data_counter;
    wire[DATA_COUNTER_WIDTH - 1:0]  data_counter_decr;
    reg                             stop_bit_counter;
    wire                            stop_bit_counter_decr;
    
    reg  transaction_start_toggle;
    reg  transaction_stop_toggle;
    
    logic[DATA_COUNTER_WIDTH - 1:0] data_counter_load;
    logic                           parity_bit_load;
    logic                           stop_bit_counter_load;
    
    assign transaction_en = transaction_start_toggle ^ transaction_stop_toggle;
    assign data_out_rx = rx_buffer;
    assign data_counter_decr = data_counter - 1;
    assign stop_bit_counter_decr = stop_bit_counter - 1;
    assign fifo_wr = fifo_wr_reg;
    assign valid_data_flag = (parity_bit_load == parity_buffer) | (parity_option == PARITY_NOT_ENCODE);
    // Configuration transaction
    always_comb begin
        case(data_width_option) 
            DATA_WIDTH_OPTION_1ENC: begin
                data_counter_load <= DATA_WIDTH_OPTION_1 - 1;   // 5bit
            end 
            DATA_WIDTH_OPTION_2ENC: begin
                data_counter_load <= DATA_WIDTH_OPTION_2 - 1;   // 6bit
            end 
            DATA_WIDTH_OPTION_3ENC: begin
                data_counter_load <= DATA_WIDTH_OPTION_3 - 1;   //7bit
            end 
            DATA_WIDTH_OPTION_4ENC: begin
                data_counter_load <= DATA_WIDTH_OPTION_4 - 1;   // 8bit
            end 
        endcase
        
        case(parity_option) 
            PARITY_NOT_ENCODE: begin
                parity_bit_load = 1'b0;
            end
            PARITY_ODD_ENCODE: begin
                parity_bit_load = ~(^rx_buffer);
            end
            PARITY_EVEN_ENCODE: begin
                parity_bit_load = (^rx_buffer);
            end
            default: begin
                parity_bit_load = 1'b0;
            end 
        endcase
        
        case(stop_bit_option)
            STOP_BIT_1BIT: begin
                stop_bit_counter_load = 1'b0;
            end
            STOP_BIT_2BIT: begin
                stop_bit_counter_load = 1'b1;
            end
        endcase
    end
    
    always @(posedge clk) begin
        if(!rst_n) begin
            rx_state <= IDLE_STATE;
            transaction_start_toggle <= 0;
            fifo_wr_reg <= 0;
        end 
        else begin
            case(rx_state) 
                IDLE_STATE: begin
                    if(RX == START_BIT) begin
                        rx_state <= RECV_STATE;
                        transaction_start_toggle <= ~transaction_stop_toggle;
                    end
                    fifo_wr_reg <= 0;
                end 
                RECV_STATE: begin
                    if(!transaction_en) begin
                        rx_state <= IDLE_STATE;
                        fifo_wr_reg <= 1;
                    end 
                end
            endcase
        end
    end 
    
    always @(posedge clk) begin
        if(!rst_n) begin
            transaction_state <= IDLE_STATE;
            transaction_stop_toggle <= 0;
            parity_buffer <= 0;
            rx_buffer <= 0;
        end 
        else if(rx_state == RECV_STATE & baudrate_clk_en) begin
            case(transaction_state)
                IDLE_STATE: begin
                    transaction_state <= START_BIT_STATE;
                    data_counter <= data_counter_load;
                    stop_bit_counter <= stop_bit_counter_load;
                end 
                START_BIT_STATE: begin
                    transaction_state <= DATA_STATE;
                    rx_buffer[data_counter] <= RX; 
                    data_counter <= data_counter_decr;
                end 
                DATA_STATE: begin
                    if(data_counter == {DATA_COUNTER_WIDTH{1'b1}}) begin   // Overflow
                        if(parity_option == PARITY_NOT_ENCODE) begin
                            transaction_state <= IDLE_STATE;
                            transaction_stop_toggle <= transaction_start_toggle;
                        end 
                        else begin
                            transaction_state <= PARITY_STATE;
                            parity_buffer <= RX;
                        end
                        data_counter <= data_counter_load;
                    end 
                    else begin
                        rx_buffer[data_counter] <= RX; 
                        data_counter <= data_counter_decr;
                    end
                end 
                PARITY_STATE: begin
                    transaction_state <= IDLE_STATE;
                    transaction_stop_toggle <= transaction_start_toggle;        
                end 
                STOP_STATE: begin
                // Don't care stop bit
                end
            endcase 
        end 
    end 
    
endmodule
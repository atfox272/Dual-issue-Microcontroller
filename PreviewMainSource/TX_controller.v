//  clk             _____|------|_____ ... __|------|______|------|
//  transaction_en  ______/----------- ... --------------------------
//  baudrate_clk_en __________________ ... __/--------------\______
//                                          ^
//  Note:                           (counter is overflow)
//
//
module TX_controller
    #(
    parameter DATA_WIDTH            = 8,
    
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
    parameter PARITY_NOT_ENCODE     = 2'b00,
    parameter PARITY_ODD_ENCODE     = 2'b10,
    parameter PARITY_EVEN_ENCODE    = 2'b11,
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
    
    output                              TX,
    // Baudrate generator interconnect
    input                               baudrate_clk_en,
    output                              transaction_en,
    // FIFO interconnect
    input [DATA_WIDTH - 1:0]            data_in_tx,
    input                               fifo_available,  // FIFO is not EMPTY
    output                              fifo_rd,
    
    input [DATA_WIDTH_OPTION_W - 1:0]   data_width_option, 
    input [PARITY_OPTION_W - 1:0]       parity_option, 
    input [STOP_BIT_OPTION_W - 1:0]     stop_bit_option,
    
    input                       rst_n
    );
    localparam IDLE_STATE       = 0;
    // TX main state
    localparam TRANS_STATE      = 1;
    // Transaction state
    localparam START_BIT_STATE  = 1;
    localparam DATA_STATE       = 2;
    localparam PARITY_STATE     = 3;
    localparam STOP_STATE       = 4;
    // Function bit
    localparam START_BIT        = 1'b0; 
    localparam STOP_BIT         = 1'b1; 
    
    reg [2:0]                       tx_state;
    reg [2:0]                       transaction_state;
    reg                             TX_reg;
    
    reg [DATA_WIDTH - 1:0]          tx_buffer;
    wire                            cur_bit;
    reg [DATA_COUNTER_WIDTH - 1:0]  data_counter;
    wire[DATA_COUNTER_WIDTH - 1:0]  data_counter_decr;
    wire[DATA_COUNTER_WIDTH - 1:0]  buffer_pointer_msb_first;
    wire[DATA_COUNTER_WIDTH - 1:0]  buffer_pointer_lsb_first;
    wire[DATA_COUNTER_WIDTH - 1:0]  buffer_pointer;
    reg                             stop_bit_counter;
    wire                            stop_bit_counter_decr;
    
    reg fifo_rd_reg;
    
    reg  transaction_start_toggle;
    reg  transaction_stop_toggle;
    
    logic[DATA_COUNTER_WIDTH - 1:0] data_counter_load;
    logic                           parity_bit_load;
    logic                           stop_bit_counter_load;
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
                parity_bit_load = ~(^tx_buffer);
            end
            PARITY_EVEN_ENCODE: begin
                parity_bit_load = (^tx_buffer);
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
    assign TX = TX_reg;
    assign cur_bit = tx_buffer[buffer_pointer];
    assign data_counter_decr = data_counter - 1;
    assign stop_bit_counter_decr = ~stop_bit_counter;
    assign transaction_en = transaction_start_toggle ^ transaction_stop_toggle;
    assign fifo_rd = fifo_rd_reg;
    assign buffer_pointer_msb_first = data_counter;
    assign buffer_pointer_lsb_first = ~data_counter;
    assign buffer_pointer = (1'b1) ? buffer_pointer_lsb_first : buffer_pointer_msb_first;
    always @(posedge clk) begin
        if(!rst_n) begin
            tx_state <= IDLE_STATE;
            fifo_rd_reg <= 0;
            transaction_start_toggle <= 1'b0;   // INIT: stop
        end 
        else begin
        case(tx_state) 
            IDLE_STATE: begin
                if(fifo_available) begin
                    tx_state <= TRANS_STATE;
                    tx_buffer <= data_in_tx;                                   // Load data
                    transaction_start_toggle <= ~transaction_stop_toggle;   // en == 1 -> start
                    fifo_rd_reg <= 1;
                end
            end 
            TRANS_STATE: begin
                tx_state <= (transaction_en) ? tx_state : IDLE_STATE ;
                fifo_rd_reg <= 0;
            end 
            
        endcase 
        end
    end 
    always @(posedge clk) begin
        if(!rst_n) begin
            transaction_state <= IDLE_STATE;
            TX_reg <= 1;
            data_counter <= 0;
            transaction_stop_toggle <= 1'b0;
        end
        else if(tx_state == TRANS_STATE & baudrate_clk_en) begin
            case(transaction_state)
                IDLE_STATE: begin
                    transaction_state <= START_BIT_STATE;
                    data_counter <= data_counter_load;
                    stop_bit_counter <= stop_bit_counter_load;
                    TX_reg <= START_BIT;
                end 
                START_BIT_STATE: begin
                    transaction_state <= DATA_STATE;
                    TX_reg <= cur_bit;
                    data_counter <= data_counter_decr;
                end 
                DATA_STATE: begin
                    if(data_counter == {DATA_COUNTER_WIDTH{1'b1}}) begin   // Overflow
                        if(parity_option == PARITY_NOT_ENCODE) begin
                            transaction_state <= STOP_STATE;
                            TX_reg <= STOP_BIT;
                        end 
                        else begin
                            transaction_state <= PARITY_STATE;
                            TX_reg <= parity_bit_load;
                        end
                        data_counter <= data_counter_load;  // Update for next usage
                    end 
                    else begin
                        TX_reg <= cur_bit;
                        data_counter <= data_counter_decr;
                    end
                end 
                PARITY_STATE: begin
                    transaction_state <= STOP_STATE;
                    TX_reg <= STOP_BIT;
                end
                STOP_STATE: begin
                    if(stop_bit_counter == {1{1'b0}}) begin    // Overflow
                        transaction_state <= IDLE_STATE;
                        stop_bit_counter <= stop_bit_counter_load;
                        transaction_stop_toggle <= transaction_start_toggle;
                    end
                    else begin
                        stop_bit_counter <= stop_bit_counter_decr;
                    end 
                end
            endcase 
        end 
    end 
endmodule

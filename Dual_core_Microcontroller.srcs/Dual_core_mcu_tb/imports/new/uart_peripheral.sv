module uart_peripheral
    #(
    parameter INTERNAL_CLOCK        = 125000000,
    // FIFO type
    parameter RX_FLAG_TYPE          = 1,    // 1-internal fifo  0-external fifo
    
    parameter DATA_WIDTH            = 8,
    parameter FIFO_DEPTH            = 32,
    
    parameter BAUDRATE_MUX_INDEX_MSB= 7,
    parameter BAUDRATE_MUX_INDEX_LSB= 5,
    parameter BAUDRATE_MUX_W        = BAUDRATE_MUX_INDEX_MSB - BAUDRATE_MUX_INDEX_LSB + 1,
    
    parameter STOP_BIT_OPTION_MSB   = 4,       
    parameter STOP_BIT_OPTION_LSB   = 4,       
    parameter STOP_BIT_OPTION_W     = STOP_BIT_OPTION_MSB - STOP_BIT_OPTION_LSB + 1,
    
    parameter PARITY_OPTION_MSB     = 3,
    parameter PARITY_OPTION_LSB     = 2,
    parameter PARITY_OPTION_LSB_W   = PARITY_OPTION_MSB - PARITY_OPTION_LSB + 1,
    
    parameter DATA_OPTION_MSB       = 1,
    parameter DATA_OPTION_LSB       = 0,
    parameter DATA_OPTION_W         = DATA_OPTION_MSB - DATA_OPTION_LSB + 1
    
    )
    (
    input                       clk,
    
    input                       RX,
    output                      TX,
    
    input [DATA_WIDTH - 1:0]    RX_config_register,
    input [DATA_WIDTH - 1:0]    TX_config_register,
    
    input                       TX_use,
    output                      TX_flag,
    output                      TX_complete,
    output                      TX_available,
    
    input                       RX_use,
    output                      RX_flag,
    output                      RX_available,
    
    input [DATA_WIDTH - 1:0]    data_in,
    output[DATA_WIDTH - 1:0]    data_out,
    
    input                       rst_n
    );
    // Baudrate generator TX
    wire                                baudrate_clk_en_tx;
    wire                                transaction_en_tx;
    // Baudrate generator RX
    wire                                baudrate_clk_en_rx;
    wire                                transaction_en_rx;
    // FIFO interface TX 
    wire [DATA_WIDTH - 1:0]             data_in_tx;
    wire                                fifo_tx_rd;
    wire                                fifo_tx_wr;
    wire                                fifo_tx_empty;
    wire                                fifo_tx_full;
    // FIFO interface RX
    wire [DATA_WIDTH - 1:0]             data_out_rx;
    wire [DATA_WIDTH - 1:0]             data_out_fifo;
    wire                                fifo_rx_rd;
    wire                                fifo_rx_wr;
    wire                                fifo_rx_empty;
    wire                                fifo_rx_full;
    // Configuration register 
    wire [BAUDRATE_MUX_W - 1:0]         baudrate_mux_tx;
    wire [BAUDRATE_MUX_W - 1:0]         baudrate_mux_rx;
    wire [DATA_OPTION_W - 1:0]          data_width_option_tx;
    wire [DATA_OPTION_W - 1:0]          data_width_option_rx;
    wire [STOP_BIT_OPTION_W - 1:0]      stop_bit_option_tx;
    wire [STOP_BIT_OPTION_W - 1:0]      stop_bit_option_rx;
    wire [PARITY_OPTION_LSB_W - 1:0]    parity_option_tx;
    wire [PARITY_OPTION_LSB_W - 1:0]    parity_option_rx;
    // Configuration mapping
    assign baudrate_mux_tx      = TX_config_register[BAUDRATE_MUX_INDEX_MSB:BAUDRATE_MUX_INDEX_LSB];
    assign baudrate_mux_rx      = RX_config_register[BAUDRATE_MUX_INDEX_MSB:BAUDRATE_MUX_INDEX_LSB];
    assign data_width_option_tx = TX_config_register[DATA_OPTION_MSB:DATA_OPTION_LSB];
    assign data_width_option_rx = RX_config_register[DATA_OPTION_MSB:DATA_OPTION_LSB];
    assign stop_bit_option_tx   = TX_config_register[STOP_BIT_OPTION_MSB:STOP_BIT_OPTION_LSB];
    assign stop_bit_option_rx   = RX_config_register[STOP_BIT_OPTION_MSB:STOP_BIT_OPTION_LSB];
    assign parity_option_tx     = TX_config_register[PARITY_OPTION_MSB:PARITY_OPTION_LSB];
    assign parity_option_rx     = RX_config_register[PARITY_OPTION_MSB:PARITY_OPTION_LSB];
    // State of UART_TX 
    assign TX_complete = fifo_tx_empty;
    assign TX_available = ~fifo_tx_full;
    assign TX_flag = fifo_tx_rd;
    // State of UART_RX
    if(RX_FLAG_TYPE) begin
        assign RX_flag = ~fifo_rx_empty;
        assign data_out = data_out_fifo;
    end 
    else begin
        assign RX_flag = fifo_rx_wr;
        assign data_out = data_out_rx;
    end 
    assign RX_available = ~transaction_en_rx;   // RX is IDLE (RX is not in transaction)
    TX_controller
        #(
        ) tx_controller (
        .clk(clk),
        .TX(TX),
        // FIFO interconnect
        .data_in_tx(data_in_tx),
        .fifo_available(~fifo_tx_empty),
        .fifo_rd(fifo_tx_rd),
        // Baudrate generator interconnect
        .baudrate_clk_en(baudrate_clk_en_tx),
        .transaction_en(transaction_en_tx),
        // Configuration
        .data_width_option(data_width_option_tx),
        .stop_bit_option(stop_bit_option_tx),
        .parity_option(parity_option_tx),
        .rst_n(rst_n)
        );
    RX_controller
        #(
        ) rx_controller (
        .clk(clk),
        
        .RX(RX),
        // FIFO interconnect
        .data_out_rx(data_out_rx),
        .fifo_wr(fifo_rx_wr),
        // Baudrate generator interconnect
        .baudrate_clk_en(baudrate_clk_en_rx),
        .transaction_en(transaction_en_rx),
        .valid_data_flag(),
        // Configuration
        .data_width_option(data_width_option_rx),
        .stop_bit_option(stop_bit_option_rx),
        .parity_option(parity_option_rx),
        
        .rst_n(rst_n)
        );
    baudrate_generator
        #(
        .INTERNAL_CLOCK(INTERNAL_CLOCK),
        .GENERATE_BD_4TX(1'b1)
        ) tx_baudrate_generator (
        .clk(clk),
        // controller interconnect 
        .transaction_en(transaction_en_tx),
        .baudrate_clk_en(baudrate_clk_en_tx),
        // Configuration
        .baudrate_mux(baudrate_mux_tx),
        
        .rst_n(rst_n)
        );
    baudrate_generator
        #(
        .INTERNAL_CLOCK(INTERNAL_CLOCK),
        .GENERATE_BD_4TX(1'b0)
        ) rx_baudrate_generator (
        .clk(clk),
        // controller interconnect 
        .baudrate_clk_en(baudrate_clk_en_rx),
        .transaction_en(transaction_en_rx),
        // Configuration
        .baudrate_mux(baudrate_mux_rx),
        .rst_n(rst_n)
        );
    fifo_module
        #(
        .DEPTH(FIFO_DEPTH)
        ) fifo_tx (
        .data_bus_in(data_in),
        .data_bus_out(data_in_tx),
        .write_ins(TX_use),
        .read_ins(fifo_tx_rd),
        .empty(fifo_tx_empty),
        .full(fifo_tx_full),
        .almost_empty(),
        .almost_full(),
        .reach_limit(),
        .enable(),
        .rst_n(rst_n)
        );   
    if(RX_FLAG_TYPE) begin    
    fifo_module
        #(
        .DEPTH(FIFO_DEPTH)
        ) fifo_rx (
        .data_bus_in(data_out_rx),
        .data_bus_out(data_out_fifo),
        .write_ins(fifo_rx_wr),
        .read_ins(RX_use),
        .empty(fifo_rx_empty),
        .full(fifo_rx_full),
        .almost_empty(),
        .almost_full(),
        .reach_limit(),
        .enable(),
        .rst_n(rst_n)
        ); 
    end               
endmodule
// Wire out module
//uart_peripheral 
//        #(
//        .INTERNAL_CLOCK(INTERNAL_CLOCK)
//        )uart_peripheral(
//        .clk(),
//        .RX(),
//        .TX(),
//        .RX_config_register(),
//        .TX_config_register(),
//        // TX interface 
//        .data_in(),
//        .TX_use(),
//        .TX_flag(),
//        .TX_complete(),
//        .TX_available(),
//        // RX interface
//        .data_out(),
//        .RX_use(),
//        .RX_flag(),
//        .RX_available(),
        
//        .rst_n(rst_n)
//        );
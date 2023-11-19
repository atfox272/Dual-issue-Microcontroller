module com_uart
    #(  
        // Mode Configuration 
        parameter SLEEP_MODE = 0,
            
        // Prescaler 
        parameter INTERNAL_CLK = 125000000,
        // CLOCK_DIVIDER = INTERNAL_CLK / ((128 * 2) * (4800 * 2))
        parameter CLOCK_DIVIDER          =  51,     // <value> = ceil(Internal clock / (256 * 2))
        parameter CLOCK_DIVIDER_UNIQUE_1 =  542,    // <value> = ceil(Internal clock / (<BAUDRATE_SPEED> * 2))  (115200)
        parameter CLOCK_DIVIDER_UNIQUE_2 =  6511,   // <value> = ceil(Internal clock / (<BAUDRATE_SPEED> * 2))  (9600)
        // TX module
        parameter FIRST_BIT = 0,
        parameter START_BIT = 0,
        parameter STOP_BIT = 1,
        // Unique clock use for reducing error percent (increase accuracy)
        parameter BD4800_ENCODE =           0,
        parameter BD9600_ENCODE =           1,
        parameter BD19200_ENCODE =          2,
        parameter BD38400_ENCODE =          3,   
        parameter BD_UNIQUE_1_ENCODE =      4,      // Unique Baudrate 
        parameter BD_UNIQUE_2_ENCODE  =     5,      // Unique Baudrate 
        parameter BAUDRATE_SEL_WIDTH = $clog2(BD_UNIQUE_2_ENCODE + 1),  // Max encode value
        parameter FIRST_COUNTER_WIDTH = $clog2(CLOCK_DIVIDER),
        // Configuration register
        parameter UART_CONFIG_WIDTH     = 8,
        parameter BAUDRATE_SEL_MSB      = 7, // Index of this bit in reg
        parameter BAUDRATE_SEL_LSB      = 5, // Index of this bit in reg
        parameter STOP_BIT_CONFIG       = 4, // Index of this bit in reg
        parameter PARITY_BIT_CONFIG_MSB = 3, // Index of this bit in reg
        parameter PARITY_BIT_CONFIG_LSB = 2, // Index of this bit in reg
        parameter DATA_BIT_CONFIG_MSB   = 1, // Index of this bit in reg
        parameter DATA_BIT_CONFIG_LSB   = 0, // Index of this bit in reg
        // FIFO hardware configuration
        parameter   FIFO_WIDTH = 8,    // 1'b<read / write bit> + 7'b<Address-device>
        parameter   FIFO_DEPTH = 32,   // Capacity of fifo is 31 slots
        // In FIFO, this module have 2 options
        // First, You can build external FIFO and use RX_flag as a write_data in FIFO
        // Second, you can use internal FIFO and RX_flag is output pin of state of FIFO (empty or not empty)
        // And you will use RX_use to take out data from the FIFO
        // In First way, you should use "interupt" of MCU to take out data immediately
        // In second way, you can poll the RX_flag. when RX_flag is HIGH, you can set RX_use HIGH to take out data
        parameter RX_FLAG_CONFIG = 0,   // 1: Internal FIFO || 0 : external FIFO
        // Data 
        parameter DATA_WIDTH = 8
        
    )
    (
    input   wire                            clk,    // Internal clock
    
    // Serial port 
    output  wire                            TX,
    input   wire                            RX,                          
    
    // Configuration register
    input   wire    [UART_CONFIG_WIDTH - 1:0] TX_config_register, 
    input   wire    [UART_CONFIG_WIDTH - 1:0] RX_config_register, 
    
    // TX Flag 
    input   wire                        TX_use,     // CPu want to use TX, set LOW->HIGH
    output  wire                        TX_flag,
    output  wire                        TX_complete,// Module has transmitted all data in buffer already  
    output  wire                        TX_available,  
    
    // RX Flag
    input   wire                        RX_use,
    output  wire                        RX_flag,
    output  wire                        RX_available,
    
    input   wire    [DATA_WIDTH - 1:0]  data_bus_in,           // Take data from CPU and send it to another UART module
    output  wire    [DATA_WIDTH - 1:0]  data_bus_out,
    
    // Sleep mode 
    input   wire                        RX_enable,
    input   wire                        TX_enable,                
    
    // Optional in parity checker
    output  wire                        valid_data_packet,
    // Debug TX module 
//    output wire [3:0] counter_packet_trans,
//    output timer_baudrate_trans_clone,
    
    // Debug TX 
    // Controller TX
//    output [3:0] state_counter_wire,
//    // Timer TX
//    output TX_disable_wire,
//    output debug_timer_tx, 
//    output [FIRST_COUNTER_WIDTH - 1:0] debug_timer_tx_1, 
////    output TX_free_clone,
    
    // Reset_negetive
    input   wire                        rst_n  
    );
    // RX module area ///////////////////////////////////
    // Linking area
    wire baudrate_clk_RX;
    // Configuration area
    wire [BAUDRATE_SEL_MSB - BAUDRATE_SEL_LSB:0] baudrate_sel_receiver;
    wire rx_stop_bit_config;
    wire [PARITY_BIT_CONFIG_MSB - PARITY_BIT_CONFIG_LSB:0] rx_parity_bit_config;
    wire [DATA_BIT_CONFIG_MSB - DATA_BIT_CONFIG_LSB:0] rx_data_bit_config;
//    wire 
    
    assign baudrate_sel_receiver = RX_config_register[BAUDRATE_SEL_MSB:BAUDRATE_SEL_LSB];
    assign rx_stop_bit_config = RX_config_register[STOP_BIT_CONFIG];    
    assign rx_parity_bit_config = RX_config_register[PARITY_BIT_CONFIG_MSB:PARITY_BIT_CONFIG_LSB];
    assign rx_data_bit_config = RX_config_register[DATA_BIT_CONFIG_MSB:DATA_BIT_CONFIG_LSB];
    
    wire [DATA_WIDTH - 1:0] data_from_RX_ctrl;
    wire write_out_fifo;
    wire out_fifo_empty;
//    wire out_fifo_full;
    wire [DATA_WIDTH - 1:0] data_bus_out_from_RX_fifo;
    assign data_bus_out = (RX_FLAG_CONFIG) ? data_bus_out_from_RX_fifo : data_from_RX_ctrl;
    assign RX_flag = (RX_FLAG_CONFIG) ? !out_fifo_empty : write_out_fifo;
    assign RX_available = !out_fifo_empty;
    
    // Implement sleep mode for UART module
    reg RX_enable_sync;
    
    // Wake-up state
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) RX_enable_sync <= 0;
        else RX_enable_sync <= RX_enable;
    end
    
    fifo_module  #(
                    .WIDTH(FIFO_WIDTH),
                    .DEPTH(FIFO_DEPTH),
                    .SLEEP_MODE(SLEEP_MODE)
                   )data_bus_out_fifo(
                    .data_bus_in(data_from_RX_ctrl),
                    .data_bus_out(data_bus_out_from_RX_fifo),
                    .write_ins(write_out_fifo),
                    .read_ins(RX_use),
                    .enable(1'b1),      // Do not modify (Hasn't debug yet)
                    .empty(out_fifo_empty),
//                    .full(out_fifo_full),
                    .rst_n(rst_n)
                    );
    
    wire rx_port_sync;
    wire rx_module_clk;
    
    generate
        if(SLEEP_MODE) begin
            assign rx_module_clk = (RX_enable_sync) ? clk : 1'b0;
            assign rx_port_sync = (RX_enable_sync) ? RX : 1'b1;
        end
        else begin
            assign rx_module_clk = clk;    
            assign rx_port_sync = RX;
        end
	endgenerate
    
    com_uart_receiver_timer  #( .CLOCK_DIVIDER(CLOCK_DIVIDER),
                                .CLOCK_DIVIDER_UNIQUE_1(CLOCK_DIVIDER_UNIQUE_1),
                                .CLOCK_DIVIDER_UNIQUE_2(CLOCK_DIVIDER_UNIQUE_2),
                                .BD4800_ENCODE(BD4800_ENCODE),
                                .BD9600_ENCODE(BD9600_ENCODE),
                                .BD19200_ENCODE(BD19200_ENCODE),
                                .BD38400_ENCODE(BD38400_ENCODE),
                                .BD_UNIQUE_1_ENCODE(BD_UNIQUE_1_ENCODE),
                                .BD_UNIQUE_2_ENCODE(BD_UNIQUE_2_ENCODE),
                                .BAUDRATE_SEL_WIDTH(BAUDRATE_SEL_WIDTH)
                                )rx_timer(   
                                .clk(rx_module_clk),  
                                .rx_port(rx_port_sync),
                                .baudrate_clk(baudrate_clk_RX),
                                .stop_cond(write_out_fifo),
                                .rst_n(rst_n),
                                // Configuration
                                .baudrate_sel(baudrate_sel_receiver)
                                //Debug area
//                                        ,.debug_128(debug_128),
//                                        .debug_40(debug_40)
                                        ); 
                                        
    com_uart_receiver rx_module(.timer_baudrate(baudrate_clk_RX), 
                                .rx_port(rx_port_sync), 
                                .rst_n(rst_n),
                                .data_in_buffer(data_from_RX_ctrl), 
                                .write_en(write_out_fifo) ,
                                // Configuration
                                .stop_bit_config(rx_stop_bit_config),
                                .parity_bit_config(rx_parity_bit_config),
                                .data_bit_config(rx_data_bit_config),
                                // Parity Checker
                                .valid_data_packet(valid_data_packet)
                                //Debug area
//                                ,.state_debug(state_debug)
                                );
//////////////////////////////////////////////////////////////////////////

    // TX module area ///////////////////////////////////
    wire baudrate_clk_TX;
    // TX FIFO 
    wire [DATA_WIDTH - 1:0] data_to_TX_ctrl;
    // Configuration
    wire [BAUDRATE_SEL_MSB - BAUDRATE_SEL_LSB:0] baudrate_sel_trans;
    wire tx_stop_bit_config;
    wire [PARITY_BIT_CONFIG_MSB - PARITY_BIT_CONFIG_LSB:0] tx_parity_bit_config;
    wire [DATA_BIT_CONFIG_MSB - DATA_BIT_CONFIG_LSB:0] tx_data_bit_config;
    // FIFO 
    wire FIFO_empty;
    wire FIFO_full;
    // Controller state 
    wire ctrl_idle_state;
    wire ctrl_stop_state;
    // Configuration register
    assign baudrate_sel_trans = TX_config_register[BAUDRATE_SEL_MSB:BAUDRATE_SEL_LSB];
    assign tx_stop_bit_config = TX_config_register[STOP_BIT_CONFIG];   
    assign tx_parity_bit_config = TX_config_register[PARITY_BIT_CONFIG_MSB:PARITY_BIT_CONFIG_LSB];
    assign tx_data_bit_config = TX_config_register[DATA_BIT_CONFIG_MSB:DATA_BIT_CONFIG_LSB];
    // TX state controller
    assign TX_flag = ctrl_idle_state;
    assign TX_available = !FIFO_full;
    
    reg TX_enable_sync;
    wire tx_module_clk;
    
    // Wake-up module
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) TX_enable_sync <= 0;
        else TX_enable_sync <= TX_enable;
    end
    
    fifo_module #(  .WIDTH(FIFO_WIDTH),
                    .DEPTH(FIFO_DEPTH),
                    .SLEEP_MODE(SLEEP_MODE)
                    )data_bus_in_fifo(   
                    .data_bus_in(data_bus_in),
                    .data_bus_out(data_to_TX_ctrl),
                    .write_ins(TX_use),
                    .read_ins(ctrl_stop_state),
                    .enable(1'b1),  // Do not modify (Hasn't debug yet)
                    .empty(FIFO_empty),
                    .full(FIFO_full),
                    .rst_n(rst_n)
                   );
    
	generate 
        if(SLEEP_MODE) begin                          
            assign tx_module_clk = (TX_enable_sync) ? clk : 1'b0;  
        end
        else begin
            assign tx_module_clk = clk;      
        end
	endgenerate
    com_uart_trans_timer        #(
                                    .CLOCK_DIVIDER(CLOCK_DIVIDER),
                                    .CLOCK_DIVIDER_UNIQUE_1(CLOCK_DIVIDER_UNIQUE_1),
                                    .CLOCK_DIVIDER_UNIQUE_2(CLOCK_DIVIDER_UNIQUE_2),
                                    .BD4800_ENCODE(BD4800_ENCODE),
                                    .BD9600_ENCODE(BD9600_ENCODE),
                                    .BD19200_ENCODE(BD19200_ENCODE),
                                    .BD38400_ENCODE(BD38400_ENCODE),
                                    .BD_UNIQUE_1_ENCODE(BD_UNIQUE_1_ENCODE),
                                    .BD_UNIQUE_2_ENCODE(BD_UNIQUE_2_ENCODE),
                                    .BAUDRATE_SEL_WIDTH(BAUDRATE_SEL_WIDTH)
                                 )tx_timer(  
                                    .clk(tx_module_clk), 
                                    .baudrate_sel(baudrate_sel_trans), 
                                    .FIFO_empty(FIFO_empty),
                                    .ctrl_idle_state(ctrl_idle_state),
                                    .ctrl_stop_state(ctrl_stop_state),
                                    .TX_complete(TX_complete),
                                    .rst_n(rst_n), 
                                    // Configuration
                                    .baudrate_clk(baudrate_clk_TX)
                                    // Debug 
//                                    ,.TX_disable_wire(TX_disable_wire)
//                                    ,.debug(debug_timer_tx)
//                                    ,.debug_1(debug_timer_tx_1)
                                    ); 
                                        
    com_uart_trans tx_module(   .timer_baudrate(baudrate_clk_TX), 
                                .data_bus_in_TX(data_to_TX_ctrl),
                                .rst_n(rst_n),
                                .tx_port(TX),
                                // Configuration
                                .stop_bit_config(tx_stop_bit_config),
                                .parity_bit_config(tx_parity_bit_config),
                                .data_bit_config(tx_data_bit_config),
                                // controller state 
                                .ctrl_idle_state(ctrl_idle_state),
                                .ctrl_stop_state(ctrl_stop_state)
                                
                                // Debug 
//                                ,.state_counter_wire(state_counter_wire)
                                );
    
//////////////////////////////////////////////////////////////////////////
    
//    assign timer_baudrate_trans_clone = baudrate_clk_TX;
//    assign TX_disable_wire = TX_disable_wire;
//    assign TX_free_clone = TX_free;
endmodule

`timescale 1ns / 1ps
module processor_1_case1_tb;
    
    parameter DATA_WIDTH            = 8;
    parameter DOUBLEWORD_WIDTH      = 64;
    parameter DATA_MEMORY_SIZE      = 10'd256;      // 256 bytes (2Kb)
    
    parameter ADDR_WIDTH_DM         = $clog2(DATA_MEMORY_SIZE);
    parameter DATA_TYPE_WIDTH       = 2;
    
    parameter INSTRUCTION_WIDTH     = 32;   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 32;
    // PM
    parameter ADDR_WIDTH_PM         = $clog2(PROGRAM_MEMORY_SIZE);
    parameter START_WR_ADDR_PM      = 8'h00;

    // Deep configuration
    parameter FINISH_PROGRAM_OPCODE  = 7'b0001011;
    parameter FINISH_PROGRAM_TIMER   = 1250000;
    
    reg clk;
    reg rst_n;
    // Processor & UART_1
    wire [DATA_WIDTH - 1:0]         data_bus_out_uart_1;
    wire                            RX_use_1;
    wire                            RX_flag_1;
    // Processor & Program memory
    wire    [DATA_WIDTH - 1:0]      data_bus_wr_pm;
    wire                            wr_idle_pm;
    wire    [ADDR_WIDTH_PM - 1:0]   addr_wr_pm;
    wire                            wr_ins_pm;
    // Common 
    wire    [1:0]                   main_state;
    
    // UART_ex & UART_1
    wire    TX_ex;
    reg     TX_use_ex;
    reg     [7:0] data_bus_in_tx_ex;
    wire    [7:0] TX_config_register_ex = 8'b00101111;
    wire    [7:0] RX_config_register_1  = 8'b00101111;
    wire    RX_1;
    
    // Debug 
    wire    [63:0] debug_1;
    wire    [DATA_WIDTH - 1:0]  registers_wire [0: PROGRAM_MEMORY_SIZE - 1];
//    // Debug area
//    assign debug_1 = {0, _4byte_counter};
    assign RX_1 = TX_ex ;
    
    com_uart    #(
                .SLEEP_MODE(0)
                )
                uart_ex
                (
                .clk(clk),
                .TX(TX_ex),
                .TX_use(TX_use_ex),
                .data_bus_in(data_bus_in_tx_ex),
                .TX_config_register(TX_config_register_ex),
                .rst_n(rst_n)
                );
                
    com_uart    #(
                .SLEEP_MODE(0), 
                .RX_FLAG_CONFIG(1'b1) /// Internal FIFO
                )             
                uart_1
                (
                .clk(clk),
                .data_bus_out(data_bus_out_uart_1),
                .RX_use(RX_use_1),
                .RX_flag(RX_flag_1),
                .RX_config_register(RX_config_register_1),
                .RX(RX_1),
                .rst_n(rst_n)
                ); 
                
    ram_module      #(
                    .ADDR_DEPTH(PROGRAM_MEMORY_SIZE)
                    )program_memory(
                    .clk(clk),
                    .data_bus_wr(data_bus_wr_pm),
                    .data_type_wr(BYTE_SIZE_ENCODE),    // Write byte (8bit)
                    .addr_wr(addr_wr_pm),
                    .wr_ins(wr_ins_pm),
                    .wr_idle(wr_idle_pm),
                    .rst_n(rst_n)
                    //Debug 
                    ,.registers_wire(registers_wire)
                    );
                
//    wire    [DATA_WIDTH - 1:0]      data_bus_wr_pm;
//    wire                            wr_idle_pm;
//    wire    [ADDR_WIDTH_PM - 1:0]   addr_wr_pm;
//    wire                            wr_ins_pm;
    Processor           #(
                        .MAIN_RPOCESSOR(1'b1),
                        .FINISH_PROGRAM_OPCODE(FINISH_PROGRAM_OPCODE),
                        .FINISH_PROGRAM_TIMER(FINISH_PROGRAM_TIMER)
                        )
                        processor_1
                        (
                        .clk(clk),
                        // UART_1
                        .data_bus_out_uart_1(data_bus_out_uart_1),
                        .RX_use_1(RX_use_1),
                        .RX_flag_1(RX_flag_1),
                        // Program memory 
                        .data_bus_wr_pm(data_bus_wr_pm),
                        .wr_idle_pm(wr_idle_pm),
                        .addr_wr_pm(addr_wr_pm),
                        .wr_ins_pm(wr_ins_pm),
                        // Main state
                        .main_state(main_state),
                        .rst_n(rst_n)
                        
                        // Debug
                        ,.debug_1(debug_1)
                        );                
    
    initial begin
        clk <= 0;
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 0;
        
        rst_n <= 1;
        #1 rst_n <= 0;
        #9 rst_n <= 1;
    end
    initial begin
        forever #1 clk <= ~clk;
    end
    
        int i;
        
    initial begin
        #11;
        
        for(i = 0; i < 31; i = i + 1) begin
            
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= i + 1;
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
        end
        
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= FINISH_PROGRAM_OPCODE;
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
        
    end
endmodule

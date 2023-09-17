`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: HCMUT
// Engineer: ATFox
// 
// Create Date: 09/11/2023 08:46:02 PM
// Design Name: 
// Module Name: Dual_core_mcu
// Project Name: Dual_core Microcontroller
// Target Devices: Arty-Z7 20
// Tool Versions: Vivado 2020.2
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Dual_core_mcu
    #(
    parameter DATA_WIDTH        = 8,
    parameter WORD_WIDTH        = 32,
    parameter DOUBLEWORD_WIDTH  = 64,
    
    // Common
    parameter REGISTER_AMOUNT       = 32,
    
    // GPIO
    parameter GPIO_NUM = 10,
    
    // Data memory 
    parameter      DATA_MEMORY_SIZE                                =  10'd256,      // 256 bytes (2Kb)
    parameter      RESERVED_REG_AMOUNT                             =  10'd10,
    parameter byte RESERVED_REG_DEFAULT[0:RESERVED_REG_AMOUNT - 1] = {8'b00000000,  // address 0x0  (PORT_A)
                                                                      8'b00000000,  // address 0x1  (PORT_B)
                                                                      8'b00000000,  // address 0x2  (DEBUGGER) 
                                                                      8'b00100011,  // address 0x3  (UART_1_RX_CONFIG)
                                                                      8'b00100011,  // address 0x4  (UART_1_TX_CONFIG)
                                                                      8'b00000000,  // address 0x5  (COM_PERIPHERAL)
                                                                      8'b00000000,  // address 0x6  (ADDRESS_ENCODER_PERIPHERAL)
                                                                      8'b00100011,  // address 0x7  (UART_2_RX_CONFIG)
                                                                      8'b00100011,  // address 0x8  (UART_2_TX_CONFIG)
                                                                      8'b11111000}, // address 0x9  (SPI_CONFIG)
    // Program memory
    parameter INSTRUCTION_WIDTH     = 32,   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 64,   // 64 instruction
    parameter WORD_SIZE_ENCODE      = 2'b01,
    parameter BYTE_SIZE_ENCODE      = 2'b00,
    
    // Deep configuration
    parameter DATA_TYPE         = 3,                // byte - word - doubleword
    parameter DATA_TYPE_WIDTH   = $clog2(DATA_TYPE),
    parameter ADDR_WIDTH_DM     = $clog2(DATA_MEMORY_SIZE),
    parameter ADDR_WIDTH_PM     = $clog2(PROGRAM_MEMORY_SIZE),
    parameter FIFO_BUFFER_SIZE  = 10'd64,
    
    // Reserved Instruction
    parameter TERMINATE_OPCODE  = 7'b0001011
    )
    (
    input   wire            device_clk,
    
    // UART_1 for program device
    input   wire            RX_1,
    output  wire            TX_1,
    
    // UART_2
    input   wire            RX_2,
    output  wire            TX_2,
    
    // SPI
    inout   wire            MOSI,            
    inout   wire            MISO, 
    inout   wire            SCK,
    inout   wire            SS,           
    
    // GPIO
    inout   wire    [9:0]   IO_PORT,
    
    // Reset negedge
    input   wire            rst_n
    );
    
    
    wire internal_clk;
    
    // Declare interface //////////////////////////////////////////////
    // DATA_MEMORY (dm)
    wire [DOUBLEWORD_WIDTH - 1:0]    data_bus_wr_dm;
    wire [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd_dm;
    wire [DATA_TYPE_WIDTH - 1:0]     data_type_wr_dm;
    wire [DATA_TYPE_WIDTH - 1:0]     data_type_rd_dm;
    wire [ADDR_WIDTH_DM - 1:0]       addr_wr_dm;
    wire [ADDR_WIDTH_DM - 1:0]       addr_rd_dm;
    wire                             wr_ins_dm;
    wire                             rd_ins_dm;
    wire                             wr_idle_dm;
    wire                             rd_idle_dm;
    wire [DATA_WIDTH - 1:0]          reserved_registers  [0:RESERVED_REG_AMOUNT - 1];
    // PROGRAM_MEMORY (pm)
    wire [INSTRUCTION_WIDTH - 1:0]   data_bus_wr_pm;
    wire [INSTRUCTION_WIDTH - 1:0]   data_bus_rd_pm;
    wire [ADDR_WIDTH_PM - 1:0]       addr_wr_pm;
    wire [ADDR_WIDTH_PM - 1:0]       addr_rd_pm;
    wire                             wr_ins_pm;
    wire                             rd_ins_pm;
    wire                             wr_idle_pm;
    wire                             rd_idle_pm;
    
    // UART_TX_1
    wire [DATA_WIDTH - 1:0] data_bus_in_uart_1;
    wire [DATA_WIDTH - 1:0] TX_config_register_1;
    wire                    TX_use_1;
    wire                    TX_complete_1;
    wire                    TX_enable_1;
    // UART_RX_1
    wire [DATA_WIDTH - 1:0] data_bus_out_uart_1;
    wire [DATA_WIDTH - 1:0] RX_config_register_1;
    wire                    RX_use_1;
    wire                    RX_flag_1;
    wire                    RX_enable_1;
    
        
    // UART_TX_2
    wire [DATA_WIDTH - 1:0] data_bus_in_uart_2;
    wire [DATA_WIDTH - 1:0] TX_config_register_2;
    wire                    TX_use_2;
    wire                    TX_complete_2;
    wire                    TX_enable_2;   
    // UART_RX_2
    wire [DATA_WIDTH - 1:0] data_bus_out_uart_2;
    wire [DATA_WIDTH - 1:0] RX_config_register_2;
    wire                    RX_flag_2;
    wire                    RX_enable_2;
    
    // SPI
    wire                    SPI_enable;
    wire [DATA_WIDTH - 1:0] SPI_config_register;
    wire [DATA_WIDTH - 1:0] data_bus_in_spi;
    wire [DATA_WIDTH - 1:0] data_bus_out_spi;
    wire                    SPI_use;
    wire                    SPI_read;
    wire                    SPI_write;
    wire                    SPI_avail;   // SPI_en (SPI available to read data)
    
    
    // I2C (Uncertain)
    wire I2C_enable;
    
    // GPIO 
    wire [DATA_WIDTH - 1:0] PORT_A;
    wire [DATA_WIDTH - 1:0] PORT_B;
    
    ///////////////////////////////////////////////////////////////
    
    // Connect wire /////////////////////////////////////////////////////////////
    assign internal_clk = device_clk;
    
    // Reserved register (configuration register)
    assign PORT_A               = reserved_registers[8'h00];
    assign PORT_B               = reserved_registers[8'h01];
    assign TX_enable_1          = reserved_registers[8'h02][0];     // Debugger
    assign RX_config_register_1 = reserved_registers[8'h03];
    assign TX_config_register_1 = reserved_registers[8'h04];
    assign I2C_enable           = reserved_registers[8'h05][8'h00];
    assign SPI_enable           = reserved_registers[8'h05][8'h01];
    assign RX_enable_2          = reserved_registers[8'h05][8'h02];
    assign TX_enable_2          = reserved_registers[8'h05][8'h03];
    // Address encoder place    = reserved_registers[8'h06]
    assign RX_config_register_2 = reserved_registers[8'h07];
    assign TX_config_register_2 = reserved_registers[8'h08];
    assign SPI_config_register  = reserved_registers[8'h09];
    ////////////////////////////////////////////////////////////////////////////
    
    Processor processor_1   
                        (
                        );
    
    Processor processor_2  
                        (
                        );
                            
    Sync_primitive synchronization_primitive  
                        (
                        );
                        
    Multi_processor_manager multi_processor_manager
                        (
                        );
    
    // Data Memory (2Kb - 256bytes)
    ram_module      #(
                    .ADDR_DEPTH(DATA_MEMORY_SIZE),
                    .DOUBLEWORD_WIDTH(DOUBLEWORD_WIDTH),
                    .RESERVED_REG_AMOUNT(RESERVED_REG_AMOUNT),
                    .RESERVED_REG_DEFAULT(RESERVED_REG_DEFAULT)
                    )data_memory(
                    .clk(internal_clk),
                    .data_bus_wr(data_bus_wr_dm),
                    .data_bus_rd(data_bus_rd_dm),
                    .data_type_wr(data_type_wr_dm),
                    .data_type_rd(data_type_rd_dm),
                    .addr_wr(addr_wr_dm),
                    .addr_rd(addr_rd_dm),
                    .wr_ins(wr_ins_dm),
                    .rd_ins(rd_ins_dm),
                    .wr_idle(wr_idle_dm),
                    .rd_idle(rd_idle_dm),
                    .reserved_registers(reserved_registers),
                    .rst_n(rst_n)
                    );
    // Program memory (1Kb - 32 instructions)
    ram_module      #(
                    .ADDR_DEPTH(PROGRAM_MEMORY_SIZE)
                    )program_memory(
                    .clk(internal_clk),
                    .data_bus_wr(data_bus_wr_pm),
                    .data_bus_rd(data_bus_rd_pm),
                    .data_type_wr(BYTE_SIZE_ENCODE),    // Write byte (8bit)
                    .data_type_rd(WORD_SIZE_ENCODE),    // Read  word (32bit)
                    .addr_wr(addr_wr_pm),
                    .addr_rd(addr_rd_pm),
                    .wr_ins(wr_ins_pm),
                    .rd_ins(rd_ins_pm),
                    .wr_idle(wr_idle_pm),
                    .rd_idle(rd_idle_pm),
                    .rst_n(rst_n)
                    );
    
    // Communication peripherals            
    com_uart            #(
                        .RX_FLAG_CONFIG(1'b1),              // Use internal FIFO
                        .FIFO_DEPTH(FIFO_BUFFER_SIZE)
                        )uart_peripheral_1(
                        .clk(internal_clk),
                        .TX(TX_1),
                        .RX(RX_1),
                        // TX
                        .data_bus_in(data_bus_in_uart_1),
                        .TX_config_register(TX_config_register_1),
                        .TX_use(TX_use_1),
                        .TX_complete(TX_complete_1),
                        .TX_enable(TX_enable_1),
                        // RX
                        .data_bus_out(data_bus_out_uart_1),
                        .RX_config_register(RX_config_register_1),
                        .RX_use(RX_use_1),
                        .RX_flag(RX_flag_1),
                        .RX_enable(RX_enable_1),
                        .rst_n(rst_n)
                        );
    com_uart            #(
                        .RX_FLAG_CONFIG(1'b1),              // Use internal FIFO
                        .FIFO_DEPTH(FIFO_BUFFER_SIZE)
                        )uart_peripheral_2( 
                        );      
    com_spi  spi_peripheral 
                        (
                        );                                                                                     
endmodule

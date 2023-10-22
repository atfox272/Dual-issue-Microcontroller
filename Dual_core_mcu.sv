`timescale 1ns / 1ps
//`define DEBUG
`define EXTI
`define TIM
`define UART_PROT_1    
`define UART_PROT_2 
//`define SPI_PROT
//`define I2C_PROT
module Dual_core_mcu
    #(
    parameter DATA_WIDTH        = 8,
    parameter WORD_WIDTH        = 32,
    parameter DOUBLEWORD_WIDTH  = 64,
    
    // Special register     x0:     hardwired zero
    //                      x2:     stack pointer   (default: 0xF0)
    //                      x3:     global pointer  (default: 0x2B)
    parameter           REGISTER_AMOUNT                          =   32,
    parameter longint   REGISTER_DEFAULT [0:REGISTER_AMOUNT - 1] =  {64'h0000000000000000, 64'h0000000000000000, 64'h00000000000000F0, 64'h000000000000002B,  // x0   -   x3
                                                                     64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000,  // x4   -   x7
                                                                     64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000,  // x8   -   x11
                                                                     64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000,  // x12  -   x15
                                                                     64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000,  // x16  -   x19
                                                                     64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000,  // x20  -   x23
                                                                     64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000,  // x24  -   x27
                                                                     64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000, 64'h0000000000000000}, // x28  -   x31  
    
    // GPIO
    parameter GPIO_NUM                                             = 13,
    parameter GPIO_PER_PORT                                        = 4,
    // Data memory 
    parameter      DATA_MEMORY_SIZE                                =  1024,      // 256 bytes (2Kb)
    parameter      RESERVED_REG_AMOUNT                             =  17,
    parameter byte RESERVED_REG_DEFAULT[0:RESERVED_REG_AMOUNT - 1] = {8'b00000000,  // address 0x00  (PORT_A)
                                                                      8'b00000000,  // address 0x01  (PORT_B)
                                                                      8'b00000000,  // address 0x02  (PORT_C)
                                                                      8'b00000000,  // address 0x03  (DEBUGGER) 
                                                                      8'b10001111,  // address 0x04  (UART_1_RX_CONFIG) 
                                                                      8'b10001111,  // address 0x05  (UART_1_TX_CONFIG)
                                                                      8'b00001100,  // address 0x06  (COM_PERIPHERAL) // Do not enable (set 1) in initial state
                                                                      8'b00000000,  // address 0x07  (NOTHING)
                                                                      8'b10001111,  // address 0x08  (UART_2_RX_CONFIG)
                                                                      8'b10001111,  // address 0x09  (UART_2_TX_CONFIG)
                                                                      8'b11111000,  // address 0x0A  (SPI_CONFIG)
                                                                      8'b00000000,  // address 0x0B  (I2C_CONFIG)
                                                                      8'b00000000,  // address 0x0C  (EXTERNAL_INT_CONFIG)
                                                                      8'b00000000,  // address 0x0D  (PINCHANGE_INT_CONFIG)
                                                                      8'b00000000,  // address 0x0E  (TIMER_INT_CONFIG)
                                                                      8'b11111111,  // address 0x0F  (TIMER_LIMIT_VALUE_H)
                                                                      8'b11111111}, // address 0x10  (TIMER_LIMIT_VALUE_L)
    // Program memory
    parameter INSTRUCTION_WIDTH     = 32,   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 1024,   // 64 instruction
    parameter DWORD_SIZE_ENCODE     = 2'b10,
    parameter WORD_SIZE_ENCODE      = 2'b01,
    parameter BYTE_SIZE_ENCODE      = 2'b00,
    // - program address 
    parameter MAIN_PROGRAM_ADDR     = 64'hC0,
    parameter INT1_PROGRAM_ADDR     = 64'h00,
    parameter INT2_PROGRAM_ADDR     = 64'h40,
    parameter INT3_PROGRAM_ADDR     = 64'h80,
    
    // Interrupt 
    parameter INTERRUPT_BUFFER      = 16,
    parameter PROGRAM_AMOUNT        = 4,
    parameter PROGRAM_COUNTER_WIDTH = $clog2(PROGRAM_AMOUNT),
    parameter MAIN_PROGRAM_ENCODE   = 2'b00,
    parameter INT1_PROGRAM_ENCODE   = 2'b01,
    parameter INT2_PROGRAM_ENCODE   = 2'b10,
    parameter INT3_PROGRAM_ENCODE   = 2'b11,
    // Timer interrupt
    parameter PRESCALER_TIMER_WIDTH = 3,
    parameter REGISTER_TIMER_WIDTH  = 8,
    
    // Address mapping  (protocol peripheral)
    parameter ADDR_MAPPING_PERIPHERAL   = 5,
    parameter ADDDR_MAPPING_WIDTH       = $clog2(ADDR_MAPPING_PERIPHERAL),
    // Protocol peripheral communication 
    parameter AMOUNT_SND_BYTE           = 16,  
    parameter AMOUNT_RCV_BYTE           = 16,
    parameter AMOUNT_SND_WIDTH          = $clog2(AMOUNT_SND_BYTE),
    parameter AMOUNT_RCV_WIDTH          = $clog2(AMOUNT_RCV_BYTE),
    
    // Deep configuration
    parameter DATA_TYPE             = 3,                // byte - word - doubleword
    parameter DATA_TYPE_WIDTH       = $clog2(DATA_TYPE),
    parameter ADDR_WIDTH_DM         = $clog2(DATA_MEMORY_SIZE),
    parameter ADDR_WIDTH_PM         = $clog2(PROGRAM_MEMORY_SIZE),
    parameter FIFO_BUFFER_SIZE      = 64,
    parameter REG_SPACE_WIDTH       = $clog2(REGISTER_AMOUNT),
    `ifdef DEBUG
    parameter CLOCK_DIVIDER_UNIQUE_1= 5,
    `endif
    // Change main_state
    parameter FINISH_PROGRAM_OPCODE  = 7'b0001011,
    parameter FINISH_PROGRAM_TIMER   = 1250000
    )
    (
    input   wire                        clk,
    
    `ifdef UART_PROT_1                  // For load bitstream file
    input   wire                        RX_1,
    output  wire                        TX_1,
    `endif
    
    `ifdef UART_PROT_2
    input   wire                        RX_2,
    output  wire                        TX_2,
    `endif
    
    `ifdef SPI_PROT
    inout   wire                        MOSI,            
    inout   wire                        MISO, 
    inout   wire                        SCK,
    inout   wire                        SS,           
    `endif
    
    `ifdef I2C_PROT
    inout   wire                        SDA,
    inout   wire                        SDL,
    `endif
    
    // GPIO
    inout           [GPIO_NUM - 1:0]    IO_PORT,
    
    // Reset negedge
    input   wire                        rst_n
    
    `ifdef DEBUG
    ,output  wire    [DATA_WIDTH - 1:0]      program_memory_wire [0: PROGRAM_MEMORY_SIZE - 1]
    ,output  wire    [DATA_WIDTH - 1:0]      data_memory_wire [0: DATA_MEMORY_SIZE - 1]
    `endif
    );
    
    // Declare interface //////////////////////////////////////////////
    // PROCESSOR 1
    wire[INSTRUCTION_WIDTH - 1:0]       fetch_instruction_1;
    wire                                boot_processor_1;
    wire                                processor_idle_1;
    wire[1:0]                           main_state;
    // PROCESSOR 2
    wire[INSTRUCTION_WIDTH - 1:0]       fetch_instruction_2;
    wire                                boot_processor_2;
    wire                                processor_idle_2;
    
    // REGISTERS MANAGEMENT
    // -- Registers management signal
    wire    [DOUBLEWORD_WIDTH - 1:0]    ra_register;
    wire    [REG_SPACE_WIDTH*3 - 1:0]   register_num;
    wire                                boot_renew_register_1;
    wire                                boot_renew_register_2;
    wire                                boot_renew_3registers_2;
    wire                                synchronized_processors;
    wire    [0:REGISTER_AMOUNT - 1]     processing_register_table; 
    wire    [DOUBLEWORD_WIDTH - 1:0]    processor_registers_1 [0:REGISTER_AMOUNT - 1];
    wire    [DOUBLEWORD_WIDTH - 1:0]    processor_registers_2 [0:REGISTER_AMOUNT - 1];
    wire    [DOUBLEWORD_WIDTH - 1:0]    registers_renew [0:REGISTER_AMOUNT - 1]; 
    // -- Synchronization signal
    wire                                new_data_register       [0:REGISTER_AMOUNT - 1];
    wire                                synchronization_processor_1;
    wire                                synchronization_processor_2;
    // INTERRUPT CONTROLLER
    wire    interrupt_flag_1;
    wire    interrupt_flag_2;
    wire    interrupt_flag_3;
    wire    RETI_1;
    wire    RETI_2;
    wire    RETI_3;
    wire    interrupt_handling_1;
    wire    interrupt_handling_2;
    wire    interrupt_handling_3;
    wire    interrupt_request_1;
    wire    interrupt_request_2;
    wire    interrupt_request_3; 
    wire    interrupt_enable_1; 
    wire    interrupt_enable_2; 
    wire    interrupt_enable_3; 
    // -- Reset interrupt 
    wire [DATA_WIDTH - 1:0]             rst_interrupt_option;
    // -- External interrupt
    wire                                exti_pin;                   
    wire                                exti_enable;                // maskable
    wire[1:0]                           exti_sense;                 // Rising - Falling - Change
    wire                                exti_debounce_option;       // Debounce enable
    // -- Timer interruption
    wire                                timer_interrupt_enable;     // maskable
    wire                                timer_interrupt_option;     // Overflow counter / Limit counter
    wire [PRESCALER_TIMER_WIDTH - 1:0]  timer_prescaler;
    wire [REGISTER_TIMER_WIDTH*2 - 1:0] timer_interrupt_limit_value;
                                                                   
    // SYNCHRONIZATION PRIMITIVE (sp)
    // -- Read-Handler of processor 1
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_rd_p1;
    wire [ADDR_WIDTH_DM - 1:0]          addr_rd_p1;
    wire [DATA_TYPE_WIDTH - 1:0]        data_type_rd_p1;
    wire                                rd_idle_p1;
    wire                                rd_ins_p1;
    wire                                rd_access_p1;
    wire                                rd_finish_p1;
    // -- Write-Handler of processor 1
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_wr_p1;
    wire [ADDR_WIDTH_DM - 1:0]          addr_wr_p1;
    wire [DATA_TYPE_WIDTH - 1:0]        data_type_wr_p1;
    wire                                wr_idle_p1;   
    wire                                wr_ins_p1;
    wire                                wr_access_p1;
    // -- Read-Handler of processor 2
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_rd_p2;
    wire [ADDR_WIDTH_DM - 1:0]          addr_rd_p2;
    wire [DATA_TYPE_WIDTH - 1:0]        data_type_rd_p2;
    wire                                rd_idle_p2;
    wire                                rd_ins_p2;
    wire                                rd_access_p2;
    wire                                rd_finish_p2;
    // -- Write-Handler of processor 2
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_wr_p2;
    wire [ADDR_WIDTH_DM - 1:0]          addr_wr_p2;
    wire [DATA_TYPE_WIDTH - 1:0]        data_type_wr_p2;
    wire                                wr_idle_p2;   
    wire                                wr_ins_p2;
    wire                                wr_access_p2;
    
    // DATA_MEMORY (dm)
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_wr_dm;
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_rd_dm;
    wire [DATA_TYPE_WIDTH - 1:0]        data_type_wr_dm;
    wire [DATA_TYPE_WIDTH - 1:0]        data_type_rd_dm;
    wire [ADDR_WIDTH_DM - 1:0]          addr_wr_dm;
    wire [ADDR_WIDTH_DM - 1:0]          addr_rd_dm;
    wire                                wr_ins_dm;
    wire                                rd_ins_dm;
    wire                                wr_idle_dm;
    wire                                rd_idle_dm;
    wire [DATA_WIDTH - 1:0]             reserved_registers  [0:RESERVED_REG_AMOUNT - 1];
    
    // PROGRAM_MEMORY (pm)
    wire [DATA_WIDTH - 1:0]             data_bus_wr_pm;
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_rd_pm;
    wire [ADDR_WIDTH_PM - 1:0]          addr_wr_pm;
    wire [ADDR_WIDTH_PM - 1:0]          addr_rd_pm;
    wire                                wr_ins_pm;
    wire                                rd_ins_pm;
    wire                                wr_idle_pm;
    wire                                rd_idle_pm;
    
    // GPIO 
    wire [DATA_WIDTH - 1:0]             PORT_A_CONFIGURATION;
    wire [DATA_WIDTH - 1:0]             PORT_B_CONFIGURATION;
    wire [DATA_WIDTH - 1:0]             PORT_C_CONFIGURATION;
    wire                                PORT_SPEC;  // External interrupt port
    wire [0:GPIO_PER_PORT - 1]          GPIO_PORT_A;
    wire [0:GPIO_PER_PORT - 1]          GPIO_PORT_B;
    wire [0:GPIO_PER_PORT - 1]          GPIO_PORT_C;
    // FIFO advanced
    wire [ADDDR_MAPPING_WIDTH - 1:0]    protocol_address_mapping;
    wire [DOUBLEWORD_WIDTH*2 - 1:0]     data_snd_protocol_per;
    wire [DOUBLEWORD_WIDTH*2 - 1:0]     data_rcv_protocol_per;
    wire                                send_protocol_clk;
    wire                                receive_protocol_clk;
    wire [AMOUNT_SND_WIDTH - 1:0]       amount_snd_byte_protocol;
    wire [AMOUNT_RCV_WIDTH - 1:0]       amount_rcv_byte_protocol;
    reg                                 snd_protocol_available;
    reg                                 rcv_protocol_available;
    wire [DATA_WIDTH - 1:0]             data_snd_small;
    wire                                snd_small_clk;
    
    `ifdef UART_PROT_1
    // UART_TX_1
    wire [DATA_WIDTH - 1:0]             data_bus_in_uart_1;
    wire [DATA_WIDTH - 1:0]             TX_config_register_1;
    wire                                TX_use_1;
    wire                                TX_complete_1;
    wire                                TX_enable_1;
    // UART_RX_1
    wire [DATA_WIDTH - 1:0]             data_bus_out_uart_1;
    wire [DATA_WIDTH - 1:0]             RX_config_register_1;
    wire                                RX_use_1;
    wire                                RX_flag_1;
    wire                                RX_enable_1;
    `endif
    
    `ifdef UART_PROT_2
    // UART_TX_2
    wire [DATA_WIDTH - 1:0]             data_bus_in_uart_2;
    wire [DATA_WIDTH - 1:0]             TX_config_register_2;
    wire                                TX_use_2;
    wire                                TX_complete_2;
    wire                                TX_enable_2;   
    // UART_RX_2
    wire [DATA_WIDTH - 1:0]             data_bus_out_uart_2;
    wire [DATA_WIDTH - 1:0]             RX_config_register_2;
    wire                                RX_flag_2;
    wire                                RX_enable_2;
    `endif
    
    `ifdef SPI_PROT
    wire                                SPI_enable;
    wire [DATA_WIDTH - 1:0]             SPI_config_register;
    wire [DATA_WIDTH - 1:0]             data_bus_in_spi;
    wire [DATA_WIDTH - 1:0]             data_bus_out_spi;
    wire                                SPI_use;
    wire                                SPI_read;
    wire                                SPI_write;
    wire                                SPI_avail;   // SPI_en (SPI available to read data)
    `endif
    `ifdef I2C_PROT
    // I2C (Uncertain)
    wire                                I2C_enable;
    wire [DATA_WIDTH - 1:0]             I2C_config_register;
    `endif
    ///////////////////////////////////////////////////////////////
    
    // Connect wire /////////////////////////////////////////////////////////////
    // Reserved register (configuration register)
    assign PORT_A_CONFIGURATION         =  reserved_registers[8'h00];
    assign PORT_B_CONFIGURATION         =  reserved_registers[8'h01];
    assign PORT_C_CONFIGURATION         =  reserved_registers[8'h02];
    assign TX_enable_1                  =  reserved_registers[8'h03][0];     // Debugger
    assign RX_config_register_1         =  reserved_registers[8'h04];
    assign TX_config_register_1         =  reserved_registers[8'h05];
    assign I2C_enable                   =  reserved_registers[8'h06][8'h00];
    assign SPI_enable                   =  reserved_registers[8'h06][8'h01];
    assign RX_enable_2                  =  reserved_registers[8'h06][8'h02];
    assign TX_enable_2                  =  reserved_registers[8'h06][8'h03];
    // Address encoder place            =  reserved_registers[8'h07]
    assign RX_config_register_2         =  reserved_registers[8'h08];
    assign TX_config_register_2         =  reserved_registers[8'h09];
    assign SPI_config_register          =  reserved_registers[8'h0A];
    assign I2C_config_register          =  reserved_registers[8'h0B];
    assign rst_interrupt_option         =  reserved_registers[8'h0C];
    assign exti_enable                  =  reserved_registers[8'h0D][8'h07];
    assign exti_sense                   =  reserved_registers[8'h0D][8'h06:8'h05];
    assign exti_debounce_option         =  reserved_registers[8'h0D][8'h04];
    assign timer_interrupt_enable       =  reserved_registers[8'h0E][8'h07];
    assign timer_interrupt_option       =  reserved_registers[8'h0E][8'h06];
    assign timer_prescaler              =  reserved_registers[8'h0E][8'h02:8'h00];
    assign timer_interrupt_limit_value  = {reserved_registers[8'h10],
                                           reserved_registers[8'h0F]};
    // GPIO
    for(genvar i = 0; i < GPIO_PER_PORT; i = i + 1) begin
    assign GPIO_PORT_A[i] = IO_PORT[i];
    assign IO_PORT[i] = (PORT_A_CONFIGURATION[i*2]) ? PORT_A_CONFIGURATION[i*2 + 1] : 1'bz;
    end
    for(genvar i = 0; i < GPIO_PER_PORT; i = i + 1) begin
    assign GPIO_PORT_B[i] = IO_PORT[4 + i];
    assign IO_PORT[4 + i] = (PORT_B_CONFIGURATION[i*2]) ? PORT_B_CONFIGURATION[i*2 + 1] : 1'bz;
    end
    for(genvar i = 0; i < GPIO_PER_PORT; i = i + 1) begin
    assign GPIO_PORT_C[i] = IO_PORT[8 + i];
    assign IO_PORT[8 + i] = (PORT_C_CONFIGURATION[i*2]) ? PORT_C_CONFIGURATION[i*2 + 1] : 1'bz;
    end
    assign exti_pin    = IO_PORT[12];
    assign IO_PORT[12] = (exti_enable) ? 1'bz : 1'b1;
    
    ////////////////////////////////////////////////////////////////////////////
    
    Processor           #(
                        .MAIN_RPOCESSOR(1'b1),
                        .DATA_MEMORY_SIZE(DATA_MEMORY_SIZE),
                        .PROGRAM_MEMORY_SIZE(PROGRAM_MEMORY_SIZE),
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
                        // Case 2 start
                        // Multi-processor manager
                        .fetch_instruction(fetch_instruction_1),
                        .boot_processor(boot_processor_1),
                        .processor_idle(processor_idle_1),
                        // Synchronization primitive
                        // - read
                        .data_bus_rd(data_bus_rd_p1),
                        .addr_rd(addr_rd_p1),
                        .data_type_rd(data_type_rd_p1),
                        .rd_idle(rd_idle_p1),
                        .rd_ins(rd_ins_p1),
                        .rd_access(rd_access_p1),
                        .rd_finish(rd_finish_p1),
                        // - write
                        .data_bus_wr(data_bus_wr_p1),
                        .addr_wr(addr_wr_p1),
                        .data_type_wr(data_type_wr_p1),
                        .wr_idle(wr_idle_p1),
                        .wr_ins(wr_ins_p1),
                        .wr_access(wr_access_p1),
                        
                        // Register maangement
                        .processor_registers(processor_registers_1),
                        .registers_renew(registers_renew),
                        .synchronization_processor(synchronization_processor_1),
                        
                        // GPIO
                        .GPIO_PORT_A(GPIO_PORT_A),
                        .GPIO_PORT_B(GPIO_PORT_B),
                        .GPIO_PORT_C(GPIO_PORT_C),
                        
                        .rst_n(rst_n)
                        
                        // Debug
//                        ,.debug_1(debug_1)
                        );                
    Processor           #(
                        .MAIN_RPOCESSOR(1'b0),
                        .DATA_MEMORY_SIZE(DATA_MEMORY_SIZE),
                        .PROGRAM_MEMORY_SIZE(PROGRAM_MEMORY_SIZE),
                        .FINISH_PROGRAM_OPCODE(FINISH_PROGRAM_OPCODE),
                        .FINISH_PROGRAM_TIMER(FINISH_PROGRAM_TIMER)
                        )
                        processor_2
                        (
                        .clk(clk),
                        // Case 2 start
                        // Multi-processor manager
                        .fetch_instruction(fetch_instruction_2),
                        .boot_processor(boot_processor_2),
                        .processor_idle(processor_idle_2),
                        // Synchronization primitive
                        // - read
                        .data_bus_rd(data_bus_rd_p2),
                        .addr_rd(addr_rd_p2),
                        .data_type_rd(data_type_rd_p2),
                        .rd_idle(rd_idle_p2),
                        .rd_ins(rd_ins_p2),
                        .rd_access(rd_access_p2),
                        .rd_finish(rd_finish_p2),
                        // - write
                        .data_bus_wr(data_bus_wr_p2),
                        .addr_wr(addr_wr_p2),
                        .data_type_wr(data_type_wr_p2),
                        .wr_idle(wr_idle_p2),
                        .wr_ins(wr_ins_p2),
                        .wr_access(wr_access_p2),
                        // Register maangement
                        .processor_registers(processor_registers_2),
                        .registers_renew(registers_renew),
                        .synchronization_processor(synchronization_processor_2),
                        // Protocol interface
                        .protocol_address_mapping(protocol_address_mapping),
                        // -- send
                        .data_snd_protocol_per(data_snd_protocol_per),
                        .send_protocol_clk(send_protocol_clk),
                        .amount_snd_byte_protocol(amount_snd_byte_protocol),
                        .snd_protocol_available(snd_protocol_available),
                        // -- receive
                        .data_rcv_protocol_per(data_rcv_protocol_per),
                        .receive_protocol_clk(receive_protocol_clk),
                        .amount_rcv_byte_protocol(amount_rcv_byte_protocol),
                        .rcv_protocol_available(rcv_protocol_available),
                        
                        .rst_n(rst_n)
                        
                        // Debug
//                        ,.debug_2(debug_2)
                        );   
                            
    Sync_primitive      #(
                        .DATA_MEMORY_SIZE(DATA_MEMORY_SIZE)
                        )synchronization_primitive(
                        .clk(clk),
                        // Processor 1
                        // - read
                        .data_bus_rd_p1(data_bus_rd_p1),
                        .addr_rd_p1(addr_rd_p1),
                        .data_type_rd_p1(data_type_rd_p1),
                        .rd_idle_p1(rd_idle_p1),
                        .rd_ins_p1(rd_ins_p1),
                        .rd_access_p1(rd_access_p1),
                        .rd_finish_p1(rd_finish_p1),
                        // - write
                        .data_bus_wr_p1(data_bus_wr_p1),
                        .addr_wr_p1(addr_wr_p1),
                        .data_type_wr_p1(data_type_wr_p1),
                        .wr_idle_p1(wr_idle_p1),
                        .wr_ins_p1(wr_ins_p1),
                        .wr_access_p1(wr_access_p1),
                        // Processor 2
                        // - read
                        .data_bus_rd_p2(data_bus_rd_p2),
                        .addr_rd_p2(addr_rd_p2),
                        .data_type_rd_p2(data_type_rd_p2),
                        .rd_idle_p2(rd_idle_p2),
                        .rd_ins_p2(rd_ins_p2),
                        .rd_access_p2(rd_access_p2),
                        .rd_finish_p2(rd_finish_p2),
                        // - write
                        .data_bus_wr_p2(data_bus_wr_p2),
                        .addr_wr_p2(addr_wr_p2),
                        .data_type_wr_p2(data_type_wr_p2),
                        .wr_idle_p2(wr_idle_p2),
                        .wr_ins_p2(wr_ins_p2),
                        .wr_access_p2(wr_access_p2),
                        // Data memory
                        // -- read
                        .data_bus_rd_dm(data_bus_rd_dm),
                        .addr_rd_dm(addr_rd_dm),
                        .data_type_rd_dm(data_type_rd_dm),
                        .rd_idle_dm(rd_idle_dm),
                        .rd_ins_dm(rd_ins_dm),
                        // -- write
                        .data_bus_wr_dm(data_bus_wr_dm),
                        .addr_wr_dm(addr_wr_dm),
                        .data_type_wr_dm(data_type_wr_dm),
                        .wr_idle_dm(wr_idle_dm),
                        .wr_ins_dm(wr_ins_dm),
        
                        .rst_n(rst_n)
                        );
    Registers_management#(
                        )registers_management(
                        .clk(clk),
                        .processor_registers_1(processor_registers_1),
                        .processor_registers_2(processor_registers_2),
                        .processor_idle_1(processor_idle_1),
                        .processor_idle_2(processor_idle_2),
                        .boot_renew_register_1(boot_renew_register_1),
                        .boot_renew_register_2(boot_renew_register_2),
                        .boot_renew_3registers_2(boot_renew_3registers_2),
                        .register_num(register_num),
                        .new_data_register(new_data_register),
                        .registers_renew(registers_renew),
                        .ra_register(ra_register),
                        .processing_register_table(processing_register_table),
                        // New synchronization
                        .synchronization_processor_1(synchronization_processor_1),
                        .synchronization_processor_2(synchronization_processor_2),
                        .synchronized_processors(synchronized_processors),
                        .rst_n(rst_n)
                        );
    Multi_processor_manager #(
                        .PROGRAM_MEMORY_SIZE(PROGRAM_MEMORY_SIZE)
                        ) multi_processor_manager (
                        .clk(clk),
                        // Program memory
                        .data_bus_rd_pm(data_bus_rd_pm),
                        .rd_idle_pm(rd_idle_pm),
                        .addr_rd_pm(addr_rd_pm),
                        .rd_ins_pm(rd_ins_pm),
                        // Processor 1
                        .main_state(main_state),
                        .fetch_instruction_1(fetch_instruction_1),
                        .boot_processor_1(boot_processor_1),
                        .processor_idle_1(processor_idle_1),
                        // Processor 2
                        .fetch_instruction_2(fetch_instruction_2),
                        .boot_processor_2(boot_processor_2),
                        .processor_idle_2(processor_idle_2),
                        // Register management 
                        .registers_renew(registers_renew),
                        .ra_register(ra_register),
                        .register_num(register_num),
                        .boot_renew_register_1(boot_renew_register_1),
                        .boot_renew_register_2(boot_renew_register_2),
                        .boot_renew_3registers_2(boot_renew_3registers_2),
                        .synchronized_processors(synchronized_processors),
                        .processing_register_table(processing_register_table),
                        // Interrup control
                        .interrupt_flag_1(interrupt_flag_1),
                        .interrupt_flag_2(interrupt_flag_2),
                        .interrupt_flag_3(interrupt_flag_3),
                        .RETI_1(RETI_1),
                        .RETI_2(RETI_2),
                        .RETI_3(RETI_3),
                        .interrupt_handling_1(interrupt_handling_1),
                        .interrupt_handling_2(interrupt_handling_2),
                        .interrupt_handling_3(interrupt_handling_3),
                        // Hardware support instruction
                        .rd_idle_dm(rd_idle_dm),
                        .wr_idle_dm(wr_idle_dm),
                        
                        .rst_n(rst_n)
                        );
    
    // Data Memory (8Kb - 1kB)
    ram                 #(
                        .ADDR_DEPTH(DATA_MEMORY_SIZE),
                        .RESERVED_REG_AMOUNT(RESERVED_REG_AMOUNT),
                        .RESERVED_REG_DEFAULT(RESERVED_REG_DEFAULT)
                        )data_memory(
                        .clk(clk),
                        // -- write
                        .data_bus_wr(data_bus_wr_dm),
                        .data_type_wr(data_type_wr_dm),    // Write byte (8bit)
                        .addr_wr(addr_wr_dm),
                        .wr_ins(wr_ins_dm),
                        .wr_idle(wr_idle_dm),
                        // -- read
                        .data_bus_rd(data_bus_rd_dm),
                        .rd_idle(rd_idle_dm),
                        .addr_rd(addr_rd_dm),
                        .rd_ins(rd_ins_dm),
                        .data_type_rd(data_type_rd_dm),
                        // -- reserved register
                        .reserved_registers(reserved_registers),
                        .rst_n(rst_n)
                        //Debug 
                        `ifdef DEBUG
                        ,.registers_wire(data_memory_wire)
                        `endif
                        );
    // Program memory (8Kb)
    ram_pm              #(
                        .ADDR_DEPTH(PROGRAM_MEMORY_SIZE),
                        .RESERVED_REG_AMOUNT(1'b1)
                        )program_memory(
                        .clk(clk),
                        .data_bus_wr(data_bus_wr_pm),
                        .data_type_wr(2'b00),    // Write byte (8bit)
                        .addr_wr(addr_wr_pm),
                        .wr_ins(wr_ins_pm),
                        .wr_idle(wr_idle_pm),
                        // Read region
                        .data_bus_rd(data_bus_rd_pm),
                        .rd_idle(rd_idle_pm),
                        .addr_rd(addr_rd_pm),
                        .rd_ins(rd_ins_pm),
                        .data_type_rd(2'b10),
                        .rst_n(rst_n)
                        //Debug 
                        `ifdef DEBUG
                        ,.registers_wire(program_memory_wire)
                        `endif
                        );
    Interrupt_controller
                        #(
                        )interrupt_controller(
                        .interrupt_flag_1(interrupt_flag_1),
                        .interrupt_flag_2(interrupt_flag_2),
                        .interrupt_flag_3(interrupt_flag_3),
                        .RETI_1(RETI_1),
                        .RETI_2(RETI_2),
                        .RETI_3(RETI_3),
                        .interrupt_handling_1(interrupt_handling_1),
                        .interrupt_handling_2(interrupt_handling_2),
                        .interrupt_handling_3(interrupt_handling_3),
                        .interrupt_request_1(interrupt_request_1),
                        .interrupt_request_2(interrupt_request_2),
                        .interrupt_request_3(interrupt_request_3),
                        .interrupt_enable_1(interrupt_enable_1),
                        .interrupt_enable_2(interrupt_enable_2),
                        .interrupt_enable_3(interrupt_enable_3),
                        .rst_n(rst_n)
                        );  
    timer_INT_handler timer_interrupt_handler
                        (
                        .clk(clk),
                        .enable_interrupt(timer_interrupt_enable),
                        .interrupt_option(timer_interrupt_option),
                        .prescaler_selector(timer_prescaler),
                        .timer_limit_value(timer_interrupt_limit_value),
                        .interrupt_request(interrupt_request_3),
                        .rst_n(rst_n)
                        );
    external_INT_handler external_interrupt_handler
                        (
                        .clk(clk),
                        .int_pin(exti_pin),
                        .enable_interrupt(exti_enable),
                        .interrupt_sense_control(exti_sense),
                        .debounce_option(exti_debounce_option),
                        .interrupt_request(interrupt_request_1),
                        .rst_n(rst_n)
                        );           
                                 
    fifo_advanced_module #(
                        )fifo_advanced_module(
                        .clk(clk),
                        // TX
                        // -- to Processor
                        .snd_big_clk(send_protocol_clk),
                        .amount_byte_snd_big(amount_snd_byte_protocol),
                        .data_snd_big(data_snd_protocol_per),
                        .available_snd(snd_protocol_available),
                        // -- to UART
                        .data_snd_small(data_bus_in_uart_2),
                        .snd_small_clk(TX_use_2),
                        .snd_small_available(TX_available_2),
                        // RX 
                        // -- to Processor
                        .rcv_big_clk(receive_protocol_clk),
                        .amount_byte_rcv_big(amount_rcv_byte_protocol),
                        .data_rcv_big(data_rcv_protocol_per),
                        .available_rcv(rcv_protocol_available),
                        // -- to UART
                        .data_rcv_small(data_bus_out_uart_2),
                        .rcv_small_clk(RX_use_2),
                        .rcv_small_available(RX_available_2),
                        `ifdef DEBUG_DATA
                        .queue_snd_wire(queue_snd_wire),
                        .queue_rcv_wire(queue_rcv_wire),
                        `endif
                        
                        .rst_n(rst_n)
                        );        
    // Communication peripherals    
    `ifdef UART_PROT_1        
    com_uart            #(
                        .SLEEP_MODE(0), 
                        .RX_FLAG_CONFIG(1'b1),              // Use internal FIFO
                        .FIFO_DEPTH(FIFO_BUFFER_SIZE),
                        `ifdef DEBUG
                        .CLOCK_DIVIDER_UNIQUE_1(CLOCK_DIVIDER_UNIQUE_1)
                        `endif
                        )uart_peripheral_1(
                        .clk(clk),
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
                        .RX_enable(1'b1),
                        .rst_n(rst_n)
                        );
    `endif
    `ifdef UART_PROT_2
    com_uart            #(
                        .SLEEP_MODE(0), 
                        .RX_FLAG_CONFIG(1'b1), /// Internal FIFO
                        `ifdef DEBUG
                        .CLOCK_DIVIDER_UNIQUE_1(CLOCK_DIVIDER_UNIQUE_1)
                        `endif
                        )             
                        uart_peripheral_2
                        (
                        .clk(clk),
                        // TX 
                        .data_bus_in(data_bus_in_uart_2),
                        .TX_use(TX_use_2),
                        .TX_available(TX_available_2),
                        .TX_complete(TX_complete_2),
                        .TX_config_register(TX_config_register_2),
                        .TX_enable(TX_enable_2),
                        .TX(TX_2),
                        // RX
                        .data_bus_out(data_bus_out_uart_2),
                        .RX_use(RX_use_2),
                        .RX_available(RX_available_2),
                        .RX_config_register(RX_config_register_2),
                        .RX_enable(RX_enable_2),
                        .RX(RX_2),
                        
                        .rst_n(rst_n)
                        );      
    `endif
    `ifdef SPI_PROT                    
    com_spi             #(
                        )spi_peripheral(
                        );                                                                                     
    `endif
endmodule

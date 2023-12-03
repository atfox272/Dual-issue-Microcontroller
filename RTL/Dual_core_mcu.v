`timescale 1ns / 1ps
`define DEBUG
`define EXTI
`define TIM
`define UART_PROT_1    
`define UART_PROT_2 
//`define SPI_PROT
//`define I2C_PROT
module Dual_core_mcu
    #(
    
    parameter INTERNAL_CLOCK    = 25000000,
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
    // Data memory 
    parameter      DATA_MEMORY_SIZE                                =  256,      // 256 bytes (2Kb)
    parameter      RESERVED_REG_AMOUNT                             =  17,
    parameter byte RESERVED_REG_DEFAULT[0:RESERVED_REG_AMOUNT - 1] = {8'b00000000,  // address 0x00  (PORT_A)
                                                                      8'b00000000,  // address 0x01  (PORT_B)
                                                                      8'b00000000,  // address 0x02  (PORT_C)
                                                                      8'b00000000,  // address 0x03  (DEBUGGER) 
                                                                      8'b00100011,  // address 0x04  (UART_1_RX_CONFIG) 
                                                                      8'b00100011,  // address 0x05  (UART_1_TX_CONFIG)
                                                                      8'b00001100,  // address 0x06  (COM_PERIPHERAL) // Do not enable (set 1) in initial state
                                                                      8'b00000000,  // address 0x07  (NOTHING)
                                                                      8'b00100011,  // address 0x08  (UART_2_RX_CONFIG)
                                                                      8'b00100011,  // address 0x09  (UART_2_TX_CONFIG)
                                                                      8'b11111000,  // address 0x0A  (SPI_CONFIG)
                                                                      8'b00000000,  // address 0x0B  (I2C_CONFIG)
                                                                      8'b00000000,  // address 0x0C  (EXTERNAL_INT_CONFIG)
                                                                      8'b00000000,  // address 0x0D  (PINCHANGE_INT_CONFIG)
                                                                      8'b00000000,  // address 0x0E  (TIMER_INT_CONFIG)
                                                                      8'b11111111,  // address 0x0F  (TIMER_LIMIT_VALUE_H)
                                                                      8'b11111111}, // address 0x10  (TIMER_LIMIT_VALUE_L)
    // System Bus Structure
    /* DATA_BUS */
    parameter DATA_BUS_WIDTH        = 64,
    /* ADDRESS_BUS */
    parameter ADDR_BUS_WIDTH        = 64,
    parameter CHANNEL_AMOUNT        = 4,    // 4 device for Processor 3(DMEM - UART1 - UART2 - GPIO)
    parameter CHANNEL_BUS_WIDTH     = $clog2(CHANNEL_AMOUNT),
    parameter CHANNEL_ID_DMEM       = 2'b00,
    parameter CHANNEL_ID_GPIO       = 2'b01,
    parameter CHANNEL_ID_UART1      = 2'b10,
    parameter CHANNEL_ID_UART2      = 2'b11,
    /* Interface Encode */
    parameter INTERFACE_MASTER_ENCODE = 3,
    parameter INTERFACE_DMEM_ENCODE   = 0,
    parameter INTERFACE_PERP_ENCODE   = 1,
    parameter INTERFACE_GPIO_ENCODE   = 2,
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
    // GPIO (Total = GPIO_PORT_AMOUNT * GPIO_PIN_AMOUNT)
    parameter GPIO_PORT_AMOUNT      = 2,
    parameter GPIO_PIN_AMOUNT       = 8,    // Pins per Port
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
    // External Interrupt
    parameter EXT_GPIO_PORT_INDEX   = 0,
    parameter EXT_GPIO_PIN_INDEX    = 2,
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
    parameter RUNNING_PROGRAM_STATE = 2,
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
    
    output                              RUNNING_PROGRAM,
    // GPIO
    inout   [GPIO_PIN_AMOUNT - 1:0]     GPIO_PORT [0:GPIO_PORT_AMOUNT - 1],
    
    // Reset negedge
    input   wire                        rst
    
    `ifdef DEBUG
//    ,output                             clk_out
//    ,output  wire    [DATA_WIDTH - 1:0]      program_memory_wire [0: PROGRAM_MEMORY_SIZE - 1]
//    ,output  wire    [DATA_WIDTH - 1:0]      data_memory_wire [0: DATA_MEMORY_SIZE - 1]
    `endif
    );
    
    wire                                system_tick_25;
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
    wire    [REG_SPACE_WIDTH - 1:0]     register_num;
    wire                                boot_renew_register_1;
    wire                                boot_renew_register_2;
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
    // -- Write-Handler of processor 1
    wire                                wr_idle_p1;   
    // -- Write-Handler of processor 2
    wire                                wr_idle_p2;   
    // System Bus Structure (ATI Bus structure) 
    /* Master Interface for Processor 1*/
    wire [DATA_BUS_WIDTH - 1:0]         m_ati_p1_wdata;
    wire [DATA_BUS_WIDTH - 1:0]         m_ati_p1_rdata;
    wire [ADDR_BUS_WIDTH - 1:0]         m_ati_p1_addr;
    wire                                m_ati_p1_rd_req;
    wire                                m_ati_p1_wr_req;
    wire [DATA_TYPE_WIDTH - 1:0]        m_ati_p1_data_type;
    wire                                m_ati_p1_rd_available;
    wire                                m_ati_p1_wr_available;
    /* Master Stream for Processor 1*/
    wire [DATA_BUS_WIDTH - 1:0]         m_atis_p1_rdata_bus    [0:CHANNEL_AMOUNT - 1];
    wire [DATA_BUS_WIDTH - 1:0]         m_atis_p1_wdata_bus;
    wire [ADDR_BUS_WIDTH - 1:0]         m_atis_p1_addr_bus;
    wire                                m_atis_p1_rd_req;
    wire                                m_atis_p1_wr_req;
    wire [DATA_TYPE_WIDTH - 1:0]        m_atis_p1_data_type;
    wire                                m_atis_p1_rd_available [0:CHANNEL_AMOUNT - 1];
    wire                                m_atis_p1_wr_available [0:CHANNEL_AMOUNT - 1];
    /* Master Interface for Processor 2*/
    wire [DATA_BUS_WIDTH - 1:0]         m_ati_p2_wdata;
    wire [DATA_BUS_WIDTH - 1:0]         m_ati_p2_rdata;
    wire [ADDR_BUS_WIDTH - 1:0]         m_ati_p2_addr;
    wire                                m_ati_p2_rd_req;
    wire                                m_ati_p2_wr_req;
    wire [DATA_TYPE_WIDTH - 1:0]        m_ati_p2_data_type;
    wire                                m_ati_p2_rd_available;
    wire                                m_ati_p2_wr_available;
    /* Master Stream for Processor 1*/
    wire [DATA_BUS_WIDTH - 1:0]         m_atis_p2_rdata_bus    [0:CHANNEL_AMOUNT - 1];
    wire [DATA_BUS_WIDTH - 1:0]         m_atis_p2_wdata_bus;
    wire [ADDR_BUS_WIDTH - 1:0]         m_atis_p2_addr_bus;
    wire                                m_atis_p2_rd_req;
    wire                                m_atis_p2_wr_req;
    wire [DATA_TYPE_WIDTH - 1:0]        m_atis_p2_data_type;
    wire                                m_atis_p2_rd_available [0:CHANNEL_AMOUNT - 1];
    wire                                m_atis_p2_wr_available [0:CHANNEL_AMOUNT - 1];
    /* Slave Interface (UART_1) */ /* Use For Debugger */ 
    wire [DATA_WIDTH - 1:0]             s_ati_uart1_data_in;
    wire [DATA_WIDTH - 1:0]             s_ati_uart1_data_out;
    wire                                s_ati_uart1_rd_req;
    wire                                s_ati_uart1_wr_req;
    wire                                s_ati_uart1_rd_available;
    wire                                s_ati_uart1_wr_available;
    /* Slave Interface (UART_2) */
    wire [DATA_WIDTH - 1:0]             s_ati_uart2_data_in;
    wire [DATA_WIDTH - 1:0]             s_ati_uart2_data_out;
    wire                                s_ati_uart2_rd_req;
    wire                                s_ati_uart2_wr_req;
    wire                                s_ati_uart2_rd_available;
    wire                                s_ati_uart2_wr_available;
    /* Slave Interface (Data memory handler_1) */
    wire [DATA_BUS_WIDTH - 1:0]         s_ati_dmem_p1_data_in;
    wire [DATA_BUS_WIDTH - 1:0]         s_ati_dmem_p1_data_out;
    wire                                s_ati_dmem_p1_rd_req;
    wire                                s_ati_dmem_p1_wr_req;
    wire                                s_ati_dmem_p1_rd_available;
    wire                                s_ati_dmem_p1_wr_available;
    wire [ADDR_BUS_WIDTH - 1:0]         s_ati_dmem_p1_addr_rd;
    wire [ADDR_BUS_WIDTH - 1:0]         s_ati_dmem_p1_addr_wr;
    wire [DATA_TYPE_WIDTH - 1:0]        s_ati_dmem_p1_data_type_rd;
    wire [DATA_TYPE_WIDTH - 1:0]        s_ati_dmem_p1_data_type_wr;
    /* Slave Interface (Data memory handler_2) */
    wire [DATA_BUS_WIDTH - 1:0]         s_ati_dmem_p2_data_in;
    wire [DATA_BUS_WIDTH - 1:0]         s_ati_dmem_p2_data_out;
    wire                                s_ati_dmem_p2_rd_req;
    wire                                s_ati_dmem_p2_wr_req;
    wire                                s_ati_dmem_p2_rd_available;
    wire                                s_ati_dmem_p2_wr_available;
    wire [ADDR_BUS_WIDTH - 1:0]         s_ati_dmem_p2_addr_rd;
    wire [ADDR_BUS_WIDTH - 1:0]         s_ati_dmem_p2_addr_wr;
    wire [DATA_TYPE_WIDTH - 1:0]        s_ati_dmem_p2_data_type_rd;
    wire [DATA_TYPE_WIDTH - 1:0]        s_ati_dmem_p2_data_type_wr;
    /* Slave Interface (GPIO) */
    wire [GPIO_PIN_AMOUNT - 1:0]        s_ati_gpio_data_in;
    wire [GPIO_PIN_AMOUNT - 1:0]        s_ati_gpio_data_out;
    wire                                s_ati_gpio_rd_req;
    wire                                s_ati_gpio_wr_req;
    wire                                s_ati_gpio_rd_available;
    wire                                s_ati_gpio_wr_available;
    wire [ADDR_BUS_WIDTH - 1:0]         s_ati_gpio_addr_rd;
    wire [ADDR_BUS_WIDTH - 1:0]         s_ati_gpio_addr_wr;
    wire [GPIO_PIN_AMOUNT - 1:0]        GPIO_PORT_i [0:GPIO_PORT_AMOUNT - 1];
    wire [GPIO_PIN_AMOUNT - 1:0]        GPIO_PORT_o [0:GPIO_PORT_AMOUNT - 1];
    // DATA_MEMORY (dm)
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_wr_dm;
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_rd_dm;
    wire [DATA_TYPE_WIDTH - 1:0]        data_type_wr_dm;
    wire [DATA_TYPE_WIDTH - 1:0]        data_type_rd_dm;
    wire [ADDR_WIDTH_DM - 1:0]          addr_wr_dm;
    wire                                wr_ins_dm;
    wire                                rd_ins_dm;
    wire                                wr_idle_dm;
    wire                                rd_idle_dm;
    wire [DATA_WIDTH - 1:0]             reserved_registers  [0:RESERVED_REG_AMOUNT - 1];
    
    // PROGRAM_MEMORY (pm)
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_wr_pm;
    wire [DOUBLEWORD_WIDTH - 1:0]       data_bus_rd_pm;
    wire [ADDR_WIDTH_PM - 1:0]          addr_wr_pm;
    wire [ADDR_WIDTH_PM - 1:0]          addr_rd_pm;
    wire                                wr_ins_pm;
    wire                                rd_ins_pm;
    wire                                wr_idle_pm;
    wire                                rd_idle_pm;
    
    // GPIO 
    wire [GPIO_PIN_AMOUNT - 1:0]        PORT_CONFIGURATION [0:GPIO_PORT_AMOUNT - 1];
    
    `ifdef UART_PROT_1
    // UART_TX_1
    wire [DATA_WIDTH - 1:0]             data_bus_in_uart_1;
    wire [DATA_WIDTH - 1:0]             TX_config_register_1;
    wire                                TX_use_1;
    wire                                TX_available_1;
    wire                                TX_enable_1;
    // UART_RX_1
    wire [DATA_WIDTH - 1:0]             RX_config_register_1;
    wire                                RX_use_1;
    wire                                RX_use_1_processor;
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
    assign PORT_CONFIGURATION           = {reserved_registers[8'h00], 
                                           reserved_registers[8'h01]};
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
    assign timer_interrupt_limit_value  = {reserved_registers[8'h0F],
                                           reserved_registers[8'h10]};
    
    // External Interrupt pin 
    assign exti_pin = GPIO_PORT_i[EXT_GPIO_PORT_INDEX][EXT_GPIO_PIN_INDEX];    
    
    /* Clock Generator */
    clk_wiz_0 system_tick
        ( 
        .clk_in1(clk),
        .clk_out1(system_tick_25)
        );
    /* End Clock generator */
    Processor           
        #(
        .MAIN_RPOCESSOR(1'b1),
        .DATA_MEMORY_SIZE(DATA_MEMORY_SIZE),
        .PROGRAM_MEMORY_SIZE(PROGRAM_MEMORY_SIZE),
        .FINISH_PROGRAM_OPCODE(FINISH_PROGRAM_OPCODE),
        .FINISH_PROGRAM_TIMER(FINISH_PROGRAM_TIMER)
        )
        PROCESSOR_1
        (
        .clk(system_tick_25),
        // UART_1
        .data_bus_out_uart_1(s_ati_uart1_data_in),
        .RX_use_1(RX_use_1_processor),
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
        // - common
        .addr_dmem(m_ati_p1_addr),
        .data_type_dmem(m_ati_p1_data_type),
        // - read
        .data_bus_rd(m_ati_p1_rdata),
        .rd_idle(m_ati_p1_rd_available),
        .rd_ins(m_ati_p1_rd_req),
        // - write
        .data_bus_wr(m_ati_p1_wdata),
        .wr_idle(wr_idle_p1),           // 
        .wr_ins(m_ati_p1_wr_req),
        .wr_access(m_ati_p1_wr_available),
        
        // Register maangement
        .processor_registers(processor_registers_1),
        .registers_renew(registers_renew),
        .synchronization_processor(synchronization_processor_1),
        .rst_n(~rst)
        
        );                 
    Processor           
        #(
        .MAIN_RPOCESSOR(1'b0),
        .DATA_MEMORY_SIZE(DATA_MEMORY_SIZE),
        .PROGRAM_MEMORY_SIZE(PROGRAM_MEMORY_SIZE),
        .FINISH_PROGRAM_OPCODE(FINISH_PROGRAM_OPCODE),
        .FINISH_PROGRAM_TIMER(FINISH_PROGRAM_TIMER)
        )
        PROCESSOR_2
        (
        .clk(system_tick_25),
        // Case 2 start
        // Multi-processor manager
        .fetch_instruction(fetch_instruction_2),
        .boot_processor(boot_processor_2),
        .processor_idle(processor_idle_2),
        // Synchronization primitive
        // - common
        .addr_dmem(m_ati_p2_addr),
        .data_type_dmem(m_ati_p2_data_type),
        // - read
        .data_bus_rd(m_ati_p2_rdata),
        .rd_idle(m_ati_p2_rd_available),
        .rd_ins(m_ati_p2_rd_req),
        // - write
        .data_bus_wr(m_ati_p2_wdata),
        .wr_idle(wr_idle_p2),           // 
        .wr_ins(m_ati_p2_wr_req),
        .wr_access(m_ati_p2_wr_available),
        // Register maangement
        .processor_registers(processor_registers_2),
        .registers_renew(registers_renew),
        .synchronization_processor(synchronization_processor_2),
        
        .rst_n(~rst)
        
        );  
    Atfox_exTensible_Interface
        #(
        .CHANNEL_ID(),
        .INTERFACE_MASTER_ENCODE(INTERFACE_MASTER_ENCODE),
        .INTERFACE_DMEM_ENCODE(INTERFACE_DMEM_ENCODE),
        .INTERFACE_PERP_ENCODE(INTERFACE_PERP_ENCODE),
        .INTERFACE_GPIO_ENCODE(INTERFACE_GPIO_ENCODE),
        .INTERFACE_TYPE(INTERFACE_MASTER_ENCODE)
        ) ATI_PROC1_BUS1 (
        .clk(system_tick_25),
        .m_ati_rdata(m_ati_p1_rdata),
        .m_ati_wdata(m_ati_p1_wdata),
        .m_ati_addr(m_ati_p1_addr),
        .m_ati_rd_available(m_ati_p1_rd_available),
        .m_ati_wr_available(m_ati_p1_wr_available),
        .m_ati_rd_req(m_ati_p1_rd_req),
        .m_ati_wr_req(m_ati_p1_wr_req),
        .m_ati_data_type(m_ati_p1_data_type),
        .m_atis_rdata_bus(m_atis_p1_rdata_bus),
        .m_atis_wdata_bus(m_atis_p1_wdata_bus),
        .m_atis_addr_bus(m_atis_p1_addr_bus),
        .m_atis_rd_available(m_atis_p1_rd_available),
        .m_atis_wr_available(m_atis_p1_wr_available),
        .m_atis_rd_req(m_atis_p1_rd_req),
        .m_atis_wr_req(m_atis_p1_wr_req),
        .m_atis_data_type(m_atis_p1_data_type),
        .s_atis_wdata(),
        .s_atis_rdata(),
        .s_atis_addr(),
        .s_atis_rd_available(),
        .s_atis_wr_available(),
        .s_atis_rd_req(),
        .s_atis_wr_req(),
        .s_atis_data_type(),
        .s_ati_rdata(),
        .s_ati_wdata(),
        .s_ati_wr_req(),
        .s_ati_rd_req(),
        .s_ati_rd_available(),
        .s_ati_wr_available(),
        .s_ati_raddr(),
        .s_ati_waddr(),
        .s_ati_rdata_type(),
        .s_ati_wdata_type(),
        .rst_n(~rst)
        );  
    Atfox_exTensible_Interface
        #(
        .CHANNEL_ID(),
        .INTERFACE_MASTER_ENCODE(INTERFACE_MASTER_ENCODE),
        .INTERFACE_DMEM_ENCODE(INTERFACE_DMEM_ENCODE),
        .INTERFACE_PERP_ENCODE(INTERFACE_PERP_ENCODE),
        .INTERFACE_GPIO_ENCODE(INTERFACE_GPIO_ENCODE),
        .INTERFACE_TYPE(INTERFACE_MASTER_ENCODE)
        ) ATI_PROC2_BUS2 (
        .clk(system_tick_25),
        .m_ati_rdata(m_ati_p2_rdata),
        .m_ati_wdata(m_ati_p2_wdata),
        .m_ati_addr(m_ati_p2_addr),
        .m_ati_rd_available(m_ati_p2_rd_available),
        .m_ati_wr_available(m_ati_p2_wr_available),
        .m_ati_rd_req(m_ati_p2_rd_req),
        .m_ati_wr_req(m_ati_p2_wr_req),
        .m_ati_data_type(m_ati_p2_data_type),
        .m_atis_rdata_bus(m_atis_p2_rdata_bus),
        .m_atis_wdata_bus(m_atis_p2_wdata_bus),
        .m_atis_addr_bus(m_atis_p2_addr_bus),
        .m_atis_rd_available(m_atis_p2_rd_available),
        .m_atis_wr_available(m_atis_p2_wr_available),
        .m_atis_rd_req(m_atis_p2_rd_req),
        .m_atis_wr_req(m_atis_p2_wr_req),
        .m_atis_data_type(m_atis_p2_data_type),
        .s_atis_wdata(),
        .s_atis_rdata(),
        .s_atis_addr(),
        .s_atis_rd_available(),
        .s_atis_wr_available(),
        .s_atis_rd_req(),
        .s_atis_wr_req(),
        .s_atis_data_type(),
        .s_ati_rdata(),
        .s_ati_wdata(),
        .s_ati_wr_req(),
        .s_ati_rd_req(),
        .s_ati_rd_available(),
        .s_ati_wr_available(),
        .s_ati_raddr(),
        .s_ati_waddr(),
        .s_ati_rdata_type(),
        .s_ati_wdata_type(),
        .rst_n(~rst)
        );  
    Atfox_exTensible_Interface
        #(
        .CHANNEL_ID(CHANNEL_ID_DMEM),
        .INTERFACE_DMEM_ENCODE(INTERFACE_DMEM_ENCODE),
        .INTERFACE_PERP_ENCODE(INTERFACE_PERP_ENCODE),
        .INTERFACE_GPIO_ENCODE(INTERFACE_GPIO_ENCODE),
        .INTERFACE_TYPE(INTERFACE_DMEM_ENCODE)
        ) ATI_DMEM_BUS1 (
        .clk(system_tick_25),
        .m_ati_rdata(),
        .m_ati_wdata(),
        .m_ati_addr(),
        .m_ati_rd_available(),
        .m_ati_wr_available(),
        .m_ati_rd_req(),
        .m_ati_wr_req(),
        .m_ati_data_type(),
        .m_atis_rdata_bus(),
        .m_atis_wdata_bus(),
        .m_atis_addr_bus(),
        .m_atis_rd_available(),
        .m_atis_wr_available(),
        .m_atis_rd_req(),
        .m_atis_wr_req(),
        .m_atis_data_type(), 
        .s_atis_wdata(m_atis_p1_wdata_bus),
        .s_atis_rdata(m_atis_p1_rdata_bus[CHANNEL_ID_DMEM]),
        .s_atis_addr(m_atis_p1_addr_bus),
        .s_atis_rd_available(m_atis_p1_rd_available[CHANNEL_ID_DMEM]),
        .s_atis_wr_available(m_atis_p1_wr_available[CHANNEL_ID_DMEM]),
        .s_atis_rd_req(m_atis_p1_rd_req),
        .s_atis_wr_req(m_atis_p1_wr_req),
        .s_atis_data_type(m_atis_p1_data_type),
        .s_ati_rdata(s_ati_dmem_p1_data_in),
        .s_ati_wdata(s_ati_dmem_p1_data_out),
        .s_ati_rd_req(s_ati_dmem_p1_rd_req),
        .s_ati_wr_req(s_ati_dmem_p1_wr_req),
        .s_ati_rd_available(s_ati_dmem_p1_rd_available),
        .s_ati_wr_available(s_ati_dmem_p1_wr_available),
        .s_ati_raddr(s_ati_dmem_p1_addr_rd),
        .s_ati_waddr(s_ati_dmem_p1_addr_wr),
        .s_ati_rdata_type(s_ati_dmem_p1_data_type_rd),
        .s_ati_wdata_type(s_ati_dmem_p1_data_type_wr),
        .rst_n(~rst)
        );
      
    Atfox_exTensible_Interface
        #(
        .CHANNEL_ID(CHANNEL_ID_DMEM),
        .INTERFACE_DMEM_ENCODE(INTERFACE_DMEM_ENCODE),
        .INTERFACE_PERP_ENCODE(INTERFACE_PERP_ENCODE),
        .INTERFACE_GPIO_ENCODE(INTERFACE_GPIO_ENCODE),
        .INTERFACE_TYPE(INTERFACE_DMEM_ENCODE)  
        ) ATI_DMEM_BUS2 (
        .clk(system_tick_25),
        .m_ati_rdata(),
        .m_ati_wdata(),
        .m_ati_addr(),
        .m_ati_rd_available(),
        .m_ati_wr_available(),
        .m_ati_rd_req(),
        .m_ati_wr_req(),
        .m_ati_data_type(),
        .m_atis_rdata_bus(),
        .m_atis_wdata_bus(),
        .m_atis_addr_bus(),
        .m_atis_rd_available(),
        .m_atis_wr_available(),
        .m_atis_rd_req(),
        .m_atis_wr_req(),
        .m_atis_data_type(), 
        .s_atis_wdata(m_atis_p2_wdata_bus),
        .s_atis_rdata(m_atis_p2_rdata_bus[CHANNEL_ID_DMEM]),
        .s_atis_addr(m_atis_p2_addr_bus),
        .s_atis_rd_available(m_atis_p2_rd_available[CHANNEL_ID_DMEM]),
        .s_atis_wr_available(m_atis_p2_wr_available[CHANNEL_ID_DMEM]),
        .s_atis_rd_req(m_atis_p2_rd_req),
        .s_atis_wr_req(m_atis_p2_wr_req),
        .s_atis_data_type(m_atis_p2_data_type),
        .s_ati_rdata(s_ati_dmem_p2_data_in),
        .s_ati_wdata(s_ati_dmem_p2_data_out),
        .s_ati_rd_req(s_ati_dmem_p2_rd_req),
        .s_ati_wr_req(s_ati_dmem_p2_wr_req),
        .s_ati_rd_available(s_ati_dmem_p2_rd_available),
        .s_ati_wr_available(s_ati_dmem_p2_wr_available),
        .s_ati_raddr(s_ati_dmem_p2_addr_rd),
        .s_ati_waddr(s_ati_dmem_p2_addr_wr),
        .s_ati_rdata_type(s_ati_dmem_p2_data_type_rd),
        .s_ati_wdata_type(s_ati_dmem_p2_data_type_wr),
        .rst_n(~rst)
        );                   
    pram_consistency      
        #(
        .DATA_MEMORY_SIZE(DATA_MEMORY_SIZE)
        )PRAM_CONSISTENCY(  /* Sequential Parallel-Random-Access-Machine Consistency */ 
        .clk(system_tick_25),
        // Processor 1
        // - write
        .data_bus_wr_p1(s_ati_dmem_p1_data_out),
        .addr_wr_p1(s_ati_dmem_p1_addr_wr[ADDR_WIDTH_DM - 1:0]),
        .data_type_wr_p1(s_ati_dmem_p1_data_type_wr),
        .wr_idle_p1(wr_idle_p1),
        .wr_ins_p1(s_ati_dmem_p1_wr_req),
        .wr_access_p1(s_ati_dmem_p1_wr_available),
        // Processor 2
        // - write
        .data_bus_wr_p2(s_ati_dmem_p2_data_out),
        .addr_wr_p2(s_ati_dmem_p2_addr_wr[ADDR_WIDTH_DM - 1:0]),
        .data_type_wr_p2(s_ati_dmem_p2_data_type_wr),
        .wr_idle_p2(wr_idle_p2),
        .wr_ins_p2(s_ati_dmem_p2_wr_req),
        .wr_access_p2(s_ati_dmem_p2_wr_available),
        // Data memory
        // -- write
        .data_bus_wr_dm(data_bus_wr_dm),
        .addr_wr_dm(addr_wr_dm),
        .data_type_wr_dm(data_type_wr_dm),
        .wr_idle_dm(wr_idle_dm),
        .wr_ins_dm(wr_ins_dm),
        
        .rst_n(~rst)
        );
    
    // Data Memory (8Kb - 1kB)
    ram                 
         #(
        .ADDR_DEPTH(DATA_MEMORY_SIZE),
        .RESERVED_REG_AMOUNT(RESERVED_REG_AMOUNT),
        .RESERVED_REG_DEFAULT(RESERVED_REG_DEFAULT),
        .ADDR_BUS_WIDTH(ADDR_BUS_WIDTH),
        .DUAL_READ_HANDLER(1'b1)
        )DMEM(
        .clk(system_tick_25),
        // -- write
        .data_bus_wr(data_bus_wr_dm),
        .data_type_wr(data_type_wr_dm),    // Write byte (8bit)
        .addr_wr(addr_wr_dm),
        .wr_ins(wr_ins_dm),
        .wr_idle(wr_idle_dm),
        // -- read handler 1
        .data_bus_rd_1(s_ati_dmem_p1_data_in),
        .rd_idle_1(s_ati_dmem_p1_rd_available),
        .addr_rd_1(s_ati_dmem_p1_addr_rd),
        .rd_en_1(s_ati_dmem_p1_rd_req),
        .data_type_rd_1(s_ati_dmem_p1_data_type_rd),
        .invalid_rd_flag_1(),
        // -- read handler 2
        .data_bus_rd_2(s_ati_dmem_p2_data_in),
        .rd_idle_2(s_ati_dmem_p2_rd_available),
        .addr_rd_2(s_ati_dmem_p2_addr_rd),
        .rd_en_2(s_ati_dmem_p2_rd_req),
        .data_type_rd_2(s_ati_dmem_p2_data_type_rd),
        .invalid_rd_flag_2(),
        // -- reserved register
        .reserved_registers(reserved_registers),
        .rst_n(~rst)
        //Debug 
        `ifdef DEBUG
//        ,.registers_wire(data_memory_wire)
        `endif
        ); 
    Registers_management
        #(
        )registers_management(
        .clk(system_tick_25),
        .processor_registers_1(processor_registers_1),
        .processor_registers_2(processor_registers_2),
        .processor_idle_1(processor_idle_1),
        .processor_idle_2(processor_idle_2),
        .boot_renew_register_1(boot_renew_register_1),
        .boot_renew_register_2(boot_renew_register_2),
        .register_num(register_num),
        .new_data_register(new_data_register),
        .registers_renew(registers_renew),
        .ra_register(ra_register),
        .processing_register_table(processing_register_table),
        // New synchronization
        .synchronization_processor_1(synchronization_processor_1),
        .synchronization_processor_2(synchronization_processor_2),
        .synchronized_processors(synchronized_processors),
        .rst_n(~rst)
        ); 
    Multi_processor_manager 
        #(
        .PROGRAM_MEMORY_SIZE(PROGRAM_MEMORY_SIZE)
        ) PROGRAM_PROCESSOR (
        .clk(system_tick_25),
        // Program memory
        .data_bus_rd_pm(data_bus_rd_pm),
        .rd_idle_pm(rd_idle_pm),
        .addr_rd_pm(addr_rd_pm),
//        .rd_ins_pm(rd_ins_pm),
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
        .rd_idle_dm(s_ati_dmem_p1_rd_available & s_ati_dmem_p2_rd_available),
        .wr_idle_dm(wr_idle_dm),
        
        .rst_n(~rst)
        );
    
    
    // Program memory (8Kb)
    ram_pm              
        #(
        .ADDR_DEPTH(PROGRAM_MEMORY_SIZE),
        .RESERVED_REG_AMOUNT(1'b1)
        )IMEM(
        .clk(system_tick_25),
        .data_bus_wr(data_bus_wr_pm),
        .data_type_wr(2'b00),    // Write byte (8bit)
        .addr_wr(addr_wr_pm),
        .wr_ins(wr_ins_pm),
        .wr_idle(wr_idle_pm),
        // Read region
        .data_bus_rd(data_bus_rd_pm),
        .rd_idle(rd_idle_pm),
        .addr_rd(addr_rd_pm),
//        .rd_ins(rd_ins_pm),
        .data_type_rd(2'b10),
        .rst_n(~rst)
        //Debug 
        `ifdef DEBUG
//        ,.registers_wire(program_memory_wire)
        `endif
        );
       
    Atfox_exTensible_Interface
        #(
        .CHANNEL_ID(CHANNEL_ID_GPIO),
        .INTERFACE_DMEM_ENCODE(INTERFACE_DMEM_ENCODE),
        .INTERFACE_PERP_ENCODE(INTERFACE_PERP_ENCODE),
        .INTERFACE_GPIO_ENCODE(INTERFACE_GPIO_ENCODE),
        .INTERFACE_TYPE(INTERFACE_GPIO_ENCODE),
        /* For GPIO */
        .PORT_AMOUNT(GPIO_PORT_AMOUNT),
        .PIN_AMOUNT(GPIO_PIN_AMOUNT)
        ) ATI_GPIO_BUS1 (
        .clk(system_tick_25),
        .m_ati_rdata(),
        .m_ati_wdata(),
        .m_ati_addr(),
        .m_ati_rd_available(),
        .m_ati_wr_available(),
        .m_ati_rd_req(),
        .m_ati_wr_req(),
        .m_ati_data_type(),
        .m_atis_rdata_bus(),
        .m_atis_wdata_bus(),
        .m_atis_addr_bus(),
        .m_atis_rd_available(),
        .m_atis_wr_available(),
        .m_atis_rd_req(),
        .m_atis_wr_req(),
        .m_atis_data_type(),        
        .s_atis_wdata(m_atis_p1_wdata_bus),
        .s_atis_rdata(m_atis_p1_rdata_bus[CHANNEL_ID_GPIO]),
        .s_atis_addr(m_atis_p1_addr_bus),
        .s_atis_rd_available(m_atis_p1_rd_available[CHANNEL_ID_GPIO]),
        .s_atis_wr_available(m_atis_p1_wr_available[CHANNEL_ID_GPIO]),
        .s_atis_rd_req(m_atis_p1_rd_req),
        .s_atis_wr_req(m_atis_p1_wr_req),
        .s_atis_data_type(m_atis_p1_data_type),
        .s_ati_rdata(s_ati_gpio_data_in),
        .s_ati_wdata(s_ati_gpio_data_out),
        .s_ati_rd_req(s_ati_gpio_rd_req),
        .s_ati_wr_req(s_ati_gpio_wr_req),
        .s_ati_rd_available(s_ati_gpio_rd_available),
        .s_ati_wr_available(s_ati_gpio_wr_available),
        .s_ati_raddr(s_ati_gpio_addr_rd),
        .s_ati_waddr(s_ati_gpio_addr_wr),
        .s_ati_rdata_type(),
        .s_ati_wdata_type(),
        .rst_n(~rst)
        );
    generate     
    for(genvar port_index = 0; port_index < GPIO_PORT_AMOUNT; port_index = port_index + 1) begin
    for(genvar pin_index = 0; pin_index < GPIO_PIN_AMOUNT; pin_index = pin_index + 1) begin    
    IOBUF 
        #(
        .DRIVE(12),             // Specify the output drive strength
        .IBUF_LOW_PWR("TRUE"),  // Low Power - "TRUE", High Performance = "FALSE"
        .IOSTANDARD("DEFAULT"), // Specify the I/O standard
        .SLEW("SLOW")           // Specify the output slew rate
        ) IOBUF_inst (
        .O(GPIO_PORT_i[port_index][pin_index]),         // Buffer output
        .IO(GPIO_PORT[port_index][pin_index]),          // Buffer inout port (connect directly to top-level port)
        .I(GPIO_PORT_o[port_index][pin_index]),         // Buffer input
        .T(PORT_CONFIGURATION[port_index][pin_index])   // 3-state enable input, high=input, low=output
        );                                              // End of IOBUF_inst instantiation    
    end
    end    
    endgenerate    
     
    GPIO_module
        #(
        .PORT_AMOUNT(GPIO_PORT_AMOUNT),
        .PIN_AMOUNT(GPIO_PIN_AMOUNT)
        )GPIO(
        .clk(system_tick_25),
        .rdata_PORT(s_ati_gpio_data_in),
        .wdata_PORT(s_ati_gpio_data_out),
        .raddr_PORT(s_ati_gpio_addr_rd),
        .waddr_PORT(s_ati_gpio_addr_wr),
        .rd_req(s_ati_gpio_rd_req),
        .wr_req(s_ati_gpio_wr_req),
        .rd_available(s_ati_gpio_rd_available),
        .wr_available(s_ati_gpio_wr_available),
        .PORT_i(GPIO_PORT_i),
        .PORT_o(GPIO_PORT_o),
        .config_PIN(PORT_CONFIGURATION),
        
        .rst_n(~rst)
        );  
    Interrupt_controller
        #(
        )INT_CONTROLLER(
        .clk(system_tick_25),
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
        .interrupt_enable_2(exti_enable),
        .interrupt_enable_3(timer_interrupt_enable),
        .rst_n(~rst)
        );    
    timer_INT_handler 
        TIM_INT
        (
        .clk(system_tick_25),
        .enable_interrupt(timer_interrupt_enable),
        .interrupt_option(timer_interrupt_option),
        .prescaler_selector(timer_prescaler),
        .timer_limit_value(timer_interrupt_limit_value),
        .interrupt_request(interrupt_request_3),
        .rst_n(~rst)
        );  
    external_INT_handler 
        EXT_INT
        (
        .clk(system_tick_25),
        .int_pin(exti_pin),
        .enable_interrupt(exti_enable),
        .interrupt_sense_control(exti_sense),
        .debounce_option(exti_debounce_option),
        .interrupt_request(interrupt_request_2),
        .rst_n(~rst)
        );                   
  
    // Communication peripherals    
    `ifdef UART_PROT_1        
    Atfox_exTensible_Interface
        #(
        .CHANNEL_ID(CHANNEL_ID_UART1),
        .INTERFACE_DMEM_ENCODE(INTERFACE_DMEM_ENCODE),
        .INTERFACE_PERP_ENCODE(INTERFACE_PERP_ENCODE),
        .INTERFACE_GPIO_ENCODE(INTERFACE_GPIO_ENCODE),
        .INTERFACE_TYPE(INTERFACE_PERP_ENCODE)
        ) ATI_UART1_BUS1 (
        .clk(system_tick_25),
        .m_ati_rdata(),
        .m_ati_wdata(),
        .m_ati_addr(),
        .m_ati_rd_available(),
        .m_ati_wr_available(),
        .m_ati_rd_req(),
        .m_ati_wr_req(),
        .m_ati_data_type(),
        .m_atis_rdata_bus(),
        .m_atis_wdata_bus(),
        .m_atis_addr_bus(),
        .m_atis_rd_available(),
        .m_atis_wr_available(),
        .m_atis_rd_req(),
        .m_atis_wr_req(),
        .m_atis_data_type(), 
        .s_atis_wdata(m_atis_p1_wdata_bus),
        .s_atis_rdata(m_atis_p1_rdata_bus[CHANNEL_ID_UART1]),
        .s_atis_addr(m_atis_p1_addr_bus),
        .s_atis_rd_available(m_atis_p1_rd_available[CHANNEL_ID_UART1]),
        .s_atis_wr_available(m_atis_p1_wr_available[CHANNEL_ID_UART1]),
        .s_atis_rd_req(m_atis_p1_rd_req),
        .s_atis_wr_req(m_atis_p1_wr_req),
        .s_atis_data_type(m_atis_p1_data_type),
        .s_ati_rdata(s_ati_uart1_data_in),
        .s_ati_wdata(s_ati_uart1_data_out),
        .s_ati_rd_req(s_ati_uart1_rd_req),
        .s_ati_wr_req(s_ati_uart1_wr_req),
        .s_ati_rd_available(s_ati_uart1_rd_available),
        .s_ati_wr_available(s_ati_uart1_wr_available),
        .s_ati_raddr(),
        .s_ati_waddr(),
        .s_ati_rdata_type(),
        .s_ati_wdata_type(),
        .rst_n(~rst)
        );          
    uart_peripheral
         #(
        .INTERNAL_CLOCK(INTERNAL_CLOCK),
        .RX_FLAG_TYPE(1'b1),              // Use internal FIFO
        .FIFO_DEPTH(FIFO_BUFFER_SIZE)
        )UART_PERIPHERAL_1(
        .clk(system_tick_25),
        .TX(TX_1),
        .RX(RX_1),
        // TX
        .data_in(s_ati_uart1_data_out),
        .TX_config_register(TX_config_register_1),
        .TX_use(s_ati_uart1_wr_req),
        .TX_available(s_ati_uart1_wr_available),
        .TX_flag(),
        .TX_complete(),
//        .TX_enable(TX_enable_1),
        // RX
        .data_out(s_ati_uart1_data_in),
        .RX_config_register(RX_config_register_1),
        .RX_use(RX_use_1),
        .RX_flag(RX_flag_1),
        .RX_available(),
        .rst_n(~rst)
        );
    `endif
    `ifdef UART_PROT_2  
    Atfox_exTensible_Interface
        #(
        .CHANNEL_ID(CHANNEL_ID_UART2),
        .INTERFACE_DMEM_ENCODE(INTERFACE_DMEM_ENCODE),
        .INTERFACE_PERP_ENCODE(INTERFACE_PERP_ENCODE),
        .INTERFACE_GPIO_ENCODE(INTERFACE_GPIO_ENCODE),
        .INTERFACE_TYPE(INTERFACE_PERP_ENCODE)
        ) ATI_UART2_BUS2 (
        .clk(system_tick_25),
        .m_ati_rdata(),
        .m_ati_wdata(),
        .m_ati_addr(),
        .m_ati_rd_available(),
        .m_ati_wr_available(),
        .m_ati_rd_req(),
        .m_ati_wr_req(),
        .m_ati_data_type(),
        .m_atis_rdata_bus(),
        .m_atis_wdata_bus(),
        .m_atis_addr_bus(),
        .m_atis_rd_available(),
        .m_atis_wr_available(),
        .m_atis_rd_req(),
        .m_atis_wr_req(),
        .m_atis_data_type(), 
        .s_atis_wdata(m_atis_p2_wdata_bus),
        .s_atis_rdata(m_atis_p2_rdata_bus[CHANNEL_ID_UART2]),
        .s_atis_addr(m_atis_p2_addr_bus),
        .s_atis_rd_available(m_atis_p2_rd_available[CHANNEL_ID_UART2]),
        .s_atis_wr_available(m_atis_p2_wr_available[CHANNEL_ID_UART2]),
        .s_atis_rd_req(m_atis_p2_rd_req),
        .s_atis_wr_req(m_atis_p2_wr_req),
        .s_atis_data_type(m_atis_p2_data_type),
        .s_ati_rdata(s_ati_uart2_data_in),
        .s_ati_wdata(s_ati_uart2_data_out),
        .s_ati_wr_req(s_ati_uart2_wr_req),
        .s_ati_rd_req(s_ati_uart2_rd_req),
        .s_ati_rd_available(s_ati_uart2_rd_available),
        .s_ati_wr_available(s_ati_uart2_wr_available),
        .s_ati_raddr(),
        .s_ati_waddr(),
        .s_ati_rdata_type(),
        .s_ati_wdata_type(),
        .rst_n(~rst)
        );  
    uart_peripheral
        #(
        .INTERNAL_CLOCK(INTERNAL_CLOCK),
        .RX_FLAG_TYPE(1'b1) /// Internal FIFO
        )             
        UART_PERIPHERAL_2
        (
        .clk(system_tick_25),
        // TX 
        .data_in(s_ati_uart2_data_out),
        .TX_use(s_ati_uart2_wr_req),
        .TX_available(s_ati_uart2_wr_available),
        .TX_complete(TX_complete_2),
        .TX_config_register(TX_config_register_2),
//        .TX_enable(TX_enable_2),
        .TX_flag(),
        .TX(TX_2),
        // RX
        .data_out(s_ati_uart2_data_in),
        .RX_use(s_ati_uart2_rd_req),
        .RX_flag(s_ati_uart2_rd_available),
        .RX_available(RX_available_2),
        .RX_config_register(RX_config_register_2),
//        .RX_enable(RX_enable_2),
        .RX(RX_2),
        
        .rst_n(~rst)
        );      
    `endif
    `ifdef SPI_PROT                    
    com_spi             #(
                        )spi_peripheral(
                        );                                                                                     
    `endif
    
    /* Special wires are used for Stored-program */
    assign RUNNING_PROGRAM          = (main_state == RUNNING_PROGRAM_STATE);
    assign RX_use_1                 = (RUNNING_PROGRAM) ? s_ati_uart1_rd_req : RX_use_1_processor;    
    assign s_ati_uart1_rd_available = (RUNNING_PROGRAM) ? RX_flag_1 : 1'b0;
    `ifdef DEBUG 
//    assign clk_out = clk;
    `endif 
endmodule

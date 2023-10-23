`timescale 1ns / 1ps
`define UART_PROT_1
`define UART_PROT_2
//`define SPI_PROT
//`define I2C_PROT
// Test case
//`define NORMAL_TESTCASE
//`define GPIO_TESTCASE
//`define PERIPHERAL_TESTCASE
//`define INTERRUPT_HANDLER_TESTCASE
`define PARALLEL_TESTCASE

module Dual_core_mcu_tb;
    parameter DATA_WIDTH = 8;
    parameter GPIO_NUM = 13;
    parameter CLOCK_DIVIDER_UNIQUE_1 = 5;
    parameter PROGRAM_MEMORY_SIZE = 1024;
    parameter DATA_MEMORY_SIZE = 1024;
    parameter FINISH_PROGRAM_OPCODE  = 7'b0001011;
    parameter FINISH_RECEIVE_TIMER = 10000;
    reg clk;
    
    `ifdef UART_PROT_1                  // For load bitstream file
    wire RX_1;
    wire TX_1;
    `endif
    
    `ifdef UART_PROT_2
    wire RX_2;
    wire TX_2;
    `endif
    
    `ifdef SPI_PROT
    wire MOSI;        
    wire MISO; 
    wire SCK;
    wire SS;           
    `endif
    
    `ifdef I2C_PROT
    wire SDA;
    wire SDL;
    `endif
    
    // GPIO
    wire [GPIO_NUM - 1:0]    IO_PORT;
    reg  [GPIO_NUM - 1:0]    IO_driver;     
    reg                      external_int_pin;
    for(genvar i = 0; i < 12; i = i + 1) begin      
    assign IO_PORT[i] = (IO_driver[i]) ? 1'b1 : 1'b0;
    end
    assign IO_PORT[12] = external_int_pin;
    // Debug
    wire    [DATA_WIDTH - 1:0]      program_memory_wire [0: PROGRAM_MEMORY_SIZE - 1];
    wire    [DATA_WIDTH - 1:0]      data_memory_wire    [0: DATA_MEMORY_SIZE - 1];
    
    // Reset negedge
    reg rst_n;
    
    
    wire    TX_ex;
    reg     TX_use_ex;
    reg     [7:0] data_bus_in_tx_ex;
    wire    [7:0] TX_config_register_ex = 8'b10001111;
//    wire    [7:0] RX_config_register_1  = 8'b10001111;
// external UART_2
    reg  [DATA_WIDTH - 1:0] data_bus_in_uart_ex_2;
    reg                     TX_use_ex_2;
    wire                    TX_flag_ex_2;
    wire                    TX_complete_ex_2;
    wire                    TX_ex_2;                  
                    
    wire [DATA_WIDTH - 1:0] data_bus_out_uart_ex_2;                
    wire                    RX_use_ex_2;                
    wire                    RX_flag_ex_2;                
    wire                    RX_ex_2;  
    
    assign RX_1 = TX_ex;
    assign RX_ex_2 = TX_2;
    assign RX_2 = TX_ex_2; 
    
    Dual_core_mcu       #(
                        .CLOCK_DIVIDER_UNIQUE_1(CLOCK_DIVIDER_UNIQUE_1),
                        .FINISH_RECEIVE_TIMER(FINISH_RECEIVE_TIMER)
                        ) dual_core_mcu (
                        .clk(clk),
                        .RX_1(RX_1),
                        .TX_1(TX_1),
                        .RX_2(RX_2),
                        .TX_2(TX_2),
                        .IO_PORT(IO_PORT),
                        .rst_n(rst_n)
                        ,.program_memory_wire(program_memory_wire)
                        ,.data_memory_wire(data_memory_wire)
                        );
    // External UART_1                    
    com_uart            #(
                        .SLEEP_MODE(0),
                        .FIFO_DEPTH(13'd4096),
                        .CLOCK_DIVIDER_UNIQUE_1(CLOCK_DIVIDER_UNIQUE_1)
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
    // External UART_2
    com_uart            #(
                        .SLEEP_MODE(0), 
                        .RX_FLAG_CONFIG(0), /// External FIFO
                        .CLOCK_DIVIDER_UNIQUE_1(CLOCK_DIVIDER_UNIQUE_1)
                        )             
                        uart_ex_2
                        (
                        .clk(clk),
                        // TX 
                        .data_bus_in(data_bus_in_uart_ex_2),
                        .TX_use(TX_use_ex_2),
                        .TX_flag(TX_flag_ex_2),
                        .TX_complete(TX_complete_ex_2),
                        .TX_config_register(TX_config_register_ex),
                        .TX(TX_ex_2),
                        // RX
                        .data_bus_out(data_bus_out_uart_ex_2),
                        .RX_use(RX_use_ex_2),
                        .RX_flag(RX_flag_ex_2),
                        .RX_config_register(TX_config_register_ex),
                        .RX(RX_ex_2),
                        
                        .rst_n(rst_n)
                        ); 
    
    initial begin
        clk <= 0;
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 0;
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 0;
        IO_driver <= 0;
        external_int_pin <= 1;
        rst_n <= 1;
        #1 rst_n <= 0;
        #9 rst_n <= 1;
    end                    
    initial begin
    forever #1 clk <= ~clk;
    end
    
    
    parameter ADD_INS_17    = 17'b00000000000110011;// ADD:     <5-rd><5-rs1><5-rs2>
    parameter ADDI_INS_10   = 10'b0000010011;       // ADDI:    <5-rd><5rs1><12-imm>
    parameter SUB_INS_17    = 17'b10000000000110011;// SUB:     <5-rd><5-rs1><5-rs2>
    parameter SLL_INS_17    = 17'b00000000010110011;// SLL:     <5-rd><5-rs1><5-rs2>
    parameter SRL_INS_17    = 17'b00000001010110011;// SRL:     <5-rd><5-rs1><5-rs2>
    parameter MUL_INS_17    = 17'b00000010000110011;// MUL:     <5-rd><5-rs1><5-rs2>
    // Load                                                     dest  base  offset
    parameter LB_INS_10     = 10'b0000000011;       // LB:      <5-rd><5rs1><12-imm>
    parameter LW_INS_10     = 10'b0100000011;       // LW:      <5-rd><5rs1><12-imm>
    parameter LD_INS_10     = 10'b0110000011;       // LD:      <5-rd><5rs1><12-imm>
    // Store                                                    offset  base   src   offset
    parameter SB_INS_10     = 10'b0000100011;       // SB:      <5-immh><5-rs1><5rs2><7-imml>
    parameter SW_INS_10     = 10'b0100100011;       // SW:      <5-immh><5-rs1><5rs2><7-imml>
    parameter SD_INS_10     = 10'b0110100011;       // SD:      <5-immh><5-rs1><5rs2><7-imml>
    
    parameter J_INS_7       = 7'b1100111;           // J:       <25-imm>
    parameter JAL_INS_7     = 7'b1101111;           // J:       <25-imm>
    parameter JALR_INS_7    = 7'b1101011;           // J:       <25-imm>
    
    parameter FENCE_INS_10  = 10'b0100101111;       // FENCE:   Don't care
    
    parameter RETI_INS_10   = 10'b0111110111;       // RETI:   Don't care
    
    parameter UART_TX_INS_12= 12'b010001000001;      // UART_TX: <5-rs3><5-rs1><5-rs2><5-imm>
    parameter UART_RX_INS_12= 12'b000001000001;      // UART_TX: <5-rd1><5-rd2><5-rd3><5-imm>
    
    parameter GPIO_READ_INS_10  = 10'b0001110101;      // UART_TX: <5-rs3><5-rs1><5-rs2><6-imm>
    parameter GPIO_WRITE_INS_10 = 10'b0011110101;      // UART_TX: <5-rs3><5-rs1><5-rs2><6-imm>
    
    parameter LUI_INS_7     = 7'b0110111;
    
        int i;
    reg [31:0] instruction;
    initial begin
        #11;
        `ifdef NORMAL_TESTCASE
            // Skip interrupt-program
            for(i = 0; i < 48; i = i + 1) begin
                instruction <= {5'd10,5'd0,5'd10,ADDI_INS_10};
                begin 
                    #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
                end
            end
            
        
            // PC = 0xC0
            instruction <= {5'd09,5'd0,12'd09,ADDI_INS_10};      // x9 = x0 + 9     = 9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd08,5'd0,12'd08,ADDI_INS_10};      // x8 = x0 + 8     = 8
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC8
            instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};      // x7 = x8 * x9     = 72
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
                
            // PC = 0xCC
            instruction <= {5'd10,5'd09,5'd08,ADD_INS_17};      // x10 = x9 + x8    = 17
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD0
            instruction <= {25'd08,JAL_INS_7};      //          // Jump to 0xD8     &   x1 <= PC + 4 = 0xD4
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD4
            instruction <= {25'd20,J_INS_7};                    // Jump to 0xD4 + 25'd24 = 0xE8
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
                // PC = 0xD8
            instruction <= {5'd11,5'd09,5'd08,SUB_INS_17};      // x11 = x9 - x8    = 1
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xDC
            instruction <= {5'd07,5'd07,5'd11,SUB_INS_17};      // x7 = x7 - x11    = 71
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xE0
            instruction <= {25'b0,JALR_INS_7};                  // Jump to 0(x1)    = 0xD4
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xE4
            instruction <= {32'b0};                             // If (PC is here), it's failed this testcase
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xE8
            instruction <= {5'd07,5'd07,5'd11,SUB_INS_17};      // x7 = x7 - x11    = 71 - 1 = 70
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xEC
            instruction <= {5'd00,5'd00,5'd10,7'd16,SW_INS_10}; // Store x10(17) data to 0x10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF0
            instruction <= {5'd00,5'd00,5'd07,7'd12,SW_INS_10}; // Store x7(70) data to 0x0C
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF4
            instruction <= {5'd00,5'd00,12'd00,FENCE_INS_10};   // Fence memory access
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF8
            instruction <= {5'd15,5'd00,12'd16,LB_INS_10};      // Load 16(x0) to x15 = 17
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xFC
            instruction <= {5'd16,5'd00,12'd12,LB_INS_10};      // Load 12(x0) to x16 = 70
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x100
            instruction <= {5'd09,5'd00,-12'd05,ADDI_INS_10};        // x9 = x0 - 5     = -5
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x104
            instruction <= {5'b111,5'd00,5'd09,7'b1101000,SW_INS_10}; // Store x9(-5) data to 0d1000
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x108
            instruction <= {5'd20,5'd00,12'b1111101000,LB_INS_10};      // Load 1000(x0) to x20 = -5
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x10C
            instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};      // x7 = x8 * x9     = -40
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x110
            instruction <= {5'd10,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x114
            instruction <= {5'd11,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x11
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x118
            instruction <= {5'd12,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x12
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x11C
            instruction <= {5'd13,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x12
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x120
            instruction <= {5'd14,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x12
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x124
            instruction <= {5'd15,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x12
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            
            // PC = 0x128
            instruction <= {25'd00,J_INS_7};                   // While(1) {};
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        
        `endif
        `ifdef GPIO_TESTCASE
        // ISR 1 ////////////////////////////////////////////////////////////////////////////////////////
        // PC = 0x00 - 1
            instruction <= {5'd00,5'd00,5'd09,7'd50,SB_INS_10};      // Restore x9 to 0x50 (Store x9 to 0x50)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x04 - 2
            instruction <= {5'd09,5'd00,12'd60,LB_INS_10};           // Load global data (at 0x60) to x9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x08 - 3
            instruction <= {5'd09,5'd09,12'd01,ADDI_INS_10};        // x9 = x9 + 1     
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x0C - 4
            instruction <= {5'd00,5'd00,5'd09,7'd60,SB_INS_10};     // Store x9 to 0x60
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x10 - 5
            instruction <= {5'd09,5'd00,12'd50,LB_INS_10};         // Recovery previous x9 from 0x50
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x14 - 6
            instruction <= {5'd00,5'd00,12'd00,RETI_INS_10};      // RETI
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        for(int i = 0; i < 10; i = i + 1) begin    
        // PC = 0x18 - 7
            instruction <= {5'd00,5'd00,12'd00,10'd00};      
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        end
        /////////////////////////////////////////////////////////////////////////////////////////////////   
        
        // ISR 2 //////////////////////////////////////////////////////////////////////////////////////// 
        // PC = 0x40 - 1
            instruction <= {5'd00,5'd00,5'd09,7'd100,SB_INS_10};      // Restore x9 to 0x50 (Store x9 to 0x100)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x44 - 2
            instruction <= {5'd09,5'd00,12'd120,LB_INS_10};           // Load global data (at 0x120) to x9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x48 - 3
            instruction <= {5'd09,5'd09,12'd02,ADDI_INS_10};        // x9 = x9 + 2     
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x4C - 4
            instruction <= {5'd00,5'd00,5'd09,7'd120,SB_INS_10};     // Store x9 to 0x120
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x50 - 5
            instruction <= {5'd09,5'd00,12'd100,LB_INS_10};         // Recovery previous x9 from 0x100
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x54 - 6
            instruction <= {5'd00,5'd00,12'd00,RETI_INS_10};      // RETI
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        for(int i = 0; i < 10; i = i + 1) begin    
        // PC = 0x58 - 7
            instruction <= {5'd00,5'd00,12'd00,10'd00};      
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        end
        ///////////////////////////////////////////////////////////////////////////////////////////////// 
        
        // ISR 3 //////////////////////////////////////////////////////////////////////////////////////// 
        // PC = 0x80 - 1
            instruction <= {5'd00,5'd00,5'd09,7'd30,SB_INS_10};      // Restore x9 to 0x50 (Store x9 to 0x30)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x84 - 2
            instruction <= {5'd09,5'd00,12'd40,LB_INS_10};           // Load global data (at 0x40) to x9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x88 - 3
            instruction <= {5'd09,5'd09,12'd03,ADDI_INS_10};        // x9 = x9 + 3    
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x8C - 4
            instruction <= {5'd00,5'd00,5'd09,7'd40,SB_INS_10};     // Store x9 to 0x40
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x90 - 5
            instruction <= {5'd09,5'd00,12'd30,LB_INS_10};         // Recovery previous x9 from 0x30
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x94 - 6
            instruction <= {5'd00,5'd00,12'd00,RETI_INS_10};      // RETI
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        for(int i = 0; i < 10; i = i + 1) begin    
        // PC = 0x98 - 7
            instruction <= {5'd00,5'd00,12'd00,10'd00};      
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        end
        ///////////////////////////////////////////////////////////////////////////////////////////////// 
        
        // MAIN ///////////////////////////////////////////////////////////////////////////////////////// 
        // PC = 0xC0
            instruction <= {5'd05,5'd00,12'd00,ADDI_INS_10};            // x5 = x0 + 0     = 0 (PORT A)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xC4
            instruction <= {5'd06,5'd00,12'd02,ADDI_INS_10};            // x6 = 2 (Pin 2)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xC8
            instruction <= {5'd07,5'd05,5'd06,7'd00,GPIO_READ_INS_10};     // READ GPIO from <PORT_A> <Pin 2>
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xCC
            instruction <= {5'd08,5'd08,5'd07,ADD_INS_17};     // x8 = x8 + x7 
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD4
            instruction <= {5'd09,5'd09,12'd1,ADDI_INS_10};            // x9 = x9 + 1     = 
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD0
            instruction <= {-25'd12,J_INS_7};                        // Jump to C8;
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD4
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD8
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xDC
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xE0
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xE4
            instruction <= {5'd07,5'd00,12'b10010000,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xE8
            instruction <= {5'd00,5'd00,5'd07,7'h0C,SB_INS_10};     // Store x7 to 0x0C (configure external interrupt) 
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xEC
            instruction <= {25'd00,J_INS_7};                        // While(1);
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        ///////////////////////////////////////////////////////////////////////////////////////////////// 
    `endif
    `ifdef PERIPHERAL_TESTCASE
    
        // Skip interrupt-program
            for(i = 0; i < 48; i = i + 1) begin
                instruction <= {5'd10,5'd0,5'd10,ADDI_INS_10};
                begin 
                    #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
                end
            end
            
            // PC = 0xC0
            instruction <= {5'd10,5'd0,12'b00001100,ADDI_INS_10};      // COM_PERIPHERAL = TX_ENABLE << 1 & RX_ENABLE << 1;
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x0C 
            instruction <= {5'd00,5'd00,5'd10,7'h06,SB_INS_10};     // Load COM_PERIPHERAL 
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC0
            instruction <= {5'd09,5'd0,12'd9,ADDI_INS_10};      // x9 = x0 + 9     = 9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd08,5'd0,12'd2,ADDI_INS_10};      // x8 = x0 + 2     = 2
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC8
            instruction <= {5'd07,5'd09,5'd08,ADD_INS_17};      // x7 = x8 + x9     = 11
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xCC
            instruction <= {5'd08,5'd07,5'd09,5'b00,UART_TX_INS_12};      // Send 9bytes of x7 -> x8
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD0
            instruction <= {5'd15,5'd16,5'd17,5'b00,UART_RX_INS_12};      // Receive data (waiting until available)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD4
            instruction <= {-25'd04,J_INS_7};                    // Jump to 0xD4 + 25'd24 = 0xE8
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
    
    `endif
    
    `ifdef INTERRUPT_HANDLER_TESTCASE
        // ISR 1 ////////////////////////////////////////////////////////////////////////////////////////
        // PC = 0x00 - 1
            instruction <= {5'd00,5'd00,5'd09,7'd50,SB_INS_10};      // Restore x9 to 0x50 (Store x9 to 0x50)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x04 - 2
            instruction <= {5'd09,5'd00,12'd60,LB_INS_10};           // Load global data (at 0x60) to x9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x08 - 3
            instruction <= {5'd09,5'd09,12'd01,ADDI_INS_10};        // x9 = x9 + 1     
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x0C - 4
            instruction <= {5'd00,5'd00,5'd09,7'd60,SB_INS_10};     // Store x9 to 0x60
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x10 - 5
            instruction <= {5'd09,5'd00,12'd50,LB_INS_10};         // Recovery previous x9 from 0x50
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x14 - 6
            instruction <= {5'd00,5'd00,12'd00,RETI_INS_10};      // RETI
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        for(int i = 0; i < 10; i = i + 1) begin    
        // PC = 0x18 - 7
            instruction <= {5'd00,5'd00,12'd00,10'd00};      
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        end
        /////////////////////////////////////////////////////////////////////////////////////////////////   
        
        // ISR 2 //////////////////////////////////////////////////////////////////////////////////////// 
        // PC = 0x40 - 1
            instruction <= {5'd00,5'd00,5'd09,7'd100,SB_INS_10};      // Restore x9 to 0x50 (Store x9 to 0x100)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x44 - 2
            instruction <= {5'd09,5'd00,12'd120,LB_INS_10};           // Load global data (at 0x120) to x9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x48 - 3
            instruction <= {5'd09,5'd09,12'd02,ADDI_INS_10};        // x9 = x9 + 2     
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x4C - 4
            instruction <= {5'd00,5'd00,5'd09,7'd120,SB_INS_10};     // Store x9 to 0x120
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x50 - 5
            instruction <= {5'd09,5'd00,12'd100,LB_INS_10};         // Recovery previous x9 from 0x100
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x54 - 6
            instruction <= {5'd00,5'd00,12'd00,RETI_INS_10};      // RETI
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        for(int i = 0; i < 10; i = i + 1) begin    
        // PC = 0x58 - 7
            instruction <= {5'd00,5'd00,12'd00,10'd00};      
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        end
        ///////////////////////////////////////////////////////////////////////////////////////////////// 
        
        // ISR 3 //////////////////////////////////////////////////////////////////////////////////////// 
        // PC = 0x80 - 1
            instruction <= {5'd00,5'd00,5'd09,7'd30,SB_INS_10};      // Restore x9 to 0x50 (Store x9 to 0x30)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x84 - 2
            instruction <= {5'd09,5'd00,12'd40,LB_INS_10};           // Load global data (at 0x40) to x9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x88 - 3
            instruction <= {5'd09,5'd09,12'd03,ADDI_INS_10};        // x9 = x9 + 3    
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x8C - 4
            instruction <= {5'd00,5'd00,5'd09,7'd40,SW_INS_10};     // Store x9 to 0x40
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x90 - 5
            instruction <= {5'd09,5'd00,12'd30,LB_INS_10};         // Recovery previous x9 from 0x30
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x94 - 6
            instruction <= {5'd00,5'd00,12'd00,RETI_INS_10};      // RETI
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        for(int i = 0; i < 10; i = i + 1) begin    
        // PC = 0x98 - 7
            instruction <= {5'd00,5'd00,12'd00,10'd00};      
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        end
        ///////////////////////////////////////////////////////////////////////////////////////////////// 
        
        // MAIN ///////////////////////////////////////////////////////////////////////////////////////// 
        // PC = 0xC0
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 5     = 5
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xC4
            instruction <= {5'd08,5'd00,12'b10100000,ADDI_INS_10};      // x8 = 8'b11000010 (store configuration data)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xC8
            instruction <= {5'd00,5'd00,5'd08,7'h0D,SB_INS_10};     // Store x8 to 0x0D (configure EXTERNAL_INTERRUPT) 8'h0D
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xCC
            instruction <= {5'd08,5'd00,12'b11000000,ADDI_INS_10};      // x8 = 8'b11000010 (store configuration data)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD0
            instruction <= {5'd00,5'd00,5'd07,7'h10,SB_INS_10};     // Store x7 to 0x10 (configure timer_limit_low) 8'd09
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD4
            instruction <= {5'd00,5'd00,5'd00,7'h0F,SB_INS_10};     // Store x0 to 0x0F (configure timer_limit_high) 8'd00
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD8
            instruction <= {5'd00,5'd00,5'd08,7'h0E,SB_INS_10};     // Store x8 to 0x0E (configure timer0) 8'b11000001
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xDC
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xE0
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xDC
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xE0
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xE4
            instruction <= {5'd07,5'd00,12'b10010000,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xE8
            instruction <= {5'd00,5'd00,5'd07,7'h0C,SB_INS_10};     // Store x7 to 0x0C (configure external interrupt) 
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xEC
            instruction <= {25'd00,J_INS_7};                        // While(1);
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        ///////////////////////////////////////////////////////////////////////////////////////////////// 
    `endif
    `ifdef PARALLEL_TESTCASE
            // Skip interrupt-program
            for(i = 0; i < 48; i = i + 1) begin
                instruction <= {5'd10,5'd0,5'd10,ADDI_INS_10};
                begin 
                    #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
                end
            end
            
            // PC = 0xC0
            instruction <= {5'd09,5'd0,12'd4095,ADDI_INS_10};      // x9 = x0 + -1     = -1
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd08,5'd0,12'd4095,ADDI_INS_10};      // x8 = x0 + -1     = -1
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC8    (Stress processor_1)
            instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};      // x7 = x8 * x9     = 1
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xCC
            instruction <= {5'd10,5'd09,5'd08,ADD_INS_17};      // x10 = x9 + x8    = -1 + -1
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
             
            // PC = 0xD0
            instruction <= {5'd11,5'd08,12'd05,ADDI_INS_10};      // x11 = x8 + 8    = -1 + 5 = 4100
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD4
            instruction <= {5'd12,5'd08,5'd09,ADD_INS_17};      // x12 = x8 + x9    = -1 + -1 = 8190
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD8
            instruction <= {25'd00,J_INS_7};                    // While(1);
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        `endif
    
    // Finish program
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= FINISH_PROGRAM_OPCODE;
        #1 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
    end
    `ifdef GPIO_TESTCASE
    initial begin
//        GPIO_PORT_A[2] <= 1;
//        IO_PORT[2] <= 1;
        IO_driver[2] <= 1;
        #59950;
        
//        interrupt_request_3 <= 0;
//        #1 interrupt_request_3 <= 1;
//        #2 interrupt_request_3 <= 0;
        
        
//        GPIO_PORT_A[2] <= 0;
        IO_driver[2] <= 0;
        
        
        #5000;
        
//        GPIO_PORT_A[2] <= 1;
        IO_driver[2] <= 1;
    end
    `endif
    
    `ifdef PERIPHERAL_TESTCASE
    initial begin
        #70000;
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h27;
        #1 TX_use_ex_2 <= 1;
        #3 TX_use_ex_2 <= 0;
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h02;
        #1 TX_use_ex_2 <= 1;
        #3 TX_use_ex_2 <= 0;
        
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h21;
        #1 TX_use_ex_2 <= 1;
        #3 TX_use_ex_2 <= 0;
        
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h10;
        #1 TX_use_ex_2 <= 1;
        #3 TX_use_ex_2 <= 0;
        
        #4760;
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h07;
        #1 TX_use_ex_2 <= 1;
        #3 TX_use_ex_2 <= 0;
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h07;
        #1 TX_use_ex_2 <= 1;
        #3 TX_use_ex_2 <= 0;
        
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h20;
        #1 TX_use_ex_2 <= 1;
        #3 TX_use_ex_2 <= 0;
        
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h11;
        #1 TX_use_ex_2 <= 1;
        #3 TX_use_ex_2 <= 0;
    end
    `endif
    
    `ifdef INTERRUPT_HANDLER_TESTCASE
    initial begin
        #59950;
        
//        interrupt_request_3 <= 0;
//        #1 interrupt_request_3 <= 1;
//        #2 interrupt_request_3 <= 0;
        
        #16;
        
        external_int_pin <= 0;
        #2 external_int_pin <= 1;
        #10 external_int_pin <= 0;
        
        #100;
        
        external_int_pin <= 0;
        #2 external_int_pin <= 1;
        #10 external_int_pin <= 0;
        
        #400;
        
        external_int_pin <= 0;
        #2 external_int_pin <= 1;
        #10 external_int_pin <= 0;
        
        #200;
        
        external_int_pin <= 0;
        #2 external_int_pin <= 1;
        #10 external_int_pin <= 0;
    end
    `endif 
    
    initial begin   : STOP_BLOCK
        `ifdef PERIPHERAL_TESTCASE
        #(70000 * 2) $stop;
        #2750200 $stop;
        `else
        #66200 $stop;
        #02200 $stop;
        #02200 $stop;
        #02200 $stop;
        #02200 $stop;
        #02200 $stop;
        #02200 $stop;
        `endif
    end 
endmodule

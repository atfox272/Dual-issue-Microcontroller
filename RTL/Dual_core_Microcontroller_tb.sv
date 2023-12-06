`timescale 1ns / 1ps
// System testcase
//`define REAL_TIME_DEBUG
`define FINISH_PROGRAM_OPCODE

`define UART_PROT_1
`define UART_PROT_2
//`define SPI_PROT
//`define I2C_PROT

// Instruction testcase 
//`define NORMAL_TESTCASE
//`define GPIO_TESTCASE
//`define PERIPHERAL_TESTCASE
`define INTERRUPT_HANDLER_TESTCASE
//`define PARALLEL_TESTCASE

module Dual_core_mcu_tb;
    parameter DATA_WIDTH = 8;
    parameter GPIO_PORT_AMOUNT  = 2;
    parameter GPIO_PIN_AMOUNT   = 8;
    `ifdef REAL_TIME_DEBUG
    parameter INTERNAL_CLOCK = 125000000;
    `else 
    parameter INTERNAL_CLOCK = 9600 * 10;
    `endif
    parameter CLOCK_DIVIDER_UNIQUE_1 = 5;
    parameter PROGRAM_MEMORY_SIZE = 1024;
    parameter DATA_MEMORY_SIZE = 256;
    parameter FINISH_PROGRAM_OPCODE  = 7'b0001011;
    parameter FINISH_PROGRAM_TIMER = 10000;
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
    wire [GPIO_PIN_AMOUNT - 1:0]    GPIO_PORT  [0:GPIO_PORT_AMOUNT - 1];
    reg  [GPIO_PIN_AMOUNT - 1:0]    GPIO_driver [0:GPIO_PORT_AMOUNT - 1];
    reg  [GPIO_PIN_AMOUNT - 1:0]    GPIO_ext    [0:GPIO_PORT_AMOUNT - 1];
    reg                      external_int_pin;
    for(genvar port_index = 0; port_index < GPIO_PORT_AMOUNT; port_index = port_index + 1) begin
        for(genvar pin_index = 0; pin_index < GPIO_PIN_AMOUNT; pin_index = pin_index + 1) begin
            assign GPIO_PORT[port_index][pin_index] = (~GPIO_driver[port_index][pin_index]) ? GPIO_ext[port_index][pin_index] : 1'bz;
        end
    end 
    // Debug
    wire    [DATA_WIDTH - 1:0]      program_memory_wire [0: PROGRAM_MEMORY_SIZE - 1];
    wire    [DATA_WIDTH - 1:0]      data_memory_wire    [0: DATA_MEMORY_SIZE - 1];
    
    // Reset negedge
    reg rst_n;
    
    
    wire    TX_ex;
    reg     TX_use_ex;
    reg     [7:0] data_bus_in_tx_ex;
    wire    [7:0] TX_config_register_ex = 8'b00100011;
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
    
    Dual_core_mcu       
        #(
        .INTERNAL_CLOCK(INTERNAL_CLOCK)
        ,.FINISH_RECEIVE_TIMER(FINISH_RECEIVE_TIMER)
        ,.FINISH_PROGRAM_TIMER(FINISH_PROGRAM_TIMER)
        ) dual_core_mcu (
        .clk(clk),
        .RX_1(RX_1),
        .TX_1(TX_1),
        .RX_2(RX_2),
        .TX_2(TX_2),
        .GPIO_PORT(GPIO_PORT),
        .rst(~rst_n)
//        ,.program_memory_wire(program_memory_wire)
//        ,.data_memory_wire(data_memory_wire)
        );
    // External UART_1                    
    uart_peripheral
        #(
        .SLEEP_MODE(0),
        .FIFO_DEPTH(65536),
        .INTERNAL_CLOCK(INTERNAL_CLOCK)
        )
        uart_ex
        (
        .clk(clk),
        .TX(TX_ex),
        .TX_use(TX_use_ex),
        .data_in(data_bus_in_tx_ex),
        .TX_config_register(TX_config_register_ex),
        .rst_n(rst_n)
        );
    // External UART_2
    uart_peripheral
        #(
        .SLEEP_MODE(0), 
        .RX_FLAG_CONFIG(0), /// External FIFO
        .INTERNAL_CLOCK(INTERNAL_CLOCK)
        )             
        uart_ex_2
        (
        .clk(clk),
        // TX 
        .data_in(data_bus_in_uart_ex_2),
        .TX_use(TX_use_ex_2),
        .TX_flag(TX_flag_ex_2),
        .TX_complete(TX_complete_ex_2),
        .TX_config_register(TX_config_register_ex),
        .TX(TX_ex_2),
        // RX
        .data_out(data_bus_out_uart_ex_2),
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
//        IO_driver <= 0;
        for(int port_index = 0; port_index < GPIO_PORT_AMOUNT; port_index = port_index + 1) begin
        for(int pin_index = 0; pin_index < GPIO_PIN_AMOUNT; pin_index = pin_index + 1) begin
        GPIO_driver[port_index][pin_index] <= 1;
        end
    end 
        external_int_pin <= 1;
        rst_n <= 1;
        #1 rst_n <= 0;
        #9 rst_n <= 1;
    end                    
    initial begin
    forever #1 clk <= ~clk;
    end
    
    reg [12- 1:0] quick_seperate_imm12;
    wire[7 - 1:0] imm12lo;
    wire[5 - 1:0] imm12hi;
    assign imm12hi = quick_seperate_imm12[11:7];
    assign imm12lo = quick_seperate_imm12[6:0];
    
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
    // Unocndition jump
    parameter J_INS_7       = 7'b1100111;           // J:       <25-imm>
    parameter JAL_INS_7     = 7'b1101111;           // J:       <25-imm>
    parameter JALR_INS_7    = 7'b1101011;           // J:       <25-imm>
    // Condition jumo
    parameter BEQ_INS_10    = 10'b0001100011;       // BEQ:     <imm-h><rs1><rs2><imm-l>
    parameter BNE_INS_10    = 10'b0011100011;       // BNE:     <imm-h><rs1><rs2><imm-l>
    parameter BLT_INS_10    = 10'b1001100011;       // BLT:     <imm-h><rs1><rs2><imm-l>
    parameter BGE_INS_10    = 10'b1011100011;       // BGE:     <imm-h><rs1><rs2><imm-l>
    
    parameter FENCE_INS_10  = 10'b0100101111;       // FENCE:   Don't care
    
    parameter RETI_INS_10   = 10'b0111110111;       // RETI:   Don't care
    parameter DEBUG_INS_10  = 10'b1011110111;       // RETI:   Don't care
    
    parameter UART_TX_INS_17= 17'b10000000001000001;      // UART_TX: <5-rs3><5-rs1><5-rs2><5-imm>
    parameter UART_RX_INS_17= 17'b00000000001000001;      // UART_TX: <5-rd1>
    parameter LOAD_RX_H_INS_17= 17'b00000100001000001;      // UART_TX: <5-rd1>
    parameter LOAD_RX_L_INS_17= 17'b00000110001000001;      // UART_TX: <5-rd1>
    
    
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
                instruction <= {8'h04,8'h03,8'h02,8'h01};
                begin 
                    #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
                end
            end
            
        
            // PC = 0xC0
            instruction <= {5'd09,5'd0,12'd09,ADDI_INS_10};      // x9 = x0 + 9     = 9
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd08,5'd0,12'd08,ADDI_INS_10};      // x8 = x0 + 8     = 8
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC8
            instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};      // x7 = x8 * x9     = 72
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
                
            // PC = 0xCC
            instruction <= {5'd10,5'd09,5'd08,ADD_INS_17};      // x10 = x9 + x8    = 17
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD0
            instruction <= {25'd02,JAL_INS_7};      //          // Jump to 0xD8     &   x1 <= PC + 4 = 0xD4
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD4
            instruction <= {25'd5,J_INS_7};                    // Jump to 0xD4 + 5 * 4 = 0xE8
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
                // PC = 0xD8
            instruction <= {5'd11,5'd09,5'd08,SUB_INS_17};      // x11 = x9 - x8    = 1
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xDC
            instruction <= {5'd07,5'd07,5'd11,SUB_INS_17};      // x7 = x7 - x11    = 71
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xE0
            instruction <= {25'b0,JALR_INS_7};                  // Jump to 0(x1)    = 0xD4
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xE4
            instruction <= {32'b0};                             // If (PC is here), it's failed this testcase
            begin 
                #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xE8
            instruction <= {5'd07,5'd07,5'd11,SUB_INS_17};      // x7 = x7 - x11    = 71 - 1 = 70
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xEC
            instruction <= {5'd00,5'd00,5'd10,7'd16,SW_INS_10}; // Store x10(17) data to 0x10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF0
            instruction <= {5'd00,5'd00,5'd07,7'd12,SW_INS_10}; // Store x7(70) data to 0x0C
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF4
            instruction <= {5'd00,5'd00,12'd00,FENCE_INS_10};   // Fence memory access
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF8
            instruction <= {5'd15,5'd00,12'd16,LB_INS_10};      // Load 16(x0) to x15 = 17
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xFC
            instruction <= {5'd16,5'd00,12'd12,LB_INS_10};      // Load 12(x0) to x16 = 70
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x100
            instruction <= {5'd09,5'd00,-12'd05,ADDI_INS_10};        // x9 = x0 - 5     = -5
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x104
            instruction <= {5'b000,5'd00,5'd09,7'b1101000,SW_INS_10}; // Store x9(-5) data to 0d1000
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x108
            instruction <= {5'd20,5'd00,12'b1111101000,LB_INS_10};      // Load 1000(x0) to x20 = -5
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x10C
            instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};      // x7 = x8 * x9     = -40
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x110
            instruction <= {5'd10,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x114
            instruction <= {5'd11,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x11
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x118
            instruction <= {5'd12,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x12
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x11C
            instruction <= {5'd13,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x13
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x120
            instruction <= {5'd14,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x14
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x124
            instruction <= {5'd15,20'hfffff,LUI_INS_7};      // Load upper 20bit (1) in x12
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x128
            instruction <= {5'd00,5'd00,5'd16,7'd20,SB_INS_10}; // Store x16(70 - 0x46) data to 0x20
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x12C
            instruction <= {5'd00,5'd00,5'd01,7'd21,SB_INS_10}; // Store x1(0xD4) data to 0x21
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x130
            instruction <= {5'd00,5'd00,5'd02,7'd22,SB_INS_10}; // Store x02(0xF0) data to 0x22
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x134
            instruction <= {5'd00,5'd00,5'd03,7'd23,SB_INS_10}; // Store x03(0x2B) data to 0x23
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x138
            instruction <= {5'd20,5'd00,12'd20,LB_INS_10};      // Load 20(x0) to x20 = 0x46
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x13C
            instruction <= {5'd21,5'd00,12'd21,LB_INS_10};      // Load 21(x0) to x21 = 0xD4
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x140
            instruction <= {5'd22,5'd00,12'd22,LB_INS_10};      // Load 22(x0) to x22 = 0xFFFFFFFFFFFFFFF0
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x144
            instruction <= {5'd23,5'd00,12'd23,LB_INS_10};      // Load 23(x0) to x23 = 0x2b
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x148
            instruction <= {5'd25,5'd00,12'd20,LW_INS_10};      // Load 20(x0) to x25 = 0x000000002bf0d446
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x14C
            instruction <= {5'd26,5'd00,12'd22,LW_INS_10};      // Load 23(x0) to x23 = 0x000000002bf0d446 (invalid_flag == 1)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x150
            instruction <= {5'd26,5'd00,12'd22,LW_INS_10};      // Load 23(x0) to x23 = 0x000000002bf0d446 (invalid_flag == 1)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x154
            quick_seperate_imm12 <= 2; #1;
            instruction <= {imm12hi,5'd20,5'd23,imm12lo,BGE_INS_10};      // JUMP to 0x15C (without sending debug data)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x158
            instruction <= {5'd23,5'd22,5'd00,7'd00,DEBUG_INS_10};      // Send debug x23 & x22
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x15C
            instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};      // x7 = x8 * x9     = -40
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0x160 (EXIT)
            instruction <= {25'd00,J_INS_7};                   // While(1) {};
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        
        `endif
        `ifdef GPIO_TESTCASE
        // Skip interrupt-program
            for(i = 0; i < 48; i = i + 1) begin
                instruction <= {8'h04,8'h03,8'h02,8'h01};
                begin 
                    #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
                end
            end
            
            // PC = 0xC0
            instruction <= {5'd10,5'd0,12'b00001111,ADDI_INS_10};      // PORT[0][7:4]: Output - PORT[0][3:0]: Input
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd00,5'd00,5'd10,7'h00,SB_INS_10};         // PORT[0][7:4]: Output - PORT[0][3:0]: Input (Addr at 0x00)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC8
            instruction <= {5'd09,5'd0,12'b01010000,ADDI_INS_10};      // x9 = 64'b01010000
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xCC
            instruction <= {5'd15,20'h40000,LUI_INS_7};      // Load upper 20bit (h30000) to x15 (to map to UART_2)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD0
            instruction <= {5'd00,5'd15,5'd09,7'h00,SB_INS_10};     // SET PORT[0][7:4]: LOW - HIGH - LOW - HIGH
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD4
            instruction <= {5'd17,5'd15,12'd00,LB_INS_10};          // Receive data (waiting until available)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            // PC = 0xD8
            instruction <= {-25'd01,J_INS_7};                        // Jump to D4;
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
//          
    
    `endif
    `ifdef PERIPHERAL_TESTCASE
    
        // Skip interrupt-program
            for(i = 0; i < 48; i = i + 1) begin
                instruction <= {8'h04,8'h03,8'h02,8'h01};
                begin 
                    #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
                end
            end
            
            // PC = 0xC0
            instruction <= {5'd10,5'd0,12'b00001100,ADDI_INS_10};      // COM_PERIPHERAL = TX_ENABLE << 1 & RX_ENABLE << 1;
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd00,5'd00,5'd10,7'h06,SB_INS_10};     // Load COM_PERIPHERAL 
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC8
            instruction <= {5'd09,5'd0,12'd9,ADDI_INS_10};      // x9 = x0 + 9     = 9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xCC
            instruction <= {5'd15,20'hC0000,LUI_INS_7};      // Load upper 20bit (h30000) to x15 (to map to UART_2)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD0
            instruction <= {5'd08,5'd0,12'd2,ADDI_INS_10};      // x8 = x0 + 2     = 2
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD4
            instruction <= {5'd07,5'd09,5'd08,ADD_INS_17};      // x7 = x8 + x9     = 11
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD8
            instruction <= {5'd00,5'd15,5'd07,7'h00,SW_INS_10};     // Send 0x0B 0x00 0x00 0x00
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            //  PC = 0xDC
            instruction <= {5'd15,20'h80000,LUI_INS_7};      // Load upper 20bit (h30000) to x15 (to map to UART_2)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xE0
            instruction <= {5'd08,5'd0,12'd2,ADDI_INS_10};      // x8 = x0 + 2     = 2
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xE4
            instruction <= {5'd07,5'd09,5'd08,ADD_INS_17};      // x7 = x8 + x9     = 11
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xE8
            instruction <= {5'd00,5'd15,5'd07,7'h00,SW_INS_10};     // Send 0x0B 0x00 0x00 0x00
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xEC
            instruction <= {5'd17,5'd15,12'd00,LD_INS_10};          // Receive data (waiting until available)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
//            // PC = 0xD8
//            instruction <= {5'd16,5'd09,5'd07,MUL_INS_17};
//            begin
//                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
//            end
//            // PC = 0xDC
//            instruction <= {5'd16,5'd00,5'd00,LOAD_RX_L_INS_17};      // Receive data (waiting until available)
//            begin
//                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
//            end
            
//            // PC = 0xE0
//            instruction <= {5'd16,5'd16,12'd01,ADDI_INS_10};
//            begin
//                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
//            end
//            // PC = 0xE4
//            instruction <= {-25'd4,J_INS_7};                    // Jump to 0xD4 + 25'd24 = 0xE8
//            begin
//                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
//            end
            
    
    `endif
    
    `ifdef INTERRUPT_HANDLER_TESTCASE
        // ISR 1 ////////////////////////////////////////////////////////////////////////////////////////
        // PC = 0x00 - 1
            instruction <= {5'd00,5'd00,5'd09,7'd50,SB_INS_10};      // Restore x9 to 0x50 (Store x9 to 0x50)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x04 - 2
            instruction <= {5'd09,5'd00,12'd60,LB_INS_10};           // Load global data (at 0x60) to x9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x08 - 3
            instruction <= {5'd09,5'd09,12'd01,ADDI_INS_10};        // x9 = x9 + 1     
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x0C - 4
            instruction <= {5'd00,5'd00,5'd09,7'd60,SB_INS_10};     // Store x9 to 0x60
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x10 - 5
            instruction <= {5'd09,5'd00,12'd50,LB_INS_10};         // Recovery previous x9 from 0x50
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x14 - 6
            instruction <= {5'd00,5'd00,12'd00,RETI_INS_10};      // RETI
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        for(int i = 0; i < 10; i = i + 1) begin    
        // PC = 0x18 - 7
            instruction <= {5'd00,5'd00,12'd00,10'd00};      
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        end
        /////////////////////////////////////////////////////////////////////////////////////////////////   
        
        // ISR 2 //////////////////////////////////////////////////////////////////////////////////////// 
        // PC = 0x40 - 1
            instruction <= {5'd00,5'd00,5'd09,7'd100,SB_INS_10};      // Restore x9 to 0x50 (Store x9 to 0x100)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x44 - 2
            instruction <= {5'd09,5'd00,12'd120,LB_INS_10};           // Load global data (at 0x120) to x9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x48 - 3
            instruction <= {5'd09,5'd09,12'd02,ADDI_INS_10};        // x9 = x9 + 2     
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x4C - 4
            instruction <= {5'd00,5'd00,5'd09,7'd120,SB_INS_10};     // Store x9 to 0x120
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x50 - 5
            instruction <= {5'd09,5'd00,12'd100,LB_INS_10};         // Recovery previous x9 from 0x100
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x54 - 6
            instruction <= {5'd00,5'd00,12'd00,RETI_INS_10};      // RETI
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        for(int i = 0; i < 10; i = i + 1) begin    
        // PC = 0x58 - 7
            instruction <= {5'd00,5'd00,12'd00,10'd00};      
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        end
        ///////////////////////////////////////////////////////////////////////////////////////////////// 
        
        // ISR 3 //////////////////////////////////////////////////////////////////////////////////////// 
        // PC = 0x80 - 1
            instruction <= {5'd00,5'd00,5'd09,7'd30,SB_INS_10};      // Restore x9 to 0x50 (Store x9 to 0x30)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x84 - 2
            instruction <= {5'd09,5'd00,12'd40,LB_INS_10};           // Load global data (at 0x40) to x9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x88 - 3
            instruction <= {5'd09,5'd09,12'd03,ADDI_INS_10};        // x9 = x9 + 3    
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x8C - 4
            instruction <= {5'd00,5'd00,5'd09,7'd40,SW_INS_10};     // Store x9 to 0x40
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x90 - 5
            instruction <= {5'd09,5'd00,12'd30,LB_INS_10};         // Recovery previous x9 from 0x30
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0x94 - 6
            instruction <= {5'd00,5'd00,12'd00,RETI_INS_10};      // RETI
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        for(int i = 0; i < 10; i = i + 1) begin    
        // PC = 0x98 - 7
            instruction <= {5'd00,5'd00,12'd00,10'd00};      
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        end
        ///////////////////////////////////////////////////////////////////////////////////////////////// 
        
        // MAIN ///////////////////////////////////////////////////////////////////////////////////////// 
        // PC = 0xC0
            instruction <= {5'd20,5'd0,12'b00001111,ADDI_INS_10};      // PORT[0][7:4]: Output - PORT[0][3:0]: Input
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0xC4
            instruction <= {5'd00,5'd00,5'd20,7'h00,SB_INS_10};         // PORT[0][7:4]: Output - PORT[0][3:0]: Input (Addr at 0x00)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0xC0
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 5     = 5
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xC4
            instruction <= {5'd08,5'd00,12'b10100000,ADDI_INS_10};      // x8 = 8'b11000010 (store configuration data)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xC8
            instruction <= {5'd00,5'd00,5'd08,7'h0D,SB_INS_10};     // Store x8 to 0x0D (configure EXTERNAL_INTERRUPT) 8'h0D
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xCC
            instruction <= {5'd08,5'd00,12'b11100000,ADDI_INS_10};      // x8 = 8'b11100010 (store configuration data)
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD0
            instruction <= {5'd00,5'd00,5'd07,7'h10,SB_INS_10};     // Store x7 to 0x10 (configure timer_limit_low) 8'd09
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD4
            instruction <= {5'd00,5'd00,5'd00,7'h0F,SB_INS_10};     // Store x0 to 0x0F (configure timer_limit_high) 8'd00
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xD8
            instruction <= {5'd00,5'd00,5'd08,7'h0E,SB_INS_10};     // Store x8 to 0x0E (configure timer0) 8'b11000001
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
        // PC = 0xE4
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xE8
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xEC
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xF0
            instruction <= {5'd07,5'd00,12'd10,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xF4
            instruction <= {5'd07,5'd00,12'b10010000,ADDI_INS_10};            // x7 = x0 + 10     = 10
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xF8
            instruction <= {5'd00,5'd00,5'd07,7'h0C,SB_INS_10};     // Store x7 to 0x0C (configure external interrupt) 
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xFC
            instruction <= {25'd00,J_INS_7};                        // While(1);
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        ///////////////////////////////////////////////////////////////////////////////////////////////// 
    `endif
    `ifdef PARALLEL_TESTCASE
            // Skip interrupt-program
            for(i = 0; i < 48; i = i + 1) begin
                instruction <= {5'd10,5'd0,5'd10,ADDI_INS_10};
                begin 
                    #3;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
                end
            end
            
            // PC = 0xC0
            instruction <= {5'd09,5'd0,12'd4095,ADDI_INS_10};      // x9 = x0 + -1     = -1
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd08,5'd0,12'd4095,ADDI_INS_10};      // x8 = x0 + -1     = -1
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC8    (Stress processor_1)
            instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};      // x7 = x8 * x9     = 1
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xCC
            instruction <= {5'd10,5'd09,5'd08,ADD_INS_17};      // x10 = x9 + x8    = -1 + -1
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
             
            // PC = 0xD0
            instruction <= {5'd11,5'd08,12'd05,ADDI_INS_10};      // x11 = x8 + 8    = -1 + 5 = 4100
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD4
            instruction <= {5'd12,5'd08,5'd09,ADD_INS_17};      // x12 = x8 + x9    = -1 + -1 = 8190
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD8
            instruction <= {25'd00,J_INS_7};                    // While(1);
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#3 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        `endif
    
    // Finish program
    `ifdef FINISH_PROGRAM_OPCODE
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= {1'b0, FINISH_PROGRAM_OPCODE};
        #3 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
    `endif    
    end
    `ifdef GPIO_TESTCASE
    initial begin
//        GPIO_PORT_A[2] <= 1;
//        IO_PORT[2] <= 1;
//        GPIO_driver[2] <= 1;
        #59950;
        GPIO_driver[0][0] <= 0;
        GPIO_driver[0][1] <= 0;
        GPIO_driver[0][2] <= 0;
        GPIO_driver[0][3] <= 0;
        GPIO_driver[0][4] <= 1;
        GPIO_driver[0][5] <= 1;
        GPIO_driver[0][6] <= 1;
        GPIO_driver[0][7] <= 1;
        GPIO_ext[0][0] <= 0;
        GPIO_ext[0][1] <= 0;
        GPIO_ext[0][2] <= 1;
        GPIO_ext[0][3] <= 1;
//        interrupt_request_3 <= 0;
//        #1 interrupt_request_3 <= 1;
//        #2 interrupt_request_3 <= 0;
        
        
//        GPIO_PORT_A[2] <= 0;
//        IO_driver[2] <= 0;
        
        
        #5000;
        
//        GPIO_PORT_A[2] <= 1;
//        IO_driver[2] <= 1;
    end
    `endif
    
    `ifdef PERIPHERAL_TESTCASE
    initial begin
        #70000;
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h27;
        #3 TX_use_ex_2 <= 1;
        #2 TX_use_ex_2 <= 0;
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h02;
        #3 TX_use_ex_2 <= 1;
        #2 TX_use_ex_2 <= 0;
        
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h20;
        #3 TX_use_ex_2 <= 1;
        #2 TX_use_ex_2 <= 0;
        
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h03;
        #3 TX_use_ex_2 <= 1;
        #2 TX_use_ex_2 <= 0;
        
        #4760;
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h21;
        #3 TX_use_ex_2 <= 1;
        #2 TX_use_ex_2 <= 0;
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h10;
        #3 TX_use_ex_2 <= 1;
        #2 TX_use_ex_2 <= 0;
        
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h20;
        #3 TX_use_ex_2 <= 1;
        #2 TX_use_ex_2 <= 0;
        
        
        
        TX_use_ex_2 <= 0;
        data_bus_in_uart_ex_2 <= 8'h03;
        #3 TX_use_ex_2 <= 1;
        #2 TX_use_ex_2 <= 0;
        
        /* UART_1 Receive data */
        #4760;
        
        #10;
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 8'h01;
        #3 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
        
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 8'h02;
        #3 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
        
        
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 8'h03;
        #3 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
        
        
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 8'h04;
        #3 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
        #4760;
        
        
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 8'h05;
        #3 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
        
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 8'h06;
        #3 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
        
        
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 8'h07;
        #3 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
        
        
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 8'h08;
        #3 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
    end
    `endif
    
    `ifdef INTERRUPT_HANDLER_TESTCASE
    initial begin
        #89950;
        
//        interrupt_request_3 <= 0;
//        #1 interrupt_request_3 <= 1;
//        #2 interrupt_request_3 <= 0;
        
        #16;
        GPIO_driver[0][2] <= 0;
        
        GPIO_ext[0][2] <= 1;
        
//        external_int_pin <= 0;
//        #2 external_int_pin <= 1;
//        #10 external_int_pin <= 0;
        
        #100;
        GPIO_ext[0][2] <= 0;
  
//        external_int_pin <= 0;
//        #2 external_int_pin <= 1;
//        #10 external_int_pin <= 0;
        
        #400;
        
        GPIO_ext[0][2] <= 1;
//        external_int_pin <= 0;
//        #2 external_int_pin <= 1;
//        #10 external_int_pin <= 0;
        
        #200;
        
        GPIO_ext[0][2] <= 0;
//        external_int_pin <= 0;
//        #2 external_int_pin <= 1;
//        #10 external_int_pin <= 0;
    end
    `endif 
    
    initial begin   : STOP_BLOCK
        `ifdef PERIPHERAL_TESTCASE
        #(70000 * 2) $stop;
        #2750200 $stop;
        `else
        #202000 $stop;
        #(02200*9) $stop;
        #(02200*7) $stop;
        `endif
    end 
endmodule
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/29/2023 01:01:06 AM
// Design Name: 
// Module Name: general_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//`define NORMAL_TESTCASE
//`define PARALLEL_TESTCASE
//`define PREVENT_OUTDATED_DATA_TESTCASE
//`define INTERRUPT_TESTCASE
`define PERIPHERAL_TESTCASE

// Interrrupt option
//`define PREEMPTIVE_CASE
//

module general_tb;
    parameter DATA_WIDTH            = 8;
    parameter DOUBLEWORD_WIDTH      = 64;
    parameter DATA_MEMORY_SIZE      = 256;      // 256 bytes (2Kb)
    
    parameter ADDR_WIDTH_DM         = $clog2(DATA_MEMORY_SIZE);
    parameter DATA_TYPE_WIDTH       = 2;
    
    parameter INSTRUCTION_WIDTH     = 32;   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 256;   
    // PM
    parameter ADDR_WIDTH_PM         = $clog2(PROGRAM_MEMORY_SIZE);
    parameter START_WR_ADDR_PM      = 8'h00;

    // Deep configuration
    parameter FINISH_PROGRAM_OPCODE  = 7'b0001011;
    parameter FINISH_PROGRAM_TIMER   = 1250000;
    
    parameter REGISTER_AMOUNT        = 32;
    parameter REG_SPACE_WIDTH        = $clog2(REGISTER_AMOUNT);
    
    parameter ADDR_MAPPING_PERIPHERAL   = 5; 
    parameter ADDDR_MAPPING_WIDTH       = $clog2(ADDR_MAPPING_PERIPHERAL);
    // Protocol peripheral communication 
    parameter AMOUNT_SND_BYTE           = 16;  
    parameter AMOUNT_RCV_BYTE           = 16;
    parameter AMOUNT_SND_WIDTH          = $clog2(AMOUNT_SND_BYTE);
    parameter AMOUNT_RCV_WIDTH          = $clog2(AMOUNT_RCV_BYTE);
    
    // Faster for simulation
    parameter CLOCK_DIVIDER_UNIQUE_1 = 5;
    
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
    
    // UART_ex & UART_1
    wire    TX_ex;
    reg     TX_use_ex;
    reg     [7:0] data_bus_in_tx_ex;
    wire    [7:0] TX_config_register_ex = 8'b10001111;
    wire    [7:0] RX_config_register_1  = 8'b10001111;
    wire    RX_1;
    
    // Processor 1
    // Multi-processor Manager
    wire[INSTRUCTION_WIDTH - 1:0]   fetch_instruction_1;
    wire                            boot_processor_1;
    wire                            processor_idle_1;
    wire[1:0]                       main_state;
    // Processor 2
    wire[INSTRUCTION_WIDTH - 1:0]   fetch_instruction_2;
    wire                            boot_processor_2;
    wire                            processor_idle_2;
    
    // Synchrization primitive 
    wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd_p1;
    wire [ADDR_WIDTH_DM - 1:0]      addr_rd_p1;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd_p1;
    wire                            rd_idle_p1;
    wire                            rd_ins_p1;
    wire                            rd_access_p1;
    wire                            rd_finish_p1;
    // Synchronization primitive (WRITE_STATE)
    wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_p1;
    wire [ADDR_WIDTH_DM - 1:0]      addr_wr_p1;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr_p1;
    wire                            wr_idle_p1;   
    wire                            wr_ins_p1;
    wire                            wr_access_p1;
    // Synchrization primitive 
    wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd_p2;
    wire [ADDR_WIDTH_DM - 1:0]      addr_rd_p2;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd_p2;
    wire                            rd_idle_p2;
    wire                            rd_ins_p2;
    wire                            rd_access_p2;
    wire                            rd_finish_p2;
    // Synchronization primitive (WRITE_STATE)
    wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_p2;
    wire [ADDR_WIDTH_DM - 1:0]      addr_wr_p2;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr_p2;
    wire                            wr_idle_p2;   
    wire                            wr_ins_p2;
    wire                            wr_access_p2;
    
    // Data memory 
    wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd_dm;
    wire [ADDR_WIDTH_DM - 1:0]      addr_rd_dm;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd_dm;
    wire                            rd_idle_dm;   
    wire                            rd_ins_dm;   
    // Data memory (Write handler)
    wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_dm;
    wire [ADDR_WIDTH_DM - 1:0]      addr_wr_dm;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr_dm;
    wire                            wr_idle_dm;   
    wire                            wr_ins_dm;  
    
    // Debug 
    wire    [63:0] debug_1;
    wire    [63:0] debug_2;
    
    // 
    wire    [DATA_WIDTH - 1:0]      program_memory_wire [0: PROGRAM_MEMORY_SIZE - 1];
    wire    [DATA_WIDTH - 1:0]      data_memory_wire [0: DATA_MEMORY_SIZE - 1];
    wire [DOUBLEWORD_WIDTH - 1:0]   processor_registers_1 [0:REGISTER_AMOUNT - 1];
    wire [DOUBLEWORD_WIDTH - 1:0]   processor_registers_2 [0:REGISTER_AMOUNT - 1];
    wire [DOUBLEWORD_WIDTH - 1:0]   registers_renew [0:REGISTER_AMOUNT - 1];
    
    
    // Multi-processor manager
    // Program memory
    wire    [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd_pm;
    wire                                rd_idle_pm;
    wire    [ADDR_WIDTH_PM - 1:0]       addr_rd_pm;
    wire                                rd_ins_pm;
    // Registers management
    wire    [DOUBLEWORD_WIDTH - 1:0]    ra_register;
    wire    [REG_SPACE_WIDTH*3 - 1:0]   register_num;
    wire                                boot_renew_register_1;
    wire                                boot_renew_register_2;
    wire                                synchronized_processors;
    wire    [0:REGISTER_AMOUNT - 1]     processing_register_table;  
    // Interrupt control
    wire    interrupt_flag_1;
    wire    interrupt_flag_2;
    wire    interrupt_flag_3;
    wire    RETI_1;
    wire    RETI_2;
    wire    RETI_3;
    wire    interrupt_handling_1;
    wire    interrupt_handling_2;
    wire    interrupt_handling_3;
    // Interrupt unit
    reg     interrupt_request_1;
    reg     interrupt_request_2;
    reg     interrupt_request_3;
    // Confuration register
    wire    interrupt_enable_1 = 1'b1;
    wire    interrupt_enable_2 = 1'b1;
    wire    interrupt_enable_3 = 1'b1;
    
    // Registers management
    wire                                new_data_register       [0:REGISTER_AMOUNT - 1];
    wire                                synchronization_processor_1;
    wire                                synchronization_processor_2;
    
    
    wire [ADDDR_MAPPING_WIDTH - 1:0]    protocol_address_mapping;
    wire [DOUBLEWORD_WIDTH*2 - 1:0]     data_snd_protocol_per;
//    wire [DOUBLEWORD_WIDTH*2 - 1:0]     data_rcv_protocol_per;
    wire                                send_protocol_clk;
//    wire                                receive_protocol_clk;
    wire [AMOUNT_SND_WIDTH - 1:0]       amount_snd_byte_protocol;
//    wire [AMOUNT_RCV_WIDTH - 1:0]       amount_rcv_byte_protocol;
    reg                                 snd_protocol_available;
//    reg                                 rcv_protocol_available;
    wire [DATA_WIDTH - 1:0]             data_snd_small;
    wire                                snd_small_clk;
//    wire    empty_rcv;
//    wire    full_rcv;
    
    assign RX_1 = TX_ex ;
    
    com_uart    #(
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
                
    com_uart    #(
                .SLEEP_MODE(0), 
                .RX_FLAG_CONFIG(1'b1), /// Internal FIFO
                .CLOCK_DIVIDER_UNIQUE_1(CLOCK_DIVIDER_UNIQUE_1)
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
//                wire    [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd_pm;
//    wire                                rd_idle_pm;
//    wire    [ADDR_WIDTH_PM - 1:0]       addr_rd_pm;
//    wire                                rd_ins_pm;
    ram_module      #(
                    .ADDR_DEPTH(PROGRAM_MEMORY_SIZE),
                    .RESERVED_REG_AMOUNT(1'b1)
                    )program_memory(
                    .clk(clk),
                    .data_bus_wr(data_bus_wr_pm),
                    .data_type_wr(DATA_WIDTH),    // Write byte (8bit)
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
                    ,.registers_wire(program_memory_wire)
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
                        
                        .rst_n(rst_n)
                        
                        // Debug
                        ,.debug_1(debug_1)
                        );                
    Processor           #(
                        .MAIN_RPOCESSOR(1'b0),
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
                        .data_snd_protocol_per(data_snd_protocol_per),
                        .send_protocol_clk(send_protocol_clk),
                        .amount_snd_byte_protocol(amount_snd_byte_protocol),
                        .snd_protocol_available(snd_protocol_available),
                        
                        .rst_n(rst_n)
                        
                        // Debug
                        ,.debug_2(debug_2)
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
                        
    Sync_primitive  #(
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
    Interrupt_control
                    #(
                    )interrupt_control(
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
    ram_module      #(
                    .ADDR_DEPTH(DATA_MEMORY_SIZE),
                    .RESERVED_REG_AMOUNT(1'b1)
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
                    .rst_n(rst_n)
                    //Debug 
                    ,.registers_wire(data_memory_wire)
                    );
    fifo_advanced_module #(
    
                    )fifo_advanced_module(
                    .clk(clk),
                    
                    .snd_big_clk(send_protocol_clk),
                    .amount_byte_snd_big(amount_snd_byte_protocol),
                    .data_snd_big(data_snd_protocol_per),
                    .available_snd(snd_protocol_available),
                    .data_snd_small(data_snd_small),
                    .snd_small_clk(snd_small_clk),
                    
                    .rst_n(rst_n)
                    );
    initial begin
        clk <= 0;
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 0;
        interrupt_request_1 <= 0;
        interrupt_request_2 <= 0;
        interrupt_request_3 <= 0;
        snd_protocol_available <= 1;
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
    
    parameter UART_TX_INS_11= 11'b10001000001;      // UART_TX: <5-rs3><5-rs1><5-rs2><6-imm>
    
        int i;
    reg [31:0] instruction;
    initial begin : FAKE_RPOGRAM_BLOCK
        #11;
        
        
        `ifdef PARALLEL_TESTCASE
            // Skip interrupt-program
            for(i = 0; i < 48; i = i + 1) begin
                instruction <= {5'd10,5'd0,5'd10,ADDI_INS_10};
                begin 
                    #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
                end
            end
            
            // PC = 0xC0
            instruction <= {5'd09,5'd0,12'd4095,ADDI_INS_10};      // x9 = x0 + 4095     = 4095
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd08,5'd0,12'd4095,ADDI_INS_10};      // x8 = x0 + 4095     = 4095
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC8    (Stress processor_1)
            instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};      // x7 = x8 * x9     = 16,769,025
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xCC
            instruction <= {5'd10,5'd09,5'd08,ADD_INS_17};      // x10 = x9 + x8    = 4095 + 4095
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
             
            // PC = 0xD0
            instruction <= {5'd11,5'd08,12'd05,ADDI_INS_10};      // x11 = x8 + 8    = 4095 + 5 = 4100
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD4
            instruction <= {5'd12,5'd08,5'd09,ADD_INS_17};      // x12 = x8 + x9    = 4095 + 4095 = 8190
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD8
            instruction <= {25'd00,J_INS_7};                    // While(1);
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        `endif
        
        `ifdef PREVENT_OUTDATED_DATA_TESTCASE
            // Skip interrupt-program
            for(i = 0; i < 48; i = i + 1) begin
                instruction <= {5'd10,5'd0,5'd10,ADDI_INS_10};
                begin 
                    #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
                end
            end
            
            // PC = 0xC0
            instruction <= {5'd09,5'd0,12'd4095,ADDI_INS_10};      // x9 = x0 + 4095     = 4095
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd08,5'd0,12'd4095,ADDI_INS_10};      // x8 = x0 + 4095     = 4095
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Case: Instruction 0xC8 and 0xCC is fetched same time, but Instruction 0xC8 have x7 (register destination), instruction 0xCC use data in x7 as oprand
            // MPM will block parallel mechanisim of this case 
            // PC = 0xC8    (Stress processor_1)
            instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};      // x7 = x8 * x9     = 16769025
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xCC
            instruction <= {5'd10,5'd09,5'd07,ADD_INS_17};      // x10 = x9 + x7    = 4095 + 16769025
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
             
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  
            // PC = 0xD0
            instruction <= {5'd11,5'd09,5'd08,MUL_INS_17};      // x11 = x8 * x9    = 4095 * 4095 = 16769025
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xD4
            instruction <= {5'd11,5'd08,5'd09,ADD_INS_17};      // x12 = x8 + x9    = 4095 + 4095 = 8190
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            // PC = 0xD8
            instruction <= {25'd00,J_INS_7};                    // While(1);
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        `endif
        
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
            instruction <= {5'd00,5'd00,5'd10,7'd15,SW_INS_10}; // Store x10(17) data to 0x0F
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
    //        // PC = 0xF0
    //        instruction <= {5'd00,5'd00,5'd07,7'd12,SW_INS_10}; // Store x7(79) data to 0x0C
    //        begin
    //            #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
    //        end
            
            // PC = 0xF0
            instruction <= {5'd00,5'd00,12'd00,FENCE_INS_10};   // Fence memory access
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF4
            instruction <= {5'd15,5'd00,12'd15,LB_INS_10};      // Load 15(x0) to x15 = 17
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF8
            instruction <= {5'd16,5'd00,12'd12,LB_INS_10};      // Load 12(x0) to x16 = 70
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = FC
            instruction <= {25'd00,J_INS_7};                   // While(1) {};
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        
    `endif


//    parameter ADD_INS_17    = 17'b00000000000110011;// ADD:     <5-rd><5-rs1><5-rs2>
//    parameter ADDI_INS_10   = 10'b0000010011;       // ADDI:    <5-rd><5rs1><12-imm>
//    parameter SUB_INS_17    = 17'b10000000000110011;// SUB:     <5-rd><5-rs1><5-rs2>
//    parameter SLL_INS_17    = 17'b00000000010110011;// SLL:     <5-rd><5-rs1><5-rs2>
//    parameter SRL_INS_17    = 17'b00000001010110011;// SRL:     <5-rd><5-rs1><5-rs2>
//    parameter MUL_INS_17    = 17'b00000010000110011;// MUL:     <5-rd><5-rs1><5-rs2>
//    // Load                                                     dest  base  offset
//    parameter LB_INS_10     = 10'b0000000011;       // LB:      <5-rd><5rs1><12-imm>
//    parameter LW_INS_10     = 10'b0100000011;       // LW:      <5-rd><5rs1><12-imm>
//    parameter LD_INS_10     = 10'b0110000011;       // LD:      <5-rd><5rs1><12-imm>
//    // Store                                                    offset  base   src   offset
//    parameter SB_INS_10     = 10'b0000100011;       // SB:      <5-immh><5-rs1><5rs2><7-imml>
//    parameter SW_INS_10     = 10'b0100100011;       // SW:      <5-immh><5-rs1><5rs2><7-imml>
//    parameter SD_INS_10     = 10'b0110100011;       // SD:      <5-immh><5-rs1><5rs2><7-imml>
    
//    parameter J_INS_7       = 7'b1100111;           // J:       <25-imm>
//    parameter JAL_INS_7     = 7'b1101111;           // J:       <25-imm>
//    parameter JALR_INS_7    = 7'b1101011;           // J:       <25-imm>
    
//    parameter FENCE_INS_10  = 10'b0100101111;       // FENCE:   Don't care
    
//    parameter RETI_INS_10   = 10'b0111110111;       // RETI:   Don't care
    `ifdef INTERRUPT_TESTCASE
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
            instruction <= {5'd09,5'd00,12'd09,ADDI_INS_10};      // x9 = x0 + 9     = 9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
        // PC = 0xC4
            instruction <= {25'd00,J_INS_7};                    // While(1);
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
            instruction <= {5'd09,5'd0,12'd9,ADDI_INS_10};      // x9 = x0 + 9     = 9
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC4
            instruction <= {5'd08,5'd0,12'd2,ADDI_INS_10};      // x8 = x0 + 8     = 2
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xC8
            instruction <= {5'd07,5'd09,5'd08,ADD_INS_17};      // x7 = x8 + x9     = 11
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xCC
            instruction <= {5'd08,5'd07,5'd09,6'b00,UART_TX_INS_11};      // Send 9bytes of x7 -> x8
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
            instruction <= {5'd00,5'd00,5'd10,7'd15,SW_INS_10}; // Store x10(17) data to 0x0F
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF0
            instruction <= {5'd00,5'd00,12'd00,FENCE_INS_10};   // Fence memory access
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF4
            instruction <= {5'd15,5'd00,12'd15,LB_INS_10};      // Load 15(x0) to x15 = 17
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = 0xF8
            instruction <= {5'd16,5'd00,12'd12,LB_INS_10};      // Load 12(x0) to x16 = 70
            begin
                #1;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[7:0];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[15:8];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[23:16];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;TX_use_ex <= 0;data_bus_in_tx_ex <= instruction[31:24];#1 TX_use_ex <= 1;#2 TX_use_ex <= 0;
            end
            
            // PC = FC
            instruction <= {25'd00,J_INS_7};                   // While(1) {};
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
        
    `ifdef INTERRUPT_TESTCASE
    initial begin   : INTERRUPT_EVENT
        
        `ifdef PREEMPTIVE_CASE
        #48550;
        
        interrupt_request_3 <= 0;
        #1 interrupt_request_3 <= 1;
        #2 interrupt_request_3 <= 0;
        
        #16;
        
        interrupt_request_2 <= 0;
        #1 interrupt_request_2 <= 1;
        #2 interrupt_request_2 <= 0;
        
        #16;
        
        interrupt_request_1 <= 0;
        #1 interrupt_request_1 <= 1;
        #2 interrupt_request_1 <= 0;
        `endif
        
        
    end    
    `endif
    
    initial begin   : STOP_BLOCK
        #62200 $stop;
    end
endmodule

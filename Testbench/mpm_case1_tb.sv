`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/27/2023 06:37:15 PM
// Design Name: 
// Module Name: mpm_case1_tb
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


module mpm_case1_tb;

    parameter DATA_WIDTH            = 8;
    parameter DOUBLEWORD_WIDTH      = 64;
    parameter DATA_MEMORY_SIZE      = 10'd256;      // 256 bytes (2Kb)
    
    parameter ADDR_WIDTH_DM         = $clog2(DATA_MEMORY_SIZE);
    parameter DATA_TYPE_WIDTH       = 2;
    
    parameter INSTRUCTION_WIDTH     = 32;   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 256;   // 64 instruction
    // PM
    parameter ADDR_WIDTH_PM         = $clog2(PROGRAM_MEMORY_SIZE);
    parameter START_WR_ADDR_PM      = 8'h00;

    // Deep configuration
    parameter FINISH_PROGRAM_OPCODE  = 7'b0001011;
    parameter FINISH_PROGRAM_TIMER   = 1250000;
    
    parameter REGISTER_AMOUNT        = 32;
    parameter REG_SPACE_WIDTH        = $clog2(REGISTER_AMOUNT);
    
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
    // Synchrization primitive 
    reg  [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd_1;
    wire [ADDR_WIDTH_DM - 1:0]      addr_rd_1;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd_1;
    reg                             rd_idle_1;
    wire                            rd_ins_1;
    reg                             rd_access_1;
    wire                            rd_finish_1;
    // Synchronization primitive (WRITE_STATE)
    wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_1;
    wire [ADDR_WIDTH_DM - 1:0]      addr_wr_1;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr_1;
    reg                             wr_idle_1;   
    wire                            wr_ins_1;
    reg                             wr_access_1;
    
    // Processor 2
    wire[INSTRUCTION_WIDTH - 1:0]   fetch_instruction_2;
    wire                            boot_processor_2;
    wire                            processor_idle_2;
    // Synchrization primitive 
    reg  [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd_2;
    wire [ADDR_WIDTH_DM - 1:0]      addr_rd_2;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd_2;
    reg                             rd_idle_2;
    wire                            rd_ins_2;
    reg                             rd_access_2;
    wire                            rd_finish_2;
    // Synchronization primitive (WRITE_STATE)
    wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_2;
    wire [ADDR_WIDTH_DM - 1:0]      addr_wr_2;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr_2;
    reg                             wr_idle_2;   
    wire                            wr_ins_2;
    reg                             wr_access_2;
    
    // Debug 
    wire    [63:0] debug_1;
    wire    [63:0] debug_2;
    wire    [DATA_WIDTH - 1:0]      registers_wire [0: PROGRAM_MEMORY_SIZE - 1];
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
    wire    [REG_SPACE_WIDTH - 1:0]     register_num;
    wire                                boot_renew_register_1;
    wire                                boot_renew_register_2;
    wire                                main_program_state; 
    // Interrupt control
    wire                                interrupt_flag_1 = 1'b0;
    wire                                interrupt_flag_2 = 1'b0;
    wire                                interrupt_flag_3 = 1'b0;
    
    // Registers management
    wire                                new_data_register       [0:REGISTER_AMOUNT - 1];
    
    assign RX_1 = TX_ex ;
    
    com_uart    #(
                .SLEEP_MODE(0),
                .FIFO_DEPTH(13'd4096)
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
                    .data_type_wr(BYTE_SIZE_ENCODE),    // Write byte (8bit)
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
                        // Case 2 start
                        // Multi-processor manager
                        .fetch_instruction(fetch_instruction_1),
                        .boot_processor(boot_processor_1),
                        .processor_idle(processor_idle_1),
                        // Synchronization primitive
                        // - read
                        .data_bus_rd(data_bus_rd_1),
                        .addr_rd(addr_rd_1),
                        .data_type_rd(data_type_rd_1),
                        .rd_idle(rd_idle_1),
                        .rd_ins(rd_ins_1),
                        .rd_access(rd_access_1),
                        .rd_finish(rd_finish_1),
                        // - write
                        .data_bus_wr(data_bus_wr_1),
                        .addr_wr(addr_wr_1),
                        .data_type_wr(data_type_wr_1),
                        .wr_idle(wr_idle_1),
                        .wr_ins(wr_ins_1),
                        .wr_access(wr_access_1),
                        
                        .rst_n(rst_n)
                        
                        // Debug
                        ,.debug_1(debug_1)
                        ,.processor_registers(processor_registers_1)
                        ,.registers_renew(registers_renew)
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
                        .data_bus_rd(data_bus_rd_2),
                        .addr_rd(addr_rd_2),
                        .data_type_rd(data_type_rd_2),
                        .rd_idle(rd_idle_2),
                        .rd_ins(rd_ins_2),
                        .rd_access(rd_access_2),
                        .rd_finish(rd_finish_2),
                        // - write
                        .data_bus_wr(data_bus_wr_2),
                        .addr_wr(addr_wr_2),
                        .data_type_wr(data_type_wr_2),
                        .wr_idle(wr_idle_2),
                        .wr_ins(wr_ins_2),
                        .wr_access(wr_access_2),
                        
                        .rst_n(rst_n)
                        
                        // Debug
                        ,.debug_2(debug_2)
                        ,.processor_registers(processor_registers_2)
                        ,.registers_renew(registers_renew)
                        );   
    registers_management#(
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
                        .main_program_state(main_program_state),
                        .registers_renew(registers_renew),
                        .ra_register(ra_register),
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
                        .main_program_state(main_program_state),
                        // Interrup control
                        .interrupt_flag_1(interrupt_flag_1),
                        .interrupt_flag_2(interrupt_flag_2),
                        .interrupt_flag_3(interrupt_flag_3),
                        
                        .rst_n(rst_n)
                        );
    initial begin
        clk <= 0;
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 0;
        data_bus_rd_1 <= 0;
        data_bus_rd_2 <= 0;
        rd_idle_1 <= 0;
        rd_idle_2 <= 0;
        rd_idle_1 <= 0;
        rd_idle_2 <= 0;
        rd_access_1 <= 0;
        rd_access_2 <= 0;
        wr_idle_1 <= 0;
        wr_idle_2 <= 0;
        wr_access_1 <= 0;
        wr_access_2 <= 0;
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
    parameter LW_INS_10     = 10'b0100000011;       // LW:      <5-rd><5rs1><12-imm>
    
    parameter J_INS_7       = 7'b1100111;           // J:       <25-imm>
    parameter JAL_INS_7     = 7'b1101111;           // J:       <25-imm>
    parameter JALR_INS_7    = 7'b1101011;           // J:       <25-imm>
        int i;
    reg [31:0] instruction;
    initial begin : FAKE_RPOGRAM_BLOCK
        #11;
        
        // Skip interrupt-program
        for(i = 0; i < 48; i = i + 1) begin
            instruction <= {5'd10,5'd0,5'd10,ADDI_INS_10};
            begin 
                #1;
                TX_use_ex <= 0;
                data_bus_in_tx_ex <= instruction[7:0];
                #1 TX_use_ex <= 1;
                #2 TX_use_ex <= 0;
                TX_use_ex <= 0;
                data_bus_in_tx_ex <= instruction[15:8];
                #1 TX_use_ex <= 1;
                #2 TX_use_ex <= 0;
                TX_use_ex <= 0;
                data_bus_in_tx_ex <= instruction[23:16];
                #1 TX_use_ex <= 1;
                #2 TX_use_ex <= 0;
                TX_use_ex <= 0;
                data_bus_in_tx_ex <= instruction[31:24];
                #1 TX_use_ex <= 1;
                #2 TX_use_ex <= 0;
            end
        end
        
        instruction <= {5'd09,5'd0,12'd09,ADDI_INS_10};      //
        begin : SENDING_INS
                #1;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[7:0];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[15:8];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[23:16];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[31:24];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
        end
        
        instruction <= {5'd08,5'd0,12'd08,ADDI_INS_10};      //
        begin 
                #1;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[7:0];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[15:8];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[23:16];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[31:24];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
        end
        
        instruction <= {5'd07,5'd09,5'd08,MUL_INS_17};
        begin 
                #1;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[7:0];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[15:8];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[23:16];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[31:24];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
        end
        
        instruction <= {5'd10,5'd09,5'd08,ADD_INS_17};
        begin 
                #1;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[7:0];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[15:8];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[23:16];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[31:24];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
        end
        
        // Return first instrunction in MAIN PROGRAM
        instruction <= {-25'd16,JAL_INS_7};      //
        begin
                #1;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[7:0];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[15:8];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[23:16];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
            TX_use_ex <= 0;
            data_bus_in_tx_ex <= instruction[31:24];
            #1 TX_use_ex <= 1;
            #2 TX_use_ex <= 0;
        end
        
        // Finish program
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= FINISH_PROGRAM_OPCODE;
        #1 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
    end
endmodule
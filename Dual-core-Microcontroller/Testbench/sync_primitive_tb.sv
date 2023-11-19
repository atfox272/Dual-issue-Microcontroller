`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/28/2023 09:50:27 PM
// Design Name: 
// Module Name: sync_primitive_tb
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


module sync_primitive_tb;
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
    
    // Processor 1
    // Multi-processor Manager
    reg [INSTRUCTION_WIDTH - 1:0]   fetch_instruction_1;
    reg                             boot_processor_1;
    wire                            processor_idle_1;
    
    // Processor 2
    reg [INSTRUCTION_WIDTH - 1:0]   fetch_instruction_2;
    reg                             boot_processor_2;
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
    
    // Debug 
    wire    [63:0] debug_1;
    wire    [63:0] debug_2;
    
    // 
    wire    [DATA_WIDTH - 1:0]      registers_wire [0: DATA_MEMORY_SIZE - 1];
    wire [DOUBLEWORD_WIDTH - 1:0]   processor_registers_1 [0:REGISTER_AMOUNT - 1];
    wire [DOUBLEWORD_WIDTH - 1:0]   processor_registers_2 [0:REGISTER_AMOUNT - 1];
//    wire [DOUBLEWORD_WIDTH - 1:0]   registers_renew [0:REGISTER_AMOUNT - 1];
    
    
    // Multi-processor manager
    // Program memory
//    wire    [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd_pm;
//    wire                                rd_idle_pm;
//    wire    [ADDR_WIDTH_PM - 1:0]       addr_rd_pm;
//    wire                                rd_ins_pm;
    // Registers management
//    wire    [DOUBLEWORD_WIDTH - 1:0]    ra_register;
//    wire    [REG_SPACE_WIDTH - 1:0]     register_num;
//    wire                                boot_renew_register_1;
//    wire                                boot_renew_register_2;
//    wire                                main_program_state; 
//    wire                                synchronized_processors;
    // Interrupt control
//    wire                                interrupt_flag_1 = 1'b0;
//    wire                                interrupt_flag_2 = 1'b0;
//    wire                                interrupt_flag_3 = 1'b0;
    
    // Registers management
//    wire                                new_data_register       [0:REGISTER_AMOUNT - 1];
//    wire                                synchronization_processor_1;
//    wire                                synchronization_processor_2;
    
    wire[1:0]                       main_state;
    
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
//                    .data_bus_rd(data_bus_rd_pm),
//                    .rd_idle(rd_idle_pm),
//                    .addr_rd(addr_rd_pm),
//                    .rd_ins(rd_ins_pm),
//                    .data_type_rd(2'b10),
                    .rst_n(rst_n)
                    //Debug 
//                    ,.registers_wire(registers_wire)
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
                        .registers_renew(processor_registers_1),
//                        .synchronization_processor(synchronization_processor_1),
                        
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
                        .registers_renew(processor_registers_2),
//                        .synchronization_processor(synchronization_processor_2),
                        
                        .rst_n(rst_n)
                        
                        // Debug
                        ,.debug_2(debug_2)
                        );   
   
    Sync_primitive  #(
                    )sp(
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
                    ,.registers_wire(registers_wire)
                    );
                                             
    initial begin
        clk <= 0;
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 0;
        fetch_instruction_1 <= 0;
        boot_processor_1 <= 0;
        fetch_instruction_2 <= 0;
        boot_processor_2 <= 0;
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
    // Load 
    parameter LB_INS_10     = 10'b0000000011;       // LB:      <5-rd><5rs1><12-imm>
    parameter LW_INS_10     = 10'b0100000011;       // LW:      <5-rd><5rs1><12-imm>
    parameter LD_INS_10     = 10'b0110000011;       // LD:      <5-rd><5rs1><12-imm>
    // Store
    parameter SB_INS_10     = 10'b0000100011;       // SB:      <5-immh><5-rd><5rs1><7-imml>
    parameter SW_INS_10     = 10'b0100100011;       // SW:      <5-immh><5-rd><5rs1><7-imml>
    parameter SD_INS_10     = 10'b0110100011;       // SD:      <5-immh><5-rd><5rs1><7-imml>
    
    parameter J_INS_7       = 7'b1100111;           // J:       <25-imm>
    parameter JAL_INS_7     = 7'b1101111;           // J:       <25-imm>
    parameter JALR_INS_7    = 7'b1101011;           // J:       <25-imm>
        int i;
    reg [31:0] instruction;
    initial begin : FAKE_RPOGRAM_BLOCK
        
        #15;
        // Finish program
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= FINISH_PROGRAM_OPCODE;
        #1 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
        
        #1500;
        
        
        #10;
        fetch_instruction_1 <= {5'd09,5'd00,12'd09,ADDI_INS_10};    // x9 = x0 + 9
        boot_processor_1 <= 0;
        #1 boot_processor_1 <= 1;
        #2 boot_processor_1 <= 0;
        
        
        #10;
        fetch_instruction_1 <= {5'd00,5'd00,5'd09,7'd12,SB_INS_10}; // Store x9(9) to 12(x0)
        boot_processor_1 <= 0;
        #1 boot_processor_1 <= 1;
        #2 boot_processor_1 <= 0;
        
        #20;
        fetch_instruction_1 <= {5'd08,5'd00,12'd12,LB_INS_10};       // Load 12(x0) to x8
        boot_processor_1 <= 0;
        #1 boot_processor_1 <= 1;
        #2 boot_processor_1 <= 0;
        
        #150 $stop;
    end
    initial begin
        #15;
        // Finish program
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= FINISH_PROGRAM_OPCODE;
        #1 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
        
        #1500;
        
        
        #11;
        fetch_instruction_2 <= {5'd09,5'd00,12'd1023,ADDI_INS_10};    // x9 = x0 + 9
        boot_processor_2 <= 0;
        #1 boot_processor_2 <= 1;
        #2 boot_processor_2 <= 0;
        
        
        #11;
        fetch_instruction_2 <= {5'd00,5'd00,5'd09,7'd12,SW_INS_10}; // Store x9(9) to 12(x0)
        boot_processor_2 <= 0;
        #1 boot_processor_2 <= 1;
        #2 boot_processor_2 <= 0;
        
        #22;
        fetch_instruction_2 <= {5'd08,5'd00,12'd13,LB_INS_10};       // Load 12(x0) to x8
        boot_processor_2 <= 0;
        #1 boot_processor_2 <= 1;
        #2 boot_processor_2 <= 0;
        
        #150 $stop;
    end
endmodule

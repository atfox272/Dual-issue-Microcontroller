`timescale 1ns / 1ps
module processor_1_case2_tb;
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
    
    parameter REGISTER_AMOUNT        = 32;
    
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
    
    // Case 2 start 
    // Multi-processor Manager
    reg [INSTRUCTION_WIDTH - 1:0]   fetch_instruction;
    reg                             boot_processor;
    wire                            processor_idle;
    wire [31:0]                     new_data_register = {32{1'b1}};
    // Synchrization primitive 
    reg  [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd;
    wire [ADDR_WIDTH_DM - 1:0]      addr_rd;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd;
    reg                             rd_idle;
    wire                            rd_ins;
    reg                             rd_access;
    wire                            rd_finish;
    // Synchronization primitive (WRITE_STATE)
    wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr;
    wire [ADDR_WIDTH_DM - 1:0]      addr_wr;
    wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr;
    reg                             wr_idle;   
    wire                            wr_ins;
    reg                             wr_access;
    
    // Debug 
    wire    [63:0] debug_1;
//    wire    [DATA_WIDTH - 1:0]  registers_wire [0: PROGRAM_MEMORY_SIZE - 1];
    wire [DOUBLEWORD_WIDTH - 1:0]   processor_registers [0:REGISTER_AMOUNT - 1];
    wire [DOUBLEWORD_WIDTH - 1:0]   registers_renew [0:REGISTER_AMOUNT - 1];
    generate
    for(genvar i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
        assign registers_renew[i] = processor_registers[i];
    end
    endgenerate
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
                    .ADDR_DEPTH(PROGRAM_MEMORY_SIZE),
                    .RESERVED_REG_AMOUNT(1'b1)
                    )program_memory(
                    .clk(clk),
                    .data_bus_wr(data_bus_wr_pm),
                    .data_type_wr(BYTE_SIZE_ENCODE),    // Write byte (8bit)
                    .addr_wr(addr_wr_pm),
                    .wr_ins(wr_ins_pm),
                    .wr_idle(wr_idle_pm),
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
                        .fetch_instruction(fetch_instruction),
                        .boot_processor(boot_processor),
                        .processor_idle(processor_idle),
                        .new_data_register(new_data_register),
                        // Synchronization primitive
                        // - read
                        .data_bus_rd(data_bus_rd),
                        .addr_rd(addr_rd),
                        .data_type_rd(data_type_rd),
                        .rd_idle(rd_idle),
                        .rd_ins(rd_ins),
                        .rd_access(rd_access),
                        .rd_finish(rd_finish),
                        // - write
                        .data_bus_wr(data_bus_wr),
                        .addr_wr(addr_wr),
                        .data_type_wr(data_type_wr),
                        .wr_idle(wr_idle),
                        .wr_ins(wr_ins),
                        .wr_access(wr_access),
                        
                        .rst_n(rst_n)
                        
                        // Debug
                        ,.debug_1(debug_1)
                        ,.processor_registers(processor_registers)
                        ,.registers_renew(registers_renew)
                        );                
    
    initial begin
        clk <= 0;
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= 0;
        fetch_instruction <= 0;
        boot_processor <= 0;
        data_bus_rd <= 0;
        rd_idle <= 0;
        rd_access <= 0;
        wr_idle <= 0;
        wr_access <= 0;
        rst_n <= 1;
        #1 rst_n <= 0;
        #9 rst_n <= 1;
    end
    initial begin
        forever #1 clk <= ~clk;
    end
    
        int i;
        
    initial begin : FAKE_RPOGRAM_BLOCK
        #11;
        
        TX_use_ex <= 0;
        data_bus_in_tx_ex <= FINISH_PROGRAM_OPCODE;
        #1 TX_use_ex <= 1;
        #2 TX_use_ex <= 0;
        
    end
    

    parameter ADD_INS_17    = 17'b00000000000110011;// ADD:     <5-rd><5-rs1><5-rs2>
    parameter ADDI_INS_10   = 10'b0000010011;       // ADDI:    <5-rd><5rs1><12-imm>
    parameter SUB_INS_17    = 17'b10000000000110011;// SUB:     <5-rd><5-rs1><5-rs2>
    parameter SLL_INS_17    = 17'b00000000010110011;// SLL:     <5-rd><5-rs1><5-rs2>
    parameter SRL_INS_17    = 17'b00000001010110011;// SRL:     <5-rd><5-rs1><5-rs2>
    parameter MUL_INS_17    = 17'b00000010000110011;// MUL:     <5-rd><5-rs1><5-rs2>
    parameter LW_INS_10     = 10'b0100000011;       // LW:      <5-rd><5rs1><12-imm>
    
    initial begin
        #1250000;
        
        // ADD
        fetch_instruction <= {5'd4, 5'd0, 12'd30, ADDI_INS_10}; // x4 = x0 + 30 (x3 = 0)
        #2 boot_processor <= 1;
        #2 boot_processor <= 0;
        
        #20;
        
        // ADDI
        fetch_instruction <= {5'd5, 5'd4, 5'd4, ADD_INS_17};    // x5 = x4 + x4
        #2 boot_processor <= 1;
        #2 boot_processor <= 0;
        
        #20;
        
        // SUB
        fetch_instruction <= {5'd6, 5'd5, 5'd4, SUB_INS_17};    // x6 = x5 - x4
        #2 boot_processor <= 1;
        #2 boot_processor <= 0;
        
        #20;
        
        // ADDI
        fetch_instruction <= {5'd7, 5'd0, 12'd3, ADDI_INS_10};   // x7 = 0 + 3;
        #2 boot_processor <= 1;
        #2 boot_processor <= 0;
        
        #60;
        
        // SLL
        fetch_instruction <= {5'd8, 5'd4, 5'd7, SLL_INS_17};
        #2 boot_processor <= 1;
        #2 boot_processor <= 0;
        
        #20;
        
        // ADDI
        fetch_instruction <= {5'd9, 5'd0, 12'd4, ADDI_INS_10};   // x9 = 0 + 4;
        #2 boot_processor <= 1;
        #2 boot_processor <= 0;
        
        #60;
        
        // SRL
        fetch_instruction <= {5'd10, 5'd5, 5'd9, SRL_INS_17};    // x10 = x5(60) / 2^x9(4)
        #2 boot_processor <= 1;
        #2 boot_processor <= 0;
        
        #60;
        
        // MUL
        fetch_instruction <= {5'd11, 5'd8, 5'd10, MUL_INS_17};
        #2 boot_processor <= 1;
        #2 boot_processor <= 0;
        
        #60;
        
        // LW
        fetch_instruction <= {5'd12, 5'd11, 12'd100, LW_INS_10};   // x12 = data of (720 + 100);
        #2 boot_processor <= 1;
        #2 boot_processor <= 0;

        #60;
        data_bus_rd <= 64'd272203;
        #1 rd_access <= 1;
        #10 rd_idle <= 1;
        #2 rd_idle <= 0;
        #2 rd_access <= 0;
        
//        // ADDI
//        fetch_instruction <= {5'd4, 5'd4, 5'd4, ADD_INS_17};
//        #2 boot_processor <= 1;
//        #2 boot_processor <= 0;
        
        #60;
        $stop;
    end
endmodule

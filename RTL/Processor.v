`include "configuration.vh"
//`define DEBUG
module Processor
    #(
    parameter           MAIN_RPOCESSOR          = 1,            // Main processor - Sub processor
    
    parameter           DATA_WIDTH              = 8,
    parameter           BYTE_WIDTH              = 8,
    parameter           WORD_WIDTH              = 32,
    parameter           DOUBLEWORD_WIDTH        = 64,
    parameter           DATA_MEMORY_SIZE        = 1024,      // 256 bytes (2Kb)
    
    parameter           DATA_BUS_WIDTH          = 64,
    parameter           ADDR_BUS_WIDTH          = 64,
    
    parameter           ADDR_WIDTH_DM           = $clog2(DATA_MEMORY_SIZE),
    parameter           DATA_TYPE_WIDTH         = 2,
    
    parameter           INSTRUCTION_WIDTH       = 32,   //32-bit instruction
    parameter           PROGRAM_MEMORY_SIZE     = 1024,
    
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
    // Program memory
    parameter ADDR_WIDTH_PM         = $clog2(PROGRAM_MEMORY_SIZE),
    parameter START_WR_ADDR_PM      = 8'h00,
    // RAM's data_type encode
    parameter BYTE_TYPE_ENCODE      = 2'b00,
    parameter WORD_TYPE_ENCODE      = 2'b01,
    parameter DWORD_TYPE_ENCODE     = 2'b10,
    // Instruction Format 
    parameter OPCODE_SPACE_MSB      = 6,
    parameter OPCODE_SPACE_LSB      = 0,
    parameter OPCODE_SPACE_WIDTH    = OPCODE_SPACE_MSB - OPCODE_SPACE_LSB + 1,        
    parameter FUNCT10_SPACE_MSB     = 16,
    parameter FUNCT10_SPACE_LSB     = 7,
    parameter FUNCT10_SPACE_WIDTH   = FUNCT10_SPACE_MSB - FUNCT10_SPACE_LSB + 1,      
    parameter FUNCT3_SPACE_MSB      = 9,
    parameter FUNCT3_SPACE_LSB      = 7,
    parameter FUNCT3_SPACE_WIDTH    = FUNCT3_SPACE_MSB - FUNCT3_SPACE_LSB + 1,
    parameter RD_SPACE_MSB          = 31,
    parameter RD_SPACE_LSB          = 27,
    parameter RS1_SPACE_MSB         = 26,
    parameter RS1_SPACE_LSB         = 22,
    parameter RS2_SPACE_MSB         = 21,
    parameter RS2_SPACE_LSB         = 17,
    parameter REGISTER_SPACE_WIDTH  = 5,
    parameter IMM_1_SPACE_MSB       = 16,   
    parameter IMM_1_SPACE_LSB       = 10,
    parameter IMM_1_SPACE_WIDTH     = IMM_1_SPACE_MSB - IMM_1_SPACE_LSB + 1,
    parameter IMM_2_SPACE_MSB       = 21,
    parameter IMM_2_SPACE_LSB       = 17,
    parameter IMM_2_SPACE_WIDTH     = IMM_2_SPACE_MSB - IMM_2_SPACE_LSB + 1,
    parameter IMM_3_SPACE_MSB       = 31,
    parameter IMM_3_SPACE_LSB       = 27,
    parameter IMM_3_SPACE_WIDTH     = IMM_3_SPACE_MSB - IMM_3_SPACE_LSB + 1,
    parameter IMM_20_SPACE_MSB      = 26,
    parameter IMM_20_SPACE_LSB      = 7,
    parameter IMM_20_SPACE_WIDTH    = IMM_20_SPACE_MSB - IMM_20_SPACE_LSB + 1,
    parameter JUMP_OFS_SPACE_MSB    = 31,
    parameter JUMP_OFS_SPACE_LSB    = 7,
    parameter JUMP_OFS_SPACE_WIDTH  = JUMP_OFS_SPACE_MSB - JUMP_OFS_SPACE_LSB + 1,
    parameter OPT_SPACE_MSB         = 16,               // Option space
    parameter OPT_SPACE_LSB         = 10,               // Option space    
    parameter OPT_SPACE_WIDTH       = OPT_SPACE_MSB - OPT_SPACE_LSB + 1,
    parameter IMM_4_SPACE_MSB       = 16,               // Option space
    parameter IMM_4_SPACE_LSB       = 11,               // Option space    
    parameter IMM_4_SPACE_WIDTH     = IMM_4_SPACE_MSB - IMM_4_SPACE_LSB + 1,
    // Opcode encoder
    // -- Integer type
    parameter R_TYPE_ENCODE         = 7'b0110011,       // R-type
    parameter I_TYPE_ENCODE         = 7'b0010011,       // I-type
    parameter LUI_TYPE_ENCODE       = 7'b0110111,       // LUI-type  
    // -- Branching type 
    parameter J_TYPE_ENCODE         = 7'b1100111,       // J-type
    parameter JAL_TYPE_ENCODE       = 7'b1101111,       // J-type
    parameter JALR_TYPE_ENCODE      = 7'b1101011,       // J-type
    parameter B_TYPE_ENCODE         = 7'b1100011,       // B-type   
    // -- Data transfer type 
    parameter LOAD_ENCODE           = 7'b0000011,       // LOAD-type
    parameter STORE_ENCODE          = 7'b0100011,       // STORE-type
    // -- System type
    parameter MISC_MEM_ENCODE       = 7'b0101111,       // Hardware-supportive memory type
    parameter PROTOCOL_ENCODE       = 7'b1000001,       // Protocol-type
    parameter GPIO_ENCODE           = 7'b1110101,       // GPIO-type
    parameter SYSTEM_ENCODE         = 7'b1110111,       // System-type
    // Funct3 enoder (for B-type)
    parameter BEQ_ENCODE           = 3'b000,
    parameter BNE_ENCODE           = 3'b001,
    parameter BLT_ENCODE           = 3'b100,
    parameter BGE_ENCODE           = 3'b101,
    // Funct3 encoder (for SYSTEM-type) 
    parameter BREAK_ENCODE         = 3'b001,            // BREAK: use for debugger
    parameter DEBUG_ENCODE         = 3'b101,
    parameter EXIT_ENCODE          = 3'b010,
    parameter RETI_ENCODE          = 3'b011,
    // Funct3 encoder (for MISC-MEM type)
    parameter FENCE_ENCODE         = 3'b010,
    // Funct3 encoder (for Protocol-type)
    parameter UART_ENCODE          = 3'b000,
    parameter SPI_ENCODE           = 3'b010,
    parameter I2C_ENCODE           = 3'b011,
    // Funct10 encoder
    parameter ADD_ENCODE            = 10'b0000000000,   
    parameter SUB_ENCODE            = 10'b1000000000,
    parameter SRL_ENCODE            = 10'b0000000101,   // Shift right logical
    parameter SLL_ENCODE            = 10'b0000000001,   // Shift left logical
    parameter SLT_ENCODE            = 10'b0000000010,   // Set less than 
    parameter SLTU_ENCODE           = 10'b0000000011,   // Set less than unsigned
    parameter AND_ENCODE            = 10'b0000000111,
    parameter XOR_ENCODE            = 10'b0000000100,
    parameter OR_ENCODE             = 10'b0000000110,
    parameter MUL_ENCODE            = 10'b0000001000,
    // Funct3 encoder (for I-type)
    parameter ADDI_ENCODE           = 3'b000,
    parameter SLLI_ENCODE           = 3'b001,
    parameter SRLI_ENCODE           = 3'b101,
    parameter SLTI_ENCODE           = 3'b010,
    parameter SLTIU_ENCODE          = 3'b011,
    parameter XORI_ENCODE           = 3'b100,
    parameter ORI_ENCODE            = 3'b110,
    parameter ANDI_ENCODE           = 3'b111,
    // Funct3 encoder (for LOAD-type & STORE-type) 
    parameter BYTE_WIDTH_ENCODE     = 3'b000,  
    parameter WORD_WIDTH_ENCODE     = 3'b010,  
    parameter DWORD_WIDTH_ENCODE    = 3'b011,
    // Funct3 encoder (for I/O type)
    parameter READ_GPIO_ENCODE      = 3'b000,
    parameter WRITE_GPIO_ENCODE     = 3'b001,
    // Option encode (for Protocol type)
    parameter RECEIVE_PACKET_ENCODE = 7'b0000000,
    parameter LOAD_PACKET_H_ENCODE  = 7'b0000010,
    parameter LOAD_PACKET_L_ENCODE  = 7'b0000011,
    parameter TRANS_PACKET_L_ENCODE = 7'b1000000,       // Note: Transmit encode always have HIGH MSB (MSB == 1) (mpm detect this difference)
    // Operator decode (Level_2 - Instruction decode - Operator decode)
    parameter EXECUTION_1CYCLE             = 0,
    parameter EXECUTION_MULTI_CYCLE        = 1,
    parameter EXECUTION_1CYCLE_IMM         = 2,
    parameter EXECUTION_1CYCLE_LOAD_UPPER  = 3,
    parameter EXECUTION_MULTI_CYCLE_IMM    = 4,
    parameter EXECUTION_1CYCLE_ADDR_LOAD   = 5,
    parameter EXECUTION_1CYCLE_ADDR_STORE  = 6,
    parameter EXECUTION_1CYCLE_SYSTEM      = 7,
    
    // Finish program condition 
    parameter FINISH_PROGRAM_OPCODE  = 7'b0001011,      // Detect finish_program_opcode (this opcode is RESERVED)
    parameter FINISH_PROGRAM_TIMER   = 125000,          // Counting time of EMPTY RX_module (assume 1ms)
    
    // Protocol peripheral address encoder
    parameter ADDR_MAPPING_PERIPHERAL   = 5, 
    parameter NOTHING_MAPPING           = 0, 
    parameter UART_TX_MAPPING           = 1, 
    parameter UART_RX_MAPPING           = 2, 
    parameter SPI_MAPPING               = 3, 
    parameter I2C_MAPPING               = 4, 
    parameter ADDDR_MAPPING_WIDTH       = $clog2(ADDR_MAPPING_PERIPHERAL),
    // Protocol peripheral communication 
    parameter AMOUNT_SND_BYTE           = 16,  
    parameter AMOUNT_RCV_BYTE           = 16,
    parameter AMOUNT_SND_WIDTH          = $clog2(AMOUNT_SND_BYTE),
    parameter AMOUNT_RCV_WIDTH          = $clog2(AMOUNT_RCV_BYTE),
    parameter SIGN_EXTEND_WIDTH         = 52
    )(
    input clk,
    
    // Multi-processor Manager
    input   wire [INSTRUCTION_WIDTH - 1:0]  fetch_instruction,
    input   wire                            boot_processor,
    output  wire                            processor_idle,    
    
    // Synchronization primitive (COMMON)
    output  logic[ADDR_BUS_WIDTH - 1:0]     addr_dmem,
    output  logic[DATA_TYPE_WIDTH - 1:0]    data_type_dmem,
    // Synchronization primitive (READ_STATE)
    input   wire [DATA_BUS_WIDTH - 1:0]     data_bus_rd,
//    output  wire [ADDR_WIDTH_DM - 1:0]      addr_rd,
//    output  wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd,
    input   wire                            rd_idle,   
    output  wire                            rd_ins, 
    input   wire                            rd_access,
    // Synchronization primitive (WRITE_STATE)
    output  wire [DATA_BUS_WIDTH - 1:0]     data_bus_wr,
//    output  wire [ADDR_WIDTH_DM - 1:0]      addr_wr,
//    output  wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr,
    input   wire                            wr_idle,   
    output  wire                            wr_ins, 
    input   wire                            wr_access,
    
    // For main Processor ///////////////////////////////////////////////
    // UART_RX_1 (Use for stored-program)
    input   wire [DATA_WIDTH - 1:0]         data_bus_out_uart_1,
    output  wire                            RX_use_1,
    input   wire                            RX_flag_1,
    // Program memory
    output  wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_pm,
    input   wire                            wr_idle_pm,
    output  wire    [ADDR_WIDTH_PM - 1:0]   addr_wr_pm,
    output  wire                            wr_ins_pm,
    // Common 
    output  wire    [1:0]                   main_state,
    //////////////////////////////////////////////////////////////////////
    
    //////////////////////////////////////////////////////////////////////
    // Register management
    output  wire [DOUBLEWORD_WIDTH - 1:0]   processor_registers [0:REGISTER_AMOUNT - 1],
    input   wire [DOUBLEWORD_WIDTH - 1:0]   registers_renew     [0:REGISTER_AMOUNT - 1],
    input   wire                            synchronization_processor,
    
    input rst_n
    
    `ifdef DEBUG
    // Debug 
//    ,output wire    [63:0]                      debug_1
//    ,output wire    [63:0]                      debug_2
    `endif
    );
    generate
    if(MAIN_RPOCESSOR == 1) begin   : MAIN_RPOCESSOR_BLOCK
    
    // Declare CPU registers
    reg [DOUBLEWORD_WIDTH - 1:0] registers_owner    [0:REGISTER_AMOUNT - 1];
    wire[DOUBLEWORD_WIDTH - 1:0] rs1_regFile;
    wire[DOUBLEWORD_WIDTH - 1:0] rs2_regFile;
    wire[DOUBLEWORD_WIDTH - 1:0] rd_regFile;
    // Declare wires and states
    reg       RX_use_1_reg;
    reg [1:0] _4byte_counter;         // Overflow at 4
    wire      finish_program_condition;
    wire      finish_program_condition_1;
    wire      finish_program_condition_2;
    wire      start_counting_finish_program_condition;
    // Program Memory 
    // Store-Program state
    reg [DATA_WIDTH - 1:0]          data_bus_wr_pm_reg;
    reg [ADDR_WIDTH_PM - 1:0]       addr_wr_pm_reg;
    reg                             wr_ins_pm_reg;
    // Decode state
    wire[OPCODE_SPACE_WIDTH - 1:0]      opcode_space;
    wire[FUNCT10_SPACE_WIDTH - 1:0]     funct10_space;
    wire[FUNCT3_SPACE_WIDTH - 1:0]      funct3_space;
    wire[REGISTER_SPACE_WIDTH - 1:0]    rd_space;
    wire[REGISTER_SPACE_WIDTH - 1:0]    rs1_space;
    wire[REGISTER_SPACE_WIDTH - 1:0]    rs2_space;
    wire[IMM_1_SPACE_WIDTH - 1:0]       immediate_1_space;
    wire[IMM_2_SPACE_WIDTH - 1:0]       immediate_2_space;
    wire[IMM_3_SPACE_WIDTH - 1:0]       immediate_3_space;
    wire[IMM_20_SPACE_WIDTH - 1:0]      immediate_20_space;  // immediate20
//    reg [DOUBLEWORD_WIDTH - 1:0]        rs1_buffer; 
//    reg [DOUBLEWORD_WIDTH - 1:0]        rs2_buffer; 
    logic[SIGN_EXTEND_WIDTH - 1:0]      alu_sign_extend_imm2;
    logic[SIGN_EXTEND_WIDTH - 1:0]      alu_sign_extend_imm3;
    logic[DOUBLEWORD_WIDTH - 1:0]       data_bus_rd_sign_extend;
    logic[WORD_WIDTH - 1:0]             register_sign_extend_word;
    logic[BYTE_WIDTH*7 - 1:0]           register_sign_extend_7byte;
    // Operator decode (level_2)
    // Instruction  \----------- R-type  ------   1cycle     -------- 0 
    //                                 \------ multi-cycle   -------- 1
    //              \----------- I-type  -----    1cycle     -------- 2
    //                                 \------ multi-cycle   -------- 3
    //              \----------- DMEM   -----  LOAD_calAddr  -------- 4  
    //                                 \----   STORE_calAddr -------- 5
    logic[2:0]                          operator_decode;
    // Synchronization primitive
    //  - read 
    reg  [ADDR_BUS_WIDTH - 1:0]         addr_rd_reg;
    logic[DATA_TYPE_WIDTH - 1:0]        data_type_rd_logic;   
    reg                                 rd_ins_reg;
    //  - write
    reg  [DOUBLEWORD_WIDTH - 1:0]       data_bus_wr_reg;
    reg  [ADDR_BUS_WIDTH - 1:0]         addr_wr_reg;
    logic[DATA_TYPE_WIDTH - 1:0]        data_type_wr_logic;
    reg                                 wr_ins_reg;
    
    // ALU block
    logic[DOUBLEWORD_WIDTH - 1:0]       alu_operand_1; 
    logic[DOUBLEWORD_WIDTH - 1:0]       alu_operand_2; 
    logic[DOUBLEWORD_WIDTH - 1:0]       alu_result_1cycle; 
    logic[DOUBLEWORD_WIDTH - 1:0]       alu_result_multi_cycle; 
    logic[OPCODE_ALU_WIDTH - 1:0]       alu_opcode;
    logic                               alu_enable_comb;
    logic                               alu_enable_seq;
    logic                               multi_cycle_type;
    logic                               lui_type;
    wire                                alu_idle;
    // GPIO 
    wire                                GPIO_PORT_A_mapping;
    wire                                GPIO_PORT_B_mapping;
    wire                                GPIO_PORT_C_mapping;
    // Debug UART 
    reg                                 send_debug_clk_reg;
    
    reg [1:0] main_state_reg;
    reg [1:0] stored_program_state_reg;
    reg [2:0] running_program_state_reg;      
    // Main state encoder
    localparam IDLE_STATE                   = 2'd0;
    localparam STORE_PROGRAM_STATE          = 2'd1;
    localparam RUNNING_PROGRAM_STATE        = 2'd2;
    // Store program state
    localparam RD_FIFO_STATE                = 2'd1;
    localparam WR_RAM_STATE                 = 2'd2;
    localparam FINISH_PROGRAMMING_STATE     = 2'd3;
    // Running program state 
    localparam EXECUTE_INSTRUCTION_STATE    = 3'd0;
    localparam WRITE_BACK_STATE             = 3'd2;
    localparam DMEM_RD_ACCESS_STATE         = 3'd3;
    localparam DMEM_WR_ACCESS_STATE         = 3'd5;
    localparam SYSTEM_ACCESS_STATE          = 3'd4;
    localparam LOAD_REG_STATE               = 3'd1;
    
    assign main_state = main_state_reg;
    assign RX_use_1 = RX_use_1_reg;
    // Finish program condition
    assign finish_program_condition = finish_program_condition_1 | finish_program_condition_2;
    assign finish_program_condition_1 = (data_bus_out_uart_1[6:0] == FINISH_PROGRAM_OPCODE & RX_flag_1 == 1) & (_4byte_counter == 0);   // (_4byte_counter == 0) <=> Opdcode location
    assign start_counting_finish_program_condition = (main_state_reg == STORE_PROGRAM_STATE) & (RX_flag_1 == 0);        // MCU in STORE_PROGRAM_STATE & RX_module is empty 
    // Program Memory 
    assign data_bus_wr_pm = {{56{1'b0}}, data_bus_wr_pm_reg};
    assign addr_wr_pm = addr_wr_pm_reg;
    assign wr_ins_pm = wr_ins_pm_reg;
    // Synchronization primitive
    // -- Common
    always_comb begin
    if(opcode_space == LOAD_ENCODE) begin
        addr_dmem = addr_rd_reg;
        data_type_dmem = data_type_rd_logic;
    end
    else begin
        addr_dmem = addr_wr_reg;
        data_type_dmem = data_type_wr_logic;
    end
    end
    // -- read
//    assign data_type_rd = data_type_rd_logic;
    assign rd_ins = rd_ins_reg;
    assign register_sign_extend_word  = {32{data_bus_rd[WORD_WIDTH - 1]}};
    assign register_sign_extend_7byte = {56{data_bus_rd[BYTE_WIDTH - 1]}};
    always_comb begin
        case(funct3_space) 
            BYTE_WIDTH_ENCODE: begin
                data_bus_rd_sign_extend = {register_sign_extend_7byte, data_bus_rd[BYTE_WIDTH - 1:0]};
                data_type_rd_logic = BYTE_TYPE_ENCODE;
            end
            WORD_WIDTH_ENCODE: begin
                data_bus_rd_sign_extend = {register_sign_extend_word, data_bus_rd[WORD_WIDTH - 1:0]};
                data_type_rd_logic = WORD_TYPE_ENCODE;
            end
            DWORD_WIDTH_ENCODE: begin 
                data_bus_rd_sign_extend = data_bus_rd[DOUBLEWORD_WIDTH - 1:0];
                data_type_rd_logic = DWORD_TYPE_ENCODE;
            end
            default: begin
                data_bus_rd_sign_extend = data_bus_rd[DOUBLEWORD_WIDTH - 1:0];
                data_type_rd_logic = BYTE_TYPE_ENCODE;
            end
        endcase
    end
    // -- write
    assign wr_ins = wr_ins_reg;
    assign data_bus_wr = data_bus_wr_reg;
//    assign addr_wr = addr_wr_reg;
//    assign data_type_wr = data_type_wr_logic;
    always_comb begin
        case(funct3_space) 
            BYTE_WIDTH_ENCODE: data_type_wr_logic = BYTE_TYPE_ENCODE;
            WORD_WIDTH_ENCODE: data_type_wr_logic = WORD_TYPE_ENCODE;
            DWORD_WIDTH_ENCODE:data_type_wr_logic = DWORD_TYPE_ENCODE;
            default: data_type_wr_logic = BYTE_TYPE_ENCODE;
        endcase
    end
    // Decode instruciton
    assign opcode_space = fetch_instruction[OPCODE_SPACE_MSB:OPCODE_SPACE_LSB];
    assign funct10_space = fetch_instruction[FUNCT10_SPACE_MSB:FUNCT10_SPACE_LSB];
    assign funct3_space = fetch_instruction[FUNCT3_SPACE_MSB:FUNCT3_SPACE_LSB];
    assign rd_space = fetch_instruction[RD_SPACE_MSB:RD_SPACE_LSB];
    assign rs1_space = fetch_instruction[RS1_SPACE_MSB:RS1_SPACE_LSB];
    assign rs2_space = fetch_instruction[RS2_SPACE_MSB:RS2_SPACE_LSB];
    assign immediate_1_space = fetch_instruction[IMM_1_SPACE_MSB:IMM_1_SPACE_LSB];
    assign immediate_2_space = fetch_instruction[IMM_2_SPACE_MSB:IMM_2_SPACE_LSB];
    assign immediate_3_space = fetch_instruction[IMM_3_SPACE_MSB:IMM_3_SPACE_LSB];
    assign immediate_20_space= fetch_instruction[IMM_20_SPACE_MSB:IMM_20_SPACE_LSB];
    // Multi-processor manager
    assign processor_idle = (running_program_state_reg == EXECUTE_INSTRUCTION_STATE);
    // Register management
    assign rs1_regFile = registers_renew[rs1_space];
    assign rd_regFile  = registers_renew[rd_space];
    assign rs2_regFile = registers_renew[rs2_space];
    for(genvar index_register = 0; index_register < REGISTER_AMOUNT; index_register = index_register + 1) begin
        assign processor_registers[index_register] = registers_owner[index_register];
    end
    // Debug UART
    assign send_debug_clk = send_debug_clk_reg;
    assign amount_snd_byte_debug = {{AMOUNT_SND_WIDTH{1'b1}}};  // Always 128bit
    assign data_snd_debug = {rd_regFile, rs1_regFile};    
    // ALU block
    assign alu_sign_extend_imm2 = {52{fetch_instruction[IMM_2_SPACE_MSB]}}; 
    assign alu_sign_extend_imm3 = {52{fetch_instruction[IMM_3_SPACE_MSB]}};
    
    logic [3:0]                     exec_state;
    logic                           wb_immediate_en;
    logic [DOUBLEWORD_WIDTH - 1:0]  wb_immediate_value;
    always_comb begin
        case(operator_decode) 
            EXECUTION_1CYCLE: begin    
                exec_state = EXECUTE_INSTRUCTION_STATE;
                // ALU control
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = rs2_regFile;
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 1;
                wb_immediate_value = alu_result_1cycle;
            end
            EXECUTION_MULTI_CYCLE: begin   
                exec_state = LOAD_REG_STATE;
                // ALU control 
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = rs2_regFile;
                alu_enable_comb = 0;
                multi_cycle_type = 1;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
            EXECUTION_1CYCLE_IMM: begin    
                exec_state = EXECUTE_INSTRUCTION_STATE;
                // ALU control
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = {alu_sign_extend_imm2, immediate_2_space, immediate_1_space};
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 1;
                wb_immediate_value = alu_result_1cycle;
            end
            EXECUTION_MULTI_CYCLE_IMM: begin  
                exec_state = LOAD_REG_STATE;
                // ALU control  
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = {alu_sign_extend_imm2, immediate_2_space, immediate_1_space};
                alu_enable_comb = 0;
                multi_cycle_type = 1;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
            EXECUTION_1CYCLE_LOAD_UPPER: begin
                exec_state = EXECUTE_INSTRUCTION_STATE;
                // ALU control
                alu_operand_1 = 64'h00;
                alu_operand_2 = 64'h00;
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 1;
                wb_immediate_value = {immediate_20_space, {12{1'b0}}, {32{1'b0}}};
            end
            EXECUTION_1CYCLE_ADDR_LOAD: begin 
                exec_state = DMEM_RD_ACCESS_STATE;
                // ALU control   
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = {alu_sign_extend_imm2, immediate_2_space, immediate_1_space};
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
            EXECUTION_1CYCLE_ADDR_STORE: begin   
                exec_state = DMEM_WR_ACCESS_STATE;
                // ALU control    
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = {alu_sign_extend_imm3, immediate_3_space, immediate_1_space};
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
            EXECUTION_1CYCLE_SYSTEM: begin
                exec_state = SYSTEM_ACCESS_STATE;
                // ALU control    
                alu_operand_1 = 0;
                alu_operand_2 = 0;
                alu_enable_comb = 0;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
            default: begin
                exec_state = EXECUTE_INSTRUCTION_STATE;
                // ALU control   
                alu_operand_1 = 64'h00;
                alu_operand_2 = 64'h00;
                alu_enable_comb = 0;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
        endcase
    end
    always_comb begin
        case(opcode_space) 
            R_TYPE_ENCODE: begin    
                case(funct10_space) 
                    ADD_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = ADD_ALU_ENCODE;
                    end
                    SUB_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = SUB_ALU_ENCODE;
                    end
                    AND_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = AND_ALU_ENCODE;
                    end
                    XOR_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = XOR_ALU_ENCODE;
                    end
                    OR_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = OR_ALU_ENCODE;
                    end
                    SLT_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = SLT_ALU_ENCODE;
                    end
                    SRL_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE;
                        alu_opcode = SRL_ALU_ENCODE;
                    end
                    SLL_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE;
                        alu_opcode = SLL_ALU_ENCODE;
                    end
                    MUL_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE;
                        alu_opcode = MUL_ALU_ENCODE;
                    end
                    default: begin
                        operator_decode = 0;
                        alu_opcode = 0;
                    end
                endcase
            end
            I_TYPE_ENCODE: begin    
                case(funct3_space) 
                    ADDI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = ADD_ALU_ENCODE;
                    end
                    ANDI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = AND_ALU_ENCODE;
                    end
                    XORI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = XOR_ALU_ENCODE;
                    end
                    ORI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = OR_ALU_ENCODE;
                    end
                    SLTI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = SLT_ALU_ENCODE;
                    end
                    SRLI_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE_IMM;
                        alu_opcode = SRL_ALU_ENCODE;
                    end
                    SLLI_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE_IMM;
                        alu_opcode = SLL_ALU_ENCODE;
                    end
                    default: begin
                        operator_decode = 0;
                        alu_opcode = 0;
                    end
                endcase
            end
            LUI_TYPE_ENCODE: begin
                operator_decode = EXECUTION_1CYCLE_LOAD_UPPER;
                alu_opcode = 0;
            end
            LOAD_ENCODE: begin      // result: address of target -> access Data memory to write value
                operator_decode = EXECUTION_1CYCLE_ADDR_LOAD;
                alu_opcode = ADD_ALU_ENCODE;
            end
            STORE_ENCODE: begin
                operator_decode = EXECUTION_1CYCLE_ADDR_STORE;
                alu_opcode = ADD_ALU_ENCODE;
            end
            SYSTEM_ENCODE: begin
                operator_decode = EXECUTION_1CYCLE_SYSTEM;
                alu_opcode = 0;
            end
            default: begin
                operator_decode = 0;
                alu_opcode = 0;
            end
        endcase
    end
    
    logic [ADDR_BUS_WIDTH - 1:0]    dmem_addr_rd;
    logic [ADDR_BUS_WIDTH - 1:0]    dmem_addr_wr;
    logic                           dmem_ins_rd_en;
    logic                           dmem_ins_wr_en;
    logic [DOUBLEWORD_WIDTH - 1:0]  dmem_data_wr;
    always_comb begin
        if(opcode_space == LOAD_ENCODE) begin
            dmem_addr_rd = alu_result_1cycle;
            dmem_ins_rd_en = 1;
        end
        else begin
            dmem_addr_rd = addr_rd_reg;
            dmem_ins_rd_en = 0;
        end
       
        if(opcode_space == STORE_ENCODE) begin
            dmem_addr_wr = alu_result_1cycle;
            dmem_ins_wr_en = 1;
            dmem_data_wr = rs2_regFile;
        end
        else begin
            dmem_addr_wr = addr_wr_reg;
            dmem_ins_wr_en = 0;
            dmem_data_wr = data_bus_wr_reg; // Don't care (not modify register)
        end
    end
    alu         #(
                .OPERAND_WIDTH(DOUBLEWORD_WIDTH)
                )
    alu         (
                .clk(clk),
                .operand_1(alu_operand_1),
                .operand_2(alu_operand_2),
                .result_1cycle(alu_result_1cycle),
                .result_multi_cycle(alu_result_multi_cycle),
                .op_code(alu_opcode),
                .alu_enable_comb(alu_enable_comb),
                .alu_enable_seq(alu_enable_seq),
                .alu_idle(alu_idle),
//                .revert_result(),
                .overflow_flag(),
                .invalid_flag(),
                .rst_n(rst_n)
                ); 
                
    // Timer for finishing program (time out)
    real_time 
        #(
        .MAX_COUNTER(FINISH_PROGRAM_TIMER)
        ) programming_timeout (
        .clk(clk),
        .counter_enable(start_counting_finish_program_condition),
        .limit_counter(FINISH_PROGRAM_TIMER),
        .limit_flag(finish_program_condition_2),
        .rst_n(rst_n)
        );
//    waiting_module #(
//                .END_COUNTER(FINISH_PROGRAM_TIMER),
//                .WAITING_TYPE(0),
//                .LEVEL_PULSE(1)
//                )timout_programming(
//                .clk(clk),
//                .start_counting(start_counting_finish_program_condition),
//                .reach_limit(finish_program_condition_2),
//                .stop_counting(),
//                .rst_counting(),
//                .rst_n(rst_n)
//                );             
                
    // Main state 
    logic [1:0]                 main_state_n;
    always_comb begin           : MAIN_FSM_GENERATOR
        main_state_n = main_state_reg;
        case(main_state_reg)
            IDLE_STATE: main_state_n = (RX_flag_1) ? STORE_PROGRAM_STATE : main_state_reg;
            STORE_PROGRAM_STATE: main_state_n = (stored_program_state_reg == FINISH_PROGRAMMING_STATE) ? RUNNING_PROGRAM_STATE : main_state_reg;
            RUNNING_PROGRAM_STATE: main_state_n = main_state_reg;
        endcase
    end 
    
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            main_state_reg <= IDLE_STATE;
        end
        else main_state_reg <= main_state_n;
    end
    
    logic [1:0]                 stored_program_state_n;
    logic                       RX_use_1_n;
    logic [1:0]                 _4byte_counter_n;
    logic [DATA_WIDTH - 1:0]    data_bus_wr_pm_n;
    logic [ADDR_WIDTH_PM - 1:0] addr_wr_pm_n;
    logic                       wr_ins_pm_n;
    always_comb begin           : STORE_PROGRAM_FSM_GENERATOR
        stored_program_state_n = stored_program_state_reg;
        RX_use_1_n = RX_use_1_reg;
        _4byte_counter_n = _4byte_counter;
        data_bus_wr_pm_n = data_bus_wr_pm_reg;
        addr_wr_pm_n = addr_wr_pm_reg;
        wr_ins_pm_n = wr_ins_pm_reg;
        case(stored_program_state_reg)
            IDLE_STATE: begin
                if(finish_program_condition) begin
                    stored_program_state_n = FINISH_PROGRAMMING_STATE;
                    RX_use_1_n = (RX_flag_1 == 1) ? 1'b1 : 1'b0;   // Clear FIFO
                end
                else begin
                    stored_program_state_n = RD_FIFO_STATE;
                    data_bus_wr_pm_n = data_bus_out_uart_1;
                    RX_use_1_n = 1'b1;
                    _4byte_counter_n = _4byte_counter + 1;
                end
            end
            RD_FIFO_STATE: begin
                RX_use_1_n = 0;
                if(wr_idle_pm) begin
                    stored_program_state_n = WR_RAM_STATE;
                    wr_ins_pm_n = 1'b1;
                end
            end
            WR_RAM_STATE: begin
                wr_ins_pm_n = 0;
                if(finish_program_condition) begin              // Finish program
                    stored_program_state_n = FINISH_PROGRAMMING_STATE;
                    RX_use_1_n = (RX_flag_1 == 1) ? 1'b1 : 1'b0;   // Clear FIFO
                end    
                else if(RX_flag_1) begin
                    stored_program_state_n = RD_FIFO_STATE;
                    // UART signal
                    RX_use_1_n = 1;
                    // Program memory signal
                    data_bus_wr_pm_n = data_bus_out_uart_1;
                    addr_wr_pm_n = addr_wr_pm_reg + 1'b1;
                    _4byte_counter_n = _4byte_counter + 1'b1;
                end
            end
            FINISH_PROGRAMMING_STATE: begin
                stored_program_state_n = IDLE_STATE;
                RX_use_1_n = 1'b0;
            end
        endcase
    end
    // Store program FSM
    always @(posedge clk) begin
        if(!rst_n) begin
            stored_program_state_reg <= IDLE_STATE;
            // Reset buffer
            RX_use_1_reg <= 0;
            _4byte_counter <= 0;
            // Reset buffer interact with Program-memory
            data_bus_wr_pm_reg <= 8'h00;
            addr_wr_pm_reg <= START_WR_ADDR_PM;
            wr_ins_pm_reg <= 0;
        end
        else if(main_state_reg == STORE_PROGRAM_STATE) begin
            stored_program_state_reg <= stored_program_state_n;
            RX_use_1_reg <= RX_use_1_n;
            _4byte_counter <= _4byte_counter_n;
            data_bus_wr_pm_reg <= data_bus_wr_pm_n;
            addr_wr_pm_reg <= addr_wr_pm_n;
            wr_ins_pm_reg <= wr_ins_pm_n;
        end
    end
    logic [2:0]                     running_program_state_n;
    logic                           alu_enable_seq_n;
    logic [DOUBLEWORD_WIDTH - 1:0]  registers_owner_n    [0:REGISTER_AMOUNT - 1];
    logic [ADDR_BUS_WIDTH - 1:0]    addr_rd_n;
    logic [ADDR_BUS_WIDTH - 1:0]    addr_wr_n;
    logic                           rd_ins_n;
    logic                           wr_ins_n;
    logic [DOUBLEWORD_WIDTH - 1:0]  data_bus_wr_n;
    always_comb begin               : RUNNING_PROGRAM_FSM_GENERATOR
        running_program_state_n = running_program_state_reg;
        alu_enable_seq_n = alu_enable_seq;// Reset register in processor
        for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
        registers_owner_n[i] = registers_owner[i];
        end
        addr_rd_n = addr_rd_reg;
        addr_wr_n = addr_wr_reg;
        rd_ins_n = rd_ins_reg;
        wr_ins_n = wr_ins_reg;
        data_bus_wr_n = data_bus_wr_reg;
        
        case(running_program_state_reg) 
            EXECUTE_INSTRUCTION_STATE: begin
                if(boot_processor) begin
                    running_program_state_n = exec_state;
                    alu_enable_seq_n = (multi_cycle_type) ? 1'b1 : 1'b0;
                    if(wb_immediate_en) registers_owner_n[rd_space] = wb_immediate_value;
                    // DMEM ACESS
                    addr_rd_n = dmem_addr_rd;
                    addr_wr_n = dmem_addr_wr;
                    rd_ins_n = dmem_ins_rd_en;
                    wr_ins_n = dmem_ins_wr_en;
                    data_bus_wr_n = dmem_data_wr;
                end
                else if(synchronization_processor) begin
                    for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) registers_owner_n[i] = registers_renew[i];
                end
            end
            WRITE_BACK_STATE: begin
                if(alu_idle == 1) begin
                    registers_owner_n[rd_space] = alu_result_multi_cycle;
                    running_program_state_n = EXECUTE_INSTRUCTION_STATE;
                end
            end
            DMEM_RD_ACCESS_STATE: begin
                if(rd_idle) begin
                    running_program_state_n = EXECUTE_INSTRUCTION_STATE;
                    registers_owner_n[rd_space] = data_bus_rd_sign_extend;
                    rd_ins_n = 0;
                end
            end
            DMEM_WR_ACCESS_STATE: begin
                if(wr_access) begin // Write Memory completely
                    running_program_state_n = EXECUTE_INSTRUCTION_STATE;
                    wr_ins_n = 0;
                end
            end
            SYSTEM_ACCESS_STATE: begin
            
            end
            LOAD_REG_STATE: begin
                running_program_state_n = WRITE_BACK_STATE;
            end
            default: begin
                running_program_state_n = EXECUTE_INSTRUCTION_STATE;
            end        
        endcase
    end
    // Critical path: 
    always @(posedge clk) begin
        if(!rst_n) begin
            running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
            alu_enable_seq <= 0;// Reset register in processor
            for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
                registers_owner[i] <= REGISTER_DEFAULT[i];
            end
            addr_rd_reg <= 0;
            addr_wr_reg <= 0;
            rd_ins_reg <= 0;
            wr_ins_reg <= 0;
            data_bus_wr_reg <= 0;
        end
        else if(main_state_reg == RUNNING_PROGRAM_STATE) begin
            running_program_state_reg <= running_program_state_n;
            alu_enable_seq <= alu_enable_seq_n;// Reset register in processor
            for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
            registers_owner[i] <= registers_owner_n[i];
            end
            addr_rd_reg <= addr_rd_n;
            addr_wr_reg <= addr_wr_n;
            rd_ins_reg <= rd_ins_n;
            wr_ins_reg <= wr_ins_n;
            data_bus_wr_reg <= data_bus_wr_n;
            
        end
    end
    `ifdef DEBUG
    // Debug area
//    assign debug_1 = {32'b0, alu_result_1cycle};
    `endif
    end
    else begin                      : SUB_PROCESSOR_BLOCK
    reg [DOUBLEWORD_WIDTH - 1:0]        registers_owner    [0:REGISTER_AMOUNT - 1];
    wire[DOUBLEWORD_WIDTH - 1:0]        rs1_regFile;
    wire[DOUBLEWORD_WIDTH - 1:0]        rs2_regFile;
    wire[DOUBLEWORD_WIDTH - 1:0]        rs3_regFile;
    wire[DOUBLEWORD_WIDTH - 1:0]        rd_regFile;
    wire[DOUBLEWORD_WIDTH - 1:0]        rd2_regFile;
    // Execute-program state
    wire[OPCODE_SPACE_WIDTH - 1:0]      opcode_space;
    wire[FUNCT10_SPACE_WIDTH - 1:0]     funct10_space;
    wire[FUNCT3_SPACE_WIDTH - 1:0]      funct3_space;
    wire[REGISTER_SPACE_WIDTH - 1:0]    rd_space;
    wire[REGISTER_SPACE_WIDTH - 1:0]    rs1_space;
    wire[REGISTER_SPACE_WIDTH - 1:0]    rs2_space;
    wire[REGISTER_SPACE_WIDTH - 1:0]    rs3_space;
    wire[IMM_1_SPACE_WIDTH - 1:0]       immediate_1_space;
    wire[IMM_2_SPACE_WIDTH - 1:0]       immediate_2_space;
    wire[IMM_3_SPACE_WIDTH - 1:0]       immediate_3_space;
    wire[IMM_4_SPACE_WIDTH - 1:0]       immediate_4_space;
    wire[IMM_20_SPACE_WIDTH - 1:0]      immediate_20_space;  // immediate20
    wire[OPT_SPACE_WIDTH - 1:0]         option_space;
//    reg [DOUBLEWORD_WIDTH - 1:0]        rs1_buffer; 
//    reg [DOUBLEWORD_WIDTH - 1:0]        rs2_buffer; 
//    reg [DOUBLEWORD_WIDTH - 1:0]        rs3_buffer; 
    logic[SIGN_EXTEND_WIDTH - 1:0]      alu_sign_extend_imm2;
    logic[SIGN_EXTEND_WIDTH - 1:0]      alu_sign_extend_imm3;
    logic[DOUBLEWORD_WIDTH - 1:0]       data_bus_rd_sign_extend;
    logic[WORD_WIDTH - 1:0]             register_sign_extend_word;
    logic[BYTE_WIDTH*7 - 1:0]           register_sign_extend_7byte;
    // Operator decode (level_2) (register mapping - regFile or buffer)
    // Instruction  \----------- R-type  ------   1cycle     -------- 0 
    //                                 \------ multi-cycle   -------- 1
    //              \----------- I-type  -----    1cycle     -------- 2
    //                                 \------ multi-cycle   -------- 3
    //              \----------- DMEM   -----  LOAD_calAddr  -------- 4  
    //                                 \----   STORE_calAddr -------- 5
    logic[2:0]                          operator_decode;
    // Protocol interface
    reg [ADDDR_MAPPING_WIDTH - 1:0]     protocol_address_mapping_reg;
    reg                                 send_protocol_clk_reg;
    reg                                 receive_protocol_clk_reg;
    reg [AMOUNT_SND_WIDTH - 1:0]        amount_snd_byte_protocol_reg;
    
    // Protocol buffer
    reg [DOUBLEWORD_WIDTH - 1:0]        protocol_packet_h;
    reg [DOUBLEWORD_WIDTH - 1:0]        protocol_packet_l;
    // Synchronization primitive
    //  - read 
    reg [ADDR_BUS_WIDTH - 1:0]          addr_rd_reg;
    logic[DATA_TYPE_WIDTH - 1:0]        data_type_rd_logic;
    reg                                 rd_ins_reg;
    //  - write
    reg [DOUBLEWORD_WIDTH - 1:0]        data_bus_wr_reg;
    reg [ADDR_BUS_WIDTH - 1:0]          addr_wr_reg;
    logic[DATA_TYPE_WIDTH - 1:0]        data_type_wr_logic;
    reg                                 wr_ins_reg;
    
    // ALU block
    logic[DOUBLEWORD_WIDTH - 1:0]       alu_operand_1; 
    logic[DOUBLEWORD_WIDTH - 1:0]       alu_operand_2; 
    logic[DOUBLEWORD_WIDTH - 1:0]       alu_result_1cycle; 
    logic[DOUBLEWORD_WIDTH - 1:0]       alu_result_multi_cycle; 
    logic[OPCODE_ALU_WIDTH - 1:0]       alu_opcode;
    logic                               alu_enable_comb;
    logic                               alu_enable_seq;
    logic                               multi_cycle_type;
    wire                                alu_idle;
    
    
    reg [2:0]                           running_program_state_reg;
    // Running program state 
    localparam EXECUTE_INSTRUCTION_STATE    = 3'd0;
    localparam WRITE_BACK_STATE             = 3'd2;
    localparam DMEM_RD_ACCESS_STATE         = 3'd3;
    localparam DMEM_WR_ACCESS_STATE         = 3'd5;
    localparam SYSTEM_ACCESS_STATE          = 3'd4;
    localparam LOAD_REG_STATE               = 3'd1;
    // Synchronization primitive
    // -- Common
    always_comb begin
    if(opcode_space == LOAD_ENCODE) begin
        addr_dmem = addr_rd_reg;
        data_type_dmem = data_type_rd_logic;
    end
    else begin
        addr_dmem = addr_wr_reg;
        data_type_dmem = data_type_wr_logic;
    end
    end
    // -- read
//    assign addr_rd = addr_rd_reg;
//    assign data_type_rd = data_type_rd_logic;
    assign rd_ins = rd_ins_reg;
    assign register_sign_extend_word  = {32{data_bus_rd[WORD_WIDTH - 1]}};
    assign register_sign_extend_7byte = {56{data_bus_rd[BYTE_WIDTH - 1]}};
    always_comb begin
        case(funct3_space) 
            BYTE_WIDTH_ENCODE: begin
                data_bus_rd_sign_extend = {register_sign_extend_7byte, data_bus_rd[BYTE_WIDTH - 1:0]};
                data_type_rd_logic = BYTE_TYPE_ENCODE;
            end
            WORD_WIDTH_ENCODE: begin
                data_bus_rd_sign_extend = {register_sign_extend_word, data_bus_rd[WORD_WIDTH - 1:0]};
                data_type_rd_logic = WORD_TYPE_ENCODE;
            end
            DWORD_WIDTH_ENCODE: begin 
                data_bus_rd_sign_extend = data_bus_rd[DOUBLEWORD_WIDTH - 1:0];
                data_type_rd_logic = DWORD_TYPE_ENCODE;
            end
            default: begin
                data_bus_rd_sign_extend = data_bus_rd[DOUBLEWORD_WIDTH - 1:0];
                data_type_rd_logic = BYTE_TYPE_ENCODE;
            end
        endcase
    end
    // -- write
    assign wr_ins = wr_ins_reg;
    assign data_bus_wr = data_bus_wr_reg;
//    assign addr_wr = addr_wr_reg;
//    assign data_type_wr = data_type_wr_logic;
    always_comb begin
        case(funct3_space) 
            BYTE_WIDTH_ENCODE: data_type_wr_logic = BYTE_TYPE_ENCODE;
            WORD_WIDTH_ENCODE: data_type_wr_logic = WORD_TYPE_ENCODE;
            DWORD_WIDTH_ENCODE:data_type_wr_logic = DWORD_TYPE_ENCODE;
            default: data_type_wr_logic = BYTE_TYPE_ENCODE;
        endcase
    end
    // Decode instruciton
    assign opcode_space = fetch_instruction[OPCODE_SPACE_MSB:OPCODE_SPACE_LSB];
    assign funct10_space = fetch_instruction[FUNCT10_SPACE_MSB:FUNCT10_SPACE_LSB];
    assign funct3_space = fetch_instruction[FUNCT3_SPACE_MSB:FUNCT3_SPACE_LSB];
    assign rd_space = fetch_instruction[RD_SPACE_MSB:RD_SPACE_LSB];
    assign rs1_space = fetch_instruction[RS1_SPACE_MSB:RS1_SPACE_LSB];
    assign rs2_space = fetch_instruction[RS2_SPACE_MSB:RS2_SPACE_LSB];
    assign immediate_1_space = fetch_instruction[IMM_1_SPACE_MSB:IMM_1_SPACE_LSB];
    assign immediate_2_space = fetch_instruction[IMM_2_SPACE_MSB:IMM_2_SPACE_LSB];
    assign immediate_3_space = fetch_instruction[IMM_3_SPACE_MSB:IMM_3_SPACE_LSB];
    assign immediate_4_space = fetch_instruction[IMM_4_SPACE_MSB:IMM_4_SPACE_LSB];
    assign immediate_20_space= fetch_instruction[IMM_20_SPACE_MSB:IMM_20_SPACE_LSB];
    assign option_space = fetch_instruction[OPT_SPACE_MSB:OPT_SPACE_LSB];
    assign rs3_space = rd_space;
    // Multi-processor manager
    assign processor_idle = (running_program_state_reg == EXECUTE_INSTRUCTION_STATE);
    // Register management
    assign rs1_regFile = registers_renew[rs1_space];
    assign rs2_regFile = registers_renew[rs2_space];
    assign rs3_regFile = registers_renew[rs3_space];
    assign rd_regFile = registers_renew[rd_space];
    
    for(genvar index_register = 0; index_register < REGISTER_AMOUNT; index_register = index_register + 1) begin
        assign processor_registers[index_register] = registers_owner[index_register];
    end
    // Protocol management 
    assign protocol_address_mapping = protocol_address_mapping_reg;
    assign data_snd_protocol_per = {rs3_regFile, rs1_regFile};
    assign send_protocol_clk = send_protocol_clk_reg;
    assign receive_protocol_clk = receive_protocol_clk_reg;
    assign amount_snd_byte_protocol = amount_snd_byte_protocol_reg;
    
    // ALU Block
    assign alu_sign_extend_imm2 = {52{fetch_instruction[IMM_2_SPACE_MSB]}}; 
    assign alu_sign_extend_imm3 = {52{fetch_instruction[IMM_3_SPACE_MSB]}}; 
    
    
    logic [3:0]                     exec_state;
    logic                           wb_immediate_en;
    logic [DOUBLEWORD_WIDTH - 1:0]  wb_immediate_value;
    always_comb begin
        case(operator_decode) 
            EXECUTION_1CYCLE: begin    
                exec_state = EXECUTE_INSTRUCTION_STATE;
                // ALU control
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = rs2_regFile;
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 1;
                wb_immediate_value = alu_result_1cycle;
            end
            EXECUTION_MULTI_CYCLE: begin   
                exec_state = LOAD_REG_STATE;
                // ALU control 
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = rs2_regFile;
                alu_enable_comb = 0;
                multi_cycle_type = 1;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
            EXECUTION_1CYCLE_IMM: begin    
                exec_state = EXECUTE_INSTRUCTION_STATE;
                // ALU control
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = {alu_sign_extend_imm2, immediate_2_space, immediate_1_space};
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 1;
                wb_immediate_value = alu_result_1cycle;
            end
            EXECUTION_MULTI_CYCLE_IMM: begin  
                exec_state = LOAD_REG_STATE;
                // ALU control  
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = {alu_sign_extend_imm2, immediate_2_space, immediate_1_space};
                alu_enable_comb = 0;
                multi_cycle_type = 1;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
            EXECUTION_1CYCLE_LOAD_UPPER: begin
                exec_state = EXECUTE_INSTRUCTION_STATE;
                // ALU control
                alu_operand_1 = 64'h00;
                alu_operand_2 = 64'h00;
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 1;
                wb_immediate_value = {immediate_20_space, {12{1'b0}}, {32{1'b0}}};
            end
            EXECUTION_1CYCLE_ADDR_LOAD: begin 
                exec_state = DMEM_RD_ACCESS_STATE;
                // ALU control   
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = {alu_sign_extend_imm2, immediate_2_space, immediate_1_space};
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
            EXECUTION_1CYCLE_ADDR_STORE: begin   
                exec_state = DMEM_WR_ACCESS_STATE;
                // ALU control    
                alu_operand_1 = rs1_regFile;
                alu_operand_2 = {alu_sign_extend_imm3, immediate_3_space, immediate_1_space};
                alu_enable_comb = 1;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
            default: begin
                exec_state = EXECUTE_INSTRUCTION_STATE;
                // ALU control   
                alu_operand_1 = 64'h00;
                alu_operand_2 = 64'h00;
                alu_enable_comb = 0;
                multi_cycle_type = 0;
                // Write back immediate
                wb_immediate_en = 0;
                wb_immediate_value = 0;
            end
        endcase
    end
    always_comb begin
        case(opcode_space) 
            R_TYPE_ENCODE: begin    
                case(funct10_space) 
                    ADD_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = ADD_ALU_ENCODE;
                    end
                    SUB_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = SUB_ALU_ENCODE;
                    end
                    AND_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = AND_ALU_ENCODE;
                    end
                    XOR_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = XOR_ALU_ENCODE;
                    end
                    OR_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = OR_ALU_ENCODE;
                    end
                    SLT_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE;
                        alu_opcode = SLT_ALU_ENCODE;
                    end
                    SRL_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE;
                        alu_opcode = SRL_ALU_ENCODE;
                    end
                    SLL_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE;
                        alu_opcode = SLL_ALU_ENCODE;
                    end
                    MUL_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE;
                        alu_opcode = MUL_ALU_ENCODE;
                    end
                    default: begin
                        operator_decode = 0;
                        alu_opcode = 0;
                    end
                endcase
            end
            I_TYPE_ENCODE: begin    
                case(funct3_space) 
                    ADDI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = ADD_ALU_ENCODE;
                    end
                    ANDI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = AND_ALU_ENCODE;
                    end
                    XORI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = XOR_ALU_ENCODE;
                    end
                    ORI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = OR_ALU_ENCODE;
                    end
                    SLTI_ENCODE: begin
                        operator_decode = EXECUTION_1CYCLE_IMM;
                        alu_opcode = SLT_ALU_ENCODE;
                    end
                    SRLI_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE_IMM;
                        alu_opcode = SRL_ALU_ENCODE;
                    end
                    SLLI_ENCODE: begin
                        operator_decode = EXECUTION_MULTI_CYCLE_IMM;
                        alu_opcode = SLL_ALU_ENCODE;
                    end
                    default: begin
                        operator_decode = 0;
                        alu_opcode = 0;
                    end
                endcase
            end
            LUI_TYPE_ENCODE: begin
                operator_decode = EXECUTION_1CYCLE_LOAD_UPPER;
                alu_opcode = 0;
            end
            LOAD_ENCODE: begin      // result: address of target -> access Data memory to write value
                operator_decode = EXECUTION_1CYCLE_ADDR_LOAD;
                alu_opcode = ADD_ALU_ENCODE;
            end
            STORE_ENCODE: begin
                operator_decode = EXECUTION_1CYCLE_ADDR_STORE;
                alu_opcode = ADD_ALU_ENCODE;
            end
            default: begin
                operator_decode = 0;
                alu_opcode = 0;
            end
        endcase
    end
    
    logic [ADDR_BUS_WIDTH - 1:0]    dmem_addr_rd;
    logic [ADDR_BUS_WIDTH - 1:0]    dmem_addr_wr;
    logic                           dmem_ins_rd_en;
    logic                           dmem_ins_wr_en;
    logic [DOUBLEWORD_WIDTH - 1:0]  dmem_data_wr;
    always_comb begin
        if(opcode_space == LOAD_ENCODE) begin
            dmem_addr_rd = alu_result_1cycle;
            dmem_ins_rd_en = 1;
        end
        else begin
            dmem_addr_rd = addr_rd_reg;
            dmem_ins_rd_en = 0;
        end
       
        if(opcode_space == STORE_ENCODE) begin
            dmem_addr_wr = alu_result_1cycle;
            dmem_ins_wr_en = 1;
            dmem_data_wr = rs2_regFile;
        end
        else begin
            dmem_addr_wr = addr_wr_reg;
            dmem_ins_wr_en = 0;
            dmem_data_wr = data_bus_wr_reg; // Don't care (not modify register)
        end
    end
    alu         
        #(
        .OPERAND_WIDTH(DOUBLEWORD_WIDTH)
        )alu(
        .clk(clk),
        .operand_1(alu_operand_1),
        .operand_2(alu_operand_2),
        .result_1cycle(alu_result_1cycle),
        .result_multi_cycle(alu_result_multi_cycle),
        .op_code(alu_opcode),
        .alu_enable_comb(alu_enable_comb),
        .alu_enable_seq(alu_enable_seq),
        .alu_idle(alu_idle),
//        .revert_result(),
        .overflow_flag(),
        .invalid_flag(),
        .rst_n(rst_n)
        ); 
                
    logic [2:0]                     running_program_state_n;
    logic                           alu_enable_seq_n;
    logic [DOUBLEWORD_WIDTH - 1:0]  registers_owner_n    [0:REGISTER_AMOUNT - 1];
    logic [ADDR_BUS_WIDTH - 1:0]    addr_rd_n;
    logic [ADDR_BUS_WIDTH - 1:0]    addr_wr_n;
    logic                           rd_ins_n;
    logic                           wr_ins_n;
    logic [DOUBLEWORD_WIDTH - 1:0]  data_bus_wr_n;
    always_comb begin               : RUNNING_PROGRAM_FSM_GENERATOR
        running_program_state_n = running_program_state_reg;
        alu_enable_seq_n = alu_enable_seq;// Reset register in processor
        for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
        registers_owner_n[i] = registers_owner[i];
        end
        addr_rd_n = addr_rd_reg;
        addr_wr_n = addr_wr_reg;
        rd_ins_n = rd_ins_reg;
        wr_ins_n = wr_ins_reg;
        data_bus_wr_n = data_bus_wr_reg;
        
        case(running_program_state_reg) 
            EXECUTE_INSTRUCTION_STATE: begin
                if(boot_processor) begin
                    running_program_state_n = exec_state;
                    alu_enable_seq_n = (multi_cycle_type) ? 1'b1 : 1'b0;
                    if(wb_immediate_en) registers_owner_n[rd_space] = wb_immediate_value;
                    // DMEM ACESS
                    addr_rd_n = dmem_addr_rd;
                    addr_wr_n = dmem_addr_wr;
                    rd_ins_n = dmem_ins_rd_en;
                    wr_ins_n = dmem_ins_wr_en;
                    data_bus_wr_n = dmem_data_wr;
                end
                else if(synchronization_processor) begin
                    for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) registers_owner_n[i] = registers_renew[i];
                end
            end
            WRITE_BACK_STATE: begin
                if(alu_idle == 1) begin
                    registers_owner_n[rd_space] = alu_result_multi_cycle;
                    running_program_state_n = EXECUTE_INSTRUCTION_STATE;
                end
            end
            DMEM_RD_ACCESS_STATE: begin
                if(rd_idle) begin
                    running_program_state_n = EXECUTE_INSTRUCTION_STATE;
                    registers_owner_n[rd_space] = data_bus_rd_sign_extend;
                    rd_ins_n = 0;
                end
            end
            DMEM_WR_ACCESS_STATE: begin
                if(wr_access) begin // Write Memory completely
                    running_program_state_n = EXECUTE_INSTRUCTION_STATE;
                    wr_ins_n = 0;
                end
            end
            SYSTEM_ACCESS_STATE: begin
            
            end
            LOAD_REG_STATE: begin
                running_program_state_n = WRITE_BACK_STATE;
            end
            default: begin
            
            end        
        endcase
    end
    // Critical path: 
    always @(posedge clk) begin
        if(!rst_n) begin
            running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
            alu_enable_seq <= 0;// Reset register in processor
            for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
                registers_owner[i] <= REGISTER_DEFAULT[i];
            end
            addr_rd_reg <= 0;
            addr_wr_reg <= 0;
            rd_ins_reg <= 0;
            wr_ins_reg <= 0;
            data_bus_wr_reg <= 0;
        end
        else begin
            running_program_state_reg <= running_program_state_n;
            alu_enable_seq <= alu_enable_seq_n;// Reset register in processor
            for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
            registers_owner[i] <= registers_owner_n[i];
            end
            addr_rd_reg <= addr_rd_n;
            addr_wr_reg <= addr_wr_n;
            rd_ins_reg <= rd_ins_n;
            wr_ins_reg <= wr_ins_n;
            data_bus_wr_reg <= data_bus_wr_n;
            
        end
    end
    // Synchronization reigsters block
    `ifdef DEBUG
    // Debug area
//    assign debug_2 = {32'b0, 32'b0};
    `endif
    end
    
    
    endgenerate
    
endmodule

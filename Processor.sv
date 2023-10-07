`define DEBUG
module Processor
    #(
    parameter           MAIN_RPOCESSOR          = 1,            // Main processor - Sub processor
    
    parameter           DATA_WIDTH              = 8,
    parameter           DOUBLEWORD_WIDTH        = 64,
    parameter           DATA_MEMORY_SIZE        = 10'd256,      // 256 bytes (2Kb)
    
    parameter           ADDR_WIDTH_DM           = $clog2(DATA_MEMORY_SIZE),
    parameter           DATA_TYPE_WIDTH         = 2,
    
    parameter           INSTRUCTION_WIDTH       = 32,   //32-bit instruction
    parameter           PROGRAM_MEMORY_SIZE     = 256,
    
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
    parameter IMM_3_SPACE_WIDTH     = IMM_3_SPACE_MSB - IMM_3_SPACE_LSB + 1,// Opcode encoder
    parameter JUMP_OFS_SPACE_MSB    = 31,
    parameter JUMP_OFS_SPACE_LSB    = 7,
    parameter JUMP_OFS_SPACE_WIDTH  = JUMP_OFS_SPACE_MSB - JUMP_OFS_SPACE_LSB + 1,// Opcode encoder
    parameter OPT_SPACE_MSB         = 11,               // Option space
    parameter OPT_SPACE_LSB         = 10,               // Option space    
    parameter OPT_SPACE_WIDTH       = OPT_SPACE_MSB - OPT_SPACE_LSB + 1,
    parameter IMM_4_SPACE_MSB       = 16,               // Option space
    parameter IMM_4_SPACE_LSB       = 11,               // Option space    
    parameter IMM_4_SPACE_WIDTH     = IMM_4_SPACE_MSB - IMM_4_SPACE_LSB + 1,
    // Opcode encoder
    parameter R_TYPE_ENCODE         = 7'b0110011,       // R-type
    parameter I_TYPE_ENCODE         = 7'b0010011,       // I-type
    parameter J_TYPE_ENCODE         = 7'b1100111,       // J-type
    parameter JAL_TYPE_ENCODE       = 7'b1101111,       // J-type
    parameter JALR_TYPE_ENCODE      = 7'b1101011,       // J-type
    parameter B_TYPE_ENCODE         = 7'b1100011,       // B-type   
    parameter LOAD_ENCODE           = 7'b0000011,       // LOAD-type
    parameter STORE_ENCODE          = 7'b0100011,       // STORE-type
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
    // I/O peripheral 
    parameter GPIO_PORT_A_NUM           = 4,
    parameter GPIO_PORT_B_NUM           = 4,
    parameter GPIO_PORT_C_NUM           = 4,
    parameter GPIO_PORT_A_MAP           = 0,
    parameter GPIO_PORT_B_MAP           = 1,
    parameter GPIO_PORT_C_MAP           = 2,
    parameter GPIO_PORT_AMOUNT          = 3,
    parameter GPIO_PORT_WIDTH           = $clog2(GPIO_PORT_AMOUNT),
    //ALU parameter 
    parameter OPCODE_ALU_WIDTH          = 17
    )(
    input clk,
    
    // Multi-processor Manager
    input   wire [INSTRUCTION_WIDTH - 1:0]  fetch_instruction,
    input   wire                            boot_processor,
    output  wire                            processor_idle,    
    
    // Synchronization primitive (READ_STATE)
    input   wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd,
    output  wire [ADDR_WIDTH_DM - 1:0]      addr_rd,
    output  wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd,
    input   wire                            rd_idle,   
    output  wire                            rd_ins, 
    input   wire                            rd_access,
    output  wire                            rd_finish,
    // Synchronization primitive (WRITE_STATE)
    output  wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr,
    output  wire [ADDR_WIDTH_DM - 1:0]      addr_wr,
    output  wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr,
    input   wire                            wr_idle,   
    output  wire                            wr_ins, 
    input   wire                            wr_access,
    
    // For main Processor ///////////////////////////////////////////////
    // UART_RX_1
    input   wire [DATA_WIDTH - 1:0]         data_bus_out_uart_1,
    output  wire                            RX_use_1,
    output  wire                            RX_flag_1,
    // UART_TX_1
    output  wire [DATA_WIDTH - 1:0]         data_bus_in_uart_1,
    output  wire                            TX_use_1,
    // Program memory
    output  wire    [DATA_WIDTH - 1:0]      data_bus_wr_pm,
    input   wire                            wr_idle_pm,
    output  wire    [ADDR_WIDTH_PM - 1:0]   addr_wr_pm,
    output  wire                            wr_ins_pm,
    // GPIO 
    input   wire    [0:GPIO_PORT_A_NUM - 1] GPIO_PORT_A,
    input   wire    [0:GPIO_PORT_B_NUM - 1] GPIO_PORT_B,
    input   wire    [0:GPIO_PORT_C_NUM - 1] GPIO_PORT_C,
    // Common 
    output  wire    [1:0]                   main_state,
    //////////////////////////////////////////////////////////////////////
    
    // For sub Processor ///////////////////////////////////////////////
    // Data_bus block
    output  wire [ADDDR_MAPPING_WIDTH - 1:0]protocol_address_mapping,
    output  wire [DOUBLEWORD_WIDTH*2 - 1:0] data_snd_protocol_per,      // Maximum size of packet is 128bytes
    input   wire [DOUBLEWORD_WIDTH*2 - 1:0] data_rcv_protocol_per,
    output  wire                            send_protocol_clk,
    output  wire                            receive_protocol_clk,
    output  wire [AMOUNT_SND_WIDTH - 1:0]   amount_snd_byte_protocol,
    input   wire [AMOUNT_RCV_WIDTH - 1:0]   amount_rcv_byte_protocol,
    input   wire                            snd_protocol_available,
    input   wire                            rcv_protocol_available,
    //////////////////////////////////////////////////////////////////////
    // Register management
    output  wire [DOUBLEWORD_WIDTH - 1:0]   processor_registers [0:REGISTER_AMOUNT - 1],
    input   wire [DOUBLEWORD_WIDTH - 1:0]   registers_renew     [0:REGISTER_AMOUNT - 1],
    input   wire                            synchronization_processor,
    
    input rst_n
    
    `ifdef DEBUG
    // Debug 
    ,output wire    [63:0]                      debug_1
    ,output wire    [63:0]                      debug_2
    `endif
    );
    generate
    if(MAIN_RPOCESSOR == 1) begin   : MAIN_RPOCESSOR_BLOCK
        
        // Declare CPU registers
        reg [DOUBLEWORD_WIDTH - 1:0] registers_owner    [0:REGISTER_AMOUNT - 1];
        // Declare wires and states
        reg [1:0] main_state_reg;
        reg [1:0] load_program_state_reg;
        reg [3:0] running_program_state_reg;
        reg       RX_use_1_reg;
        reg [1:0] _4byte_counter;         // Overflow at 4
        wire      finish_program_condition;
        wire      finish_program_condition_1;
        wire      finish_program_condition_2;
        wire      start_counting_finish_program_condition;
        // Program Memory 
        // Load-Program state
        reg [DATA_WIDTH - 1:0]          data_bus_wr_pm_reg;
        reg [ADDR_WIDTH_PM - 1:0]       addr_wr_pm_reg;
        reg                             wr_ins_pm_reg;
        // Execute-program state
        reg [INSTRUCTION_WIDTH - 1:0]       fetch_instruction_reg;
        wire[OPCODE_SPACE_WIDTH - 1:0]      opcode_space;
        wire[FUNCT10_SPACE_WIDTH - 1:0]     funct10_space;
        wire[FUNCT3_SPACE_WIDTH - 1:0]      funct3_space;
        wire[REGISTER_SPACE_WIDTH - 1:0]    rd_space;
        wire[REGISTER_SPACE_WIDTH - 1:0]    rs1_space;
        wire[REGISTER_SPACE_WIDTH - 1:0]    rs2_space;
        wire[IMM_1_SPACE_WIDTH - 1:0]       immediate_1_space;
        wire[IMM_2_SPACE_WIDTH - 1:0]       immediate_2_space;
        wire[IMM_3_SPACE_WIDTH - 1:0]       immediate_3_space;
        reg [DOUBLEWORD_WIDTH - 1:0]        rs1_buffer; 
        reg [DOUBLEWORD_WIDTH - 1:0]        rs2_buffer; 
        
        // Synchronization primitive
        //  - read 
        reg [ADDR_WIDTH_DM - 1:0]           addr_rd_reg;
        reg [DATA_TYPE_WIDTH - 1:0]         data_type_rd_reg;   
        reg                                 rd_ins_reg;
        reg                                 rd_finish_reg;
        //  - write
        reg [DOUBLEWORD_WIDTH - 1:0]        data_bus_wr_reg;
        reg [ADDR_WIDTH_DM - 1:0]           addr_wr_reg;
        reg [DATA_TYPE_WIDTH - 1:0]         data_type_wr_reg;
        reg                                 wr_ins_reg;
        
        // ALU block
        reg [DOUBLEWORD_WIDTH - 1:0]        alu_operand_1; 
        reg [DOUBLEWORD_WIDTH - 1:0]        alu_operand_2; 
        wire[DOUBLEWORD_WIDTH - 1:0]        alu_result; 
        reg [OPCODE_ALU_WIDTH - 1:0]        alu_opcode;
        reg                                 alu_use;
        wire                                alu_idle;
        // GPIO 
        wire[GPIO_PORT_WIDTH - 1:0]         pin_rs2_align;
        wire[GPIO_PORT_WIDTH - 1:0]         port_rs1_align;
        
        // Main state encoder
        localparam IDLE_STATE = 0;
        localparam STORE_PROGRAM_STATE = 1;
        localparam RUNNING_PROGRAM_STATE = 2;
        // Load program state
        localparam RD_FIFO_STATE = 1;
        localparam WR_RAM_STATE = 2;
        // Running program state 
        localparam DECODE_INSTRUCTION_STATE = 1;
        localparam EXECUTE_INSTRUCTION_STATE = 2;
        localparam EXECUTE_ADDR_INSTRUCTION_STATE = 3;
        localparam EXECUTE_WR_ADDR_INSTRUCTION_STATE = 8;
        localparam EXECUTE_RD_PRE_ACCESS_INSTRUCTION_STATE = 4;
        localparam EXECUTE_RD_ACCESS_INSTRUCTION_STATE = 5;
        localparam EXECUTE_WR_PRE_ACCESS_INSTRUCTION_STATE = 6;
        localparam EXECUTE_WR_ACCESS_INSTRUCTION_STATE = 7;
        localparam ADDRESS_MAPPING_PROT_STATE = 8;
        
        assign main_state = main_state_reg;
        assign RX_use_1 = RX_use_1_reg;
        // Finish program condition
        assign finish_program_condition = finish_program_condition_1 | finish_program_condition_2;
        assign finish_program_condition_1 = (data_bus_out_uart_1[6:0] == FINISH_PROGRAM_OPCODE & RX_flag_1 == 1) & (_4byte_counter == 0);   // (_4byte_counter == 0) <=> Opdcode location
        assign start_counting_finish_program_condition = (main_state_reg == STORE_PROGRAM_STATE) & (RX_flag_1 == 0);        // MCU in STORE_PROGRAM_STATE & RX_module is empty 
        // Program Memory 
        assign data_bus_wr_pm = data_bus_wr_pm_reg;
        assign addr_wr_pm = addr_wr_pm_reg;
        assign wr_ins_pm = wr_ins_pm_reg;
        // Synchronization primitive
        // -- read
        assign addr_rd = addr_rd_reg;
        assign data_type_rd = data_type_rd_reg;
        assign rd_ins = rd_ins_reg;
        assign rd_finish = rd_finish_reg;
        // -- write
        assign wr_ins = wr_ins_reg;
        assign data_bus_wr = data_bus_wr_reg;
        assign addr_wr = addr_wr_reg;
        assign data_type_wr = data_type_wr_reg;
        // Decode instruciton
        assign opcode_space = fetch_instruction_reg[OPCODE_SPACE_MSB:OPCODE_SPACE_LSB];
        assign funct10_space = fetch_instruction_reg[FUNCT10_SPACE_MSB:FUNCT10_SPACE_LSB];
        assign funct3_space = fetch_instruction_reg[FUNCT3_SPACE_MSB:FUNCT3_SPACE_LSB];
        assign rd_space = fetch_instruction_reg[RD_SPACE_MSB:RD_SPACE_LSB];
        assign rs1_space = fetch_instruction_reg[RS1_SPACE_MSB:RS1_SPACE_LSB];
        assign rs2_space = fetch_instruction_reg[RS2_SPACE_MSB:RS2_SPACE_LSB];
        assign immediate_1_space = fetch_instruction_reg[IMM_1_SPACE_MSB:IMM_1_SPACE_LSB];
        assign immediate_2_space = fetch_instruction_reg[IMM_2_SPACE_MSB:IMM_2_SPACE_LSB];
        assign immediate_3_space = fetch_instruction_reg[IMM_3_SPACE_MSB:IMM_3_SPACE_LSB];
        // Multi-processor manager
        assign processor_idle = (running_program_state_reg == IDLE_STATE);
        // Register management
        for(genvar index_register = 0; index_register < REGISTER_AMOUNT; index_register = index_register + 1) begin
            assign processor_registers[index_register] = registers_owner[index_register];
        end
        // GPIO block 
        assign pin_rs2_align  = registers_renew[rs2_space][GPIO_PORT_WIDTH - 1:0];
        assign port_rs1_align = registers_renew[rs1_space][GPIO_PORT_WIDTH - 1:0];
        // ALU block
        always @* begin
            case(opcode_space) 
                R_TYPE_ENCODE: begin    
                    alu_operand_1 <= rs1_buffer;
                    alu_operand_2 <= rs2_buffer;
                    alu_opcode <= {funct10_space, opcode_space};
                end
                I_TYPE_ENCODE: begin    
                    alu_operand_1 <= rs1_buffer;
                    alu_operand_2 <= {52'h00, immediate_2_space, immediate_1_space};
                    alu_opcode <= {7'b00, funct3_space, opcode_space};
                end
                LOAD_ENCODE: begin      // result: address of target -> access Data memory to write value
                    alu_operand_1 <= rs1_buffer;
                    alu_operand_2 <= {52'h00, immediate_2_space, immediate_1_space};
                    alu_opcode <= {ADD_ENCODE, R_TYPE_ENCODE};  // Add: Base + offset = Address of target
                end
                STORE_ENCODE: begin
                    alu_operand_1 <= rs1_buffer;
                    alu_operand_2 <= {52'h00, immediate_3_space, immediate_1_space};
                    alu_opcode <= {ADD_ENCODE, R_TYPE_ENCODE};  // Add: Base + offset = Address of target
                end
                J_TYPE_ENCODE: begin
                // Processor doesn't process J-type instruction
                    alu_operand_1 <= 64'h00;
                    alu_operand_2 <= 64'h00;
                    alu_opcode <= 17'b00;
                end
                B_TYPE_ENCODE: begin
                // Processor doesn't process B-type instruction
                    alu_operand_1 <= 64'h00;
                    alu_operand_2 <= 64'h00;
                    alu_opcode <= 17'b00;
                end
                default: begin
                    alu_operand_1 <= 64'h00;
                    alu_operand_2 <= 64'h00;
                    alu_opcode <= 17'b00;
                end
            endcase
        end
        ALU_module #(
                    .OPERAND_WIDTH(DOUBLEWORD_WIDTH)
                    )
        alu_module  (
                    .clk(clk),
                    .operand_1(alu_operand_1),
                    .operand_2(alu_operand_2),
                    .result(alu_result),
                    .op_code(alu_opcode),
                    .alu_trig(alu_use),
                    .alu_idle(alu_idle),
                    .rst_n(rst_n)
                    ); 
                    
        // Timer for finishing program (time out)
        waiting_module #(
                    .END_COUNTER(FINISH_PROGRAM_TIMER),
                    .WAITING_TYPE(0),
                    .LEVEL_PULSE(1)
                    )timout_programming(
                    .clk(clk),
                    .start_counting(start_counting_finish_program_condition),
                    .reach_limit(finish_program_condition_2),
                    .rst_n(rst_n)
                    );             
                    
        // Main state controller
        always @(posedge clk, negedge rst_n) begin
            if(!rst_n) begin
                main_state_reg <= IDLE_STATE;
                load_program_state_reg <= IDLE_STATE;
                running_program_state_reg <= IDLE_STATE;
                
                // Reset buffer interact with Program-memory
                data_bus_wr_pm_reg <= 8'h00;
                addr_wr_pm_reg <= START_WR_ADDR_PM;
                wr_ins_pm_reg <= 0;
                // Reset register in processor
                for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
                    registers_owner[i] <= REGISTER_DEFAULT[i];
                end
                // Reset buffer
                RX_use_1_reg <= 0;
                _4byte_counter <= 0;
                alu_use <= 0;
                addr_rd_reg <= 0;
                rd_ins_reg <= 0;
                data_type_rd_reg <= 0;
                rd_finish_reg <= 0;
                addr_wr_reg <= 0;
                data_bus_wr_reg <= 0;
                wr_ins_reg <= 0;
                data_type_wr_reg <= 0;
                rs1_buffer <= 0;
                rs2_buffer <= 0;
            end
            else begin
                case(main_state_reg) 
                    IDLE_STATE: begin
                        if(RX_flag_1) begin
                            main_state_reg <= STORE_PROGRAM_STATE;
                        end 
                        else main_state_reg <= IDLE_STATE;              // Polling RX_1 data 
                        RX_use_1_reg <= 0;
                    end
                    STORE_PROGRAM_STATE: begin
                        case(load_program_state_reg) 
                            IDLE_STATE: begin
                                if(finish_program_condition) begin              // Finish program
                                    main_state_reg <= RUNNING_PROGRAM_STATE;
                                    RX_use_1_reg <= (RX_flag_1 == 1) ? 1 : 0;   // Clear FIFO
                                end    
                                else begin
                                    load_program_state_reg <= RD_FIFO_STATE;
                                    data_bus_wr_pm_reg <= data_bus_out_uart_1;
                                    RX_use_1_reg <= 1;
                                    _4byte_counter <= _4byte_counter + 1;
                                end
                            end
                            RD_FIFO_STATE: begin
                                // Reset signal
                                RX_use_1_reg <= 0;
                                
                                if(wr_idle_pm) begin
                                    load_program_state_reg <= WR_RAM_STATE;
                                    wr_ins_pm_reg <= 1;
                                end
                            end
                            WR_RAM_STATE: begin
                                // Reset signal
                                wr_ins_pm_reg <= 0;
                                
                                if(finish_program_condition) begin              // Finish program
                                    main_state_reg <= RUNNING_PROGRAM_STATE;
                                    load_program_state_reg <= IDLE_STATE;
                                    
                                    RX_use_1_reg <= (RX_flag_1 == 1) ? 1 : 0;   // Clear FIFO
                                end    
                                else begin
                                    if(RX_flag_1) begin
                                        load_program_state_reg <= RD_FIFO_STATE;
                                        // UART signal
                                        RX_use_1_reg <= 1;
                                        // Program memory signal
                                        data_bus_wr_pm_reg <= data_bus_out_uart_1;
                                        addr_wr_pm_reg <= addr_wr_pm_reg + 1;
                                        _4byte_counter <= _4byte_counter + 1;
                                    end
                                end
                            end
                            default: begin
                            
                            end
                        endcase 
                    end
                    RUNNING_PROGRAM_STATE: begin
                        // Reset signal
                        RX_use_1_reg <= 0;
                        
                        case(running_program_state_reg) 
                            IDLE_STATE: begin
                                if(boot_processor) begin
                                    running_program_state_reg <= DECODE_INSTRUCTION_STATE;
                                    
                                    fetch_instruction_reg <= fetch_instruction; 
                                end
                                else running_program_state_reg <= IDLE_STATE;
                            end
                            DECODE_INSTRUCTION_STATE: begin
                            
                                case(opcode_space) 
                                    R_TYPE_ENCODE: begin
                                        // Confirm data
                                        rs1_buffer <= registers_renew[rs1_space];
                                        rs2_buffer <= registers_renew[rs2_space];
                                        running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
                                        alu_use <= 1;
//                                        case(funct10_space) 
//                                            ADD_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SUB_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SLT_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SLTU_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SRL_ENCODE: begin
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SLL_ENCODE: begin
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            AND_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            XOR_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            OR_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            MUL_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            default: begin
//                                                running_program_state_reg <= IDLE_STATE;
//                                            end
//                                        endcase
                                    end
                                    I_TYPE_ENCODE: begin
                                        // Confirm data
                                        rs1_buffer <= registers_renew[rs1_space];
                                        alu_use <= 1;
                                        running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                        case(funct3_space) 
//                                            ADDI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end               
//                                            SLLI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SRLI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end  
//                                            SLTI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SLTIU_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            XORI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            ORI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            ANDI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end    
//                                            default: begin
//                                                running_program_state_reg <= IDLE_STATE;
//                                            end
//                                        endcase 
                                    end    
                                    J_TYPE_ENCODE: begin
                                    // Processor don't process J-type instruction
                                    end
                                    B_TYPE_ENCODE: begin
                                    // Processor don't process J-type instruction
                                    end
                                    LOAD_ENCODE: begin
                                        // Confirm data
                                        rs1_buffer <= registers_renew[rs1_space];
                                        alu_use <= 1;
                                        running_program_state_reg <= EXECUTE_ADDR_INSTRUCTION_STATE;
                                    end
                                    STORE_ENCODE: begin
                                        // Confirm data
                                        rs1_buffer <= registers_renew[rs1_space];
                                        alu_use <= 1;
                                        running_program_state_reg <= EXECUTE_WR_ADDR_INSTRUCTION_STATE;
                                    end
                                    GPIO_ENCODE: begin        
                                        running_program_state_reg <= IDLE_STATE;
                                        case(funct3_space) 
                                            READ_GPIO_ENCODE: begin
                                                case(port_rs1_align)
                                                    GPIO_PORT_A_MAP: begin
                                                       registers_owner[rd_space] <= {63'd0, GPIO_PORT_A[pin_rs2_align]};
                                                    end
                                                    GPIO_PORT_B_MAP: begin
                                                       registers_owner[rd_space] <= {63'd0, GPIO_PORT_B[pin_rs2_align]};
                                                    end
                                                    GPIO_PORT_C_MAP: begin
                                                       registers_owner[rd_space] <= {63'd0, GPIO_PORT_C[pin_rs2_align]};
                                                    end
                                                    default: begin
                                                    
                                                    end
                                                endcase 
                                            end
                                            default: begin
                                            
                                            end
                                        endcase
                                    end
                                endcase 
                            end
                            EXECUTE_INSTRUCTION_STATE: begin
                                alu_use <= 0;
                                if(alu_idle == 1) begin
                                    registers_owner[rd_space] <= alu_result;
                                    running_program_state_reg <= IDLE_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                            EXECUTE_ADDR_INSTRUCTION_STATE: begin
                                alu_use <= 0;
                                if(alu_idle == 1) begin
                                    addr_rd_reg <= alu_result;
                                    rd_ins_reg <= 1;
                                    case(funct3_space) 
                                        BYTE_WIDTH_ENCODE: data_type_rd_reg <= BYTE_TYPE_ENCODE;
                                        WORD_WIDTH_ENCODE: data_type_rd_reg <= WORD_TYPE_ENCODE;
                                        DWORD_WIDTH_ENCODE: data_type_rd_reg <= DWORD_TYPE_ENCODE;
                                        default: data_type_rd_reg <= BYTE_TYPE_ENCODE;
                                    endcase
                                    rd_finish_reg <= 0;      
                                    running_program_state_reg <= EXECUTE_RD_PRE_ACCESS_INSTRUCTION_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                                
                            end
                            EXECUTE_RD_PRE_ACCESS_INSTRUCTION_STATE: begin
                                if(rd_access) begin
                                    running_program_state_reg <= EXECUTE_RD_ACCESS_INSTRUCTION_STATE;
                                    rd_ins_reg <= 0;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                            EXECUTE_RD_ACCESS_INSTRUCTION_STATE: begin
                                if(rd_idle) begin
                                    registers_owner[rd_space] <= data_bus_rd;
                                    rd_finish_reg <= 1;
                                    running_program_state_reg <= IDLE_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                            EXECUTE_WR_ADDR_INSTRUCTION_STATE: begin
                                alu_use <= 0;
                                if(alu_idle == 1) begin
                                    addr_wr_reg <= alu_result;
                                    data_bus_wr_reg <= registers_renew[rs2_space];
                                    wr_ins_reg <= 1;
                                    case(funct3_space) 
                                        BYTE_WIDTH_ENCODE: data_type_wr_reg <= BYTE_TYPE_ENCODE;
                                        WORD_WIDTH_ENCODE: data_type_wr_reg <= WORD_TYPE_ENCODE;
                                        DWORD_WIDTH_ENCODE: data_type_wr_reg <= DWORD_TYPE_ENCODE;
                                        default: data_type_wr_reg <= BYTE_TYPE_ENCODE;
                                    endcase
                                    running_program_state_reg <= EXECUTE_WR_PRE_ACCESS_INSTRUCTION_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                            EXECUTE_WR_PRE_ACCESS_INSTRUCTION_STATE: begin
                                if(wr_access) begin
                                    running_program_state_reg <= EXECUTE_WR_ACCESS_INSTRUCTION_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                            EXECUTE_WR_ACCESS_INSTRUCTION_STATE: begin  
                                // Notation of EXECUTE_WR_ACCESS_INSTRUCTION_STATE: Just wait for 1 cycle to confirm stablization of interactive buffer
                                running_program_state_reg <= IDLE_STATE;
                                wr_ins_reg <= 0;
                            end
                        endcase
                    end
                endcase 
            end
        end
        // Synchronization reigsters block
        always @(posedge clk) begin
            if(synchronization_processor) begin
                for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
                    registers_owner[i] <= registers_renew[i];
                end
            end
        end
        `ifdef DEBUG
        // Debug area
        assign debug_1 = {0, alu_result};
        `endif
    end
    else begin                      : SUB_PROCESSOR_BLOCK
        reg [3:0]                           running_program_state_reg;
        reg [DOUBLEWORD_WIDTH - 1:0]        registers_owner    [0:REGISTER_AMOUNT - 1];
        // Execute-program state
        reg [INSTRUCTION_WIDTH - 1:0]       fetch_instruction_reg;
        wire[OPCODE_SPACE_WIDTH - 1:0]      opcode_space;
        wire[FUNCT10_SPACE_WIDTH - 1:0]     funct10_space;
        wire[FUNCT3_SPACE_WIDTH - 1:0]      funct3_space;
        wire[REGISTER_SPACE_WIDTH - 1:0]    rd1_space;
        wire[REGISTER_SPACE_WIDTH - 1:0]    rd2_space;
        wire[REGISTER_SPACE_WIDTH - 1:0]    rd3_space;
        wire[REGISTER_SPACE_WIDTH - 1:0]    rs1_space;
        wire[REGISTER_SPACE_WIDTH - 1:0]    rs2_space;
        wire[REGISTER_SPACE_WIDTH - 1:0]    rs3_space;
        wire[IMM_1_SPACE_WIDTH - 1:0]       immediate_1_space;
        wire[IMM_2_SPACE_WIDTH - 1:0]       immediate_2_space;
        wire[IMM_3_SPACE_WIDTH - 1:0]       immediate_3_space;
        wire[IMM_4_SPACE_WIDTH - 1:0]       immediate_4_space;
        wire[OPT_SPACE_WIDTH - 1:0]         option_space;
        reg [DOUBLEWORD_WIDTH - 1:0]        rs1_buffer; 
        reg [DOUBLEWORD_WIDTH - 1:0]        rs2_buffer; 
        reg [DOUBLEWORD_WIDTH - 1:0]        rs3_buffer; 
        // Protocol interface
        reg [ADDDR_MAPPING_WIDTH - 1:0]     protocol_address_mapping_reg;
        reg                                 send_protocol_clk_reg;
        reg                                 receive_protocol_clk_reg;
        reg [AMOUNT_SND_WIDTH - 1:0]        amount_snd_byte_protocol_reg;
        // Synchronization primitive
        //  - read 
        reg [ADDR_WIDTH_DM - 1:0]           addr_rd_reg;
        reg [DATA_TYPE_WIDTH - 1:0]         data_type_rd_reg;   
        reg                                 rd_ins_reg;
        reg                                 rd_finish_reg;
        //  - write
        reg [DOUBLEWORD_WIDTH - 1:0]        data_bus_wr_reg;
        reg [ADDR_WIDTH_DM - 1:0]           addr_wr_reg;
        reg [DATA_TYPE_WIDTH - 1:0]         data_type_wr_reg;
        reg                                 wr_ins_reg;
        
        // ALU block
        reg [DOUBLEWORD_WIDTH - 1:0]        alu_operand_1; 
        reg [DOUBLEWORD_WIDTH - 1:0]        alu_operand_2; 
        wire[DOUBLEWORD_WIDTH - 1:0]        alu_result; 
        reg [OPCODE_ALU_WIDTH - 1:0]        alu_opcode;
        reg                                 alu_use;
        wire                                alu_idle;
        
        // Running program state 
        localparam IDLE_STATE = 0;
        localparam DECODE_INSTRUCTION_STATE = 1;
        localparam EXECUTE_INSTRUCTION_STATE = 2;
        localparam EXECUTE_ADDR_INSTRUCTION_STATE = 3;
        localparam EXECUTE_WR_ADDR_INSTRUCTION_STATE = 8;
        localparam EXECUTE_RD_PRE_ACCESS_INSTRUCTION_STATE = 4;
        localparam EXECUTE_RD_ACCESS_INSTRUCTION_STATE = 5;
        localparam EXECUTE_WR_PRE_ACCESS_INSTRUCTION_STATE = 6;
        localparam EXECUTE_WR_ACCESS_INSTRUCTION_STATE = 7;
        localparam ADDRESS_MAPPING_PROT_STATE = 11;
        localparam SEND_REQUEST_PROTOCOL_STATE = 9;
        localparam RECEIVE_REQUEST_PROTOCOL_STATE = 10;
        
        // Synchronization primitive
        // -- read
        assign addr_rd = addr_rd_reg;
        assign data_type_rd = data_type_rd_reg;
        assign rd_ins = rd_ins_reg;
        assign rd_finish = rd_finish_reg;
        // -- write
        assign wr_ins = wr_ins_reg;
        assign data_bus_wr = data_bus_wr_reg;
        assign addr_wr = addr_wr_reg;
        assign data_type_wr = data_type_wr_reg;
        // Decode instruciton
        assign opcode_space = fetch_instruction_reg[OPCODE_SPACE_MSB:OPCODE_SPACE_LSB];
        assign funct10_space = fetch_instruction_reg[FUNCT10_SPACE_MSB:FUNCT10_SPACE_LSB];
        assign funct3_space = fetch_instruction_reg[FUNCT3_SPACE_MSB:FUNCT3_SPACE_LSB];
        assign rd1_space = fetch_instruction_reg[RD_SPACE_MSB:RD_SPACE_LSB];
        assign rs1_space = fetch_instruction_reg[RS1_SPACE_MSB:RS1_SPACE_LSB];
        assign rs2_space = fetch_instruction_reg[RS2_SPACE_MSB:RS2_SPACE_LSB];
        assign immediate_1_space = fetch_instruction_reg[IMM_1_SPACE_MSB:IMM_1_SPACE_LSB];
        assign immediate_2_space = fetch_instruction_reg[IMM_2_SPACE_MSB:IMM_2_SPACE_LSB];
        assign immediate_3_space = fetch_instruction_reg[IMM_3_SPACE_MSB:IMM_3_SPACE_LSB];
        assign immediate_4_space = fetch_instruction_reg[IMM_4_SPACE_MSB:IMM_4_SPACE_LSB];
        assign option_space = fetch_instruction_reg[OPT_SPACE_MSB:OPT_SPACE_LSB];
        assign rd2_space = rs1_space;
        assign rd3_space = rs2_space;
        assign rs3_space = rd1_space;
        // Multi-processor manager
        assign processor_idle = (running_program_state_reg == IDLE_STATE);
        // Register management
        for(genvar index_register = 0; index_register < REGISTER_AMOUNT; index_register = index_register + 1) begin
            assign processor_registers[index_register] = registers_owner[index_register];
        end
        // Protocol management 
        assign protocol_address_mapping = protocol_address_mapping_reg;
        assign data_snd_protocol_per = {rs3_buffer, rs1_buffer};
        assign send_protocol_clk = send_protocol_clk_reg;
        assign receive_protocol_clk = receive_protocol_clk_reg;
        assign amount_snd_byte_protocol = amount_snd_byte_protocol_reg;
        // ALU block
        always @* begin
            case(opcode_space) 
                R_TYPE_ENCODE: begin    
                    alu_operand_1 <= rs1_buffer;
                    alu_operand_2 <= rs2_buffer;
                    alu_opcode <= {funct10_space, opcode_space};
                end
                I_TYPE_ENCODE: begin    
                    alu_operand_1 <= rs1_buffer;
                    alu_operand_2 <= {52'h00, immediate_2_space, immediate_1_space};
                    alu_opcode <= {7'b00, funct3_space, opcode_space};
                end
                LOAD_ENCODE: begin      // result: address of target -> access Data memory to write value
                    alu_operand_1 <= rs1_buffer;
                    alu_operand_2 <= {52'h00, immediate_2_space, immediate_1_space};
                    alu_opcode <= {ADD_ENCODE, R_TYPE_ENCODE};  // Add: Base + offset = Address of target
                end
                STORE_ENCODE: begin
                    alu_operand_1 <= rs1_buffer;
                    alu_operand_2 <= {52'h00, immediate_3_space, immediate_1_space};
                    alu_opcode <= {ADD_ENCODE, R_TYPE_ENCODE};  // Add: Base + offset = Address of target
                end
                J_TYPE_ENCODE: begin
                // Processor doesn't process J-type instruction
                    alu_operand_1 <= 64'h00;
                    alu_operand_2 <= 64'h00;
                    alu_opcode <= 17'b00;
                end
                B_TYPE_ENCODE: begin
                // Processor doesn't process B-type instruction
                    alu_operand_1 <= 64'h00;
                    alu_operand_2 <= 64'h00;
                    alu_opcode <= 17'b00;
                end
                default: begin
                    alu_operand_1 <= 64'h00;
                    alu_operand_2 <= 64'h00;
                    alu_opcode <= 17'b00;
                end
            endcase
        end
        ALU_module #(
                    .OPERAND_WIDTH(DOUBLEWORD_WIDTH)
                    )
        alu_module  (
                    .clk(clk),
                    .operand_1(alu_operand_1),
                    .operand_2(alu_operand_2),
                    .result(alu_result),
                    .op_code(alu_opcode),
                    .alu_trig(alu_use),
                    .alu_idle(alu_idle),
                    .rst_n(rst_n)
                    ); 
        
        always @(posedge clk, negedge rst_n) begin
            if(!rst_n) begin
                running_program_state_reg <= IDLE_STATE;
                // Reset register in processor
                for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
                    registers_owner[i] <= REGISTER_DEFAULT[i];
                end
                // Reset buffer interact ALU block
                alu_use <= 0;
                // Reset buffer interact with RAM 
                addr_rd_reg <= 0;
                rd_ins_reg <= 0;
                data_type_rd_reg <= 0;
                rd_finish_reg <= 0;
                addr_wr_reg <= 0;
                data_bus_wr_reg <= 0;
                wr_ins_reg <= 0;
                data_type_wr_reg <= 0;
                rs1_buffer <= 0;
                rs2_buffer <= 0;
                rs3_buffer <= 0;
                protocol_address_mapping_reg <= NOTHING_MAPPING;
                send_protocol_clk_reg <= 0;
                receive_protocol_clk_reg <= 0;
                amount_snd_byte_protocol_reg <= 0;
            end
            else begin
                case(running_program_state_reg) 
                    IDLE_STATE: begin
                                if(boot_processor) begin
                                    running_program_state_reg <= DECODE_INSTRUCTION_STATE;
                                    
                                    fetch_instruction_reg <= fetch_instruction; 
                                end
                                else running_program_state_reg <= IDLE_STATE;
                                // Reset clk
                                send_protocol_clk_reg <= 0;
                                receive_protocol_clk_reg <= 0;
                            end
                    DECODE_INSTRUCTION_STATE: begin
                            
                                case(opcode_space) 
                                    R_TYPE_ENCODE: begin
                                        // Confirm data
                                        rs1_buffer <= registers_renew[rs1_space];
                                        rs2_buffer <= registers_renew[rs2_space];
                                        alu_use <= 1;
                                        running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                        case(funct10_space) 
//                                            ADD_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SUB_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SLT_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SLTU_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SRL_ENCODE: begin
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SLL_ENCODE: begin
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            AND_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            XOR_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            OR_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            MUL_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            default: begin
//                                                running_program_state_reg <= IDLE_STATE;
//                                            end
//                                        endcase
                                    end
                                    I_TYPE_ENCODE: begin
                                        // Confirm data
                                        rs1_buffer <= registers_renew[rs1_space];
                                        alu_use <= 1;
                                        running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                        case(funct3_space) 
//                                            ADDI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end               
//                                            SLLI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SRLI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end  
//                                            SLTI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            SLTIU_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            XORI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            ORI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end
//                                            ANDI_ENCODE: begin
//                                                // ALU signal here
//                                                alu_use <= 1;
//                                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
//                                            end    
//                                            default: begin
//                                                running_program_state_reg <= IDLE_STATE;
//                                            end
//                                        endcase 
                                    end    
                                    J_TYPE_ENCODE: begin
                                    // Processor don't process J-type instruction
                                    end
                                    B_TYPE_ENCODE: begin
                                    // Processor don't process J-type instruction
                                    end
                                    LOAD_ENCODE: begin
                                        // Confirm data
                                        rs1_buffer <= registers_renew[rs1_space];
                                        alu_use <= 1;
                                        running_program_state_reg <= EXECUTE_ADDR_INSTRUCTION_STATE;
                                    end
                                    STORE_ENCODE: begin
                                        // Confirm data
                                        rs1_buffer <= registers_renew[rs1_space];
                                        alu_use <= 1;
                                        running_program_state_reg <= EXECUTE_WR_ADDR_INSTRUCTION_STATE;
                                    end
                                    SYSTEM_ENCODE: begin
                                    // Processor 2 doesn't process SYSTEM-type intruction
                                    end
                                    PROTOCOL_ENCODE: begin
                                        running_program_state_reg <= ADDRESS_MAPPING_PROT_STATE;
                                        case(funct3_space) 
                                            UART_ENCODE: begin
                                                case(option_space)
                                                    2'b00: begin    // Only receive
                                                        protocol_address_mapping_reg <= UART_RX_MAPPING;
                                                    end
                                                    2'b01: begin    // Only transmit
                                                        protocol_address_mapping_reg <= UART_TX_MAPPING;
                                                        // Confirm data
                                                        rs1_buffer <= registers_renew[rs1_space];
                                                        rs2_buffer <= registers_renew[rs2_space];
                                                        rs3_buffer <= registers_renew[rs3_space];
                                                    end
                                                    2'b10: begin    // Full-duplex
                                                    
                                                    end
                                                    2'b11: begin
                                                    
                                                    end
                                                    default: begin
                                                    
                                                    end
                                                endcase
                                            end
                                            SPI_ENCODE: begin
                                                protocol_address_mapping_reg <= SPI_MAPPING;
                                            end
                                            I2C_ENCODE: begin
                                                protocol_address_mapping_reg <= I2C_MAPPING;
                                            end
                                            default: begin
                                            
                                            end
                                        endcase
                                    end
                                endcase 
                            end
                    EXECUTE_INSTRUCTION_STATE: begin
                                alu_use <= 0;
                                if(alu_idle == 1) begin
                                    registers_owner[rd1_space] <= alu_result;
                                    running_program_state_reg <= IDLE_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                    EXECUTE_ADDR_INSTRUCTION_STATE: begin
                                alu_use <= 0;
                                if(alu_idle == 1) begin
                                    addr_rd_reg <= alu_result;
                                    rd_ins_reg <= 1;
                                    case(funct3_space) 
                                        BYTE_WIDTH_ENCODE: data_type_rd_reg <= BYTE_TYPE_ENCODE;
                                        WORD_WIDTH_ENCODE: data_type_rd_reg <= WORD_TYPE_ENCODE;
                                        DWORD_WIDTH_ENCODE: data_type_rd_reg <= DWORD_TYPE_ENCODE;
                                        default: data_type_rd_reg <= BYTE_TYPE_ENCODE;
                                    endcase
                                    rd_finish_reg <= 0;      
                                    running_program_state_reg <= EXECUTE_RD_PRE_ACCESS_INSTRUCTION_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                                
                            end
                    EXECUTE_RD_PRE_ACCESS_INSTRUCTION_STATE: begin
                                if(rd_access) begin
                                    running_program_state_reg <= EXECUTE_RD_ACCESS_INSTRUCTION_STATE;
                                    rd_ins_reg <= 0;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                    EXECUTE_RD_ACCESS_INSTRUCTION_STATE: begin
                                if(rd_idle) begin
                                    registers_owner[rd1_space] <= data_bus_rd;
                                    rd_finish_reg <= 1;
                                    running_program_state_reg <= IDLE_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                    EXECUTE_WR_ADDR_INSTRUCTION_STATE: begin
                                alu_use <= 0;
                                if(alu_idle == 1) begin
                                    addr_wr_reg <= alu_result;
                                    data_bus_wr_reg <= registers_renew[rs2_space];
                                    wr_ins_reg <= 1;
                                    case(funct3_space) 
                                        BYTE_WIDTH_ENCODE: data_type_wr_reg <= BYTE_TYPE_ENCODE;
                                        WORD_WIDTH_ENCODE: data_type_wr_reg <= WORD_TYPE_ENCODE;
                                        DWORD_WIDTH_ENCODE: data_type_wr_reg <= DWORD_TYPE_ENCODE;
                                        default: data_type_wr_reg <= BYTE_TYPE_ENCODE;
                                    endcase
                                    running_program_state_reg <= EXECUTE_WR_PRE_ACCESS_INSTRUCTION_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                    EXECUTE_WR_PRE_ACCESS_INSTRUCTION_STATE: begin
                                if(wr_access) begin
                                    running_program_state_reg <= EXECUTE_WR_ACCESS_INSTRUCTION_STATE;
                                end
                                else running_program_state_reg <= running_program_state_reg;
                            end
                    EXECUTE_WR_ACCESS_INSTRUCTION_STATE: begin  
                                // Notation of EXECUTE_WR_ACCESS_INSTRUCTION_STATE: Just wait for 1 cycle to confirm stablization of interactive buffer
                                running_program_state_reg <= IDLE_STATE;
                                wr_ins_reg <= 0;
                            end
                    ADDRESS_MAPPING_PROT_STATE: begin
//                        running_program_state_reg <= SEND_REQUEST_PROTOCOL_STATE;
                        case(protocol_address_mapping_reg) 
                            UART_TX_MAPPING: begin
                                if(snd_protocol_available) begin
                                    running_program_state_reg <= IDLE_STATE;
                                    send_protocol_clk_reg <= 1;
                                    amount_snd_byte_protocol_reg <= rs2_buffer[AMOUNT_SND_WIDTH - 1:0];
                                end
                            end
                            UART_RX_MAPPING: begin
                                if(rcv_protocol_available) begin
                                    running_program_state_reg <= IDLE_STATE;
                                    receive_protocol_clk_reg <= 1;
                                    registers_owner[rd1_space] <= data_rcv_protocol_per[63:0];
                                    registers_owner[rd2_space] <= data_rcv_protocol_per[127:64];
                                    registers_owner[rd3_space] <= amount_rcv_byte_protocol;
                                end
                            end
                            SPI_MAPPING: begin
                            
                            end
                            I2C_MAPPING: begin
                            
                            end
                        endcase 
                    end
                    SEND_REQUEST_PROTOCOL_STATE: begin
                    
                    end
                    RECEIVE_REQUEST_PROTOCOL_STATE: begin
                    
                    end            
                endcase
            end
        end
        // Synchronization reigsters block
        always @(posedge clk) begin
            if(synchronization_processor) begin
                for(int i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
                    registers_owner[i] <= registers_renew[i];
                end
            end
        end
        `ifdef DEBUG
        // Debug area
        assign debug_2 = {0, 0};
        `endif
    end
    
    
    endgenerate
    
endmodule

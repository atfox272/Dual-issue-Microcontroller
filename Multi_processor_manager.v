module Multi_processor_manager
    #(
    parameter INSTRUCTION_WIDTH     = 32,   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 1024,
    
    parameter REGISTER_AMOUNT       = 32,
    
    parameter DOUBLEWORD_WIDTH      = 64,
    
    // Program address 
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
    parameter OPT_SPACE_MSB         = 16,               // Option space
    parameter OPT_SPACE_LSB         = 10,               // Option space    
    parameter OPT_SPACE_WIDTH       = OPT_SPACE_MSB - OPT_SPACE_LSB + 1,
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
    // DEVICE MAPPING
    parameter BUS_MODULE_AMOUNT    = 3,
    parameter BUS_MODULE_WIDTH     = $clog2(BUS_MODULE_AMOUNT),
    parameter DATA_MEMORY_MAPPING  = 0,
    parameter PERIPHERAL_MAPPING   = 1,
    parameter GPIO_MAPPING         = 2,
    // NOT MODIFY
    parameter ADDR_WIDTH_PM         = $clog2(PROGRAM_MEMORY_SIZE),
    parameter REG_SPACE_WIDTH       = $clog2(REGISTER_AMOUNT),
    parameter RUNNING_PROGRAM_STATE = 2
    )
    (
    input   wire                                clk,
    
    // Program memory
    input   wire    [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd_pm,
    input   wire                                rd_idle_pm,
    output  wire    [ADDR_WIDTH_PM - 1:0]       addr_rd_pm,
//    output  wire                                rd_ins_pm,
    
    // Processor 1
    input   wire    [1:0]                       main_state,
    output  wire    [INSTRUCTION_WIDTH - 1:0]   fetch_instruction_1,
    output  wire                                boot_processor_1,
    input   wire                                processor_idle_1,
    
    // Processor 2
    output  wire    [INSTRUCTION_WIDTH - 1:0]   fetch_instruction_2,
    output  wire                                boot_processor_2,
    input   wire                                processor_idle_2,
    
    // Registers management
    input   wire    [DOUBLEWORD_WIDTH - 1:0]    registers_renew [0:REGISTER_AMOUNT - 1],
    input   wire                                synchronized_processors,
    output  wire    [REG_SPACE_WIDTH - 1:0]     register_num,
    output  wire                                boot_renew_register_1,
    output  wire                                boot_renew_register_2,
    output  wire    [DOUBLEWORD_WIDTH - 1:0]    ra_register,
    input   wire    [0:REGISTER_AMOUNT - 1]     processing_register_table,
    // Interrupt control
    input   wire                                interrupt_flag_1,
    input   wire                                interrupt_flag_2,
    input   wire                                interrupt_flag_3,
    output  wire                                RETI_1,
    output  wire                                RETI_2,
    output  wire                                RETI_3,
    output  wire                                interrupt_handling_1,
    output  wire                                interrupt_handling_2,
    output  wire                                interrupt_handling_3,
    
    // Hardware supportive instruction
    // -- MICS-MEM (Fence) confirm that the previuos STORE/LOAD instruciton is finished
    input   wire                                rd_idle_dm,
    input   wire                                wr_idle_dm,
    
    input   wire                                rst_n
    );
   
    // Internal register 
    reg [DOUBLEWORD_WIDTH - 1:0]    x1;   // register ra (return address)
    wire[DOUBLEWORD_WIDTH - 1:0]    rs1_regFile;
    wire[DOUBLEWORD_WIDTH - 1:0]    rs2_regFile;
    reg [ADDR_WIDTH_PM - 1:0]       PC;
    logic[ADDR_WIDTH_PM - 1:0]      PC_update_ins_flow;
    logic[ADDR_WIDTH_PM - 1:0]      PC_update_main_flow;
    wire[ADDR_WIDTH_PM - 1:0]       PC_next;
    wire[ADDR_WIDTH_PM - 1:0]       PC_jump_condition;
    wire[ADDR_WIDTH_PM - 1:0]       PC_jump_uncondition;
    // State machine
    reg [4:0] program_state;
    // reg
    reg [DOUBLEWORD_WIDTH - 1:0] IR_2ins;   // Instruction register (2 instructions)
    reg contain_ins;
    wire[REG_SPACE_WIDTH - 1:0] rs1_cur;
    wire[REG_SPACE_WIDTH - 1:0] rs2_cur;
    wire[REG_SPACE_WIDTH - 1:0] rd1_cur;
    wire[REG_SPACE_WIDTH - 1:0] rd2_cur;
    wire[REG_SPACE_WIDTH - 1:0] rd3_cur;
    reg [1:0]                   processor_prev;             // 0-mpm, 1-main, 2-sub (processor_using log)
//    reg rd_ins_pm_reg;
    wire stall_Rtype_condition;
    wire stall_Itype_condition;
    wire stall_LUItype_condition;
    wire stall_STORE_condition;
    wire stall_PROT_condition;
    wire release_stall_Rtype_condition;
    wire release_stall_Itype_condition;
    wire release_stall_LUItype_condition;
    wire release_stall_STORE_condition;
    wire release_stall_PROT_condition;
    wire release_stall_condition;
    // Processor 
    reg boot_processor_1_reg;
    reg boot_processor_2_reg;
    // Interrupt control
    reg RETI_1_reg;
    reg RETI_2_reg;
    reg RETI_3_reg;
    reg interrupt_handling_1_reg;
    reg interrupt_handling_2_reg;
    reg interrupt_handling_3_reg;
    // instruction space
    wire[INSTRUCTION_WIDTH - 1:0]       cur_instruction;
    wire[OPCODE_SPACE_WIDTH - 1:0]      opcode_space;
    wire[FUNCT10_SPACE_WIDTH - 1:0]     funct10_space;
    wire[FUNCT3_SPACE_WIDTH - 1:0]      funct3_space;
    wire[IMM_1_SPACE_WIDTH - 1:0]       immediate_1_space;
    wire[IMM_2_SPACE_WIDTH - 1:0]       immediate_2_space;
    wire[IMM_3_SPACE_WIDTH - 1:0]       immediate_3_space;
    wire[JUMP_OFS_SPACE_WIDTH - 1:0]    jump_offset_space;
    wire[OPT_SPACE_WIDTH - 1:0]         option_space;
    // Registers management
    reg [REG_SPACE_WIDTH - 1:0]         register1_num_reg;
    reg [REG_SPACE_WIDTH - 1:0]         register2_num_reg;
    reg [REG_SPACE_WIDTH - 1:0]         register3_num_reg;
    reg                                 boot_renew_register_1_reg;
    reg                                 boot_renew_register_2_reg;
    // LIFO 
    reg                                 active_stack_clk;
    reg                                 wr_stack_ins;
    reg                                 rd_stack_ins;
    wire                                program_stack_empty;
    reg  [PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 1:0] PC_info_in_stack;    // <program-type> + <PC>
    wire [PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 1:0] PC_info_out_stack;
    wire [PROGRAM_COUNTER_WIDTH - 1:0] program_type_out_stack;
    wire [ADDR_WIDTH_PM - 1:0] PC_out_stack;
    
    localparam INIT_STATE = 1;
    localparam PROGRAM_IDLE_STATE = 0;
    localparam FETCH_INS_STATE = 2;                                 // Fetch instruction
    localparam DISPATCH_INS_STATE = 3;                              // Dispath instruction 
    localparam PARALLEL_BLOCKING_B_TYPE_STATE = 4;                  // Parallel stall
    localparam PARALLEL_BLOCKING_R_TYPE_STATE = 5;
    localparam PARALLEL_BLOCKING_I_TYPE_STATE = 6;
    localparam PARALLEL_BLOCKING_LUI_TYPE_STATE = 18;
    localparam PARALLEL_BLOCKING_STORE_STATE = 7;
    localparam PARALLEL_BLOCKING_PROTOCOL_STATE = 16;
    localparam PARALLEL_BLOCKING_GPIO_STATE = 17;
    localparam PARALLEL_BLOCKING_EX_INTERNAL_STATE = 8;
    localparam EXECUTE_BTYPE_INSTRUCTION_INTERNAL_STATE = 9;        // Execute isntruction
    localparam EXECUTE_JTYPE_INSTRUCTION_INTERNAL_STATE = 10;
    localparam EXECUTE_JALRTYPE_INSTRUCTION_INTERNAL_STATE = 11;
    localparam RESTORE_PC_STATE = 12;
    localparam DETECT_HIGHER_PRIO_PROGRAM_STATE = 13;               // Detect interrupt
    localparam RECOVERY_PC_STATE = 14;
    localparam EXIT_STATE = 15;
    
    assign addr_rd_pm = PC;
    assign PC_next              = PC + 4;
    assign PC_jump_uncondition  = PC + {{37{jump_offset_space[JUMP_OFS_SPACE_WIDTH - 1]}}, jump_offset_space, 2'b00};                   // Sign extend included
    assign PC_jump_condition    = PC + {{50{immediate_3_space[IMM_3_SPACE_WIDTH - 1]}}, immediate_3_space, immediate_1_space, 2'b00};   // Sign extend included
//    assign rd_ins_pm = rd_ins_pm_reg;
    // Dispatch Instruction management
    assign cur_instruction  = (contain_ins) ? IR_2ins[31:0] : IR_2ins[63:32]; 
    assign fetch_instruction_1 = cur_instruction;
    assign fetch_instruction_2 = cur_instruction;
    assign boot_processor_1 = boot_processor_1_reg;
    assign boot_processor_2 = boot_processor_2_reg;
    // Decode Instruction
    assign opcode_space = cur_instruction[OPCODE_SPACE_MSB:OPCODE_SPACE_LSB];
    assign funct10_space = cur_instruction[FUNCT10_SPACE_MSB:FUNCT10_SPACE_LSB];
    assign funct3_space = cur_instruction[FUNCT3_SPACE_MSB:FUNCT3_SPACE_LSB];
    assign rd1_cur = cur_instruction[RD_SPACE_MSB:RD_SPACE_LSB];
    assign rs1_cur = cur_instruction[RS1_SPACE_MSB:RS1_SPACE_LSB];
    assign rs2_cur = cur_instruction[RS2_SPACE_MSB:RS2_SPACE_LSB];
    assign immediate_1_space = cur_instruction[IMM_1_SPACE_MSB:IMM_1_SPACE_LSB];
    assign immediate_2_space = cur_instruction[IMM_2_SPACE_MSB:IMM_2_SPACE_LSB];
    assign immediate_3_space = cur_instruction[IMM_3_SPACE_MSB:IMM_3_SPACE_LSB];
    assign jump_offset_space = cur_instruction[JUMP_OFS_SPACE_MSB:JUMP_OFS_SPACE_LSB];
    assign option_space = cur_instruction[OPT_SPACE_MSB:OPT_SPACE_LSB];
    assign rd2_cur = rs1_cur;
    assign rd3_cur = rs2_cur;
    // Registers File 
    assign rs1_regFile = registers_renew[rs1_cur];
    assign rs2_regFile = registers_renew[rs2_cur];
    // register management 
    assign register_num = register1_num_reg;
    assign boot_renew_register_1 = boot_renew_register_1_reg;
    assign boot_renew_register_2 = boot_renew_register_2_reg;
    assign ra_register = x1;
    // Stall Condition
    assign stall_Rtype_condition = (processing_register_table[rs1_cur] == 1) | 
                                               (processing_register_table[rs2_cur] == 1) |
                                               (processing_register_table[rd1_cur]  == 1);
    assign release_stall_Rtype_condition  = ~stall_Rtype_condition;
                                               
    assign stall_Itype_condition = (processing_register_table[rs1_cur] == 1) |
                                               (processing_register_table[rd1_cur]  == 1);
    assign release_stall_Itype_condition  = ~stall_Itype_condition;
                                                  
    assign stall_LUItype_condition = (processing_register_table[rd1_cur] == 1);
    assign release_stall_LUItype_condition  = ~stall_LUItype_condition;
    
    assign stall_STORE_condition = (processing_register_table[rs1_cur] == 1) |
                                               (processing_register_table[rs2_cur] == 1);
    assign release_stall_STORE_condition  = ~stall_STORE_condition;
    
    assign stall_PROT_condition  = ((processing_register_table[rd1_cur]  == 1) & (option_space[OPT_SPACE_WIDTH - 1] == 0) |     // Receive state  (1 reg)
                                                (stall_Rtype_condition         & (option_space[OPT_SPACE_WIDTH - 1] == 1)));    // Transmit state (3 reg)
    assign release_stall_PROT_condition   = ((processing_register_table[rd1_cur]  == 0) & (option_space[OPT_SPACE_WIDTH - 1] == 0) | 
                                                (release_stall_Rtype_condition          & (option_space[OPT_SPACE_WIDTH - 1] == 1)));
    assign release_stall_condition        = ((release_stall_Rtype_condition & opcode_space == R_TYPE_ENCODE) |
                                                (release_stall_Itype_condition & (opcode_space == I_TYPE_ENCODE | opcode_space == LOAD_ENCODE)) |
                                                (release_stall_LUItype_condition & opcode_space == LUI_TYPE_ENCODE) |
                                                (release_stall_STORE_condition & (opcode_space == STORE_ENCODE | opcode_space == B_TYPE_ENCODE)) |
                                                (synchronized_processors & (opcode_space == JAL_TYPE_ENCODE | opcode_space == JALR_TYPE_ENCODE | opcode_space == J_TYPE_ENCODE | opcode_space == B_TYPE_ENCODE)));
    
    // Interrupt control
    assign RETI_1 = RETI_1_reg;
    assign RETI_2 = RETI_2_reg;
    assign RETI_3 = RETI_3_reg;
    assign interrupt_handling_1 = interrupt_handling_1_reg;
    assign interrupt_handling_2 = interrupt_handling_2_reg;
    assign interrupt_handling_3 = interrupt_handling_3_reg;
    
    // Small ALU (for sign-comparison)
    localparam EQUAL_RESULT     = 0;
    localparam GREATER_RESULT   = 1;
    localparam LESS_RESULT      = 2;
    localparam NOTHING_RESULT   = 3;
    logic [1:0] sign_comparison_result;
    always_comb begin
        if(rs1_regFile[DOUBLEWORD_WIDTH - 1] == rs2_regFile[DOUBLEWORD_WIDTH - 1]) begin    // Same sign
            if(rs1_regFile[DOUBLEWORD_WIDTH - 2:0] == rs2_regFile[DOUBLEWORD_WIDTH - 2:0]) begin
                sign_comparison_result = EQUAL_RESULT;
            end
            else if (rs1_regFile[DOUBLEWORD_WIDTH - 2:0] < rs2_regFile[DOUBLEWORD_WIDTH - 2:0]) begin
                if(rs1_regFile[DOUBLEWORD_WIDTH - 1] == 0) sign_comparison_result = LESS_RESULT;
                else sign_comparison_result = GREATER_RESULT;
            end
            else begin
                if(rs1_regFile[DOUBLEWORD_WIDTH - 1] == 0) sign_comparison_result = GREATER_RESULT;
                else sign_comparison_result = LESS_RESULT;
            end
        end
        else if (rs1_regFile[DOUBLEWORD_WIDTH - 1] == 1) begin                              // rs1 is negative number
            sign_comparison_result = LESS_RESULT;
        end
        else begin                                                                          // rs2 is negative number
            sign_comparison_result = GREATER_RESULT;
        end
    end
    
    // LIFO_out decode
    assign program_type_out_stack = PC_info_out_stack[PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 1 : PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 2];
    assign PC_out_stack = PC_info_out_stack[PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 3:0];
    
    // data_bus format : {prgram_type (2-bit), PC}
    LIFO_module #(
                .DATA_WIDTH(PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM),
                .LIFO_DEPTH(INTERRUPT_BUFFER)
                ) program_buffer (
                .active_clk(active_stack_clk),
                .data_bus_in(PC_info_in_stack),
                .data_bus_out(PC_info_out_stack),
                .wr_ins(wr_stack_ins),
                .rd_ins(rd_stack_ins),
                .empty(program_stack_empty),
                .rst_n(rst_n)
                );     
    
    logic[4:0]                      decode_state;
    logic                           boot_p1_en;
    logic                           boot_p2_en;
    logic                           boot_renew_register_1_en;
    logic                           boot_renew_register_2_en;
    logic                           rd_PC_stack_en;
    logic                           reti_det;
    logic                           IF_update;
    logic[DOUBLEWORD_WIDTH - 1:0]   quick_adder_result;
    logic[BUS_MODULE_WIDTH - 1:0]   bus_module_encode;
    logic                           bus_ready;
    logic                           bus_path_p1;
    logic                           bus_path_p2;
    
    logic       interrupt_handling_1_update_remain;
    logic       interrupt_handling_2_update_remain;
    logic       interrupt_handling_3_update_remain;
    always_comb begin                                   : BUS_MODULE_BLOCK
        // Quick adder to bus mapping
        assign quick_adder_result = rs1_regFile + rs2_regFile;
        // Bus module is higher-order
        assign bus_module_encode = quick_adder_result[DOUBLEWORD_WIDTH - 1:DOUBLEWORD_WIDTH - BUS_MODULE_WIDTH];
        // Data transfer 
        if(opcode_space == STORE_ENCODE | opcode_space == LOAD_ENCODE) begin
            case(bus_module_encode)
                PERIPHERAL_MAPPING: begin   // Only Processor 2
                    bus_ready = processor_idle_2;
                    bus_path_p1 = 0;
                    bus_path_p2 = processor_idle_2;
                end
                DATA_MEMORY_MAPPING: begin
                    bus_ready = processor_idle_1 | processor_idle_2;
                    bus_path_p1 = processor_idle_1;
                    bus_path_p2 = ~processor_idle_1 & processor_idle_2;
                end
                GPIO_MAPPING: begin
                    bus_ready = processor_idle_1;
                    bus_path_p1 = processor_idle_1;
                    bus_path_p2 = 0;
                end
                default: begin
                    bus_ready = processor_idle_1 | processor_idle_2;
                    bus_path_p1 = processor_idle_1;
                    bus_path_p2 = ~processor_idle_1 & processor_idle_2;
                end
            endcase
        end
        else begin
            bus_ready = 0;
            bus_path_p1 = 0;
            bus_path_p2 = 0;
        end
    end     
    
    always_comb begin                                   : DECODE_STAGE_BLOCK
        decode_state = DISPATCH_INS_STATE;
        boot_p1_en = 0;
        boot_p2_en = 0;
        boot_renew_register_1_en = 0;
        boot_renew_register_2_en = 0;
        rd_PC_stack_en = 0;
        reti_det = 0;
        IF_update = 0;
        PC_update_ins_flow =  PC_next;
        case(opcode_space)
            R_TYPE_ENCODE: begin
                if(stall_Rtype_condition | (!processor_idle_1 & !processor_idle_2)) begin
                    decode_state = DISPATCH_INS_STATE;
                    boot_p1_en = 0;
                    boot_p2_en = 0;
                    boot_renew_register_1_en = 0;
                    boot_renew_register_2_en = 0;
                end
                else begin
                    decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                    boot_p1_en = processor_idle_1;
                    boot_p2_en = ~processor_idle_1 & processor_idle_2;
                    boot_renew_register_1_en = processor_idle_1;
                    boot_renew_register_2_en = ~processor_idle_1 & processor_idle_2;
                end
                PC_update_ins_flow = PC_next;
            end
            I_TYPE_ENCODE: begin
                if(stall_Itype_condition | (!processor_idle_1 & !processor_idle_2)) begin
                    decode_state = DISPATCH_INS_STATE;
                    boot_p1_en = 0;
                    boot_p2_en = 0;
                    boot_renew_register_1_en = 0;
                    boot_renew_register_2_en = 0;
                end
                else begin
                    decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                    boot_p1_en = processor_idle_1;
                    boot_p2_en = ~processor_idle_1 & processor_idle_2;
                    boot_renew_register_1_en = processor_idle_1;
                    boot_renew_register_2_en = ~processor_idle_1 & processor_idle_2;
                end
                PC_update_ins_flow = PC_next;
            end
            LUI_TYPE_ENCODE: begin
                if(stall_LUItype_condition | (!processor_idle_1 & !processor_idle_2)) begin
                    decode_state = DISPATCH_INS_STATE;
                    boot_p1_en = 0;
                    boot_p2_en = 0;
                    boot_renew_register_1_en = 0;
                    boot_renew_register_2_en = 0;
                end
                else begin
                    decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                    boot_p1_en = processor_idle_1;
                    boot_p2_en = ~processor_idle_1 & processor_idle_2;
                    boot_renew_register_1_en = processor_idle_1;
                    boot_renew_register_2_en = ~processor_idle_1 & processor_idle_2;
                end
                PC_update_ins_flow = PC_next;
            end
            LOAD_ENCODE: begin
                if(stall_Itype_condition | !bus_ready) begin
                    decode_state = DISPATCH_INS_STATE;
                    boot_p1_en = 0;
                    boot_p2_en = 0;
                    boot_renew_register_1_en = 0;
                    boot_renew_register_2_en = 0;
                end
                else begin
                    decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                    boot_p1_en = bus_path_p1;
                    boot_p2_en = bus_path_p2;
                    boot_renew_register_1_en = bus_path_p1;
                    boot_renew_register_2_en = bus_path_p2;
                end
                PC_update_ins_flow = PC_next;
            end
            STORE_ENCODE: begin
                if(stall_STORE_condition | !bus_ready) begin
                    decode_state = DISPATCH_INS_STATE;
                    boot_p1_en = 0;
                    boot_p2_en = 0;
                end
                else begin
                    decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                    boot_p1_en = bus_path_p1;
                    boot_p2_en = bus_path_p2;
                end
                boot_renew_register_1_en = 0;
                boot_renew_register_2_en = 0;
                PC_update_ins_flow = PC_next;
            end
            B_TYPE_ENCODE: begin
                if(stall_STORE_condition | !synchronized_processors) begin // Same format as STORE instruction
                    decode_state = DISPATCH_INS_STATE;
                end
                else begin
                    decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                end
                boot_p1_en = 0;
                boot_p2_en = 0;
                // Quick compare
                case(funct3_space)
                    BEQ_ENCODE: begin
                        if(sign_comparison_result == EQUAL_RESULT) begin
                            PC_update_ins_flow = PC_jump_condition;
                            IF_update = 1;
                        end
                        else begin
                            PC_update_ins_flow = PC_next;
                            IF_update = 0;
                        end
                    end
                    BNE_ENCODE: begin
                        if(sign_comparison_result != EQUAL_RESULT) begin
                            PC_update_ins_flow = PC_jump_condition;
                            IF_update = 1;
                        end
                        else begin
                            PC_update_ins_flow = PC_next;
                            IF_update = 0;
                        end
                    end
                    BLT_ENCODE: begin
                        if(sign_comparison_result == LESS_RESULT) begin
                            PC_update_ins_flow = PC_jump_condition;
                            IF_update = 1;
                        end
                        else begin
                            PC_update_ins_flow = PC_next;
                            IF_update = 0;
                        end
                    end
                    BGE_ENCODE: begin
                        if(sign_comparison_result == GREATER_RESULT) begin
                            PC_update_ins_flow = PC_jump_condition;
                            IF_update = 1;
                        end
                        else begin
                            PC_update_ins_flow = PC_next;
                            IF_update = 0;
                        end
                    end
                    default: begin
                        PC_update_ins_flow = PC_next;
                        IF_update = 0;
                    end
                endcase
            end
            J_TYPE_ENCODE: begin
                if(synchronized_processors) begin   // Stall
                    decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                end
                else begin
                    decode_state = DISPATCH_INS_STATE;
                end
                boot_p1_en = 0;
                boot_p2_en = 0;
                boot_renew_register_1_en = 0;
                boot_renew_register_2_en = 0;
                IF_update = 1;
                PC_update_ins_flow = PC_jump_uncondition;
            end
            JAL_TYPE_ENCODE: begin
                if(synchronized_processors) begin
                    decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                end
                else begin
                    decode_state = DISPATCH_INS_STATE;
                end
                boot_p1_en = 0;
                boot_p2_en = 0;
                boot_renew_register_1_en = 0;
                boot_renew_register_2_en = 0;
                IF_update = 1;
                PC_update_ins_flow = PC_jump_uncondition;
            end
            JALR_TYPE_ENCODE: begin
                if(synchronized_processors) begin 
                    decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                end
                else begin
                    decode_state = DISPATCH_INS_STATE;
                end
                boot_p1_en = 0;
                boot_p2_en = 0;
                boot_renew_register_1_en = 0;
                boot_renew_register_2_en = 0;
                IF_update = 1;
                PC_update_ins_flow = x1;
            end
            MISC_MEM_ENCODE: begin
                case(funct3_space)
                    FENCE_ENCODE: begin
                        case(processor_prev)
                        0: decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                        1: decode_state = (rd_idle_dm & wr_idle_dm & processor_idle_1) ? DETECT_HIGHER_PRIO_PROGRAM_STATE : DISPATCH_INS_STATE;
                        2: decode_state = (rd_idle_dm & wr_idle_dm & processor_idle_2) ? DETECT_HIGHER_PRIO_PROGRAM_STATE : DISPATCH_INS_STATE;
                        default: decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                        endcase
                    end
                endcase
                PC_update_ins_flow = PC_next;
            end
            SYSTEM_ENCODE: begin
            case (funct3_space) 
                RETI_ENCODE: begin
                    reti_det = 1;
                    IF_update = 1;
                    if(program_stack_empty) begin   // Return in main loop
                        decode_state = EXIT_STATE;  
                        PC_update_ins_flow = MAIN_PROGRAM_ADDR;
                        rd_PC_stack_en = 0;
                    end
                    else begin
                        decode_state = RECOVERY_PC_STATE;
                        PC_update_ins_flow = PC_out_stack;
                        rd_PC_stack_en = 1;
                    end
                end
                BREAK_ENCODE: begin
                // Send all data in Memory and Registers to UART_TX
                end
                DEBUG_ENCODE: begin
                    PC_update_ins_flow = MAIN_PROGRAM_ADDR;        // Return init addr of main program 
                    if(processor_idle_1) begin
                        decode_state = EXIT_STATE;
                        boot_p1_en = 1;
                    end
                    else begin
                        decode_state = DISPATCH_INS_STATE;
                        boot_p1_en = 0;
                    end
                end
                EXIT_ENCODE: begin
                //
                end
                default: begin
                
                end
            endcase
            end
//            GPIO_ENCODE: begin
            
//            end
            PROTOCOL_ENCODE: begin    // Convert to LOAD/STORE instruction (add quick-adder to DISPATCH to Processor2)
                decode_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
                PC_update_ins_flow = PC_next;
            end
            default: begin
                decode_state = DISPATCH_INS_STATE;
                boot_p1_en = 0;
                boot_p2_en = 0;
                boot_renew_register_1_en = 0;
                boot_renew_register_2_en = 0;
                rd_PC_stack_en = 0;
                reti_det = 0;
                IF_update = 0;
                PC_update_ins_flow = PC_next;
            end
        endcase
        
        case(program_type_out_stack) 
            MAIN_PROGRAM_ENCODE: begin
                interrupt_handling_1_update_remain = 0;
                interrupt_handling_2_update_remain = 0;
                interrupt_handling_3_update_remain = 0;
            end
            INT1_PROGRAM_ENCODE: begin
                interrupt_handling_1_update_remain = 1;
                interrupt_handling_2_update_remain = 0;
                interrupt_handling_3_update_remain = 0;
            end
            INT2_PROGRAM_ENCODE: begin
                interrupt_handling_1_update_remain = 0;
                interrupt_handling_2_update_remain = 1;
                interrupt_handling_3_update_remain = 0;
            end
            INT3_PROGRAM_ENCODE: begin
                interrupt_handling_1_update_remain = 0;
                interrupt_handling_2_update_remain = 0;
                interrupt_handling_3_update_remain = 1;
            end
            default: begin
                interrupt_handling_1_update_remain = 0;
                interrupt_handling_2_update_remain = 0;
                interrupt_handling_3_update_remain = 0;
            end
        endcase
        
        
    end
    
    logic[4:0]  detect_interrupt_state;
    logic       renew_IR_en;  // Renew instruction register enable 
    logic       restore_PC_en;
    logic[1:0]  cur_program_encode;
    logic       contain_ins_update;
    logic       interrupt_handling_1_update_new;
    logic       interrupt_handling_2_update_new;
    logic       interrupt_handling_3_update_new;
    
    always_comb begin
        renew_IR_en = (contain_ins == 0 | IF_update);
        // Default
        detect_interrupt_state = DETECT_HIGHER_PRIO_PROGRAM_STATE;
        PC_update_main_flow = PC_update_ins_flow;
        restore_PC_en = 0;
        contain_ins_update = contain_ins;
        interrupt_handling_1_update_new = interrupt_handling_1_reg;
        interrupt_handling_2_update_new = interrupt_handling_2_reg;
        interrupt_handling_3_update_new = interrupt_handling_3_reg;
        if(interrupt_handling_1) begin
            detect_interrupt_state = (renew_IR_en) ? FETCH_INS_STATE : DISPATCH_INS_STATE;
            PC_update_main_flow = PC_update_ins_flow;
            contain_ins_update = renew_IR_en;
        end
        else if(interrupt_handling_2) begin
            if(interrupt_flag_1) begin
                detect_interrupt_state = RESTORE_PC_STATE;
                // Assign Interrupt address
                PC_update_main_flow = INT1_PROGRAM_ADDR;
                cur_program_encode = INT2_PROGRAM_ENCODE[1:0];
                restore_PC_en <= 1;
                // Interrupt control
                interrupt_handling_1_update_new = 1;
                interrupt_handling_2_update_new = 0;
                interrupt_handling_3_update_new = 0;
                // Reset IF_buf_2ins
                contain_ins_update = 1;
            end
            else begin
               detect_interrupt_state = (renew_IR_en) ? FETCH_INS_STATE : DISPATCH_INS_STATE;
                PC_update_main_flow = PC_update_ins_flow;
                contain_ins_update = renew_IR_en;
            end
        end
        else if(interrupt_handling_3) begin
            if(interrupt_flag_1) begin
                detect_interrupt_state = RESTORE_PC_STATE;
                // Assign Interrupt address
                PC_update_main_flow = INT1_PROGRAM_ADDR;
                cur_program_encode = INT3_PROGRAM_ENCODE[1:0];
                restore_PC_en <= 1;
                // Interrupt control
                interrupt_handling_1_update_new = 1;
                interrupt_handling_2_update_new = 0;
                interrupt_handling_3_update_new = 0;
                // Reset IR_buf_2ins
                contain_ins_update = 1;
            end
            else if(interrupt_flag_2) begin
                detect_interrupt_state = RESTORE_PC_STATE;
                // Assign Interrupt address
                PC_update_main_flow = INT2_PROGRAM_ADDR;
                cur_program_encode = INT3_PROGRAM_ENCODE[1:0];
                restore_PC_en <= 1;
                // Interrupt control
                interrupt_handling_1_update_new = 0;
                interrupt_handling_2_update_new = 1;
                interrupt_handling_3_update_new = 0;
                // Reset IF_buf_2ins
                contain_ins_update = 1;
            end
            else begin
               detect_interrupt_state = (renew_IR_en) ? FETCH_INS_STATE : DISPATCH_INS_STATE;
                PC_update_main_flow = PC_update_ins_flow;
                contain_ins_update = renew_IR_en;
            end
        end
        else begin      // In main program
            if(interrupt_flag_1) begin
                detect_interrupt_state = RESTORE_PC_STATE;
                // Assign Interrupt address
                PC_update_main_flow = INT1_PROGRAM_ADDR;
                cur_program_encode = MAIN_PROGRAM_ENCODE[1:0];
                restore_PC_en <= 1;
                // Interrupt control
                interrupt_handling_1_update_new = 1;
                interrupt_handling_2_update_new = 0;
                interrupt_handling_3_update_new = 0;
                // Reset IF_buf_2ins
                contain_ins_update = 1;
            end
            else if(interrupt_flag_2) begin
                detect_interrupt_state = RESTORE_PC_STATE;
                // Assign Interrupt address
                PC_update_main_flow = INT2_PROGRAM_ADDR;
                cur_program_encode = MAIN_PROGRAM_ENCODE[1:0];
                restore_PC_en <= 1;
                // Interrupt control
                interrupt_handling_1_update_new = 0;
                interrupt_handling_2_update_new = 1;
                interrupt_handling_3_update_new = 0;
                // Reset IF_buf_2ins
                contain_ins_update = 1;
            end
            else if(interrupt_flag_3) begin
                detect_interrupt_state = RESTORE_PC_STATE;
                // Assign Interrupt address
                PC_update_main_flow = INT3_PROGRAM_ADDR;
                cur_program_encode = MAIN_PROGRAM_ENCODE[1:0];
                restore_PC_en <= 1;
                // Interrupt control
                interrupt_handling_1_update_new = 0;
                interrupt_handling_2_update_new = 0;
                interrupt_handling_3_update_new = 1;
                // Reset IF_buf_2ins
                contain_ins_update = 1;
            end
            else begin
                detect_interrupt_state = (renew_IR_en) ? FETCH_INS_STATE : DISPATCH_INS_STATE;
                PC_update_main_flow = PC_update_ins_flow;
                contain_ins_update = renew_IR_en;
            end
        end
        
    end
    
    always_comb begin
    
    
    end
    
    always @(posedge clk) begin
        if(!rst_n) begin
            program_state <= INIT_STATE;
            // Init program
            PC <= MAIN_PROGRAM_ADDR;
            // Init reg
            x1 <= 0;
            contain_ins <= 0;
//            rd_ins_pm_reg <= 0;
            boot_processor_1_reg <= 0;
            boot_processor_2_reg <= 0;
            active_stack_clk <= 0;
            PC_info_in_stack <= 0;
            wr_stack_ins <= 0;
            rd_stack_ins <= 0;
            register1_num_reg <= 0;
            register2_num_reg <= 0;
            register3_num_reg <= 0;
            boot_renew_register_1_reg <= 0;
            boot_renew_register_2_reg <= 0;
            RETI_1_reg <= 0;
            RETI_2_reg <= 0;
            RETI_3_reg <= 0;
            interrupt_handling_1_reg <= 0;
            interrupt_handling_2_reg <= 0;
            interrupt_handling_3_reg <= 0;
            // Processor_using log
            processor_prev <= 0;    // 0-mpm, 1-main processor, 2-sub processor (initial value = 0)
        end
        else begin
        case(program_state)
            INIT_STATE: begin
                program_state <= (main_state == RUNNING_PROGRAM_STATE) ? PROGRAM_IDLE_STATE : program_state;
            end
            PROGRAM_IDLE_STATE: begin
                program_state <= (contain_ins) ? DISPATCH_INS_STATE : FETCH_INS_STATE;
                contain_ins <= ~contain_ins;
                // Recovery state
                active_stack_clk <= 0;
                rd_stack_ins <= 0;
                wr_stack_ins <= 0;
            end
            FETCH_INS_STATE: begin
                program_state <= DISPATCH_INS_STATE;
                IR_2ins <= data_bus_rd_pm;
                // Recovery state of Stack 
                active_stack_clk <= 0;
                rd_stack_ins <= 0;
                wr_stack_ins <= 0;
            end
            DISPATCH_INS_STATE: begin
                program_state <= decode_state;
                boot_processor_1_reg <= boot_p1_en;
                boot_processor_2_reg <= boot_p2_en;
                boot_renew_register_1_reg <= boot_renew_register_1_en;
                boot_renew_register_2_reg <= boot_renew_register_2_en;
                register1_num_reg <= rd1_cur;
                rd_stack_ins <= rd_PC_stack_en;
                if(reti_det) begin        // Renew interrupt
                    interrupt_handling_1_reg <= interrupt_handling_1_update_remain;
                    interrupt_handling_2_reg <= interrupt_handling_2_update_remain;
                    interrupt_handling_3_reg <= interrupt_handling_3_update_remain;
                    RETI_1_reg <= (interrupt_handling_1) ? 1'b1 : 1'b0;
                    RETI_2_reg <= (interrupt_handling_2) ? 1'b1 : 1'b0;
                    RETI_3_reg <= (interrupt_handling_3) ? 1'b1 : 1'b0;
                    // Quick update PC (Force return from interrupt0
                    PC <= PC_update_ins_flow;
                end
            end
            DETECT_HIGHER_PRIO_PROGRAM_STATE: begin
                program_state <= detect_interrupt_state;
                PC <= PC_update_main_flow; 
                x1 <= (opcode_space == JAL_TYPE_ENCODE) ? PC_next : x1;
                processor_prev <= (boot_processor_1_reg) ? 1 : (boot_processor_2_reg) ? 2 : 0;
                if(restore_PC_en) begin
                    PC_info_in_stack <= {cur_program_encode, PC};
                    wr_stack_ins <= 1;
                end
                contain_ins <= contain_ins_update;
                interrupt_handling_1_reg <= interrupt_handling_1_update_new;
                interrupt_handling_2_reg <= interrupt_handling_2_update_new;
                interrupt_handling_3_reg <= interrupt_handling_3_update_new;
                // Reset boot
                boot_processor_1_reg <= 0;
                boot_renew_register_1_reg <= 0;
                boot_processor_2_reg <= 0;
                boot_renew_register_2_reg <= 0;
            end
            RECOVERY_PC_STATE: begin
                program_state <= PROGRAM_IDLE_STATE;
                active_stack_clk <= 1;
                // Recovery RETI state
                RETI_1_reg <= 0;
                RETI_2_reg <= 0;
                RETI_3_reg <= 0;
            end
            RESTORE_PC_STATE: begin
                program_state <= FETCH_INS_STATE;
                // Write PC to Stack
                active_stack_clk <= 1;
            end
            EXIT_STATE: begin
                
                // Reset clk
                boot_processor_1_reg <= 0;
                boot_processor_2_reg <= 0;
            end
            default: begin
            
            end
        endcase
        end
    end
endmodule

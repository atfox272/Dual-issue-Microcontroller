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
    wire[ADDR_WIDTH_PM - 1:0]       PC_next;
    wire[ADDR_WIDTH_PM - 1:0]       PC_jump_condition;
    wire[ADDR_WIDTH_PM - 1:0]       PC_jump_uncondition;
    // State machine
    reg [4:0] program_state;
    // reg
    reg [DOUBLEWORD_WIDTH - 1:0] _2_instructions_buf;
    reg contain_ins;
    wire[REG_SPACE_WIDTH - 1:0] rs1_cur;
    wire[REG_SPACE_WIDTH - 1:0] rs2_cur;
    wire[REG_SPACE_WIDTH - 1:0] rd1_cur;
    wire[REG_SPACE_WIDTH - 1:0] rd2_cur;
    wire[REG_SPACE_WIDTH - 1:0] rd3_cur;
    reg [1:0]                   processor_prev;             // 0-mpm, 1-main, 2-sub (processor_using log)
//    reg rd_ins_pm_reg;
    wire parallel_blocking_Rtype_condition;
    wire parallel_blocking_Itype_condition;
    wire parallel_blocking_LUItype_condition;
    wire parallel_blocking_STORE_condition;
    wire parallel_blocking_PROT_condition;
    wire release_blocking_Rtype_condition;
    wire release_blocking_Itype_condition;
    wire release_blocking_LUItype_condition;
    wire release_blocking_STORE_condition;
    wire release_blocking_PROT_condition;
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
    localparam PARALLEL_BLOCKING_B_TYPE_STATE = 4;                  // Parallel blocking
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
    assign cur_instruction  = (contain_ins) ? _2_instructions_buf[31:0] : _2_instructions_buf[63:32]; 
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
    // Condition
    assign parallel_blocking_Rtype_condition = (processing_register_table[rs1_cur] == 1) | 
                                               (processing_register_table[rs2_cur] == 1) |
                                               (processing_register_table[rd1_cur]  == 1);
    assign release_blocking_Rtype_condition  = ~parallel_blocking_Rtype_condition;
                                               
    assign parallel_blocking_Itype_condition = (processing_register_table[rs1_cur] == 1) |
                                               (processing_register_table[rd1_cur]  == 1);
    assign release_blocking_Itype_condition  = ~parallel_blocking_Itype_condition;
                                                  
    assign parallel_blocking_LUItype_condition = (processing_register_table[rd1_cur] == 1);
    assign release_blocking_LUItype_condition  = ~parallel_blocking_LUItype_condition;
    
    assign parallel_blocking_STORE_condition = (processing_register_table[rs1_cur] == 1) |
                                               (processing_register_table[rs2_cur] == 1);
    assign release_blocking_STORE_condition  = ~parallel_blocking_STORE_condition;
    
    assign parallel_blocking_PROT_condition  = ((processing_register_table[rd1_cur]  == 1) & (option_space[OPT_SPACE_WIDTH - 1] == 0) |     // Receive state  (1 reg)
                                                (parallel_blocking_Rtype_condition         & (option_space[OPT_SPACE_WIDTH - 1] == 1)));    // Transmit state (3 reg)
    assign release_blocking_PROT_condition   = ((processing_register_table[rd1_cur]  == 0) & (option_space[OPT_SPACE_WIDTH - 1] == 0) | 
                                                (release_blocking_Rtype_condition          & (option_space[OPT_SPACE_WIDTH - 1] == 1)));

    
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
            processor_prev <= 3;    // 0-mpm, 1-main processor, 2-sub processor (initial value = 3)
        end
        else begin
        case(program_state)
            INIT_STATE: begin
                if(main_state == RUNNING_PROGRAM_STATE) begin
                    program_state <= PROGRAM_IDLE_STATE;
                end
                else program_state <= INIT_STATE;
            end
            PROGRAM_IDLE_STATE: begin
                if(contain_ins == 0) begin
                    program_state <= FETCH_INS_STATE;
                    contain_ins <= 1;
                    // Request 
//                    rd_ins_pm_reg <= 1;
                end
                else begin
                    program_state <= DISPATCH_INS_STATE;
                    contain_ins <= contain_ins - 1;
                end
                // Recovery state
                active_stack_clk <= 0;
                rd_stack_ins <= 0;
                wr_stack_ins <= 0;
            end
            FETCH_INS_STATE: begin
                // Read Instruction
                program_state <= DISPATCH_INS_STATE;
                _2_instructions_buf <= data_bus_rd_pm;
                // Recovery state of Stack 
                active_stack_clk <= 0;
                rd_stack_ins <= 0;
                wr_stack_ins <= 0;
            end
            DISPATCH_INS_STATE: begin
                case(opcode_space) 
                    R_TYPE_ENCODE: begin
                        if(parallel_blocking_Rtype_condition) begin
                            program_state <= PARALLEL_BLOCKING_R_TYPE_STATE;
                        end
                        else begin                             
                            if(processor_idle_1) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_1_reg <= 1;
                                boot_renew_register_1_reg <= 1;
                                // Log 
                                processor_prev <= 1;    // 1-main processor
                            end
                            else if(processor_idle_2)begin  
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_2_reg <= 1;
                                boot_renew_register_2_reg <= 1;
                                // Log 
                                processor_prev <= 2;    // 2-sub processor
                            end
                        end
                        register1_num_reg <= rd1_cur;
                    end
                    I_TYPE_ENCODE: begin
                        if(parallel_blocking_Itype_condition) begin
                            program_state <= PARALLEL_BLOCKING_I_TYPE_STATE;
                        end
                        else begin
                            if(processor_idle_1) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_1_reg <= 1;
                                boot_renew_register_1_reg <= 1;
                                // Log 
                                processor_prev <= 1;    // 1-main processor
                            end
                            else if(processor_idle_2)begin  
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_2_reg <= 1;
                                boot_renew_register_2_reg <= 1;
                                // Log 
                                processor_prev <= 2;    // 2-sub processor
                            end 
                        end
                        register1_num_reg <= rd1_cur;
                    end
                    LUI_TYPE_ENCODE: begin
                        if(parallel_blocking_LUItype_condition) begin
                            program_state <= PARALLEL_BLOCKING_LUI_TYPE_STATE;
                        end
                        else begin
                            if(processor_idle_1) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_1_reg <= 1;
                                boot_renew_register_1_reg <= 1;
                                // Log 
                                processor_prev <= 1;    // 1-main processor
                            end
                            else if(processor_idle_2)begin  
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_2_reg <= 1;
                                boot_renew_register_2_reg <= 1;
                                // Log 
                                processor_prev <= 2;    // 2-sub processor
                            end 
                        end
                        register1_num_reg <= rd1_cur;
                    end
                    LOAD_ENCODE: begin
                        if(parallel_blocking_Itype_condition) begin
                            program_state <= PARALLEL_BLOCKING_I_TYPE_STATE;
                        end
                        else begin 
                            if(processor_idle_1) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_1_reg <= 1;
                                boot_renew_register_1_reg <= 1;
                                // Log 
                                processor_prev <= 1;    // 1-main processor
                            end
                            else if(processor_idle_2)begin  
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_2_reg <= 1;
                                boot_renew_register_2_reg <= 1;
                                // Log 
                                processor_prev <= 2;    // 2-sub processor
                            end
                        end
                        register1_num_reg <= rd1_cur;
                    end
                    STORE_ENCODE: begin
                        if(parallel_blocking_STORE_condition) begin
                            program_state <= PARALLEL_BLOCKING_STORE_STATE;
                        end
                        else begin 
                            if(processor_idle_1) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_1_reg <= 1;
                                // Log 
                                processor_prev <= 1;    // 1-main processor
                            end
                            else if(processor_idle_2)begin  
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_2_reg <= 1;
                                // Log 
                                processor_prev <= 2;    // 2-sub processor
                            end
                        end
                    end
                    MISC_MEM_ENCODE: begin
                        case(funct3_space)
                            FENCE_ENCODE: begin
                                case(processor_prev)
                                    0: begin            // Multi-processor manager (Skip this case)
                                        program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                        PC <= PC_next;
                                    end
                                    1: begin            // Main processor   (waiting for "Data memory is IDLE" & "Previous processor is IDLE")
                                        if(rd_idle_dm & wr_idle_dm & processor_idle_1) begin
                                            program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                            PC <= PC_next;
                                        end
                                    end
                                    2: begin            // Sub processor    (waiting for "Data memory is IDLE" & "Previous processor is IDLE")
                                        if(rd_idle_dm & wr_idle_dm & processor_idle_2) begin
                                            program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                            PC <= PC_next;
                                        end
                                    end
                                    default: begin      // Initial value (Skip this case)
                                        program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                        PC <= PC_next;
                                    end
                                endcase
                            end
                            default: begin
                            
                            end
                        endcase
                    end
                    // Execute immediate
                    B_TYPE_ENCODE: begin
                        if(parallel_blocking_STORE_condition) begin // Same format as STORE instruction
                            program_state <= PARALLEL_BLOCKING_B_TYPE_STATE;
                        end
                        else program_state <= EXECUTE_BTYPE_INSTRUCTION_INTERNAL_STATE;
                        
                    end
                    J_TYPE_ENCODE: begin
                        // PC-relative
                        if(synchronized_processors) begin
                            program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                            x1 <= (opcode_space == JAL_TYPE_ENCODE) ? PC_next : x1;
                            PC <= PC_jump_uncondition;
                            contain_ins <= 0;
                        end
                        // Log 
                        processor_prev <= 0;    // 0-mpm
                    end
                    JAL_TYPE_ENCODE: begin
                        // PC-relative                      
                        if(synchronized_processors) begin
                            program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                            x1 <= (opcode_space == JAL_TYPE_ENCODE) ? PC_next : x1;
                            PC <= PC_jump_uncondition;
                            contain_ins <= 0;
                        end
                        // Log 
                        processor_prev <= 0;    // 0-mpm
                    end
                    JALR_TYPE_ENCODE: begin
//                        program_state <= EXECUTE_JALRTYPE_INSTRUCTION_INTERNAL_STATE;
                        if(synchronized_processors) begin
                            program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                            PC <= x1;
                            contain_ins <= 0;
                        end
                        // Log 
                        processor_prev <= 0;    // 0-mpm
                    end
                    SYSTEM_ENCODE: begin
                        case (funct3_space) 
                            RETI_ENCODE: begin
                                if(program_stack_empty) begin
                                    program_state <= EXIT_STATE;
                                end
                                else begin
                                    program_state <= RECOVERY_PC_STATE;
                                    PC <= PC_out_stack;
                                    rd_stack_ins <= 1;
                                    // Reset Instruction buffer
                                    contain_ins <= 0;
                                    case(program_type_out_stack) 
                                        MAIN_PROGRAM_ENCODE: begin
                                            interrupt_handling_1_reg <= 0;
                                            interrupt_handling_2_reg <= 0;
                                            interrupt_handling_3_reg <= 0;
                                        end
                                        INT1_PROGRAM_ENCODE: begin
                                            interrupt_handling_1_reg <= 1;
                                            interrupt_handling_2_reg <= 0;
                                            interrupt_handling_3_reg <= 0;
                                        end
                                        INT2_PROGRAM_ENCODE: begin
                                            interrupt_handling_1_reg <= 0;
                                            interrupt_handling_2_reg <= 1;
                                            interrupt_handling_3_reg <= 0;
                                        end
                                        INT3_PROGRAM_ENCODE: begin
                                            interrupt_handling_1_reg <= 0;
                                            interrupt_handling_2_reg <= 0;
                                            interrupt_handling_3_reg <= 1;
                                        end
                                        default: begin
                                        
                                        end
                                    endcase 
                                    RETI_1_reg <= (interrupt_handling_1) ? 1'b1 : 1'b0;
                                    RETI_2_reg <= (interrupt_handling_2) ? 1'b1 : 1'b0;
                                    RETI_3_reg <= (interrupt_handling_3) ? 1'b1 : 1'b0;
                                end
                            end
                            BREAK_ENCODE: begin
                            // Send all data in Memory and Registers to UART_TX
                            end
                            DEBUG_ENCODE: begin
                                if(processor_idle_1) begin
                                    program_state <= EXIT_STATE;
                                    PC <= MAIN_PROGRAM_ADDR;        // Return init addr of main program 
                                    // Boot
                                    boot_processor_1_reg <= 1;
                                    // Log 
                                    processor_prev <= 3;            // 3-mpm
                                end
                            end
                            EXIT_ENCODE: begin
                            //
                            end
                            default: begin
                            
                            end
                        endcase
                    end
                    PROTOCOL_ENCODE: begin
                        case(funct3_space)
                            UART_ENCODE: begin
                                if(parallel_blocking_PROT_condition) begin          
                                    program_state <= PARALLEL_BLOCKING_PROTOCOL_STATE;
                                end
                                else begin 
                                    if(processor_idle_2) begin
                                        program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                        PC <= PC_next;
                                        // Boot
                                        boot_processor_2_reg <= 1;
                                        // Log 
                                        processor_prev <= 2;
                                        // Boot_renew_register
                                        if(option_space[OPT_SPACE_WIDTH - 1] == 0) begin  // (option_space_msb == 0) -> Receive data 
                                            register1_num_reg <= rd1_cur;
                                            boot_renew_register_2_reg <= 1;
                                        end
                                    end
                                end
                            end
                            SPI_ENCODE: begin
                            
                            end
                            I2C_ENCODE: begin
                            
                            end
                            default: begin
                            
                            end
                        endcase
                    end
                    GPIO_ENCODE: begin
                        if(parallel_blocking_Rtype_condition) begin
                            program_state <= PARALLEL_BLOCKING_GPIO_STATE;
                        end
                        else begin
                            if(processor_idle_1) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_1_reg <= 1;
                                boot_renew_register_1_reg <= 1;
                                // Log 
                                processor_prev <= 1;    // 1-main processor
                            end
                        end
                        register1_num_reg <= rd1_cur;        
                    end
                    default: program_state <= program_state;
                endcase 
            end
            PARALLEL_BLOCKING_R_TYPE_STATE: begin
                if(release_blocking_Rtype_condition) begin
                    if(processor_idle_1) begin
                        program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                        PC <= PC_next;
                        // Boot
                        boot_processor_1_reg <= 1;
                        boot_renew_register_1_reg <= 1;
                        // Log 
                        processor_prev <= 1;    // 1-main processor
                    end
                    else if(processor_idle_2)begin  
                        program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                        PC <= PC_next;
                        // Boot
                        boot_processor_2_reg <= 1;
                        boot_renew_register_2_reg <= 1;
                        // Log 
                        processor_prev <= 2;    // 2-sub processor
                    end
                end
            end
            PARALLEL_BLOCKING_I_TYPE_STATE: begin
                    if(release_blocking_Itype_condition) begin
                        if(processor_idle_1) begin
                            program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                            PC <= PC_next;
                            // Boot
                            boot_processor_1_reg <= 1;
                            boot_renew_register_1_reg <= 1;
                            // Log 
                            processor_prev <= 1;    // 1-main processor
                        end
                        else if(processor_idle_2)begin  
                            program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                            PC <= PC_next;
                            // Boot
                            boot_processor_2_reg <= 1;
                            boot_renew_register_2_reg <= 1;
                            // Log 
                            processor_prev <= 2;    // 2-sub processor
                        end
                    end
            end
            PARALLEL_BLOCKING_LUI_TYPE_STATE: begin
                if(release_blocking_LUItype_condition) begin
                        if(processor_idle_1) begin
                            program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                            PC <= PC_next;
                            // Boot
                            boot_processor_1_reg <= 1;
                            boot_renew_register_1_reg <= 1;
                            // Log 
                            processor_prev <= 1;    // 1-main processor
                        end
                        else if(processor_idle_2)begin  
                            program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                            PC <= PC_next;
                            // Boot
                            boot_processor_2_reg <= 1;
                            boot_renew_register_2_reg <= 1;
                            // Log 
                            processor_prev <= 2;    // 2-sub processor
                        end
                    end
            end 
            PARALLEL_BLOCKING_STORE_STATE: begin
                        if(release_blocking_STORE_condition) begin
//                                program_state <= FETCH_INSTRUCTION_STATE;
                            if(processor_idle_1) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_1_reg <= 1;
//                                boot_renew_register_1_reg <= 1;
                                // Log 
                                processor_prev <= 1;    // 1-main processor
                            end
                            else if(processor_idle_2)begin  
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next; 
                                // Boot
                                boot_processor_2_reg <= 1;
//                                boot_renew_register_2_reg <= 1;
                                // Log 
                                processor_prev <= 2;    // 2-sub processor
                            end
                        end
            end
            PARALLEL_BLOCKING_B_TYPE_STATE: begin
                if(release_blocking_STORE_condition) begin
                    program_state <= EXECUTE_BTYPE_INSTRUCTION_INTERNAL_STATE;
                end
            end
            PARALLEL_BLOCKING_PROTOCOL_STATE: begin
                if(release_blocking_PROT_condition) begin
                    if(processor_idle_2) begin
                        program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                        PC <= PC_next;
                        // Boot
                        boot_processor_2_reg <= 1;
                        // Log 
                        processor_prev <= 2;
                        // Boot_renew_register
                        if(option_space[OPT_SPACE_WIDTH - 1] == 0) begin  // (option_space_msb == 0) -> Receive data 
                            register1_num_reg <= rd1_cur;
                            boot_renew_register_2_reg <= 1;
                        end
                    end
                end
            end
            PARALLEL_BLOCKING_GPIO_STATE: begin
                        if(release_blocking_Rtype_condition) begin
                            if(processor_idle_1) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                                // Boot
                                boot_processor_1_reg <= 1;
                                boot_renew_register_1_reg <= 1;
                                // Log 
                                processor_prev <= 1;    // 1-main processor
                            end
                        end
            end
            EXECUTE_BTYPE_INSTRUCTION_INTERNAL_STATE: begin
                if(synchronized_processors) begin
                    case(funct3_space)
                        BEQ_ENCODE: begin
                            if(sign_comparison_result == EQUAL_RESULT) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_jump_condition;
                            end
                            else begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next; 
                            end 
                        end
                        BNE_ENCODE: begin
                            if(sign_comparison_result != EQUAL_RESULT) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_jump_condition;
                            end
                            else begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                            end 
                        end
                        BLT_ENCODE: begin
                            if(sign_comparison_result == LESS_RESULT) begin 
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_jump_condition;
                            end
                            else begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                            end 
                        end
                        BGE_ENCODE: begin
                            if(sign_comparison_result == GREATER_RESULT) begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_jump_condition;
                            end
                            else begin
                                program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                                PC <= PC_next;
                            end 
                        end
                        default: begin
                        
                        end
                    endcase
                    contain_ins <= 0;
                end
                // Log 
                processor_prev <= 0;    // 0-mpm
            end
            EXECUTE_JTYPE_INSTRUCTION_INTERNAL_STATE: begin
                if(synchronized_processors) begin
                    program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                    x1 <= (opcode_space == JAL_TYPE_ENCODE) ? PC_next : x1;
                    PC <= PC + jump_offset_space[ADDR_WIDTH_PM - 1:0];
                    contain_ins <= 0;
                end
                // Log 
                processor_prev <= 0;    // 0-mpm
            end
            EXECUTE_JALRTYPE_INSTRUCTION_INTERNAL_STATE: begin
                if(synchronized_processors) begin
                    program_state <= DETECT_HIGHER_PRIO_PROGRAM_STATE;
                    PC <= x1;
                    contain_ins <= 0;
                end
                // Log 
                processor_prev <= 0;    // 0-mpm
            end
            DETECT_HIGHER_PRIO_PROGRAM_STATE: begin
                // Reset boot
                boot_processor_1_reg <= 0;
                boot_renew_register_1_reg <= 0;
                boot_processor_2_reg <= 0;
                boot_renew_register_2_reg <= 0;
                if(interrupt_handling_1) begin
                    if(contain_ins) begin
                        program_state <= DISPATCH_INS_STATE;
                        // 
                        contain_ins <= contain_ins - 1;
                    end
                    else begin
                        program_state <= FETCH_INS_STATE;
                        // Request Instruction
                        contain_ins <= 1;
//                        rd_ins_pm_reg <= 1;
                    end
                end
                else if(interrupt_handling_2) begin
                    if(interrupt_flag_1) begin
                        program_state <= RESTORE_PC_STATE;
                        // Assign Interrupt address
                        PC <= INT1_PROGRAM_ADDR;
                        // Prepare data to push in stack
                        PC_info_in_stack <= {INT2_PROGRAM_ENCODE[1:0],PC};
                        wr_stack_ins <= 1;
                        // Interrupt control
                        interrupt_handling_1_reg <= 1;
                        interrupt_handling_2_reg <= 0;
                        interrupt_handling_3_reg <= 0;
                    end
                    else begin
                       if(contain_ins) begin
                            program_state <= DISPATCH_INS_STATE;
                            // 
                            contain_ins <= contain_ins - 1;
                        end
                        else begin
                            program_state <= FETCH_INS_STATE;
                            // Request Instruction
                            contain_ins <= 1;
//                            rd_ins_pm_reg <= 1;
                        end 
                    end
                end
                else if(interrupt_handling_3) begin
                    if(interrupt_flag_1) begin
                        program_state <= RESTORE_PC_STATE;
                        // Assign Interrupt address
                        PC <= INT1_PROGRAM_ADDR;
                        // Prepare data to push in stack
                        PC_info_in_stack <= {INT3_PROGRAM_ENCODE[1:0],PC};
                        wr_stack_ins <= 1;
                        // Interrupt control
                        interrupt_handling_1_reg <= 1;
                        interrupt_handling_2_reg <= 0;
                        interrupt_handling_3_reg <= 0; 
                    end
                    else if(interrupt_flag_2) begin
                        program_state <= RESTORE_PC_STATE;
                        // Assign Interrupt address
                        PC <= INT2_PROGRAM_ADDR;
                        // Prepare data to push in stack
                        PC_info_in_stack <= {INT3_PROGRAM_ENCODE[1:0],PC};
                        wr_stack_ins <= 1;
                        // Interrupt control
                        interrupt_handling_1_reg <= 0;
                        interrupt_handling_2_reg <= 1;
                        interrupt_handling_3_reg <= 0;
                    end
                    else begin
                       if(contain_ins) begin
                            program_state <= DISPATCH_INS_STATE;
                            // 
                            contain_ins <= contain_ins - 1;
                        end
                        else begin
                            program_state <= FETCH_INS_STATE;
                            // Request Instruction
                            contain_ins <= 1;
//                            rd_ins_pm_reg <= 1;
                        end 
                    end
                end
                else begin      // In main program
                    if(interrupt_flag_1) begin
                        program_state <= RESTORE_PC_STATE;
                        // Assign Interrupt address
                        PC <= INT1_PROGRAM_ADDR;
                        // Prepare data to push in stack
                        PC_info_in_stack <= {MAIN_PROGRAM_ENCODE[1:0],PC};
                        wr_stack_ins <= 1;
                        // Interrupt control
                        interrupt_handling_1_reg <= 1;
                        interrupt_handling_2_reg <= 0;
                        interrupt_handling_3_reg <= 0;
                    end
                    else if(interrupt_flag_2) begin
                        program_state <= RESTORE_PC_STATE;
                        // Assign Interrupt address
                        PC <= INT2_PROGRAM_ADDR;
                        // Prepare data to push in stack
                        PC_info_in_stack <= {MAIN_PROGRAM_ENCODE[1:0],PC};
                        wr_stack_ins <= 1;
                        // Interrupt control
                        interrupt_handling_1_reg <= 0;
                        interrupt_handling_2_reg <= 1;
                        interrupt_handling_3_reg <= 0;
                    end
                    else if(interrupt_flag_3) begin
                        program_state <= RESTORE_PC_STATE;
                        // Assign Interrupt address
                        PC <= INT3_PROGRAM_ADDR;
                        // Prepare data to push in stack
                        PC_info_in_stack <= {MAIN_PROGRAM_ENCODE[1:0],PC};
                        wr_stack_ins <= 1;
                        // Interrupt control
                        interrupt_handling_1_reg <= 0;
                        interrupt_handling_2_reg <= 0;
                        interrupt_handling_3_reg <= 1;
                    end
                    else begin
                        program_state <= PROGRAM_IDLE_STATE;
                    end
                end
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
                if(synchronized_processors) begin
                    program_state <= FETCH_INS_STATE;
                    contain_ins <= 1;
                    // Write PC to Stack
                    active_stack_clk <= 1;
                    // Request Instruction
//                    rd_ins_pm_reg <= 1;
                end
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

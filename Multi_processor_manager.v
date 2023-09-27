module Multi_processor_manager
    #(
    parameter INSTRUCTION_WIDTH     = 32,   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 256,
    
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
    parameter JUMP_OFS_SPACE_LSB    = 7,// Opcode encoder
    parameter JUMP_OFS_SPACE_WIDTH  = JUMP_OFS_SPACE_MSB - JUMP_OFS_SPACE_LSB + 1,// Opcode encoder
    // Opcode encoder
    parameter R_TYPE_ENCODE         = 7'b0110011,       // R-type
    parameter I_TYPE_ENCODE         = 7'b0010011,       // I-type
    parameter J_TYPE_ENCODE         = 7'b1100111,       // J-type
    parameter JAL_TYPE_ENCODE       = 7'b1101111,       // J-type
    parameter B_TYPE_ENCODE         = 7'b1100011,       // B-type   
    parameter LOAD_ENCODE           = 7'b0000011,       // LOAD-type
    parameter STORE_ENCODE          = 7'b0100011,       // STORE-type
    parameter SYSTEM_ENCODE         = 7'b1110111,       // Return interrupt
    // Funct3 enoder (for B-type)
    parameter BEQ_ENCODE           = 3'b000,
    parameter BNE_ENCODE           = 3'b001,
    parameter BLT_ENCODE           = 3'b100,
    parameter BGE_ENCODE           = 3'b101,
    // Funct3 encoder (for SYSTEM-type) 
    parameter BREAK_ENCODE         = 3'b001,            // BREAK: use for debugger
    parameter EXIT_ENCODE          = 3'b010,
    parameter RETI_ENCODE          = 3'b011,
    
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
    output  wire                                rd_ins_pm,
    
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
    input   wire    [DOUBLEWORD_WIDTH - 1:0]    register_renew [0:REGISTER_AMOUNT - 1],
    output  wire    [REG_SPACE_WIDTH - 1:0]     register_num,
    output  wire                                boot_renew_register_1,
    output  wire                                boot_renew_register_2,
    output  wire                                main_program_state,
    
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
    
    input   wire                                rst_n
    );
   
    // Internal register 
    reg [DOUBLEWORD_WIDTH - 1:0] x1;   // register ra (return address)
    reg [ADDR_WIDTH_PM - 1:0] PC;
    // State machine
    reg [4:0] program_state;
    // reg
    reg [DOUBLEWORD_WIDTH - 1:0] _2_instructions_buf;
    reg contain_ins;
    wire[REG_SPACE_WIDTH - 1:0] rs1_cur;
    wire[REG_SPACE_WIDTH - 1:0] rs2_cur;
    wire[REG_SPACE_WIDTH - 1:0] rd_cur;
    reg [REG_SPACE_WIDTH - 1:0] rd_prev;
    reg [REG_SPACE_WIDTH - 1:0] rd_prev_interrupt_program;
    reg rd_ins_pm_reg;
    wire parallel_blocking_Rtype_condition;
    wire parallel_blocking_Itype_condition;
    wire pipeline_blocking_Rtype_condition;
    wire pipeline_blocking_Itype_condition;
    wire release_blocking_condition;
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
    // Registers management
    reg [REG_SPACE_WIDTH - 1:0]         register_num_reg;
    reg                                 boot_renew_register_1_reg;
    reg                                 boot_renew_register_2_reg;
    // LIFO 
    reg  active_stack_clk;
    reg wr_stack_ins;
    reg rd_stack_ins;
    reg  [PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 1:0] PC_info_in_stack;    // <program-type> + <PC>
    wire [PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 1:0] PC_info_out_stack;
    wire [PROGRAM_COUNTER_WIDTH - 1:0] program_type_out_stack;
    wire [ADDR_WIDTH_PM - 1:0] PC_out_stack;
    
    localparam INIT_STATE = 1;
    localparam MAIN_PROGRAM_IDLE_STATE = 0;
    localparam REQUEST_INS_STATE = 2;
    localparam DECODE_INS_STATE = 3;
    localparam PARALLEL_BLOCKING_STATE = 4;
    localparam PARALLEL_BLOCKING_EX_INTERNAL_STATE = 24;
    localparam FETCH_INSTRUCTION_STATE = 5;
    localparam EXECUTE_BTYPE_INSTRUCTION_INTERNAL_STATE = 6;
    localparam EXECUTE_JTYPE_INSTRUCTION_INTERNAL_STATE = 9;
    localparam CONFIRM_FETCH_INSTRUCTION_P1_STATE = 7;
    localparam CONFIRM_FETCH_INSTRUCTION_P2_STATE = 8;
    localparam PC_INSCREASE_STATE = 10;
    localparam DETECT_INTERRUPT_STATE = 11;
    localparam RESTORE_PC_STATE = 12;
    localparam REQUEST_INT_INS_STATE = 13;
    localparam DECODE_INT_INS_STATE = 14;
    localparam FETCH_INT_INS_STATE = 15;
    localparam CONFIRM_FETCH_INT_INS_STATE = 16;
    localparam PIPELINE_BLOCKING_STATE = 17;
    localparam EXECUTE_BTYPE_INT_INS_INTERNAL_STATE = 18;
    localparam EXECUTE_JTYPE_INT_INS_INTERNAL_STATE = 19;
    localparam INT_PC_INSCREASE_STATE = 20;
    localparam RECOVERY_INT_PC_STATE = 21;
    localparam RECOVERY_MAIN_PC_STATE = 22;
    localparam DETECT_HIGHER_INT_STATE = 23;
    
    assign addr_rd_pm = PC;
    assign rd_ins_pm = rd_ins_pm_reg;
    // Fetch Instruction management
    assign cur_instruction  = (contain_ins) ? _2_instructions_buf[31:0] : _2_instructions_buf[63:32]; 
    assign fetch_instruction_1 = cur_instruction;
    assign fetch_instruction_2 = cur_instruction;
    assign boot_processor_1 = boot_processor_1_reg;
    assign boot_processor_2 = boot_processor_2_reg;
    // Decode Instruction
    assign opcode_space = cur_instruction[OPCODE_SPACE_MSB:OPCODE_SPACE_LSB];
    assign funct10_space = cur_instruction[FUNCT10_SPACE_MSB:FUNCT10_SPACE_LSB];
    assign funct3_space = cur_instruction[FUNCT3_SPACE_MSB:FUNCT3_SPACE_LSB];
    assign rd_cur = cur_instruction[RD_SPACE_MSB:RD_SPACE_LSB];
    assign rs1_cur = cur_instruction[RS1_SPACE_MSB:RS1_SPACE_LSB];
    assign rs2_cur = cur_instruction[RS2_SPACE_MSB:RS2_SPACE_LSB];
    assign immediate_1_space = cur_instruction[IMM_1_SPACE_MSB:IMM_1_SPACE_LSB];
    assign immediate_2_space = cur_instruction[IMM_2_SPACE_MSB:IMM_2_SPACE_LSB];
    assign immediate_3_space = cur_instruction[IMM_3_SPACE_MSB:IMM_3_SPACE_LSB];
    assign jump_offset_space = cur_instruction[JUMP_OFS_SPACE_MSB:JUMP_OFS_SPACE_LSB];
    
    // register management 
    assign register_num = register_num_reg;
    assign boot_renew_register_1 = boot_renew_register_1_reg;
    assign boot_renew_register_2 = boot_renew_register_2_reg;
    // Condition
    assign parallel_blocking_Rtype_condition = (rs1_cur == rd_prev) | (rs2_cur == rd_prev);
    assign parallel_blocking_Itype_condition = (rs1_cur == rd_prev);
    assign pipeline_blocking_condition = (rs1_cur == rd_prev_interrupt_program | rs2_cur == rd_prev_interrupt_program);
    assign release_blocking_condition = (processor_idle_1 == 1) & (processor_idle_2 == 1);
    
    // Interrupt control
    assign RETI_1 = RETI_1_reg;
    assign RETI_2 = RETI_2_reg;
    assign RETI_3 = RETI_3_reg;
    assign interrupt_handling_1 = interrupt_handling_1_reg;
    assign interrupt_handling_2 = interrupt_handling_2_reg;
    assign interrupt_handling_3 = interrupt_handling_3_reg;
    assign main_program_state = ~(interrupt_handling_1 | interrupt_handling_2 | interrupt_handling_3);
    
    // LIFO_out decode
    assign program_type_out_stack = PC_info_out_stack[PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 1 : PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 2];
    assign PC_out_stack = PC_info_out_stack[PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM - 3:0];
    
    // data_bus format : {prgram_type (2-bit), PC}
    LIFO_module #(
                .DATA_WIDTH(PROGRAM_COUNTER_WIDTH + ADDR_WIDTH_PM),
                .LIFO_DEPTH(INTERRUPT_BUFFER)
                ) interrupt_buffer (
                .active_clk(active_stack_clk),
                .data_bus_in(PC_info_in_stack),
                .data_bus_out(PC_info_out_stack),
                .wr_ins(wr_stack_ins),
                .rd_ins(rd_stack_ins),
                .rst_n(rst_n)
                );
                            
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            program_state <= INIT_STATE;
            // Init program
            PC <= MAIN_PROGRAM_ADDR;
            // Init reg
            x1 <= 0;
            rd_prev <= 1;   // (because x1 is reserved register, compiler will not put x1 to register destination (rd))
            contain_ins <= 0;
            rd_ins_pm_reg <= 0;
            boot_processor_1_reg <= 0;
            boot_processor_2_reg <= 0;
            active_stack_clk <= 0;
            PC_info_in_stack <= 0;
            wr_stack_ins <= 0;
            rd_stack_ins <= 0;
            rd_prev_interrupt_program <= 0;
            RETI_1_reg <= 0;
            RETI_2_reg <= 0;
            RETI_3_reg <= 0;
            interrupt_handling_1_reg <= 0;
            interrupt_handling_2_reg <= 0;
            interrupt_handling_3_reg <= 0;
        end
        else begin
            case(program_state)
                INIT_STATE: begin
                    if(main_state == RUNNING_PROGRAM_STATE) begin
                        program_state <= MAIN_PROGRAM_IDLE_STATE;
                    end
                    else program_state <= INIT_STATE;
                end
                MAIN_PROGRAM_IDLE_STATE: begin
                    if(contain_ins == 0) begin
                        program_state <= REQUEST_INS_STATE;
                        contain_ins <= 1;
                        // Request 
                        rd_ins_pm_reg <= 1;
                    end
                    else begin
                        program_state <= DECODE_INS_STATE;
                        contain_ins <= contain_ins - 1;
                    end
                    // Recovery state
                    active_stack_clk <= 0;
                    rd_stack_ins <= 0;
                    wr_stack_ins <= 0;
                end
                REQUEST_INS_STATE: begin
                    if(rd_idle_pm) begin
                        program_state <= DECODE_INS_STATE;
                        _2_instructions_buf <= data_bus_rd_pm;
                        rd_ins_pm_reg <= 0;
                    end
                    else program_state <= REQUEST_INS_STATE;
                end
                DECODE_INS_STATE: begin
                    case(opcode_space) 
                        R_TYPE_ENCODE: begin
                            if(parallel_blocking_Rtype_condition) begin
                                if(release_blocking_condition) begin
                                    program_state <= FETCH_INSTRUCTION_STATE;
                                end
                                else program_state <= PARALLEL_BLOCKING_STATE;
                            end
                            else program_state <= FETCH_INSTRUCTION_STATE;
                            rd_prev <= rd_cur;
                            register_num_reg <= rd_cur;
                        end
                        I_TYPE_ENCODE: begin
                            if(parallel_blocking_Itype_condition) begin
                                if(release_blocking_condition) begin
                                    program_state <= FETCH_INSTRUCTION_STATE;
                                end
                                else program_state <= PARALLEL_BLOCKING_STATE;
                            end
                            else program_state <= FETCH_INSTRUCTION_STATE;
                            rd_prev <= rd_cur;
                            register_num_reg <= rd_cur;
                        end
                        LOAD_ENCODE: begin
                            if(parallel_blocking_Itype_condition) begin
                                if(release_blocking_condition) begin
                                    program_state <= FETCH_INSTRUCTION_STATE;
                                end
                                else program_state <= PARALLEL_BLOCKING_STATE;
                            end
                            else program_state <= FETCH_INSTRUCTION_STATE;
                            rd_prev <= rd_cur;
                            register_num_reg <= rd_cur;
                        end
                        STORE_ENCODE: begin
                            if(parallel_blocking_Rtype_condition) begin
                                if(release_blocking_condition) begin
                                    program_state <= FETCH_INSTRUCTION_STATE;
                                end
                                else program_state <= PARALLEL_BLOCKING_STATE;
                            end
                            else program_state <= FETCH_INSTRUCTION_STATE;
                        end
                        // Execute immediate
                        B_TYPE_ENCODE: begin
                            contain_ins <= 0;
                            if(parallel_blocking_Rtype_condition) begin
                                if(release_blocking_condition) begin
                                    program_state <= EXECUTE_BTYPE_INSTRUCTION_INTERNAL_STATE;
                                end
                                else program_state <= PARALLEL_BLOCKING_EX_INTERNAL_STATE;
                            end
                            else program_state <= EXECUTE_BTYPE_INSTRUCTION_INTERNAL_STATE;
                            
                        end
                        J_TYPE_ENCODE: begin
                            program_state <= EXECUTE_JTYPE_INSTRUCTION_INTERNAL_STATE;
                            contain_ins <= 0;
                        end
                        JAL_TYPE_ENCODE: begin
                            program_state <= EXECUTE_JTYPE_INSTRUCTION_INTERNAL_STATE;
                            contain_ins <= 0;
                        end
                        default: program_state <= FETCH_INSTRUCTION_STATE;
                    endcase 
                end
                PARALLEL_BLOCKING_STATE: begin
                    if(release_blocking_condition) begin
                        program_state <= FETCH_INSTRUCTION_STATE;
                    end
                    else program_state <= PARALLEL_BLOCKING_STATE;
                end
                FETCH_INSTRUCTION_STATE: begin
                    if(processor_idle_1) begin
                        program_state <= CONFIRM_FETCH_INSTRUCTION_P1_STATE;
                        // Boot
                        boot_processor_1_reg <= 1;
                        boot_renew_register_1_reg <= 1;
                    end
                    else begin  // processor_idle_2 == 1 (because previous state had confirmed "one more processor is IDLE"
                        program_state <= CONFIRM_FETCH_INSTRUCTION_P2_STATE;
                        // Boot
                        boot_processor_2_reg <= 1;
                        boot_renew_register_2_reg <= 1;
                    end
                end
                CONFIRM_FETCH_INSTRUCTION_P1_STATE: begin
                    if(processor_idle_1 == 0) begin
                        program_state <= PC_INSCREASE_STATE;
                        // Reset boot
                        boot_processor_1_reg <= 0;
                        boot_renew_register_1_reg <= 0;
                    end
                    else program_state <= CONFIRM_FETCH_INSTRUCTION_P1_STATE;
                end
                CONFIRM_FETCH_INSTRUCTION_P2_STATE: begin
                    if(processor_idle_2 == 0) begin
                        program_state <= PC_INSCREASE_STATE;
                        // Reset boot 
                        boot_processor_2_reg <= 0;
                        boot_renew_register_2_reg <= 0;
                    end
                    else program_state <= CONFIRM_FETCH_INSTRUCTION_P2_STATE;
                end
                EXECUTE_BTYPE_INSTRUCTION_INTERNAL_STATE: begin
                    case(funct3_space)
                        BEQ_ENCODE: begin
                            if(register_renew[rs1_cur] == register_renew[rs2_cur]) begin
                                program_state <= DETECT_INTERRUPT_STATE;
                                PC <= {immediate_3_space, immediate_1_space};
                            end
                            else begin
                                program_state <= PC_INSCREASE_STATE;
                            end 
                        end
                        BNE_ENCODE: begin
                            if(register_renew[rs1_cur] != register_renew[rs2_cur]) begin
                                program_state <= DETECT_INTERRUPT_STATE;
                                PC <= {immediate_3_space, immediate_1_space};
                            end
                            else begin
                                program_state <= PC_INSCREASE_STATE;
                            end 
                        end
                        BLT_ENCODE: begin
                            if(register_renew[rs1_cur] < register_renew[rs2_cur]) begin
                                program_state <= DETECT_INTERRUPT_STATE;
                                PC <= {immediate_3_space, immediate_1_space};
                            end
                            else begin
                                program_state <= PC_INSCREASE_STATE;
                            end 
                        end
                        BGE_ENCODE: begin
                            if(register_renew[rs1_cur] > register_renew[rs2_cur]) begin
                                program_state <= DETECT_INTERRUPT_STATE;
                                PC <= {immediate_3_space, immediate_1_space};
                            end
                            else begin
                                program_state <= PC_INSCREASE_STATE;
                            end 
                        end
                        default: begin
                        
                        end
                    endcase
                end
                EXECUTE_JTYPE_INSTRUCTION_INTERNAL_STATE: begin
                    program_state <= DETECT_INTERRUPT_STATE;
                    x1 <= (opcode_space == JAL_TYPE_ENCODE) ? PC + 4 : x1;
                    PC <= PC + jump_offset_space;
                end
                PC_INSCREASE_STATE: begin
                    program_state <= DETECT_INTERRUPT_STATE;
                    PC <= PC + 4;
                end
                DETECT_INTERRUPT_STATE: begin
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
                        program_state <= MAIN_PROGRAM_IDLE_STATE;
                    end
                end
                RESTORE_PC_STATE: begin
                    program_state <= REQUEST_INT_INS_STATE;
                    contain_ins <= 1;
                    // Write PC to Stack
                    active_stack_clk <= 1;
                    // Request Instruction
                    rd_ins_pm_reg <= 1;
                end
                REQUEST_INT_INS_STATE: begin
                    if(rd_idle_pm) begin
                        program_state <= DECODE_INT_INS_STATE;
                        // Store instruction to buffer
                        _2_instructions_buf <= data_bus_rd_pm;
                        rd_ins_pm_reg <= 0;
                    end
                    else program_state <= program_state;
                    // Recovery state of Stack 
                    active_stack_clk <= 0;
                    rd_stack_ins <= 0;
                    wr_stack_ins <= 0;
                end
                DECODE_INT_INS_STATE: begin
                    case(opcode_space) 
                        R_TYPE_ENCODE: begin
                            if(processor_idle_1) begin
                                program_state <= CONFIRM_FETCH_INT_INS_STATE;
                                boot_processor_1_reg <= 1;
                            end
                            // Update rd_prev_interrupt_program
                            rd_prev_interrupt_program <= rd_cur;
                        end
                        I_TYPE_ENCODE: begin
                            if(processor_idle_1) begin
                                program_state <= CONFIRM_FETCH_INT_INS_STATE;
                                boot_processor_1_reg <= 1;
                            end
                            // Update rd_prev_interrupt_program
                            rd_prev_interrupt_program <= rd_cur;
                        end
                        LOAD_ENCODE: begin
                            if(processor_idle_1) begin
                                program_state <= CONFIRM_FETCH_INT_INS_STATE;
                                boot_processor_1_reg <= 1;
                            end
                            // Update rd_prev_interrupt_program
                            rd_prev_interrupt_program <= rd_cur;
                        end
                        STORE_ENCODE: begin
                            if(processor_idle_1) begin
                                program_state <= CONFIRM_FETCH_INT_INS_STATE;
                                boot_processor_1_reg <= 1;
                            end
                        end
                        // Execute immediate
                        B_TYPE_ENCODE: begin
                            if(pipeline_blocking_condition) begin
                                if(processor_idle_1) program_state <= EXECUTE_BTYPE_INT_INS_INTERNAL_STATE;
                                else program_state <= PIPELINE_BLOCKING_STATE;
                            end
                            contain_ins <= 0;
                        end
                        J_TYPE_ENCODE: begin
                            if(pipeline_blocking_condition) begin
                                if(processor_idle_1) program_state <= EXECUTE_JTYPE_INT_INS_INTERNAL_STATE;
                                else program_state <= PIPELINE_BLOCKING_STATE;
                            end
                            contain_ins <= 0;
                        end
                        JAL_TYPE_ENCODE: begin
                            if(pipeline_blocking_condition) begin
                                if(processor_idle_1) program_state <= EXECUTE_JTYPE_INT_INS_INTERNAL_STATE;
                                else program_state <= PIPELINE_BLOCKING_STATE;
                            end
                            contain_ins <= 0;
                        end
                        SYSTEM_ENCODE: begin
                            case(funct3_space) 
                                RETI_ENCODE: begin
                                    if(processor_idle_1) begin
                                        case(program_type_out_stack)
                                            MAIN_PROGRAM_ENCODE: begin
                                                program_state <= RECOVERY_MAIN_PC_STATE;
                                                PC <= PC_out_stack;
                                                rd_stack_ins <= 1;
                                                // Interrupt control
                                                interrupt_handling_1_reg <= 0;
                                                interrupt_handling_2_reg <= 0;
                                                interrupt_handling_3_reg <= 0;
                                            end
                                            INT1_PROGRAM_ENCODE: begin
                                                program_state <= RECOVERY_INT_PC_STATE;
                                                PC <= PC_out_stack;
                                                rd_stack_ins <= 1;
                                                // Interrupt control
                                                interrupt_handling_1_reg <= 1;
                                                interrupt_handling_2_reg <= 0;
                                                interrupt_handling_3_reg <= 0;
                                            end
                                            INT2_PROGRAM_ENCODE: begin
                                                program_state <= RECOVERY_INT_PC_STATE;
                                                PC <= PC_out_stack;
                                                rd_stack_ins <= 1;
                                                // Interrupt control
                                                interrupt_handling_1_reg <= 0;
                                                interrupt_handling_2_reg <= 1;
                                                interrupt_handling_3_reg <= 0;
                                            end
                                            INT3_PROGRAM_ENCODE: begin
                                                program_state <= RECOVERY_INT_PC_STATE;
                                                PC <= PC_out_stack;
                                                rd_stack_ins <= 1;
                                                // Interrupt control
                                                interrupt_handling_1_reg <= 0;
                                                interrupt_handling_2_reg <= 0;
                                                interrupt_handling_3_reg <= 1;
                                            end
                                        endcase
                                        contain_ins <= 0;
                                    end
                                end
                                EXIT_ENCODE: begin
                                
                                end
                                BREAK_ENCODE: begin
                                
                                end
                                default: begin
                                
                                end
                            endcase 
                            
                        end
                        default: program_state <= program_state;
                    endcase 
                end
                PIPELINE_BLOCKING_STATE: begin
                    if(processor_idle_1) begin
                        program_state <= (opcode_space == B_TYPE_ENCODE) ? EXECUTE_BTYPE_INT_INS_INTERNAL_STATE : EXECUTE_JTYPE_INT_INS_INTERNAL_STATE;
                    end
                end
                CONFIRM_FETCH_INT_INS_STATE: begin
                    if(processor_idle_1 == 0) begin
                        program_state <= INT_PC_INSCREASE_STATE;
                        // Recovery state
                        boot_processor_1_reg <= 0;
                    end
                end
                EXECUTE_BTYPE_INT_INS_INTERNAL_STATE: begin
                    case(funct3_space)
                        BEQ_ENCODE: begin
                            if(register_renew[rs1_cur] == register_renew[rs2_cur]) begin
                                program_state <= DETECT_HIGHER_INT_STATE;
                                PC <= {immediate_3_space, immediate_1_space};
                            end
                            else begin
                                program_state <= INT_PC_INSCREASE_STATE;
                            end 
                        end
                        BNE_ENCODE: begin
                            if(register_renew[rs1_cur] != register_renew[rs2_cur]) begin
                                program_state <= DETECT_HIGHER_INT_STATE;
                                PC <= {immediate_3_space, immediate_1_space};
                            end
                            else begin
                                program_state <= INT_PC_INSCREASE_STATE;
                            end 
                        end
                        BLT_ENCODE: begin
                            if(register_renew[rs1_cur] < register_renew[rs2_cur]) begin
                                program_state <= DETECT_HIGHER_INT_STATE;
                                PC <= {immediate_3_space, immediate_1_space};
                            end
                            else begin
                                program_state <= INT_PC_INSCREASE_STATE;
                            end 
                        end
                        BGE_ENCODE: begin
                            if(register_renew[rs1_cur] > register_renew[rs2_cur]) begin
                                program_state <= DETECT_HIGHER_INT_STATE;
                                PC <= {immediate_3_space, immediate_1_space};
                            end
                            else begin
                                program_state <= INT_PC_INSCREASE_STATE;
                            end 
                        end
                        default: begin
                        
                        end
                    endcase
                end
                EXECUTE_JTYPE_INT_INS_INTERNAL_STATE: begin
                    program_state <= DETECT_HIGHER_INT_STATE;
                    x1 <= (opcode_space == JAL_TYPE_ENCODE) ? PC + 4 : x1;
                    PC <= PC + jump_offset_space;
                end
                INT_PC_INSCREASE_STATE: begin
                    program_state <= DETECT_HIGHER_INT_STATE;
                    PC <= PC + 4;
                end
                RECOVERY_MAIN_PC_STATE: begin
                    program_state <= MAIN_PROGRAM_IDLE_STATE;
                    // 
                    active_stack_clk <= 1;
                end
                RECOVERY_INT_PC_STATE: begin
                    program_state <= REQUEST_INT_INS_STATE;
                    // 
                    active_stack_clk <= 1;
                    // Recovery rd_prev_interrupt_program (because x1 is reserved register, compiler will not put x1 to register destination (rd)
                    rd_prev_interrupt_program <= 1; 
                    // Request Instructions
                    contain_ins <= 1;
                    rd_ins_pm_reg <= 1;
                end
                DETECT_HIGHER_INT_STATE: begin
                    if(interrupt_handling_1) begin
                        if(contain_ins) begin
                            program_state <= DECODE_INT_INS_STATE;
                            // 
                            contain_ins <= contain_ins - 1;
                        end
                        else begin
                            program_state <= REQUEST_INT_INS_STATE;
                            // Request Instruction
                            contain_ins <= 1;
                            rd_ins_pm_reg <= 1;
                        end
                    end
                    if(interrupt_handling_2) begin
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
                            // Change program pre-process
                            rd_prev_interrupt_program <= 1; 
                        end
                        else begin
                           if(contain_ins) begin
                                program_state <= DECODE_INT_INS_STATE;
                                // 
                                contain_ins <= contain_ins - 1;
                            end
                            else begin
                                program_state <= REQUEST_INT_INS_STATE;
                                // Request Instruction
                                contain_ins <= 1;
                                rd_ins_pm_reg <= 1;
                            end 
                        end
                    end
                    if(interrupt_handling_3) begin
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
                            // Change program pre-process
                            rd_prev_interrupt_program <= 1; 
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
                            // Change program pre-process
                            rd_prev_interrupt_program <= 1;
                        end
                        else begin
                           if(contain_ins) begin
                                program_state <= DECODE_INT_INS_STATE;
                                // 
                                contain_ins <= contain_ins - 1;
                            end
                            else begin
                                program_state <= REQUEST_INT_INS_STATE;
                                // Request Instruction
                                contain_ins <= 1;
                                rd_ins_pm_reg <= 1;
                            end 
                        end
                    end
                end
                default: begin
                
                end
            endcase
        end
    end
endmodule

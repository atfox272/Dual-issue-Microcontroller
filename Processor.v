`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/11/2023 08:46:02 PM
// Design Name: 
// Module Name: Processor
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


module Processor
    #(
    parameter MAIN_RPOCESSOR        = 1,            // Main processor - Sub processor
    
    parameter DATA_WIDTH            = 8,
    parameter DOUBLEWORD_WIDTH      = 64,
    parameter DATA_MEMORY_SIZE      = 10'd256,      // 256 bytes (2Kb)
    
    parameter ADDR_WIDTH_DM         = $clog2(DATA_MEMORY_SIZE),
    parameter DATA_TYPE_WIDTH       = 2,
    
    parameter INSTRUCTION_WIDTH     = 32,   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 64,
    
    parameter REGISTER_AMOUNT       = 32,
    // Program memory
    parameter ADDR_WIDTH_PM         = $clog2(PROGRAM_MEMORY_SIZE),
    parameter START_WR_ADDR_PM      = 8'h00,
    
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
    
    // Finish program condition 
    parameter FINISH_PROGRAM_OPCODE  = 7'b0001011,      // Detect finish_program_opcode (this opcode is RESERVED)
    parameter FINISH_PROGRAM_TIMER   = 125000,          // Counting time of EMPTY RX_module (assume 1ms)
    
    // Opcode encoder
    parameter R_TYPE_ENCODE         = 7'b0110011,       // R-type
    parameter I_TYPE_ENCODE         = 7'b0010011,       // I-type
    parameter J_TYPE_ENCODE         = 7'b1100111,       // J-type
    parameter B_TYPE_ENCODE         = 7'b1100011,       // B-type   
    parameter LOAD_ENCODE           = 7'b0000011,       // LOAD-type
    parameter STORE_ENCODE          = 7'b0100011,       // STORE-type
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
    // Funct3 encoder
    parameter ADDI_ENCODE           = 3'b000,
    parameter SLLI_ENCODE           = 3'b001,
    parameter SRLI_ENCODE           = 3'b101,
    parameter SLTI_ENCODE           = 3'b010,
    parameter SLTIU_ENCODE          = 3'b011,
    parameter XORI_ENCODE           = 3'b100,
    parameter ORI_ENCODE            = 3'b110,
    parameter ANDI_ENCODE           = 3'b111
    )(
    input clk,
    
    // Multi-processor Manager
    input   wire [INSTRUCTION_WIDTH - 1:0]  fetch_instruction,
    input   wire                            boot_processor,
    output  wire                            processor_idle,
    input   wire [REGISTER_AMOUNT - 1:0]    new_data_register,  // 1-main processor ; 0-sub processor
    
    
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
    // Common 
    output  wire    [1:0]                   main_state,
    //////////////////////////////////////////////////////////////////////
    
    // For sub Processor ///////////////////////////////////////////////
    // Data_bus block
    input   wire [DATA_WIDTH - 1:0]         data_bus_in_com_per,
    output  wire [DATA_WIDTH - 1:0]         data_bus_out_com_per,
    output  wire                            wr_use,
    output  wire                            rd_use,
    //////////////////////////////////////////////////////////////////////
    `ifdef MAIN_RPOCESSOR
        output  wire [DOUBLEWORD_WIDTH - 1:0]   processor_register_main [0:REGISTER_AMOUNT - 1],
        input   wire [DOUBLEWORD_WIDTH - 1:0]   processor_register_sub  [0:REGISTER_AMOUNT - 1],
    `else
        input   wire [DOUBLEWORD_WIDTH - 1:0]   processor_register_main [0:REGISTER_AMOUNT - 1],
        output  wire [DOUBLEWORD_WIDTH - 1:0]   processor_register_sub  [0:REGISTER_AMOUNT - 1],
    `endif
    
    input rst_n
    );
    generate
    if(MAIN_RPOCESSOR == 1) begin   : MAIN_RPOCESSOR_BLOCK
        
        // Declare CPU registers
        reg [DOUBLEWORD_WIDTH - 1:0] registers_owner    [0:REGISTER_AMOUNT - 1];
        wire[DOUBLEWORD_WIDTH - 1:0] registers_renew    [0:REGISTER_AMOUNT - 1];
        // Declare wires and states
        reg [1:0] main_state_reg;
        reg [1:0] load_program_state_reg;
        reg [2:0] running_program_state_reg;
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
        // Register (new data)
        
        // ALU block
        reg [DOUBLEWORD_WIDTH - 1:0]        alu_operand_1; 
        reg [DOUBLEWORD_WIDTH - 1:0]        alu_operand_2; 
        reg [DOUBLEWORD_WIDTH - 1:0]        alu_result; 
        
        // Main state encoder
        localparam IDLE_STATE = 0;
        localparam LOAD_PROGRAM_STATE = 1;
        localparam RUNNING_PROGRAM_STATE = 2;
        // Load program state
        localparam RD_FIFO_STATE = 1;
        localparam WR_RAM_STATE = 2;
        // Running program state 
        localparam FETCH_INSTRUCTION_STATE = 1;
        localparam EXECUTE_INSTRUCTION_STATE = 2;
        
        assign main_state = main_state_reg;
        assign RX_use_1 = RX_use_1_reg;
        // Finish program condition
        assign finish_program_condition = finish_program_condition_1 | finish_program_condition_2;
        assign finish_program_condition_1 = (data_bus_out_uart_1[6:0] == FINISH_PROGRAM_OPCODE & RX_flag_1 == 1) & (_4byte_counter == 0);   // (_4byte_counter == 0) <=> Opdcode location
        assign start_counting_finish_program_condition = (main_state_reg == LOAD_PROGRAM_STATE) & (RX_flag_1 == 0);        // MCU in LOAD_PROGRAM_STATE & RX_module is empty 
        // Program Memory 
        assign data_bus_wr_pm = data_bus_wr_pm_reg;
        assign addr_wr_pm = addr_wr_pm_reg;
        assign wr_ins_pm = wr_ins_pm_reg;
        // Instruction format
        assign opcode_space = fetch_instruction_reg[OPCODE_SPACE_MSB:OPCODE_SPACE_LSB];
        assign funct10_space = fetch_instruction_reg[FUNCT10_SPACE_MSB:FUNCT10_SPACE_LSB];
        assign funct3_space = fetch_instruction_reg[FUNCT3_SPACE_MSB:FUNCT3_SPACE_LSB];
        assign rd_space = fetch_instruction_reg[RD_SPACE_MSB:RD_SPACE_LSB];
        assign rs1_space = fetch_instruction_reg[RS1_SPACE_MSB:RS1_SPACE_LSB];
        assign rs2_space = fetch_instruction_reg[RS2_SPACE_MSB:RS2_SPACE_LSB];
        assign immediate_1_space = fetch_instruction_reg[IMM_1_SPACE_MSB:IMM_1_SPACE_LSB];
        assign immediate_2_space = fetch_instruction_reg[IMM_2_SPACE_MSB:IMM_2_SPACE_LSB];
        assign immediate_3_space = fetch_instruction_reg[IMM_3_SPACE_MSB:IMM_3_SPACE_LSB];
        // Avoid out-dated data in register
        
        for(genvar register_index = 0; register_index < REGISTER_AMOUNT; register_index = register_index + 1) begin : wire_out_register_block
            assign processor_register_main[register_index] = registers_owner[register_index];
            assign registers_renew[register_index] = (new_data_register[register_index]) ? processor_register_main[register_index] : processor_register_sub[register_index];
        end
        
        // Sensitive block
        always @* begin
            case(opcode_space) 
                R_TYPE_ENCODE: begin    
                    alu_operand_1 <= registers_renew[rs1_space];
                    alu_operand_2 <= registers_renew[rs2_space];
                end
                I_TYPE_ENCODE: begin    
                    alu_operand_1 <= registers_renew[rs1_space];
                    alu_operand_2 <= {52'h00, immediate_2_space, immediate_1_space};
                end
                LOAD_ENCODE: begin      // result: address of target -> access Data memory to write value
                    alu_operand_1 <= registers_renew[rs1_space];
                    alu_operand_2 <= {52'h00, immediate_2_space, immediate_1_space};
                end
                STORE_ENCODE: begin
                    alu_operand_1 <= registers_renew[rs1_space];
                    alu_operand_2 <= {52'h00, immediate_3_space, immediate_1_space};
                end
                J_TYPE_ENCODE: begin
                // Processor doesn't process J-type instruction
                    alu_operand_1 <= 64'h00;
                    alu_operand_2 <= 64'h00;
                end
                B_TYPE_ENCODE: begin
                // Processor doesn't process B-type instruction
                    alu_operand_1 <= 64'h00;
                    alu_operand_2 <= 64'h00;
                end
                default: begin
                    alu_operand_1 <= 64'h00;
                    alu_operand_2 <= 64'h00;
                end
            endcase
        end
        // Timer for finishing program (time out)
        waiting_module #(
                    .END_COUNTER(FINISH_PROGRAM_TIMER),
                    .WAITING_TYPE(0),
                    .LEVEL_PULSE(1)
                    )waiting_process_command(
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
                
                RX_use_1_reg <= 0;
                // Counter for instruction
                _4byte_counter <= 0;
                // PM's interface initialization
                data_bus_wr_pm_reg <= 8'h00;
                addr_wr_pm_reg <= START_WR_ADDR_PM;
                wr_ins_pm_reg <= 0;
            end
            else begin
                case(main_state_reg) 
                    IDLE_STATE: begin
                        if(RX_flag_1) begin
                            main_state_reg <= LOAD_PROGRAM_STATE;
                        end 
                        else main_state_reg <= IDLE_STATE;              // Polling RX_1 data 
                        RX_use_1_reg <= 0;
                    end
                    LOAD_PROGRAM_STATE: begin
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
                                    running_program_state_reg <= FETCH_INSTRUCTION_STATE;
                                    
                                    fetch_instruction_reg <= fetch_instruction; 
                                end
                                else running_program_state_reg <= IDLE_STATE;
                            end
                            FETCH_INSTRUCTION_STATE: begin
                                running_program_state_reg <= EXECUTE_INSTRUCTION_STATE;
                            
                                case(opcode_space) 
                                    R_TYPE_ENCODE: begin
                                        case(funct10_space) 
                                            ADD_ENCODE: begin
                                                // ALU signal here
                                            end
                                            SUB_ENCODE: begin
                                                // ALU signal here
                                            end
                                            SLT_ENCODE: begin
                                                // ALU signal here
                                            end
                                            SLTU_ENCODE: begin
                                                // ALU signal here
                                            end
                                            SRL_ENCODE: begin
                                            
                                            end
                                            SLL_ENCODE: begin
                                            
                                            end
                                            AND_ENCODE: begin
                                                // ALU signal here
                                            end
                                            XOR_ENCODE: begin
                                                // ALU signal here
                                            end
                                            OR_ENCODE: begin
                                                // ALU signal here
                                            end
                                            MUL_ENCODE: begin
                                                // ALU signal here
                                            end
                                        endcase
                                    end
                                    I_TYPE_ENCODE: begin
                                        case(funct3_space) 
                                            ADDI_ENCODE: begin
                                                // ALU signal here
                                            end               
                                            SLLI_ENCODE: begin
                                                // ALU signal here
                                            end
                                            SRLI_ENCODE: begin
                                                // ALU signal here
                                            end  
                                            SLTI_ENCODE: begin
                                                // ALU signal here
                                            end
                                            SLTIU_ENCODE: begin
                                                // ALU signal here
                                            end
                                            XORI_ENCODE: begin
                                                // ALU signal here
                                            end
                                            ORI_ENCODE: begin
                                                // ALU signal here
                                            end
                                            ANDI_ENCODE: begin
                                                // ALU signal here
                                            end    
                                            default: begin
                                            
                                            end
                                        endcase 
                                    end    
                                    J_TYPE_ENCODE: begin
                                    // Processor don't process J-type instruction
                                    end
                                    B_TYPE_ENCODE: begin
                                    // Processor don't process J-type instruction
                                    end
                                    LOAD_ENCODE: begin
                                    
                                    end
                                    STORE_ENCODE: begin
                                    
                                    end
                                endcase 
                            end
                            EXECUTE_INSTRUCTION_STATE: begin
                            
                            end
                        endcase
                    end
                endcase 
            end
        end
    end
    else begin                      : SUB_PROCESSOR_BLOCK
    
    end
    
    
    endgenerate
endmodule

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
    parameter PROCESSOR_TYPE    = 1,            // Main processor - Sub processor
    
    parameter DATA_WIDTH        = 8,
    parameter DOUBLEWORD_WIDTH  = 64,
    parameter DATA_MEMORY_SIZE  = 10'd256,      // 256 bytes (2Kb)
    
    parameter ADDR_WIDTH_DM     = $clog2(DATA_MEMORY_SIZE),
    parameter DATA_TYPE_WIDTH   = 2,
    
    parameter INSTRUCTION_WIDTH     = 32,   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 64,
    
    parameter REGISTER_AMOUNT       = 32,
    // Program memory
    parameter ADDR_WIDTH_PM         = $clog2(PROGRAM_MEMORY_SIZE),
    parameter START_WR_ADDR_PM      = 8'h00,
    
    // Finish program condition 
    parameter FINISH_PROGRAM_OPCODE  = 7'b0001011,      // Detect finish_program_opcode (this opcode is RESERVED)
    parameter FINISH_PROGRAM_TIMER   = 125000           // Counting time of EMPTY RX_module (assume 1ms)
    )(
    input clk,
    
    // Multi-processor Manager
    input   wire    [INSTRUCTION_WIDTH - 1:0]   fetch_instruction,
    input   wire                                boot_processor,
    output  wire                                processor_idle,
    input   wire    [REGISTER_AMOUNT - 1:0]     new_data_register,
    
    // Another Processor
    output  wire    [REGISTER_AMOUNT - 1:0]     processor_register_main,
    input   wire    [REGISTER_AMOUNT - 1:0]     processor_register_sub,
    
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

    input rst_n
    );
    generate
    if(PROCESSOR_TYPE == 1) begin   : MAIN_RPOCESSOR
        
        // Declare internal registers & wires
        reg [1:0] main_state_reg;
        reg [1:0] load_program_state_reg;
        reg       RX_use_1_reg;
        reg [1:0] _4byte_counter;         // Overflow at 4
        wire      finish_program_condition;
        wire      finish_program_condition_1;
        wire      finish_program_condition_2;
        wire      start_counting_finish_program_condition;
        // Program Memory 
        reg [DATA_WIDTH - 1:0]      data_bus_wr_pm_reg;
        reg [ADDR_WIDTH_PM - 1:0]   addr_wr_pm_reg;
        reg                         wr_ins_pm_reg;
        
        // Main state encoder
        localparam IDLE_STATE = 0;
        localparam LOAD_PROGRAM_STATE = 1;
        localparam RUNNING_PROGRAM_STATE = 2;
        // Load program state
        localparam RD_FIFO_STATE = 1;
        localparam WR_RAM_STATE = 2;
        
        assign main_state = main_state;
        assign RX_use_1 = RX_use_1_reg;
        // Finish program condition
        assign finish_program_condition = finish_program_condition_1 | finish_program_condition_2;
        assign finish_program_condition_1 = (data_bus_out_uart_1[6:0] == FINISH_PROGRAM_OPCODE & RX_flag_1 == 1) & (_4byte_counter == 0);   // (_4byte_counter == 0) <=> Opdcode location
        assign start_counting_finish_program_condition = (main_state_reg == LOAD_PROGRAM_STATE) & (RX_flag_1 == 0);        // MCU in LOAD_PROGRAM_STATE & RX_module is empty 
        // Program Memory 
        assign data_bus_wr_pm = data_bus_wr_pm_reg;
        assign addr_wr_pm = addr_wr_pm_reg;
        assign wr_ins_pm = wr_ins_pm_reg;
        
        
        // Timer for finishing program 
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
//                        if(finish_program_condition) begin              // Finish program
//                            main_state_reg <= RUNNING_PROGRAM_STATE;
//                            RX_use_1_reg <= (RX_flag_1 == 1) ? 1 : 0;   // Clear FIFO
//                        end
//                        else begin
                            
//                        end
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
                        
                        
                    end
                endcase 
            end
        end
    end
    else begin                      : SUB_PROCESSOR
    
    end
    
    
    endgenerate
endmodule

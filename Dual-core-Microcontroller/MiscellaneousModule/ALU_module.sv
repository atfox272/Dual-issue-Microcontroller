`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/19/2023 02:37:04 AM
// Design Name: 
// Module Name: ALU_module
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


module ALU_module
    #(
    parameter OPERAND_WIDTH     = 64,
    parameter OPCODE_WIDTH      = 17,
    
    // Deep configuration
    parameter DEFAULT_DATA      = 64'h00,
    //     R-type ENCODE        = {funct10, opcode}
    parameter ADD_ENCODE        = {10'b0000000000, 7'b0110011},
    parameter SUB_ENCODE        = {10'b1000000000, 7'b0110011},
    parameter SRL_ENCODE        = {10'b0000000101, 7'b0110011},
    parameter SLL_ENCODE        = {10'b0000000001, 7'b0110011},
    parameter SLT_ENCODE        = {10'b0000000010, 7'b0110011},
    parameter SLTU_ENCODE       = {10'b0000000011, 7'b0110011},
    parameter AND_ENCODE        = {10'b0000000111, 7'b0110011},
    parameter XOR_ENCODE        = {10'b0000000100, 7'b0110011},
    parameter OR_ENCODE         = {10'b0000000110, 7'b0110011},
    parameter MUL_ENCODE        = {10'b0000001000, 7'b0110011},
    //     I-type ENCODE        = {7'b00, funct3, opcode}
    parameter ADDI_ENCODE       = {7'b00, 3'b000, 7'b0010011},
    parameter SLLI_ENCODE       = {7'b00, 3'b001, 7'b0010011},
    parameter SRLI_ENCODE       = {7'b00, 3'b101, 7'b0010011},
    parameter SLTI_ENCODE       = {7'b00, 3'b010, 7'b0010011},
    parameter SLTIU_ENCODE      = {7'b00, 3'b011, 7'b0010011},
    parameter XORI_ENCODE       = {7'b00, 3'b100, 7'b0010011},
    parameter ORI_ENCODE        = {7'b00, 3'b110, 7'b0010011},
    parameter ANDI_ENCODE       = {7'b00, 3'b111, 7'b0010011}
    )
    (
    input   clk,
    
    input   wire [OPERAND_WIDTH - 1:0]  operand_1,
    input   wire [OPERAND_WIDTH - 1:0]  operand_2,
    input   wire [OPCODE_WIDTH - 1:0]   op_code,
    
    input   wire                        alu_trig,
    output  wire                        alu_idle,
    
    output  wire [OPERAND_WIDTH - 1:0]  result,
    output  wire                        overflow_flag,
    output  wire                        invalid_flag,
    
    input   rst_n
    
    // Debug 
//    ,output alu_trig_sync_wire
    );
    
    reg [2:0]                   alu_state_reg;
    reg                         alu_occupied_start;
    reg                         alu_occupied_stop;
    reg [OPERAND_WIDTH - 1:0]   result_reg;
    wire                        alu_trig_sync;
    reg [OPERAND_WIDTH - 1:0]   repitions;
    reg [OPERAND_WIDTH - 1:0]   operand_1_reg;
    reg [OPERAND_WIDTH - 1:0]   operand_2_reg;
    
    localparam IDLE_STATE = 0;
    localparam PROCESS_STATE = 1;
    localparam SRL_STATE = 2;
    localparam SLL_STATE = 3;
    localparam MUL_STATE = 4;
    
    assign alu_idle = ~(alu_occupied_start ^ alu_occupied_stop);
    assign result = result_reg;
    
    edge_detector edge_detector
            (
            .clk(clk),
            .sig_in(alu_trig),
            .out(alu_trig_sync),
            .rst_n(rst_n)
            );
    always @(posedge alu_trig, negedge rst_n) begin
        if(!rst_n) begin
            alu_occupied_start <= 0;
        end
        else begin
            alu_occupied_start <= ~alu_occupied_stop;
        end
    end
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            alu_state_reg <= IDLE_STATE;
            result_reg <= DEFAULT_DATA;
            alu_occupied_stop <= 0;
            repitions <= 0;
        end
        else begin
            case(alu_state_reg) 
                IDLE_STATE: begin
                    
                    repitions <= 0;
                    
                    if(alu_trig_sync) begin
                        case(op_code) 
                            ADD_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= operand_1 + operand_2; 
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            ADDI_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= operand_1 + operand_2; 
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            SUB_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= operand_1 - operand_2;
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            SRL_ENCODE: begin
                                alu_state_reg <= SRL_STATE;
                                result_reg <= operand_1 >> 1;
                                repitions <= repitions + 1;
                            end
                            SRLI_ENCODE: begin
                                alu_state_reg <= SRL_STATE;
                                result_reg <= operand_1 >> 1;
                                repitions <= repitions + 1;
                            end
                            SLL_ENCODE: begin
                                alu_state_reg <= SLL_STATE;
                                result_reg <= operand_1 << 1;
                                repitions <= repitions + 1;
                            end
                            SLLI_ENCODE: begin
                                alu_state_reg <= SLL_STATE;
                                result_reg <= operand_1 << 1;
                                repitions <= repitions + 1;
                            end
                            SLT_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= {63'h00, (operand_1 < operand_2)};
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            SLTI_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= {63'h00, (operand_1 < operand_2)};
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            SLTU_ENCODE: begin
                            // Compare unsigned
                            end
                            AND_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= operand_1 & operand_2; 
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            ANDI_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= operand_1 & operand_2; 
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            XOR_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= operand_1 ^ operand_2; 
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            XORI_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= operand_1 ^ operand_2; 
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            OR_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= operand_1 | operand_2;
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            ORI_ENCODE: begin
                                alu_state_reg <= IDLE_STATE;
                                result_reg <= operand_1 | operand_2;
                                alu_occupied_stop <= alu_occupied_start;
                            end
                            MUL_ENCODE: begin
                                alu_state_reg <= MUL_STATE;
                                operand_1_reg <= operand_1; // Multiplicand
                                operand_2_reg <= operand_2; // Multiplier
                                result_reg <= 0;
                            end
                            default: begin
                                alu_state_reg <= IDLE_STATE; 
                                alu_occupied_stop <= alu_occupied_start;                               
                            end
                        endcase
                    end
                    else alu_state_reg <= IDLE_STATE;
                end
                SRL_STATE: begin
                    if(repitions == operand_2 | (|repitions == 0)) begin
                        alu_state_reg <= IDLE_STATE;
                        alu_occupied_stop <= alu_occupied_start;
                        repitions <= 0;
                    end
                    else begin
                        result_reg <= result_reg >> 1;
                        repitions <= repitions + 1;
                    end
                end
                SLL_STATE: begin
                    if(repitions == operand_2 | (|repitions == 0)) begin
                        alu_state_reg <= IDLE_STATE;
                        alu_occupied_stop <= alu_occupied_start;
                        repitions <= 0;
                    end
                    else begin
                        result_reg <= result_reg << 1;
                        repitions <= repitions + 1;
                    end
                end
                MUL_STATE: begin
                    if((|operand_1_reg == 0) | (|operand_2_reg == 0)) begin
                        alu_state_reg <= IDLE_STATE;
                        alu_occupied_stop <= alu_occupied_start;
                    end
                    else begin
                        alu_state_reg <= MUL_STATE;
                        if(operand_2_reg[0] == 1) begin
                            result_reg <= result_reg + operand_1_reg;
                            operand_1_reg <= operand_1_reg << 1;
                            operand_2_reg <= operand_2_reg >> 1;
                        end
                        else begin
                            operand_1_reg <= operand_1_reg << 1;
                            operand_2_reg <= operand_2_reg >> 1;
                        end
                    end
                end
                default: begin
                    alu_state_reg <= IDLE_STATE;
                    result_reg <= DEFAULT_DATA;
                    alu_occupied_stop <= alu_occupied_start;
                end
            endcase
        end
    end
    
    // debug 
//    assign alu_trig_sync_wire = alu_trig_sync;
endmodule

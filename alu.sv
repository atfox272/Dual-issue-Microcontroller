
`define DEBUG
//`define REVERT_RESULT
module alu
    #(
    `include "configuration.vh"
    parameter OPERAND_WIDTH     = 64
    )
    (
    input   clk,
    
    input   wire [OPERAND_WIDTH - 1:0]      operand_1,
    input   wire [OPERAND_WIDTH - 1:0]      operand_2,
    `ifdef REVERT_RESULT
    input   wire                            revert_result,
    `endif
    input   wire [OPCODE_ALU_WIDTH - 1:0]   op_code,
    
    input   wire                            alu_enable_comb,
    input   wire                            alu_enable_seq,
    output  wire                            alu_idle,
    
    output  wire [OPERAND_WIDTH - 1:0]      result_1cycle,
    output  wire [OPERAND_WIDTH - 1:0]      result_multi_cycle,
    output  wire                            overflow_flag,
    output  wire                            invalid_flag,
    
    input   rst_n
    
    `ifdef DEBUG    
    // Debug 
    ,output alu_enable_seq_sync_wire
    `endif
    );
    // Sequential circuit 
    logic[OPERAND_WIDTH - 1:0]      operand_1_seq;
    logic[OPERAND_WIDTH - 1:0]      operand_2_seq;
    logic[OPCODE_ALU_WIDTH - 1:0]   op_code_seq;
    logic[OPERAND_WIDTH - 1:0]      result_seq;
    // Combintional circuit
    logic[OPERAND_WIDTH - 1:0]      operand_1_comb;
    logic[OPERAND_WIDTH - 1:0]      operand_2_comb;
    logic[OPCODE_ALU_WIDTH - 1:0]   op_code_comb;
    logic[OPERAND_WIDTH - 1:0]      result_comb;
    
    `ifdef REVERT_RESULT
        assign result_1cycle = (revert_result) ? ~result_comb : result_comb;
        assign result_multi_cycle = (revert_result) ? ~result_seq : result_seq;
    `else
        assign result_1cycle = result_comb;
        assign result_multi_cycle = result_seq;
    `endif
    
    always_comb begin
        operand_1_comb = (alu_enable_comb) ? operand_1 : {64{1'b0}};
        operand_2_comb = (alu_enable_comb) ? operand_2 : {64{1'b0}};
        op_code_comb   = (alu_enable_comb) ? op_code   : 1'b0;
        case(op_code_comb)
            ADD_ALU_ENCODE:  result_comb = operand_1_comb + operand_2_comb;
            SUB_ALU_ENCODE:  result_comb = operand_1_comb - operand_2_comb;
            SLT_ALU_ENCODE:  result_comb = {{63{1'b0}}, (operand_1_comb < operand_2_comb)};
            AND_ALU_ENCODE:  result_comb = operand_1_comb & operand_2_comb; 
            XOR_ALU_ENCODE:  result_comb = operand_1_comb ^ operand_2_comb;
            OR_ALU_ENCODE:   result_comb = operand_1_comb | operand_2_comb;
            default:         result_comb = {64{1'b0}};
        endcase
    end
    
    
    reg [2:0]                   alu_state_reg;
    reg                         alu_occupied_start;
    reg                         alu_occupied_stop;
    wire                        alu_enable_seq_sync;
    reg [OPERAND_WIDTH - 1:0]   repitions;
    
    localparam IDLE_STATE = 0;
    localparam PROCESS_STATE = 1;
    localparam SRL_STATE = 2;
    localparam SLL_STATE = 3;
    localparam MUL_STATE = 4;
    
    assign alu_idle = ~(alu_occupied_start ^ alu_occupied_stop);
    
    edge_detector edge_detector
            (
            .clk(clk),
            .sig_in(alu_enable_seq),
            .out(alu_enable_seq_sync),
            .rst_n(rst_n)
            );
            
    always @(posedge alu_enable_seq, negedge rst_n) begin
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
            result_seq <= 0;
            alu_occupied_stop <= 0;
            repitions <= 0;
        end
        else begin
            case(alu_state_reg) 
                IDLE_STATE: begin
                    repitions <= 0;
                    if(alu_enable_seq_sync) begin
                        case(op_code) 
                            SRL_ALU_ENCODE: begin
                                alu_state_reg <= SRL_STATE;
                                result_seq <= operand_1 >> 1;
                                repitions <= repitions + 1;
                            end
                            SLL_ALU_ENCODE: begin
                                alu_state_reg <= SLL_STATE;
                                result_seq <= operand_1 << 1;
                                repitions <= repitions + 1;
                            end
                            MUL_ALU_ENCODE: begin
                                alu_state_reg <= MUL_STATE;
                                operand_1_seq <= operand_1; // Multiplicand
                                operand_2_seq <= operand_2; // Multiplier
                                result_seq <= 0;
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
                        result_seq <= result_seq >> 1;
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
                        result_seq <= result_seq << 1;
                        repitions <= repitions + 1;
                    end
                end
                MUL_STATE: begin
                    if((|operand_1_seq == 0) | (|operand_2_seq == 0)) begin
                        alu_state_reg <= IDLE_STATE;
                        alu_occupied_stop <= alu_occupied_start;
                    end
                    else begin
                        alu_state_reg <= MUL_STATE;
                        if(operand_2_seq[0] == 1) begin
                            result_seq <= result_seq + operand_1_seq;
                            operand_1_seq <= operand_1_seq << 1;
                            operand_2_seq <= operand_2_seq >> 1;
                        end
                        else begin
                            operand_1_seq <= operand_1_seq << 1;
                            operand_2_seq <= operand_2_seq >> 1;
                        end
                    end
                end
                default: begin
                    alu_state_reg <= IDLE_STATE;
                    alu_occupied_stop <= alu_occupied_start;
                end
            endcase
        end
    end
    
    `ifdef DEBUG    
    // debug 
    assign alu_enable_seq_sync_wire = alu_enable_seq_sync;
    `endif
endmodule

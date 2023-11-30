
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
//    ,output alu_enable_seq_sync_wire
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
    reg [OPERAND_WIDTH - 1:0]   repitions;
    
    localparam IDLE_STATE       = 3'd0;
    localparam PROCESS_STATE    = 3'd1;
    localparam SRL_STATE        = 3'd2;
    localparam SLL_STATE        = 3'd3;
    localparam MUL_STATE        = 3'd4;
    
    assign alu_idle = alu_state_reg == IDLE_STATE;
    
    logic[2:0]                      alu_state_next;
    logic[OPERAND_WIDTH - 1:0]      repitions_next;
    logic[OPERAND_WIDTH - 1:0]      repitions_incr;
    logic[OPERAND_WIDTH - 1:0]      operand_1_seq_next;
    logic[OPERAND_WIDTH - 1:0]      operand_2_seq_next;
    logic[OPERAND_WIDTH - 1:0]      result_seq_next;
    always_comb begin
        alu_state_next = alu_state_reg;
        repitions_next = repitions;
        repitions_incr = repitions + 1;
        operand_1_seq_next = operand_1_seq;
        operand_2_seq_next = operand_2_seq;
        result_seq_next = result_seq;
        case(alu_state_reg) 
            IDLE_STATE: begin
                if (alu_enable_seq) begin
                case(op_code) 
                SRL_ALU_ENCODE: begin
                    alu_state_next = SRL_STATE;
                    result_seq_next = operand_1 >> 1;
                    repitions_next = repitions_incr;
                end
                SLL_ALU_ENCODE: begin
                    alu_state_next = SLL_STATE;
                    result_seq_next = operand_1 << 1;
                    repitions_next = repitions_incr;
                end
                MUL_ALU_ENCODE: begin
                    alu_state_next = MUL_STATE;
                    operand_1_seq_next = operand_1; // Multiplicand
                    operand_2_seq_next = operand_2; // Multiplier
                    result_seq_next = 0;
                end
                default: begin
                    alu_state_next = IDLE_STATE;                              
                end
                endcase
                end
            end
            SRL_STATE: begin
                if(repitions == operand_2 | (|repitions == 0)) begin
                    alu_state_next = IDLE_STATE;
                    repitions_next = 0;
                end
                else begin
                    result_seq_next = result_seq >> 1;
                    repitions_next = repitions_incr;
                end
            end
            SLL_STATE: begin
                if(repitions == operand_2 | (|repitions == 0)) begin
                    alu_state_next = IDLE_STATE;
                    repitions_next = 0;
                end
                else begin
                    result_seq_next = result_seq << 1;
                    repitions_next = repitions_incr;
                end
            end
            MUL_STATE: begin
                if((|operand_1_seq == 0) | (|operand_2_seq == 0)) begin
                    alu_state_next = IDLE_STATE;
                end
                else begin
                    alu_state_next = MUL_STATE;
                    if(operand_2_seq[0] == 1) begin
                        result_seq_next = result_seq + operand_1_seq;
                        operand_1_seq_next = operand_1_seq << 1;
                        operand_2_seq_next = operand_2_seq >> 1;
                    end
                    else begin
                        operand_1_seq_next = operand_1_seq << 1;
                        operand_2_seq_next = operand_2_seq >> 1;
                    end
                end
            end
            default: begin
                alu_state_next = IDLE_STATE;
            end
        endcase
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            alu_state_reg <= IDLE_STATE;
            result_seq <= 0;
            repitions <= 0;
        end
        else begin
            alu_state_reg <= alu_state_next;
            repitions <= repitions_next;
            operand_1_seq <= operand_1_seq_next;
            operand_2_seq <= operand_2_seq_next;
            result_seq <= result_seq_next;
        end
    end
    
    `ifdef DEBUG    
    // debug 
    `endif
endmodule

`timescale 1ns / 1ps
//`define PERIPHERAL_TESTCASE
`define GPIO_TESTCASE
module get_instruction_hex;
    reg[31:0] instruction;
    wire [7:0] first_hex = instruction[7:0];
    wire [7:0] second_hex = instruction[15:8];
    wire [7:0] third_hex = instruction[23:16];
    wire [7:0] fourth_hex = instruction[31:24];
    
    parameter ADD_INS_17    = 17'b00000000000110011;// ADD:     <5-rd><5-rs1><5-rs2>
    parameter ADDI_INS_10   = 10'b0000010011;       // ADDI:    <5-rd><5rs1><12-imm>
    parameter SUB_INS_17    = 17'b10000000000110011;// SUB:     <5-rd><5-rs1><5-rs2>
    parameter SLL_INS_17    = 17'b00000000010110011;// SLL:     <5-rd><5-rs1><5-rs2>
    parameter SRL_INS_17    = 17'b00000001010110011;// SRL:     <5-rd><5-rs1><5-rs2>
    parameter MUL_INS_17    = 17'b00000010000110011;// MUL:     <5-rd><5-rs1><5-rs2>
    // Load                                                     dest  base  offset
    parameter LB_INS_10     = 10'b0000000011;       // LB:      <5-rd><5rs1><12-imm>
    parameter LW_INS_10     = 10'b0100000011;       // LW:      <5-rd><5rs1><12-imm>
    parameter LD_INS_10     = 10'b0110000011;       // LD:      <5-rd><5rs1><12-imm>
    // Store                                                    offset  base   src   offset
    parameter SB_INS_10     = 10'b0000100011;       // SB:      <5-immh><5-rs1><5rs2><7-imml>
    parameter SW_INS_10     = 10'b0100100011;       // SW:      <5-immh><5-rs1><5rs2><7-imml>
    parameter SD_INS_10     = 10'b0110100011;       // SD:      <5-immh><5-rs1><5rs2><7-imml>
    // Unocndition jump
    parameter J_INS_7       = 7'b1100111;           // J:       <25-imm>
    parameter JAL_INS_7     = 7'b1101111;           // J:       <25-imm>
    parameter JALR_INS_7    = 7'b1101011;           // J:       <25-imm>
    // Condition jumo
    parameter BEQ_INS_10    = 10'b0001100011;       // BEQ:     <imm-h><rs1><rs2><imm-l>
    parameter BNE_INS_10    = 10'b0011100011;       // BNE:     <imm-h><rs1><rs2><imm-l>
    parameter BLT_INS_10    = 10'b1001100011;       // BLT:     <imm-h><rs1><rs2><imm-l>
    parameter BGE_INS_10    = 10'b1011100011;       // BGE:     <imm-h><rs1><rs2><imm-l>
    
    parameter FENCE_INS_10  = 10'b0100101111;       // FENCE:   Don't care
    
    parameter RETI_INS_10   = 10'b0111110111;       // RETI:   Don't care
    parameter DEBUG_INS_10  = 10'b1011110111;       // RETI:   Don't care
    
    parameter UART_TX_INS_17= 17'b10000000001000001;      // UART_TX: <5-rs3><5-rs1><5-rs2><5-imm>
    parameter UART_RX_INS_17= 17'b00000000001000001;      // UART_TX: <5-rd1>
    parameter LOAD_RX_H_INS_17= 17'b00000100001000001;      // UART_TX: <5-rd1>
    parameter LOAD_RX_L_INS_17= 17'b00000110001000001;      // UART_TX: <5-rd1>
    
    
    parameter GPIO_READ_INS_10  = 10'b0001110101;      // UART_TX: <5-rs3><5-rs1><5-rs2><6-imm>
    parameter GPIO_WRITE_INS_10 = 10'b0011110101;      // UART_TX: <5-rs3><5-rs1><5-rs2><6-imm>
    
    parameter LUI_INS_7     = 7'b0110111;
    
    initial begin
        `ifdef PERIPHERAL_TESTCASE
        instruction <= {5'd10,5'd0,12'b00001100,ADDI_INS_10};
        #1000;
        instruction <= {5'd00,5'd00,5'd10,7'h06,SB_INS_10};
        #1000;
        instruction <= {5'd09,5'd0,12'd9,ADDI_INS_10};
        #1000;
        instruction <= {5'd15,20'hC0000,LUI_INS_7};
        #1000;
        instruction <= {5'd08,5'd0,12'd2,ADDI_INS_10};
        #1000;
        instruction <= {5'd07,5'd09,5'd08,ADD_INS_17};
        #1000;
        instruction <= {5'd00,5'd15,5'd07,7'h00,SW_INS_10};
        #1000;
        `endif
        
        `ifdef GPIO_TESTCASE
        
        // PC = 0xC0
        // PC = 0xC8
        instruction <= {5'd20,5'd0,12'b00001100,ADDI_INS_10};         // SET PORT[0][7:4]: LOW - HIGH - LOW - HIGH
        #1000;  
        // PC = 0xC8
        instruction <= {5'd00,5'd00,5'd20,7'h00,SB_INS_10};         // SET PORT[0][7:4]: LOW - HIGH - LOW - HIGH
        #1000;  
        instruction <= {5'd15,20'h40000,LUI_INS_7};                // Load upper 20bit (h30000) to x15 (to map to UART_2)
        #1000;
        // PC = 0xC4
        instruction <= {5'd09,5'd0,12'b00001010,ADDI_INS_10};      // SET PORT[0][7:4]: LOW - HIGH - LOW - HIGH
        #1000;
        // PC = 0xC8
        instruction <= {5'd00,5'd15,5'd09,7'h00,SB_INS_10};         // SET PORT[0][7:4]: LOW - HIGH - LOW - HIGH
        #1000;            
        `endif
        
        $stop;
    end
endmodule

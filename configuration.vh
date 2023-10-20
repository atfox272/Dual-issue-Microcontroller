`ifndef _configuration_vh_
`define _configuration_vh_
    parameter ADD_ALU_ENCODE    = 0;
    parameter SUB_ALU_ENCODE    = 1; 
    parameter AND_ALU_ENCODE    = 2;
    parameter OR_ALU_ENCODE     = 3;
    parameter XOR_ALU_ENCODE    = 4;
    parameter SLT_ALU_ENCODE    = 5; 
    parameter NAND_ALU_ENCODE   = 6; 
    parameter NOR_ALU_ENCODE    = 7; 
    parameter MUL_ALU_ENCODE    = 8;
    parameter DIV_ALU_ENCODE    = 9;
    parameter SLL_ALU_ENCODE    = 10;
    parameter SRL_ALU_ENCODE    = 11;
    parameter HIGHEST_ENCODE    = 12;
    parameter OPCODE_ALU_WIDTH  = $clog2(HIGHEST_ENCODE);
`endif
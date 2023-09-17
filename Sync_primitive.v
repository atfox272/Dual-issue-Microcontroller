module Sync_primitive
    #(
    parameter DOUBLEWORD_WIDTH  = 64,
    parameter DATA_MEMORY_SIZE  =  10'd256,      // 256 bytes (2Kb)
    
    parameter ADDR_WIDTH_DM     = $clog2(DATA_MEMORY_SIZE),
    parameter DATA_TYPE_WIDTH   = 2,
    parameter TEMP              = 0
    )
    (
    input clk,
    
    // Data memory (READ_STATE)
    input   wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd_dm,
    output  wire [ADDR_WIDTH_DM - 1:0]      addr_rd_dm,
    output  wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd_dm,
    input   wire                            rd_idle_dm,   
    output  wire                            rd_ins_dm,   
    // Data memory (WRITE_STATE)
    output  wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_dm,
    output  wire [ADDR_WIDTH_DM - 1:0]      addr_wr_dm,
    output  wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr_dm,
    input   wire                            wr_idle_dm,   
    output  wire                            wr_ins_dm,   
    
    // Processor 1 (READ_STATE)
    output  wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd_p1,
    input   wire [ADDR_WIDTH_DM - 1:0]      addr_rd_p1,
    input   wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd_p1,
    output  wire                            rd_idle_p1,   
    input   wire                            rd_ins_p1, 
    output  wire                            rd_access_p1,
    input   wire                            rd_finish_p1,
    // Processor 2 (READ_STATE)
    output  wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd_p2,
    input   wire [ADDR_WIDTH_DM - 1:0]      addr_rd_p2,
    input   wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd_p2,
    output  wire                            rd_idle_p2,   
    input   wire                            rd_ins_p2, 
    output  wire                            rd_access_p2,
    input   wire                            rd_finish_p2,
    
    // Processor 1 (WRITE_STATE)
    input   wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_p1,
    input   wire [ADDR_WIDTH_DM - 1:0]      addr_wr_p1,
    input   wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr_p1,
    output  wire                            wr_idle_p1,   
    input   wire                            wr_ins_p1, 
    output  wire                            wr_access_p1,
    // Processor 2 (WRITE_STATE)
    input   wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_p2,
    input   wire [ADDR_WIDTH_DM - 1:0]      addr_wr_p2,
    input   wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr_p2,
    output  wire                            wr_idle_p2,   
    input   wire                            wr_ins_p2, 
    output  wire                            wr_access_p2,
    
    input rst_n
    );
endmodule

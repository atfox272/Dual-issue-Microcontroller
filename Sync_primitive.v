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
    
    // Data memory (Read handler)
    input   wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_rd_dm,
    output  wire [ADDR_WIDTH_DM - 1:0]      addr_rd_dm,
    output  wire [DATA_TYPE_WIDTH - 1:0]    data_type_rd_dm,
    input   wire                            rd_idle_dm,   
    output  wire                            rd_ins_dm,   
    // Data memory (Write handler)
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
    // State machine
    reg [1:0]                       rd_sync_primitive_state;
    reg [1:0]                       wr_sync_primitive_state;
    // Data memory
    // -- Read handler
    reg [ADDR_WIDTH_DM - 1:0]       addr_rd_dm_reg;
    reg [DATA_TYPE_WIDTH - 1:0]     data_type_rd_dm_reg;
    reg                             rd_ins_dm_reg;
    // -- Write handler
    reg [DOUBLEWORD_WIDTH - 1:0]    data_bus_wr_dm_reg;
    reg [ADDR_WIDTH_DM - 1:0]       addr_wr_dm_reg;
    reg [DATA_TYPE_WIDTH - 1:0]     data_type_wr_dm_reg;
    reg                             wr_ins_dm_reg;
    // Processor 1 
    // -- Read handler
    reg [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd_p1_reg;
    reg                             rd_access_p1_reg;
    // -- Write handler
    reg                             wr_access_p1_reg;
    
    // Processor 2
    // -- Read handler
    reg [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd_p2_reg;
    reg                             rd_access_p2_reg;
    // -- Write handler
    reg                             wr_access_p2_reg;
    
    localparam IDLE_STATE = 0;
    localparam P1_ACCESS_STATE = 1;
    localparam P2_ACCESS_STATE = 2;
    
    // Processor 1 
    // -- Read handler
    assign rd_access_p1 = rd_access_p1_reg;
    assign data_bus_rd_p1 = data_bus_rd_dm;
    assign rd_idle_p1 = (rd_access_p1_reg) ? rd_idle_dm : 1'b1;
    // -- Write handler 
    assign wr_access_p1 = wr_access_p1_reg;
    assign wr_idle_p1 = (wr_access_p1_reg) ? wr_idle_dm : 1'b1;
    
    // Processor 2 
    // -- Read handler
    assign rd_access_p2 = rd_access_p2_reg;
    assign data_bus_rd_p2 = data_bus_rd_dm;
    assign rd_idle_p2 = (rd_access_p2_reg) ? rd_idle_dm : 1'b1;
    // -- Write handler 
    assign wr_access_p2 = wr_access_p2_reg;
    assign wr_idle_p2 = (wr_access_p2_reg) ? wr_idle_dm : 1'b1;
    
    // Data memory
    // -- Read handler
    assign addr_rd_dm = (rd_access_p1_reg) ? addr_rd_p1 : addr_rd_p2;
    assign data_type_rd_dm = (rd_access_p1_reg) ? data_type_rd_p1 : data_type_rd_p2;
    assign rd_ins_dm = (rd_access_p1_reg) ? rd_ins_p1 : (rd_access_p2_reg) ? rd_ins_p2 : 1'b0;
    // -- Write handler
    assign data_bus_wr_dm = (wr_access_p1_reg) ? data_bus_wr_p1 : data_bus_wr_p2;
    assign addr_wr_dm = (wr_access_p1_reg) ? addr_wr_p1 : addr_wr_p2;
    assign data_type_wr_dm = (wr_access_p1_reg) ? data_type_wr_p1 : data_type_wr_p2;
    assign wr_ins_dm = (wr_access_p1_reg) ? wr_ins_p1 : (wr_access_p2_reg) ? wr_ins_p2 : 1'b0; 
    
    
    // Read handler 
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            rd_sync_primitive_state <= IDLE_STATE;
            //
            rd_access_p1_reg <= 0;
            rd_access_p2_reg <= 0;
        end
        else begin
            case(rd_sync_primitive_state) 
                IDLE_STATE: begin
                    if(rd_ins_p1) begin
                        rd_sync_primitive_state <= P1_ACCESS_STATE;
                        rd_access_p1_reg <= 1;
                    end
                    else if(rd_ins_p2) begin
                        rd_sync_primitive_state <= P2_ACCESS_STATE;
                        rd_access_p2_reg <= 1;
                    end
                    else rd_sync_primitive_state <= rd_sync_primitive_state;
                end
                P1_ACCESS_STATE: begin
                    if(rd_idle_dm) begin
                        rd_sync_primitive_state <= IDLE_STATE;
                        rd_access_p1_reg <= 0;
                    end                  
                end
                P2_ACCESS_STATE: begin
                    if(rd_idle_dm) begin
                        rd_sync_primitive_state <= IDLE_STATE;
                        rd_access_p2_reg <= 0;
                    end      
                end
                default: begin
                
                end
            endcase
        end
    end
    // Write handler
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            wr_sync_primitive_state <= IDLE_STATE;
            //
            wr_access_p1_reg <= 0;
            wr_access_p2_reg <= 0;
        end
        else begin
            case(wr_sync_primitive_state)
                IDLE_STATE: begin
                    if(wr_ins_p1) begin
                        wr_sync_primitive_state <= P1_ACCESS_STATE;
                        wr_access_p1_reg <= 1;
                    end
                    else if(wr_ins_p2) begin
                        wr_sync_primitive_state <= P2_ACCESS_STATE;
                        wr_access_p2_reg <= 1;
                    end
                    else wr_sync_primitive_state <= wr_sync_primitive_state;
                end
                P1_ACCESS_STATE: begin
                    if(wr_idle_dm) begin
                        wr_sync_primitive_state <= IDLE_STATE;
                        wr_access_p1_reg <= 0;
                    end
                end
                P2_ACCESS_STATE: begin
                    if(wr_idle_dm) begin
                        wr_sync_primitive_state <= IDLE_STATE;
                        wr_access_p2_reg <= 0;
                    end
                end
                default: begin
                
                end
            endcase 
        end
    end
endmodule

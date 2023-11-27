/* Sequential Parallel-Random-Access-Machine Consistency */ 
module pram_consistency 
    #(
    parameter DOUBLEWORD_WIDTH  = 64,
    parameter DATA_MEMORY_SIZE  = 1024,      // 256 bytes (2Kb)
    
    parameter ADDR_WIDTH_DM     = $clog2(DATA_MEMORY_SIZE),
    parameter DATA_TYPE_WIDTH   = 2,
    parameter TEMP              = 0
    )
    (
    input clk,
    
    // Data memory (Write handler)
    output  wire [DOUBLEWORD_WIDTH - 1:0]   data_bus_wr_dm,
    output  wire [ADDR_WIDTH_DM - 1:0]      addr_wr_dm,
    output  wire [DATA_TYPE_WIDTH - 1:0]    data_type_wr_dm,
    input   wire                            wr_idle_dm,   
    output  wire                            wr_ins_dm,   
    
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
    // Data memory
    // -- Write handler
    reg [DOUBLEWORD_WIDTH - 1:0]    data_bus_wr_dm_reg;
    reg [ADDR_WIDTH_DM - 1:0]       addr_wr_dm_reg;
    reg [DATA_TYPE_WIDTH - 1:0]     data_type_wr_dm_reg;
    reg                             wr_ins_dm_reg;
    // Processor 1 
    // -- Write handler
    reg                             wr_access_p1_reg;
    
    // Processor 2
    // -- Write handler
    reg                             wr_access_p2_reg;
    
    // State machine
    reg [1:0]                 wr_sync_primitive_state;
    localparam IDLE_STATE     = 2'd0;
    localparam P1_WR_STATE    = 2'd1;
    localparam P2_WR_STATE    = 2'd2;
    localparam ACCESS_STATE   = 2'd3;
    
    // Processor 1 
    // -- Write handler 
    assign wr_access_p1 = wr_access_p1_reg;
    assign wr_idle_p1 = (wr_access_p1_reg) ? wr_idle_dm : 1'b1;
    
    // Processor 2 
    // -- Write handler 
    assign wr_access_p2 = wr_access_p2_reg;
    assign wr_idle_p2 = (wr_access_p2_reg) ? wr_idle_dm : 1'b1;
    
    // Data memory    
    // -- Write handler
    assign data_bus_wr_dm = (wr_access_p1_reg) ? data_bus_wr_p1 : data_bus_wr_p2;
    assign addr_wr_dm = (wr_access_p1_reg) ? addr_wr_p1 : addr_wr_p2;
    assign data_type_wr_dm = (wr_access_p1_reg) ? data_type_wr_p1 : data_type_wr_p2;
    assign wr_ins_dm = (wr_access_p1_reg) ? wr_ins_p1 : (wr_access_p2_reg) ? wr_ins_p2 : 1'b0; 
    
    
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
                        wr_sync_primitive_state <= ACCESS_STATE;
                        wr_access_p1_reg <= 1;
                    end
                    else if(wr_ins_p2) begin
                        wr_sync_primitive_state <= ACCESS_STATE;
                        wr_access_p2_reg <= 1;
                    end
                    else wr_sync_primitive_state <= wr_sync_primitive_state;
                end
                ACCESS_STATE: begin
                    if(~wr_idle_dm) begin   // Access successfully
                        wr_sync_primitive_state <= (wr_access_p1_reg) ? P1_WR_STATE : P2_WR_STATE;
                    end
                end
                P1_WR_STATE: begin
                    if(wr_idle_dm) begin
                        wr_sync_primitive_state <= IDLE_STATE;
                        wr_access_p1_reg <= 0;
                    end
                end
                P2_WR_STATE: begin
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

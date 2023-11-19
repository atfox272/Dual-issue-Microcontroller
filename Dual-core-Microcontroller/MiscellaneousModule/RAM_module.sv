module ram_module
    #(
        parameter DATA_WIDTH        = 8,
        parameter DOUBLEWORD_WIDTH  = 64,
        parameter ADDR_DEPTH        = 512 ,  // <=> 512 bytes
        parameter ADDR_WIDTH        = $clog2(ADDR_DEPTH),
        
        // Deep configuartion
        parameter DEFAULT_ADDR  = 0,
        parameter DEFAULT_DATA  = 0,
        
        // Data type (3)
        parameter DATA_TYPE                     = 3,
        parameter DATA_TYPE_WIDTH               = $clog2(DATA_TYPE),
        parameter DATA_TYPE_BYTE_ENCODE         = 0,
        parameter DATA_TYPE_WORD_ENCODE         = 1,
        parameter DATA_TYPE_DOUBLEWORD_ENCODE   = 2,
        
        // Reserved register in RAM (for Peripheral configuration)
        parameter RESERVED_REG_AMOUNT           = 1,
        // Set default data for RESERVED_REGISTERs
        parameter byte RESERVED_REG_DEFAULT[0:RESERVED_REG_AMOUNT - 1] = {8'b00000000}  // address 0x0
    )
    (
    input   wire                        clk,
    
    input   wire    [DOUBLEWORD_WIDTH - 1:0]    data_bus_wr,
    output  wire    [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd,
    
    
    input   wire    [DATA_TYPE_WIDTH - 1:0]     data_type_rd,
    input   wire    [DATA_TYPE_WIDTH - 1:0]     data_type_wr,
    
    input   wire    [ADDR_WIDTH - 1:0]  addr_rd,
    input   wire    [ADDR_WIDTH - 1:0]  addr_wr,
    
    input   wire                        rd_ins,
    input   wire                        wr_ins,
    
    // Flag read 
    output  wire                        rd_flag,
    
    // State
    output  wire                        rd_idle,
    output  wire                        wr_idle,
    
    // Optional (to save configuration data)
    output  wire    [DATA_WIDTH - 1:0]  reserved_registers  [0:RESERVED_REG_AMOUNT - 1],
    
    input   wire                        rst_n
    
    // Debug
    ,output  wire    [DATA_WIDTH - 1:0]  registers_wire [0: ADDR_DEPTH - 1]
    );
    
    // Power checking: single register
    // -> Total power is 2.566W
    //  + Signal: 0.194W
    //  + Logic: 0.270W
    //  + IO: 1.938W
    //  + Static: 0.162W
    
//    wire    [ADDR_WIDTH - 1:0]  addr_rd;
//    wire    [ADDR_WIDTH - 1:0]  addr_wr;
//    assign addr_rd = 27;
//    assign addr_wr = 02;
    
    // Set of registers
    reg [DATA_WIDTH - 1:0] registers [0: ADDR_DEPTH - 1];
    
    reg [ADDR_WIDTH - 1:0] addr_rd_buf;
    reg [ADDR_WIDTH - 1:0] addr_wr_buf;
    
    reg [DATA_TYPE_WIDTH - 1:0] data_type_rd_buf;
    reg [DATA_TYPE_WIDTH - 1:0] data_type_wr_buf;
    
    reg [1:0] state_counter_rd;
    reg [1:0] state_counter_wr;
    
    wire rd_ins_sync;
    wire wr_ins_sync;
    
    reg [DOUBLEWORD_WIDTH - 1:0] data_bus_wr_buf;
    
    reg wr_clk;         // Write clock 
    wire [ADDR_WIDTH - 1:0] locate_addr_wr  [0:ADDR_DEPTH - 1];
    wire                    reg_clk         [0:ADDR_DEPTH - 1];
    
    // Common state 
    localparam INIT_STATE = 3;
    localparam IDLE_STATE = 0;
    localparam LOAD_ADDR_STATE = 1;
    // Write state
    localparam WRITE_STATE = 2;
    
    // Reserved register management 
    genvar index_reserved_reg;
    generate
        for(index_reserved_reg = 0; index_reserved_reg < RESERVED_REG_AMOUNT; index_reserved_reg = index_reserved_reg + 1) begin
            assign reserved_registers[index_reserved_reg] = registers[index_reserved_reg];
        end
    endgenerate
    
    // Read management
    reg rd_occupy_start;
    reg rd_occupy_stop;
    wire rd_occupied;
    
    assign rd_occupied = rd_occupy_start ^ rd_occupy_stop;
    assign rd_idle = ~rd_occupied;
    
    always @(posedge rd_ins, negedge rst_n) begin
        if(!rst_n) rd_occupy_start <= 0;
        else rd_occupy_start <= ~rd_occupy_stop;
    end
    
    edge_detector rd_sig_management
                            (    
                            .clk(clk),
                            .sig_in(rd_ins),
                            .out(rd_ins_sync),
                            .rst_n(rst_n)
                            );
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            state_counter_rd <= IDLE_STATE;
            addr_rd_buf <= DEFAULT_ADDR;
            rd_occupy_stop <= 0;
        end
        else begin
            case(state_counter_rd)
                IDLE_STATE: begin
                    if(rd_ins_sync) begin
                        state_counter_rd <= LOAD_ADDR_STATE;
                        addr_rd_buf <= addr_rd;
                        data_type_rd_buf <= data_type_rd;
                    end
                end
                LOAD_ADDR_STATE: begin
                    state_counter_rd <= IDLE_STATE;
                    rd_occupy_stop <= rd_occupy_start;
                end
                default: state_counter_rd <= IDLE_STATE;
            endcase
        end
    end
    // Multiplexer <ADDR_DEPTH> to 1
    wire [DOUBLEWORD_WIDTH - 1:0] data_bus_dword_rd [0:ADDR_DEPTH - 1];
    genvar rd_index;
    generate
        for(rd_index = 0; rd_index < ADDR_DEPTH - 7; rd_index = rd_index + 1) begin : read_select_block
            assign data_bus_dword_rd[rd_index] = (data_type_rd_buf == DATA_TYPE_BYTE_ENCODE) ? {56'h00000000000000, registers[rd_index]} : 
                                                 (data_type_rd_buf == DATA_TYPE_WORD_ENCODE) ? {32'h000000, registers[rd_index + 3], registers[rd_index + 2], registers[rd_index + 1], registers[rd_index + 0]} :
                                                 {registers[rd_index + 7], registers[rd_index + 6], registers[rd_index + 5], registers[rd_index + 4], registers[rd_index + 3], registers[rd_index + 2], registers[rd_index + 1], registers[rd_index + 0]};
        end
        assign data_bus_dword_rd[ADDR_DEPTH - 7] =  (data_type_rd_buf == DATA_TYPE_BYTE_ENCODE) ? {56'h00000000000000, registers[ADDR_DEPTH - 7]} : 
                                                    (data_type_rd_buf == DATA_TYPE_WORD_ENCODE) ? {32'h000000, registers[ADDR_DEPTH - 4], registers[ADDR_DEPTH - 5], registers[ADDR_DEPTH - 6], registers[ADDR_DEPTH - 7]} :
                                                    {8'h00, registers[ADDR_DEPTH - 1], registers[ADDR_DEPTH - 2], registers[ADDR_DEPTH - 3], registers[ADDR_DEPTH - 4], registers[ADDR_DEPTH - 5], registers[ADDR_DEPTH - 6], registers[ADDR_DEPTH - 7]};
        assign data_bus_dword_rd[ADDR_DEPTH - 6] =  (data_type_rd_buf == DATA_TYPE_BYTE_ENCODE) ? {56'h00000000000000, registers[ADDR_DEPTH - 6]} : 
                                                    (data_type_rd_buf == DATA_TYPE_WORD_ENCODE) ? {32'h000000, registers[ADDR_DEPTH - 3], registers[ADDR_DEPTH - 4], registers[ADDR_DEPTH - 5], registers[ADDR_DEPTH - 6]} :
                                                    {16'h0000, registers[ADDR_DEPTH - 1], registers[ADDR_DEPTH - 2], registers[ADDR_DEPTH - 3], registers[ADDR_DEPTH - 4], registers[ADDR_DEPTH - 5], registers[ADDR_DEPTH - 6]};
        assign data_bus_dword_rd[ADDR_DEPTH - 5] =  (data_type_rd_buf == DATA_TYPE_BYTE_ENCODE) ? {56'h00000000000000, registers[ADDR_DEPTH - 5]} : 
                                                    (data_type_rd_buf == DATA_TYPE_WORD_ENCODE) ? {32'h000000, registers[ADDR_DEPTH - 2], registers[ADDR_DEPTH - 3], registers[ADDR_DEPTH - 4], registers[ADDR_DEPTH - 5]} :
                                                    {24'h000000, registers[ADDR_DEPTH - 1], registers[ADDR_DEPTH - 2], registers[ADDR_DEPTH - 3], registers[ADDR_DEPTH - 4], registers[ADDR_DEPTH - 5]};
        assign data_bus_dword_rd[ADDR_DEPTH - 4] =  (data_type_rd_buf == DATA_TYPE_BYTE_ENCODE) ? {56'h00000000000000, registers[ADDR_DEPTH - 4]} : 
                                                    (data_type_rd_buf == DATA_TYPE_WORD_ENCODE) ? {32'h000000, registers[ADDR_DEPTH - 1], registers[ADDR_DEPTH - 2], registers[ADDR_DEPTH - 3], registers[ADDR_DEPTH - 4]} :
                                                    {32'h00000000, registers[ADDR_DEPTH - 1], registers[ADDR_DEPTH - 2], registers[ADDR_DEPTH - 3], registers[ADDR_DEPTH - 4]};
        assign data_bus_dword_rd[ADDR_DEPTH - 3] =  (data_type_rd_buf == DATA_TYPE_BYTE_ENCODE) ? {56'h00000000000000, registers[ADDR_DEPTH - 3]} : 
                                                    (data_type_rd_buf == DATA_TYPE_WORD_ENCODE) ? {40'h000000000, registers[ADDR_DEPTH - 1], registers[ADDR_DEPTH - 2], registers[ADDR_DEPTH - 3]} :
                                                    {40'h0000000000, registers[ADDR_DEPTH - 1], registers[ADDR_DEPTH - 2], registers[ADDR_DEPTH - 3]};
        assign data_bus_dword_rd[ADDR_DEPTH - 2] =  (data_type_rd_buf == DATA_TYPE_BYTE_ENCODE) ? {56'h00000000000000, registers[ADDR_DEPTH - 2]} : 
                                                    (data_type_rd_buf == DATA_TYPE_WORD_ENCODE) ? {48'h00000000000, registers[ADDR_DEPTH - 1], registers[ADDR_DEPTH - 2]} :
                                                    {48'h000000000000, registers[ADDR_DEPTH - 1], registers[ADDR_DEPTH - 2]};
        assign data_bus_dword_rd[ADDR_DEPTH - 1] =  (data_type_rd_buf == DATA_TYPE_BYTE_ENCODE) ? {56'h00000000000000, registers[ADDR_DEPTH - 1]} : 
                                                    (data_type_rd_buf == DATA_TYPE_WORD_ENCODE) ? {56'h00000000000000, registers[ADDR_DEPTH - 1]} :
                                                    {56'h00000000000000, registers[ADDR_DEPTH - 1]};
        
    endgenerate
    
    assign data_bus_rd = data_bus_dword_rd[addr_rd_buf];
    
    // Write management 
    reg wr_occupy_start;
    reg wr_occupy_stop;
    wire wr_occupied;
    
    assign wr_occupied = wr_occupy_start ^ wr_occupy_stop;
    assign wr_idle = ~wr_occupied;
    
    always @(posedge wr_ins, negedge rst_n) begin
        if(!rst_n) wr_occupy_start <= 0;
        else wr_occupy_start <= ~wr_occupy_stop;
    end 
    
    edge_detector wr_sig_management
                            (    
                            .clk(clk),
                            .sig_in(wr_ins),
                            .out(wr_ins_sync),
                            .rst_n(rst_n)
                            );             
                            
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            state_counter_wr <= IDLE_STATE;
            addr_wr_buf <= DEFAULT_ADDR;
            wr_clk <= 0;
            // State 
            wr_occupy_stop <= 0;
        end
        else begin
            case(state_counter_wr) 
                IDLE_STATE: begin
                    if(wr_ins_sync) begin
                        state_counter_wr <= LOAD_ADDR_STATE;
                        addr_wr_buf <= addr_wr;
                        data_bus_wr_buf <= data_bus_wr;
                        data_type_wr_buf <= data_type_wr;
                    end
                    wr_clk <= 0;
                end
                LOAD_ADDR_STATE: begin
                    state_counter_wr <= IDLE_STATE;
                    wr_clk <= 1;
                    wr_occupy_stop <= wr_occupy_start;
                end
//                WRITE_STATE: begin
//                    state_counter_wr <= IDLE_STATE;
//                    wr_clk <= 0;
//                end
                default: state_counter_wr <= IDLE_STATE;
            endcase 
        end
    end

    genvar i;
    genvar i_1;
    
    generate
        for(i = 0; i < ADDR_DEPTH; i = i + 1) begin
            assign locate_addr_wr[i] = i - addr_wr_buf;
        end
        assign reg_clk[6] = (((data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) & ((locate_addr_wr[6] == 6) | (locate_addr_wr[6] == 5) | (locate_addr_wr[6] == 4) | (locate_addr_wr[6] == 3) | (locate_addr_wr[6] == 2) | (locate_addr_wr[6] == 1))) | 
                             ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) & ((locate_addr_wr[6] == 3) | (locate_addr_wr[6] == 2) | (locate_addr_wr[6] == 1))) | 
                              (locate_addr_wr[6] == 0)) ? wr_clk : 1'b0;
        
        assign reg_clk[5] = (((data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) & ((locate_addr_wr[5] == 5) | (locate_addr_wr[5] == 4) | (locate_addr_wr[5] == 3) | (locate_addr_wr[5] == 2) | (locate_addr_wr[5] == 1))) | 
                             ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) & ((locate_addr_wr[5] == 3) | (locate_addr_wr[5] == 2) | (locate_addr_wr[5] == 1))) | 
                              (locate_addr_wr[5] == 0)) ? wr_clk : 1'b0;
        
        assign reg_clk[4] = (((data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) & ((locate_addr_wr[4] == 4) | (locate_addr_wr[4] == 3) | (locate_addr_wr[4] == 2) | (locate_addr_wr[4] == 1))) | 
                             ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) & ((locate_addr_wr[4] == 3) | (locate_addr_wr[4] == 2) | (locate_addr_wr[4] == 1))) | 
                              (locate_addr_wr[4] == 0)) ? wr_clk : 1'b0;
        
        assign reg_clk[3] = (((data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) & ((locate_addr_wr[3] == 3) | (locate_addr_wr[3] == 2) | (locate_addr_wr[3] == 1))) | 
                             ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) & ((locate_addr_wr[3] == 3) | (locate_addr_wr[3] == 2) | (locate_addr_wr[3] == 1))) | 
                              (locate_addr_wr[3] == 0)) ? wr_clk : 1'b0;
        
        assign reg_clk[2] = (((data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) & ((locate_addr_wr[2] == 2) | (locate_addr_wr[2] == 1))) | 
                             ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) & ((locate_addr_wr[2] == 2) | (locate_addr_wr[2] == 1))) | 
                              (locate_addr_wr[2] == 0)) ? wr_clk : 1'b0;
        
        assign reg_clk[1] = (((data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) & (locate_addr_wr[1] == 1)) | 
                             ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) & (locate_addr_wr[1] == 1)) | 
                              (locate_addr_wr[1] == 0)) ? wr_clk : 1'b0;
        
        assign reg_clk[0] = (locate_addr_wr[0] == 0) ? wr_clk : 1'b0;
        
        for(i = 7; i < ADDR_DEPTH; i = i + 1) begin
            assign reg_clk[i] = (((data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) & ((locate_addr_wr[i] == 7) | (locate_addr_wr[i] == 6) | (locate_addr_wr[i] == 5) | (locate_addr_wr[i] == 4) | (locate_addr_wr[i] == 3) | (locate_addr_wr[i] == 2) | (locate_addr_wr[i] == 1))) | 
                                 ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) & ((locate_addr_wr[i] == 3) | (locate_addr_wr[i] == 2) | (locate_addr_wr[i] == 1))) | 
                                  (locate_addr_wr[i] == 0)) ? wr_clk : 1'b0;
                                 
        end
        for(i = 0; i < ADDR_DEPTH; i = i + 1) begin
            if(i < RESERVED_REG_AMOUNT) begin : reserved_block
                always @(posedge reg_clk[i], negedge rst_n) begin
                    if(!rst_n) registers[i] <= RESERVED_REG_DEFAULT[i];
                    else begin
                        case(locate_addr_wr[i]) 
                            0: begin
                                registers[i] <= data_bus_wr_buf[7:0];
                            end
                            1: begin
                                registers[i] <= ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) | (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE)) ? data_bus_wr_buf[15:8] : registers[i];
                            end
                            2: begin
                                registers[i] <= ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) | (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE)) ? data_bus_wr_buf[23:16] : registers[i];
                            end
                            3: begin
                                registers[i] <= ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) | (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE)) ? data_bus_wr_buf[31:24] : registers[i];
                            end
                            4: begin
                                registers[i] <= (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? data_bus_wr_buf[39:32] : registers[i];
                            end
                            5: begin
                                registers[i] <= (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? data_bus_wr_buf[47:40] : registers[i];
                            end
                            6: begin
                                registers[i] <= (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? data_bus_wr_buf[55:48] : registers[i];
                            end
                            7: begin
                                registers[i] <= (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? data_bus_wr_buf[63:56] : registers[i];
                            end
                            default: begin
                                registers[i] <= registers[i];
                            end
                        endcase
                    end
                end
            end
            else begin : data_block
                always @(posedge reg_clk[i], negedge rst_n) begin
                    if(!rst_n) registers[i] <= DEFAULT_DATA;
                    else begin
                        case(locate_addr_wr[i]) 
                            0: begin
                                registers[i] <= data_bus_wr_buf[7:0];
                            end
                            1: begin
                                registers[i] <= ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) | (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE)) ? data_bus_wr_buf[15:8] : registers[i];
                            end
                            2: begin
                                registers[i] <= ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) | (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE)) ? data_bus_wr_buf[23:16] : registers[i];
                            end
                            3: begin
                                registers[i] <= ((data_type_wr_buf == DATA_TYPE_WORD_ENCODE) | (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE)) ? data_bus_wr_buf[31:24] : registers[i];
                            end
                            4: begin
                                registers[i] <= (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? data_bus_wr_buf[39:32] : registers[i];
                            end
                            5: begin
                                registers[i] <= (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? data_bus_wr_buf[47:40] : registers[i];
                            end
                            6: begin
                                registers[i] <= (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? data_bus_wr_buf[55:48] : registers[i];
                            end
                            7: begin
                                registers[i] <= (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? data_bus_wr_buf[63:56] : registers[i];
                            end
                            default: begin
                                registers[i] <= registers[i];
                            end
                        endcase
                    end
                end
            end
        end
    endgenerate
    
    // Debug area
    assign registers_wire = registers;
endmodule

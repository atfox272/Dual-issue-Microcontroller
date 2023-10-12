`define DEBUG
//`define RESERVED_REG_DEBUG
// Byte order: Little Endian
module ram_pm // (Just read doubleword (8bytes)) 
    #(
        parameter DATA_WIDTH        = 8,
        parameter BYTE_WIDTH        = 8,
        parameter WORD_WIDTH        = BYTE_WIDTH*4,
        parameter DOUBLEWORD_WIDTH  = BYTE_WIDTH*8,
        parameter ADDR_DEPTH        = 2048,  // <=> 512 bytes
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
        `ifdef RESERVED_REG_DEBUG
        parameter      RESERVED_REG_AMOUNT                             =  17,
        parameter byte RESERVED_REG_DEFAULT[0:RESERVED_REG_AMOUNT - 1] = {8'b00000000,  // address 0x00  (PORT_A)
                                                                          8'b00000000,  // address 0x01  (PORT_B)
                                                                          8'b00000000,  // address 0x02  (PORT_C)
                                                                          8'b00000000,  // address 0x03  (DEBUGGER) 
                                                                          8'b00100011,  // address 0x04  (UART_1_RX_CONFIG)
                                                                          8'b00100011,  // address 0x05  (UART_1_TX_CONFIG)
                                                                          8'b00000000,  // address 0x06  (COM_PERIPHERAL) // Do not enable (set 1) in initial state
                                                                          8'b00000000,  // address 0x07  (NOTHING)
                                                                          8'b10001111,  // address 0x08  (UART_2_RX_CONFIG)
                                                                          8'b10001111,  // address 0x09  (UART_2_TX_CONFIG)
                                                                          8'b11111000,  // address 0x0A  (SPI_CONFIG)
                                                                          8'b00000000,  // address 0x0B  (I2C_CONFIG)
                                                                          8'b00000000,  // address 0x0C  (EXTERNAL_INT_CONFIG)
                                                                          8'b00000000,  // address 0x0D  (PINCHANGE_INT_CONFIG)
                                                                          8'b00000000,  // address 0x0E  (TIMER_INT_CONFIG)
                                                                          8'b11111111,  // address 0x0F  (TIMER_LIMIT_VALUE_H)
                                                                          8'b11111111} // address 0x10  (TIMER_LIMIT_VALUE_L)
        `else                                                                   
        // Reserved register in RAM (for Peripheral configuration)
        parameter RESERVED_REG_AMOUNT           = 1,
        // Set default data for RESERVED_REGISTERs
        parameter byte RESERVED_REG_DEFAULT[0:RESERVED_REG_AMOUNT - 1] = {8'b00000000}  // address 0x0
        `endif
    )
    (
    input   wire                        clk,
    
    input   wire    [DOUBLEWORD_WIDTH - 1:0]    data_bus_wr,
    output  logic   [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd,
    
    
    input   wire    [DATA_TYPE_WIDTH - 1:0]     data_type_rd,
    input   wire    [DATA_TYPE_WIDTH - 1:0]     data_type_wr,
    
    input   wire    [ADDR_WIDTH - 1:0]  addr_rd,
    input   wire    [ADDR_WIDTH - 1:0]  addr_wr,
    
    input   wire                        rd_ins,
    input   wire                        wr_ins,
    
    // Flag read 
    output  wire                        invalid_rd_flag,
    output  wire                        invalid_wr_flag,
    output  wire                        overflow_rd_flag,
    
    // State
    output  wire                        rd_idle,
    output  wire                        wr_idle,
    
    // Optional (to save configuration data)
    output  wire    [DATA_WIDTH - 1:0]  reserved_registers  [0:RESERVED_REG_AMOUNT - 1],
    
    input   wire                        rst_n
    
    // Debug
    `ifdef DEBUG
    ,output  wire    [DATA_WIDTH - 1:0]  registers_wire [0: ADDR_DEPTH - 1]
    `endif
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
    reg     [DATA_WIDTH - 1:0]      registers [0: ADDR_DEPTH - 1];
    
    reg     [ADDR_WIDTH - 1:0]      addr_rd_buf;
    reg     [ADDR_WIDTH - 1:0]      addr_wr_buf;
    
    logic   [DATA_TYPE_WIDTH - 1:0] data_type_wr_buf;
    wire                            data_type_dword_en;
    wire                            data_type_word_en;
    wire                            data_type_byte_en;
    wire                            data_type_dword_word_en;
    
    reg [1:0] state_counter_rd;
    reg [1:0] state_counter_wr;
    
    wire rd_ins_sync;
    wire wr_ins_sync;
    
    reg [DOUBLEWORD_WIDTH - 1:0] data_bus_wr_buf;
    
    reg wr_clk_en;         // Write clock 
    wire [ADDR_WIDTH - 1:0] locate_addr_wr  [0:ADDR_DEPTH - 1];
    wire                    reg_clk         [0:ADDR_DEPTH - 1];
    
    // Common state 
    localparam INIT_STATE = 3;
    localparam IDLE_STATE = 0;
    localparam LOAD_ADDR_STATE = 1;
    // Write state
    localparam WRITE_STATE = 2;
    // Address encode doubleword
    localparam ADDR_DWORD_ENCODE = $clog2(ADDR_DEPTH / 8);
    // Address encode word
    localparam ADDR_WORD_ENCODE  = $clog2(ADDR_DEPTH / 4);
    // Address encode byte
    localparam ADDR_BYTE_ENCODE  = $clog2(ADDR_DEPTH);
    
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
    // Declare logic
    wire [DOUBLEWORD_WIDTH - 1:0] data_bus_dword_rd [0:ADDR_DEPTH/4 - 1];
    wire [DOUBLEWORD_WIDTH - 1:0] dword_multiplexer;
    wire [ADDR_WORD_ENCODE - 1:0] address_rd_dword_encode;              // dword, but it's encoded in word case
    wire                          invalid_rd_dword_flag;
    
    assign address_rd_dword_encode = addr_rd_buf[ADDR_WIDTH - 1: 2];    // Index of double word 
    
    assign invalid_rd_flag       = invalid_rd_dword_flag;
    assign invalid_rd_dword_flag = (addr_rd_buf[1:0] != 2'b00);       // Valid alignment (divisible by 8)
    assign overflow_rd_flag      = (addr_rd_buf == ADDR_DEPTH - 4);
    
    // Byte order: Little endian (MSB is at highest addr)
    // Double word multiplexer (8bytes)
    for(genvar rd_index_dword = 0; rd_index_dword < ADDR_DEPTH / 4; rd_index_dword = rd_index_dword + 1) begin
        if(rd_index_dword == ADDR_DEPTH / 4 - 1) begin  // Last index is overflow
            assign data_bus_dword_rd[rd_index_dword] = {32'h00000000, registers[rd_index_dword*4 + 3], registers[rd_index_dword*4 + 2], registers[rd_index_dword*4 + 1], registers[rd_index_dword*4 + 0]};        
        end
        else begin
            assign data_bus_dword_rd[rd_index_dword] = {registers[rd_index_dword*4 + 7], registers[rd_index_dword*4 + 6], registers[rd_index_dword*4 + 5], registers[rd_index_dword*4 + 4], registers[rd_index_dword*4 + 3], registers[rd_index_dword*4 + 2], registers[rd_index_dword*4 + 1], registers[rd_index_dword*4 + 0]};
        end
    end
    
    assign dword_multiplexer = data_bus_dword_rd[address_rd_dword_encode];
    assign data_bus_rd = dword_multiplexer;
    
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
    
    assign data_type_dword_en = (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE);
    assign data_type_word_en  = (data_type_wr_buf == DATA_TYPE_WORD_ENCODE);
    assign data_type_byte_en  = (data_type_wr_buf == DATA_TYPE_BYTE_ENCODE);
    assign data_type_dword_word_en  = data_type_word_en | data_type_dword_en;
    
    edge_detector wr_sig_management
                            (    
                            .clk(clk),
                            .sig_in(wr_ins),
                            .out(wr_ins_sync),
                            .rst_n(rst_n)
                            );             
    
    wire                           invalid_wr_dword_flag;
    wire                           invalid_wr_word_flag;
    
    assign invalid_wr_flag       =      invalid_wr_dword_flag | invalid_wr_word_flag;             
    assign invalid_wr_dword_flag =      (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? {addr_wr_buf[2:0] != 3'b000} : 1'b0;                   
    assign invalid_wr_word_flag  =      (data_type_wr_buf == DATA_TYPE_WORD_ENCODE)       ? {addr_wr_buf[1:0] != 2'b00}  : 1'b0;                   
    
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            state_counter_wr <= IDLE_STATE;
            addr_wr_buf <= DEFAULT_ADDR;
            wr_clk_en <= 0;
            // State 
            wr_occupy_stop <= 0;
        end
        else begin
            wr_clk_en <= 0;
            case(state_counter_wr) 
                IDLE_STATE: begin
                    if(wr_ins_sync) begin
                        state_counter_wr <= LOAD_ADDR_STATE;
                        addr_wr_buf <= addr_wr;
                        data_bus_wr_buf <= data_bus_wr;
                        data_type_wr_buf <= data_type_wr;
                    end
                end
                LOAD_ADDR_STATE: begin
                    state_counter_wr <= IDLE_STATE;
                    wr_occupy_stop <= wr_occupy_start;
                    wr_clk_en <= (invalid_wr_word_flag) ? 1'b0 : 1'b1;
                end
                default: state_counter_wr <= IDLE_STATE;
            endcase 
        end
    end

    // Write Clock enable management
    wire in_dword_block         [0:ADDR_DEPTH/8 - 1];
    wire in_word_block          [0:ADDR_DEPTH/4 - 1];
    wire in_byte_block          [0:ADDR_DEPTH/1 - 1];
    wire element_in_block       [0:ADDR_DEPTH - 1];
    wire element_clk_en         [0:ADDR_DEPTH - 1];
    
    wire [ADDR_DWORD_ENCODE - 1:0] dword_block_cur_index;
    wire [ADDR_WORD_ENCODE - 1:0]  word_block_cur_index;
    
    assign dword_block_cur_index = addr_wr_buf[ADDR_WIDTH - 1:3];
    assign word_block_cur_index  = addr_wr_buf[ADDR_WIDTH - 1:2];
    
    for(genvar dword_block_i = 0; dword_block_i < ADDR_DEPTH/8; dword_block_i = dword_block_i + 1) begin
        assign in_dword_block[dword_block_i] = (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE) ? (dword_block_cur_index == dword_block_i) : 1'b0;
    end
    for(genvar word_block_i = 0; word_block_i < ADDR_DEPTH/4; word_block_i = word_block_i + 1) begin
        assign in_word_block[word_block_i] = (data_type_wr_buf == DATA_TYPE_WORD_ENCODE) ? (word_block_cur_index == word_block_i) : 1'b0;
    end
    for(genvar byte_block_i = 0; byte_block_i < ADDR_DEPTH/1; byte_block_i = byte_block_i + 1) begin
        assign in_byte_block[byte_block_i] = (data_type_wr_buf == DATA_TYPE_BYTE_ENCODE) ? (addr_wr_buf == byte_block_i) : 1'b0;
    end
    for(genvar element_i = 0; element_i < ADDR_DEPTH; element_i = element_i + 1) begin
        assign element_in_block[element_i] = in_dword_block[element_i/8] | in_word_block[element_i/4] | in_byte_block[element_i/1];
    end 
    for(genvar element_i = 0; element_i < ADDR_DEPTH; element_i = element_i + 1) begin
        assign element_clk_en[element_i] = element_in_block[element_i] & wr_clk_en;
    end 
    // Write Data management
    logic [DATA_WIDTH - 1:0] data_wr_element [0: ADDR_DEPTH - 1];
    
    // Byte order: Little Endian
    for(genvar element_i = 0; element_i < ADDR_DEPTH; element_i = element_i + 1) begin
        if(element_i % 8 == 0) begin
            assign data_wr_element[element_i] = data_bus_wr_buf[7:0];
        end
        else if(element_i % 8 == 1) begin
            assign data_wr_element[element_i] = (data_type_dword_en | data_type_word_en) ? data_bus_wr_buf[15:8] :
                                                (data_type_byte_en)  ? data_bus_wr_buf[7:0] : 8'h00;
        end
        else if(element_i % 8 == 2) begin
            assign data_wr_element[element_i] = (data_type_dword_en | data_type_word_en) ? data_bus_wr_buf[23:16] :
                                                (data_type_byte_en)  ? data_bus_wr_buf[7:0] : 8'h00;
        end
        else if(element_i % 8 == 3) begin
            assign data_wr_element[element_i] = (data_type_dword_en | data_type_word_en) ? data_bus_wr_buf[31:24] :
                                                (data_type_byte_en)  ? data_bus_wr_buf[7:0] : 8'h00;
        end
        else if(element_i % 8 == 4) begin
            assign data_wr_element[element_i] = (data_type_dword_en) ? data_bus_wr_buf[39:32] :
                                                (data_type_word_en)  ? data_bus_wr_buf[7:0] : 
                                                (data_type_byte_en)  ? data_bus_wr_buf[7:0] : 8'h00;
        end
        else if(element_i % 8 == 5) begin
            assign data_wr_element[element_i] = (data_type_dword_en) ? data_bus_wr_buf[47:40] : 
                                                (data_type_word_en)  ? data_bus_wr_buf[15:8] :
                                                (data_type_byte_en)  ? data_bus_wr_buf[7:0] : 8'h00;
        end
        else if(element_i % 8 == 6) begin
            assign data_wr_element[element_i] = (data_type_dword_en) ? data_bus_wr_buf[55:48] : 
                                                (data_type_word_en)  ? data_bus_wr_buf[23:16] :
                                                (data_type_byte_en)  ? data_bus_wr_buf[7:0] : 8'h00;
        end
        else if(element_i % 8 == 7) begin
            assign data_wr_element[element_i] = (data_type_dword_en) ? data_bus_wr_buf[63:56] : 
                                                (data_type_word_en)  ? data_bus_wr_buf[31:24] :
                                                (data_type_byte_en)  ? data_bus_wr_buf[7:0] : 8'h00;
        end
    end
    
    for(genvar element_i = 0; element_i < ADDR_DEPTH; element_i = element_i + 1) begin
        if(element_i < RESERVED_REG_AMOUNT) begin : reserved_block
            always @(posedge clk) begin
                if(!rst_n) begin
                    registers[element_i] <= RESERVED_REG_DEFAULT[element_i];
                end
                else if(element_clk_en[element_i]) begin
                    registers[element_i] <= data_wr_element[element_i];
                end
            end
        end
        else begin
            always @(posedge clk) begin
                if(!rst_n) begin
                    registers[element_i] <= 8'h00;
                end
                else if(element_clk_en[element_i]) begin
                    registers[element_i] <= data_wr_element[element_i];
                end
            end
        end
    end
    // Debug area
    assign registers_wire = registers;
endmodule

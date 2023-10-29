`define DEBUG
//`define RESERVED_REG_DEBUG
// Byte order:    Little Endian
// Acess option:  1)  1wr-1rd
//                2)  1wr-2rd (set DUAL_READ_HANDLER = 1)
module ram
    #(
        parameter DATA_WIDTH        = 8,
        parameter BYTE_WIDTH        = 8,
        parameter WORD_WIDTH        = BYTE_WIDTH*4,
        parameter DOUBLEWORD_WIDTH  = BYTE_WIDTH*8,
        parameter ADDR_DEPTH        = 1024,  // <=> 512 bytes
        parameter ADDR_WIDTH        = $clog2(ADDR_DEPTH),
        
        // Read handler
        parameter DUAL_READ_HANDLER = 0,    // Increase power consumption
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
    output  logic   [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd_1,
    output  logic   [DOUBLEWORD_WIDTH - 1:0]    data_bus_rd_2,
    
    
    input   wire    [DATA_TYPE_WIDTH - 1:0]     data_type_rd_1,
    input   wire    [DATA_TYPE_WIDTH - 1:0]     data_type_rd_2,
    input   wire    [DATA_TYPE_WIDTH - 1:0]     data_type_wr,
    
    input   wire    [ADDR_WIDTH - 1:0]          addr_rd_1,
    input   wire    [ADDR_WIDTH - 1:0]          addr_rd_2,
    input   wire    [ADDR_WIDTH - 1:0]          addr_wr,
    
    input   wire                                rd_en_1,
    input   wire                                rd_en_2,
    input   wire                                wr_ins,
    
    // Flag read 
    output  wire                                invalid_rd_flag_1,
    output  wire                                invalid_rd_flag_2,
    output  wire                                invalid_wr_flag,
    
    // State
    output  wire                                rd_idle_1,
    output  wire                                rd_idle_2,
    output  wire                                wr_idle,
    
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
    
//    wire    [ADDR_WIDTH - 1:0]  addr_rd_1;
//    wire    [ADDR_WIDTH - 1:0]  addr_wr;
//    assign addr_rd_1 = 27;
//    assign addr_wr = 02;

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
    
    
    // Set of registers
    reg     [DATA_WIDTH - 1:0]      registers [0: ADDR_DEPTH - 1];
    
    reg     [ADDR_WIDTH - 1:0]      addr_wr_buf;
    reg     [DOUBLEWORD_WIDTH - 1:0]data_bus_wr_buf;
    logic   [DATA_TYPE_WIDTH - 1:0] data_type_wr_buf;
    
    wire                            data_type_dword_en;
    wire                            data_type_word_en;
    wire                            data_type_byte_en;
    wire                            data_type_dword_word_en;
    
    reg [1:0] state_counter_wr;
    
    reg wr_clk_en;         // Write clock 
    wire [ADDR_WIDTH - 1:0] locate_addr_wr  [0:ADDR_DEPTH - 1];
    wire                    reg_clk         [0:ADDR_DEPTH - 1];
    
    // Reserved register management 
    genvar index_reserved_reg;
    generate
        for(index_reserved_reg = 0; index_reserved_reg < RESERVED_REG_AMOUNT; index_reserved_reg = index_reserved_reg + 1) begin
            assign reserved_registers[index_reserved_reg] = registers[index_reserved_reg];
        end
    endgenerate
    
    assign data_type_dword_en = (data_type_wr_buf == DATA_TYPE_DOUBLEWORD_ENCODE);
    assign data_type_word_en  = (data_type_wr_buf == DATA_TYPE_WORD_ENCODE);
    assign data_type_byte_en  = (data_type_wr_buf == DATA_TYPE_BYTE_ENCODE);
    assign data_type_dword_word_en  = data_type_word_en | data_type_dword_en;
    
    
    // Aligment block Index 
    wire [DOUBLEWORD_WIDTH - 1:0] data_bus_dword_rd [0:ADDR_DEPTH/8 - 1];
    wire [WORD_WIDTH - 1:0]       data_bus_word_rd [0:ADDR_DEPTH/4 - 1];
    wire [BYTE_WIDTH - 1:0]       data_bus_byte_rd [0:ADDR_DEPTH/1 - 1];
    // Byte order: Little endian (MSB is at highest addr)
    // Double word multiplexer (8bytes)
    for(genvar rd_index_dword = 0; rd_index_dword < ADDR_DEPTH / 8; rd_index_dword = rd_index_dword + 1) begin
        assign data_bus_dword_rd[rd_index_dword] = {registers[rd_index_dword*8 + 7], registers[rd_index_dword*8 + 6], registers[rd_index_dword*8 + 5], registers[rd_index_dword*8 + 4], registers[rd_index_dword*8 + 3], registers[rd_index_dword*8 + 2], registers[rd_index_dword*8 + 1], registers[rd_index_dword*8 + 0]};
    end
        // Word multiplexer (4bytes)
    for(genvar rd_index_word = 0; rd_index_word < ADDR_DEPTH / 4; rd_index_word = rd_index_word + 1) begin
        assign data_bus_word_rd[rd_index_word] = {registers[rd_index_word*4 + 3], registers[rd_index_word*4 + 2], registers[rd_index_word*4 + 1], registers[rd_index_word*4 + 0]};
    end
    // Byte multiplexer (1byte)
    for(genvar rd_index_byte = 0; rd_index_byte < ADDR_DEPTH / 1; rd_index_byte = rd_index_byte + 1) begin
        assign data_bus_byte_rd[rd_index_byte] = registers[rd_index_byte];
    end
    // Read handler 1
    wire    [ADDR_WIDTH - 1:0]      addr_rd_internal_1;
    reg     [DATA_TYPE_WIDTH - 1:0] data_type_rd_internal_1;
    assign rd_idle_1 = 1'b1;
    assign addr_rd_internal_1      = (rd_en_1) ? addr_rd_1 : 0;
    assign data_type_rd_internal_1 = (rd_en_1) ? data_type_rd_1 : 0;
    // Multiplexer
    wire [DOUBLEWORD_WIDTH - 1:0] dword_mulitplexer_1;
    wire [DOUBLEWORD_WIDTH - 1:0] word_mulitplexer_1;
    wire [DOUBLEWORD_WIDTH - 1:0] byte_mulitplexer_1;
    
    // Alignment
    wire [ADDR_DWORD_ENCODE - 1:0]  address_rd_dword_encode_1;
    wire                            invalid_rd_dword_flag_1;
    wire [ADDR_WORD_ENCODE - 1:0]   address_rd_word_encode_1;
    wire                            invalid_rd_word_flag_1;
    wire [ADDR_BYTE_ENCODE - 1:0]   address_rd_byte_encode_1;
        
    assign address_rd_dword_encode_1 = addr_rd_internal_1[ADDR_WIDTH - 1: 3];    // Index of double word 
    assign address_rd_word_encode_1  = addr_rd_internal_1[ADDR_WIDTH - 1: 2];    // Index of word
    assign address_rd_byte_encode_1  = addr_rd_internal_1;                       // Index of byte
        
    assign invalid_rd_flag_1        = invalid_rd_dword_flag_1 | invalid_rd_word_flag_1;
    assign invalid_rd_dword_flag_1  = (data_type_rd_internal_1 == DATA_TYPE_DOUBLEWORD_ENCODE) ? {addr_rd_internal_1[2:0] != 3'b000} : 1'b0;       // Valid alignment (divisible by 8)
    assign invalid_rd_word_flag_1   = (data_type_rd_internal_1 == DATA_TYPE_WORD_ENCODE)       ? {addr_rd_internal_1[1:0] != 2'b00}  : 1'b0;         // Valid alignment (divisible by 8)
        
    assign dword_mulitplexer_1 = data_bus_dword_rd[address_rd_dword_encode_1];
    assign word_mulitplexer_1  = {32'b00, data_bus_word_rd[address_rd_word_encode_1]};
    assign byte_mulitplexer_1  = {56'b00, data_bus_byte_rd[address_rd_byte_encode_1]};
    always_comb begin
        case(data_type_rd_internal_1)
            DATA_TYPE_DOUBLEWORD_ENCODE: data_bus_rd_1 = dword_mulitplexer_1;
            DATA_TYPE_WORD_ENCODE: data_bus_rd_1 = word_mulitplexer_1;
            DATA_TYPE_BYTE_ENCODE: data_bus_rd_1 = byte_mulitplexer_1;
            default: data_bus_rd_1 = {DOUBLEWORD_WIDTH{1'b0}};
        endcase 
    end   
    if(DUAL_READ_HANDLER == 0) begin                        : SINGLE_READ_HANDLER_BLOCK
        assign invalid_rd_flag_2        = 1'b0;
        assign rd_idle_2                = 1'b0;
        assign data_bus_rd_2            = {DOUBLEWORD_WIDTH{1'b0}};
    end
    else begin                                              : DUAL_READ_HANDLER_BLOCK
        // Read handler 2
        assign rd_idle_2               = 1'b1;
        wire    [ADDR_WIDTH - 1:0]      addr_rd_internal_2;
        reg     [DATA_TYPE_WIDTH - 1:0] data_type_rd_internal_2;
        assign addr_rd_internal_2      = (rd_en_2) ? addr_rd_2 : 0;
        assign data_type_rd_internal_2 = (rd_en_2) ? data_type_rd_2 : 0;
        // Multiplexer
        wire [DOUBLEWORD_WIDTH - 1:0] dword_mulitplexer_2;
        wire [DOUBLEWORD_WIDTH - 1:0] word_mulitplexer_2;
        wire [DOUBLEWORD_WIDTH - 1:0] byte_mulitplexer_2;
        
        // Alignment
        wire [ADDR_DWORD_ENCODE - 1:0]  address_rd_dword_encode_2;
        wire                            invalid_rd_dword_flag_2;
        wire [ADDR_WORD_ENCODE - 1:0]   address_rd_word_encode_2;
        wire                            invalid_rd_word_flag_2;
        wire [ADDR_BYTE_ENCODE - 1:0]   address_rd_byte_encode_2;
        
        assign address_rd_dword_encode_2 = addr_rd_internal_2[ADDR_WIDTH - 1: 3];    // Index of double word 
        assign address_rd_word_encode_2  = addr_rd_internal_2[ADDR_WIDTH - 1: 2];    // Index of word
        assign address_rd_byte_encode_2  = addr_rd_internal_2;                       // Index of byte
        
        assign invalid_rd_flag_2        = invalid_rd_dword_flag_2 | invalid_rd_word_flag_2;
        assign invalid_rd_dword_flag_2  = (data_type_rd_internal_2 == DATA_TYPE_DOUBLEWORD_ENCODE) ? {addr_rd_internal_2[2:0] != 3'b000} : 1'b0;       // Valid alignment (divisible by 8)
        assign invalid_rd_word_flag_2   = (data_type_rd_internal_2 == DATA_TYPE_WORD_ENCODE)       ? {addr_rd_internal_2[1:0] != 2'b00}  : 1'b0;         // Valid alignment (divisible by 8)
        
        assign dword_mulitplexer_2 = data_bus_dword_rd[address_rd_dword_encode_2];
        assign word_mulitplexer_2  = {32'b00, data_bus_word_rd[address_rd_word_encode_2]};
        assign byte_mulitplexer_2  = {56'b00, data_bus_byte_rd[address_rd_byte_encode_2]};
        always_comb begin
            case(data_type_rd_internal_2)
                DATA_TYPE_DOUBLEWORD_ENCODE: data_bus_rd_2 = dword_mulitplexer_2;
                DATA_TYPE_WORD_ENCODE: data_bus_rd_2 = word_mulitplexer_2;
                DATA_TYPE_BYTE_ENCODE: data_bus_rd_2 = byte_mulitplexer_2;
                default: data_bus_rd_2 = {DOUBLEWORD_WIDTH{1'b0}};
            endcase 
        end
    end
    
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
                    if(wr_ins) begin
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

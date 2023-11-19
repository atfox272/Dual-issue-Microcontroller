// ATI Bus Structure (Interface for slave)
// Address line format
// <CHANNEL_ID> - <ADDRESS_ID>
// Note:
//  - LOAD_TYPE:    + BYTE*         -> invalid (align to WORD automatically) (cause segmentation in buffer)
//                  + WORD          -> valid     
//                  + DOUBLE_WORD   -> valid
//
//  - STORE_TYPE:   + BYTE          -> valid 
//                  + WORD          -> valid     
//                  + DOUBLE_WORD   -> valid
//

module Atfox_exTensible_Interface
    #(
    // SYSTEM BUS parameter
    parameter DATA_BUS_WIDTH            = 64,
    parameter ADDR_BUS_WIDTH            = 64,
    parameter CHANNEL_WIDTH             = 2,
    parameter DEVICE_CHANNEL_ID         = 2'h00,
    parameter DATA_TYPE_WIDTH           = 2,    // DoubleWord - Word - Byte (-> 3 type)
    parameter BYTE_TYPE_ENCODE          = 0,
    parameter WORD_TYPE_ENCODE          = 1,
    parameter DOUBLEWORD_TYPE_ENCODE    = 2,
    // Device parameter 
    parameter DEVICE_DATA_WIDTH         = 8,
    
    // DEEP CONFIGURATION
    parameter INTERNAL_CLOCK            = 125000000,
    parameter PACKET_TIMEOUT            = 260416,       // 2 transactions timeout ( = 120Mhz / (9600 / 20))
    parameter MASTER_BUFFER_SIZE        = 4,
    parameter SLAVE_BUFFER_SIZE         = 32,           // 4 DoubleWords
    
    parameter POINTER_MASTER_BUF        = $clog2(MASTER_BUFFER_SIZE),
    parameter POINTER_SLAVE_BUF         = $clog2(MASTER_BUFFER_SIZE)
    )
    (
    input                               clk,
    // SYSTEM BUS interface ////////////////////
    // DATA_BUS
    input  [DATA_BUS_WIDTH - 1:0]       data_bus_in,
    inout  [DATA_BUS_WIDTH - 1:0]       data_bus_out,
    // ADDRESS_BUS
    input  [ADDR_BUS_WIDTH - 1:0]       addr_bus_in,
    // CONTROL_BUS
    inout                               buffer_rd_available,
    inout                               buffer_wr_available,
    input                               rd_req,
    input                               wr_req,
    input  [DATA_TYPE_WIDTH - 1:0]      data_type_encode,      // Doubleword - Word - Byte
    // DEVICE interface ////////////////////////
    input   [DEVICE_DATA_WIDTH - 1:0]   device_data_in,
    output  [DEVICE_DATA_WIDTH - 1:0]   device_data_out,
    output  reg                         device_wr_ins,
    output  reg                         device_rd_ins,
    input                               device_idle,            // Connect to DEVICE_IDLE port
    input                               device_rd_available,    // Buffer of device is available (not full)
    input                               device_wr_available,    // Buffer of device is available (not full)
    input                               device_rd_idle,         // Read-Device  is IDLE (~transaction_en)
    input                               device_wr_idle,         // Write-Device is IDLE (~transaction_en)
    input                               rst_n
    );
    
    localparam BYTE_WIDTH       = 8;
    localparam WORD_WIDTH       = 32;
    localparam DOUBLEWORD_WIDTH = 64;
    
    reg [DATA_BUS_WIDTH - 1:0]          master_buffer [0:MASTER_BUFFER_SIZE - 1];
    reg [DATA_TYPE_WIDTH - 1:0]         data_type_mbuf[0:MASTER_BUFFER_SIZE - 1];   // data_type of element in master_buffer
    reg [POINTER_MASTER_BUF - 1:0]      front_pointer_mbuf;
    reg [POINTER_MASTER_BUF - 1:0]      rear_pointer_mbuf;
    
    reg [DEVICE_DATA_WIDTH - 1:0]       slave_buffer  [0:SLAVE_BUFFER_SIZE - 1];
    reg [DATA_TYPE_WIDTH - 1:0]         data_type_sbuf[0:SLAVE_BUFFER_SIZE - 1];    // data_type of element in slave_buffer
    reg [POINTER_SLAVE_BUF - 1:0]       front_pointer_sbuf;
    reg [POINTER_SLAVE_BUF - 1:0]       rear_pointer_sbuf;
    
    wire                                wr_req_decode;
    wire                                rd_req_decode;
    wire                                full_mbuf;
    wire                                empty_mbuf;
    wire                                full_sbuf;
    wire                                empty_sbuf;
    logic[DATA_BUS_WIDTH - 1:0]         data_bus_out_buf;
    
    logic buffer_rd_valid;
    logic buffer_rd_valid_dword;
    logic buffer_rd_valid_word;
    
    // Address decoder   
    assign address_decoder      = (addr_bus_in[ADDR_BUS_WIDTH - 1: ADDR_BUS_WIDTH - CHANNEL_WIDTH] == DEVICE_CHANNEL_ID);
    assign data_bus_out         = (address_decoder) ? data_bus_out_buf  : {64{1'bz}};
    assign buffer_wr_available  = (address_decoder) ? ~full_mbuf        : {1{1'bz}};
    assign buffer_rd_available  = (address_decoder) ? ~buffer_rd_valid       : {1{1'bz}};
    assign wr_req_decode        = (address_decoder) ? wr_req : 1'b0;
    assign rd_req_decode        = (address_decoder) ? rd_req : 1'b0;
    
    assign full_mbuf = (rear_pointer_mbuf + 1) == front_pointer_mbuf;
    assign empty_mbuf= rear_pointer_mbuf == front_pointer_mbuf;
    
    always @(posedge clk) begin
        if(!rst_n) begin
            rear_pointer_mbuf <= 0;
        end 
        else if(wr_req) begin
            master_buffer[rear_pointer_mbuf] <= data_bus_in;
            data_type_mbuf[rear_pointer_mbuf] <= data_type_encode;
            rear_pointer_mbuf <= rear_pointer_mbuf + 1;
        end
    end
    localparam BLOCK_AMOUNT             = DATA_BUS_WIDTH / DEVICE_DATA_WIDTH;
    localparam BLOCK_INDEX_WIDTH        = $clog2(BLOCK_AMOUNT);
    
    logic[DEVICE_DATA_WIDTH - 1:0]      block_mux;
    reg  [BLOCK_INDEX_WIDTH - 1:0]      block_index;
    wire [DATA_BUS_WIDTH - 1:0]         front_element_mbuf;
    
    assign front_element_mbuf = master_buffer[front_pointer_mbuf];
    assign device_data_out = block_mux;
    always_comb begin
    case(block_index)
        0: block_mux = front_element_mbuf[7:0];
        1: block_mux = front_element_mbuf[15:8];
        2: block_mux = front_element_mbuf[23:16];
        3: block_mux = front_element_mbuf[31:24];
        4: block_mux = front_element_mbuf[39:32];
        5: block_mux = front_element_mbuf[47:40];
        6: block_mux = front_element_mbuf[55:48];
        7: block_mux = front_element_mbuf[63:56];
        default: block_mux = 8'h00;
    endcase
    end 
    
    always @(posedge clk) begin
        if(!rst_n) begin
            front_pointer_mbuf <= 0;
            block_index <= 0;
            device_wr_ins <= 0; // Sample tp "block_mux" directly
        end
        else if(~empty_mbuf & device_wr_available) begin
            if(device_wr_ins) begin                 // Sample 
                block_index <= block_index + 1;
                front_pointer_mbuf <= (block_mux == 7) ? front_pointer_mbuf + 1 : front_pointer_mbuf;
                device_wr_ins <= 0;
            end
            else begin                              // Sending
                device_wr_ins <= 1;
            end
        end 
    end
    
    localparam WORD_BLOCK_SIZE_BYTE = (WORD_WIDTH / BYTE_WIDTH);    // 4
    localparam WORD_BLOCK_AMOUNT    = SLAVE_BUFFER_SIZE / WORD_BLOCK_SIZE_BYTE;
    wire[WORD_WIDTH - 1:0]          word_block_sbuf         [0:WORD_BLOCK_AMOUNT - 1];
    reg [BYTE_WIDTH - 1:0]          amt_word_block_sbuf     [0:WORD_BLOCK_AMOUNT - 1];
    wire                            valid_word_block_sbuf   [0:WORD_BLOCK_AMOUNT - 1];
    reg                             valid_word_block_sbuf_hi[0:WORD_BLOCK_AMOUNT - 1];
    reg                             valid_word_block_sbuf_lo[0:WORD_BLOCK_AMOUNT - 1];
    for(genvar i = 0; i < WORD_BLOCK_AMOUNT; i = i + 1) begin
    assign valid_word_block_sbuf[i] = valid_word_block_sbuf_hi[i] ^ valid_word_block_sbuf_lo[i];
    end 
    for(genvar i = 0; i < WORD_BLOCK_AMOUNT; i = i + 1) begin
        // Format of DATA_BUS in WORD_TYPE
        //  ---------------------------------------------------
        //                  |                    |
        //                  |     DON'T CARE     | // Will be preempted by sign-extend value
        //                  |                    |
        //  ---------------------------------------------------
        //                  |                    |
        //                  |DATA_WIDTH  = 3bytes|
        //                  |                    |
        //  ---------------------------------------------------
        //                  |AMOUNT_WIDTH= 1byte |
        //  ---------------------------------------------------
        assign word_block_sbuf[i][WORD_WIDTH - 1:BYTE_WIDTH] = {slave_buffer[i*WORD_BLOCK_SIZE_BYTE + 3], slave_buffer[i*WORD_BLOCK_SIZE_BYTE + 2], slave_buffer[i*WORD_BLOCK_SIZE_BYTE + 1]};
        assign word_block_sbuf[i][BYTE_WIDTH - 1:0] = amt_word_block_sbuf[i];
    end 
    
    localparam DOUBLEWORD_BLOCK_SIZE_WORD   = (DOUBLEWORD_WIDTH / WORD_WIDTH);
    localparam DOUBLEWORD_BLOCK_AMOUNT      = WORD_BLOCK_AMOUNT / DOUBLEWORD_BLOCK_SIZE_WORD;
    wire[DOUBLEWORD_WIDTH - 1:0]    dword_block_sbuf        [0:DOUBLEWORD_BLOCK_AMOUNT - 1];
    wire[BYTE_WIDTH - 1:0]          amt_dword_block_sbuf    [0:DOUBLEWORD_BLOCK_AMOUNT - 1];
    wire                            valid_dword_block_sbuf  [0:DOUBLEWORD_BLOCK_AMOUNT - 1];
    for(genvar i = 0; i < DOUBLEWORD_BLOCK_AMOUNT; i = i + 1) begin
        // Format of DATA_BUS in DOUBLE_WORD_TYPE:
        //  ---------------------------------------------------
        //                  |                    |
        //                  |                    |
        //                  |                    |
        //                  |DATA_WIDTH  = 6bytes|
        //                  |                    |
        //                  |                    |
        //                  |                    |
        //  ---------------------------------------------------
        //                  |AMOUNT_WIDTH= 2byte |
        //  ---------------------------------------------------
        assign dword_block_sbuf[i][DOUBLEWORD_WIDTH - 1:BYTE_WIDTH*DOUBLEWORD_BLOCK_SIZE_WORD] = {word_block_sbuf[i*DOUBLEWORD_BLOCK_SIZE_WORD + 1][WORD_WIDTH - 1:BYTE_WIDTH],
                                                                                                  word_block_sbuf[i*DOUBLEWORD_BLOCK_SIZE_WORD + 0][WORD_WIDTH - 1:BYTE_WIDTH]};
        assign dword_block_sbuf[i][BYTE_WIDTH*DOUBLEWORD_BLOCK_SIZE_WORD - 1:0] = amt_word_block_sbuf[i*DOUBLEWORD_BLOCK_SIZE_WORD + 1] + amt_word_block_sbuf[i*DOUBLEWORD_BLOCK_SIZE_WORD + 0];
        assign valid_dword_block_sbuf[i] = valid_word_block_sbuf[i*DOUBLEWORD_BLOCK_SIZE_WORD + 1] & valid_word_block_sbuf[i*DOUBLEWORD_BLOCK_SIZE_WORD + 0]; 
    end 
    
    reg     word_block_timeout_en;
    wire    word_block_timeout_flag;
    real_time 
        #(
        .MAX_COUNTER(PACKET_TIMEOUT)
        ) timeout_packet (
        .clk(clk),
        .counter_enable(word_block_timeout_en),
        .limit_counter(PACKET_TIMEOUT),
        .limit_flag(word_block_timeout_flag),
        .rst_n(rst_n) 
        );
    reg [1:0] rd_device_state;
    localparam OUT_WORD_BLOCK_STATE = 0;    
    localparam IN_WORD_BLOCK_STATE  = 1;   
    
    localparam WORD_BLOCK_INDEX_W = $clog2(WORD_BLOCK_AMOUNT);
    wire [WORD_BLOCK_INDEX_W - 1:0]                     index_word_block_wr_cur;
    wire [WORD_BLOCK_INDEX_W - 1:0]                     index_word_block_wr_next;
    wire                                                full_word_block_wr;
    wire [WORD_BLOCK_INDEX_W - 1:0]                     index_word_block_rd_cur;
    wire [WORD_BLOCK_INDEX_W - 1:0]                     index_word_block_rd_next;
    wire [POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W - 1:0] offset_word_block;
    // Buffer Format
    //
    //      Block Contain           |
    //            1 or more data    |                   |        |              |                |
    // |----------------|-----------|-------------------|--------|--------------|----------------|
    //                  |           |       DATA        |   #3   |              |                |
    //                  |           |-------------------|        |              |                |
    //                  |           |       DATA        |   #2   | WORD         |                |
    //                  |   VALID   |-------------------|        |   BLOCK #1   |                |
    //                  |WORD_BLOCK |       DATA        |   #1   |              |                |  <- offset_word_block
    //      VALID       |           |-------------------|        |              |                |
    //    DOUBLE_WORD   |           |      RESERVED     |   #0   |              | DOUBLE_WORD    |//  Replaced by AMOUNT value in DATA_BUS 
    //      BLOCK       |-----------|-------------------|--------|--------------|       BLOCK #0 |
    //                  |           |       DATA        |   #3   |              |                |
    //                  |           |-------------------|        |              |                |
    //                  |           |       DATA        |   #2   | WORD         |                |
    //                  |   VALID   |-------------------|        |   BLOCK #0   |                |
    //                  |WORD_BLOCK |       DATA        |   #1   |              |                |  <- offset_word_block
    //                  |           |-------------------|        |              |                |
    //                  |           |      RESERVED     |   #0   |              |                |//  Replaced by AMOUNT value in DATA_BUS
    // |----------------|-----------|-------------------|--------|--------------|----------------|
    //          
    //  
    //
    assign index_word_block_wr_cur  = rear_pointer_sbuf[POINTER_SLAVE_BUF - 1: POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W];
    assign index_word_block_wr_next = rear_pointer_sbuf[POINTER_SLAVE_BUF - 1: POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W] + 1;
    assign full_word_block_wr = (rear_pointer_sbuf[POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W - 1: 0] == {(POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W){1'b0}});
    assign index_word_block_rd_cur  = front_pointer_sbuf[POINTER_SLAVE_BUF - 1: POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W];
    assign index_word_block_rd_next  = front_pointer_sbuf[POINTER_SLAVE_BUF - 1: POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W] + 1;
    assign offset_word_block = {{(POINTER_SLAVE_BUF - WORD_BLOCK_INDEX_W - 1){1'b0}}, 1'b1};
    assign full_sbuf = rear_pointer_sbuf == front_pointer_sbuf;
    always @(posedge clk) begin
        if(!rst_n) begin
            rear_pointer_sbuf <= 1;
            device_rd_ins <= 1;
            word_block_timeout_en <= 0;
            valid_word_block_sbuf_hi[0:WORD_BLOCK_AMOUNT - 1] <= {{WORD_BLOCK_AMOUNT{1'b0}}};
            rd_device_state <= OUT_WORD_BLOCK_STATE;
        end
        else begin
            case(rd_device_state) 
                OUT_WORD_BLOCK_STATE: begin
                    word_block_timeout_en <= 0;
                    if(device_rd_available & !full_sbuf) begin
                        rd_device_state <= IN_WORD_BLOCK_STATE;
                        valid_word_block_sbuf_hi[index_word_block_wr_cur] <= ~valid_word_block_sbuf_lo[index_word_block_wr_cur];
                    end
                end 
                IN_WORD_BLOCK_STATE: begin
                    if(device_rd_available) begin
                        if(device_rd_ins) begin
                            slave_buffer[rear_pointer_sbuf] <= device_data_out;
                            device_rd_ins <= 0;
                        end 
                        else begin
                            device_rd_ins <= 1;
                            rear_pointer_sbuf <= rear_pointer_sbuf + 1;
                        end 
                    end if(full_word_block_wr | word_block_timeout_flag) begin
                        rd_device_state <= OUT_WORD_BLOCK_STATE;
                        rear_pointer_sbuf <= (full_word_block_wr) ? {index_word_block_wr_cur, offset_word_block} : {index_word_block_wr_next, offset_word_block};
                        word_block_timeout_en <= 1;
                    end  
                    word_block_timeout_en <= (word_block_timeout_flag | full_word_block_wr) ? 0 : !device_rd_available;
                end 
                
            endcase 
        end
    end
    
    logic [POINTER_SLAVE_BUF - 1:0] front_pointer_sbuf_next;
    always @(posedge clk) begin
        if(!rst_n) begin
            front_pointer_sbuf <= 1;
            valid_word_block_sbuf_lo[0:WORD_BLOCK_AMOUNT - 1] <= {{WORD_BLOCK_AMOUNT{1'b0}}};
        end 
        else if(rd_req & buffer_rd_valid) begin
            front_pointer_sbuf <= front_pointer_sbuf_next;
                    valid_word_block_sbuf_lo[index_word_block_rd_cur] <= valid_word_block_sbuf_hi[index_word_block_rd_cur];
        end 
    end
    for(genvar i = 0; i < WORD_BLOCK_AMOUNT; i = i + 1) begin
    always @(posedge clk) begin
        if(!rst_n) begin
            valid_word_block_sbuf_lo[i] <= 1'b0;
        end
        else if(rd_req & buffer_rd_valid) begin
            if((data_type_encode == BYTE_TYPE_ENCODE | data_type_encode == WORD_TYPE_ENCODE) & index_word_block_rd_cur == i) begin
                valid_word_block_sbuf_lo[i] <= valid_word_block_sbuf_hi[i];
            end
            else if(data_type_encode == DOUBLEWORD_TYPE_ENCODE & (index_word_block_rd_next == i)) begin
                valid_word_block_sbuf_lo[i] <= valid_word_block_sbuf_hi[i];
            end
        end
    end 
    end
    always_comb begin
    case(data_type_encode)
        BYTE_TYPE_ENCODE: begin
//            data_bus_out_buf = 
            buffer_rd_valid = buffer_rd_valid_word;
            front_pointer_sbuf_next <= {index_word_block_rd_next, offset_word_block};
        end
        WORD_TYPE_ENCODE: begin
        
            buffer_rd_valid = buffer_rd_valid_word;
            front_pointer_sbuf_next <= {index_word_block_rd_next, offset_word_block};
        end
        DOUBLEWORD_TYPE_ENCODE: begin
        
            buffer_rd_valid = buffer_rd_valid_dword;
            front_pointer_sbuf_next <= {index_word_block_rd_next + 1, offset_word_block};
        end
        default: begin
        
            buffer_rd_valid = 1'b0;
            front_pointer_sbuf_next <= {index_word_block_rd_next, offset_word_block};
        end
    endcase
    buffer_rd_valid_word    = valid_word_block_sbuf[index_word_block_rd_cur];
    buffer_rd_valid_dword   = buffer_rd_valid_word & valid_word_block_sbuf[index_word_block_rd_next];
    end 
endmodule

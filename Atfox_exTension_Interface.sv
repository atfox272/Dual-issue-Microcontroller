// ATI Bus Structure (Interface for slave)
// Address line format
// <CHANNEL_ID> - <ADDRESS_ID>
module Atfox_exTensible_Interface
    #(
    // SYSTEM BUS parameter
    parameter DATA_BUS_WIDTH            = 64,
    parameter ADDR_BUS_WIDTH            = 64,
    parameter CHANNEL_WIDTH             = 2,
    parameter DATA_TYPE_WIDTH           = 2,    // DoubleWord - Word - Byte (-> 3 type)
    parameter DEVICE_CHANNEL_ID         = 2'h00,
    // Device parameter 
    parameter DEVICE_DATA_WIDTH         = 8,
    
    // DEEP CONFIGURATION
    parameter MASTER_BUFFER_SIZE        = 4,
    parameter SLAVE_BUFFER_SIZE         = 64,
    
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
    inout                               buffer_available,
    input                               rd_req,
    input                               wr_req,
    input  [DATA_TYPE_WIDTH - 1:0]      data_type_encode,      // Doubleword - Word - Byte
    // DEVICE interface ////////////////////////
    input   [DEVICE_DATA_WIDTH - 1:0]   device_data_in,
    output  [DEVICE_DATA_WIDTH - 1:0]   device_data_out,
    output  reg                         device_wr_ins,
    output  reg                         device_rd_ins,
    input                               device_idle,        // Connect to DEVICE_IDLE port
    input                               device_available,   // Buffer of device is available (not full)
    input                               rst_n
    );
    
    reg [DATA_BUS_WIDTH - 1:0]          master_buffer [0:MASTER_BUFFER_SIZE - 1];
    reg [DATA_TYPE_WIDTH - 1:0]         data_type_mbuf[0:MASTER_BUFFER_SIZE - 1];   // data_type of element in master_buffer
    reg [POINTER_MASTER_BUF - 1:0]      front_pointer_mbuf;
    reg [POINTER_MASTER_BUF - 1:0]      rear_pointer_mbuf;
    
    reg [DEVICE_DATA_WIDTH - 1:0]       slave_buffer  [0:SLAVE_BUFFER_SIZE - 1];
    reg [DATA_TYPE_WIDTH - 1:0]         data_type_sbuf[0:SLAVE_BUFFER_SIZE - 1];    // data_type of element in slave_buffer
    reg [POINTER_SLAVE_BUF - 1:0]       front_pointer_sbuf;
    reg [POINTER_SLAVE_BUF - 1:0]       rear_pointer_sbuf;
    
    reg [DATA_BUS_WIDTH - 1:0]          data_bus_out_buf;
    wire                                wr_req_decode;
    wire                                rd_req_decode;
    wire                                full_mbuf;
    wire                                empty_mbuf;
    
    // Address decoder   
    assign address_decoder  = (addr_bus_in[ADDR_BUS_WIDTH - 1: ADDR_BUS_WIDTH - CHANNEL_WIDTH] == DEVICE_CHANNEL_ID);
    assign data_bus_out     = (address_decoder) ? data_bus_out_buf  : {64{1'hz}};
    assign device_available = (address_decoder) ? ~full_mbuf        : {1{1'hz}};
    assign wr_req_decode    = (address_decoder) ? wr_req : 1'b0;
    assign rd_req_decode    = (address_decoder) ? rd_req : 1'b0;
    
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
        else if(~empty_mbuf & device_available) begin
            if(device_wr_ins) begin
                block_index <= block_index + 1;
                front_pointer_mbuf <= (block_mux == 7) ? front_pointer_mbuf + 1 : front_pointer_mbuf;
                device_wr_ins <= 0;
            end
            else begin
                device_wr_ins <= 1;
            end
        end 
    end 
endmodule

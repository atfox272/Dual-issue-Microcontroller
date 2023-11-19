// Requirement: DEPTH of FIFO must be a power of 2
//              Or you can use limit-fifo option (set parameter LIMIT_FIFO_FLAG = 1 and load limit value to limit_fifo),
//              Notice: Limit value must be less than Depth of FIFO
module fifo_module
    #(
        parameter DEPTH             = 32,
        parameter WIDTH             = 8,
        parameter SLEEP_MODE        = 0,
        parameter LIMIT_COUNTER     = DEPTH,
        // Do not configuration below parameter 
        parameter COUNTER_WIDTH    = $clog2(DEPTH + 1),
        parameter DEPTH_ALIGN      = DEPTH + 1
     )(
    // Data
    input   wire    [WIDTH - 1:0]   data_bus_in,
    output  wire    [WIDTH - 1:0]   data_bus_out,
    // Instruction
    input   wire                    write_ins,
    input   wire                    read_ins,
    // State 
    output  wire                    full,
    output  wire                    almost_full,
    output  wire                    empty,
    output  wire                    almost_empty,
    // Option feature
    output  wire                    reach_limit,
    // Enable pin
    input   wire                    enable,
    // Reset 
    input   wire                    rst_n
    
    // DEBUG
//    ,output  wire [COUNTER_WIDTH - 1:0] front_queue_wire
//    ,output  wire [COUNTER_WIDTH - 1:0] rear_queue_wire
//    ,output  wire [COUNTER_WIDTH - 2:0] counter_elem_wire
    );
    
    localparam init_buffer = 8'h00;
    localparam init_index_front = 0;
    localparam init_index_rear = 0;
    
    reg [COUNTER_WIDTH - 1:0] front_queue;
    reg [COUNTER_WIDTH - 1:0] rear_queue;
    wire[COUNTER_WIDTH - 2:0] counter_elem;
    
    reg [WIDTH - 1:0] queue [0: DEPTH_ALIGN - 1];
    wire read_ins_sleepmode;
    wire write_ins_sleepmode;
    
    
	generate
        if(SLEEP_MODE) begin
            assign read_ins_sleepmode = (enable) ? read_ins : 1'b0;
            assign write_ins_sleepmode = (enable) ? write_ins : 1'b0;
        end
        else begin
            assign read_ins_sleepmode = read_ins;
            assign write_ins_sleepmode = write_ins;
        end
	endgenerate
    //Data 
    assign data_bus_out = queue[front_queue];
    // State 
    assign counter_elem = rear_queue - front_queue;
    
    assign full = (rear_queue + 1 == front_queue) | 
                  ((rear_queue == DEPTH_ALIGN - 1) & (front_queue == 0));
    assign almost_full = (rear_queue + 2 == front_queue) |
                         ((rear_queue == DEPTH_ALIGN - 2) & (front_queue == 0)) |
                         ((rear_queue == DEPTH_ALIGN - 1) & (front_queue == 1));
    
    assign empty = (rear_queue == front_queue);
    assign almost_empty = (rear_queue == front_queue + 1) | 
                          ((rear_queue == 0) & (front_queue == DEPTH_ALIGN - 1));
                          
    assign reach_limit = counter_elem > (LIMIT_COUNTER - 1);
    // Write instruction
    always @(posedge write_ins_sleepmode, negedge rst_n) begin
        if(!rst_n) begin
            rear_queue <= init_index_rear;
    //            for(i = 0; i < capacity; i = i + 1) begin
    //                queue[i] <= init_buffer;
    //            end
    //            queue[capacity - 1:0] <= 0;
        end
        else begin
            if(!full) begin
                queue[rear_queue] <= data_bus_in;
                rear_queue <= (rear_queue == DEPTH_ALIGN - 1) ? 0 : rear_queue + 1;
            end
        end
    end 
    // Read instruction
    always @(posedge read_ins_sleepmode, negedge rst_n) begin
        if(!rst_n) begin
            front_queue <= init_index_front;
        end
        else begin
            if(!empty) begin
                front_queue <= (front_queue == DEPTH_ALIGN - 1) ? 0 : front_queue + 1;
            end
        end 
    end
    
    // Debug
//    assign front_queue_wire = front_queue;
//    assign rear_queue_wire = rear_queue;
//    assign counter_elem_wire = counter_elem;
endmodule
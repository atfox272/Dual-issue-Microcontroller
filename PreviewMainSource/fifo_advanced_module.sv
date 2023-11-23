module fifo_advanced_module
    #(
    parameter DATA_BIG_BUFFER       = 4,
    parameter DATA_SND_SMALL_WIDTH  = 8,
    parameter DATA_SND_BIG_WIDTH    = 192,
    parameter DATA_RCV_SMALL_WIDTH  = 8,
    parameter DATA_RCV_BIG_WIDTH    = 128,  // 64bit * 3
    parameter AMOUNT_BYTE_SND_WIDTH = $clog2(DATA_SND_BIG_WIDTH / 8),
    parameter AMOUNT_BYTE_RCV_WIDTH = $clog2(DATA_RCV_BIG_WIDTH / 8),
    // Finish receiving
    parameter FINISH_RECEIVE_TIMER  = 500000,
    // Do not change this 
    parameter POINTER_WIDTH         = $clog2(DATA_BIG_BUFFER),
    parameter POINTER_ELEM_WIDTH    = $clog2(DATA_RCV_BIG_WIDTH)
    )
    (
    input   wire    clk,
    
    // Send handler (synchronous)
    // Connect to Processor 
    input   wire                                    snd_big_clk,
    input   wire    [AMOUNT_BYTE_SND_WIDTH - 1:0]   amount_byte_snd_big,
    input   wire    [DATA_SND_BIG_WIDTH - 1:0]      data_snd_big,
    output  wire                                    available_snd,
    // Connect to UART
    output  wire    [DATA_SND_SMALL_WIDTH - 1:0]    data_snd_small,
    output  wire                                    snd_small_clk,
    input   wire                                    snd_small_available,
    // Connect to Processor
    input   wire                                    rcv_big_clk,
    output  wire    [AMOUNT_BYTE_RCV_WIDTH - 1:0]   amount_byte_rcv_big,
    output  wire    [DATA_RCV_BIG_WIDTH - 1:0]      data_rcv_big,
    output  wire                                    available_rcv,
    // Connect to UART_RX (use internal FIFO)
    input   wire    [DATA_RCV_SMALL_WIDTH - 1:0]    data_rcv_small,
    output  wire                                    rcv_small_clk,          // Read clcok
    input   wire                                    rcv_small_available,    // Connect to RX_flag (RX is not empty)
    
    output  wire    empty_snd,
    output  wire    full_snd,
    
    output  wire    empty_rcv,
    output  wire    full_rcv,
    
    input   wire    rst_n
    );
    reg [DATA_SND_BIG_WIDTH - 1:0]      queue_snd [0: DATA_BIG_BUFFER];
    reg [AMOUNT_BYTE_SND_WIDTH - 1:0]   amount_byte_snd [0: DATA_BIG_BUFFER];
    reg [POINTER_WIDTH - 1:0]           front_snd;
    reg [POINTER_WIDTH - 1:0]           rear_snd;
    reg [DATA_SND_SMALL_WIDTH - 1:0]    data_snd_small_reg;
    reg [AMOUNT_BYTE_SND_WIDTH - 1:0]   index_elem_snd;
    reg                                 snd_small_clk_reg;
    reg [1:0]                           fetcher_state;
    
    localparam IDLE_STATE = 0;
    localparam FETCHING_STATE = 1;
    
    assign available_snd = ~full_snd;
    assign full_snd = (rear_snd + 1 == front_snd) | 
                      ((rear_snd == DATA_BIG_BUFFER - 1) & (front_snd == 0));
    assign empty_snd = rear_snd == front_snd;
    
    assign data_snd_small = data_snd_small_reg;
    assign snd_small_clk = snd_small_clk_reg;            
                
    always @* begin
        case(index_elem_snd)
            0: begin
                data_snd_small_reg <= queue_snd[front_snd][7:0];
            end
            1: begin
                data_snd_small_reg <= queue_snd[front_snd][15:8];
            end
            2: begin
                data_snd_small_reg <= queue_snd[front_snd][23:16];
            end
            3: begin
                data_snd_small_reg <= queue_snd[front_snd][31:24];
            end
            4: begin
                data_snd_small_reg <= queue_snd[front_snd][39:32];
            end
            5: begin
                data_snd_small_reg <= queue_snd[front_snd][47:40];
            end
            6: begin
                data_snd_small_reg <= queue_snd[front_snd][55:48];
            end
            7: begin
                data_snd_small_reg <= queue_snd[front_snd][63:56];
            end
            8: begin
                data_snd_small_reg <= queue_snd[front_snd][71:64];
            end
            9: begin
                data_snd_small_reg <= queue_snd[front_snd][79:72];
            end
            10: begin
                data_snd_small_reg <= queue_snd[front_snd][87:80];
            end
            11: begin
                data_snd_small_reg <= queue_snd[front_snd][95:88];
            end
            12: begin
                data_snd_small_reg <= queue_snd[front_snd][103:96];
            end
            13: begin
                data_snd_small_reg <= queue_snd[front_snd][111:104];
            end
            14: begin
                data_snd_small_reg <= queue_snd[front_snd][119:112];
            end
            15: begin
                data_snd_small_reg <= queue_snd[front_snd][127:120];
            end
            16: begin
                data_snd_small_reg <= queue_snd[front_snd][135:128];
            end
            17: begin
                data_snd_small_reg <= queue_snd[front_snd][143:136];
            end
            18: begin
                data_snd_small_reg <= queue_snd[front_snd][151:144];
            end
            19: begin
                data_snd_small_reg <= queue_snd[front_snd][159:152];
            end
            20: begin
                data_snd_small_reg <= queue_snd[front_snd][167:160];
            end
            21: begin
                data_snd_small_reg <= queue_snd[front_snd][175:168];
            end
            22: begin
                data_snd_small_reg <= queue_snd[front_snd][183:176];
            end
            23: begin
                data_snd_small_reg <= queue_snd[front_snd][191:184];
            end
            default: begin
                data_snd_small_reg <= queue_snd[front_snd][7:0];
            end
            
        endcase
    end            
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            rear_snd <= 0;
        end
        else begin
            if(snd_big_clk) begin
                if(!full_snd) begin
                    queue_snd[rear_snd] <= data_snd_big;
                    amount_byte_snd[rear_snd] <= amount_byte_snd_big;
                    rear_snd <= rear_snd + 1;
                end
            end
        end
    end
    // Fetcher
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            front_snd <= 0;
            index_elem_snd <= 0;
            snd_small_clk_reg <= 0;
            fetcher_state <= IDLE_STATE;
        end
        else begin
            case(fetcher_state) 
                IDLE_STATE: begin
                    if(!empty_snd) begin
                        fetcher_state <= FETCHING_STATE;
                        snd_small_clk_reg <= 1;
                    end
                end
                FETCHING_STATE: begin
                    if(snd_small_clk_reg) begin     // FALLING - sample buffer
                        snd_small_clk_reg <= 0;
                        if(index_elem_snd + 1 == amount_byte_snd[front_snd]) begin
                            fetcher_state <= IDLE_STATE;
                            front_snd <= front_snd + 1;
                            // Reset state 
                            index_elem_snd <= 0;
                        end
                        else begin
                            index_elem_snd <= index_elem_snd + 1;
                        end
                    end
                    else begin                      // RISING - send data
                        snd_small_clk_reg <= (snd_small_available) ? 1 : snd_small_clk_reg;
                    end
                end
            endcase
        end
    end
    
    
    // change packet 
    //      ~ waiting for count-down LOW state of "available" signal RX module (100ms)
    //      ~ overflow packet size (192 bytes)
    reg [DATA_RCV_BIG_WIDTH - 1:0]      queue_rcv [0: DATA_BIG_BUFFER];
    reg [AMOUNT_BYTE_RCV_WIDTH - 1:0]   amount_byte_rcv [0: DATA_BIG_BUFFER];
    reg [POINTER_WIDTH - 1:0]           front_rcv;
    reg [POINTER_WIDTH - 1:0]           rear_rcv;
    reg [DATA_RCV_SMALL_WIDTH - 1:0]    data_rcv_small_reg;
    reg [AMOUNT_BYTE_RCV_WIDTH - 1:0]   index_elem_rcv;
    reg                                 rcv_small_clk_reg;
    reg [1:0]                           receiver_state;
    wire                                stop_receiving_condition;
    localparam RECEIVING_STATE = 1;
    
    assign available_rcv = !empty_rcv;
    assign rcv_small_clk = rcv_small_clk_reg;
    assign full_rcv = (rear_rcv + 1 == front_rcv) | 
                      ((rear_rcv == DATA_BIG_BUFFER - 1) & (front_rcv == 0));
    assign empty_rcv = rear_rcv == front_rcv;
    assign data_rcv_big = queue_rcv[front_rcv];
    assign amount_byte_rcv_big = amount_byte_rcv[front_rcv];
    // Timer for finishing program (time out)
    waiting_module #(
                    .END_COUNTER(FINISH_RECEIVE_TIMER),
                    .WAITING_TYPE(0),
                    .LEVEL_PULSE(0)
                    )timout_programming(
                    .clk(clk),
                    .start_counting(rcv_small_available),
                    .reach_limit(stop_receiving_condition),
                    .rst_n(rst_n)
                    );  
    
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            receiver_state <= IDLE_STATE;
            rcv_small_clk_reg <= 0;
            rear_rcv <= 0;
            index_elem_rcv <= 0;
        end
        else begin
            case(receiver_state) 
                IDLE_STATE: begin
                    if(rcv_small_available & !full_rcv) begin
                        receiver_state <= RECEIVING_STATE;
                    end
                end
                RECEIVING_STATE: begin
                    if(!rcv_small_available) begin
                        if(stop_receiving_condition) begin  // Timeout 
                            receiver_state <= IDLE_STATE;
                            rear_rcv <= rear_rcv + 1;
                            amount_byte_rcv[rear_rcv] <= index_elem_rcv;
                            index_elem_rcv <= 0;
                            rcv_small_clk_reg <= 0;
                        end
                    end
                    else begin
                        if(rcv_small_clk_reg) begin
                            rcv_small_clk_reg <= 0; 
                        end
                        else begin
                            if(index_elem_rcv == (DATA_RCV_BIG_WIDTH / 8)) begin
                                rear_rcv <= rear_rcv + 1;
                                amount_byte_rcv[rear_rcv] <= index_elem_rcv;
                                index_elem_rcv <= 0;
                                receiver_state <= IDLE_STATE;   // Check (if full == 1) then PAUSE
                            end
                            else begin
                                index_elem_rcv <= index_elem_rcv + 1;
                                case(index_elem_rcv)
                                    0: begin
                                        queue_rcv[rear_rcv][7:0]<= data_rcv_small;
                                    end
                                    1: begin
                                        queue_rcv[rear_rcv][15:8]<= data_rcv_small;
                                    end
                                    2: begin
                                        queue_rcv[rear_rcv][23:16]<= data_rcv_small;
                                    end
                                    3: begin
                                        queue_rcv[rear_rcv][31:24]<= data_rcv_small;
                                    end
                                    4: begin
                                        queue_rcv[rear_rcv][39:32]<= data_rcv_small;
                                    end
                                    5: begin
                                        queue_rcv[rear_rcv][47:40]<= data_rcv_small;
                                    end
                                    6: begin
                                        queue_rcv[rear_rcv][55:48]<= data_rcv_small;
                                    end
                                    7: begin
                                        queue_rcv[rear_rcv][63:56]<= data_rcv_small;
                                    end
                                    8: begin
                                        queue_rcv[rear_rcv][71:64]<= data_rcv_small;
                                    end
                                    9: begin
                                        queue_rcv[rear_rcv][79:72]<= data_rcv_small;
                                    end
                                    10: begin
                                        queue_rcv[rear_rcv][87:80]<= data_rcv_small;
                                    end
                                    11: begin
                                        queue_rcv[rear_rcv][95:88]<= data_rcv_small;
                                    end
                                    12: begin
                                        queue_rcv[rear_rcv][103:96]<= data_rcv_small;
                                    end
                                    13: begin
                                        queue_rcv[rear_rcv][111:104]<= data_rcv_small;
                                    end
                                    14: begin
                                        queue_rcv[rear_rcv][119:112]<= data_rcv_small;
                                    end
                                    15: begin
                                        queue_rcv[rear_rcv][127:120]<= data_rcv_small;
                                        
                                        rear_rcv <= rear_rcv + 1;
                                        amount_byte_rcv[rear_rcv] <= index_elem_rcv;
                                        receiver_state <= IDLE_STATE;   // Check (if full == 1) then PAUSE
                                    end
                                    default: begin
                                        queue_rcv[rear_rcv][7:0]<= data_rcv_small;
                                    end
                                endcase
                            end
                            rcv_small_clk_reg <= 1;
                        end
                    end
                end
                default: begin
                    
                end
            endcase 
        end
    end
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) front_rcv <= 0;
        else begin
            if(rcv_big_clk) begin
                if(!empty_rcv) front_rcv <= front_rcv + 1;
            end
        end
    end
endmodule

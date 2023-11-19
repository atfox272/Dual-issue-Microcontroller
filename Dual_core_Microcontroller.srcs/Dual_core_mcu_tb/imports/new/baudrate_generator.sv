//  ----------------TX_module------------------------------
//  clk             _____|------|______|------|______|------|_... __|------|______|------|
//  transaction_en  ______/----------- -----------------------... --------------------------
//  baudrate_clk_en __________________ /--------------\_______... __/--------------\______
//                                          ^
//  Note:                           (counter is overflow)
//  ----------------RX_module------------------------------
//  clk             _____|------|______|------|______|------|_... __|------|______|------|
//  transaction_en  ______/----------- -----------------------... --------------------------
//  baudrate_clk_en __________________________________________... __/--------------\______
//                                                                  ^
//  Note:                                                   (1/2 cycle baudrate)
//
//
module baudrate_generator 
    #(
    parameter INTERNAL_CLOCK        = 125000000,
    
    parameter GENERATE_BD_4TX       = 1,
    
    parameter BAUDRATE_AMOUNT       = 8,
    parameter BAUDRATE_4800_ENC     = 0,
    parameter BAUDRATE_9600_ENC     = 1,
    parameter BAUDRATE_19200_ENC    = 2,
    parameter BAUDRATE_38400_ENC    = 3,
    parameter BAUDRATE_14400_ENC    = 4,
    parameter BAUDRATE_28800_ENC    = 5,
    parameter BAUDRATE_57600_ENC    = 6,
    parameter BAUDRATE_115200_ENC   = 7,
    parameter BAUDRATE_AMOUNT_W     = $clog2(BAUDRATE_AMOUNT),
    
    parameter BAUDRATE_DIV_1        = 4800,
    parameter int COUNTER_DIV_MAX = $ceil(INTERNAL_CLOCK / BAUDRATE_DIV_1),
    parameter COUNTER_DIV_W       = $clog2(COUNTER_DIV_MAX)
    
    )
    (
    input clk,
    
    input                           transaction_en,
    input [BAUDRATE_AMOUNT_W - 1:0] baudrate_mux,
    output reg                      baudrate_clk_en,
    
    input rst_n
    );
    
    
    localparam BD4800               = 4800;
    localparam BD9600               = 9600;
    localparam BD19200              = 19200;
    localparam BD38400              = 38400;
    localparam BD14400              = 14400;
    localparam BD28800              = 28800;
    localparam BD57600              = 57600;
    localparam BD115200             = 115200;
    localparam int COUNTER_BD4800   = $ceil(INTERNAL_CLOCK / BD4800);
    localparam int COUNTER_BD9600   = $ceil(INTERNAL_CLOCK / BD9600);
    localparam int COUNTER_BD19200  = $ceil(INTERNAL_CLOCK / BD19200);
    localparam int COUNTER_BD38400  = $ceil(INTERNAL_CLOCK / BD38400);
    localparam int COUNTER_BD14400  = $ceil(INTERNAL_CLOCK / BD14400);
    localparam int COUNTER_BD28800  = $ceil(INTERNAL_CLOCK / BD28800);
    localparam int COUNTER_BD57600  = $ceil(INTERNAL_CLOCK / BD57600);
    localparam int COUNTER_BD115200 = $ceil(INTERNAL_CLOCK / BD115200);
    
    reg  [COUNTER_DIV_W - 1:0]  counter_div;
    logic[COUNTER_DIV_W - 1:0]  counter_div_load;
    logic[COUNTER_DIV_W - 1:0]  counter_div_decr;
    
    assign counter_div_decr = counter_div - 1;
    always_comb begin
        case(baudrate_mux) 
            BAUDRATE_4800_ENC: begin
                counter_div_load <= COUNTER_BD4800 - 1;
            end
            BAUDRATE_9600_ENC: begin
                counter_div_load <= COUNTER_BD9600 - 1;
            end
            BAUDRATE_19200_ENC: begin
                counter_div_load <= COUNTER_BD19200 - 1;
            end
            BAUDRATE_38400_ENC: begin
                counter_div_load <= COUNTER_BD38400 - 1;
            end
            BAUDRATE_14400_ENC: begin
                counter_div_load <= COUNTER_BD14400 - 1;
            end
            BAUDRATE_28800_ENC: begin
                counter_div_load <= COUNTER_BD28800 - 1;
            end
            BAUDRATE_57600_ENC: begin
                counter_div_load <= COUNTER_BD57600 - 1;
            end
            BAUDRATE_115200_ENC: begin
                counter_div_load <= COUNTER_BD115200 - 1;
            end
        endcase
    end 
    if(GENERATE_BD_4TX) begin       : BLOCK_FOR_TX
    always @(posedge clk) begin
        if(!rst_n) begin
            baudrate_clk_en <= 0;
            counter_div <= 0;
        end 
        else begin
            if(transaction_en) begin
                if(counter_div == {COUNTER_DIV_W{1'b1}}) begin // Overflow
                    baudrate_clk_en <= 1;
                    counter_div <= counter_div_load;
                end 
                else begin
                    baudrate_clk_en <= 0;
                    counter_div <= counter_div_decr;
                end 
            end 
            else begin
                counter_div <= {COUNTER_DIV_W{1'b1}};       // overflow immediately when transaction enable
                baudrate_clk_en <= 0;
            end
        end 
    end 
    end 
    else begin                      : BLOCK_FOR_RX
    always @(posedge clk) begin
        if(!rst_n) begin
            baudrate_clk_en <= 0;
            counter_div <= 0;
        end 
        else begin
            if(transaction_en) begin
                if(counter_div == {COUNTER_DIV_W{1'b1}}) begin // Overflow
                    baudrate_clk_en <= 1;
                    counter_div <= counter_div_load;
                end 
                else begin
                    baudrate_clk_en <= 0;
                    counter_div <= counter_div_decr;
                end 
            end 
            else begin
                counter_div <= (counter_div_load >> 1);  // Load 1/2 cycle of baudrate at start bit (sample stable data)
                baudrate_clk_en <= 0;
            end
        end 
    end 
    end
endmodule    
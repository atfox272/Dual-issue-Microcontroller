// Clock source = internal clock / 2; (Arty-Z7 = 62.5Mhz) (DE10 = 25Mhz)
module timer_INT_handler
    #(
    parameter REGISTER_WIDTH    = 8,
    parameter PRESCALER_WIDTH   = 3,
    parameter PRESCALER_MAX     = 256,
    parameter PRESCALER_0       = 1,
    parameter PRESCALER_1       = 8,
    parameter PRESCALER_2       = 64,
    parameter PRESCALER_3       = 256,
    parameter PRESCALER_1_WIDTH = $clog2(PRESCALER_1),
    parameter PRESCALER_2_WIDTH = $clog2(PRESCALER_2),
    parameter PRESCALER_3_WIDTH = $clog2(PRESCALER_3),
    parameter MAX_LIMIT_VALUE   = 2**16,    // 65536
    parameter TIMER_INT_OVERFLOW= 0,
    parameter TIMER_INT_COMPARE = 1,
    parameter PRESCALER_COUNTER = $clog2(PRESCALER_MAX)
    )
    (
    input   wire                            clk,
    
    // Configuration register 
    input   wire                            enable_interrupt,
    input   wire                            interrupt_option,       // Value of register or Overflow
    input   wire [PRESCALER_WIDTH - 1:0]    prescaler_selector,
    input   wire [REGISTER_WIDTH*2 - 1:0]   timer_limit_value,
    
    output  wire                            interrupt_request,
    
    input   wire                            rst_n
    );
    
    reg interrupt_request_reg;
    reg [PRESCALER_COUNTER - 1:0] prescaler_counter;
    reg [REGISTER_WIDTH*2 - 1:0]  timer_counter;
    wire clk_en_prescaler_0;
    wire clk_en_prescaler_1;
    wire clk_en_prescaler_2;
    wire clk_en_prescaler_3;
    reg  clk_en_sync;
    wire clk_en_int;
    reg [REGISTER_WIDTH*2 - 1:0]  interrupt_value;
    
    assign interrupt_request = interrupt_request_reg;
    assign clk_en_prescaler_0 = 1'b1;   // clock source
    assign clk_en_prescaler_1 = (prescaler_counter[PRESCALER_1_WIDTH - 1:0] == PRESCALER_1 - 1);
    assign clk_en_prescaler_2 = (prescaler_counter[PRESCALER_2_WIDTH - 1:0] == PRESCALER_2 - 1);
    assign clk_en_prescaler_3 = (prescaler_counter[PRESCALER_3_WIDTH - 1:0] == PRESCALER_3 - 1);
    assign clk_en_int = (enable_interrupt) ? clk_en_sync : 1'b0;
    always @* begin
        case(prescaler_selector) 
            0: begin
                clk_en_sync <= clk_en_prescaler_0;
            end
            1: begin
                clk_en_sync <= clk_en_prescaler_1;
            end
            2: begin
                clk_en_sync <= clk_en_prescaler_2;
            end
            3: begin
                clk_en_sync <= clk_en_prescaler_3;
            end
            default: begin
                clk_en_sync <= clk_en_prescaler_0;         
            end
        endcase 
    end
    always @* begin
        case(interrupt_option)
            TIMER_INT_OVERFLOW: begin
                interrupt_value <= MAX_LIMIT_VALUE - 1;
            end
            TIMER_INT_COMPARE: begin
                interrupt_value <= timer_limit_value - 1;
            end 
            default: begin
                interrupt_value <= MAX_LIMIT_VALUE - 1;
            end
        endcase 
    end
    // Clock source
    wire [PRESCALER_COUNTER - 1:0] prescaler_counter_next;
    assign prescaler_counter_next = prescaler_counter + 1'b1;
    always @(posedge clk) begin
        if(!rst_n) begin
            prescaler_counter <= 0;
        end
        else if(enable_interrupt & prescaler_selector != 0) begin
            prescaler_counter <= prescaler_counter_next;
        end
    end
    logic [REGISTER_WIDTH*2 - 1:0]  timer_counter_next;
    logic                           interrupt_request_next;
    always_comb begin
        timer_counter_next = timer_counter + 1;
        interrupt_request_next = 0;
        if(timer_counter == interrupt_value) begin
            timer_counter_next = 0;
            interrupt_request_next = 1;
        end
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            timer_counter <= 0;
            interrupt_request_reg <= 0;
        end
        else if(clk_en_int) begin
            timer_counter <= timer_counter_next;
            interrupt_request_reg <= interrupt_request_next;
        end
    end
endmodule

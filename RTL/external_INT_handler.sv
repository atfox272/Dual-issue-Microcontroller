module external_INT_handler
    #(
    parameter PIN_IDLE_STATE        = 1,
    parameter RISE_SENSE_CONTROL    = 2'b00,
    parameter FALL_SENSE_CONTROL    = 2'b01,
    parameter CHANGE_SENSE_CONTROL  = 2'b10,
    parameter DEBOUNCE_TIMEOUT      = 5000,
    parameter DEBOUNCE_COUNTER      = $clog2(DEBOUNCE_TIMEOUT)
    )
    (
    input   wire            clk,
    input   wire            int_pin,
    
    // Configuration register
    input   wire            enable_interrupt,
    input   wire    [1:0]   interrupt_sense_control,    // Rising - Falling - Change
    input   wire            debounce_option,     
    
    output  wire            interrupt_request,
    
    input   wire            rst_n
    );
    
    reg [1:0] state;
    reg  prev_int_pin;
    reg  prev_detect;    
    wire start_counting_debounce_time;
    wire stop_counting_debounce_signal;
    reg  interrupt_request_reg;
    
    localparam RISING_DETECT_STATE  = 2'd0;
    localparam FALLING_DETECT_STATE = 2'd2;
    localparam DEBOUNCE_STATE       = 2'd1;
    
    assign interrupt_request = interrupt_request_reg;
    assign start_counting_debounce_time = (state == DEBOUNCE_STATE);
    
    real_time
        #(
        .MAX_COUNTER(DEBOUNCE_TIMEOUT)
        )debounce(
        .clk(clk),
        .counter_enable(start_counting_debounce_time),
        .limit_counter(DEBOUNCE_TIMEOUT),
        .limit_flag(stop_counting_debounce_signal),
        .rst_n(rst_n)
        );
             
    logic [1:0] state_n;
    logic       prev_int_pin_n;
    logic       prev_detect_n;
    logic       interrupt_request_next;
    always_comb begin
        state_n = state;
        prev_int_pin_n = prev_int_pin;
        prev_detect_n = prev_detect;
        interrupt_request_next = 0;
        case(state) 
            RISING_DETECT_STATE: begin
                if(interrupt_sense_control == FALL_SENSE_CONTROL) begin
                    state_n = FALLING_DETECT_STATE;
                end
                else begin      // Rising - Change
                    if(~prev_int_pin & int_pin) begin
                        state_n = (debounce_option) ? DEBOUNCE_STATE : (interrupt_sense_control == CHANGE_SENSE_CONTROL) ? FALLING_DETECT_STATE : RISING_DETECT_STATE;
                        interrupt_request_next = 1;
                        prev_detect_n = 1;
                    end
                    prev_int_pin_n = int_pin;
                end
            end
            DEBOUNCE_STATE: begin
                if(stop_counting_debounce_signal) begin
                    case(interrupt_sense_control)
                        RISE_SENSE_CONTROL: begin
                            state_n = RISING_DETECT_STATE;
                        end
                        FALL_SENSE_CONTROL: begin
                            state_n = FALLING_DETECT_STATE;                            
                        end
                        CHANGE_SENSE_CONTROL: begin
                            state_n = (prev_detect) ? FALLING_DETECT_STATE : RISING_DETECT_STATE;
                        end
                        default: begin
                        end
                    endcase 
                end
            end
            FALLING_DETECT_STATE: begin
                if(interrupt_sense_control == RISE_SENSE_CONTROL) begin
                    state_n = RISING_DETECT_STATE;
                end
                else begin
                    if(prev_int_pin & ~int_pin) begin
                        state_n = (debounce_option) ? DEBOUNCE_STATE : (interrupt_sense_control == CHANGE_SENSE_CONTROL) ? RISING_DETECT_STATE : FALLING_DETECT_STATE;
                        interrupt_request_next = 1;
                        prev_detect_n = 0;
                    end
                    prev_int_pin_n = int_pin;
                end
            end
            default: begin
            end
        endcase 
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            state <= FALLING_DETECT_STATE;
            prev_int_pin <= PIN_IDLE_STATE;
            interrupt_request_reg <= 0;
            
        end
        else if(enable_interrupt) begin
            state <= state_n;
            prev_int_pin <= prev_int_pin_n;
            prev_detect <= prev_detect_n;
            interrupt_request_reg <= interrupt_request_next;
        end
    end 
    
endmodule

module com_uart_trans_timer
    #(  parameter CLOCK_DIVIDER =           51,     // <value> = ceil(Internal clock / (256 * 2))
        // Unique clock use for reducing error percent (increase accuracy)
        parameter CLOCK_DIVIDER_UNIQUE_1 =  542,    // <value> = ceil(Internal clock / (<BAUDRATE_SPEED> * 2))  (115200)
        parameter CLOCK_DIVIDER_UNIQUE_2 =  6511,   // <value> = ceil(Internal clock / (<BAUDRATE_SPEED> * 2))  (9600)
        parameter BD4800_ENCODE =           0,
        parameter BD9600_ENCODE =           1,
        parameter BD19200_ENCODE =          2,
        parameter BD38400_ENCODE =          3,   
        parameter BD_UNIQUE_1_ENCODE =      4,      // Unique Baudrate 
        parameter BD_UNIQUE_2_ENCODE  =     5,      // Unique Baudrate 
        
        parameter BAUDRATE_SEL_WIDTH     = $clog2(BD_UNIQUE_2_ENCODE + 1),  // Max encode value
        parameter UNIQUE_1_COUNTER_WIDTH = $clog2(CLOCK_DIVIDER_UNIQUE_1 + 1), 
        parameter UNIQUE_2_COUNTER_WIDTH = $clog2(CLOCK_DIVIDER_UNIQUE_2 + 1),  
        parameter FIRST_COUNTER_WIDTH   = $clog2(CLOCK_DIVIDER)
    )
    (
    input   wire                                clk,
    input   wire    [BAUDRATE_SEL_WIDTH - 1:0]  baudrate_sel,
    input   wire                                rst_n,
    output  wire                                baudrate_clk,
//    output [5:0] debug_40,
//    output [6:0] debug_128
    input   wire                                FIFO_empty,
    input   wire                                ctrl_idle_state,
    input   wire                                ctrl_stop_state,
    
    // Optional
    output  wire                                TX_complete
    // Debug 
//    , output wire TX_disable_wire
//    , output wire debug
//    , output [FIRST_COUNTER_WIDTH - 1:0] debug_1
    );
    // Divider 
    // Baudrate clock (prescaler)
//    localparam Bd9600 = 3'b000;
//    localparam Bd19200 = 3'b001;
//    localparam Bd38400 = 3'b010;
//    localparam Bd76800 = 3'b011;
    
    wire TX_disable = (FIFO_empty & ctrl_idle_state);
    
    reg [FIRST_COUNTER_WIDTH - 1:0] counter_div40;
    reg baudrate_div40;
    wire normal_mode_clk;
    wire normal_mode_en = ( baudrate_sel == BD4800_ENCODE   | baudrate_sel == BD9600_ENCODE |
                            baudrate_sel == BD19200_ENCODE  | baudrate_sel == BD38400_ENCODE);
    
    assign normal_mode_clk = (normal_mode_en) ? clk : 1'b0;
    
    always @(posedge normal_mode_clk, negedge rst_n) begin
        if(!rst_n) begin
            counter_div40 <= (CLOCK_DIVIDER - 1);
            baudrate_div40 <= 0; 
        end
        else begin
            if(TX_disable) begin
                counter_div40 <= (CLOCK_DIVIDER - 1);
                baudrate_div40 <= 0;
            end
            else begin
                if(counter_div40 == (CLOCK_DIVIDER - 1)) begin
                    counter_div40 <= 0;
                    baudrate_div40 <= ~baudrate_div40;
                end
                else counter_div40 <= counter_div40 + 1;
            end
        end
    end
    
    
    reg [6:0] counter_bd9600;
    reg baudrate_div40_div128;      // Bd 9600
    reg baudrate_div40_div64;       // Bd 19200
    reg baudrate_div40_div32;       // Bd 38400
    reg baudrate_div40_div16;       // Bd 76800
//    always @(posedge baudrate_div40, negedge rst_n, reg_en) begin
    always @(posedge baudrate_div40, negedge rst_n) begin
        if(!rst_n) begin
            counter_bd9600 <= 127;
            baudrate_div40_div128 <= 0;      
            baudrate_div40_div64 <= 0;       
            baudrate_div40_div32 <= 0;       
            baudrate_div40_div16 <= 0;
        end
        else begin
            if(TX_disable) begin
                counter_bd9600 <= 127;
                baudrate_div40_div128 <= 0;      
                baudrate_div40_div64 <= 0;       
                baudrate_div40_div32 <= 0;       
                baudrate_div40_div16 <= 0;
            end
            else begin
                case (baudrate_sel)
                    BD4800_ENCODE: begin
                        if(&counter_bd9600[6:0]) begin
                            counter_bd9600 <= ((baudrate_div40_div128 == 0) & (ctrl_stop_state)) ? counter_bd9600 : 0;
                            baudrate_div40_div128 <= ~baudrate_div40_div128;
                        end
                        else counter_bd9600 <= counter_bd9600 + 1;
                    end
                    BD9600_ENCODE: begin
                        if(&counter_bd9600[5:0]) begin
                            counter_bd9600 <= ((baudrate_div40_div128 == 0) & (ctrl_stop_state)) ? counter_bd9600 : 0;
                            baudrate_div40_div64 <= ~baudrate_div40_div64;
                        end
                        else counter_bd9600 <= counter_bd9600 + 1;
                    end
                    BD19200_ENCODE: begin
                        if(&counter_bd9600[4:0]) begin
                            counter_bd9600 <= ((baudrate_div40_div128 == 0) & (ctrl_stop_state)) ? counter_bd9600 : 0;
                            baudrate_div40_div32 <= ~baudrate_div40_div32;
                        end
                        else counter_bd9600 <= counter_bd9600 + 1;
                    end
                    BD38400_ENCODE: begin
                        if(&counter_bd9600[3:0]) begin
                            counter_bd9600 <= ((baudrate_div40_div128 == 0) & (ctrl_stop_state)) ? counter_bd9600 : 0;
                            baudrate_div40_div16 <= ~baudrate_div40_div16;
                        end
                        else counter_bd9600 <= counter_bd9600 + 1;
                    end
                    default: begin
                        // default
                        if(&counter_bd9600[5:0]) begin
                            counter_bd9600 <= ((baudrate_div40_div128 == 0) & (ctrl_stop_state)) ? counter_bd9600 : 0;
                            baudrate_div40_div128 <= ~baudrate_div40_div128;
                        end
                        else counter_bd9600 <= counter_bd9600 + 1;
                    end
                endcase 
//                if(counter_bd9600 == 127 & baudrate_sel == BD9600_ENCODE) begin
//                    counter_bd9600 <= ((baudrate_div40_div128 == 0) & (ctrl_stop_state)) ? counter_bd9600 : 0;
//                    baudrate_div40_div128 <= ~baudrate_div40_div128;
//                end
//                else counter_bd9600 <= counter_bd9600 + 1;
//                if(counter_bd9600 == 63 & baudrate_sel == BD19200_ENCODE) begin
//                    counter_bd9600 <= ((baudrate_div40_div128 == 0) & (ctrl_stop_state)) ? counter_bd9600 : 0;
//                end
//                else 
//                baudrate_div40_div16 <= (&counter_bd9600[3:0]) ? ~baudrate_div40_div16 : baudrate_div40_div16;
            end
        end
    end 
    
    // Unique_1 clock mode
    wire unique_1_mode_clk;
    reg baudrate_unique_1;
    reg [UNIQUE_1_COUNTER_WIDTH - 1:0] counter_unique_1;
    
    assign unique_1_mode_clk = (baudrate_sel == BD_UNIQUE_1_ENCODE) ? clk : 1'b0;
    
    always @(posedge unique_1_mode_clk, negedge rst_n) begin
        if(!rst_n) begin
            counter_unique_1 <= (CLOCK_DIVIDER_UNIQUE_1 - 1);
            baudrate_unique_1 <= 0; 
        end
        else begin
            if(TX_disable) begin
                counter_unique_1 <= (CLOCK_DIVIDER_UNIQUE_1 - 1);
                baudrate_unique_1 <= 0; 
            end
            else begin
                if(counter_unique_1 == (CLOCK_DIVIDER_UNIQUE_1 - 1)) begin
                    counter_unique_1 <= 0;
                    baudrate_unique_1 <= ~baudrate_unique_1;
                end
                else counter_unique_1 <= counter_unique_1 + 1;
            end
        end
    end
    
    // Unique_1 clock mode
    wire unique_2_mode_clk;
    reg baudrate_unique_2;
    reg [UNIQUE_2_COUNTER_WIDTH - 1:0] counter_unique_2;
    
    assign unique_2_mode_clk = (baudrate_sel == BD_UNIQUE_2_ENCODE) ? clk : 1'b0;
    
    always @(posedge unique_2_mode_clk, negedge rst_n) begin
        if(!rst_n) begin
            counter_unique_2 <= (CLOCK_DIVIDER_UNIQUE_2 - 1);
            baudrate_unique_2 <= 0; 
        end
        else begin
            if(TX_disable) begin
                counter_unique_2 <= (CLOCK_DIVIDER_UNIQUE_2 - 1);
                baudrate_unique_2 <= 0; 
            end
            else begin
                if(counter_unique_2 == (CLOCK_DIVIDER_UNIQUE_2 - 1)) begin
                    counter_unique_2 <= 0;
                    baudrate_unique_2 <= ~baudrate_unique_2;
                end
                else counter_unique_2 <= counter_unique_2 + 1;
            end
        end
    end
    
    
    ////////////////////////////////////////////////////////////////////
    
    assign TX_complete = TX_disable;
    assign baudrate_clk =   (baudrate_sel == BD4800_ENCODE) ? baudrate_div40_div128 : 
                            (baudrate_sel == BD9600_ENCODE) ? baudrate_div40_div64 : 
                            (baudrate_sel == BD19200_ENCODE) ? baudrate_div40_div32 : 
                            (baudrate_sel == BD38400_ENCODE) ? baudrate_div40_div16 :
                            (baudrate_sel == BD_UNIQUE_1_ENCODE) ? baudrate_unique_1 : baudrate_unique_2;
                            
//  //Debug area ///////////////////////////////
//    assign debug_128 = counter_bd9600;
//    assign debug_40 = counter_div40;
//    assign debug = baudrate_div40;
//    assign debug_1 = counter_div40;
//    assign TX_disable_wire = TX_disable;
endmodule

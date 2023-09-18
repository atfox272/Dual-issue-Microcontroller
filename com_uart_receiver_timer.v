module com_uart_receiver_timer
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
        
        parameter BAUDRATE_SEL_WIDTH = $clog2(BD_UNIQUE_2_ENCODE + 1),  // Max encode value
        parameter UNIQUE_1_COUNTER_WIDTH = $clog2(CLOCK_DIVIDER_UNIQUE_1 + 1), 
        parameter UNIQUE_2_COUNTER_WIDTH = $clog2(CLOCK_DIVIDER_UNIQUE_2 + 1),  
        parameter FIRST_COUNTER_WIDTH = $clog2(CLOCK_DIVIDER)
    )
    (
    input clk,                  // 125Mhz (internal clk)
    input [BAUDRATE_SEL_WIDTH - 1:0] baudrate_sel,    // 2 option: 
    input rx_port,              // data_in
    input rst_n,
    output baudrate_clk,
    
    // From rx-controller
    input stop_cond
    // Debug area
//    ,output [6:0] debug_128,
//    output [5:0] debug_40
    );
    // Divider 
//    localparam divider = DEVICE_CLOCK_DIV4800; 
    // Baudrate clock (prescaler)
	 
    wire read_en;
    reg read_en_start;
    reg read_en_stop;
    reg [FIRST_COUNTER_WIDTH - 1:0] counter_div41;    // counter 0 -> 40 (to 50Mhz / 50 (clk mod 50)
    reg baudrate_div41;            // Baudrate select
    wire baudrate_div41_clk;
    
    assign baudrate_div41_clk = (read_en) ? baudrate_div41 : clk;
    assign read_en = ~(read_en_start ^ read_en_stop);      
	 
    // Controller of "start_state" and "stop_state"  
    always @(posedge stop_cond, negedge rst_n) begin
        if(!rst_n) read_en_stop <= 1;
        else read_en_stop <= ~read_en_start;
    end 
    always @(negedge rx_port, negedge rst_n) begin
        if(!rst_n) read_en_start <= 0;
        else begin
            read_en_start <= read_en_stop;
        end
    end
    // Controller of Divider
    
    
    wire normal_mode_clk;
    wire normol_mode_en = ( baudrate_sel == BD4800_ENCODE   | baudrate_sel == BD9600_ENCODE |
                            baudrate_sel == BD19200_ENCODE  | baudrate_sel == BD38400_ENCODE);
    
    assign normal_mode_clk = (normol_mode_en) ? clk : 1'b0;
    always @(posedge normal_mode_clk, negedge rst_n) begin
        if(!rst_n) begin
            baudrate_div41 <= 0; 
            counter_div41 <= (CLOCK_DIVIDER - 1);    
        end
        else begin 
            if(!read_en) begin
                baudrate_div41 <= 0; 
                counter_div41 <= (CLOCK_DIVIDER - 1); 
            end
            else begin
                if(counter_div41 == (CLOCK_DIVIDER - 1)) begin 
                    baudrate_div41 <= ~baudrate_div41;
                    counter_div41 <= 0;
                end
                else counter_div41 <= counter_div41 + 1;
            end
        end
    end
    reg [6:0] counter_div41_div128;
    reg baudrate_div41_div128;      // Bd 9600
    reg baudrate_div41_div64;       // Bd 19200
    reg baudrate_div41_div32;       // Bd 38400
    reg baudrate_div41_div16;       // Bd 76800
    
    always @(posedge baudrate_div41_clk, negedge rst_n) begin
        // posedge <read_en> for inscresing immediately after read_en is rising
        // posedge <read_en> for converting to ready-state (counter_div41_div128 = 127)
        if(!rst_n) begin
            counter_div41_div128 <= 127;        // Ready for next clk will toggle, It will be toggle immediately, when read_en == 1
            baudrate_div41_div128 <= 0; 
            baudrate_div41_div64 <= 0; 
            baudrate_div41_div32 <= 0; 
            baudrate_div41_div16 <= 0;
        end
        else begin 
            if(!read_en) begin
                counter_div41_div128 <= 127;        // Ready for next clk will toggle, It will be toggle immediately, when read_en == 1
                baudrate_div41_div128 <= 0; 
                baudrate_div41_div64 <= 0; 
                baudrate_div41_div32 <= 0; 
                baudrate_div41_div16 <= 0;
            end
            else begin
                if(counter_div41_div128 == 127) counter_div41_div128 <= 0;
                else counter_div41_div128 <= counter_div41_div128 + 1; 
                
                baudrate_div41_div128 <= (&counter_div41_div128[6:0]) ? ~baudrate_div41_div128: baudrate_div41_div128;
                baudrate_div41_div64 <= (&counter_div41_div128[5:0]) ? ~baudrate_div41_div64 : baudrate_div41_div64;
                baudrate_div41_div32 <= (&counter_div41_div128[4:0]) ? ~baudrate_div41_div32 : baudrate_div41_div32;
                baudrate_div41_div16 <= (&counter_div41_div128[3:0]) ? ~baudrate_div41_div16 : baudrate_div41_div16;
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
            if(!read_en) begin
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
    
    // Unique_2 clock mode 
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
            if(!read_en) begin
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
    ////////////////////////////
    assign baudrate_clk =   (baudrate_sel == BD4800_ENCODE) ? baudrate_div41_div128 : 
                            (baudrate_sel == BD9600_ENCODE) ? baudrate_div41_div64 : 
                            (baudrate_sel == BD19200_ENCODE) ? baudrate_div41_div32 : 
                            (baudrate_sel == BD38400_ENCODE) ? baudrate_div41_div16 : 
                            (baudrate_sel == BD_UNIQUE_1_ENCODE) ? baudrate_unique_1 : baudrate_unique_2;

//    Debug area /////////////////////////////////
//    assign baudrate_clk = baudrate_div41_div128;    
//    assign debug_128 = counter_div41_div128;
//    assign debug_40 = counter_div41;
endmodule

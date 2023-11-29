module Registers_management
    #(
    parameter REGISTER_AMOUNT   = 32,
    parameter REGISTER_WIDTH    = 64,
    
    parameter REG_CTN_WIDTH     = $clog2(REGISTER_AMOUNT)
    )
    (
    input   wire                        clk,
    // Processor
    input   wire [REGISTER_WIDTH - 1:0] processor_registers_1   [0:REGISTER_AMOUNT - 1],
    input   wire [REGISTER_WIDTH - 1:0] processor_registers_2   [0:REGISTER_AMOUNT - 1],
    input   wire                        processor_idle_1,
    input   wire                        processor_idle_2,
    output  wire                        synchronization_processor_1,
    output  wire                        synchronization_processor_2,
    // Multi-processor manager
    input   wire                        boot_renew_register_1,
    input   wire                        boot_renew_register_2,
    input   wire [REG_CTN_WIDTH - 1:0]  register_num,           // Maximum renew_reg is 3
    output  wire                        new_data_register       [0:REGISTER_AMOUNT - 1],  
    input   wire                        main_program_state,        
    input   wire [REGISTER_WIDTH - 1:0] ra_register,
    output  wire                        synchronized_processors,
    output  wire [0:REGISTER_AMOUNT - 1]processing_register_table,
    // To Processor & Multi-processor manager
    output  wire [REGISTER_WIDTH - 1:0] registers_renew         [0:REGISTER_AMOUNT - 1],
    
    input   wire                        rst_n
    );
    
    reg [0:REGISTER_AMOUNT - 1] new_data_register_reg_main;
    reg [0:REGISTER_AMOUNT - 1] new_data_register_reg_sub;
    reg [0:REGISTER_AMOUNT - 1] processing_register_table_reg_1;
    reg [0:REGISTER_AMOUNT - 1] processing_register_table_reg_2;
    reg                         processor_main_updater_state;
    reg                         processor_sub_updater_state;
    reg [REG_CTN_WIDTH - 1:0]   register1_num_buf_1;
    reg [REG_CTN_WIDTH - 1:0]   register1_num_buf_2;
    wire                        synchronized_processor_1;
    reg                         synchronized_processor_1_true;
    reg                         synchronized_processor_1_false;
    wire                        synchronized_processor_2;
    reg                         synchronized_processor_2_true;
    reg                         synchronized_processor_2_false;
    reg                         synchronization_processor_1_reg;
    reg                         synchronization_processor_2_reg;
    
    localparam IDLE_STATE       = 1'd0;
    localparam UPDATING_STATE   = 1'd1;
//    localparam PRE_UPDATING_STATE = 3;
//    localparam SPEC_UPDATING_STATE = 2;
//    localparam SPEC_PRE_UPDATING_STATE = 4;
    
    for(genvar i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
    assign processing_register_table[i] = processing_register_table_reg_1[i] | processing_register_table_reg_2[i];
    end
    for(genvar i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
    if(i == 1) begin
        assign new_data_register[i] = 1;
        assign registers_renew[i] = ra_register;
    end
    else begin
        assign new_data_register[i] = new_data_register_reg_main[i] ^ new_data_register_reg_sub[i];
        assign registers_renew[i] = (new_data_register[i]) ? processor_registers_1[i] : processor_registers_2[i];
    end
    end
    assign synchronization_processor_1 = synchronization_processor_1_reg;
    assign synchronization_processor_2 = synchronization_processor_2_reg;
    assign synchronized_processor_1 = synchronized_processor_1_true ^ synchronized_processor_1_false;
    assign synchronized_processor_2 = synchronized_processor_2_true ^ synchronized_processor_2_false;
    assign synchronized_processors = synchronized_processor_1 & synchronized_processor_2;
                           
                            
    // Processor 1 (main) updater
    logic [0:REGISTER_AMOUNT - 1]   new_data_register_reg_main_n;
    logic                           processor_main_updater_state_n;
    logic                           synchronization_processor_1_n;
    logic [0:REGISTER_AMOUNT - 1]   processing_register_table_reg_1_n;
    logic                           synchronized_processor_1_true_n;
    logic                           synchronized_processor_2_false_n;
    logic [REG_CTN_WIDTH - 1:0]     register1_num_buf_1_n;
    always_comb begin
        new_data_register_reg_main_n = new_data_register_reg_main;
        processor_main_updater_state_n = processor_main_updater_state;
        synchronization_processor_1_n = synchronization_processor_1_reg;
        processing_register_table_reg_1_n = processing_register_table_reg_1;
        synchronized_processor_1_true_n = synchronized_processor_1_true;
        synchronized_processor_2_false_n = synchronized_processor_2_false;
        register1_num_buf_1_n = register1_num_buf_1;
        case(processor_main_updater_state) 
            IDLE_STATE: begin
                synchronization_processor_1_n = 0;
                if(boot_renew_register_1 == 1) begin
                    processor_main_updater_state_n = UPDATING_STATE;
                    register1_num_buf_1_n = register_num[4:0];
                    processing_register_table_reg_1_n[register_num[4:0]] = 1'b1;
                    synchronized_processor_2_false_n = synchronized_processor_2_true;    // A ^ B == 0 (A == B)
                end
                else if(synchronized_processor_1 == 0 & processor_sub_updater_state == IDLE_STATE & processor_idle_1 == 1 & boot_renew_register_2 == 0) begin
                    processor_main_updater_state_n = IDLE_STATE;
                    synchronization_processor_1_n = 1;
                    synchronized_processor_1_true_n = ~synchronized_processor_1_false;// A ^ B == 1 (A == ~B)
                end
            end
            UPDATING_STATE : begin
                if(processor_idle_1) begin
                    processor_main_updater_state_n = IDLE_STATE;
                    // Update data flag (change HIGH)
                    new_data_register_reg_main_n[register1_num_buf_1] = ~new_data_register_reg_sub[register1_num_buf_1];
                    // Update processor_register_table
                    processing_register_table_reg_1_n[register1_num_buf_1] = 1'b0;
                end
                else processor_main_updater_state_n = processor_main_updater_state;
            end
            default: begin
                
            end
        endcase
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            new_data_register_reg_main[0:REGISTER_AMOUNT - 1] <= {32{1'b1}}; 
            processor_main_updater_state <= IDLE_STATE;
            synchronization_processor_1_reg <= 0;
            processing_register_table_reg_1 <= {32{1'b0}};
            synchronized_processor_1_true <= 1;
            synchronized_processor_2_false <= 0;
        end
        else begin
            new_data_register_reg_main <= new_data_register_reg_main_n;
            processor_main_updater_state <= processor_main_updater_state_n;
            synchronization_processor_1_reg <= synchronization_processor_1_n;
            processing_register_table_reg_1 <= processing_register_table_reg_1_n;
            synchronized_processor_1_true <= synchronized_processor_1_true_n;
            synchronized_processor_2_false <= synchronized_processor_2_false_n;
            register1_num_buf_1 <= register1_num_buf_1_n;
        end
    end
    // Processor 2 (sub) upadater
    logic [0:REGISTER_AMOUNT - 1]   new_data_register_reg_sub_n;
    logic                           processor_sub_updater_state_n;
    logic                           synchronization_processor_2_n;
    logic [0:REGISTER_AMOUNT - 1]   processing_register_table_reg_2_n;
    logic                           synchronized_processor_2_true_n;
    logic                           synchronized_processor_1_false_n;
    logic [REG_CTN_WIDTH - 1:0]     register1_num_buf_2_n;  
    always_comb begin
        new_data_register_reg_sub_n = new_data_register_reg_sub;
        processor_sub_updater_state_n = processor_sub_updater_state;
        synchronization_processor_2_n = synchronization_processor_2_reg;
        processing_register_table_reg_2_n = processing_register_table_reg_2;
        synchronized_processor_2_true_n = synchronized_processor_2_true;
        synchronized_processor_1_false_n = synchronized_processor_1_false;
        register1_num_buf_2_n = register1_num_buf_2;
        case(processor_sub_updater_state) 
            IDLE_STATE: begin
                synchronization_processor_2_n = 0;
                if(boot_renew_register_2 == 1) begin
                    processor_sub_updater_state_n = UPDATING_STATE;
                    register1_num_buf_2_n = register_num[4:0];
                    synchronized_processor_1_false_n = synchronized_processor_1_true;    // A ^ B == 0 (A == B)
                    processing_register_table_reg_2_n[register_num[4:0]] = 1'b1;
                end
                else if(synchronized_processor_2 == 0 & processor_main_updater_state == IDLE_STATE & processor_idle_2 == 1 & boot_renew_register_1 == 0) begin
                    processor_sub_updater_state_n = IDLE_STATE;
                    synchronization_processor_2_n = 1;
                    synchronized_processor_2_true_n = ~synchronized_processor_2_false;   // A ^ B == 1 (A == ~B)
                end
            end
            UPDATING_STATE: begin
                if(processor_idle_2) begin
                    processor_sub_updater_state_n = IDLE_STATE;
                    // Update data flag (change LOW)
                    new_data_register_reg_sub_n[register1_num_buf_2] = new_data_register_reg_main[register1_num_buf_2];
                    // Update processor_register_table
                    processing_register_table_reg_2_n[register1_num_buf_2] = 1'b0;
                end
                else processor_sub_updater_state_n = processor_sub_updater_state;
            end
            default: begin
            
            end
        endcase
    end
    always @(posedge clk) begin
        if(!rst_n) begin
            new_data_register_reg_sub[0:REGISTER_AMOUNT - 1] <= {32{1'b0}};
            processor_sub_updater_state <= IDLE_STATE;
            synchronization_processor_2_reg <= 0;
            processing_register_table_reg_2 <= {32{1'b0}};
            synchronized_processor_2_true <= 1;
            synchronized_processor_1_false <= 0;
        end
        else begin
            new_data_register_reg_sub <= new_data_register_reg_sub_n;
            processor_sub_updater_state <= processor_sub_updater_state_n;
            synchronization_processor_2_reg <= synchronization_processor_2_n;
            processing_register_table_reg_2 <= processing_register_table_reg_2_n;
            synchronized_processor_2_true <= synchronized_processor_2_true_n;
            synchronized_processor_1_false <= synchronized_processor_1_false_n;
            register1_num_buf_2 <= register1_num_buf_2_n;
        end
    end
endmodule

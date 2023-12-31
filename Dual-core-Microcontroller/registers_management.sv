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
    input   wire                        boot_renew_3registers_1,
    input   wire                        boot_renew_3registers_2,
    input   wire [REG_CTN_WIDTH*3 - 1:0]register_num,           // Maximum renew_reg is 3
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
    reg [2:0]                   processor_main_updater_state;
    reg [2:0]                   processor_sub_updater_state;
    reg [REG_CTN_WIDTH - 1:0]   register1_num_buf_1;
    reg [REG_CTN_WIDTH - 1:0]   register2_num_buf_1;
    reg [REG_CTN_WIDTH - 1:0]   register3_num_buf_1;
    reg [REG_CTN_WIDTH - 1:0]   register1_num_buf_2;
    reg [REG_CTN_WIDTH - 1:0]   register2_num_buf_2;
    reg [REG_CTN_WIDTH - 1:0]   register3_num_buf_2;
    wire                        synchronized_processor_1;
    reg                         synchronized_processor_1_true;
    reg                         synchronized_processor_1_false;
    wire                        synchronized_processor_2;
    reg                         synchronized_processor_2_true;
    reg                         synchronized_processor_2_false;
    reg                         synchronization_processor_1_reg;
    reg                         synchronization_processor_2_reg;
    
    localparam IDLE_STATE = 0;
    localparam UPDATING_STATE = 1;
    localparam PRE_UPDATING_STATE = 3;
    localparam SPEC_UPDATING_STATE = 2;
    localparam SPEC_PRE_UPDATING_STATE = 4;
    
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
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            new_data_register_reg_main[0:REGISTER_AMOUNT - 1] <= {32{1'b1}}; 
            processor_main_updater_state <= IDLE_STATE;
            synchronization_processor_1_reg <= 0;
            processing_register_table_reg_1 <= {32{1'b0}};
            synchronized_processor_1_true <= 1;
            synchronized_processor_2_false <= 0;
        end
        else begin
            case(processor_main_updater_state) 
                IDLE_STATE: begin
                    synchronization_processor_1_reg <= 0;
                    if(boot_renew_register_1 == 1) begin
                        processor_main_updater_state <= PRE_UPDATING_STATE;
                        register1_num_buf_1 <= register_num[4:0];
                        processing_register_table_reg_1[register_num[4:0]] <= 1'b1;
                        synchronized_processor_2_false <= synchronized_processor_2_true;    // A ^ B == 0 (A == B)
                    end
                    else begin
                        if(synchronized_processor_1 == 0 & processor_sub_updater_state == IDLE_STATE & processor_idle_1 == 1 & boot_renew_register_2 == 0) begin
                            processor_main_updater_state <= IDLE_STATE;
                            synchronization_processor_1_reg <= 1;
                            synchronized_processor_1_true <= ~synchronized_processor_1_false;// A ^ B == 1 (A == ~B)
                        end
                    end
                end
                PRE_UPDATING_STATE: begin
                    if(processor_idle_1 == 0) begin
                        processor_main_updater_state <= UPDATING_STATE;
                    end
                    else processor_main_updater_state <= processor_main_updater_state;
                end
                UPDATING_STATE : begin
                    if(processor_idle_1) begin
                        processor_main_updater_state <= IDLE_STATE;
                        // Update data flag (change HIGH)
                        new_data_register_reg_main[register1_num_buf_1] <= ~new_data_register_reg_sub[register1_num_buf_1];
                        // Update processor_register_table
                        processing_register_table_reg_1[register1_num_buf_1] <= 1'b0;
                    end
                    else processor_main_updater_state <= processor_main_updater_state;
                end
                default: begin
                    
                end
            endcase
        end
    end
    // Processor 2 (sub) upadater
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            new_data_register_reg_sub[0:REGISTER_AMOUNT - 1] <= {32{1'b0}};
            processor_sub_updater_state <= IDLE_STATE;
            synchronization_processor_2_reg <= 0;
            processing_register_table_reg_2 <= {32{1'b0}};
            synchronized_processor_2_true <= 1;
            synchronized_processor_1_false <= 0;
        end
        else begin
            case(processor_sub_updater_state) 
                IDLE_STATE: begin
                    synchronization_processor_2_reg <= 0;
                    if(boot_renew_register_2 == 1) begin
                        processor_sub_updater_state <= PRE_UPDATING_STATE;
                        register1_num_buf_2 <= register_num[4:0];
                        synchronized_processor_1_false <= synchronized_processor_1_true;    // A ^ B == 0 (A == B)
                        processing_register_table_reg_2[register_num[4:0]] <= 1'b1;
                    end
                    else if(boot_renew_3registers_2) begin
                        processor_sub_updater_state <= SPEC_PRE_UPDATING_STATE;
                        register1_num_buf_2 <= register_num[4:0];
                        register2_num_buf_2 <= register_num[9:5];
                        register3_num_buf_2 <= register_num[14:10];
                        processing_register_table_reg_2[register_num[4:0]] <= 1'b1;
                        processing_register_table_reg_2[register_num[9:5]] <= 1'b1;
                        processing_register_table_reg_2[register_num[14:10]] <= 1'b1;
                        synchronized_processor_1_false <= synchronized_processor_1_true;    // A ^ B == 0 (A == B)
                    end
                    else if(synchronized_processor_2 == 0 & processor_main_updater_state == IDLE_STATE & processor_idle_2 == 1 & boot_renew_register_1 == 0) begin
                            processor_sub_updater_state <= IDLE_STATE;
                            synchronization_processor_2_reg <= 1;
                            synchronized_processor_2_true <= ~synchronized_processor_2_false;   // A ^ B == 1 (A == ~B)
                    end
                end
                PRE_UPDATING_STATE: begin
                    if(processor_idle_2 == 0) begin
                        processor_sub_updater_state <= UPDATING_STATE;
                    end
                    else processor_sub_updater_state <= processor_sub_updater_state;
                end 
                SPEC_PRE_UPDATING_STATE: begin
                    if(processor_idle_2 == 0) begin
                        processor_sub_updater_state <= SPEC_UPDATING_STATE;
                    end
                    else processor_sub_updater_state <= processor_sub_updater_state;
                end 
                UPDATING_STATE: begin
                    if(processor_idle_2) begin
                        processor_sub_updater_state <= IDLE_STATE;
                        // Update data flag (change LOW)
                        new_data_register_reg_sub[register1_num_buf_2] <= new_data_register_reg_main[register1_num_buf_2];
                        // Update processor_register_table
                        processing_register_table_reg_2[register1_num_buf_2] <= 1'b0;
                    end
                    else processor_sub_updater_state <= processor_sub_updater_state;
                end
                SPEC_UPDATING_STATE: begin
                    if(processor_idle_2) begin
                        processor_sub_updater_state <= IDLE_STATE;
                        // Update data flag (change LOW)
                        new_data_register_reg_sub[register1_num_buf_2] <= new_data_register_reg_main[register1_num_buf_2];
                        new_data_register_reg_sub[register2_num_buf_2] <= new_data_register_reg_main[register2_num_buf_2];
                        new_data_register_reg_sub[register3_num_buf_2] <= new_data_register_reg_main[register3_num_buf_2];
                        // Update processor_register_table
                        processing_register_table_reg_2[register1_num_buf_2] <= 1'b0;
                        processing_register_table_reg_2[register2_num_buf_2] <= 1'b0;
                        processing_register_table_reg_2[register3_num_buf_2] <= 1'b0;
                    end
                end
                default: begin
                
                end
            endcase
        end
    end
endmodule

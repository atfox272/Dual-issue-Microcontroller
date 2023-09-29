module registers_management
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
    input   wire [REG_CTN_WIDTH - 1:0]  register_num,
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
    reg [0:REGISTER_AMOUNT - 1] processing_register_table_reg;
    reg [1:0]                   processor_main_updater_state;
    reg [1:0]                   processor_sub_updater_state;
    reg [REG_CTN_WIDTH - 1:0]   register_num_buf_1;
    reg [REG_CTN_WIDTH - 1:0]   register_num_buf_2;
    reg                         synchronized_processor_1;
    reg                         synchronized_processor_2;
    reg                         synchronization_processor_1_reg;
    reg                         synchronization_processor_2_reg;
    
    localparam IDLE_STATE = 0;
    localparam UPDATING_STATE = 1;
    localparam PRE_UPDATING_STATE = 3;
    localparam SYNCHRONIZATION_STATE = 2;
    
    assign processing_register_table = processing_register_table_reg;
    for(genvar i = 0; i < REGISTER_AMOUNT; i = i + 1) begin
    if(i == 1) begin
        assign new_data_register[i] = 1;
        assign registers_renew[i] = ra_register;
    end
    else begin
        assign new_data_register[i] = new_data_register_reg_main[i] ^ new_data_register_reg_sub[i];
        assign registers_renew[i] = (main_program_state) ? ((new_data_register[i]) ? processor_registers_1[i] : processor_registers_2[i]) : processor_registers_1[i];
    end
    end
    assign synchronization_processor_1 = synchronization_processor_1_reg;
    assign synchronization_processor_2 = synchronization_processor_2_reg;
    assign synchronized_processors = synchronized_processor_1 & synchronized_processor_2;
                           
                            
    // Processor 1 (main) updater
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            new_data_register_reg_main[0:REGISTER_AMOUNT - 1] <= {32{1'b1}}; 
            processor_main_updater_state <= IDLE_STATE;
            synchronized_processor_1 <= 1;
            synchronization_processor_1_reg <= 0;
            processing_register_table_reg <= {32{1'b0}};
        end
        else begin
            case(processor_main_updater_state) 
                IDLE_STATE: begin
                    if(boot_renew_register_1 == 1) begin
                        processor_main_updater_state <= PRE_UPDATING_STATE;
                        register_num_buf_1 <= register_num;
                        processing_register_table_reg[register_num] <= 1'b1;
                        synchronized_processor_2 <= 0;
                    end
                    else begin
                        if(synchronized_processor_1 == 0 & processor_sub_updater_state == IDLE_STATE) begin
                            processor_main_updater_state <= SYNCHRONIZATION_STATE;
                            synchronization_processor_1_reg <= 1;
                            synchronized_processor_1 <= 1;
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
                        new_data_register_reg_main[register_num_buf_1] <= ~new_data_register_reg_sub[register_num_buf_1];
                        // Update processor_register_table
                        processing_register_table_reg[register_num_buf_1] <= 1'b0;
                    end
                    else processor_main_updater_state <= processor_main_updater_state;
                end
                SYNCHRONIZATION_STATE: begin
                    processor_main_updater_state <= IDLE_STATE;
                    synchronization_processor_1_reg <= 0;
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
            synchronized_processor_2 <= 1;
            synchronization_processor_2_reg <= 0;
        end
        else begin
            case(processor_sub_updater_state) 
                IDLE_STATE: begin
                    if(boot_renew_register_2 == 1) begin
                        processor_sub_updater_state <= PRE_UPDATING_STATE;
                        register_num_buf_2 <= register_num;
                        synchronized_processor_1 <= 0;
                        processing_register_table_reg[register_num] <= 1'b1;
                    end
                    else begin
                        if(synchronized_processor_2 == 0 & processor_main_updater_state == IDLE_STATE) begin
                            processor_sub_updater_state <= SYNCHRONIZATION_STATE;
                            synchronization_processor_2_reg <= 1;
                            synchronized_processor_2 <= 1;
                        end
                    end
                end
                PRE_UPDATING_STATE: begin
                    if(processor_idle_2 == 0) begin
                        processor_sub_updater_state <= UPDATING_STATE;
                    end
                    else processor_sub_updater_state <= processor_sub_updater_state;
                end 
                UPDATING_STATE: begin
                    if(processor_idle_2) begin
                        processor_sub_updater_state <= IDLE_STATE;
                        // Update data flag (change LOW)
                        new_data_register_reg_sub[register_num_buf_2] <= new_data_register_reg_main[register_num_buf_2];
                        // Update processor_register_table
                        processing_register_table_reg[register_num_buf_2] <= 1'b0;
                    end
                    else processor_sub_updater_state <= processor_sub_updater_state;
                end
                SYNCHRONIZATION_STATE: begin
                    processor_sub_updater_state <= IDLE_STATE;
                    synchronization_processor_2_reg <= 0;
                end
                default: begin
                
                end
            endcase
        end
    end
endmodule

module Multi_processor_manager
    #(
    parameter INSTRUCTION_WIDTH     = 32,   //32-bit instruction
    parameter PROGRAM_MEMORY_SIZE   = 64,
    
    parameter REGISTER_AMOUNT       = 32,
    
    parameter ADDR_WIDTH_PM         = $clog2(PROGRAM_MEMORY_SIZE)
    )
    (
    input wire clk,
    
    // Program memory
    input   wire    [INSTRUCTION_WIDTH - 1:0]   data_bus_rd_pm,
    input   wire                                rd_idle_pm,
    output  wire    [ADDR_WIDTH_PM - 1:0]       addr_rd_pm,
    output  wire                                rd_ins_pm,
    
    
    // Processor 1
    output  wire    [INSTRUCTION_WIDTH - 1:0]   fetch_instruction_1,
    output  wire                                boot_processor_1,
    input   wire                                processor_idle_1,
    
    // Processor 2
    output  wire    [INSTRUCTION_WIDTH - 1:0]   fetch_instruction_2,
    output  wire                                boot_processor_2,
    input   wire                                processor_idle_2,
    
    // Processor_1 & Processor_2
    output  wire    [REGISTER_AMOUNT - 1:0]     new_data_register,
    
    input rst_n
    );
endmodule

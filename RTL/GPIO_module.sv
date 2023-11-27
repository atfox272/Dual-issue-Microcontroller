module GPIO_module
    #(
    parameter  PORT_AMOUNT          = 2,
    parameter  PIN_AMOUNT           = 8,     // Pin per Port
    parameter  ADDR_INTERFACE_WIDTH = 64,
    localparam ADDR_PORT_WIDTH      = $clog2(PORT_AMOUNT)
    )
    (
    input   clk,
    
    output[PIN_AMOUNT - 1:0]            rdata_PORT,
    input [PIN_AMOUNT - 1:0]            wdata_PORT,
    input [ADDR_INTERFACE_WIDTH - 1:0]  raddr_PORT,
    input [ADDR_INTERFACE_WIDTH - 1:0]  waddr_PORT,
    input                               rd_req,
    input                               wr_req,
    output                              rd_available,       // Valid or Invalid Port
    output                              wr_available,       // Valid or Invalid Port
//    inout [PIN_AMOUNT - 1:0]            PORT [0:PORT_AMOUNT - 1],
    input [PIN_AMOUNT - 1:0]            PORT_i  [0:PORT_AMOUNT - 1],
    output[PIN_AMOUNT - 1:0]            PORT_o [0:PORT_AMOUNT - 1],
    input [PIN_AMOUNT - 1:0]            config_PIN [0:PORT_AMOUNT - 1],
    
    input rst_n
    );
    wire [PIN_AMOUNT - 1:0]       PORT_io [0:PORT_AMOUNT - 1];
    reg  [PIN_AMOUNT - 1:0]       PORT_reg  [0:PORT_AMOUNT - 1];
    wire [ADDR_PORT_WIDTH - 1:0]  raddr_PORT_align;
    wire [ADDR_PORT_WIDTH - 1:0]  waddr_PORT_align;
    
    assign raddr_PORT_align = raddr_PORT[ADDR_PORT_WIDTH - 1:0];
    assign waddr_PORT_align = waddr_PORT[ADDR_PORT_WIDTH - 1:0];
    for(genvar port_index = 0; port_index < PORT_AMOUNT; port_index = port_index + 1) begin
        for(genvar pin_index = 0; pin_index < PIN_AMOUNT; pin_index = pin_index + 1) begin
            assign PORT_o[port_index][pin_index] = PORT_reg[port_index][pin_index];
            assign PORT_io[port_index][pin_index] = (config_PIN[port_index][pin_index]) ? PORT_i[port_index][pin_index] : PORT_o[port_index][pin_index];
        end
    end 
    
    assign rd_available = raddr_PORT_align < PORT_AMOUNT;
    assign wr_available = waddr_PORT_align < PORT_AMOUNT;
    assign rdata_PORT = PORT_io[raddr_PORT_align];
    
    always @(posedge clk) begin
        if(!rst_n) begin
            for(int port_index = 0; port_index < PORT_AMOUNT; port_index = port_index + 1) begin
                for(int pin_index = 0; pin_index < PIN_AMOUNT; pin_index = pin_index + 1) begin
                    PORT_reg[port_index][pin_index] <= 1'b0;
                end 
            end 
        end
        else if (wr_req & wr_available) begin
            PORT_reg[waddr_PORT_align] <= wdata_PORT;
        end
    end 
endmodule
//    GPIO_module
//        #(
//        .PORT_AMOUNT(),
//        .PIN_AMOUNT()
//        )GPIO_module(
//        .clk(),
        
//        .rdata_PORT(),
//        .wdata_PORT(),
//        .raddr_PORT(),
//        .waddr_PORT(),
        
//        .rd_req(),
//        .wr_req(),
//        .rd_available(),
//        .wr_available(),
//        .PORT_i(),
//        .PORT_o(),
//        .config_PIN(),
        
//        .rst_n()
//        );

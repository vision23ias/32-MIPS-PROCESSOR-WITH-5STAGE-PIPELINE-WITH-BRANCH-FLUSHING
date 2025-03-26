module MEM_Stage (
    input wire clk,
    input wire reset,
    input wire MemWrite,
    input wire MemRead,
    input wire [31:0] ALUResult,
    input wire [31:0] WriteData,
    output reg [31:0] ReadData
);
    reg [31:0] DMEM [0:1023];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize data memory to zeros
            for (i = 0; i < 1024; i = i + 1) begin
                DMEM[i] <= 32'h00000000;
            end
        end
        else begin
            if (MemWrite)
                DMEM[ALUResult >> 2] <= WriteData;
            if (MemRead)
                ReadData <= DMEM[ALUResult >> 2];
        end
    end
endmodule
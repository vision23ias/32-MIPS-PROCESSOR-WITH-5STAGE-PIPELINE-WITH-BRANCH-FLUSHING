module WB_Stage (
    input wire [31:0] ALUResult,
    input wire [31:0] ReadData,
    input wire MemtoReg,
    output reg [31:0] WriteData
);
    always @(*) begin
        WriteData = MemtoReg ? ReadData : ALUResult;
    end
endmodule
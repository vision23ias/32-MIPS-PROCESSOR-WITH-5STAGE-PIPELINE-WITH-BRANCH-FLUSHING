module EX_MEM_Reg (
    input wire clk,
    input wire reset,
    input wire [31:0] ALUResult_in,
    input wire [31:0] WriteData_in,
    input wire [4:0] WriteReg_in,
    input wire RegWrite_in,
    input wire MemtoReg_in,
    input wire MemRead_in,
    input wire MemWrite_in,
    output reg [31:0] ALUResult_out,
    output reg [31:0] WriteData_out,
    output reg [4:0] WriteReg_out,
    output reg RegWrite_out,
    output reg MemtoReg_out,
    output reg MemRead_out,
    output reg MemWrite_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ALUResult_out <= 0;
            WriteData_out <= 0;
            WriteReg_out <= 0;
            RegWrite_out <= 0;
            MemtoReg_out <= 0;
            MemRead_out <= 0;
            MemWrite_out <= 0;
        end else begin
            ALUResult_out <= ALUResult_in;
            WriteData_out <= WriteData_in;
            WriteReg_out <= WriteReg_in;
            RegWrite_out <= RegWrite_in;
            MemtoReg_out <= MemtoReg_in;
            MemRead_out <= MemRead_in;
            MemWrite_out <= MemWrite_in;
        end
    end
endmodule
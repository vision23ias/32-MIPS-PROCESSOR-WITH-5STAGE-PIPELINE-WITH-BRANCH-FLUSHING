module MEM_WB_Reg (
    input wire clk,
    input wire reset,
    input wire [31:0] ALUResult_in,
    input wire [31:0] ReadData_in,
    input wire [4:0] WriteReg_in,
    input wire MemtoReg_in,
    input wire RegWrite_in,
    output reg [31:0] ALUResult_out,
    output reg [31:0] ReadData_out,
    output reg [4:0] WriteReg_out,
    output reg MemtoReg_out,
    output reg RegWrite_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ALUResult_out <= 0;
            ReadData_out <= 0;
            WriteReg_out <= 0;
            MemtoReg_out <= 0;
            RegWrite_out <= 0;
        end else begin
            ALUResult_out <= ALUResult_in;
            ReadData_out <= ReadData_in;
            WriteReg_out <= WriteReg_in;
            MemtoReg_out <= MemtoReg_in;
            RegWrite_out <= RegWrite_in;
        end
    end
endmodule
module ID_EX_Reg (
    input wire clk,
    input wire reset,
    input wire flush_ID_EX,
    
    // Inputs
    input wire [31:0] ReadData1_in,
    input wire [31:0] ReadData2_in,
    input wire [31:0] SignExtImm_in,
    input wire [4:0] Rs_in, Rt_in, Rd_in,
    input wire [5:0] Funct_in,
    input wire [3:0] ALUOp_in,
    input wire RegDst_in, ALUSrc_in, MemtoReg_in, 
    input wire RegWrite_in, MemRead_in, MemWrite_in, Branch_in,
    
    // Outputs
    output reg [31:0] ReadData1_out,
    output reg [31:0] ReadData2_out,
    output reg [31:0] SignExtImm_out,
    output reg [4:0] Rs_out, Rt_out, Rd_out,
    output reg [5:0] Funct_out,
    output reg [3:0] ALUOp_out,
    output reg RegDst_out, ALUSrc_out, MemtoReg_out,
    output reg RegWrite_out, MemRead_out, MemWrite_out, Branch_out
);
    always @(posedge clk or posedge reset) begin
        if (reset || flush_ID_EX) begin
            ReadData1_out <= 0;
            ReadData2_out <= 0;
            SignExtImm_out <= 0;
            Rs_out <= 0;
            Rt_out <= 0;
            Rd_out <= 0;
            Funct_out <= 0;
            ALUOp_out <= 4'b1111; // NOP operation
            RegDst_out <= 0;
            ALUSrc_out <= 0;
            MemtoReg_out <= 0;
            RegWrite_out <= 0;
            MemRead_out <= 0;
            MemWrite_out <= 0;
            Branch_out <= 0;
        end else begin
            ReadData1_out <= ReadData1_in;
            ReadData2_out <= ReadData2_in;
            SignExtImm_out <= SignExtImm_in;
            Rs_out <= Rs_in;
            Rt_out <= Rt_in;
            Rd_out <= Rd_in;
            Funct_out <= Funct_in;
            ALUOp_out <= ALUOp_in;
            RegDst_out <= RegDst_in;
            ALUSrc_out <= ALUSrc_in;
            MemtoReg_out <= MemtoReg_in;
            RegWrite_out <= RegWrite_in;
            MemRead_out <= MemRead_in;
            MemWrite_out <= MemWrite_in;
            Branch_out <= Branch_in;
        end
    end
endmodule
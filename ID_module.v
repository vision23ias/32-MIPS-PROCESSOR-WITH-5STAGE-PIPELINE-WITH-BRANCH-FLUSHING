module ID_Stage (
    input wire clk,
    input wire reset,
    input wire [31:0] PC,
    input wire [31:0] Instruction,
    input wire [31:0] WriteData,
    input wire RegWrite,
    input wire [4:0] WriteReg,

    // Control Unit inputs
    input wire RegDst_in,
    input wire ALUSrc_in,
    input wire MemtoReg_in,
    input wire RegWrite_in,
    input wire MemRead_in,
    input wire MemWrite_in,
    input wire Branch_in,
    input wire [3:0] ALUOp_in,

    // Outputs
    output reg [31:0] ReadData1,
    output reg [31:0] ReadData2,
    output reg [31:0] SignExtImm,
    output reg [4:0] Rs, Rt, Rd,
    output reg [5:0] Opcode, Funct,
    output reg [31:0] BranchTarget,

    // Control signals passed to next stage
    output reg RegDst_out,
    output reg ALUSrc_out,
    output reg MemtoReg_out,
    output reg RegWrite_out,
    output reg MemRead_out,
    output reg MemWrite_out,
    output reg Branch_out,
    output reg [3:0] ALUOp_out
);
    reg [31:0] RegFile[0:31];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize register file
            for (i = 0; i < 32; i = i + 1) begin
                RegFile[i] <= 32'h00000000;
            end
            // Set specific initial values
            RegFile[4] <= 32'h00000004;
            RegFile[5] <= 32'hAAAA5555;
            RegFile[6] <= 32'hAAAA5555;
            RegFile[10] <= 32'hDEADBEEF;
        end
        else if (RegWrite && WriteReg != 0) begin
            RegFile[WriteReg] <= WriteData;
            $display(" WriteBack: R[%0d] <= %h", WriteReg, WriteData);
        end
    end

    always @(*) begin
        Rs = Instruction[25:21];
        Rt = Instruction[20:16];
        Rd = Instruction[15:11];
        Opcode = Instruction[31:26];
        Funct = Instruction[5:0];
        SignExtImm = {{16{Instruction[15]}}, Instruction[15:0]};
        ReadData1 = RegFile[Rs];
        ReadData2 = RegFile[Rt];
        BranchTarget = PC + (SignExtImm << 2);

        // Pass control signals to next stage
        RegDst_out   = RegDst_in;
        ALUSrc_out   = ALUSrc_in;
        MemtoReg_out = MemtoReg_in;
        RegWrite_out = RegWrite_in;
        MemRead_out  = MemRead_in;
        MemWrite_out = MemWrite_in;
        Branch_out   = Branch_in;
        ALUOp_out    = ALUOp_in;
    end
endmodule
module MIPS_Processor (
    input wire clk,
    input wire reset,
    output wire [31:0] PC_out,
    output wire [31:0] Instruction_out,
    output wire [31:0] ALUResult_out,
    output wire [31:0] WriteData_out,
    output wire RegWrite_out
);
    // Program Counter and Instruction Memory
    wire [31:0] PC, PCNext, Instruction;
    wire PCWrite;

    // IF/ID pipeline register outputs
    wire [31:0] IF_ID_PC_out, IF_ID_Instruction_out;
    wire IF_ID_Write;

    // Control Unit signals
    wire RegDst_Ctrl, ALUSrc_Ctrl, MemtoReg_Ctrl, RegWrite_Ctrl;
    wire MemRead_Ctrl, MemWrite_Ctrl, Branch_Ctrl, Jump_Ctrl;
    wire [3:0] ALUOp_Ctrl;

    // After hazard stall mux
    wire RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jump;
    wire [3:0] ALUOp;

    // ID Stage outputs
    wire [4:0] Rs, Rt, Rd;
    wire [5:0] Opcode, Funct;
    wire [31:0] ReadData1, ReadData2, SignExtImm, BranchTarget;

    // ID/EX pipeline register outputs
    wire [4:0] ID_EX_Rs_out, ID_EX_Rt_out, ID_EX_Rd_out;
    wire [5:0] ID_EX_Funct_out;
    wire [3:0] ID_EX_ALUOp_out;
    wire ID_EX_RegDst_out, ID_EX_ALUSrc_out, ID_EX_MemtoReg_out, ID_EX_RegWrite_out;
    wire ID_EX_MemRead_out, ID_EX_MemWrite_out, ID_EX_Branch_out;
    wire [31:0] ID_EX_ReadData1_out, ID_EX_ReadData2_out, ID_EX_SignExtImm_out;

    // EX stage outputs
    wire [31:0] ALUResult;
    wire Zero;

    // Forwarding unit signals
    wire [1:0] ForwardA, ForwardB;

    // EX/MEM pipeline register outputs
    wire [31:0] EX_MEM_ALUResult_out, EX_MEM_WriteData_out;
    wire [4:0] EX_MEM_WriteReg_out;
    wire EX_MEM_RegWrite_out, EX_MEM_MemtoReg_out, EX_MEM_MemRead_out, EX_MEM_MemWrite_out;

    // MEM stage outputs
    wire [31:0] ReadData;

    // MEM/WB pipeline register outputs
    wire [31:0] MEM_WB_ALUResult_out, MEM_WB_ReadData_out;
    wire [4:0] MEM_WB_WriteReg_out;
    wire MEM_WB_RegWrite_out, MEM_WB_MemtoReg_out;

    // WB stage output
    wire [31:0] WriteData;

    // Hazard Detection Unit output
    wire ControlStall;

    // Assign outputs for observation
    assign PC_out = PC;
    assign Instruction_out = IF_ID_Instruction_out;
    assign ALUResult_out = ALUResult;
    assign WriteData_out = WriteData;
    assign RegWrite_out = MEM_WB_RegWrite_out;

    // =================== Stage Instantiations ===================

    // IF Stage
    IF_Stage IF_stage (
        .clk(clk), .reset(reset), .PCWrite(PCWrite),
        .PCNext(PCNext), .PC(PC), .Instruction(Instruction)
    );

    // IF/ID Register
    IF_ID_Reg IF_ID (
        .clk(clk), .reset(reset), .IF_ID_Write(IF_ID_Write),
        .IF_PC(PC), .IF_Instruction(Instruction),
        .IF_ID_PC_out(IF_ID_PC_out), .IF_ID_Instruction_out(IF_ID_Instruction_out)
    );

    // Control Unit
    Control_Unit CU (
        .Opcode(IF_ID_Instruction_out[31:26]),
        .IF_Flush(ControlStall),
        .RegDst(RegDst_Ctrl), .ALUSrc(ALUSrc_Ctrl), .MemtoReg(MemtoReg_Ctrl),
        .RegWrite(RegWrite_Ctrl), .MemRead(MemRead_Ctrl), .MemWrite(MemWrite_Ctrl),
        .Branch(Branch_Ctrl), .Jump(Jump_Ctrl), .ALUOp(ALUOp_Ctrl)
    );

    // Mux for inserting NOP on hazard stall
    assign RegDst   = ControlStall ? 0 : RegDst_Ctrl;
    assign ALUSrc   = ControlStall ? 0 : ALUSrc_Ctrl;
    assign MemtoReg = ControlStall ? 0 : MemtoReg_Ctrl;
    assign RegWrite = ControlStall ? 0 : RegWrite_Ctrl;
    assign MemRead  = ControlStall ? 0 : MemRead_Ctrl;
    assign MemWrite = ControlStall ? 0 : MemWrite_Ctrl;
    assign Branch   = ControlStall ? 0 : Branch_Ctrl;
    assign Jump     = ControlStall ? 0 : Jump_Ctrl;
    assign ALUOp    = ControlStall ? 4'b1111 : ALUOp_Ctrl;

    // ID Stage
    ID_Stage ID (
        .clk(clk), .reset(reset), .PC(IF_ID_PC_out),
        .Instruction(IF_ID_Instruction_out), .WriteData(WriteData),
        .RegWrite(MEM_WB_RegWrite_out), .WriteReg(MEM_WB_WriteReg_out),
        .RegDst_in(RegDst), .ALUSrc_in(ALUSrc), .MemtoReg_in(MemtoReg), .RegWrite_in(RegWrite),
        .MemRead_in(MemRead), .MemWrite_in(MemWrite), .Branch_in(Branch), .ALUOp_in(ALUOp),
        .ReadData1(ReadData1), .ReadData2(ReadData2), .SignExtImm(SignExtImm),
        .Rs(Rs), .Rt(Rt), .Rd(Rd), .Opcode(Opcode), .Funct(Funct), .BranchTarget(BranchTarget),
        .RegDst_out(ID_EX_RegDst_out), .ALUSrc_out(ID_EX_ALUSrc_out), .MemtoReg_out(ID_EX_MemtoReg_out),
        .RegWrite_out(ID_EX_RegWrite_out), .MemRead_out(ID_EX_MemRead_out),
        .MemWrite_out(ID_EX_MemWrite_out), .Branch_out(ID_EX_Branch_out), .ALUOp_out(ID_EX_ALUOp_out)
    );

    // ID/EX Register
    ID_EX_Reg ID_EX (
        .clk(clk), .reset(reset), .flush_ID_EX(ControlStall),
        .ReadData1_in(ReadData1), .ReadData2_in(ReadData2),
        .SignExtImm_in(SignExtImm), .Rs_in(Rs), .Rt_in(Rt), .Rd_in(Rd),
        .Funct_in(Funct), .ALUOp_in(ID_EX_ALUOp_out),
        .RegDst_in(ID_EX_RegDst_out), .ALUSrc_in(ID_EX_ALUSrc_out), .MemtoReg_in(ID_EX_MemtoReg_out),
        .RegWrite_in(ID_EX_RegWrite_out), .MemRead_in(ID_EX_MemRead_out), .MemWrite_in(ID_EX_MemWrite_out),
        .Branch_in(ID_EX_Branch_out),
        .ReadData1_out(ID_EX_ReadData1_out), .ReadData2_out(ID_EX_ReadData2_out), .SignExtImm_out(ID_EX_SignExtImm_out),
        .Rs_out(ID_EX_Rs_out), .Rt_out(ID_EX_Rt_out), .Rd_out(ID_EX_Rd_out),
        .Funct_out(ID_EX_Funct_out), .ALUOp_out(ID_EX_ALUOp_out),
        .RegDst_out(ID_EX_RegDst_out), .ALUSrc_out(ID_EX_ALUSrc_out), .MemtoReg_out(ID_EX_MemtoReg_out),
        .RegWrite_out(ID_EX_RegWrite_out), .MemRead_out(ID_EX_MemRead_out),
        .MemWrite_out(ID_EX_MemWrite_out), .Branch_out(ID_EX_Branch_out)
    );

    // Forwarding Unit
    Forwarding_Unit FU (
        .EX_Rs(ID_EX_Rs_out), .EX_Rt(ID_EX_Rt_out),
        .MEM_RegDst(EX_MEM_WriteReg_out), .WB_RegDst(MEM_WB_WriteReg_out),
        .MEM_RegWrite(EX_MEM_RegWrite_out), .WB_RegWrite(MEM_WB_RegWrite_out),
        .ForwardA(ForwardA), .ForwardB(ForwardB)
    );

    // EX Stage
    EX_Stage EX (
        .ReadData1(ID_EX_ReadData1_out), .ReadData2(ID_EX_ReadData2_out),
        .SignExtImm(ID_EX_SignExtImm_out), .Funct(ID_EX_Funct_out), .ALUOp(ID_EX_ALUOp_out),
        .ALUSrc(ID_EX_ALUSrc_out), .ForwardA(ForwardA), .ForwardB(ForwardB),
        .ALUResult_MEM(EX_MEM_ALUResult_out), .WB_WriteData(WriteData),
        .ALUResult(ALUResult), .Zero(Zero)
    );

    // EX/MEM Register
    EX_MEM_Reg EX_MEM (
        .clk(clk), .reset(reset), 
        .ALUResult_in(ALUResult), 
        .WriteData_in(ID_EX_ReadData2_out),
        .WriteReg_in(ID_EX_Rd_out),
        .RegWrite_in(ID_EX_RegWrite_out), 
        .MemtoReg_in(ID_EX_MemtoReg_out),
        .MemRead_in(ID_EX_MemRead_out), 
        .MemWrite_in(ID_EX_MemWrite_out),
        .ALUResult_out(EX_MEM_ALUResult_out), 
        .WriteData_out(EX_MEM_WriteData_out),
        .WriteReg_out(EX_MEM_WriteReg_out),
        .RegWrite_out(EX_MEM_RegWrite_out), 
        .MemtoReg_out(EX_MEM_MemtoReg_out),
        .MemRead_out(EX_MEM_MemRead_out), 
        .MemWrite_out(EX_MEM_MemWrite_out)
    );

    // MEM Stage
    MEM_Stage MEM (
        .clk(clk), .reset(reset),
        .MemWrite(EX_MEM_MemWrite_out), 
        .MemRead(EX_MEM_MemRead_out),
        .ALUResult(EX_MEM_ALUResult_out), 
        .WriteData(EX_MEM_WriteData_out), 
        .ReadData(ReadData)
    );

    // MEM/WB Register
    MEM_WB_Reg MEM_WB (
        .clk(clk), .reset(reset), 
        .ALUResult_in(EX_MEM_ALUResult_out), 
        .ReadData_in(ReadData),
        .WriteReg_in(EX_MEM_WriteReg_out),
        .MemtoReg_in(EX_MEM_MemtoReg_out), 
        .RegWrite_in(EX_MEM_RegWrite_out),
        .ALUResult_out(MEM_WB_ALUResult_out), 
        .ReadData_out(MEM_WB_ReadData_out),
        .WriteReg_out(MEM_WB_WriteReg_out),
        .MemtoReg_out(MEM_WB_MemtoReg_out), 
        .RegWrite_out(MEM_WB_RegWrite_out)
    );

    // WB Stage
    WB_Stage WB (
        .ALUResult(MEM_WB_ALUResult_out), 
        .ReadData(MEM_WB_ReadData_out),
        .MemtoReg(MEM_WB_MemtoReg_out), 
        .WriteData(WriteData)
    );

    // Hazard Detection Unit
    Hazard_Detection_Unit HDU (
        .ID_Rs(Rs), .ID_Rt(Rt), 
        .EX_Rt(ID_EX_Rt_out), 
        .EX_MemRead(ID_EX_MemRead_out),
        .PCWrite(PCWrite), 
        .IF_ID_Write(IF_ID_Write), 
        .ControlStall(ControlStall)
    );

    // PC Next Calculation
    assign PCNext = PC + 4;
endmodule
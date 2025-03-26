module IF_Stage (
    input wire clk,
    input wire reset,
    input wire PCWrite,
    input wire [31:0] PCNext,
    output reg [31:0] PC,
    output reg [31:0] Instruction
);
    reg [31:0] IMEM [0:255];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC <= 0;
            // Initialize instruction memory
            IMEM[0]  <= 32'h8C220004; // lw  $2, 4($1)
            IMEM[1]  <= 32'h00432020; // add $4, $2, $3
            IMEM[2]  <= 32'hAC250008; // sw  $5, 8($1)
            IMEM[3]  <= 32'h00000000; // nop
            IMEM[4]  <= 32'h10430002; // beq $2, $3, +2
            IMEM[5]  <= 32'h20440005; // addi $4, $2, 5
            IMEM[6]  <= 32'h308500FF; // andi $5, $4, 0xFF
            IMEM[7]  <= 32'h34A6000F; // ori  $6, $5, 0x0F
            IMEM[8]  <= 32'h28C7000A; // slti $7, $6, 10
            IMEM[9]  <= 32'h00000000; // nop
            IMEM[10] <= 32'h08000002; // j 0x00000008
            
            // Fill remaining memory with nops
            for (i = 11; i < 256; i = i + 1) begin
                IMEM[i] <= 32'h00000000;
            end
        end
        else if (PCWrite) begin
            PC <= PCNext;
        end
    end

    always @(*) begin
        Instruction = IMEM[PC[9:2]]; // Word-aligned address access
    end
endmodule

module IF_ID_Reg (
    input wire clk,
    input wire reset,
    input wire IF_ID_Write,
    input wire [31:0] IF_PC,
    input wire [31:0] IF_Instruction,
    output reg [31:0] IF_ID_PC_out,
    output reg [31:0] IF_ID_Instruction_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            IF_ID_PC_out <= 0;
            IF_ID_Instruction_out <= 0;
        end else if (IF_ID_Write) begin
            IF_ID_PC_out <= IF_PC;
            IF_ID_Instruction_out <= IF_Instruction;
        end
    end
endmodule

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

module EX_Stage (
    input wire [31:0] ReadData1,
    input wire [31:0] ReadData2,
    input wire [31:0] SignExtImm,
    input wire [5:0] Funct,
    input wire [3:0] ALUOp,
    input wire ALUSrc,
    input wire [1:0] ForwardA,
    input wire [1:0] ForwardB,
    input wire [31:0] ALUResult_MEM,
    input wire [31:0] WB_WriteData,
    output reg [31:0] ALUResult,
    output reg Zero
);
    reg [31:0] ALUInput1, ALUInput2_preMux, ALUInput2;

    always @(*) begin
        // Forwarding Logic for ALUInput1
        case (ForwardA)
            2'b00: ALUInput1 = ReadData1;
            2'b01: ALUInput1 = WB_WriteData;
            2'b10: ALUInput1 = ALUResult_MEM;
            default: ALUInput1 = ReadData1;
        endcase

        // Forwarding Logic for ALUInput2 (before ALUSrc)
        case (ForwardB)
            2'b00: ALUInput2_preMux = ReadData2;
            2'b01: ALUInput2_preMux = WB_WriteData;
            2'b10: ALUInput2_preMux = ALUResult_MEM;
            default: ALUInput2_preMux = ReadData2;
        endcase

        // Apply ALUSrc MUX
        ALUInput2 = ALUSrc ? SignExtImm : ALUInput2_preMux;

        // ALU Operation
        case (ALUOp)
            4'b0000: ALUResult = ALUInput1 & ALUInput2;
            4'b0001: ALUResult = ALUInput1 | ALUInput2;
            4'b0010: ALUResult = ALUInput1 + ALUInput2;
            4'b0110: ALUResult = ALUInput1 - ALUInput2;
            4'b0111: ALUResult = (ALUInput1 < ALUInput2) ? 1 : 0;
            4'b1111: begin
                case (Funct)
                    6'b100000: ALUResult = ALUInput1 + ALUInput2;
                    6'b100010: ALUResult = ALUInput1 - ALUInput2;
                    6'b100100: ALUResult = ALUInput1 & ALUInput2;
                    6'b100101: ALUResult = ALUInput1 | ALUInput2;
                    6'b101010: ALUResult = (ALUInput1 < ALUInput2) ? 1 : 0;
                    default:   ALUResult = 0;
                endcase
            end
            default: ALUResult = 0;
        endcase

        Zero = (ALUResult == 0);
    end
endmodule

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

module Control_Unit (
    input wire [5:0] Opcode,
    input wire IF_Flush,
    output reg RegDst,
    output reg ALUSrc,
    output reg MemtoReg,
    output reg RegWrite,
    output reg MemRead,
    output reg MemWrite,
    output reg Branch,
    output reg Jump,
    output reg [3:0] ALUOp
);
    // Define opcodes
    localparam OP_RTYPE = 6'b000000;
    localparam OP_LW    = 6'b100011;
    localparam OP_SW    = 6'b101011;
    localparam OP_BEQ   = 6'b000100;
    localparam OP_BNE   = 6'b000101;
    localparam OP_ADDI  = 6'b001000;
    localparam OP_ANDI  = 6'b001100;
    localparam OP_ORI   = 6'b001101;
    localparam OP_SLTI  = 6'b001010;
    localparam OP_J     = 6'b000010;
    localparam OP_JAL   = 6'b000011;

    // Define ALUOp encoding
    localparam ALU_ADD  = 4'b0010;
    localparam ALU_SUB  = 4'b0110;
    localparam ALU_AND  = 4'b0000;
    localparam ALU_OR   = 4'b0001;
    localparam ALU_SLT  = 4'b0111;
    localparam ALU_NOP  = 4'b1111;

    always @(*) begin
        if (IF_Flush) begin
            RegDst   = 0;
            ALUSrc   = 0;
            MemtoReg = 0;
            RegWrite = 0;
            MemRead  = 0;
            MemWrite = 0;
            Branch   = 0;
            Jump     = 0;
            ALUOp    = ALU_NOP;
        end else begin
            // Default
            RegDst   = 0;
            ALUSrc   = 0;
            MemtoReg = 0;
            RegWrite = 0;
            MemRead  = 0;
            MemWrite = 0;
            Branch   = 0;
            Jump     = 0;
            ALUOp    = ALU_NOP;

            case (Opcode)
                OP_RTYPE: begin
                    RegDst   = 1;
                    ALUSrc   = 0;
                    MemtoReg = 0;
                    RegWrite = 1;
                    MemRead  = 0;
                    MemWrite = 0;
                    Branch   = 0;
                    Jump     = 0;
                    ALUOp    = 4'b1111; // Use funct for ALU control in EX stage
                end
                OP_LW: begin
                    RegDst   = 0;
                    ALUSrc   = 1;
                    MemtoReg = 1;
                    RegWrite = 1;
                    MemRead  = 1;
                    MemWrite = 0;
                    Branch   = 0;
                    Jump     = 0;
                    ALUOp    = ALU_ADD;
                end
                OP_SW: begin
                    RegDst   = 0;
                    ALUSrc   = 1;
                    MemtoReg = 0;
                    RegWrite = 0;
                    MemRead  = 0;
                    MemWrite = 1;
                    Branch   = 0;
                    Jump     = 0;
                    ALUOp    = ALU_ADD;
                end
                OP_BEQ: begin
                    RegDst   = 0;
                    ALUSrc   = 0;
                    MemtoReg = 0;
                    RegWrite = 0;
                    MemRead  = 0;
                    MemWrite = 0;
                    Branch   = 1;
                    Jump     = 0;
                    ALUOp    = ALU_SUB;
                end
                OP_BNE: begin
                    RegDst   = 0;
                    ALUSrc   = 0;
                    MemtoReg = 0;
                    RegWrite = 0;
                    MemRead  = 0;
                    MemWrite = 0;
                    Branch   = 1;
                    Jump     = 0;
                    ALUOp    = ALU_SUB;
                end
                OP_ADDI: begin
                    RegDst   = 0;
                    ALUSrc   = 1;
                    MemtoReg = 0;
                    RegWrite = 1;
                    MemRead  = 0;
                    MemWrite = 0;
                    Branch   = 0;
                    Jump     = 0;
                    ALUOp    = ALU_ADD;
                end
                OP_ANDI: begin
                    RegDst   = 0;
                    ALUSrc   = 1;
                    MemtoReg = 0;
                    RegWrite = 1;
                    MemRead  = 0;
                    MemWrite = 0;
                    Branch   = 0;
                    Jump     = 0;
                    ALUOp    = ALU_AND;
                end
                OP_ORI: begin
                    RegDst   = 0;
                    ALUSrc   = 1;
                    MemtoReg = 0;
                    RegWrite = 1;
                    MemRead  = 0;
                    MemWrite = 0;
                    Branch   = 0;
                    Jump     = 0;
                    ALUOp    = ALU_OR;
                end
                OP_SLTI: begin
                    RegDst   = 0;
                    ALUSrc   = 1;
                    MemtoReg = 0;
                    RegWrite = 1;
                    MemRead  = 0;
                    MemWrite = 0;
                    Branch   = 0;
                    Jump     = 0;
                    ALUOp    = ALU_SLT;
                end
                OP_J: begin
                    Jump     = 1;
                end
                OP_JAL: begin
                    RegWrite = 1; // Write return address
                    Jump     = 1;
                end
                default: begin
                    RegDst   = 0;
                    ALUSrc   = 0;
                    MemtoReg = 0;
                    RegWrite = 0;
                    MemRead  = 0;
                    MemWrite = 0;
                    Branch   = 0;
                    Jump     = 0;
                    ALUOp    = ALU_NOP;
                end
            endcase
        end
    end
endmodule

module Forwarding_Unit (
    input wire [4:0] EX_Rs,        // Source register 1 in EX stage
    input wire [4:0] EX_Rt,        // Source register 2 in EX stage
    input wire [4:0] MEM_RegDst,   // Destination register in MEM stage
    input wire [4:0] WB_RegDst,    // Destination register in WB stage
    input wire MEM_RegWrite,       // Write enable in MEM stage
    input wire WB_RegWrite,        // Write enable in WB stage
    output reg [1:0] ForwardA,     // Forwarding signal for ALU input A
    output reg [1:0] ForwardB      // Forwarding signal for ALU input B
);
    always @(*) begin
        // ForwardA: EX_Rs source check
        if (MEM_RegWrite && (MEM_RegDst != 0) && (MEM_RegDst == EX_Rs))
            ForwardA = 2'b10; // Forward from MEM stage
        else if (WB_RegWrite && (WB_RegDst != 0) && (WB_RegDst == EX_Rs))
            ForwardA = 2'b01; // Forward from WB stage
        else
            ForwardA = 2'b00; // No forwarding

        // ForwardB: EX_Rt source check
        if (MEM_RegWrite && (MEM_RegDst != 0) && (MEM_RegDst == EX_Rt))
            ForwardB = 2'b10; // Forward from MEM stage
        else if (WB_RegWrite && (WB_RegDst != 0) && (WB_RegDst == EX_Rt))
            ForwardB = 2'b01; // Forward from WB stage
        else
            ForwardB = 2'b00; // No forwarding
    end
endmodule

module Hazard_Detection_Unit (
    input wire [4:0] ID_Rs,        // Source register 1 in ID stage
    input wire [4:0] ID_Rt,        // Source register 2 in ID stage
    input wire [4:0] EX_Rt,        // Destination register in EX stage (used in LW)
    input wire EX_MemRead,         // Memory read signal in EX stage
    output reg PCWrite,            // Control signal to freeze PC update
    output reg IF_ID_Write,        // Control signal to freeze IF/ID pipeline register
    output reg ControlStall        // Control signal to flush control signals in ID stage
);
    always @(*) begin
        if (EX_MemRead && ((EX_Rt == ID_Rs) || (EX_Rt == ID_Rt))) begin
            // Stall condition due to LW hazard
            PCWrite = 0;           // Freeze PC
            IF_ID_Write = 0;       // Freeze IF/ID pipeline register
            ControlStall = 1;      // Flush control signals (NOP)
        end else begin
            PCWrite = 1;           // Allow PC update
            IF_ID_Write = 1;       // Allow pipeline register update
            ControlStall = 0;      // Normal control signal generation
        end
    end
endmodule

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
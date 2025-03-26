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
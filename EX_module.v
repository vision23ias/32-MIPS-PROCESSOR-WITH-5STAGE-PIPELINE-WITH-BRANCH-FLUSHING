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
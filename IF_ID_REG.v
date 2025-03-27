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

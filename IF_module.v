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
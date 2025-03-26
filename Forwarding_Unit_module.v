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
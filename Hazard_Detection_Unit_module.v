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
`timescale 1ns/1ps

module MIPS_Processor_Testbench;
    // Clock and Reset
    reg clk;
    reg reset;
    
    // Instantiate the MIPS Processor
    MIPS_Processor uut (
        .clk(clk),
        .reset(reset),
        .PC_out(),
        .Instruction_out(),
        .ALUResult_out(),
        .WriteData_out(),
        .RegWrite_out()
    );

    // Clock Generation (100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period (5ns high, 5ns low)
    end

    // Main Test Sequence
    initial begin
        // Initialize waveform dumping
        $dumpfile("mips_processor.vcd");
        $dumpvars(0, MIPS_Processor_Testbench);
        
        $display("=== MIPS Processor Testbench Started ===");
        $display("Time\tPC\t\tInstruction\tALU Result");
        $display("-----------------------------------------------");
        
        // Apply and release reset
        reset = 1;
        #20; // Hold reset for 20ns
        reset = 0;
        
        // Run simulation for 100 clock cycles (1000ns)
        #1000;
        
        // Display final state
        display_final_state();
        
        $display("\n=== Simulation Complete ===");
        $finish;
    end

    // Pipeline Monitor
    always @(posedge clk) begin
        if (!reset) begin // Only monitor after reset
            $display("%0t\t%h\t%h\t%h", 
                    $time,
                    uut.PC,
                    uut.IF_ID_Instruction_out,
                    uut.EX_MEM_ALUResult_out);
        end
    end

    // Helper task to print final state
    task display_final_state;
        begin
            $display("\n=== Final Processor State ===");
            
            // Register File
            $display("\nRegister File (Non-Zero Values):");
            for (integer i = 0; i < 32; i = i + 1) begin
                if (uut.ID.RegFile[i] != 0)
                    $display("R[%0d] = %h", i, uut.ID.RegFile[i]);
            end
            
            // Data Memory
            $display("\nData Memory (Non-Zero Values):");
            for (integer j = 0; j < 32; j = j + 1) begin
                if (uut.MEM.DMEM[j] != 0)
                    $display("MEM[%0d] = %h", j, uut.MEM.DMEM[j]);
            end
        end
    endtask
endmodule 
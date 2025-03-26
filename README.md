# 32-MIPS-PROCESSOR-WITH-5STAGE-PIPELINE-WITH-BRANCH-FLUSHING
Project Overview:-

This is the implementation of a 32 bit MIPS Processor based on a five-stage instruction pipeline design in verilog. The aims to carry out an instruction pipeline through the standard IF (Instruction Fetch), ID (Instruction Decode), EX (Execute), MEM (Memory Access), and WB (Write Back) forms. Effectively, branch flushing is implemented to deal with the control hazards in the pipeline. 

The processor is FPGA-yard and is synthesized via Vivado. It has all the MIPS instructions and executes them efficiently through pipelining as well as handling other pipeline hazards using hazard detection, forwarding, and control signal schemes. 

Stages of Pipelines: 
1. Instruction Fetch
The instruction fetch will use the Program Counter to fetch instructions in memory. The instruction will load in the pipeline register to Instruction Decode (ID) stage next, while the next value of the Program Counter for fetching future instructions is incremented.

2. Instruction Decode (ID): Decode the Instruction that has been
The fetching of the decoded instructions and control signals from reading operands from the Register File and sending them with the control signals to the Execute (EX) stage. This stage also has the stall pipeline feature, which checks for hazards.

3. Execute
The ALU performs operations depending on which class of instruction it belongs to while also calculating addresses of branches and if they will be taken forward or not. The forwarding logic is done here to mitigate data hazards.

5. Memory Access (MEM)
The Data Memory is read for load/store instructions in this stage. In other words, it is passed through the register for all the other issues with the memory hazards at this stage.

7. Write Back
The last stage that gives a write to the Register file, actually forming part of the executed instruction. This is indeed the last in terms of execution of the pipeline.

Features:-

5-Stage Pipelining for execution as required in IF, ID, EX, MEM, WB, with pipelines for their respective registers.

Hazard Management:
Data Hazards is processed based on forwarding and stalling.
Control hazards are treated using branch flushing to avoid the wrong execution of instructions.

Branch Handling and Flushing:
The branch decision will happen in the EX stage.
The pipeline will be flushed whenever a branch is taken to prevent the execution of incorrect instructions.
Support for MIPS instruction set:
The core instruction MIPS R, I, and J types that include arithmetic, logical, memory access as well as branch operations have been covered.

Control Unit:
It generates and handles control signals for their proper executions.
Management of Pipeline stalls, forwarding, and branching decisions.

Pipeline Register:
This refers to the stages of IF/ID, ID/EX, EX/MEM, and MEM/WB registers, which are to be created for storing intermediate values between pipeline stages. 

Design Components:-

1.Top Module (MIPS_Processor.v)
The whole pipeline stages are integrated with it along with the control which is required for the data flow.

2.Instruction Fetch (IF.v)
Only fetches the instructions and keeps them in the pipeline.

3.Instruction Decode (ID.v)
It decodes the instructions and operands for processing.

4.Execute Stage (EX.v)
It carries out mathematical operations and also logical ones apart from branch computations.

5.Memory Access (MEM.v)
It deals with data memory operations.

6.Write Back (WB.v)
It actually writes back to the registers after generating appropriate results.

7.Control Unit (Control_Unit.v)
This unit is concerned with the generation of control signals during the pipeline execution. 

8.Pipeline Registers: IF_ID.v, ID_EX.v, EX_MEM.v, and MEM_WB.v 
These registers are used to store intermediate values between different stages of the pipeline process.

9.Register File: Register_File.v 
This file is needed to store and give the register values.

10.ALU: ALU.v 
It is used to perform arithmetic and logical computations.

11.Forwarding Unit: Forwarding_Unit.v 
In order to reduce stalls, this module will carry out forwarding.

12.Hazard Detection Unit: Hazard_Detection.v 
This module would detect pipeline hazards and stall accordingly.

The complete implemented detailed specification of a 32-bit MIPS pipelined processor includes branch handling, forwarding, hazard detection, and control logic made efficient. It is a resource for learning computer architecture, design in Verilog, and FPGA implementation. Testbench for the complete processor has been added for further testing.


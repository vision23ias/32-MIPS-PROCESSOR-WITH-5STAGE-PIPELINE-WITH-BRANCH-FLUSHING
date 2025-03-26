# 32-MIPS-PROCESSOR-WITH-5STAGE-PIPELINE-WITH-BRANCH-FLUSHING

## **Project Overview**  

This is the implementation of a **32-bit MIPS Processor** based on a **five-stage instruction pipeline** design in **Verilog**. The processor carries out an instruction pipeline through the standard **IF (Instruction Fetch), ID (Instruction Decode), EX (Execute), MEM (Memory Access), and WB (Write Back)** stages. **Branch flushing** is effectively implemented to deal with control hazards in the pipeline.  

The processor is **FPGA-ready** and is synthesized via **Vivado**. It supports all **MIPS instructions**, executes them efficiently through **pipelining**, and handles **pipeline hazards** using **hazard detection, forwarding, and control signal mechanisms**.  

## **Stages of the Pipeline**  

### **1. Instruction Fetch (IF)**  
The **Instruction Fetch** stage uses the **Program Counter (PC)** to fetch instructions from memory. The fetched instruction is stored in the **pipeline register** before being sent to the **Instruction Decode (ID) stage**, while the **PC** is incremented for the next instruction.  

### **2. Instruction Decode (ID)**  
The **Instruction Decode** stage decodes the fetched instruction and extracts control signals. It reads operands from the **Register File** and passes them along with control signals to the **Execute (EX) stage**. This stage also contains a **hazard detection unit** that stalls the pipeline when necessary.  

### **3. Execute (EX)**  
The **ALU (Arithmetic Logic Unit)** performs operations depending on the instruction type. This stage also computes **branch target addresses** and determines whether a branch should be taken. **Forwarding logic** is implemented here to mitigate **data hazards**.  

### **4. Memory Access (MEM)**  
For **load/store instructions**, this stage reads or writes data from the **Data Memory**. For other instruction types, data is simply passed through the **pipeline register**. **Memory hazards** are handled in this stage.  

### **5. Write Back (WB)**  
The final stage writes computed values back to the **Register File**, ensuring correct execution of instructions. This stage completes the execution pipeline.  

## **Features**  

- **5-Stage Pipelining**: Implements **IF, ID, EX, MEM, WB** stages with **pipeline registers**.  
- **Hazard Management**:  
  - **Data Hazards** are handled using **forwarding** and **stalling**.  
  - **Control Hazards** are managed through **branch flushing**.  
- **Branch Handling and Flushing**:  
  - **Branch decisions** occur in the **EX stage**.  
  - The **pipeline is flushed** whenever a branch is taken to prevent incorrect instruction execution.  
- **MIPS Instruction Set Support**:  
  - Supports **R-type, I-type, and J-type** instructions.  
  - Includes **arithmetic, logical, memory access, and branching** operations.  
- **Control Unit**:  
  - Generates and manages **control signals** for proper execution.  
  - Handles **pipeline stalls, forwarding, and branching decisions**.  
- **Pipeline Registers**:  
  - Implements **IF/ID, ID/EX, EX/MEM, and MEM/WB registers** for storing intermediate values between pipeline stages.  

## **Design Components**  

### **1. Top Module (`MIPS_Processor.v`)**  
   - Integrates all pipeline stages and manages data flow.  

### **2. Instruction Fetch (`IF.v`)**  
   - Fetches instructions from memory and passes them to the pipeline.  

### **3. Instruction Decode (`ID.v`)**  
   - Decodes instructions and prepares operands for execution.  

### **4. Execute Stage (`EX.v`)**  
   - Performs arithmetic, logical operations, and branch computations.  

### **5. Memory Access (`MEM.v`)**  
   - Handles memory read and write operations.  

### **6. Write Back (`WB.v`)**  
   - Writes results back to the **Register File**.  

### **7. Control Unit (`Control_Unit.v`)**  
   - Generates **control signals** for proper pipeline execution.  

### **8. Pipeline Registers (`IF_ID.v`, `ID_EX.v`, `EX_MEM.v`, `MEM_WB.v`)**  
   - Stores intermediate values between pipeline stages.  

### **9. Register File (`Register_File.v`)**  
   - Stores and provides register values for execution.  

### **10. ALU (`ALU.v`)**  
   - Performs **arithmetic and logical** computations.  

### **11. Forwarding Unit (`Forwarding_Unit.v`)**  
   - Implements **data forwarding** to reduce stalls.  

### **12. Hazard Detection Unit (`Hazard_Detection.v`)**  
   - Detects **pipeline hazards** and introduces **stalls** when necessary.  

---

This **32-bit MIPS pipelined processor** includes **branch handling, forwarding, hazard detection, and efficient control logic**. It serves as an excellent resource for **learning computer architecture, Verilog design, and FPGA implementation**. The project also includes a **testbench** for simulating the complete processor.


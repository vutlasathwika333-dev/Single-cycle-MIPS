// ---------------------- alu.v ----------------------
`timescale 1ns/1ps

module alu (
    input  [31:0] a,
    input  [31:0] b,
    input  [1:0]  alu_control, // 00 ADD, 01 AND, 11 SUB
    output reg [31:0] result,
    output zero
);
    // ALU is purely combinatorial logic
    always @(*) begin
        case (alu_control)
            2'b01: result = a & b;    // AND
            2'b11: result = a - b;    // SUB (for subtraction and BEQ comparison)
            default: result = a + b;  // ADD (for ADD, ADDI, LW, SW address calculation)
        endcase
    end
    
    // Zero flag for Branch Equal (BEQ) instruction
    assign zero = (result == 32'b0);
endmodule

// ---------------------- register_file.v ----------------------
`timescale 1ns/1ps

module register_file (
    input clk,
    input reg_write,
    input [4:0] rs, rt, rd,
    input [31:0] write_data,
    output reg [31:0] rs_data, rt_data
);

    reg [31:0] registers [0:31];
    integer i;
    
    // Initialization of registers
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'h0;

        // Pre-seed initial values for testing visibility
        registers[1] = 32'h0000000A;
        registers[2] = 32'h00000003;
        registers[5] = 32'h0000000F;
    end

    // Read ports (combinational read)
    always @(*) begin
        // Read Register 0 ($zero) is always zero, but reading R0 should just yield the register content
        rs_data = registers[rs];
        rt_data = registers[rt];
    end

    // Write port (sequential write on clock edge)
    always @(posedge clk) begin
        // R0 cannot be written (rd != 0 check)
        if (reg_write && rd != 0) begin
            registers[rd] <= write_data;
            $display("@%0t: R[%0d] <= %h", $time, rd, write_data);
        end
    end

endmodule

// ---------------------- instruction_memory.v ----------------------
`timescale 1ns/1ps

module instruction_memory (
    input  [31:0] addr,
    output [31:0] instr_out
);

    reg [31:0] instr_mem [0:255];
    integer i;

    // Instruction definitions (PC is byte-addressed, memory is word-addressed)
    initial begin
        // simple visible program
        instr_mem[0] = 32'h20010005; // 0x00: ADDI $1, $0, 5        // R1 = 5
        instr_mem[1] = 32'h20020007; // 0x04: ADDI $2, $0, 7        // R2 = 7
        instr_mem[2] = 32'h00221820; // 0x08: ADD  $3, $1, $2       // R3 = 5 + 7 = 12 (0xC)
        instr_mem[3] = 32'h00622022; // 0x0C: SUB  $4, $3, $2       // R4 = 12 - 7 = 5 (0x5)
        instr_mem[4] = 32'h00822824; // 0x10: AND  $5, $4, $2       // R5 = 5 & 7 = 5 (0x5)
        instr_mem[5] = 32'hAC030000; // 0x14: SW   $3, 0($0)       // Mem[0] = R3 (0xC)
        instr_mem[6] = 32'h8C060000; // 0x18: LW   $6, 0($0)       // R6 = Mem[0] (0xC)
        instr_mem[7] = 32'h10C30001; // 0x1C: BEQ  $6, $3, skip   // R6=R3 (0xC==0xC). Branch +4. Target is 0x1C+4+(4*1) = 0x24 (address 9)
        instr_mem[8] = 32'h20070099; // 0x20: ADDI $7, $0, 0x99   // (SKIPPED)
        instr_mem[9] = 32'h200800AA; // 0x24: ADDI $8, $0, 0xAA   // R8 = 0xAA
        
        for (i = 10; i < 256; i = i + 1)
            instr_mem[i] = 32'h00000000; // NOP
    end

    // Address is byte-addressed, we access words (addr[31:2] gives word address)
    assign instr_out = instr_mem[addr[9:2]];
endmodule

// ---------------------- data_memory.v ----------------------
`timescale 1ns/1ps

module data_memory (
    input clk,
    input mem_write,
    input mem_read,
    input [31:0] addr,
    input [31:0] write_data,
    output reg [31:0] read_data
);

    reg [31:0] mem [0:255];
    integer i;
    
    // Initialization
    initial begin
        for (i = 0; i < 256; i = i + 1)
            mem[i] = 32'hDEADBEEF + i; // Visible pattern in memory
        read_data = 32'b0; // Initialize read data
    end

    always @(posedge clk) begin
        if (mem_write) begin
            mem[addr[9:2]] <= write_data;
            $display("@%0t: MEM[%0d] <= %h (SW)", $time, addr[9:2], write_data);
        end
        // Read happens on the clock edge, similar to a synchronous read memory model.
        // In this single-cycle design, it's often treated as combinational, but sequential is safer.
        if (mem_read) begin
            read_data <= mem[addr[9:2]];
            $display("@%0t: Read MEM[%0d] = %h (LW)", $time, addr[9:2], mem[addr[9:2]]);
        end
    end
endmodule

// ---------------------- control_unit.v ----------------------
`timescale 1ns/1ps

module control_unit (
    input  [5:0] opcode,
    input  [5:0] funct,       // used for R-type
    output reg   RegDst,      // 0 -> rt, 1 -> rd
    output reg   ALUSrc,      // 0 -> rt_data, 1 -> immediate
    output reg   MemtoReg,    // 0 -> ALU result, 1 -> Mem data
    output reg   RegWrite,
    output reg   MemRead,
    output reg   MemWrite,
    output reg   Branch,
    output reg [1:0] ALUControl // 2'b00 -> ADD, 2'b01 -> AND, 2'b11 -> SUB
);
    // Combinational control logic
    always @(*) begin
        // defaults (No-Op)
        RegDst   = 0;
        ALUSrc   = 0;
        MemtoReg = 0;
        RegWrite = 0;
        MemRead  = 0;
        MemWrite = 0;
        Branch   = 0;
        ALUControl = 2'b00; // Default ALU op is ADD

        case (opcode)
            6'b000000: begin // R-type instructions (add, sub, and)
                RegDst   = 1; // Write to rd
                RegWrite = 1; // Write to register
                ALUSrc   = 0; // ALU second operand is rt_data
                case (funct)
                    6'b100000: ALUControl = 2'b00; // ADD
                    6'b100010: ALUControl = 2'b11; // SUB
                    6'b100100: ALUControl = 2'b01; // AND
                    default:   ALUControl = 2'b00;
                endcase
            end
            6'b100011: begin // LW (Load Word)
                RegDst   = 0;  // Write to rt
                ALUSrc = 1;  // ALU second operand is immediate
                MemtoReg = 1; // Write-back source is Memory Data
                RegWrite = 1; // Write to register
                MemRead  = 1; // Read from Memory
                ALUControl = 2'b00; // Calculate address: Base + Offset (ADD)
            end
            6'b101011: begin // SW (Store Word)
                ALUSrc = 1;   // ALU second operand is immediate
                MemWrite = 1; // Write to Memory
                ALUControl = 2'b00; // Calculate address: Base + Offset (ADD)
            end
            6'b000100: begin // BEQ (Branch on Equal)
                Branch = 1;    // Conditional branch
                ALUControl = 2'b11; // Calculate: R1 - R2 (SUB for comparison)
            end
            6'b001000: begin // ADDI (Add Immediate)
                RegDst = 0;  // Write to rt
                ALUSrc = 1;  // ALU second operand is immediate
                RegWrite = 1; // Write to register
                ALUControl = 2'b00; // ALU Op is ADD
            end
            default: begin
                // no-op defaults
            end
        endcase
    end
endmodule


// ---------------------- program_counter.v ----------------------
`timescale 1ns/1ps

module program_counter (
    input            clk,
    input            reset,
    input      [31:0] pc_in,
    output reg [31:0] pc_out
);

    // Sequential logic to update PC on clock edge or reset
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 32'b0;
            $display("@%0t: PC reset to 0", $time);
        end else begin
            pc_out <= pc_in;
        end
    end
endmodule

// ---------------------- mips_cpu.v (top-level) ----------------------
`timescale 1ns/1ps

// =============================================================
// Single-Cycle MIPS CPU (Enhanced for Testbench Visibility)
// =============================================================
module mips_cpu (
    input  clk,
    input  reset,
    // Debug / monitor outputs for testbench
    output [31:0] pc,
    output [31:0] instr,
    output [31:0] alu_result,
    output [31:0] mem_read_data
);

    // ----------------------------------
    // Internal connections
    // ----------------------------------
    wire [31:0] pc_next;
    wire [31:0] pc_plus4;
    
    // Instruction Fields
    wire [5:0] opcode = instr[31:26];
    wire [4:0] rs     = instr[25:21];
    wire [4:0] rt     = instr[20:16];
    wire [4:0] rd     = instr[15:11];
    wire [15:0] imm   = instr[15:0];
    wire [5:0] funct  = instr[5:0];

    // Register File and ALU connections
    wire [31:0] rs_data, rt_data;
    wire [31:0] alu_b;
    wire        alu_zero;
    wire [1:0]  alu_control;

    // Control signals
    wire RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch;

    // Sign-extend immediate and shift for branch address calculation
    wire [31:0] sign_ext_imm = {{16{imm[15]}}, imm};
    // Shifted immediate for branch offset (immediate is word offset)
    wire [31:0] imm_shifted  = {sign_ext_imm[29:0], 2'b00};
    // Branch target address calculation
    wire [31:0] branch_target = pc_plus4 + imm_shifted;

    // Write register selection (RegDst Mux)
    wire [4:0] write_reg = (RegDst) ? rd : rt;

    // Write-back data selection (MemtoReg Mux)
    wire [31:0] write_back_data = (MemtoReg) ? mem_read_data : alu_result;

    // ----------------------------------
    // Module Instantiations
    // ----------------------------------

    // Program Counter (PC)
    program_counter pc_reg (
        .clk(clk),
        .reset(reset),
        .pc_in(pc_next),
        .pc_out(pc)
    );

    // Instruction Memory (IMEM)
    instruction_memory imem (
        .addr(pc),
        .instr_out(instr)
    );

    // PC + 4 incrementer
    assign pc_plus4 = pc + 4;

    // Control Unit
    control_unit ctrl (
        .opcode(opcode),
        .funct(funct),
        .RegDst(RegDst),
        .ALUSrc(ALUSrc),
        .MemtoReg(MemtoReg),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .Branch(Branch),
        .ALUControl(alu_control)
    );

    // Register File (REGS)
    register_file regs (
        .clk(clk),
        .reg_write(RegWrite),
        .rs(rs),
        .rt(rt),
        .rd(write_reg),
        .write_data(write_back_data),
        .rs_data(rs_data),
        .rt_data(rt_data)
    );

    // ALU input mux (ALUSrc)
    assign alu_b = (ALUSrc) ? sign_ext_imm : rt_data;

    // ALU
    alu the_alu (
        .a(rs_data),
        .b(alu_b),
        .alu_control(alu_control),
        .result(alu_result),
        .zero(alu_zero)
    );

    // Data Memory (DMEM)
    data_memory dmem (
        .clk(clk),
        .mem_write(MemWrite),
        .mem_read(MemRead),
        .addr(alu_result), // ALU result is the memory address
        .write_data(rt_data), // Data to store comes from rt_data
        .read_data(mem_read_data)
    );

    // PC Next Mux (Branch decision logic)
    // take_branch is true only if Branch signal is set (BEQ instruction) AND the ALU zero flag is set (rs == rt)
    wire take_branch = Branch & alu_zero;
    assign pc_next = take_branch ? branch_target : pc_plus4;

endmodule

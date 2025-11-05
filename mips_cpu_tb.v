// ---------------------- mips_cpu_tb.v ----------------------
`timescale 1ns/1ps

// =============================================================
// Testbench for single-cycle MIPS CPU (Revised for robustness)
// =============================================================
module mips_cpu_tb;

    // ----------------------------------
    // Testbench signals
    // ----------------------------------
    reg clk;
    reg reset;
    wire [31:0] pc, instr, alu_result, mem_read_data;


    // Instantiate the DUT (Device Under Test)
   mips_cpu uut (
        .clk(clk),
        .reset(reset),
        .pc(pc),
        .instr(instr),
        .alu_result(alu_result),
        .mem_read_data(mem_read_data)
    );


    // ----------------------------------
    // Clock generation (10 ns period)
    // ----------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100 MHz clock
    end

    // ----------------------------------
    // Simulation control and logging
    // ----------------------------------
    initial begin
        $dumpfile("mips_cpu_tb.vcd");
        $dumpvars(0, mips_cpu_tb);

        // Reset sequence
        $display(">> Starting Simulation and Reset");
        reset = 1;
        #15; // Hold reset for 1.5 cycles
        reset = 0;
        $display(">> Reset complete, CPU running.");

        // Monitor CPU state on every clock edge
        $monitor("@%0t: PC=%h, Instr=%h, ALU_R=%h, Mem_D=%h, R/W=%b, M/R=%b, M/W=%b", 
                 $time, pc, instr, alu_result, mem_read_data, 
                 uut.RegWrite, uut.MemRead, uut.MemWrite);
        
        // Wait for execution to complete. 
        // 10 instructions * 10ns/cycle = 100ns of execution + setup. 
        // Running for 600ns (60 cycles) guarantees all writes are finalized.
        #600; 

        $display("\n========= Simulation Finished at %0t ns =========", $time);
        $display("Register file contents (Expected R1=5, R2=7, R3=C, R4=5, R5=5, R6=C, R8=AA):");
        show_registers;
        $display("\nData memory contents (Expected MEM[0]=0000000C):");
        show_data_memory;

        $finish;
    end

    // ----------------------------------
    // Helper tasks for visibility
    // ----------------------------------
    task show_registers;
        integer i;
        begin
            // Check only the registers used in the test program
            for (i = 0; i < 10; i = i + 1) begin
                // Hierarchical reference to register file's internal array
                $display("R[%0d] = %h", i, uut.regs.registers[i]);
            end
        end
    endtask

    task show_data_memory;
        integer j;
        begin
            for (j = 0; j < 10; j = j + 1) begin
                // Hierarchical reference to data memory's internal array
                $display("MEM[%0d] = %h", j, uut.dmem.mem[j]);
            end
        end
    endtask

endmodule

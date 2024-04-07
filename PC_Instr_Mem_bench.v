`include "PF3-Control_unit.v"

module testbench;
    // Declare testbench signals
    reg clk;
    reg rst;
    reg en;
    wire [31:0] pc_out;
    wire [31:0] instruction;
    
    // Instantiate the PC register
    pc_reg pc(
        .clk(clk),
        .rst(rst),
        .en(en),
        .in(pc_out + 4),  // Increment PC by 4
        .out(pc_out)
    );
    
    // Instantiate the instruction memory
    instruction_memory imem(
        .address(pc_out[8:0]), // Assume that the PC is word-aligned and mapped directly to instruction address
        .instruction(instruction)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // Generate a clock with a period of 10 ns
    end
    
    // Test sequence
    initial begin
        rst = 1; en = 0; // Start with reset asserted and enable low
        #10;             // Wait for 10 ns
        rst = 0; en = 1; // Deassert reset and enable PC
        
        // Observe the behavior for several clock cycles
        #100;
        
        // Hold reset for some time to check reset behavior
        rst = 1;
        #20;
        rst = 0;
        
        // Finish the simulation
        #100;
        $finish;
    end
    
    // Monitor signals during the test
    initial begin
        $monitor("Time=%g, Reset=%b, Enable=%b, PC=%h, Instruction=%h",
                  $time, rst, en, pc_out, instruction);
    end
    
    // Optionally load instruction memory with the predefined content
    initial begin
        // Assuming your test-code.txt is in the current directory
        $readmemb("C:/Users/jay20/Documents/RISCV_Architecture-/test-code.txt", imem.mem);
    end
endmodule

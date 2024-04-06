/*****Instruction Memory Module - ROM*****/

module instruction_memory(
    input [8:0] address, // 9 bits address for the input
    output [31:0] instruction // 32 bits output.
    );

    reg [7:0] mem[511:0];
    
    //Reading the preload memory
    //If this is not working specified the whole directory of the file.
    initial begin
        $readmemb("test-code.txt", mem, 0, 15);
    end 
    
    //Making the arragment for the instruction
    assign instruction = {mem[address + 3], mem[address + 2], mem[address + 1], mem[address]};
    
endmodule

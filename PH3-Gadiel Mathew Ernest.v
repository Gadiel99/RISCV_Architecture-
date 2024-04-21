/* Control Unit simulation by:
    -Gadiel J. De Jesus Martinez 
    -Mathew Mendoza Rivas
    -Ernest W. Annable Gonzalez  

*/


/*****Instruction Memory Module - ROM*****/
module instruction_memory(
    input [8:0] address, // 9 bits address for the input
    output [31:0] instruction // 32 bits output.
    );

    reg [7:0] mem[511:0];
    
    //Reading the preload memory
    //If this is not working specified the whole directory of the file.
    initial begin
      $readmemb("C:/Users/jay20/Documents/RISCV_Architecture-/test-code.txt", mem, 0, 511);
    end 
    
    //Making the arragment for the instruction
    assign instruction = {mem[address + 3], mem[address + 2], mem[address + 1], mem[address]};  
endmodule


/*****Program Counter Module*****/
module pc_reg ( input wire clk,
                input wire reset,
                input wire en,
                input wire [31:0] in,
                output reg [31:0] out
);

    always@(posedge clk) begin
        if (reset) out <= 32'b0;
        else if (en) out <= in;
    end
endmodule

<<<<<<< HEAD:PH3-Gadiel Mathew Ernest.v
// // Control unit MUX
// module control_signals_mux #(
//     parameter NUM_CONTROL_SIGNALS = 10 // Here goes the quantity of signals
// )(
//     input wire s,                       
//     input wire [NUM_CONTROL_SIGNALS-1:0] in_0, 
//     input wire [NUM_CONTROL_SIGNALS-1:0] in_1, 
//     output wire [NUM_CONTROL_SIGNALS-1:0] out 
// );

//     // Makes the selection of the output signal.
//     assign out = s ? in_1 : in_0;
=======
module ALU(
    input [31:0] A,
    input [31:0] B,
    input [3:0] Op,
    output reg [31:0] Out,
    output reg Z,
    output reg N,
    output reg C,
    output reg V
    );
    always @ (A, B, Op)
    begin

        case (Op)
            4'b0000: Out = B; // Pass through B
            4'b0001: Out = B + 4; // B + 4
            4'b0010: begin // A + B
                {C, Out} = A + B; // Addition with carry out
                Z = (Out == 0); // Zero flag
                N = Out[31]; // Negative flag
    // Overflow flag for addition
                V = ~(A[31] ^ B[31]) & (A[31] ^ Out[31]);
            end

            4'b0011: begin // A - B
                 Out = A - B; 
              	 C = A < B;
                 Z = (Out == 0); // Zero flag
                 N = Out[31]; // Negative flag
    // Overflow flag for subtraction
                 V = (A[31] ^ B[31]) & (A[31] ^ Out[31]);
    
            end

            4'b0100: Out = (A + B) & 32'hFFFFFFFE; // (A + B) AND with mask for even number
            4'b0101: Out = A << B[4:0]; // Logical shift left A by the amount specified in the lower 5 bits of B
            4'b0110: Out = A >> B[4:0]; // Logical shift right A by the amount specified in the lower 5 bits of B
            4'b0111: Out = $signed(A) >>> B[4:0]; // Arithmetic shift right A by the amount specified in the lower 5 bits of B
            4'b1000: begin // if (A < B) then Out=1, else Out=0 for signed numbers
                Out = ($signed(A) < $signed(B)) ? 1 : 0;
                Z = (Out == 0);
                N = 0; // Since Out will only be 1 or 0, it's never negative.
                // V is not relevant for comparison, and there's no need to set it here.
            end

            4'b1001: begin // Set Out to 1 if A < B for unsigned numbers
            Out = (A < B) ? 1 : 0;
            Z = (Out == 0);
            // N and V are not relevant for unsigned comparison, and C is not applicable here as it's not a subtraction.
            end

            4'b1010: Out = A & B; // Bitwise AND
            4'b1011: Out = A | B; // Bitwise OR
            4'b1100: Out = A ^ B; // Bitwise XOR
            default: Out = 0; // For unused opcodes or default
        endcase
    end


 endmodule

module SecondOperandHandler(
    input [31:0] PB,
    input [11:0] imm12_I,
    input [11:0] imm12_S,
    input [19:0] imm20,
    input [31:0] PC,
    input [2:0] S, 
    output reg [31:0] N
);
  always @(*) begin
        case(S)
            3'b000: N = PB;
            3'b001: N = {{20{imm12_I[11]}}, imm12_I};
            3'b010: N = {{20{imm12_S[11]}}, imm12_S};
            3'b011: N = {imm20, 12'b0};
            3'b100: N = PC;
            default: N = 32'b0; // For 'not used' cases and default
        endcase
    end
>>>>>>> 85f94553042afb5d723c320975ab493001e2cabe:PF3-Gadiel Mathew Ernest.v

// endmodule

<<<<<<< HEAD:PH3-Gadiel Mathew Ernest.v

/*****Control Unit Module*****/
=======
//Control unit module
>>>>>>> 85f94553042afb5d723c320975ab493001e2cabe:PF3-Gadiel Mathew Ernest.v
module control_unit(input wire [31:0] instruction,
    output reg [3:0] id_alu_op, 
    output reg [2:0] id_shifter_imm,
    output reg id_rf_enable, 
    output reg id_load_inst, 
    output reg id_mem_ins_enable, 
    output reg id_mem_write, 
    output reg [1:0] size,
    output reg id_se,
    output reg [9:0] id_full_cond,
    output reg id_jalr_sig,
    output reg id_auipc_s,
    output reg id_jal_sig,
    output reg add_sub_sign,
                    output reg [2:0] func3);
    //Decode logic begins here
    
    always @(instruction)begin
        id_alu_op = 0;
        id_shifter_imm = 0;
        id_rf_enable = 0;
        id_load_inst = 0;
        id_mem_ins_enable = 0;
        id_mem_write = 0;
        size = 0;
        id_se = 0; 
        id_full_cond = 0;
        id_jalr_sig = 0;
        id_auipc_s = 0;
        id_jal_sig = 0;
        func3 = instruction[14:12];
        
        if(instruction !=0) begin
            case(instruction[6:0]) // Check the opcode
                7'b0110011: begin // R-Type
                    // Set control signals for R-Type instruction
                    id_rf_enable = 1;
                    
                    case(func3)
                    

                        3'b000: begin
                            add_sub_sign = instruction[30];

                            case(add_sub_sign)
                                1: begin // SUB case
                                    id_alu_op = 4'b0011;
                                   
                                    $display("SUB");

                                end

                                0: begin // ADD case
                                   id_alu_op = 4'b0010;
                                    $display("ADD");

                                end
                            endcase
                        end
                        3'b101: begin
                            add_sub_sign = instruction[30];

                            case(add_sub_sign)
                                1: begin // SUB case
                                  
                                  id_alu_op = 4'b0110;
                                    $display("SRA");

                                end

                                0: begin // ADD case
                                  
                                  id_alu_op = 4'b0110;
                                    $display("SRL");

                                end
                            endcase
                        end

                        3'b010: begin
                          id_alu_op = 4'b1000;
                            $display("SLT");
                        end

                        3'b011: begin
                          id_alu_op = 4'b1001;
                            $display("SLTU");
                        end

                        3'b111: begin
                           id_alu_op = 4'b1010;
                            $display("AND");
                        end

                        3'b110: begin
                          id_alu_op = 4'b1011;
                            $display("OR");
                        end

                        3'b100: begin
                          id_alu_op = 4'b1100;
                            $display("XOR");
                        end

                        3'b001: begin
                            id_alu_op = 4'b0101;
                            $display("SLL");
                        end

                    endcase
                end
            
                7'b0010011: begin // I-Type (could also include other opcodes for I-Type instructions)
                    // Set control signals for I-Type instruction

                    id_alu_op = 1;
                    id_shifter_imm = 3'b001;
                    id_rf_enable = 1;

                    case(func3)

                        3'b000: begin
                           id_alu_op = 4'b0010;
                           
                            $display("ADDI");
                        end

                        3'b010: begin
                            id_alu_op = 4'b1000;
                          
                            $display("SLTI");
                        end

                        3'b011: begin
                            id_alu_op = 4'b1001;
                           
                            $display("SLTIU");
                        end
                        
                        3'b111: begin
                            id_alu_op = 4'b1010;
                           
                            $display("ANDI");
                        end

                        3'b110: begin
                            id_alu_op = 4'b1011;
                          
                            $display("ORI");
                        end

                        3'b100: begin
                            id_alu_op = 4'b1100;
                           
                            $display("XORI");
                        end

                        3'b001: begin
                            id_alu_op = 4'b0101;
                            
                            $display("SLLI");
                        end

                        3'b101: begin
                            add_sub_sign = instruction[30];

                            case(add_sub_sign)
                                1: begin // SRAI case
                                   id_alu_op = 4'b0111;
                                
                                    $display("SRAI");

                                end

                                0: begin // SRLI case
                                  id_alu_op = 4'b0110;
                                    $display("SRLI");

                                end
                            endcase
                        end
                        
                    endcase
                end

                7'b1100111: begin // jalr
                    // Set control signals for jalr instruction
                    id_alu_op = 4'b0100;
                    id_shifter_imm = 3'b001;
                    id_rf_enable = 1;
                    id_jalr_sig = 1;
                    $display("JALR");
                    
                end 

                7'b0000011: begin // I-Type
                    // Set control signals for Load Instructions instruction
                    id_alu_op = 4'b0010;
                    id_shifter_imm = 3'b001;
                    id_rf_enable = 1;
                    id_load_inst = 1;
                    id_mem_ins_enable = 1;
                    //mem_set_inst = 0;

                    case(func3)
                        3'b010:begin
                            size = 2'b10; //LW
                          
                            $display("LW");
                        end
                        3'b001:begin
                            size = 2'b01; //LH
                            id_se = 1;
                            $display("LH");
                        end
                        3'b101:begin
                            size = 2'b01; //LHU
                            $display("LHU");
                        end
                        3'b000:begin
                            size = 2'b00; //LB
                            id_se = 1;
                            //SE igual a 1 
                            $display("LB");
                        end
                        3'b100:begin
                            size = 2'b00; //LBU
                            $display("LBU");
                        end
                    endcase
                end

                7'b0100011: begin // S-Type
                    // Set control signals for S-Type instruction
                    id_alu_op = 4'b0010;
                    id_shifter_imm = 3'b010;
                    id_mem_ins_enable = 1;
                    id_mem_write = 1;
                    case(func3)
                        3'b000: begin 
                            size <= 2'b00; // SB instruction
                            $display("SB");
                        end
                        3'b001: begin
                            size <= 2'b01; // SH instruction
                            $display("SH");
                        end
                        3'b010: begin
                            size <= 2'b10; // SW instruction
                            $display("SW");
                        end
                    endcase
                end

                7'b1100011: begin // B-Type
                    // Set control signals for B-Type instruction
                    // If it's a branch instruction, combine the opcode and funct3

                    id_full_cond <= {instruction[6:0], instruction[14:12]};
                    id_shifter_imm = 0;
                    case(func3)

                         3'b000: begin
                           id_alu_op = 4'b0011;
                           
                            $display("BEQ");
                        end

                         3'b001: begin
                           id_alu_op = 4'b0011;
                            $display("BNE");
                        end

                         3'b100: begin
                           id_alu_op = 4'b1000;
                            $display("BLT");
                        end
                        
                         3'b101: begin
                           id_alu_op = 4'b1000;
                            $display("BGE");
                        end

                         3'b110: begin
                           id_alu_op = 4'b1001;
                            $display("BLTU");
                        end

                         3'b111: begin
                            id_alu_op = 4'b1001;
                            $display("BGEU");
                        end
                    endcase 
                end
                7'b0110111: begin // U-Type (lui)
                    // Set control signals for U-Type instruction
                    //TODO: special case
                    id_alu_op = 4'b0000;
                    id_shifter_imm = 3'b011;
                    id_rf_enable = 1;
                    $display("LUI");
                end 
                
                
                7'b0010111: begin // U-Type (auipc)
                    // Set control signals for U-Type instruction
                    //TODO: special case
                    id_alu_op = 4'b0010;
                    id_shifter_imm = 3'b011;
                    id_rf_enable = 1;
                    id_auipc_s = 1;
                    $display("AUIPC");
                end
                7'b1101111: begin // J-Type
                    // Set control signals for J-Type instruction
                    //TODO: special case
                    id_rf_enable = 1;
                    id_jal_sig = 1;
                    $display("JAL");
                end
                default: begin
                    // Handle undefined opcode
                    $display("Undefined opcode");

                end
            endcase
        end else begin
                $display("NOP");
            end
    end
endmodule

/*****IF/ID Pipeline Register*****/
module IF_ID_pipeline_register( output reg [31:0] instruction, ID_PC,
                                input  clk, reset,IF_ID_LOAD,
                                input [31:0] ins_mem_out, PC);

    always@(posedge clk)
    begin

        if(reset==1 ) begin
            $display("-------------NOP IF/ID--------------");
            
            instruction <= 32'b0;
            ID_PC <= 32'b0;
        end 
        else begin
            if (IF_ID_LOAD == 1) begin 
            instruction <= ins_mem_out;
            ID_PC <= PC;
        end 
    end
        
    end

endmodule

/*****ID/EX Pipeline Register*****/
module ID_EX_pipeline_register( input wire clk, 
    input wire reset,
    input wire [3:0] id_alu_op_mux, 
    input wire [2:0] id_shifter_imm_mux,
    input wire id_rf_enable_mux, 
    input wire id_load_inst_mux, 
    input wire id_mem_ins_enable_mux, 
    input wire id_mem_write_mux, 
    input wire [1:0] size_mux,
    input wire id_se_mux,
    input wire [9:0] id_full_cond_mux,
    input wire id_jalr_sig_mux,
    input wire id_auipc_s_mux,
    input wire id_jal_sig_mux,
    output reg ex_rf_enable,
    output reg [3:0] ex_alu_op,
    output reg [2:0] ex_shifter_imm,
    output reg ex_load_inst,
    output reg ex_mem_ins_enable,
    output reg ex_mem_write, 
    output reg [1:0] ex_size,
    output reg ex_se,
    output reg [9:0] ex_full_cond,
    output reg ex_jalr_sig,
    output reg ex_auipc_s,
    output reg ex_jal_sig
);

    always@(posedge clk)
    begin
        
        if(reset==1) begin
            $display("-------------NOP ID/EXE--------------");
            ex_rf_enable <= 1'b0;
            ex_alu_op <= 4'b0;
            ex_shifter_imm <= 3'b0;
            ex_load_inst <= 1'b0;
            ex_mem_ins_enable <= 1'b0;
            ex_mem_write <= 1'b0;
            ex_size <= 2'b0;
            ex_se <= 2'b0;
            ex_full_cond <= 10'b0;
            ex_jalr_sig <= 1'b0;
            ex_auipc_s <= 1'b0;
            ex_jal_sig <= 1'b0;

        end else begin
        //Control Unit signals  
            ex_rf_enable <= id_rf_enable_mux;
            ex_alu_op <= id_alu_op_mux;
            ex_shifter_imm <= id_shifter_imm_mux;
            ex_load_inst <= id_load_inst_mux;
            ex_mem_ins_enable <= id_mem_ins_enable_mux;
            ex_mem_write <= id_mem_write_mux;
            ex_size <= size_mux;
            ex_se <= id_se_mux;
            ex_full_cond <= id_full_cond_mux;
            ex_jalr_sig <= id_jalr_sig_mux;
            ex_auipc_s <= id_auipc_s_mux;
            ex_jal_sig <= id_jal_sig_mux;
        end
    end
   
endmodule

/*****EX/MEM Pipeline Register*****/
module EX_MEM_pipeline_register(     input wire clk, 
    input wire reset,
    input wire s,
    input wire ex_rf_enable,
    input wire ex_load_inst,
    input wire ex_mem_ins_enable,
    input wire ex_mem_write, 
    input wire [1:0] ex_size,
    input wire ex_se,
    output reg mem_rf_enable,
    output reg mem_load_inst,
    output reg mem_mem_ins_enable,
    output reg mem_mem_write, 
    output reg [1:0] mem_size,
    output reg mem_se
    );

    always@(posedge clk)
    begin
        
        if(reset==1) begin
            $display("-------------NOP EXE/MEM--------------");
            mem_rf_enable <= 1'b0;
            mem_load_inst <= 1'b0;
            mem_mem_ins_enable <= 1'b0;
            mem_mem_write <= 1'b0;
            mem_size <= 2'b0;
            mem_se <= 1'b0;

        end else begin
        //Control Unit signals  
            mem_rf_enable <= ex_rf_enable;
            mem_load_inst <= ex_load_inst;
            mem_mem_ins_enable <= ex_mem_ins_enable;
            mem_mem_write <= ex_mem_write;
            mem_size <= ex_size;
            mem_se <= ex_se;            
        end
    end
   
endmodule

/*****MEM/WB Pipeline Register*****/
module MEM_WB_pipeline_register(    
                                    output reg wb_rf_enable,

                                    input wire clk, reset, s, mem_rf_enable);

    always@(posedge clk)
    begin
        
        if(reset == 1) begin
            $display("-------------NOP MEM/WB--------------");
            wb_rf_enable <= 1'b0;

        end else begin
        //Control Unit signals  
            wb_rf_enable <= mem_rf_enable;
        end
    end
   
endmodule

/*****PC adder*****/
module Adder(
    output reg [31:0] pcplus4,
    input [31:0] pc
);
    always @(*) begin
        pcplus4 = pc + 4;
    end
endmodule

/*****Control Unit MUX Module*****/
module CUMux (
    input wire s,
    input wire [3:0] id_alu_op, 
    input wire [2:0] id_shifter_imm,
    input wire id_rf_enable, 
    input wire id_load_inst, 
    input wire id_mem_ins_enable, 
    input wire id_mem_write, 
    input wire [1:0] size,
    input wire id_se,
    input wire [9:0] id_full_cond,
    input wire id_jalr_sig,
    input wire id_auipc_s,
    input wire id_jal_sig,
    output reg id_rf_enable_mux,
    output reg [3:0] id_alu_op_mux,
    output reg [2:0] id_shifter_imm_mux,
    output reg id_load_inst_mux,
    output reg id_mem_ins_enable_mux,
    output reg id_mem_write_mux, 
    output reg [1:0] size_mux,
    output reg id_se_mux,
    output reg [9:0] id_full_cond_mux,
    output reg id_jalr_sig_mux,
    output reg id_auipc_s_mux,
    output reg id_jal_sig_mux
);
 always@* begin
        
        if(s==1) begin
            $display("-------------NOP ID/EXE--------------");
            id_rf_enable_mux <= 1'b0;
            id_alu_op_mux <= 4'b0;
            id_shifter_imm_mux <= 3'b0;
            id_load_inst_mux <= 1'b0;
            id_mem_ins_enable_mux <= 1'b0;
            id_mem_write_mux <= 1'b0;
            size_mux <= 2'b0;
            id_se_mux <= 2'b0;
            id_full_cond_mux <= 10'b0;
            id_jalr_sig_mux <= 1'b0;
            id_auipc_s_mux <= 1'b0;
            id_jal_sig_mux <= 1'b0;

        end else begin
        //Control Unit signals  
            id_rf_enable_mux <= id_rf_enable;
            id_alu_op_mux <= id_alu_op;
            id_shifter_imm_mux <= id_shifter_imm;
            id_load_inst_mux <= id_load_inst;
            id_mem_ins_enable_mux <= id_mem_ins_enable;
            id_mem_write_mux <= id_mem_write;
            size_mux <= size;
            id_se_mux <= id_se;
            id_full_cond_mux <= id_full_cond;
            id_jalr_sig_mux <= id_jalr_sig;
            id_auipc_s_mux <= id_auipc_s;
            id_jal_sig_mux <= id_jal_sig;
        end
    end

endmodule

module processor(
    input wire clk,
    input wire reset,
    input wire s
);

    // Internal signals
    wire [31:0] pc_current, pc_next, instruction, id_pc;

    //alu_op
    wire [3:0] id_alu_op, ex_alu_op, id_alu_op_mux;

    //shifter_imm
    wire [2:0] id_shifter_imm, ex_shifter_imm, id_shifter_imm_mux;
    
    //rf_enable
    wire id_rf_enable, ex_rf_enable, mem_rf_enable, wb_rf_enable, id_rf_enable_mux;

    //load_inst
    wire id_load_inst, ex_load_inst, mem_load_inst, id_load_inst_mux;

    //mem_ins_enable
    wire id_mem_ins_enable, ex_mem_ins_enable, mem_mem_ins_enable, id_mem_ins_enable_mux;

    //mem_write_enable
    wire id_mem_write, ex_mem_write, mem_mem_write, id_mem_write_mux;

    //size
    wire [1:0] size, ex_size, mem_size, size_mux;

    //se
    wire id_se, ex_se, mem_se, id_se_mux;

    //full_cond
    wire [9:0] id_full_cond, ex_full_cond, id_full_cond_mux;

    //jalr_sig
    wire id_jalr_sig, ex_jalr_sig, id_jalr_sig_mux;

    //auipc_s
    wire id_auipc_s, ex_auipc_s, id_auipc_s_mux;
    wire id_jal_sig, ex_jal_sig, id_jal_sig_mux;

    //add_sub_sign
    wire add_sub_sign;
    
    //funct3 
    wire [2:0] func3;
    
    //IF_ID_LOAD
    wire IF_ID_LOAD = 1'b1; // Assuming always enabled for this phase
    
    //ins_mem_out && ID_EX_pc_next, EX_MEM_pc_next, MEM_WB_pc_next
    wire [31:0] ins_mem_out, ID_EX_pc_next, EX_MEM_pc_next, MEM_WB_pc_next;

    //s signal for NOP at CU Mux
    //wire s;

    // PC Reg
    pc_reg pc_reg_inst(
        .clk(clk),
        .reset(reset),
        .en(1'b1),
        .in(pc_next),
        .out(pc_current)
    );

    // Instruction Memory
    instruction_memory instruction_memory_inst(
        .address(pc_current[8:0]),
        .instruction(instruction)
    );

    // IF/ID Pipeline Register
    IF_ID_pipeline_register IF_ID_pipeline_register_inst(
        .clk(clk),
        .reset(reset),
        .IF_ID_LOAD(IF_ID_LOAD),
        .ins_mem_out(instruction),
        .PC(pc_current),
        .instruction(ins_mem_out),
        .ID_PC(id_pc)
    );

    // Control Unit
    control_unit control_unit_inst(
        .instruction(ins_mem_out),
        .id_alu_op(id_alu_op),
        .id_shifter_imm(id_shifter_imm),
        .id_rf_enable(id_rf_enable),
        .id_load_inst(id_load_inst),
        .id_mem_ins_enable(id_mem_ins_enable),
        .id_mem_write(id_mem_write),
        .size(size),
        .id_se(id_se),
        .id_full_cond(id_full_cond),
        .id_jalr_sig(id_jalr_sig),
        .id_auipc_s(id_auipc_s),
        .id_jal_sig(id_jal_sig),
        .add_sub_sign(add_sub_sign),
        .func3(func3)
    );

    CUMux CUMux_inst(
        .s(s),
        .id_alu_op(id_alu_op),
        .id_shifter_imm(id_shifter_imm),
        .id_rf_enable(id_rf_enable),
        .id_load_inst(id_load_inst),
        .id_mem_ins_enable(id_mem_ins_enable),
        .id_mem_write(id_mem_write),
        .size(size),
        .id_se(id_se),
        .id_full_cond(id_full_cond),
        .id_jalr_sig(id_jalr_sig),
        .id_auipc_s(id_auipc_s),
        .id_jal_sig(id_jal_sig),
        .id_rf_enable_mux(id_rf_enable_mux),
        .id_alu_op_mux(id_alu_op_mux),
        .id_shifter_imm_mux(id_shifter_imm_mux),
        .id_load_inst_mux(id_load_inst_mux),
        .id_mem_ins_enable_mux(id_mem_ins_enable_mux),
        .id_mem_write_mux(id_mem_write_mux),
        .size_mux(size_mux),
        .id_se_mux(id_se_mux),
        .id_full_cond_mux(id_full_cond_mux),
        .id_jalr_sig_mux(id_jalr_sig_mux),
        .id_auipc_s_mux(id_auipc_s_mux),
        .id_jal_sig_mux(id_jal_sig_mux)
    );
    // ID/EX Pipeline Register
    ID_EX_pipeline_register ID_EX_pipeline_register_inst(
        .clk(clk),
        .reset(reset),
        .id_alu_op_mux(id_alu_op_mux),
        .id_shifter_imm_mux(id_shifter_imm_mux),
        .id_rf_enable_mux(id_rf_enable_mux),
        .id_load_inst_mux(id_load_inst_mux),
        .id_mem_ins_enable_mux(id_mem_ins_enable_mux),
        .id_mem_write_mux(id_mem_write_mux),
        .size_mux(size_mux),
        .id_se_mux(id_se_mux),
        .id_full_cond_mux(id_full_cond_mux),
        .id_jalr_sig_mux(id_jalr_sig_mux),
        .id_auipc_s_mux(id_auipc_s_mux),
        .id_jal_sig_mux(id_jal_sig_mux),
        .ex_rf_enable(ex_rf_enable),
        .ex_alu_op(ex_alu_op),
        .ex_shifter_imm(ex_shifter_imm),
        .ex_load_inst(ex_load_inst),
        .ex_mem_ins_enable(ex_mem_ins_enable),
        .ex_mem_write(ex_mem_write),
        .ex_size(ex_size),
        .ex_se(ex_se),
        .ex_full_cond(ex_full_cond),
        .ex_jalr_sig(ex_jalr_sig),
        .ex_auipc_s(ex_auipc_s),
        .ex_jal_sig(ex_jal_sig)
    );

    // EX/MEM Pipeline Register
    EX_MEM_pipeline_register EX_MEM_pipeline_register_inst(
        .clk(clk),
        .reset(reset),      
        .ex_rf_enable(ex_rf_enable),
        .ex_load_inst(ex_load_inst),
        .ex_mem_ins_enable(ex_mem_ins_enable),
        .ex_mem_write(ex_mem_write),
        .ex_size(ex_size),
        .ex_se(ex_se),
        .mem_rf_enable(mem_rf_enable),
        .mem_load_inst(mem_load_inst),
        .mem_mem_ins_enable(mem_mem_ins_enable),
        .mem_mem_write(mem_mem_write),
        .mem_size(mem_size),
        .mem_se(mem_se)
    );

    // MEM/WB Pipeline Register
    MEM_WB_pipeline_register MEM_WB_pipeline_register_inst(
        .clk(clk),
        .reset(reset),
        .s(s),
        .mem_rf_enable(mem_rf_enable),
        .wb_rf_enable(wb_rf_enable)
    );

    Adder pc_adder(
        .pc(pc_current),
        .pcplus4(pc_next)
    );

    // Next PC Logic (Placeholder for actual logic)
    //assign pc_next = pc_current + 4;

endmodule


module processor_testbench;

    // Inputs
    reg clk;
    reg reset;
    reg s;
    wire [3:0] id_alu_op, ex_alu_op, id_alu_op_mux;
    wire id_rf_enable, ex_rf_enable, mem_rf_enable, wb_rf_enable, id_rf_enable_mux;
    wire id_load_inst, ex_load_inst, mem_load_inst, id_load_inst_mux;
    wire id_mem_ins_enable, ex_mem_ins_enable, mem_mem_ins_enable, id_mem_ins_enable_mux;
    wire id_mem_write, ex_mem_write, mem_mem_write, id_mem_write_mux;
    wire [1:0] size, ex_size, mem_size, size_mux;
    wire id_se, ex_se, mem_se, id_se_mux;
    wire [9:0] id_full_cond, ex_full_cond, id_full_cond_mux;
    wire id_jalr_sig, ex_jalr_sig, id_jalr_sig_mux;
    wire id_auipc_s, ex_auipc_s, id_auipc_s_mux;
    wire id_jal_sig, ex_jal_sig, id_jal_sig_mux;
    wire [31:0] pc_current, instruction, id_pc;
   // wire s;

    // Instantiate the processor module
    processor uut (
        .clk(clk),
        .reset(reset),
        .s(s)
    );

  initial begin
    // Initialize Inputs
    clk = 0;     // Clk starts at 0
    reset = 1;   // Reset starts at 1
    s = 0;       // s starts at 0

    // At time 3, release reset
    #3 reset = 0;

    // At time 40, set s to 1
    #37 s = 1; // 40-3 = 37 because we wait for 3 time units before this
    
    // End the simulation at 48 time units from the start
    #8 $finish; // 48-40 = 8 because we already waited for 40 time units
end

// Generate clock with period of 4 units
always #2 clk = !clk;


    // Display the states of the control signals at each positive edge of the clock
    //Heres we utilize the $strobe function which makes the output of every cycle.
    always @(posedge clk) begin

        $strobe("\nTime: %t \nPC:%d \nInstruction Fetched: %b \nS: %b", $time,uut.pc_reg_inst.out, uut.instruction_memory_inst.instruction, s);
        
       
        
        $strobe("\n| ID Signals: RF En %b, ALU Op %b, SOH %b, Load Inst %b, Mem Ins En %b, MemWrite %b, Size %b, SE %b Full Cond %b, JALR Sig %b, AUIPC S %b, JAL Sig %b |", 
                uut.CUMux_inst.id_rf_enable_mux, uut.CUMux_inst.id_alu_op_mux, uut.CUMux_inst.id_shifter_imm_mux, uut.CUMux_inst.id_load_inst_mux,
                uut.CUMux_inst.id_mem_ins_enable_mux, uut.CUMux_inst.id_mem_write_mux, uut.CUMux_inst.size_mux, uut.CUMux_inst.id_se_mux,
                uut.CUMux_inst.id_full_cond_mux, uut.CUMux_inst.id_jalr_sig_mux, uut.CUMux_inst.id_auipc_s_mux,
                uut.CUMux_inst.id_jal_sig_mux
        );

        $strobe("\n| EX Signals: RF En %b, ALU Op %b, SOH %b, Load Inst %b, Mem Ins En %b, MemWrite %b, Size %b, SE %b, Full Cond %b, JALR Sig %b, AUIPC S %b, JAL Sig %b |",
                uut.ID_EX_pipeline_register_inst.ex_rf_enable, uut.ID_EX_pipeline_register_inst.ex_alu_op, uut.ID_EX_pipeline_register_inst.ex_shifter_imm,
                uut.ID_EX_pipeline_register_inst.ex_load_inst, uut.ID_EX_pipeline_register_inst.ex_mem_ins_enable,
                uut.ID_EX_pipeline_register_inst.ex_mem_write, uut.ID_EX_pipeline_register_inst.ex_size, uut.ID_EX_pipeline_register_inst.ex_se,
                uut.ID_EX_pipeline_register_inst.ex_full_cond, uut.ID_EX_pipeline_register_inst.ex_jalr_sig,
                uut.ID_EX_pipeline_register_inst.ex_auipc_s, uut.ID_EX_pipeline_register_inst.ex_jal_sig,
        );

        $strobe("\n| MEM Signals: RF En %b, Load Inst %b, Mem Ins En %b, MemWrite %b, Size %b, SE %b |", 
                uut.EX_MEM_pipeline_register_inst.mem_rf_enable, uut.EX_MEM_pipeline_register_inst.mem_load_inst,
                uut.EX_MEM_pipeline_register_inst.mem_mem_ins_enable, uut.EX_MEM_pipeline_register_inst.mem_mem_write,
                uut.EX_MEM_pipeline_register_inst.mem_size, uut.EX_MEM_pipeline_register_inst.mem_se
        );

      
        $strobe("\n| WB Signals: RF En %b |",
                uut.MEM_WB_pipeline_register_inst.wb_rf_enable
        
        );

        

   
end


endmodule

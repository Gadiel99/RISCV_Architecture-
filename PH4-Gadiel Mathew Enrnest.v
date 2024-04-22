//Aqui va el codigo para fase 4

/*
    Primer paso:
        -Ajuntar los componentes ya hecho de la primera fase junto con la tercera
    
    Segundo paso:
        - Implementar el Hazard/Forward Unit 
    
    Tercer paso: 
        -Implementar los sub components de cada fase. 


    Component list by stage:
        - IF:
            -> Instruction Memory 
            -> PC 
            -> Adder 
            -> MUX (Target address en caso de brinco)
        
        -ID:
            -> Register File
            -> MUX (imm_value selector con signed extension)
            -> Adder
            -> 2 MUXES (PA & PB Handling and Data forwarding)

        -EX:
            -> Second Operand Handler
            -> ALU 
            -> Input MUX(A)
            -> Output MUX
            -> Condition handler 
        
        -MEM
            -> Data Memory
            -> MUX (data forwarding)

    Listas de componentes globales:
        -Control Unit 
        -Hazard/Fowarding Unit 
           
*/

module mux2x1(
    input wire [31:0] input0,
    input wire [31:0] input1,
    input wire control_signal,
    output reg [31:0] output_value
);

always @* begin
    if (control_signal) output_value <= input1;
    else output_value <= input0;
end

endmodule

module mux4x1(
    input wire [31:0] input0,
    input wire [31:0] input1,
    input wire [31:0] input2,
    input wire [31:0] input3,
    input wire [1:0] control_signal,
    output reg [31:0] output_value
);

always @* begin
    if (control_signal == 2'b01) output_value = input1;
    else if (control_signal == 2'b10) output_value = input2;
    else if (control_signal == 2'b11) output_value = input3;
    else output_value = input0; //default case
end

endmodule


/*--------------------------------------IF stage modules--------------------------------------*/

// MUX for TA in case of Jumps

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

/*****PC adder*****/
module Adder(
    output reg [31:0] pcplus4,
    input [31:0] pc
);
    always @(*) begin
        pcplus4 = pc + 4;
    end
endmodule

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

/*****IF/ID Pipeline Register*****/
module IF_ID_pipeline_register( output reg [31:0] instruction, ID_PC, id_pcplus4,
                                output reg [4:0] id_rn, id_rm,
                                output reg [11:0] id_imm12_I,
                                output reg [11:0] id_imm12_S,
                                output reg [19:0] id_imm20,
                                output reg [4:0] id_rd,
                                input  clk, reset, IF_ID_LOAD,
                                input [31:0] ins_mem_out, PC, pcplus4);

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
            id_rn <= instruction[19:15];
            id_rm <= instruction[24:20];
            id_pcplus4 <= pcplus4; 
            id_imm20 <= intruction[31:12];
            id_imm12_I <= instruction[31:20];
            id_imm12_S <= {instruction[31:25], instruction[11:7]};
            id_rd <= instruction[11:7];
        end 
    end
        
    end

endmodule

/*--------------------------------------ID stage modules--------------------------------------*/

// Here goes the register file module
// Muxes and adders in use by the stage

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
    input wire [31:0] id_TA,
    input wire [31:0] id_pc,
    input wire [31:0] id_PA,
    input wire [31:0] id_PB,
    input wire [11:0] id_imm12_I,
    input wire [11:0] id_imm12_S,
    input wire [31:0] id_pc_next,
    input wire [19:0] id_imm20,
    input wire [4:0] id_rd,
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
    output reg ex_jal_sig,
    output reg [31:0] ex_TA,
    output reg [31:0] ex_pc,
    output reg [31:0] ex_PA,
    output reg [31:0] ex_PB,
    output reg [11:0] ex_imm12_I,
    output reg [11:0] ex_imm12_S,
    output reg [31:0] ex_pc_next,
    output reg [19:0] ex_imm20,
    output reg [4:0] ex_rd
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
            ex_TA <= 0;
            ex_pc <= 0;
            ex_PA <= 0;
            ex_PB <= 0;
            ex_imm12_I <= 0;
            ex_imm12_S <= 0;
            ex_pc_next <= 0;
            ex_imm20 <= 0;
            ex_rd <= 0;
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
            ex_TA <= id_TA;
            ex_pc <= id_pc;
            ex_PA <= id_PA;
            ex_PB <= id_PB;
            ex_imm12_I <= id_imm12_I;
            ex_imm12_S <= id_imm12_S;
            ex_pc_next <= id_pc_next;
            ex_imm20 <= id_imm20;
            ex_rd <= id_rd;
        end
    end
   
endmodule

module RegisterFile(PA, PB, RW, PW, SA, SB, Ld, CLK);
    input [31:0] PW;
    input [4:0] SA, SB, RW; // Adjusted to 5 bits to support 32 registers
    input Ld, CLK;
    output [31:0] PA, PB;
    
    wire [31:0] registers [31:0]; // 32 registers of 32 bits
    wire [31:0] write_enable;
    
    // Binary Decoder - Expanded for 32 registers
    binaryDecoder decoder(write_enable, RW, Ld);

    // Registers - Instantiation of 32 registers
    generate
        genvar i;
        for (i = 0; i < 32; i = i + 1) begin : reg_block
            if (i == 0)
                register r(registers[i], 32'b0, 1'b1, CLK); // Register 0 always 0
            else
                register r(registers[i], PW, write_enable[i], CLK);
        end
    endgenerate

    // Multiplexers for the output ports
    Multiplexer32to1 muxA(PA, registers, SA);
    Multiplexer32to1 muxB(PB, registers, SB);
endmodule

module binaryDecoder(output reg [31:0] E, input [4:0] D, input Ld);
    always @(*) begin
        if(Ld)
            E = 32'b1 << D; // Desplazamiento para activar el bit correspondiente
        else
            E = 32'b0;
    end
endmodule

module Multiplexer32to1(output reg [31:0] P, input [31:0] inputs [31:0], input [4:0] S);
    always @(*) begin
        P = inputs[S];
    end
endmodule

module register(output reg [31:0] Q, input [31:0] PW, input RFLd, input CLK);
    
    always @(posedge CLK) begin
        if(RFLd) Q <= PW; // Load the data into the register when RFLd is asserted
    end
endmodule

/*--------------------------------------EX stage modules--------------------------------------*/

// Here goes the ALU module
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

// Here goes the SOH module
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

endmodule

// Here goes the Condition handler
// Muxes for the ALU 

/*****EX/MEM Pipeline Register*****/
module EX_MEM_pipeline_register(     input wire clk, 
    input wire reset,

    input wire [31:0] ex_PB,
    input wire [31:0] ex_mux2x1_alu_output_output,
    input wire ex_rf_enable,
    input wire ex_load_inst,
    input wire ex_mem_ins_enable,
    input wire ex_mem_write, 
    input wire [1:0] ex_size,
    input wire ex_se,

    output reg [31:0] mem_PB,
    output reg [31:0] mem_mux2x1_alu_output_output,
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
            mem_PB <= 32'b0;
            mem_mux2x1_alu_output_output <= 32'b0;
            mem_rf_enable <= 1'b0;
            mem_load_inst <= 1'b0;
            mem_mem_ins_enable <= 1'b0;
            mem_mem_write <= 1'b0;
            mem_size <= 2'b0;
            mem_se <= 1'b0;

        end else begin
        //Control Unit signals  
            mem_PB <= ex_PB;
            mem_mux2x1_alu_output_output <= ex_mux2x1_alu_output_output;
            mem_rf_enable <= ex_rf_enable;
            mem_load_inst <= ex_load_inst;
            mem_mem_ins_enable <= ex_mem_ins_enable;
            mem_mem_write <= ex_mem_write;
            mem_size <= ex_size;
            mem_se <= ex_se;            
        end
    end
   
endmodule

/*--------------------------------------MEM stage modules--------------------------------------*/

//Here goes the data memory module 
/*****Data Memory Module*****/
module data_memory(
    input [8:0] address,
    input [1:0] size,
    input rw,
    input enable,
    input signed_ext,
    input [31:0] data_in,
    output reg[31:0] data_out
);

    reg [7:0] mem[0:511];

    always @(*) begin
        //Enable operation
        if (enable) begin

            //Writing operation
            if (rw) begin

                case (size)
                    
                    //Writing a Byte 
                    2'b00: begin
                        mem[address] <= data_in[7:0];
                    end

                    //Writing a Half-Word
                    2'b01: begin
                        mem[address] <= data_in[7:0];
                        mem[address + 1] <= data_in[15:8];
                    end 
                    
                    //Writing a Word
                    2'b10: begin
                        mem[address] <= data_in[7:0];
                        mem[address + 1] <= data_in[15:8];
                        mem[address + 2] <= data_in[23:16];
                        mem[address + 3] <= data_in[31:24];
                    end
                endcase

            end else begin
                
                //Read Operation
                 case (size)
                    2'b00: begin // Read byte
                        data_out[7:0] = mem[address];
                        data_out[31:8] = signed_ext && mem[address][7] ? 24'hFFFFFF : 24'h0;
                    end
                    2'b01: begin // Read halfword
                        data_out[15:0] = {mem[address + 1], mem[address]};
                        data_out[31:16] = signed_ext && mem[address + 1][7] ? 16'hFFFF : 16'h0;
                    end
                    2'b10, 2'b11: begin // Read word
                        data_out = {mem[address+3], mem[address+2], mem[address+1], mem[address]};
                    end
                endcase 
            end 
        end 
    end

endmodule

// MUX module for data forwarding 


/*--------------------------------------WB stage modules--------------------------------------*/
/*****MEM/WB Pipeline Register*****/
module MEM_WB_pipeline_register(    
    
    input wire clk, reset, s,
    input wire [31:0] mem_mux2x1_mem_output,
    input wire rf_enable,
    output reg [31:0] wb_mem_mux2x1_mem_output,
    output reg wb_rf_enable
    
);

    always@(posedge clk)
    begin
        
        if(reset == 1) begin
            $display("-------------NOP MEM/WB--------------");
            wb_mem_mux2x1_mem_output  <= 1'b0;
            wb_rf_enable <= 1'b0;

        end else begin
        //Control Unit signals  
            wb_mem_mux2x1_mem_output <= mem_mux2x1_mem_output;
            wb_rf_enable <= mem_rf_enable;
        end
    end
   
endmodule

/*--------------------------------------Out-of-Pipeline Modules--------------------------------------*/

/*****Control Unit Module*****/
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

    // ALU Flags
    wire Z_alu, N_alu, C_alu, V_alu;

    // Internal signals
    wire [31:0] pc_current, pc_next, instruction, id_pc, id_TA, ex_TA, ex_pc, id_PA, id_PB, ex_PA, ex_PB, mem_PB, N_SOH, id_pc_next,
     ex_pc_next, mux2x1_alu_input_A_output, alu_output, mem_out, ex_mux2x1_alu_output_output, mem_mux2x1_alu_output_output, mux2x1_ex_TA_output, mux2x1_id_TA_output,
     mux2x1_if_TA_output;

    // imm12_I and imm12_S
    wire [11:0] id_imm12_I, ex_imm12_I, id_imm12_S, ex_imm12_S;

    // imm20
    wire [19:0] id_imm20, ex_imm20;

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
    
    //rn, rm wires
    wire [4:0] id_rn, id_rm;

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
    
    //ins_mem_out 
    wire [31:0] ins_mem_out;

    //rd
    wire [4:0] id_rd, ex_rd, mem_rd, wb_rd;

    //s signal for NOP at CU Mux
    //wire s;

    // mux2x1_alu_output_cs
    wire mux2x1_alu_output_cs = id_jal_sig | ex_jalr_sig;

    wire mux2x1_if_TA_output_cs = id_jal_sig | ex_jalr_sig;

    /*--------------------------------------IF stage--------------------------------------*/

    // PC Reg
    pc_reg pc_reg_inst(
        .clk(clk),
        .reset(reset),
        .en(1'b1),
        .in(mux2x1_if_TA_output),
        .out(pc_current)
    );

    //Adder for Program COunter
    Adder pc_adder(
        .pc(pc_current),
        .pcplus4(pc_next)
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
        .ID_PC(id_pc),
        .id_imm12_S(id_imm12_S),
        .id_imm12_I(id_imm12_I),
        .id_rn(id_rn),
        .id_rm(id_rm),
        .id_imm20(id_imm20),
        .id_rd(id_rd),
        .id_pcplus4(pcplus4)
    );

    /*--------------------------------------ID stage--------------------------------------*/
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
        .id_TA(id_TA),
        .id_pc(id_pc),
        .id_PA(id_PA),
        .id_PB(id_PB),
        .id_imm12_I(id_imm12_I),
        .id_imm12_S(id_imm12_S),
        .id_pc_next(id_pc_next),
        .id_rd(id_rd),
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
        .ex_jal_sig(ex_jal_sig),
        .ex_TA(ex_TA),
        .ex_pc(ex_pc),
        .ex_PA(ex_PA),
        .ex_PB(ex_PB),
        .ex_imm12_I(ex_imm12_I),
        .ex_imm12_S(ex_imm12_S),
        .ex_pc_next(ex_pc_next),
        .ex_rd(ex_rd)
    );


    RegisterFile registerfile_inst(
        .SA(id_rn),
        .SB(id_rm),
        .RW(wb_rd),
        .PW(wb_mem_mux2x1_mem_output),
        .clk(clk),
        .Ld(wb_rf_enable),
        .PA(PA),
        .PB(PB)
    );

    /*--------------------------------------EX stage--------------------------------------*/
    ALU ALU_inst(
        .A(mux2x1_alu_input_A_output),
        .B(N_SOH),
        .Op(ex_alu_op),
        .Out(alu_output),
        .Z(Z_alu),
        .N(N_alu),
        .C(C_alu),
        .V(V_alu)  
    );

    SecondOperandHandler SecondOperandHandler_inst(
        .PB(ex_PB),
        .imm12_I(ex_imm12_I),
        .imm12_S(ex_imm12_S),
        .imm20(ex_imm20),
        .PC(ex_pc),
        .S(ex_shifter_imm), 
        .N(N_SOH)
    );

    // EX/MEM Pipeline Register
    EX_MEM_pipeline_register EX_MEM_pipeline_register_inst(
        .clk(clk),
        .reset(reset),

        .ex_PB(ex_PB),
        .ex_mux2x1_alu_output_output(ex_mux2x1_alu_output_output),      
        .ex_rf_enable(ex_rf_enable),
        .ex_load_inst(ex_load_inst),
        .ex_mem_ins_enable(ex_mem_ins_enable),
        .ex_mem_write(ex_mem_write),
        .ex_size(ex_size),
        .ex_se(ex_se),

        .mem_PB(mem_PB),
        .mem_mux2x1_alu_output_output(mem_mux2x1_alu_output_output),
        .mem_rf_enable(mem_rf_enable),
        .mem_load_inst(mem_load_inst),
        .mem_mem_ins_enable(mem_mem_ins_enable),
        .mem_mem_write(mem_mem_write),
        .mem_size(mem_size),
        .mem_se(mem_se)
    );

    /*--------------------------------------MEM stage--------------------------------------*/
    
    data_memory data_memory_inst(

        .address(mem_mux2x1_alu_output_output),
        .size(mem_size),
        .rw(mem_write_enable),
        .enable(mem_ins_enable),
        .signed_ext(mem_se),
        .data_in(mem_PB),
        .data_out(mem_out)

    );

    //Signal Selector Muxes
    mux2x1 mux2x1_data_memory(
        .input0(mem_mux2x1_alu_output_output),
        .input1(mem_out),
        .control_signal(mem_load_inst),
        .output_value(mux2x1_mem_output)
    );

    // MEM/WB Pipeline Register
    MEM_WB_pipeline_register MEM_WB_pipeline_register_inst(
        .clk(clk),
        .reset(reset),
        .s(s),
        .mem_mux2x1_mem_output(mem_mux2x1_mem_output),
        .mem_rf_enable(mem_rf_enable),
        .wb_rf_enable(wb_rf_enable),
        .wb_mux2x1_mem_output(wb_mux2x1_mem_output)
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

   
    //Signal Selector Muxes
    mux2x1 mux2x1_if_TA(
        .input0(pc_next),
        .input1(mux2x1_id_TA_output),
        .control_signal(mux2x1_if_TA_cs),
        .output_value(mux2x1_if_TA_output)
    );

    mux2x1 mux2x1_id_TA(
        .input0(mux2x1_ex_TA_output),
        .input1(id_TA),
        .control_signal(id_jal_sig),
        .output_value(mux2x1_id_TA_output)
    );

    mux2x1 mux2x1_ex_TA(
        .input0(ex_TA),
        .input1(alu_output),
        .control_signal(ex_jalr_sig),
        .output_value(mux2x1_ex_TA_output)
    );

    mux2x1 mux2x1_alu_input_A (
        .input0(ex_PA),
        .input1(ex_pc),
        .control_signal(ex_auipc_s),
        .output_value(mux2x1_alu_input_A_output)
    );

    mux2x1 mux2x1_alu_output(
        .input0(alu_output),
        .input1(ex_pc_next),
        .control_signal(mux2x1_alu_output_cs),
        .output_value(ex_mux2x1_alu_output_output)
    );

    
    mux4x1 mux4x1_rf_PB_output(
        .input0(PB),
        .input1(ex_mux2x1_alu_output_output),
        .input2(mux2x1_mem_output),
        .input3(wb_mux2x1_mem_output),
        .control_signal(hazard_rf_mux_output),
        .output_value(id_PB_output)
    );

    // Next PC Logic (Placeholder for actual logic)
    //assign pc_next = pc_current + 4;

endmodule
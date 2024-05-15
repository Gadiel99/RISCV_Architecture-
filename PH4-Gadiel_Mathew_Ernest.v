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
    if (control_signal) output_value = input1;
    else output_value = input0;
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
        pcplus4 = pc + 32'b100;
        // $display("pcplus4 = %d", pcplus4);
    end
endmodule


// module SE_12bits(
//     output reg [31:0] id_imm12_I_SE,
//     input [11:0] id_imm12_I
// );

// always @(*) begin
//     id_imm12_I_SE = {{20{id_imm12_I[11]}}, id_imm12_I};
// end
// endmodule

module SE_21bits(
    output reg [31:0] id_imm_J_SE,
    input [20:0] id_imm_J
);

always@(*) begin
    id_imm_J_SE = {{11{id_imm_J[20]}}, id_imm_J};
end
endmodule

module SE_13bits(
    output reg [31:0] id_imm_B_SE,
    input [12:0] id_imm_B
);

always@(*) begin
    id_imm_B_SE = {{19{id_imm_B[12]}}, id_imm_B};
end
endmodule



// module SE_20bits(
//     output reg [31:0] id_imm20_SE,
//     input [19:0] id_imm20
// );

// always@(*) begin
//     id_imm20_SE = {{12{id_imm20[19]}}, id_imm20};
// end
// endmodule

module id_Adder (
    output reg [31:0] id_TA,
    input [31:0] mux2x1_id_adder_input_output,
    input [31:0] id_pc
);
    always @(*) begin
        id_TA = mux2x1_id_adder_input_output + id_pc;
    end
endmodule

/*****Instruction Memory Module - ROM*****/
module instruction_memory(
    input [8:0] address, // 9 bits address for the input
    output reg [31:0] instruction // 32 bits output.
    );

    reg [7:0] mem[0:511];
    
    //Reading the preload memory
    //If this is not working specified the whole directory of the file.
    initial begin
      $readmemb("C:/Users/maxme/Desktop/project P/RISCV_Architecture--2/test_code_2.txt", mem);
    end 
    
    //Making the arragment for the instruction
    always @(*) begin
        instruction = {mem[address+3], mem[address+2], mem[address+1], mem[address]};
    end
endmodule

/*****IF/ID Pipeline Register*****/
module IF_ID_pipeline_register( output reg [31:0] instruction, id_pc, id_pc_next,
                                output reg [4:0] id_rn, id_rm,
                                output reg [11:0] id_imm12_I,
                                output reg [11:0] id_imm12_S,
                                output reg [19:0] id_imm20,
                                output reg [12:0] id_imm_B,
                                output reg [20:0] id_imm_J,
                                output reg [4:0] id_rd,
                                input  clk, reset, LE,
                                input [31:0] ins_mem_out, PC, pc_next);

    always@(posedge clk)
    begin

        if(reset==1) begin
            $display("-------------NOP IF/ID--------------");
            
            instruction <= 32'b0;
            id_pc <= 32'b0;
            id_pc_next <= 32'b0;

        end else if(LE == 1) begin
            
            instruction <= ins_mem_out;
            id_pc <= PC;
            id_rn <= ins_mem_out[19:15];
            id_rm <= ins_mem_out[24:20];
            id_pc_next <= pc_next; 
            id_imm20 <= ins_mem_out[31:12];
            id_imm12_I <= ins_mem_out[31:20];
            id_imm12_S <= {ins_mem_out[31:25], ins_mem_out[11:7]};
            id_rd <= ins_mem_out[11:7];
            id_imm_B <= {ins_mem_out[31], ins_mem_out[7], ins_mem_out[30:25], ins_mem_out[11:8], 1'b0};
            id_imm_J <= {ins_mem_out[31], ins_mem_out[19:12], ins_mem_out[20], ins_mem_out[30:21], 1'b0};

        
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

module RegisterFile(
    input [31:0] PW,
    input [4:0] SA, SB, RW,
    input Ld, CLK,
    output [31:0] PA, PB);

    wire [31:0] registers0, registers1, registers2, registers3,
                   registers4, registers5, registers6, registers7,
                   registers8, registers9, registers10, registers11,
                   registers12, registers13, registers14, registers15,
                   registers16, registers17, registers18, registers19,
                   registers20, registers21, registers22, registers23,
                   registers24, registers25, registers26, registers27,
                   registers28, registers29, registers30, registers31;

    wire [31:0] write_enable;
    binaryDecoder decoder(write_enable, RW, Ld);

    // Registers - Instantiation of 32 registers
    register r0(registers0, 32'b0, 1'b1, CLK); // Register 0 always 0
    register r1(registers1, PW, write_enable[1], CLK);
    register r2(registers2, PW, write_enable[2], CLK);
    register r3(registers3, PW, write_enable[3], CLK);
    register r4(registers4, PW, write_enable[4], CLK);
    register r5(registers5, PW, write_enable[5], CLK);
    register r6(registers6, PW, write_enable[6], CLK);
    register r7(registers7, PW, write_enable[7], CLK);
    register r8(registers8, PW, write_enable[8], CLK);
    register r9(registers9, PW, write_enable[9], CLK);
    register r10(registers10, PW, write_enable[10], CLK);
    register r11(registers11, PW, write_enable[11], CLK);
    register r12(registers12, PW, write_enable[12], CLK);
    register r13(registers13, PW, write_enable[13], CLK);
    register r14(registers14, PW, write_enable[14], CLK);
    register r15(registers15, PW, write_enable[15], CLK);
    register r16(registers16, PW, write_enable[16], CLK);
    register r17(registers17, PW, write_enable[17], CLK);
    register r18(registers18, PW, write_enable[18], CLK);
    register r19(registers19, PW, write_enable[19], CLK);
    register r20(registers20, PW, write_enable[20], CLK);
    register r21(registers21, PW, write_enable[21], CLK);
    register r22(registers22, PW, write_enable[22], CLK);
    register r23(registers23, PW, write_enable[23], CLK);
    register r24(registers24, PW, write_enable[24], CLK);
    register r25(registers25, PW, write_enable[25], CLK);
    register r26(registers26, PW, write_enable[26], CLK);
    register r27(registers27, PW, write_enable[27], CLK);
    register r28(registers28, PW, write_enable[28], CLK);
    register r29(registers29, PW, write_enable[29], CLK);
    register r30(registers30, PW, write_enable[30], CLK);
    register r31(registers31, PW, write_enable[31], CLK);

    // Multiplexers for the output ports
    Multiplexer32to1 muxA(
        PA,
        registers0, registers1, registers2, registers3,
        registers4, registers5, registers6, registers7,
        registers8, registers9, registers10, registers11,
        registers12, registers13, registers14, registers15,
        registers16, registers17, registers18, registers19,
        registers20, registers21, registers22, registers23,
        registers24, registers25, registers26, registers27,
        registers28, registers29, registers30, registers31,
        SA
    );

    Multiplexer32to1 muxB(
        PB,
        registers0, registers1, registers2, registers3,
        registers4, registers5, registers6, registers7,
        registers8, registers9, registers10, registers11,
        registers12, registers13, registers14, registers15,
        registers16, registers17, registers18, registers19,
        registers20, registers21, registers22, registers23,
        registers24, registers25, registers26, registers27,
        registers28, registers29, registers30, registers31,
        SB
    );

endmodule

module binaryDecoder(output reg [31:0] E, input [4:0] D, input Ld);
    always @(*) begin
        if(Ld)
            E = 32'b1 << D;
        else
            E = 32'b0;
    end
endmodule

module Multiplexer32to1(
    output reg [31:0] P,
    input [31:0] input0, input [31:0] input1, input [31:0] input2, input [31:0] input3,
    input [31:0] input4, input [31:0] input5, input [31:0] input6, input [31:0] input7,
    input [31:0] input8, input [31:0] input9, input [31:0] input10, input [31:0] input11,
    input [31:0] input12, input [31:0] input13, input [31:0] input14, input [31:0] input15,
    input [31:0] input16, input [31:0] input17, input [31:0] input18, input [31:0] input19,
    input [31:0] input20, input [31:0] input21, input [31:0] input22, input [31:0] input23,
    input [31:0] input24, input [31:0] input25, input [31:0] input26, input [31:0] input27,
    input [31:0] input28, input [31:0] input29, input [31:0] input30, input [31:0] input31,
    input [4:0] S
);

    always @(*) begin
        case (S)
            5'd0: P = input0;
            5'd1: P = input1;
            5'd2: P = input2;
            5'd3: P = input3;
            5'd4: P = input4;
            5'd5: P = input5;
            5'd6: P = input6;
            5'd7: P = input7;
            5'd8: P = input8;
            5'd9: P = input9;
            5'd10: P = input10;
            5'd11: P = input11;
            5'd12: P = input12;
            5'd13: P = input13;
            5'd14: P = input14;
            5'd15: P = input15;
            5'd16: P = input16;
            5'd17: P = input17;
            5'd18: P = input18;
            5'd19: P = input19;
            5'd20: P = input20;
            5'd21: P = input21;
            5'd22: P = input22;
            5'd23: P = input23;
            5'd24: P = input24;
            5'd25: P = input25;
            5'd26: P = input26;
            5'd27: P = input27;
            5'd28: P = input28;
            5'd29: P = input29;
            5'd30: P = input30;
            5'd31: P = input31;
            default: P = 32'b0;
        endcase
    end

endmodule

module register(output reg [31:0] Q, input [31:0] PW, input RFLd, input CLK);
    always @(posedge CLK) begin
        if(RFLd)
            Q = PW; // Load the data into the register when RFLd is asserted
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
    
    always @ (A, B, Op) begin
        $display("alu_A:%d , alu_B:%d", A, B);
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
               $display("alu_Z:%b , alu_N:%b, alu_A:%d , alu_B:%d", Z, N,A,B);
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
module CONDITION_HANDLER(
    output reg control_hazard_out,
    input wire Z_flag,
    input wire N_flag,
    input wire [9:0] ex_full_cond
);

always @(*) begin
$display("cond:%b, Z:%b, N:%b", ex_full_cond,Z_flag,N_flag);
    // Assuming ex_full_cond[2:0] is the funct3 part
    if(ex_full_cond[9:3] != 7'b1100011)begin
        control_hazard_out = 1'b0;
        
    end else begin
        //if(ex)
        $display("ex_full_cond[2:0]:%b", ex_full_cond[2:0]);
        case (ex_full_cond[2:0])
            3'b000: // BEQ
                if (Z_flag == 1'b1) control_hazard_out = 1'b1;
                else control_hazard_out = 1'b0;
            3'b001: // BNE
            begin
                $display("CH_BNE");
                if (Z_flag == 1'b0) begin
                    control_hazard_out = 1'b1;
                //$display("CH_BNE");
                end else control_hazard_out = 1'b0;
            end
            3'b100: // BLT
                if (Z_flag == 1'b0) control_hazard_out = 1'b1;
                else control_hazard_out = 1'b0;
            3'b101: // BGE
                if ( (Z_flag == 1) | (N_flag == 0)) begin
                    control_hazard_out = 1;
                    //  $display("\nhazard_out:%b", control_hazard_out);
                    //  $display("Z:%b, N:%b", Z_flag,N_flag);
                end else control_hazard_out = 1'b0;
            3'b110: // BLTU
                if (Z_flag == 1'b0) control_hazard_out = 1'b1;
                else control_hazard_out = 1'b0;
            3'b111: // BGEU
                if ( Z_flag == 1'b1) control_hazard_out = 1'b1;
                else control_hazard_out = 1'b0;
            default:
                control_hazard_out = 1'b0;
        endcase
        // $display("\nhazard_out:%b", control_hazard_out);
    end
end

endmodule

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
    input wire [4:0] ex_rd,
    output reg [31:0] mem_PB,
    output reg [31:0] mem_mux2x1_alu_output_output,
    output reg mem_rf_enable,
    output reg mem_load_inst,
    output reg mem_mem_ins_enable,
    output reg mem_mem_write, 
    output reg [1:0] mem_size,
    output reg mem_se,
    output reg [4:0] mem_rd
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
            mem_rd <=0;

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
            mem_rd <= ex_rd;          
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
                        mem[address] = data_in[7:0];
                    end

                    //Writing a Half-Word
                    2'b01: begin
                        mem[address] = data_in[7:0];
                        mem[address + 1] = data_in[15:8];
                    end 
                    
                    //Writing a Word
                    2'b10: begin
                        mem[address] = data_in[7:0];
                        mem[address + 1] = data_in[15:8];
                        mem[address + 2] = data_in[23:16];
                        mem[address + 3] = data_in[31:24];
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
    initial begin
        $readmemb("C:/Users/maxme/Desktop/project P/RISCV_Architecture--2/test_code_2.txt", mem);
    end
endmodule

// MUX module for data forwarding 


/*--------------------------------------WB stage modules--------------------------------------*/
/*****MEM/WB Pipeline Register*****/
module MEM_WB_pipeline_register(    
    
    input wire clk, reset,

    input wire [31:0] mux2x1_mem_output,
    input wire mem_rf_enable,
    input wire [4:0] mem_rd,
    output reg [31:0] wb_mux2x1_mem_output,
    output reg wb_rf_enable,
    output reg [4:0] wb_rd
    
);

    always@(posedge clk)
    begin
        
        if(reset == 1) begin
            $display("-------------NOP MEM/WB--------------");
            wb_mux2x1_mem_output <= 0;
            wb_rf_enable <= 1'b0;
            wb_rd <= 0;

        end else begin
        //Control Unit signals  
            wb_mux2x1_mem_output <= mux2x1_mem_output;
            wb_rf_enable <= mem_rf_enable;
            wb_rd <= mem_rd;
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
    output reg [1:0]num_regs,
    output reg [2:0] func3,
    output reg id_b_sig);
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
        num_regs = 0;
        func3 = instruction[14:12];
        id_b_sig = 0;
        
        if(instruction !=0) begin
            case(instruction[6:0]) // Check the opcode
                7'b0110011: begin // R-Type
                    // Set control signals for R-Type instruction
                    id_rf_enable = 1;
                    num_regs = 2'b10;
                    
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
                    num_regs =  2'b01;

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
                    num_regs = 2'b01;
                    $display("JALR");
                    
                end 

                7'b0000011: begin // I-Type
                    // Set control signals for Load Instructions instruction
                    id_alu_op = 4'b0010;
                    id_shifter_imm = 3'b001;
                    id_rf_enable = 1;
                    id_load_inst = 1;
                    id_mem_ins_enable = 1;
                    num_regs = 2'b01;
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
                    num_regs = 2'b01;
                    case(func3)
                        3'b000: begin 
                            size = 2'b00; // SB instruction
                            $display("SB");
                        end
                        3'b001: begin
                            size = 2'b01; // SH instruction
                            $display("SH");
                        end
                        3'b010: begin
                            size = 2'b10; // SW instruction
                            $display("SW");
                        end
                    endcase
                end

                7'b1100011: begin // B-Type
                    // Set control signals for B-Type instruction
                    // If it's a branch instruction, combine the opcode and funct3

                    id_full_cond = {instruction[6:0], instruction[14:12]};
                    id_shifter_imm = 3'b0;
                    num_regs = 2'b10;
                    id_b_sig = 1;
                    id_alu_op = 4'b0011;
                    case(func3)

                         3'b000: begin
                         //  id_alu_op = 4'b0011;
                           
                            $display("BEQ");
                        end

                         3'b001: begin
                        //   id_alu_op = 4'b0011;
                            $display("BNE");
                        end

                         3'b100: begin
                         //  id_alu_op = 4'b1000;
                            $display("BLT");
                        end
                        
                         3'b101: begin
                         //  id_alu_op = 4'b1000;
                            $display("BGE");
                        end

                         3'b110: begin
                         //  id_alu_op = 4'b1001;
                            $display("BLTU");
                        end

                         3'b111: begin
                         //   id_alu_op = 4'b1001;
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
                    num_regs = 0;
                    $display("LUI");
                end 
                
                
                7'b0010111: begin // U-Type (auipc)
                    // Set control signals for U-Type instruction
                    //TODO: special case
                    id_alu_op = 4'b0010;
                    id_shifter_imm = 3'b011;
                    id_rf_enable = 1;
                    id_auipc_s = 1;
                    num_regs = 0;
                    $display("AUIPC");
                end
                7'b1101111: begin // J-Type
                    // Set control signals for J-Type instruction
                    //TODO: special case
                    id_rf_enable = 1;
                    id_jal_sig = 1;
                    num_regs = 0;
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
    input wire id_b_sig,
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
    input wire [1:0]num_regs,
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
    output reg id_jal_sig_mux,
    output reg [1:0]num_regs_mux,
    output reg id_b_sig_mux
);
 always@* begin
        
        if(s==1) begin
            $display("-------------NOP ID/EXE--------------");
            id_rf_enable_mux = 1'b0;
            id_alu_op_mux = 4'b0;
            id_shifter_imm_mux = 3'b0;
            id_load_inst_mux = 1'b0;
            id_mem_ins_enable_mux = 1'b0;
            id_mem_write_mux = 1'b0;
            size_mux = 2'b0;
            id_se_mux = 2'b0;
            id_full_cond_mux = 10'b0;
            id_jalr_sig_mux = 1'b0;
            id_auipc_s_mux = 1'b0;
            id_jal_sig_mux = 1'b0;
            num_regs_mux = 0;
            id_b_sig_mux = 0;

        end else begin
        //Control Unit signals  
            id_rf_enable_mux = id_rf_enable;
            id_alu_op_mux = id_alu_op;
            id_shifter_imm_mux = id_shifter_imm;
            id_load_inst_mux = id_load_inst;
            id_mem_ins_enable_mux = id_mem_ins_enable;
            id_mem_write_mux = id_mem_write;
            size_mux = size;
            id_se_mux = id_se;
            id_full_cond_mux = id_full_cond;
            id_jalr_sig_mux = id_jalr_sig;
            id_auipc_s_mux = id_auipc_s;
            id_jal_sig_mux = id_jal_sig;
            num_regs_mux = num_regs;
            id_b_sig_mux = id_b_sig;
        end
    end

endmodule

// module hazard_forwarding_unit(
//     input [4:0] id_Rn, id_Rm,
//     input [4:0] ex_Rd, mem_Rd, wb_Rd,
//     input wire ex_Rf_enable, mem_Rf_enable, wb_Rf_enable,
//     input wire ex_load_inst,

//     //Selectors for forwarding
//     output reg [1:0] forwardA,
//     output reg [1:0] forwardB,
//     output reg nop_signal,
//     output reg load_enable,
//     output reg pc_enable
// );

// always @(*) begin
//     // Default values
//     forwardA = 2'b00;
//     forwardB = 2'b00;
//     nop_signal = 1'b0;
//     load_enable = 1'b1;
//     pc_enable = 1'b1;

//     // Check for load-use hazard
//     if (ex_load_inst && (ex_Rd != 0) && ((id_Rn == ex_Rd) || (id_Rm == ex_Rd))) begin
//         // Stall the pipeline if the next instruction needs the result of a memory load
//         load_enable = 1'b0;
//         pc_enable = 1'b0;
//         nop_signal = 1'b1;

//     end else if(ex_Rf_enable && (ex_Rd != 0) && ((id_Rn == ex_Rd))) begin
//         //Forward from EX to ID
//         forwardA = 2'b01;
//     end else if (mem_Rf_enable && (mem_Rd != 0) && (id_Rn == mem_Rd)) begin
//         forwardA = 2'b10; // Forward from MEM to ID
//     end else if (wb_Rf_enable && (wb_Rd != 0) && (id_Rn == wb_Rd)) begin
//         forwardA = 2'b11; // Forward from WB to ID
//     end

//     // Data Forwarding for PB
//     if(ex_Rf_enable && (ex_Rd != 0) && ((id_Rm == ex_Rd))) begin
//         forwardB = 2'b01;
//     end else if (mem_Rf_enable && (mem_Rd != 0) && (id_Rm == mem_Rd)) begin
//         forwardB = 2'b10; // Forward from MEM to EX
//     end else if (wb_Rf_enable && (wb_Rd != 0) && (id_Rm == wb_Rd)) begin
//         forwardB = 2'b11; // Forward from WB to EX
//     end
// end

// endmodule

// // -------------------------- HAZARDS FORWARDING UNIT ------------------------
// module hazard_forwarding_unit (
//     output reg [1:0] ForwardA, ForwardB,
//     output reg [0:0] control_nop, if_id_enable, pc_load_enable,
//     output reg [87:0] fwd_stage,
//     input [0:0] wb_rf_enable, mem_rf_enable, ex_rf_enable, ex_load_instr,
//     input [4:0] rm, rn, ex_rd, mem_rd, wb_rd,
//     input [1:0] num_regs_mux
//     );

//     always @ (*) begin

//         case (num_regs_mux)

//             2'b00: begin
//                 if_id_enable = 1'b1;
//                 pc_load_enable = 1'b1;
//                 control_nop = 1'b0;
//                 ForwardA = 2'b00;
//                 ForwardB = 2'b00;
//                 fwd_stage = "           ";
//             end

//             2'b01: begin

//                 // Defaults
//                 if_id_enable = 1'b1;
//                 pc_load_enable = 1'b1;
//                 control_nop = 1'b0;
//                 ForwardA = 2'b00;
//                 ForwardB = 2'b00;
//                 fwd_stage = "           ";

//                 // Handle forwarding PA
//                 if (ex_load_instr & (rn == ex_rd)) begin
//                     if_id_enable = 1'b0;
//                     pc_load_enable = 1'b0;
//                     control_nop = 1'b1;
//                 end else begin
//                     if_id_enable = 1'b1;
//                     pc_load_enable = 1'b1;
//                     control_nop = 1'b0;
//                     if (ex_rf_enable & (rn == ex_rd)) begin
//                         ForwardA = 2'b01; // Forward ex_out (ex_rd)
//                         fwd_stage = "FWD EX!    ";
//                     end else if (mem_rf_enable & (rn == mem_rd)) begin
//                         ForwardA = 2'b10; // Forward mem_out (mem_rd)
//                         fwd_stage = "FWD MEM!   ";
//                     end else if (wb_rf_enable & (rn == wb_rd)) begin
//                         ForwardA = 2'b11; // Forward wb_out (wb_rd)
//                         fwd_stage = "FWD WB!    ";
//                     end else begin
//                         ForwardA = 2'b00; // Don't forward (keep PA)
//                     end
//                 end
//             end

//             2'b10: begin

//                 // Defaults
//                 if_id_enable = 1'b1;
//                 pc_load_enable = 1'b1;
//                 control_nop = 1'b0;
//                 ForwardA = 2'b00;
//                 ForwardB = 2'b00;
//                 fwd_stage = "           ";

//                 if (ex_load_instr & ((rn == ex_rd) | (rm == ex_rd))) begin
//                     if_id_enable = 1'b0;
//                     pc_load_enable = 1'b0;
//                     control_nop = 1'b1;
//                 end else begin
//                     if_id_enable = 1'b1;
//                     pc_load_enable = 1'b1;
//                     control_nop = 1'b0;
//                     if (ex_rf_enable & (rn == ex_rd)) begin
//                         ForwardA = 2'b01; // Forward ex_out (ex_rd)
//                         fwd_stage = "FWD EX!    ";
//                     end else if (mem_rf_enable & (rn == mem_rd)) begin
//                         ForwardA = 2'b10; // Forward mem_out (mem_rd)
//                         fwd_stage = "FWD MEM!   ";
//                     end else if (wb_rf_enable & (rn == wb_rd)) begin
//                         ForwardA = 2'b11; // Forward wb_out (wb_rd)
//                         fwd_stage = "FWD WB!    ";
//                     end else begin
//                         ForwardA = 2'b00; // Don't forward (keep PA)
//                     end
//                     if (ex_rf_enable & (rm == ex_rd)) begin
//                         ForwardB = 2'b01; // Forward ex_out (ex_rd)
//                         fwd_stage = "FWD EX!    ";
//                     end else if (mem_rf_enable & (rm == mem_rd)) begin
//                         ForwardB = 2'b10; // Forward mem_out (mem_rd)
//                         fwd_stage = "FWD MEM!   ";
//                     end else if (wb_rf_enable & (rm == wb_rd)) begin
//                         ForwardB = 2'b11; // Forward wb_out (wb_rd)
//                         fwd_stage = "FWD WB!    ";
//                     end else begin
//                         ForwardB = 2'b00; // Don't forward (keep PA)
//                     end
//                 end
//             end

//             default: begin
//                 if_id_enable = 1'b1;
//                 pc_load_enable = 1'b1;
//                 control_nop = 1'b0;
//                 ForwardA = 2'b00;
//                 ForwardB = 2'b00;
//                 fwd_stage = "           ";
//             end
            
//         endcase

//     end
// endmodule

// -------------------------- HAZARDS FORWARDING UNIT ------------------------
module hazard_forwarding_unit (
    output reg [1:0] ForwardA, ForwardB,
    output reg [0:0] control_nop, if_id_enable, pc_load_enable,
    output reg [87:0] fwd_stage,
    input [0:0] wb_rf_enable, mem_rf_enable, ex_rf_enable, ex_load_instr,
    input [4:0] rm, rn, ex_rd, mem_rd, wb_rd,
    input [1:0] num_regs_mux
    );

    always @ (*) begin
       //$display("rs1:%d , rd:%d", rn, ex_rd );
        case (num_regs_mux)

            2'b00: begin
                if_id_enable = 1'b1;
                pc_load_enable = 1'b1;
                control_nop = 1'b0;
                ForwardA = 2'b00;
                ForwardB = 2'b00;
                fwd_stage = "           ";
            end

            2'b01: begin

                // Defaults
                if_id_enable = 1'b1;
                pc_load_enable = 1'b1;
                control_nop = 1'b0;
                ForwardA = 2'b00;
              //  ForwardB = 2'b00;
                fwd_stage = "           ";

                if (ex_load_instr & (rn == ex_rd)) begin
                    if_id_enable = 1'b0;
                    pc_load_enable = 1'b0;
                    control_nop = 1'b1;
                end else begin
                    if_id_enable = 1'b1;
                    pc_load_enable = 1'b1;
                    control_nop = 1'b0;
                    if (ex_rf_enable & (rn == ex_rd)) begin
                        ForwardA = 2'b01; // Forward ex_out (ex_rd)
                        fwd_stage = "FWD EX!    ";
                    end else if (mem_rf_enable & (rn == mem_rd)) begin
                        ForwardA = 2'b10; // Forward mem_out (mem_rd)
                        fwd_stage = "FWD MEM!   ";
                    end else if (wb_rf_enable & (rn == wb_rd)) begin
                        ForwardA = 2'b11; // Forward wb_out (wb_rd)
                        fwd_stage = "FWD WB!    ";
                    end 
                     
                end
            end

            2'b10: begin

                // Defaults
                if_id_enable = 1'b1;
                pc_load_enable = 1'b1;
                control_nop = 1'b0;
                ForwardA = 2'b00;
                ForwardB = 2'b00;
                fwd_stage = "           ";

                if (ex_load_instr & ((rn == ex_rd) | (rm == ex_rd))) begin
                    if_id_enable = 1'b0;
                    pc_load_enable = 1'b0;
                    control_nop = 1'b1;
                end else begin
                    if_id_enable = 1'b1;
                    pc_load_enable = 1'b1;
                    control_nop = 1'b0;
                    if (ex_rf_enable & (rn == ex_rd)) begin
                        ForwardA = 2'b01; // Forward ex_out (ex_rd)
                        fwd_stage = "FWD EX!    ";
                    end else if (mem_rf_enable & (rn == mem_rd)) begin
                        ForwardA = 2'b10; // Forward mem_out (mem_rd)
                        fwd_stage = "FWD MEM!   ";
                    end else if (wb_rf_enable & (rn == wb_rd)) begin
                        ForwardA = 2'b11; // Forward wb_out (wb_rd)
                        fwd_stage = "FWD WB!    ";
                    end 
                    if (ex_rf_enable & (rm == ex_rd)) begin
                        ForwardB = 2'b01; // Forward ex_out (ex_rd)
                        fwd_stage = "FWD EX!    ";
                    end else if (mem_rf_enable & (rm == mem_rd)) begin
                        ForwardB = 2'b10; // Forward mem_out (mem_rd)
                        fwd_stage = "FWD MEM!   ";
                    end else if (wb_rf_enable & (rm == wb_rd)) begin
                        ForwardB = 2'b11; // Forward wb_out (wb_rd)
                        fwd_stage = "FWD WB!    ";
                    end 
                end
            end

            default: begin
                if_id_enable = 1'b1;
                pc_load_enable = 1'b1;
                control_nop = 1'b0;
                ForwardA = 2'b00;
                ForwardB = 2'b00;
                fwd_stage = "           ";
            end
            
        endcase

    end
endmodule

module processor(
    input wire clk,
    input wire reset
);
    

    // ALU Flags
    wire Z_alu, N_alu, C_alu, V_alu;

    // Internal signals
    wire [31:0] pc_current, pc_next, instruction, id_pc, id_TA, ex_TA, ex_pc, id_PA, id_PB, ex_PA, ex_PB, mem_PB, N_SOH, id_pc_next,
     ex_pc_next, mux2x1_alu_input_A_output, alu_output, mem_out, ex_mux2x1_alu_output_output, mem_mux2x1_alu_output_output, mux2x1_ex_TA_output, mux2x1_id_TA_output,
     mux2x1_if_TA_output, mux2x1_mem_output, wb_mux2x1_mem_output, id_imm20_SE, id_imm12_I_SE, mux2x1_id_adder_input_output;

    wire pc_enable, load_enable, nop_signal;

    wire [1:0] forwardA_out, forwardB_out;

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
    //wire IF_ID_LOAD = 1'b1; // Assuming always enabled for this phase
    
    //ins_mem_out 
    wire [31:0] ins_mem_out;

    //rd
    wire [4:0] id_rd, ex_rd, mem_rd, wb_rd;

    // control hazard output
    wire control_hazard_signal;

    //s signal for NOP at CU Mux
    //wire s;
    wire id_ex_pipe_reg_reset_signal = ex_jalr_sig | control_hazard_signal;
     
    // mux2x1_alu_output_cs
    wire mux2x1_alu_output_cs = id_jal_sig_mux | ex_jalr_sig;

    wire mux2x1_if_TA_output_cs = id_jal_sig_mux | ex_jalr_sig | control_hazard_signal;
    
    //wire mux2x1_if_TA_output_cs = 0;
    wire mux2x1_id_adder_input_cs = id_b_sig_mux;
    wire id_b_sig_mux;
    wire id_b_sig;

    wire [12:0] id_imm_B;
    wire [20:0] id_imm_J;
    wire [31:0] id_imm_B_SE;
    wire [31:0] id_imm_J_SE;
    wire [31:0] mux2x1_id_jump_TA_output;
    // id pa / pb output

    wire [31:0] id_PA_output, id_PB_output;

    // New Hazard fwd stage
    wire [87:0] fwd_stage;

    wire [1:0] num_regs, num_regs_mux;

    /*--------------------------------------IF stage--------------------------------------*/

    // PC Reg
    pc_reg pc_reg_inst(
        .clk(clk),
        .reset(reset),
        .en(pc_enable),
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
        .instruction(ins_mem_out)
    );

    // IF/ID Pipeline Register
    IF_ID_pipeline_register IF_ID_pipeline_register_inst(
        .clk(clk),
        .reset(mux2x1_if_TA_output_cs),
        .LE(load_enable),
        .ins_mem_out(ins_mem_out),
        .PC(pc_current),
        .instruction(instruction),
        .pc_next(pc_next),
        .id_pc(id_pc),
        .id_imm12_S(id_imm12_S),
        .id_imm12_I(id_imm12_I),
        .id_rn(id_rn),
        .id_rm(id_rm),
        .id_imm20(id_imm20),
        .id_rd(id_rd),
        .id_pc_next(id_pc_next), 
        .id_imm_B(id_imm_B),
        .id_imm_J(id_imm_J)
    );

    /*--------------------------------------ID stage--------------------------------------*/
    // ID/EX Pipeline Register
    ID_EX_pipeline_register ID_EX_pipeline_register_inst(
        .clk(clk),
        .reset(id_ex_pipe_reg_reset_signal),
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
        .id_PA(id_PA_output),
        .id_PB(id_PB_output),
        .id_imm12_I(id_imm12_I),
        .id_imm12_S(id_imm12_S),
        .id_pc_next(id_pc_next),
        .id_imm20(id_imm20),
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
        .ex_imm20(ex_imm20),
        .ex_rd(ex_rd)
    );


    RegisterFile registerfile_inst(
        .SA(id_rn),
        .SB(id_rm),
        .RW(wb_rd),
        .PW(wb_mux2x1_mem_output),
        .CLK(clk),
        .Ld(wb_rf_enable),
        .PA(id_PA),
        .PB(id_PB)
    );

    // SE_12bits SE_12bits_inst(
    //     .id_imm12_I(id_imm12_I),
    //     .id_imm12_I_SE(id_imm12_I_SE)
    // );

    // SE_20bits SE_20bits_inst(
    //     .id_imm20(id_imm20),
    //     .id_imm20_SE(id_imm20_SE)
    // );

    SE_21bits SE_21bits_inst (
        .id_imm_J_SE(id_imm_J_SE),
        .id_imm_J(id_imm_J)
    );

    SE_13bits SE_13bits_inst (
        .id_imm_B_SE(id_imm_B_SE),
        .id_imm_B(id_imm_B)
    );

    id_Adder id_Adder_inst(
        .mux2x1_id_adder_input_output(mux2x1_id_adder_input_output),
        .id_pc(id_pc),
        .id_TA(id_TA)
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

    CONDITION_HANDLER condition_handler_inst(
        .control_hazard_out(control_hazard_signal), // This would control branches
        .Z_flag(Z_alu),  // Zero flag from ALU
        .N_flag(N_alu),  // Negative flag from ALU
        .ex_full_cond(ex_full_cond)  // Condition code from ID/EX register
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
        .ex_rd(ex_rd),
        .mem_PB(mem_PB),
        .mem_mux2x1_alu_output_output(mem_mux2x1_alu_output_output),
        .mem_rf_enable(mem_rf_enable),
        .mem_load_inst(mem_load_inst),
        .mem_mem_ins_enable(mem_mem_ins_enable),
        .mem_mem_write(mem_mem_write),
        .mem_size(mem_size),
        .mem_se(mem_se),
        .mem_rd(mem_rd)
    );

    /*--------------------------------------MEM stage--------------------------------------*/
    
    data_memory data_memory_inst(

        .address(mem_mux2x1_alu_output_output[8:0]),
        .size(mem_size),
        .rw(mem_mem_write),
        .enable(mem_mem_ins_enable),
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
        .mem_rd(mem_rd),
        .mux2x1_mem_output(mux2x1_mem_output),
        .mem_rf_enable(mem_rf_enable),
        .wb_rf_enable(wb_rf_enable),
        .wb_mux2x1_mem_output(wb_mux2x1_mem_output),
        .wb_rd(wb_rd)
    );

    /*--------------------------------------OUT-OF-Pipeline-SCOPE--------------------------------------*/
    // Control Unit
    control_unit control_unit_inst(
        .instruction(instruction),
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
        .func3(func3),
        .num_regs(num_regs),
        .id_b_sig(id_b_sig)
    );

    CUMux CUMux_inst(
        .s(nop_signal),
        .id_b_sig(id_b_sig),
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
        .num_regs(num_regs),
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
        .id_jal_sig_mux(id_jal_sig_mux),
        .num_regs_mux(num_regs_mux),
        .id_b_sig_mux(id_b_sig_mux)
    );

    // //Hazard Forwarding Unit
    // hazard_forwarding_unit hazard_forwarding_unit_inst(
    //     .id_Rn(id_rn),
    //     .id_Rm(id_rm),
    //     .ex_Rd(ex_rd),
    //     .mem_Rd(mem_rd),
    //     .wb_Rd(wb_rd),
    //     .ex_Rf_enable(ex_rf_enable),
    //     .mem_Rf_enable(mem_rf_enable),
    //     .wb_Rf_enable(wb_rf_enable),
    //     .ex_load_inst(ex_load_inst),
    //     .forwardA(forwardA_out),
    //     .forwardB(forwardB_out),
    //     .nop_signal(nop_signal),
    //     .load_enable(load_enable),
    //     .pc_enable(pc_enable)
    // );

    //Hazard Forwarding Unit
        hazard_forwarding_unit hazard_forwarding_unit_inst(
            .rn(id_rn),
            .rm(id_rm),
            .ex_rd(ex_rd),
            .mem_rd(mem_rd),
            .wb_rd(wb_rd),
            .ex_rf_enable(ex_rf_enable),
            .mem_rf_enable(mem_rf_enable),
            .wb_rf_enable(wb_rf_enable),
            .ex_load_instr(ex_load_inst),
            .ForwardA(forwardA_out),
            .ForwardB(forwardB_out),
            .control_nop(nop_signal),
            .if_id_enable(load_enable),
            .pc_load_enable(pc_enable),
            .fwd_stage(fwd_stage),
            .num_regs_mux(num_regs_mux)
        );
   
    //Signal Selector Muxes
    mux2x1 mux2x1_if_TA(
        .input0(pc_next),
        .input1(mux2x1_id_TA_output),
        .control_signal(mux2x1_if_TA_output_cs),
        .output_value(mux2x1_if_TA_output)
    );

    mux2x1 mux2x1_id_TA(
        .input0(mux2x1_ex_TA_output),
        .input1(id_TA),
        .control_signal(id_jal_sig_mux),
        .output_value(mux2x1_id_TA_output)
    );

    // mux2x1 mux2x1_id_Jump_TA(
    //     .input0(id_imm_B_SE),
    //     .input1(id_imm_J_SE), id_imm_J_SE
    //     .control_signal(id_jal_sig_mux),
    //     .output_value(mux2x1_id_jump_TA_output)
    // );

    mux2x1 mux2x1_id_adder_input(
        .input0(id_imm_B_SE),
        .input1(id_imm_J_SE),
        .control_signal(id_jal_sig_mux),
        .output_value(mux2x1_id_adder_input_output)
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

    mux4x1 mux4x1_rf_PA_output(
        .input0(id_PA),
        .input1(ex_mux2x1_alu_output_output),
        .input2(mux2x1_mem_output),
        .input3(wb_mux2x1_mem_output),
        .control_signal(forwardA_out),
        .output_value(id_PA_output)
    );
    
    mux4x1 mux4x1_rf_PB_output(
        .input0(id_PB),
        .input1(ex_mux2x1_alu_output_output),
        .input2(mux2x1_mem_output),
        .input3(wb_mux2x1_mem_output),
        .control_signal(forwardB_out),
        .output_value(id_PB_output)
    );
endmodule
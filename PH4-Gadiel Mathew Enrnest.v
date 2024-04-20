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

/*--------------------------------------EX stage modules--------------------------------------*/

// Here goes the ALU module
// Here goes the SOH module
// Here goes the Condition handler
// Muxes for the ALU 

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

/*--------------------------------------MEM stage modules--------------------------------------*/

//Here goes the data memory module 
// MUX module for data forwarding 


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
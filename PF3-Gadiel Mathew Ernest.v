//Aqui es donde va el codigo para el control unit de la fase 3 

/* 
Proyecto 
Fase III: Sistema de Control 
    -El objetivo de esta fase es mostrar que  la Unidad de Control del PPU decodifica correctamente 
     las instrucciones y genera y propaga las señales de control necesarias en las etapas EX, MEM y WB. 

    -Todos los registros se deben implementar "rising edge-triggered" con reset sincrónico. Un reset tienen el efecto de hacer cero todos los bits de salida del registro. 
    
    -El registro PC y el IF/ID deben tener una señal de load enable (E) que estará habilitada con un uno para esta fase. Los demás
    pipelined registers no requieren señal de load enable. 

    -Las señales de control que salen de la Unidad de Control son las señales que se identificaron en el diagrama de bloque que se generó
    en la Fase II. 

    -La Unidad de Control debe poner todas las salidas igual a cero cuando la instrucción
    que se decodifica tiene todos sus bits igual a cero.

Demostración:
    Deben implementar un módulo de prueba en el cual deben precargar la memoria de
    instrucciones con el siguiente segmento código comenzando en la localización 0:
    
            rs2     rs1        rd
    00000000 00000010 00000010 10010011 ADDI r5, r4, 0
    01000000 00110000 00000001 10110011 SUB r3, r0, r3
    00000000 00000000 10000001 00000011 LB r2, 0(r1)
    00000000 01010000 10000011 00100011 SB r5, 6(r1)
    11111111 11000001 11011101 11100011 BGE r3, r28, -3
    00000011 11110000 10100111 10110111 LUI r15, #03F0A
    00000001 00000000 00001010 01101111 JAL r20, +8
    00000000 11100110 00001111 11100111 JALR r31, r12, +14
    00000000 00000010 00000010 10010011 ADDI r5, r4, 0
    01000000 00110000 00000001 10110011 SUB r3, r0, r3
    00000000 00000000 10000001 00000011 LBU r2, 0(r1)
    00000000 01010000 10000011 00100011 SB r5, 6(r1)
    11111111 11000001 11011101 11100011 BGE r3, r28, -3
    00000000 00000000 00000000 00000000 NOP
    00000000 00000000 00000000 00000000 NOP


    -La simulación debe comenzar haciendo, a tiempo cero, Clk igual a cero, reset igual a uno y las
    señales de enable de los registros PC y el IF/ID igual a uno. Entonces, Clk debe cambiar de estado
    cada dos unidades de tiempo de manera perpetua. 

    -La señal reset debe cambiar a 0 en tiempo 3.

    -La señal S del multiplexer debe tener un valor de cero a tiempo cero y debe cambiar a 1 a tiempo
    40. 

    -La simulación debe culminar en el tiempo 48.

    -En cada ciclo del reloj deben mostrar, en una primera línea, el keyword de la instrucción que llega
    a la Unidad de Control, seguido del valor de PC (en decimal) y el de las señales de salida de la
    Unidad de Control (en binario). 

-En líneas sucesivas deben imprimir las señales de control que llegan a las etapas EX, MEM y WB respectivamente (en binario).

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

//Program Counter module
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

module mux2to1 #(parameter WIDTH = 1 ) (
    input wire [WIDTH-1:0] input0,
    input wire [WIDTH-1:0] input1,
    input wire s, // This signal determines which input is passed to the output
    output wire [WIDTH-1:0] out
);

    assign out = s ? input1 : input0;

endmodule


//Control unit module
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
                    id_load_inst = 1;
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

//IF/ID PIPELINE REGISTER
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

//ID/EX PIPELINE REGISTER
module ID_EX_pipeline_register( input wire clk, 
    input wire reset,
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
            ex_rf_enable <= id_rf_enable;
            ex_alu_op <= id_alu_op;
            ex_shifter_imm <= id_shifter_imm;
            ex_load_inst <= id_load_inst;
            ex_mem_ins_enable <= id_mem_ins_enable;
            ex_mem_write <= id_mem_write;
            ex_size <= size;
            ex_se <= id_se;
            ex_full_cond <= id_full_cond;
            ex_jalr_sig <= id_jalr_sig;
            ex_auipc_s <= id_auipc_s;
            ex_jal_sig <= id_jal_sig;
        end
    end
   
endmodule

//EX/MEM PIPELINE REGISTER

module EX_MEM_pipeline_register(     input wire clk, 
    input wire reset,
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

//MEM/WB PIPELINE REGISTER
module MEM_WB_pipeline_register(    
                                    output reg wb_rf_enable,

                                    input  clk, reset, mem_rf_enable);

    always@(posedge clk)
    begin
        
        if(reset==1) begin
            $display("-------------NOP MEM/WB--------------");
            wb_rf_enable <= 1'b0;

        end else begin
        //Control Unit signals  
            wb_rf_enable <= mem_rf_enable;
        end
    end
   
endmodule

module Adder(
    output reg [31:0] pcplus4,
    input [31:0] pc
);
    always @(*) begin
        pcplus4 = pc + 4;
    end
endmodule


module processor(
    input wire clk,
    input wire reset,
    input wire s
);

    // Internal signals
    wire [31:0] pc_current, pc_next, instruction, id_pc;
    wire [3:0]  muxed_id_alu_op, id_alu_op, ex_alu_op;
    wire [2:0]  muxed_id_shifter_imm, id_shifter_imm, ex_shifter_imm;
    wire  muxed_id_rf_enble, id_rf_enable, ex_rf_enable, mem_rf_enable, wb_rf_enable;
    wire  muxed_id_load_inst, id_load_inst, ex_load_inst, mem_load_inst;
    wire  muxed_id_mem_ins_enable, id_mem_ins_enable, ex_mem_ins_enable, mem_mem_ins_enable;
    wire  muxed_id_mem_write, id_mem_write, ex_mem_write, mem_mem_write;
    wire [1:0]  muxed_id_size, id_size, ex_size, mem_size;
    wire  muxed_id_se, id_se, ex_se, mem_se;
    wire [9:0] muxed_id_full_cond, id_full_cond, ex_full_cond;
    wire  muxed_id_jalr_sig, id_jalr_sig, ex_jalr_sig;
    wire  muxed_id_auipc_s, id_auipc_s, ex_auipc_s;
    wire  muxed_id_jal_sig, id_jal_sig, ex_jal_sig;


    wire add_sub_sign;
    
    //funct3 
    wire [2:0] func3;
    
    //IF_ID_LOAD
    wire IF_ID_LOAD = 1'b1; // Assuming always enabled for this phase
    
    //ins_mem_out && ID_EX_pc_next, EX_MEM_pc_next, MEM_WB_pc_next
    wire [31:0] ins_mem_out, ID_EX_pc_next, EX_MEM_pc_next, MEM_WB_pc_next;

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
        .size(id_size),
        .id_se(id_se),
        .id_full_cond(id_full_cond),
        .id_jalr_sig(id_jalr_sig),
        .id_auipc_s(id_auipc_s),
        .id_jal_sig(id_jal_sig),
        .add_sub_sign(add_sub_sign),
        .func3(func3)
    );

    //Here enteres the signal from the control mux 
    mux2to1 #(4) mux_alu_op(
        .input0(id_alu_op),
        .input1(4'b0),
        .s(s),
        .out(muxed_id_alu_op)
    );

    mux2to1 #(3) mux_shifter_imm(
        .input0(id_shifter_imm),
        .input1(3'b0),
        .s(s),
        .out(muxed_id_shifter_imm)
    );

    mux2to1 #(1) mux_rf_enable(
        .input0(id_rf_enable),
        .input1(1'b0),
        .s(s),
        .out(muxed_id_rf_enable)
    );

    mux2to1 #(1) mux_load_inst(
        .input0(id_load_inst),
        .input1(1'b0),
        .s(s),
        .out(muxed_id_load_inst)
    );

    mux2to1 #(1) mux_mem_ins_enable(
        .input0(id_mem_ins_enable),
        .input1(1'b0),
        .s(s),
        .out(muxed_id_mem_ins_enable)
    );

    mux2to1 #(1) mux_mem_write(
        .input0(id_mem_write),
        .input1(1'b0),
        .s(s),
        .out(muxed_id_mem_write)
    );

    mux2to1 #(2) mux_size(
        .input0(id_size),
        .input1(2'b0),
        .s(s),
        .out(muxed_id_size)
    );

    mux2to1 #(1) mux_se(
        .input0(id_se),
        .input1(1'b0),
        .s(s),
        .out(muxed_id_se)
    );

    mux2to1 #(10) mux_full_cond(
        .input0(id_full_cond),
        .input1(10'b0),
        .s(s),
        .out(muxed_id_full_cond)
    );

    mux2to1 #(1) mux_jalr_sig(
        .input0(id_jalr_sig),
        .input1(1'b0),
        .s(s),
        .out(muxed_id_jalr_sig)
    );

    mux2to1 #(1) mux_auipc_s(
        .input0(id_auipc_s),
        .input1(1'b0),
        .s(s),
        .out(muxed_id_auipc_s)
    );

    mux2to1 #(1) mux_jal_sig(
        .input0(id_jal_sig),
        .input1(1'b0),
        .s(s),
        .out(muxed_id_jal_sig)
    );

    // ID/EX Pipeline Register
    ID_EX_pipeline_register ID_EX_pipeline_register_inst(
        .clk(clk),
        .reset(reset),
        .id_alu_op(muxed_id_alu_op),
        .id_shifter_imm(muxed_id_shifter_imm),
        .id_rf_enable(muxed_id_rf_enable),
        .id_load_inst(muxed_id_load_inst),
        .id_mem_ins_enable(muxed_id_mem_ins_enable),
        .id_mem_write(muxed_id_mem_write),
        .size(muxed_id_size),
        .id_se(muxed_id_se),
        .id_full_cond(muxed_id_full_cond),
        .id_jalr_sig(muxed_id_jalr_sig),
        .id_auipc_s(muxed_id_auipc_s),
        .id_jal_sig(muxed_id_jal_sig),
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
        .mem_rf_enable(mem_rf_enable),
        .wb_rf_enable(wb_rf_enable)
    );

    // Next PC Logic (Placeholder for actual logic)
    assign pc_next = pc_current + 4;

endmodule


module processor_testbench;

    // Inputs
    reg clk;
    reg reset;
    reg s; 

    // Instantiate the processor module
    processor uut (
        .clk(clk),
        .reset(reset),
        .s(s) 
    );

    initial begin
        // Initialize Inputs
        clk = 0;    
        reset = 1;  
        s = 0;      

        // Wait 3 units of time, then release reset
        #3 reset = 0;

        // Change selector signal to 1 at time 40
        #37 s = 1; // 37 additional units after reset is released

        // Terminate simulation at time 48
        #8 $finish; // 8 more units after changing 's' to 1
    end

    // Clock with a period of 2 units
    always #1 clk = !clk; // Toggle clock every 1 unit of time, resulting in a period of 2 units

    // Display internal states at each positive edge of the clock after reset is de-asserted
    always @(posedge clk) begin
        if (!reset) begin
            $display("\nTime: %t \nPC:%d \nInstruction Fetched: %b, \nS: %b", 
                     $time, uut.pc_reg_inst.out, uut.instruction_memory_inst.instruction, s);

            $display("\n| ID Signals: RF En %b, ALU Op %b, SOH %b, Load Inst %b, Mem Ins En %b, MemWrite %b, Size %b, SE %b Full Cond %b, JALR Sig %b, AUIPC S %b, JAL Sig %b |", 
                     uut.control_unit_inst.id_rf_enable, uut.control_unit_inst.id_alu_op, uut.control_unit_inst.id_shifter_imm, uut.control_unit_inst.id_load_inst,
                     uut.control_unit_inst.id_mem_ins_enable, uut.control_unit_inst.id_mem_write, uut.control_unit_inst.size, uut.control_unit_inst.id_se,
                     uut.control_unit_inst.id_full_cond, uut.control_unit_inst.id_jalr_sig, uut.control_unit_inst.id_auipc_s,
                     uut.control_unit_inst.id_jal_sig
            );

            $display("\n| EX Signals: RF En %b, ALU Op %b, SOH %b, Load Inst %b, Mem Ins En %b, MemWrite %b, Size %b, SE %b, Full Cond %b, JALR Sig %b, AUIPC S %b, JAL Sig %b |",
                     uut.ID_EX_pipeline_register_inst.ex_rf_enable, uut.ID_EX_pipeline_register_inst.ex_alu_op, uut.ID_EX_pipeline_register_inst.ex_shifter_imm,
                     uut.ID_EX_pipeline_register_inst.ex_load_inst, uut.ID_EX_pipeline_register_inst.ex_mem_ins_enable,
                     uut.ID_EX_pipeline_register_inst.ex_mem_write, uut.ID_EX_pipeline_register_inst.ex_size, uut.ID_EX_pipeline_register_inst.ex_se,
                     uut.ID_EX_pipeline_register_inst.ex_full_cond, uut.ID_EX_pipeline_register_inst.ex_jalr_sig,
                     uut.ID_EX_pipeline_register_inst.ex_auipc_s, uut.ID_EX_pipeline_register_inst.ex_jal_sig
            );

            $display("\n| MEM Signals: RF En %b, Load Inst %b, Mem Ins En %b, MemWrite %b, Size %b, SE %b |", 
                     uut.EX_MEM_pipeline_register_inst.mem_rf_enable, uut.EX_MEM_pipeline_register_inst.mem_load_inst,
                     uut.EX_MEM_pipeline_register_inst.mem_mem_ins_enable, uut.EX_MEM_pipeline_register_inst.mem_mem_write,
                     uut.EX_MEM_pipeline_register_inst.mem_size, uut.EX_MEM_pipeline_register_inst.mem_se
            );

            $display("\n| WB Signals: RF En %b |",
                     uut.MEM_WB_pipeline_register_inst.wb_rf_enable
            );
        end
    end
endmodule
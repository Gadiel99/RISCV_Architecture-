//Aqui es donde va el codigo para el control unit de la fase 3 

/* 
Proyecto 
Fase III: Sistema de Control 
    -El objetivo de esta fase es mostrar que  la Unidad de Control del PPU decodifica correctamente 
     las instrucciones y genera y propaga las señales de control necesarias en las etapas EX, MEM y WB. 

    -Todos los registros se deben implementar “rising edge-triggered” con reset sincrónico. Un reset tienen el efecto de hacer cero todos los bits de salida del registro. 
    
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
    00000000 0000|0010 0|000|0010 1|0010011 ADDI r5, r4, 0
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




    -La simulación debe comenzar haciendo, a tiempo cero, Clk igual a cero, Reset igual a uno y las
    señales de enable de los registros PC y el IF/ID igual a uno. Entonces, Clk debe cambiar de estado
    cada dos unidades de tiempo de manera perpetua. 

    -La señal Reset debe cambiar a 0 en tiempo 3.

    -La señal S del multiplexer debe tener un valor de cero a tiempo cero y debe cambiar a 1 a tiempo
    40. 

    -La simulación debe culminar en el tiempo 48.

    -En cada ciclo del reloj deben mostrar, en una primera línea, el keyword de la instrucción que llega
    a la Unidad de Control, seguido del valor de PC (en decimal) y el de las señales de salida de la
    Unidad de Control (en binario). 

-En líneas sucesivas deben imprimir las señales de control que llegan a las etapas EX, MEM y WB respectivamente (en binario).

*/

//Program Counter module
module pc_reg ( input wire clk,
                input wire rst,
                input wire en,
                input wire [31:0] in,
                output reg [31:0] out
);

    always@(posedge clk) begin
        if (rst) out <= 32'b0;
        else if (en) out <= in;
    end
endmodule

//Generic pipeline register module 
//Maybe use this to create all the pipeline registers | IF/ID -> ID/EX -> EX/MEM -> MEM/WB |
module pipeline_reg(input wire clk,
                    input wire rst,
                    input wire [N-1:0] in, 
                    output reg [N-1:0] out,

    parameter N = 32; //Default register size

    always @(posedge clk) begin
        if(rst) out <= {N{1'b0}};
        else out <= in;
    end 
);         
endmodule

//Control unit module
module control_unit(input [31:0] instruction,
        output reg [3:0] id_alu_op, 
               reg [2:0] id_shifter_imm,
               reg id_rf_enable, 
               reg id_load_inst, 
               reg id_mem_ins_enable, 
               reg id_mem_write, 
               reg [1:0] size,
               reg [9:0] id_full_cond,
               reg ex_jalr_sig,
               reg id_auipc_s,
               reg id_jal_sig);
    //Decode logic begeins here
    
    always @(instruction)
        begin
        id_alu_op = 0;
        id_shifter_imm = 0;
        id_rf_enable = 0;
        id_load_inst = 0;
        id_mem_ins_enable = 0;
        id_mem_write = 0;
        size = 0;
        id_full_cond = 0;
        ex_jalr_sig = 0;
        id_auipc_s = 0;
        id_jal_sig = 0;
        func3[2:0] = instruction[14:12]; 
        
        if(instruction !=0)
            case(instruction[6:0]) // Check the opcode
                7'b0110011: begin // R-Type
                    // Set control signals for R-Type instruction
                    id_alu_op = 1;
                    id_rf_enable = 1;
                    id_mem_ins_enable = 1;
                    case(func3)
                    

                        3'b000: begin
                            add_sub_sign = instruction[30];

                            case(add_sub_sign)
                                1'1: begin // SUB case
                                    $display("SUB");

                                end

                                1'0: begin // ADD case
                                    $display("SUB");

                                end
                            endcase
                        end
                        3'b101: begin
                            add_sub_sign = instruction[30];

                            case(add_sub_sign)
                                1'1: begin // SUB case
                                    $display("SRA");

                                end

                                1'0: begin // ADD case
                                    $display("SRL");

                                end
                            endcase
                        end

                        3'b010: begin
                            $display("SLT");
                        end

                        3'b011: begin
                            $display("SLTU");
                        end

                        3'b111: begin
                            $display("AND");
                        end

                        3'b110: begin
                            $display("OR");
                        end

                        3'b100: begin
                            $display("XOR");
                        end

                        3'b001: begin
                            $display("SLL");
                        end

                    endcase
                end
            
                7'b0010011: begin // I-Type (could also include other opcodes for I-Type instructions)
                    // Set control signals for I-Type instruction

                    id_alu_op = 1;
                    id_shifter_imm = 1;
                    id_mem_ins_enable = 1;
                    id_rf_enable = 1;

                    case(func3)

                         3'b000: begin
                            $display("ADDI");
                        end

                         3'b010: begin
                            $display("SLTI");
                        end

                         3'b011: begin
                            $display("SLTIU");
                        end
                        
                         3'b111: begin
                            $display("ANDI");
                        end

                         3'b110: begin
                            $display("ORI");
                        end

                         3'b100: begin
                            $display("XORI");
                        end

                         3'b001: begin
                            $display("SLLI");
                        end

                        3'b101: begin
                            add_sub_sign = instruction[30];

                            case(add_sub_sign)
                                1'1: begin // SRAI case
                                    $display("SRAI");

                                end

                                1'0: begin // SRLI case
                                    $display("SRLI");

                                end
                            endcase
                        end
                        
                    endcase
                end

                7'b1100111: begin // jalr
                    // Set control signals for jalr instruction
                    id_alu_op = 1;
                    id_shifter_imm = 1;
                    id_mem_ins_enable = 1;
                    id_rf_enable = 1;
                    ex_jalr_sig = 1;
                         $display("JALR");
                end 

                7'b0000011: begin // I-Type
                    // Set control signals for Load Instructions instruction
                    id_alu_op = 1;
                    id_shifter_imm = 1;
                    id_rf_enable = 1;
                    id_load_inst = 1;
                    id_mem_ins_enable = 1;
                    case(func3)
                        3'b010:begin
                            size = 2'b10; //LW
                            $display("LW");
                        end
                        3'b001:begin
                            size = 2'b01; //LH
                            $display("LH");
                        end
                        3'b101:begin
                            size = 2'b01; //LHU
                            $display("LHU");
                        end
                        3'b000:begin
                            size = 2'b00; //LB
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
                    id_alu_op = 1;
                    id_shifter_imm = 1;
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
                    id_alu_op = 1;
                    id_shifter_imm = 1;
                    case(func3)

                         3'b000: begin
                            $display("BEQ");
                        end

                         3'b001: begin
                            $display("BNE");
                        end

                         3'b100: begin
                            $display("BLT");
                        end
                        
                         3'b101: begin
                            $display("BGE");
                        end

                         3'b110: begin
                            $display("BLTU");
                        end

                         3'b111: begin
                            $display("BGEU");
                        end
                    endcase 
                end
                7'b0110111: begin // U-Type (lui)
                    // Set control signals for U-Type instruction
                    //TODO: special case
                    id_shifter_imm = 1;
                    id_rf_enable = 1;
                    $display("LUI");
                end 
                
                
                7'b0010111: begin // U-Type (auipc)
                    // Set control signals for U-Type instruction
                    //TODO: special case
                    id_alu_op = 1;
                    id_shifter_imm = 1;
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
        end 
    end
endmodule

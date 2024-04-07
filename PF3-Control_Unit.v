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
        
        // if reset is assertive assign 32 bits to 0.
        //Otherwise if enable is active then
        if (rst)out <= 32'b0;
        else if (en) out <= in; 
    end
endmodule


module instruction_memory(
    input wire [8:0] address, // 9 bits address for the input
    output reg [31:0] instruction // 32 bits output.
    );

    reg [7:0] mem[511:0]; // Array with 1024 positions of 32 bits for each.
    
    //Reading the preload memory
    //If this is not working specified the whole directory of the file.
    initial begin
        $readmemb("test-code.txt", mem);
    end 
    
    //Making the arragment for the instruction
    always@(address)begin
        instruction <= {mem[address + 3], mem[address + 2], mem[address + 1], mem[address]};
    end

endmodule

//Generic pipeline register module 
//Maybe use this to create all the pipeline registers | IF/ID -> ID/EX -> EX/MEM -> MEM/WB |
module pipeline_reg #(
    parameter N = 32 // Default register size
)(
    input wire clk,
    input wire rst,
    input wire [N-1:0] in,
    output reg [N-1:0] out
);
    always @(posedge clk) begin
        if (rst) out <= {N{1'b0}};
        else out <= in;
    end
endmodule

//Control unit module
module control_unit(
    input wire [31:0] instruction,
    output reg [3:0] id_alu_op, 
    output reg [2:0] id_shifter_imm,
    output reg id_rf_enable, 
    output reg id_load_inst, 
    output reg id_mem_ins_enable, 
    output reg id_mem_write, 
    output reg [1:0] size,
    output reg [9:0] id_full_cond,
    output reg ex_jalr_sig,
    output reg id_auipc_s,
    output reg id_jal_sig
    );
    // Decode logic begins here
    always @(instruction) begin
        // Initialize all control signals to 0
        id_alu_op <= 4'b0;
        id_shifter_imm <= 3'b0;
        id_rf_enable <= 1'b0;
        id_load_inst <= 1'b0;
        id_mem_ins_enable <= 1'b0;
        id_mem_write <= 1'b0;
        size <= 2'b0;
        id_full_cond <= 10'b0;
        ex_jalr_sig <= 1'b0;
        id_auipc_s <= 1'b0;
        id_jal_sig <= 1'b0;
        
        if(instruction !=0)begin
            case(instruction[6:0]) // Check the opcode
                7'b0110011: begin // R-Type
                    // Set control signals for R-Type instruction
                    if(instruction[31:25] == 7'b0100000 && instruction == 3'b000) begin
                        id_alu_op <= 4'b0001;
                        id_rf_enable <= 1'b1;
                    end
                    //After this other cases for diferente opcodes


                end
                7'b0010011: begin // I-Type (could also include other opcodes for I-Type instructions)
                    // Set control signals for I-Type instruction
                    id_alu_op <= 4'b00000;
                    id_rf_enable <= 1'b1;
                    //Other cases for diferents opcodes 

                end
                7'b0100011: begin // S-Type
                    // Set control signals for S-Type instruction
                end
                7'b1100011: begin // B-Type
                    // Set control signals for B-Type instruction
                    // If it's a branch instruction, combine the opcode and funct3
                id_full_cond <= {instruction[6:0], instruction[14:12]};
                end
                7'b0110111, 7'b0010111: begin // U-Type (lui and auipc)
                    // Set control signals for U-Type instruction
                end
                7'b1101111: begin // J-Type
                    // Set control signals for J-Type instruction
                end
                default: begin
                    // Handle undefined opcode
                end
            endcase
        end
    end
endmodule



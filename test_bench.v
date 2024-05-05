`include "PH4-Gadiel_Mathew_Ernest.v"

module processor_tb;

    reg clk, reset;
    wire [31:0] pc_current, instruction, alu_output, id_PA_output, id_PB_output;

    wire [4:0] id_rd;
    wire [31:0] wb_mux2x1_mem_output;
    wire wb_rf_enable;
    
    // Instancia del módulo procesador
    processor uut(
        .clk(clk),
        .reset(reset)
    );

    // Generador de reloj (clock)
    initial begin
        clk = 0;
        forever #2 clk = ~clk; // Cambia el estado cada 10 ns
    end

    // Inicialización y simulación
    initial begin
        // Inicializa las señales de control
        reset = 1;
        #20; // Espera 3 ns para que el reset se propague
        reset = 0; // Desactiva el reset
        
        #500
        $finish; // Termina la simulación
    end

    // Opcional: Monitoreo de señales importantes
   initial begin
        $monitor("Time = %t, PC = %d, Instruction = %b, ALU_Out = %h, ID_PA = %d, ID_PB = %d, ID_RD = %d, WB_Output = %d, WB_Rf_Enable = %d",
                 $time,
                 uut.pc_current,
                 uut.instruction,
                 uut.ALU_inst.Out,
                 uut.registerfile_inst.PA,
                 uut.registerfile_inst.PB,
                 uut.MEM_WB_pipeline_register_inst.wb_rd,
                 uut.MEM_WB_pipeline_register_inst.wb_mux2x1_mem_output,
                 uut.MEM_WB_pipeline_register_inst.wb_rf_enable
                );
   end


endmodule
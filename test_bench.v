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

    // Generador de clock
    initial begin
        reset = 1;
        clk = 0;
        forever #2 clk = ~clk; // Cambia el estado cada 2 ns
    end

    // Inicialización y simulación
    initial begin
        #3 reset = 0; // Desactiva el reset
        
        #50$finish; // Termina la simulación
    end
    
   //Monitoreo de senales
//    always @(posedge clk) begin
//         $monitor("\n\t \n\tPC = %d, \n\tInstruction = %b, \n\tid_rd = %d, \n\tex_rd = %d, \n\tmem_rd = %d, \n\twb_rd = %d,\n\t  alu_output:%d, \n\tmem_pw = %d, \n\t pw=%d, \n\tid_load_instr:%d ,\n\t ex_load_instr:%d,  \n\t mem_load_instr:%d, \n\t id_alu_op:%b, \n\t ex_alu_op:%b, \n\t alu_a:%d , \n\t alu_b:%d, \n\t soh_pb:%d,\n\t soh_imms:%d, \n\t soh_immi:%d, \n\t soh_imm20:%d, \n\t soh_shifter:%b, \n\t id_imm12_I:%d",
                 
//                  uut.pc_current,
//                  uut.instruction,
//                  uut.id_rd,
//                  uut.ex_rd,
//                  uut.mem_rd,
//                  uut.wb_rd,
//                  uut.alu_output,
//                  uut.mux2x1_mem_output,
//                  uut.wb_mux2x1_mem_output,
//                  uut.id_load_inst,
//                  uut.ex_load_inst,
//                  uut.mem_load_inst,
//                  uut.id_alu_op,
//                  uut.ex_alu_op,
//                  uut.mux2x1_alu_input_A_output,
//                  uut.N_SOH,
//                  uut.ex_PB,
//                  uut.ex_imm12_I,
//                  uut.ex_imm12_S,
//                  uut.ex_imm20,
//                  uut.ex_shifter_imm,
//                  uut.id_imm12_I
//                 );
always @(posedge clk) begin
        $monitor("\n\t \n\tPC = %d, \n\tr1:%d, \n\tr2:%d, \n\tr3:%d, \n\tr5:%d, \n\tr6:%d", uut.pc_current,
        uut.registerfile_inst.registers1,uut.registerfile_inst.registers2,uut.registerfile_inst.registers3,
        uut.registerfile_inst.registers5,uut.registerfile_inst.registers6);


        // $display("\nTime = %t, \nPC = %d, \nInstruction = %b, \nreset = %b",
        //     $time,
        //     uut.pc_current,
        //     uut.IF_ID_pipeline_register_inst.instruction,
        //     uut.reset
        // );
        
        // $display("\nAdder PC");
        // $monitor("\tpc = %d,\n\tpc_next = %d",
        //         uut.pc_adder.pc,
        //         uut.pc_adder.pcplus4
        // );
        
        
        // $display("\nIF/ID pipeline");
        // $monitor("\treset =%b, \tLE = %b, \n\tins_mem_out = %b,\n\tPC = %d,\n\tinstruction = %b,\n\tid_pc = %d,\n\tid_imm12_S = %b, \n\tid_imm12_I = %b,\n\tid_rn = %b, id_rm = %b,\n\tid_imm20 = %b,\n\tid_rd = %b,\n\tid_pc_next = %d",
        //          uut.IF_ID_pipeline_register_inst.reset,
        //          uut.IF_ID_pipeline_register_inst.LE,
        //          uut.IF_ID_pipeline_register_inst.ins_mem_out,
        //          uut.IF_ID_pipeline_register_inst.PC,
        //          uut.IF_ID_pipeline_register_inst.instruction,
        //          uut.IF_ID_pipeline_register_inst.id_pc,
        //          uut.IF_ID_pipeline_register_inst.id_imm12_S,
        //          uut.IF_ID_pipeline_register_inst.id_imm12_I,
        //          uut.IF_ID_pipeline_register_inst.id_rn,
        //          uut.IF_ID_pipeline_register_inst.id_rm,
        //          uut.IF_ID_pipeline_register_inst.id_imm20,
        //          uut.IF_ID_pipeline_register_inst.id_rd,
        //          uut.IF_ID_pipeline_register_inst.id_pc_next        
        // );

        // $display("\nID/EX pipeline");
        // $monitor("\treset = %b,\n\tid_alu_op_mux = %b,\n\tid_shifter_imm_mux = %b,\n\tid_rf_enable_mux = %b,\n\tid_load_inst_mux = %b,\n\tid_mem_ins_enable_mux = %b,\n\tid_mem_write_mux = %b,\n\tsize_mux = %b,\n\tid_se_mux = %b,\n\tid_full_cond_mux = %b,\n\tid_jalr_sig_mux = %b,\n\tid_auipc_s_mux= %b,\n\tid_jal_sig_mux = %b, \n\tid_TA = %d,\n\tid_pc = %d,\n\tid_PA = %d,\n\tid_PB = %d,\n\tid_imm12_I = %b,\n\tid_imm12_S = %b,\n\tid_pc_next = %d,\n\tid_imm20 = %b,\n\tid_rd = %d,\n\tex_rf_enable = %b,\n\tex_alu_op= %b,\n\tex_shifter_imm = %b,\n\tex_load_inst = %b,\n\tex_mem_ins_enable = %b,\n\tex_mem_write = %b,\n\tex_size= %b,\n\tex_se = %b,\n\tex_full_cond = %b,\n\tex_jalr_sig = %b,\n\tex_auipc_s = %b,\n\tex_jal_sig = %b,\n\tex_TA = %d,\n\tex_pc = %b,\n\tex_PA = %d,\n\tex_PB = %d,\n\tex_imm12_I = %b,\n\tex_imm12_S = %b,\n\tex_pc_next = %d,\n\tex_imm20 = %b\n\tex_rd = %d",
        // uut.ID_EX_pipeline_register_inst.reset,
        // uut.ID_EX_pipeline_register_inst.id_alu_op_mux,
        // uut.ID_EX_pipeline_register_inst.id_shifter_imm_mux,
        // uut.ID_EX_pipeline_register_inst.id_rf_enable_mux,
        // uut.ID_EX_pipeline_register_inst.id_load_inst_mux,
        // uut.ID_EX_pipeline_register_inst.id_mem_ins_enable_mux,
        // uut.ID_EX_pipeline_register_inst.id_mem_write_mux,
        // uut.ID_EX_pipeline_register_inst.size_mux,
        // uut.ID_EX_pipeline_register_inst.id_se_mux,
        // uut.ID_EX_pipeline_register_inst.id_full_cond_mux,
        // uut.ID_EX_pipeline_register_inst.id_jalr_sig_mux,
        // uut.ID_EX_pipeline_register_inst.id_auipc_s_mux,
        // uut.ID_EX_pipeline_register_inst.id_jal_sig_mux,
        // uut.ID_EX_pipeline_register_inst.id_TA,
        // uut.ID_EX_pipeline_register_inst.id_pc,
        // uut.ID_EX_pipeline_register_inst.id_PA,
        // uut.ID_EX_pipeline_register_inst.id_PB,
        // uut.ID_EX_pipeline_register_inst.id_imm12_I,
        // uut.ID_EX_pipeline_register_inst.id_imm12_S,
        // uut.ID_EX_pipeline_register_inst.id_pc_next,
        // uut.ID_EX_pipeline_register_inst.id_imm20,
        // uut.ID_EX_pipeline_register_inst.id_rd,
        // uut.ID_EX_pipeline_register_inst.ex_rf_enable,
        // uut.ID_EX_pipeline_register_inst.ex_alu_op,
        // uut.ID_EX_pipeline_register_inst.ex_shifter_imm,
        // uut.ID_EX_pipeline_register_inst.ex_load_inst,
        // uut.ID_EX_pipeline_register_inst.ex_mem_ins_enable,
        // uut.ID_EX_pipeline_register_inst.ex_mem_write,
        // uut.ID_EX_pipeline_register_inst.ex_size,
        // uut.ID_EX_pipeline_register_inst.ex_se,
        // uut.ID_EX_pipeline_register_inst.ex_full_cond,
        // uut.ID_EX_pipeline_register_inst.ex_jalr_sig,
        // uut.ID_EX_pipeline_register_inst.ex_auipc_s,
        // uut.ID_EX_pipeline_register_inst.ex_jal_sig,
        // uut.ID_EX_pipeline_register_inst.ex_TA,
        // uut.ID_EX_pipeline_register_inst.ex_pc,
        // uut.ID_EX_pipeline_register_inst.ex_PA,
        // uut.ID_EX_pipeline_register_inst.ex_PB,
        // uut.ID_EX_pipeline_register_inst.ex_imm12_I,
        // uut.ID_EX_pipeline_register_inst.ex_imm12_S,
        // uut.ID_EX_pipeline_register_inst.ex_pc_next,
        // uut.ID_EX_pipeline_register_inst.ex_imm20,
        // uut.ID_EX_pipeline_register_inst.ex_rd
        // );



        // $display("ID/EX Pipeline Register"); 
        // $monitor("reset = %b, id_alu_op_mux = %b, id_shifter_imm_mux = %b, id_id_rf_enable_mux = %b, id_load_inst_mux= %b, if_load_inst_mux = %b, id_mem_ins_enable_mux = %b, size_mux = %b, id_se_mux = %b, id_full_cond_mux = %b, id_TA = %b, id_pc = %b, id_PA = %b, "

        // );

        // $display("\nInstruction Memory");
        // $monitor("\taddress = %b, \n\tinstruction = %b",
        //         uut.instruction_memory_inst.address,
        //         uut.instruction_memory_inst.instruction
        // );

        // $display("\nHazard forwarding Unit");
        // $monitor("\tid_Rn =%b,\tid_Rm= %b,\n\tex_Rd = %b, \n\tmem_Rd = %b, \n\twb_Rd = %b, \n\tex_Rf_enable = %b, \n\tmem_Rf_enable = %b,\n\twb_Rf_enable = %b,\n\tforwardA = %b,\n\tforwardB = %b,\n\tnop_signal = %b,\n\tload_enable = %b,\n\tpc_enable = %b",
        //          uut.hazard_forwarding_unit_inst.id_Rn,
        //          uut.hazard_forwarding_unit_inst.id_Rm,
        //          uut.hazard_forwarding_unit_inst.ex_Rd,
        //          uut.hazard_forwarding_unit_inst.mem_Rd,
        //          uut.hazard_forwarding_unit_inst.wb_Rd,
        //          uut.hazard_forwarding_unit_inst.ex_Rf_enable,
        //          uut.hazard_forwarding_unit_inst.mem_Rf_enable,
        //          uut.hazard_forwarding_unit_inst.wb_Rf_enable,
        //          uut.hazard_forwarding_unit_inst.forwardA,
        //          uut.hazard_forwarding_unit_inst.forwardB,
        //          uut.hazard_forwarding_unit_inst.nop_signal,
        //          uut.hazard_forwarding_unit_inst.load_enable,
        //          uut.hazard_forwarding_unit_inst.pc_enable  
        // );

        // $display("\nCondition handler");
        // $monitor("\tcontrol_hazard_out = %b, \n\tZ flag = %b, \n\tN flag = %b,\n\tex_full_cond = %b",
        //         uut.condition_handler_inst.control_hazard_out,
        //         uut.condition_handler_inst.Z_flag,
        //         uut.condition_handler_inst.N_flag,
        //         uut.condition_handler_inst.ex_full_cond
        // );

        // $display("\nProgram counter & IF_TA_MUX");
        // $monitor("\n\tenable = %b,\n\tpc = %d, \n\tadder_input = %d,\n\tadder_output = %d,\n\tinput1_mux = %d,\n\tcontrol_signal_mux = %b, \n\toutput_value_mux = %d",
                  
        //           uut.pc_reg_inst.en,
        //           uut.pc_reg_inst.out,
        //           uut.pc_current,
        //           uut.pc_next,
        //           uut.mux2x1_if_TA.input1,
        //           uut.mux2x1_if_TA.control_signal,
        //           uut.mux2x1_if_TA.output_value 
        // );

        // $display("\nif_TA MUX");
        // $monitor("\tinput0 = %d,\n\tinput1 = %d,\n\tcontrol_signal = %b, \n\toutput_value = %d",
        //         uut.mux2x1_if_TA.input0,
        //         uut.mux2x1_if_TA.input1,
        //         uut.mux2x1_if_TA.control_signal,
        //         uut.mux2x1_if_TA.output_value
        // );

        // $display("\nid_TA MUX");
        // $monitor("\tpc = %d,\n\tinput0 = %d,\n\tinput1 = %d,\n\tcontrol_signal = %b, \n\toutput_value = %d, \n\tid_jal_sig = %b, \n\tex_jalr_sig = %b,\n\tcontrol_hazard_signal = %b",
        //         uut.pc_current,
        //         uut.mux2x1_id_TA.input0,
        //         uut.mux2x1_id_TA.input1,
        //         uut.mux2x1_id_TA.control_signal,
        //         uut.mux2x1_id_TA.output_value,
        //         uut.id_jal_sig,
        //         uut.ex_jalr_sig,
        //         uut.control_hazard_signal
        // );

        // $display("\nid_adder_input MUX");
        // $monitor("\tinput0 = %d,\n\tinput1 = %d,\n\tcontrol_signal = %b, \n\toutput_value = %d",
        //         uut.mux2x1_id_adder_input.input0,
        //         uut.mux2x1_id_adder_input.input1,
        //         uut.mux2x1_id_adder_input.control_signal,
        //         uut.mux2x1_id_adder_input.output_value
        // );

        // $display("\nex_TA MUX");
        // $monitor("\tinput0 = %d,\n\tinput1 = %d,\n\tcontrol_signal = %b,\n\toutput_value = %d",
        //         uut.mux2x1_ex_TA.input0,
        //         uut.mux2x1_ex_TA.input1,
        //         uut.mux2x1_ex_TA.control_signal,
        //         uut.mux2x1_ex_TA.output_value
        // );

        // $display("\nid Adder");
        // $monitor("\tmux2x1_id_adder_input_output = %d, \n\tid_pc = %d, \n\tid_TA = %d",
        //             uut.id_Adder_inst.mux2x1_id_adder_input_output,
        //             uut.id_Adder_inst.id_pc,
        //             uut.id_Adder_inst.id_TA
        // );

        
   end


endmodule

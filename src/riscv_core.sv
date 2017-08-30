/**
 * riscv_core.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 * Daniel Stiffler & Devin Gund
 *
 *
 **/

//////
////// RISC-V 447: A single-cycle RISC-V ISA simulator
//////

`include "riscvPkg.pkg"

////
//// The RISC-V standalone processor module
////
//// clk             (input) - The clock
//// inst_addr      (output) - Address of instruction to load
//// inst            (input) - Instruction from memory
//// inst_excpt      (input) - inst_addr not valid
//// mem_addr       (output) - Address of data to load
//// mem_data_store (output) - Data for memory store
//// mem_data_load   (input) - Data from memory load
//// mem_write_en   (output) - Memory write mask
//// mem_excpt       (input) - mem_addr not valid
//// halted         (output) - Processor halted
//// rst_b           (input) - Reset the processor
////
module riscv_core #(parameter TEXT_START=`USER_TEXT_START) (
    output logic [29:0] inst_addr, mem_addr,
    output logic [31:0] mem_data_store,
    output logic [3:0] mem_write_en,
    output logic halted,
    input logic clk,
    input logic inst_excpt, mem_excpt,
    input logic [31:0] inst[3:0],
    input logic [31:0] mem_data_load,
    input logic rst_b);

  // Internal exception control signals
  logic except_ri[1:0]; // Instruction decoding exception
  logic exception_halt, syscall_halt, internal_halt; // Halt indicators
  logic load_epc, load_bva, load_bva_sel, load_ex_regs;
  logic [31:0] epc, cause, bad_v_addr;
  err_code_t cause_code;

  // PC pointers and routing control signals
  logic [31:0] pc, next_pc, pc_plus_4, pc_plus_8;
  logic [31:0] pc_1_IF, pc_plus_4_1_IF, pc_1_IF_ID, pc_plus_4_1_IF_ID;

  // ALU data signals
  abi_reg_t dcd_rs1_ID_cb[1:0], dcd_rs2_ID_cb[1:0], dcd_rd_ID_cb[1:0];
  logic [31:0] rs1_data_ID_cb[1:0], rs2_data_ID_cb[1:0], dcd_imm_ID_cb[1:0];
  logic [31:0] alu_operA[1:0], alu_operB[1:0];
  logic [31:0] rd_data_MEM_cb[1:0], rd_data_MEM[1:0], alu_out_EX_cb[1:0];
  rd_data_src_t rd_data_src_ID_cb[1:0];
  pc_src_t pc_src_1_ID_cb;

  // Branch unit signals
  logic br_taken_1_EX_cb;

  // ALU control signals
  alu_op_t alu_op_ID_cb[1:0]; // Logical, arithmatic, or compare operation
  br_type_t br_type_1_ID_cb; // Branch compare operation
  logic alu_signed_ID_cb[1:0]; // Signed-ness of ALU operation
  logic is_alu_rtype_ID_cb[1:0], is_alu_pctype_ID_cb[1:0];

  // Register file writeback signals
  logic rd_we_MEM_cb[1:0], rd_we_MEM[1:0];

  // ALU with memory signals
  mem_op_t mem_op; // Memory operation
  logic mem_signed; // Signed-ness of memory load
  mem_mask_t mem_mask; // Memory byte mask
  logic [31:0] mem_data_load_masked;

  // Forwarding signals
  fwd_src_t forward_1_A, forward_1_B, forward_2_A, forward_2_B;

  // Branch target buffer signals
  logic [61:0] btb_entry, new_btb_entry;
  logic [31:0] btb_pc_mux;
  logic [29:0] btb_rd_tag, btb_rd_next_pc;
  btb_state_t btb_rd_state, new_btb_state;
  logic [6:0] btb_rd_idx, btb_wr_idx;
  logic btb_we, update_btb, btb_hit, btb_null;

  /*--------------------------------------------------------------------------
   * 5-stage Pipeline Control Signals
   *--------------------------------------------------------------------------*/
  logic pipe_2_noex;
  logic pipe_stall, pipe_flush_if_id, pipe_1_kill_id, pipe_transfer;

  logic is_syscall;
  logic is_syscall_1_ID, is_syscall_1_ID_EX, is_syscall_1_ID_EX_MEM;
  ex_state_t ex_state_1_ID, ex_state_2_ID;
  mem_state_t mem_state_1_ID, mem_state_1_ID_EX,
              mem_state_2_ID, mem_state_2_ID_EX;
  wb_state_t wb_state_1_ID, wb_state_1_ID_EX, wb_state_1_ID_EX_MEM,
             wb_state_2_ID, wb_state_2_ID_EX, wb_state_2_ID_EX_MEM;

  /*--------------------------------------------------------------------------
   * Instruction Fetch (IF)
   *--------------------------------------------------------------------------*/
  logic [31:0] ir_IF[1:0];
  btb_state_t btb_rd_state_IF;
  logic btb_hit_IF, btb_null_IF;

  /* Look-up entry in BTB based on lowest, non-fixed bits of the PC and parse
   * the 62-bit entry as specified in the lab writeup */
  assign btb_pc_mux = pipe_transfer ? pc_plus_4_1_IF : pc;
  assign btb_rd_idx = btb_pc_mux[8:2];
  assign {btb_rd_tag, btb_rd_state, btb_rd_next_pc} = btb_entry;
  assign btb_hit = btb_rd_tag == btb_pc_mux[31:2];
  assign btb_null = btb_entry == `BTB_NULL;

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      ir_IF <= '{`INST_NOP, `INST_NOP};
      pc_1_IF <= 32'h00_00_00_00;
      pc_plus_4_1_IF <= 32'h00_00_00_00;
      btb_rd_state_IF <= SNT;
      btb_hit_IF <= 1'b0;
      btb_null_IF <= 1'b0;
    end else if (pipe_flush_if_id) begin
      ir_IF <= '{`INST_NOP, `INST_NOP};
      pc_1_IF <= 32'hxx_xx_xx_xx;
      pc_plus_4_1_IF <= 32'hxx_xx_xx_xx;
      btb_rd_state_IF <= SNT;
      btb_hit_IF <= 1'b0;
      btb_null_IF <= 1'b0;
    end else if (pipe_stall) begin
      // Do nothing
    end else if (pipe_transfer) begin
      ir_IF <= '{inst[0], ir_IF[1]};
      pc_1_IF <= pc_plus_4_1_IF;
      pc_plus_4_1_IF <= pc;
      btb_rd_state_IF <= btb_rd_state;
      btb_hit_IF <= btb_hit;
      btb_null_IF <= btb_null;
    end else begin
      ir_IF <= '{inst[1], inst[0]};
      pc_1_IF <= pc;
      pc_plus_4_1_IF <= pc_plus_4;
      btb_rd_state_IF <= btb_rd_state;
      btb_hit_IF <= btb_hit;
      btb_null_IF <= btb_null;
    end
  end

  /*--------------------------------------------------------------------------
   * Instruction Decode (ID)
   *--------------------------------------------------------------------------*/
  logic [31:0] rs1_data_ID[1:0], rs2_data_ID[1:0], dcd_imm_ID[1:0];
  logic [31:0] rs1_data_forward[1:0], rs2_data_forward[1:0];
  logic is_alu_pctype_ID[1:0];
  abi_reg_t dcd_rd_ID[1:0];
  btb_state_t btb_rd_state_IF_ID;
  logic btb_hit_IF_ID, btb_null_IF_ID;

  // Pipe 1 destination forwarding
  always_comb begin
    rs1_data_forward[0] = rs1_data_ID_cb[0];
    rs2_data_forward[0] = rs2_data_ID_cb[0];

    // Operand A forwarding
    unique case (forward_1_A)
      FWD_2_EX: rs1_data_forward[0] = alu_out_EX_cb[1];
      FWD_1_EX: rs1_data_forward[0] = alu_out_EX_cb[0];

      FWD_2_MEM: rs1_data_forward[0] = rd_data_MEM_cb[1];
      FWD_1_MEM: rs1_data_forward[0] = rd_data_MEM_cb[0];

      FWD_2_WB: rs1_data_forward[0] = rd_data_MEM[1];
      FWD_1_WB: rs1_data_forward[0] = rd_data_MEM[0];

      FWD_JMP: rs1_data_forward[0] = pc_plus_4_1_IF_ID;

      FWD_NONE: ;
    endcase

    // Operand B forwarding
    unique case (forward_1_B)
      FWD_2_EX: rs2_data_forward[0] = alu_out_EX_cb[1];
      FWD_1_EX: rs2_data_forward[0] = alu_out_EX_cb[0];

      FWD_2_MEM: rs2_data_forward[0] = rd_data_MEM_cb[1];
      FWD_1_MEM: rs2_data_forward[0] = rd_data_MEM_cb[0];

      FWD_2_WB: rs2_data_forward[0] = rd_data_MEM[1];
      FWD_1_WB: rs2_data_forward[0] = rd_data_MEM[0];

      FWD_JMP: rs2_data_forward[0] = pc_plus_4_1_IF_ID;

      FWD_NONE: ;
    endcase
  end

  // Pipe 2 destination forwarding
  always_comb begin
    rs1_data_forward[1] = rs1_data_ID_cb[1];
    rs2_data_forward[1] = rs2_data_ID_cb[1];

    // Operand A forwarding
    unique case (forward_2_A)
      FWD_2_EX: rs1_data_forward[1] = alu_out_EX_cb[1];
      FWD_1_EX: rs1_data_forward[1] = alu_out_EX_cb[0];

      FWD_2_MEM: rs1_data_forward[1] = rd_data_MEM_cb[1];
      FWD_1_MEM: rs1_data_forward[1] = rd_data_MEM_cb[0];

      FWD_2_WB: rs1_data_forward[1] = rd_data_MEM[1];
      FWD_1_WB: rs1_data_forward[1] = rd_data_MEM[0];

      FWD_JMP: ;
      FWD_NONE: ;
    endcase

    // Operand B forwarding
    unique case (forward_2_B)
      FWD_2_EX: rs2_data_forward[1] = alu_out_EX_cb[1];
      FWD_1_EX: rs2_data_forward[1] = alu_out_EX_cb[0];

      FWD_2_MEM: rs2_data_forward[1] = rd_data_MEM_cb[1];
      FWD_1_MEM: rs2_data_forward[1] = rd_data_MEM_cb[0];

      FWD_2_WB: rs2_data_forward[1] = rd_data_MEM[1];
      FWD_1_WB: rs2_data_forward[1] = rd_data_MEM[0];

      FWD_JMP: ;
      FWD_NONE: ;
    endcase
  end

  // Generate control signals from instruction decoder
  riscv_decode_first
    Decoder_1 (.inst(ir_IF[0]),
               .dcd_rs1(dcd_rs1_ID_cb[0]), .dcd_rs2(dcd_rs2_ID_cb[0]),
               .dcd_rd(dcd_rd_ID_cb[0]), .dcd_imm(dcd_imm_ID_cb[0]),
               .alu_op(alu_op_ID_cb[0]), .br_type(br_type_1_ID_cb),
               .alu_signed(alu_signed_ID_cb[0]),
               .is_alu_rtype(is_alu_rtype_ID_cb[0]),
               .is_alu_pctype(is_alu_pctype_ID_cb[0]),
               .is_syscall,
               .except_ri(except_ri[0]),
               .rd_data_src(rd_data_src_ID_cb[0]),
               .pc_src(pc_src_1_ID_cb),
               .mem_op, .mem_signed, .mem_mask);
  riscv_decode_second
    Decoder_2 (.inst(ir_IF[1]),
               .pipe_2_noex,
               .dcd_rs1(dcd_rs1_ID_cb[1]), .dcd_rs2(dcd_rs2_ID_cb[1]),
               .dcd_rd(dcd_rd_ID_cb[1]), .dcd_imm(dcd_imm_ID_cb[1]),
               .alu_op(alu_op_ID_cb[1]), .alu_signed(alu_signed_ID_cb[1]),
               .is_alu_rtype(is_alu_rtype_ID_cb[1]),
               .is_alu_pctype(is_alu_pctype_ID_cb[1]),
               .except_ri(except_ri[1]),
               .rd_data_src(rd_data_src_ID_cb[1]));

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      rs1_data_ID <= '{32'h00_00_00_00, 32'h00_00_00_00};
      rs2_data_ID <= '{32'h00_00_00_00, 32'h00_00_00_00};
      dcd_imm_ID <= '{32'h00_00_00_00, 32'h00_00_00_00};
      is_alu_pctype_ID <= '{1'b0, 1'b0};
      dcd_rd_ID <= '{REG_ZERO, REG_ZERO};
      pc_1_IF_ID <= 32'h00_00_00_00;
      pc_plus_4_1_IF_ID <= 32'h00_00_00_00;
      btb_rd_state_IF_ID <= SNT;
      btb_hit_IF_ID <= 1'b0;
      btb_null_IF_ID <= 1'b0;
    end else if (pipe_flush_if_id) begin
      rs1_data_ID <= '{32'h00_00_00_00, 32'h00_00_00_00};
      rs2_data_ID <= '{32'h00_00_00_00, 32'h00_00_00_00};
      dcd_imm_ID <= '{32'h00_00_00_00, 32'h00_00_00_00};
      is_alu_pctype_ID <= '{1'b0, 1'b0};
      dcd_rd_ID <= '{REG_ZERO, REG_ZERO};
      pc_1_IF_ID <= 32'hxx_xx_xx_xx;
      pc_plus_4_1_IF_ID <= 32'hxx_xx_xx_xx;
      btb_rd_state_IF_ID <= SNT;
      btb_hit_IF_ID <= 1'b0;
      btb_null_IF_ID <= 1'b0;
    end else if (pipe_1_kill_id) begin
      if (pipe_transfer) begin // Issue NOP to pipe 2 and normal to pipe 1
        rs1_data_ID <= '{32'h00_00_00_00, rs1_data_forward[0]};
        rs2_data_ID <= '{32'h00_00_00_00, rs2_data_forward[0]};
        dcd_imm_ID <= '{32'h00_00_00_00, dcd_imm_ID_cb[0]};
        is_alu_pctype_ID <= '{1'b0, is_alu_pctype_ID_cb[0]};
        dcd_rd_ID <= '{REG_ZERO, dcd_rd_ID_cb[0]};
        pc_1_IF_ID <= pc_1_IF;
        pc_plus_4_1_IF_ID <= pc_plus_4_1_IF;
        btb_rd_state_IF_ID <= btb_rd_state_IF;
        btb_hit_IF_ID <= btb_hit_IF;
        btb_null_IF_ID <= btb_null_IF;
      end else begin // Issue NOP to pipe 1 and normal to pipe 2
        rs1_data_ID <= '{rs1_data_forward[1], 32'h00_00_00_00};
        rs2_data_ID <= '{rs2_data_forward[1], 32'h00_00_00_00};
        dcd_imm_ID <= '{dcd_imm_ID_cb[1], 32'h00_00_00_00};
        is_alu_pctype_ID <= '{is_alu_pctype_ID_cb[1], 1'b0};
        dcd_rd_ID <= '{dcd_rd_ID_cb[1], REG_ZERO};
        pc_1_IF_ID <= 32'hxx_xx_xx_xx;
        pc_plus_4_1_IF_ID <= 32'hxx_xx_xx_xx;
        btb_rd_state_IF_ID <= SNT;
        btb_hit_IF_ID <= 1'b0;
        btb_null_IF_ID <= 1'b0;
      end
    end else if (pipe_stall) begin
      rs1_data_ID <= '{32'h00_00_00_00, 32'h00_00_00_00};
      rs2_data_ID <= '{32'h00_00_00_00, 32'h00_00_00_00};
      dcd_imm_ID <= '{32'h00_00_00_00, 32'h00_00_00_00};
      is_alu_pctype_ID <= '{1'b0, 1'b0};
      dcd_rd_ID <= '{REG_ZERO, REG_ZERO};
      pc_1_IF_ID <= 32'hxx_xx_xx_xx;
      pc_plus_4_1_IF_ID <= 32'hxx_xx_xx_xx;
      btb_rd_state_IF_ID <= SNT;
      btb_hit_IF_ID <= 1'b0;
      btb_null_IF_ID <= 1'b0;
    end else if (pipe_transfer) begin
      rs1_data_ID <= '{32'h00_00_00_00, rs1_data_forward[0]};
      rs2_data_ID <= '{32'h00_00_00_00, rs2_data_forward[0]};
      dcd_imm_ID <= '{32'h00_00_00_00, dcd_imm_ID_cb[0]};
      is_alu_pctype_ID <= '{1'b0, is_alu_pctype_ID_cb[0]};
      dcd_rd_ID <= '{REG_ZERO, dcd_rd_ID_cb[0]};
      pc_1_IF_ID <= pc_1_IF;
      pc_plus_4_1_IF_ID <= pc_plus_4_1_IF;
      btb_rd_state_IF_ID <= btb_rd_state_IF;
      btb_hit_IF_ID <= btb_hit_IF;
      btb_null_IF_ID <= btb_null_IF;
    end else begin
      rs1_data_ID <= rs1_data_forward;
      rs2_data_ID <= rs2_data_forward;
      dcd_imm_ID <= dcd_imm_ID_cb;
      is_alu_pctype_ID <= is_alu_pctype_ID_cb;
      dcd_rd_ID <= dcd_rd_ID_cb;
      pc_1_IF_ID <= pc_1_IF;
      pc_plus_4_1_IF_ID <= pc_plus_4_1_IF;
      btb_rd_state_IF_ID <= btb_rd_state_IF;
      btb_hit_IF_ID <= btb_hit_IF;
      btb_null_IF_ID <= btb_null_IF;
    end
  end

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      is_syscall_1_ID <= 1'b0;
      ex_state_1_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                         alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                         pc_src: PC_SRC_IMPLICIT};
      ex_state_2_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                         alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                         pc_src: PC_SRC_IMPLICIT};
      mem_state_1_ID <= '{mem_op: MEM_SLEEP, mem_signed: 1'b0,
                          mem_mask: MASK_NONE};
      wb_state_1_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
      wb_state_2_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
    end else if (pipe_flush_if_id) begin
      is_syscall_1_ID <= 1'b0;
      ex_state_1_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                         alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                         pc_src: PC_SRC_IMPLICIT};
      ex_state_2_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                         alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                         pc_src: PC_SRC_IMPLICIT};
      mem_state_1_ID <= '{mem_op: MEM_SLEEP, mem_signed: 1'b0,
                          mem_mask: MASK_NONE};
      wb_state_1_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
      wb_state_2_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
    end else if (pipe_1_kill_id) begin
      if (pipe_transfer) begin // Issue NOPs to both pipes
        is_syscall_1_ID <= 1'b0;
        ex_state_1_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                           alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                           pc_src: PC_SRC_IMPLICIT};
        ex_state_2_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                           alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                           pc_src: PC_SRC_IMPLICIT};
        mem_state_1_ID <= '{mem_op: MEM_SLEEP, mem_signed: 1'b0,
                            mem_mask: MASK_NONE};
        wb_state_1_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
        wb_state_2_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
        is_syscall_1_ID <= 1'b0;
      end else begin // Issue NOP to pipe 1 and normal to pipe 2
        is_syscall_1_ID <= 1'b0;
        ex_state_1_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                           alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                           pc_src: PC_SRC_IMPLICIT};
        ex_state_2_ID <= '{is_alu_rtype: is_alu_rtype_ID_cb[1],
                           is_alu_pctype: is_alu_pctype_ID_cb[1],
                           alu_op: alu_op_ID_cb[1], br_type: NOBRANCH,
                           alu_signed: alu_signed_ID_cb[1],
                           pc_src: PC_SRC_IMPLICIT};
        mem_state_1_ID <= '{mem_op: MEM_SLEEP, mem_signed: 1'b0,
                            mem_mask: MASK_NONE};
        wb_state_1_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
        wb_state_2_ID <= '{rd_data_src: rd_data_src_ID_cb[1], is_nop: 1'b0};
      end
    end else if (pipe_stall) begin
      is_syscall_1_ID <= 1'b0;
      ex_state_1_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                         alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                         pc_src: PC_SRC_IMPLICIT};
      ex_state_2_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                         alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                         pc_src: PC_SRC_IMPLICIT};
      mem_state_1_ID <= '{mem_op: MEM_SLEEP, mem_signed: 1'b0,
                          mem_mask: MASK_NONE};
      wb_state_1_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
      wb_state_2_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
    end else if (pipe_transfer) begin
      is_syscall_1_ID <= is_syscall;
      ex_state_1_ID <= '{is_alu_rtype: is_alu_rtype_ID_cb[0],
                         is_alu_pctype: is_alu_pctype_ID_cb[0],
                         alu_op: alu_op_ID_cb[0], br_type: br_type_1_ID_cb,
                         alu_signed: alu_signed_ID_cb[0],
                         pc_src: pc_src_1_ID_cb};
      ex_state_2_ID <= '{is_alu_rtype: 1'b0, is_alu_pctype: 1'b0,
                         alu_op: ALU_NOP, br_type: NOBRANCH, alu_signed: 1'b0,
                         pc_src: PC_SRC_IMPLICIT};
      mem_state_1_ID <= '{mem_op: mem_op, mem_signed: mem_signed,
                          mem_mask: mem_mask};
      wb_state_1_ID <= '{rd_data_src: rd_data_src_ID_cb[0], is_nop: 1'b0};
      wb_state_2_ID <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
    end else begin
      is_syscall_1_ID <= is_syscall;
      ex_state_1_ID <= '{is_alu_rtype: is_alu_rtype_ID_cb[0],
                         is_alu_pctype: is_alu_pctype_ID_cb[0],
                         alu_op: alu_op_ID_cb[0], br_type: br_type_1_ID_cb,
                         alu_signed: alu_signed_ID_cb[0],
                         pc_src: pc_src_1_ID_cb};
      ex_state_2_ID <= '{is_alu_rtype: is_alu_rtype_ID_cb[1],
                         is_alu_pctype: is_alu_pctype_ID_cb[1],
                         alu_op: alu_op_ID_cb[1], br_type: NOBRANCH,
                         alu_signed: alu_signed_ID_cb[1],
                         pc_src: PC_SRC_IMPLICIT};
      mem_state_1_ID <= '{mem_op: mem_op, mem_signed: mem_signed,
                          mem_mask: mem_mask};
      wb_state_1_ID <= '{rd_data_src: rd_data_src_ID_cb[0], is_nop: 1'b0};
      wb_state_2_ID <= '{rd_data_src: rd_data_src_ID_cb[1], is_nop: 1'b0};
    end
  end

  /*--------------------------------------------------------------------------
   * Execute (EX)
   *--------------------------------------------------------------------------*/
  logic [31:0] rs2_data_ID_EX[1:0], alu_out_EX[1:0], pc_1_IF_ID_EX,
               pc_plus_4_1_IF_ID_EX;
  abi_reg_t dcd_rd_ID_EX[1:0];

  assign alu_operA[0] = ex_state_1_ID.is_alu_pctype
                        ? pc_1_IF_ID : rs1_data_ID[0];
  assign alu_operA[1] = ex_state_2_ID.is_alu_pctype
                        ? pc_plus_4_1_IF_ID : rs1_data_ID[1];
  assign alu_operB[0] = ex_state_1_ID.is_alu_rtype
                        ? rs2_data_ID[0] : dcd_imm_ID[0];
  assign alu_operB[1] = ex_state_2_ID.is_alu_rtype
                        ? rs2_data_ID[1] : dcd_imm_ID[1];

  // Execute, select based on the current instruction
  riscv_ALU
    ALU_1 (.alu_operA(alu_operA[0]), .alu_operB(alu_operB[0]),
           .alu_op(ex_state_1_ID.alu_op), .alu_signed(ex_state_1_ID.alu_signed),
           .alu_out(alu_out_EX_cb[0])),
    ALU_2 (.alu_operA(alu_operA[1]), .alu_operB(alu_operB[1]),
           .alu_op(ex_state_2_ID.alu_op), .alu_signed(ex_state_2_ID.alu_signed),
           .alu_out(alu_out_EX_cb[1]));

  riscv_BranchCmp
    BranchUnit_1 (.br_type(ex_state_1_ID.br_type),
                  .br_operA(rs1_data_ID[0]), .br_operB(rs2_data_ID[0]),
                  .br_taken(br_taken_1_EX_cb));

  // Update entry in BTB as 62-bit entry specified in the lab writeup
  always_comb begin
    btb_wr_idx = '0;
    btb_we = 1'b0;
    new_btb_entry = `BTB_NULL;

    /* Write the new BTB entry if there was a hit and the entry changed, or if */
    /* the original entry was null */
    if (update_btb) begin
      btb_wr_idx = pc_1_IF_ID[8:2];
      btb_we = 1'b1;
      new_btb_entry = {pc_1_IF_ID[31:2], new_btb_state, alu_out_EX_cb[0][31:2]};
    end
  end

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      rs2_data_ID_EX <= '{32'h00_00_00_00, 32'h00_00_00_00};
      alu_out_EX <= '{32'h00_00_00_00, 32'h00_00_00_00};
      dcd_rd_ID_EX <= '{REG_ZERO, REG_ZERO};
      pc_plus_4_1_IF_ID_EX <= 32'h00_00_00_00;
    end else begin
      rs2_data_ID_EX <= rs2_data_ID;
      alu_out_EX <= alu_out_EX_cb;
      dcd_rd_ID_EX <= dcd_rd_ID;
      pc_plus_4_1_IF_ID_EX <= pc_plus_4_1_IF_ID;
    end
  end

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      is_syscall_1_ID_EX <= 1'b0;
      mem_state_1_ID_EX <= '{mem_op: MEM_SLEEP, mem_signed: 1'b0,
                             mem_mask: MASK_NONE};
      wb_state_1_ID_EX <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
      wb_state_2_ID_EX <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
    end else begin
      is_syscall_1_ID_EX <= is_syscall_1_ID;
      mem_state_1_ID_EX <= mem_state_1_ID;
      wb_state_1_ID_EX <= wb_state_1_ID;
      wb_state_2_ID_EX <= wb_state_2_ID;
    end
  end

  /*--------------------------------------------------------------------------
   * Memory (MEM)
   *--------------------------------------------------------------------------*/
  abi_reg_t dcd_rd_ID_EX_MEM[1:0];

  /* Calculate mem_addr and swizzle rs2_data bytes for unaligned stores and
   * loads */
  riscv_Swizzler
    Swizzler (.mem_op(mem_state_1_ID_EX.mem_op),
              .mem_signed(mem_state_1_ID_EX.mem_signed),
              .mem_mask(mem_state_1_ID_EX.mem_mask),
              .data_store(rs2_data_ID_EX[0]), .addr(alu_out_EX[0]),
              .mem_data_load, .mem_write_en, .mem_addr, .mem_data_store,
              .mem_data_load_masked);

  // Pipe 1 writeback destination
  always_comb begin
    rd_data_MEM_cb[0] = 32'hxx_xx_xx_xx;
    rd_we_MEM_cb[0] = 1'b0;

    unique case (wb_state_1_ID_EX.rd_data_src)
      RD_PC_PLUS_4: begin
        rd_data_MEM_cb[0] = pc_plus_4_1_IF_ID_EX;
        rd_we_MEM_cb[0] = 1'b1;
      end

      RD_ALU: begin
        rd_data_MEM_cb[0] = alu_out_EX[0];
        rd_we_MEM_cb[0] = 1'b1;
      end

      RD_MEM: begin
        rd_data_MEM_cb[0] = mem_data_load_masked;
        rd_we_MEM_cb[0] = 1'b1;
      end

      RD_SYS: begin
        rd_data_MEM_cb[0] = alu_out_EX[0];
      end

      RD_NONE: ;
    endcase
  end

  // Pipe 2 writeback destination
  always_comb begin
    rd_data_MEM_cb[1] = 32'hxx_xx_xx_xx;
    rd_we_MEM_cb[1] = 1'b0;

    unique case (wb_state_2_ID_EX.rd_data_src)
      RD_ALU: begin
        rd_data_MEM_cb[1] = alu_out_EX[1];
        rd_we_MEM_cb[1] = 1'b1;
      end

      RD_PC_PLUS_4, RD_SYS, RD_MEM, RD_NONE: begin
      end
    endcase
  end

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      rd_data_MEM <= '{32'h00_00_00_00, 32'h00_00_00_00};
      rd_we_MEM <= '{1'b0, 1'b0};
      dcd_rd_ID_EX_MEM <= '{REG_ZERO, REG_ZERO};
    end else begin
      rd_data_MEM <= rd_data_MEM_cb;
      rd_we_MEM <= rd_we_MEM_cb;
      dcd_rd_ID_EX_MEM <= dcd_rd_ID_EX;
    end
  end

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      is_syscall_1_ID_EX_MEM <= 1'b0;
      wb_state_1_ID_EX_MEM <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
      wb_state_2_ID_EX_MEM <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
    end else begin
      is_syscall_1_ID_EX_MEM <= is_syscall_1_ID_EX;
      wb_state_1_ID_EX_MEM <= wb_state_1_ID_EX;
      wb_state_2_ID_EX_MEM <= wb_state_2_ID_EX;
    end
  end

  /*--------------------------------------------------------------------------
   * Register Writeback (WB)
   *--------------------------------------------------------------------------*/
  wb_state_t wb_state_1_ID_EX_MEM_WB, wb_state_2_ID_EX_MEM_WB; // Debugging only

  // Debugging only
  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      wb_state_1_ID_EX_MEM_WB <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
      wb_state_2_ID_EX_MEM_WB <= '{rd_data_src: RD_NONE, is_nop: 1'b1};
    end else begin
      wb_state_1_ID_EX_MEM_WB <= wb_state_1_ID_EX_MEM;
      wb_state_2_ID_EX_MEM_WB <= wb_state_1_ID_EX_MEM;
    end
  end

  /*--------------------------------------------------------------------------
   * Architectural State
   *--------------------------------------------------------------------------*/
  assign inst_addr = pc[31:2];
  assign pc_plus_4 = pc + 4;
  assign pc_plus_8 = pc + 8;

  // PC management and IF/ID flush control
  riscv_PCManager
    PCManagerUnit (.*);

  Register #(.WIDTH(32), .RESET_VALUE(TEXT_START))
    PCReg(.clk, .rst_b,
          .q(pc), .d(next_pc),
          .en(~internal_halt & (~pipe_stall | pipe_flush_if_id)));

  // Register file; unpacked port[1] dominates on write
  regfile2
    RegisterFile (.clk, .rst_b, .halted,
                  .rs1_data(rs1_data_ID_cb), .rs2_data(rs2_data_ID_cb),
                  .rs1_num(dcd_rs1_ID_cb), .rs2_num(dcd_rs2_ID_cb),
                  .rd_num(dcd_rd_ID_EX_MEM), .rd_data(rd_data_MEM),
                  .rd_we(rd_we_MEM));

  // Branch target buffer {tagPC[31:2], history[1:0], nextPC[31:2]}
  btbsram # (.WIDTH(62), .SIZE_LG2(7))
    BTB (.rd_data(btb_entry),
         .rd_idx(btb_rd_idx), .wr_idx(btb_wr_idx), .wr_data(new_btb_entry),
         .wr_we(btb_we), .clk, .rst_b);

  riscv_Stall
    StallUnit (.*);

  // Determine register data forwarding to prevent RAW hazards
  riscv_Forward
    ForwardUnit (.*);

  // Record dynamic branch data
  riscv_Stats
    StatsUnit (.*);

  /*-------------------------------------------------------------------------
   * Architectural State (Exceptional Control Flow)
   *--------------------------------------------------------------------------*/
  logic exc_rsv_inst = except_ri[0] | except_ri[1];
  logic exc_addr_mal = pc[1:0] ? 1'b1 : 1'b0;

  assign internal_halt = exception_halt | syscall_halt;

  syscall_unit
    SyscallUnit (.syscall_halt, .pc, .clk, .Sys(is_syscall_1_ID_EX_MEM),
                 .r_a0(rd_data_MEM[0]), .rst_b);

  // Miscellaneous signals (Exceptions, syscalls, and halt)
  exception_unit
    ExceptUnit (.exception_halt, .pc, .rst_b, .clk,
                .load_ex_regs,
                .load_bva, .load_bva_sel,
                .cause(cause_code),
                .IBE(inst_excpt), // Instruction bus error
                .DBE(1'b0), // Data bus error
                /* .RI(exc_rsv_inst), // Reserved instruction */
                .RI(1'b0), // Reserved instruction
                .Ov(1'b0), // Overflow (MIPS remnant)
                .BP(1'b0), // Branch delay slot flag (MIPS remnant)
                /* .AdEL_inst(exc_addr_mal), // Instruction address misaligned */
                .AdEL_inst(1'b0), // Instruction address misaligned
                .AdEL_data(1'b0), // Data address misaligned (MIPS remnant)
                .AdES(1'b0),
                .CpU(1'b0));

  Register #(.WIDTH(1))
    HaltReg (.clk, .rst_b, .en(1'b1), .d(internal_halt), .q(halted));
  Register #(.WIDTH(32))
    EPCReg (.clk, .rst_b, .en(load_ex_regs), .d(pc), .q(epc)),
    CauseReg (.clk, .rst_b, .en(load_ex_regs),
              .d({25'h0_00_00_00, cause_code, 2'b00}),
              .q(cause)),
    BadVAddrReg (.clk, .rst_b, .en(load_bva), .d(pc), .q(bad_v_addr));

  /*--------------------------------------------------------------------------
   * Simulation
   *--------------------------------------------------------------------------*/
  `ifdef SIMULATION_18447
  /*   initial begin */
  /*     $monitor({"=== Simulation Cycle %0t ===\n", */
  /*               "[ARCH pc=%0x next_pc=%0x halted=%b]\n\n", */

  /*               "[STALL (pipe_flush_if_id=%b pipe_1_kill_id=%b) > ", */
  /*               "pipe_stall=%b > pipe_transfer=%b]\n\n", */

  /*               "[SYS1 ID=%b EX=%b MEM=%b WB=%b]\n\n", */

  /*               "[FWD forward_1_A=%s forward_1_B=%s forward_2_A=%s ", */
  /*               "forward_2_B=%s]\n\n", */

  /*               "[BTB(rd) btb_rd_tag=%0x btb_null=%b btb_hit=%b ", */
  /*               "btb_rd_state=%s]\n\n", */

  /*               "[BTB(wr) btb_we=%b new_btb_state=%s]\n\n", */

  /*               "[PC (reg) pc_1_IF=%0x pc_1_IF_ID=%0x ", */
  /*               "pc_plus_4_1_IF_ID_EX=%0x]\n\n", */

  /*               "[IF1 ir_IF=%x]\n", */
  /*               "[IF2 ir_IF=%x]\n\n", */

  /*               "[ID1 op=%s br_type=%s rs1=%s rs2=%s rd=%s imm=%0x]\n", */
  /*               "[ID2 op=%s rs1=%s rs2=%s rd=%s imm=%0x]\n\n", */

  /*               "[EX1 alu_op=%s alu_operA=%0x alu_operB=%0x alu_out=%0x ", */
  /*               "br_type=%s rs1_data=%0x rs2_data=%0x ", */
  /*               "dcd_rd=%s pc_src=%s]\n", */
  /*               "[EX2 alu_op=%s alu_operA=%0x alu_operB=%0x alu_out=%0x ", */
  /*               "rs1_data=%0x rs2_data=%0x dcd_rd=%s]\n\n", */

  /*               "[MEM1 mem_addr=%x mem_op=%s mem_mask=%s mem_data_store=%x ", */
  /*               "mem_data_load_masked=%x rd_data_src=%s rd_data=%0x]\n", */
  /*               "[MEM2 rd_data_src=%s rd_data=%0x]\n\n", */

  /*               "[WB1 dcd_rd=%s rd_data_src=%s rd_data=%0x]\n", */
  /*               "[WB2 dcd_rd=%s rd_data_src=%s rd_data=%0x]\n", */
  /*               "============================\n\n"}, */
  /*               $time, */
  /*               pc, next_pc, halted, */

  /*               pipe_flush_if_id, pipe_1_kill_id, pipe_stall, pipe_transfer, */

  /*               is_syscall, is_syscall_1_ID, is_syscall_1_ID_EX, */
  /*               is_syscall_1_ID_EX_MEM, */

  /*               forward_1_A, forward_1_B, forward_2_A, forward_2_B, */

  /*               btb_rd_tag, btb_null, btb_hit, btb_rd_state, */
  /*               btb_we, new_btb_state, */

  /*               pc_1_IF, pc_1_IF_ID, pc_plus_4_1_IF_ID_EX, */

  /*               ir_IF[0], */
  /*               ir_IF[1], */

  /*               Decoder_1.opcode, br_type_1_ID_cb, dcd_rs1_ID_cb[0], */
  /*               dcd_rs2_ID_cb[0], dcd_rd_ID_cb[0], dcd_imm_ID_cb[0], */
  /*               Decoder_2.opcode, dcd_rs1_ID_cb[1], dcd_rs2_ID_cb[1], */
  /*               dcd_rd_ID_cb[1], dcd_imm_ID_cb[1], */

  /*               ex_state_1_ID.alu_op, alu_operA[0], alu_operB[0], */
  /*               alu_out_EX_cb[0], ex_state_1_ID.br_type, rs1_data_ID[0], */
  /*               rs2_data_ID[0], dcd_rd_ID[0], ex_state_1_ID.pc_src, */
  /*               ex_state_2_ID.alu_op, alu_operA[1], alu_operB[1], */
  /*               alu_out_EX_cb[1], rs1_data_ID[1], rs2_data_ID[1], dcd_rd_ID[1], */

  /*               mem_addr, mem_state_1_ID_EX.mem_op, mem_state_1_ID_EX.mem_mask, */
  /*               mem_data_store, mem_data_load_masked, */
  /*               wb_state_1_ID_EX.rd_data_src, rd_data_MEM_cb[0], */
  /*               wb_state_2_ID_EX.rd_data_src, rd_data_MEM_cb[1], */

  /*               dcd_rd_ID_EX_MEM[0], wb_state_1_ID_EX_MEM.rd_data_src, */
  /*               rd_data_MEM[0], */
  /*               dcd_rd_ID_EX_MEM[1], wb_state_2_ID_EX_MEM.rd_data_src, */
  /*               rd_data_MEM[1]); */
  /*   end */

    final begin
      $display({"=== Benchmark Statistics ===\n",
               "stat_cycles=%0d stat_fetched=%0d stat_executed=%0d\n",
               "stat_id_2=%0d stat_id_1=%0d\n",
               "stat_id_1_alu=%0d stat_id_1_not_alu=%0d stat_id_1_valid=%0d\n",
               "stat_id_0=%0d stat_id_0_hazard=%0d stat_id_0_valid=%0d\n",
               "stat_cf_executed=%0d stat_cf_miss=%0d\n"},
               StatsUnit.stat_cycles,
               StatsUnit.stat_fetched, StatsUnit.stat_executed,
               StatsUnit.stat_id_2, StatsUnit.stat_id_1,
               StatsUnit.stat_id_1_alu, StatsUnit.stat_id_1_not_alu,
               StatsUnit.stat_id_1_valid, StatsUnit.stat_id_0,
               StatsUnit.stat_id_0_hazard, StatsUnit.stat_id_0_valid,
               StatsUnit.stat_cf_executed, StatsUnit.stat_cf_miss);
    end
  `endif
endmodule : riscv_core

////
//// riscv_ALU: Performs all arithmetic and logical operations
////
//// alu_out   (output) - Final result of arithmetic, logic, or comparison
//// alu_operA    (input) - Operand modified by the operation
//// alu_operB    (input) - Operand used (in arithmetic ops) to modify in1
//// alu_op     (input) - Selects which operation is to be performed
//// alu_signed (input) - Selects unsigned or signed comparisions
////
module riscv_ALU (
    output logic [31:0] alu_out,
    input logic [31:0] alu_operA, alu_operB,
    input alu_op_t alu_op,
    input logic alu_signed);

  always_comb begin
    alu_out = 32'hxx_xx_xx_xx;

    unique case (alu_op)
      ALU_SL: begin
        alu_out = alu_operA << alu_operB[4:0];
      end

      ALU_SR: begin
        if (alu_signed) begin
          alu_out = $signed(alu_operA) >>> alu_operB[4:0]; // ALU_SRA
        end else begin
          alu_out = alu_operA >> alu_operB[4:0]; // ALU_SRL
        end
      end

      ALU_ADD: begin
        alu_out = alu_operA + alu_operB;
      end

      ALU_SUB: begin
        alu_out = alu_operA - alu_operB;
      end

      ALU_XOR: begin
        alu_out = alu_operA ^ alu_operB;
      end

      ALU_OR: begin
        alu_out = alu_operA | alu_operB;
      end

      ALU_AND: begin
        alu_out = alu_operA & alu_operB;
      end

      ALU_LT: begin
        if (alu_signed) begin // ALU_SLT
          alu_out = $signed(alu_operA) < $signed(alu_operB);
        end else begin // ALU_SLTU
          alu_out = alu_operA < alu_operB;
        end
      end

      ALU_NOP: ;
    endcase
  end
endmodule : riscv_ALU

module riscv_BranchCmp (
    input br_type_t br_type,
    input logic [31:0] br_operA, br_operB,
    output logic br_taken);

  logic bcond_eq, bcond_lt, bcond_ltu;

  assign bcond_eq = br_operA == br_operB;
  assign bcond_lt = $signed(br_operA) < $signed(br_operB);
  assign bcond_ltu = br_operA < br_operB;

  always_comb begin
    br_taken = 1'b0;

    unique case (br_type)
      BEQ: br_taken = bcond_eq;
      BNE: br_taken = ~bcond_eq;
      BLT: br_taken = bcond_lt;
      BGE: br_taken = ~bcond_lt;
      BLTU: br_taken = bcond_ltu;
      BGEU: br_taken = ~bcond_ltu;
      NOBRANCH: ;
    endcase
  end
endmodule : riscv_BranchCmp


//// Register: A register which may be reset to an arbitrary value
////
//// q     (output)  - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value
//// reset  (input)  - System reset
////
module Register #(parameter WIDTH=32, RESET_VALUE=0) (
    input logic clk, en, rst_b,
    input logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, negedge rst_b) begin
      if (~rst_b) q <= RESET_VALUE;
      else if (en) q <= d;
  end
endmodule : Register

////
//// RISC-V byte swizzler for unaligned stores and loads
////
//// mem_op               (input)  - Memory operation
//// mem_signed           (input)  - Signed memory loads
//// mem_mask             (input)  - Byte mask for swizzling
//// rs2_data             (input)  - Write data
//// alu_out              (input)  - Memory address aligned to opcode size
//// mem_data_load        (input)  - Data memory input
//// mem_write_en         (output) - Memory write mask
//// mem_addr             (output) - Data memory address
//// mem_data_store       (output) - Swizzled data to write to memory
//// mem_data_load_masked (output) - Swizzled data from memory
module riscv_Swizzler (
    input mem_op_t mem_op,
    input logic mem_signed,
    input mem_mask_t mem_mask,
    input logic [31:0] data_store, addr,
    input logic [31:0] mem_data_load,
    output logic [3:0] mem_write_en,
    output logic [29:0] mem_addr,
    output logic [31:0] mem_data_store, mem_data_load_masked);

  always_comb begin
    mem_addr = 30'h00_00_00_00; // Memory module has no sleep function
    mem_data_store = 32'h00_00_00_00;
    mem_data_load_masked = 32'hxx_xx_xx_xx;
    mem_write_en = 4'b0000;

    unique case (mem_op)
      MEM_RD: begin
        mem_addr = addr[31:2];

        unique case (mem_mask)
          MASK_BYTE: begin
            unique case (addr[1:0])
              2'b00: begin // Byte 0
                mem_data_load_masked = {
                    {24{mem_signed & mem_data_load[7]}}, mem_data_load[7:0]};
              end

              2'b01: begin // Byte 1
                mem_data_load_masked = {
                    {24{mem_signed & mem_data_load[15]}}, mem_data_load[15:8]};
              end

              2'b10: begin // Byte 2
                mem_data_load_masked = {
                    {24{mem_signed & mem_data_load[23]}}, mem_data_load[23:16]};
              end

              2'b11: begin // Byte 3
                mem_data_load_masked = {
                    {24{mem_signed & mem_data_load[31]}}, mem_data_load[31:24]};
              end
            endcase
          end

          MASK_HALF: begin
            unique case (addr[1:0])
              2'b00: begin // Half 0
                mem_data_load_masked = {
                    {16{mem_signed & mem_data_load[15]}}, mem_data_load[15:0]};
              end

              2'b10: begin // Half 1
                mem_data_load_masked = {
                    {16{mem_signed & mem_data_load[31]}}, mem_data_load[31:16]};
              end
           endcase
          end

          MASK_WORD: begin
            mem_data_load_masked = mem_data_load;
          end
        endcase
      end

      MEM_WR: begin
        mem_addr = addr[31:2];

        unique case (mem_mask)
          MASK_BYTE: begin
            unique case (addr[1:0])
              2'b00: begin // Byte 0
                mem_write_en = 4'b0001;
                mem_data_store = {24'h00_00_00, data_store[7:0]};
              end

              2'b01: begin // Byte 1
                mem_write_en = 4'b0010;
                mem_data_store = {16'h00_00, data_store[7:0], 8'h00};
              end

              2'b10: begin // Byte 2
                mem_write_en = 4'b0100;
                mem_data_store = {8'h00, data_store[7:0], 16'h00_00};
              end

              2'b11: begin // Byte 3
                mem_write_en = 4'b1000;
                mem_data_store = {data_store[7:0], 24'h00_00};
              end
            endcase
          end

          MASK_HALF: begin
            unique case (addr[1:0])
              2'b00: begin // Half 0
                mem_write_en = 4'b0011;
                mem_data_store = {16'h00_00, data_store[15:0]};
              end

              2'b10: begin // Half 1
                mem_write_en = 4'b1100;
                mem_data_store = {data_store[15:0], 16'h00_00};
              end
            endcase
          end

          MASK_WORD: begin
            mem_write_en = 4'b1111;
            mem_data_store = data_store;
          end
        endcase
      end

      MEM_SLEEP: ;
    endcase
  end
endmodule : riscv_Swizzler

module riscv_Stall (
    input abi_reg_t dcd_rs1_ID_cb[1:0], dcd_rs2_ID_cb[1:0], dcd_rd_ID_cb[1:0],
                    dcd_rd_ID[1:0],
    input mem_op_t mem_op,
    input mem_state_t mem_state_1_ID,
    input rd_data_src_t rd_data_src_ID_cb[1:0],
    input pc_src_t pc_src_1_ID_cb,
    input logic pipe_2_noex,
    input logic is_syscall, is_syscall_1_ID, is_syscall_1_ID_EX,
                is_syscall_1_ID_EX_MEM,
    output logic pipe_stall, pipe_transfer);

  logic syscall_stall;
  logic pipe_1_stall, pipe_2_stall;

  /* Stall for syscalls only after they have left ID. The possible paths are
   * as follows:
   *   1. PIPE1=SYS PIPE2=any -> pipe_transfer -> syscall_stall
   *   2. PIPE1=any PIPE2=SYS -> pipe_transfer -> syscall_stall
   */
  assign syscall_stall = is_syscall_1_ID | is_syscall_1_ID_EX
                         | is_syscall_1_ID_EX_MEM;
  assign pipe_stall = pipe_1_stall | syscall_stall;
  assign pipe_transfer = ~pipe_1_stall
                         & (pipe_2_stall | pipe_2_noex | is_syscall);

  /* Pipe 1 hazard resolution:
   * Load RAW hazard (distance 1): p1->p1 */
  always_comb begin
    pipe_1_stall = 1'b0;

    // Operand A stalling
    if (dcd_rs1_ID_cb[0] != REG_ZERO) begin
      if (dcd_rs1_ID_cb[0] == dcd_rd_ID[0]
          && mem_state_1_ID.mem_op == MEM_RD) begin
        pipe_1_stall = 1'b1;
      end
    end

    // Operand B stalling
    if (dcd_rs2_ID_cb[0] != REG_ZERO) begin
      if (dcd_rs2_ID_cb[0] == dcd_rd_ID[0]
          && mem_state_1_ID.mem_op == MEM_RD) begin
        pipe_1_stall = 1'b1;
      end
    end
  end

  /* Pipe 2 hazard resolution
   *  Simultaneous issue hazard (distance 1): p1->p2
   *  Load RAW hazard (distance 1): p1->p2 */
  always_comb begin
    pipe_2_stall = 1'b0;

    // Operand A stalling
    if (dcd_rs1_ID_cb[1] != REG_ZERO) begin
      if (dcd_rs1_ID_cb[1] == dcd_rd_ID_cb[0]
          && rd_data_src_ID_cb[0] != RD_NONE) begin
        pipe_2_stall = 1'b1;
      end else if (dcd_rs1_ID_cb[1] == dcd_rd_ID[0]
                   && mem_state_1_ID.mem_op == MEM_RD) begin
        pipe_2_stall = 1'b1;
      end
    end

    // Operand B stalling
    if (dcd_rs2_ID_cb[1] != REG_ZERO) begin
      if (dcd_rs2_ID_cb[1] == dcd_rd_ID_cb[0]
          && rd_data_src_ID_cb[0] != RD_NONE) begin
        pipe_2_stall = 1'b1;
      end else if (dcd_rs2_ID_cb[1] == dcd_rd_ID[0]
                   && mem_state_1_ID.mem_op == MEM_RD) begin
        pipe_2_stall = 1'b1;
      end
    end

    // Control flow stalling for simplification
    if (pc_src_1_ID_cb != PC_SRC_IMPLICIT) pipe_2_stall = 1'b1;
  end
endmodule : riscv_Stall

////
//// Forwarding unit to detect and resolve potential RAW hazards
////
//// dcd_rs1_ID_cb            (input) - rs1 register in ID stage
//// dcd_rs2_ID_cb            (input) - rs2 register in ID stage
//// dcd_rd_ID          (input) - rd register in EX stage
//// dcd_rd_ID_EX       (input) - rd register in MEM stage
//// dcd_rd_ID_EX_MEM   (input) - rd register in WB stage
//// wb_state_1_ID        (input) - writeback control in EX stage (pipe 1)
//// wp_state_1_ID_EX     (input) - writeback control in MEM stage (pipe 1)
//// wb_state_1_ID_EX_MEM (input) - writeback control in WB stage (pipe 1)
//// wb_state_2_ID        (input) - writeback control in EX stage (pipe 2)
//// wp_state_2_ID_EX     (input) - writeback control in MEM stage (pipe 2)
//// wb_state_2_ID_EX_MEM (input) - writeback control in WB stage (pipe 2)
////
//// forward_1_A         (output) - forwarding control for rs1_data_ID (pipe 1)
//// forward_2_A         (output) - forwarding control for rs1_data_ID (pipe 2)
//// forward_1_B         (output) - forwarding control for rs2_data_ID (pipe 1)
//// forward_2_B         (output) - forwarding control for rs2_data_ID (pipe 2)
////
module riscv_Forward (
    input abi_reg_t dcd_rs1_ID_cb[1:0], dcd_rs2_ID_cb[1:0], dcd_rd_ID[1:0],
                    dcd_rd_ID_EX[1:0], dcd_rd_ID_EX_MEM[1:0],
    input wb_state_t wb_state_1_ID, wb_state_1_ID_EX, wb_state_1_ID_EX_MEM,
                     wb_state_2_ID, wb_state_2_ID_EX, wb_state_2_ID_EX_MEM,
    output fwd_src_t forward_1_A, forward_1_B,
                     forward_2_A, forward_2_B);

  // Pipe 1 destination forwarding
  always_comb begin
    forward_1_A = FWD_NONE;
    forward_1_B = FWD_NONE;

    // Operand A forwarding
    if (dcd_rs1_ID_cb[0] != REG_ZERO) begin
      // EX hazard with jump
      if (dcd_rs1_ID_cb[0] == dcd_rd_ID[0]
          && wb_state_1_ID.rd_data_src == RD_PC_PLUS_4) begin
        forward_1_A = FWD_JMP;
      end
      // EX hazard (pipe 2 precedence)
      else if (dcd_rs1_ID_cb[0] == dcd_rd_ID[1]
               && wb_state_2_ID.rd_data_src != RD_NONE) begin
        forward_1_A = FWD_2_EX;
      end else if (dcd_rs1_ID_cb[0] == dcd_rd_ID[0]
                   && wb_state_1_ID.rd_data_src != RD_NONE) begin
        forward_1_A = FWD_1_EX;
      end
      // MEM hazard (pipe 2 precedence)
      else if (dcd_rs1_ID_cb[0] == dcd_rd_ID_EX[1]
               && wb_state_2_ID_EX.rd_data_src != RD_NONE) begin
        forward_1_A = FWD_2_MEM;
      end else if (dcd_rs1_ID_cb[0] == dcd_rd_ID_EX[0]
                   && wb_state_1_ID_EX.rd_data_src != RD_NONE) begin
        forward_1_A = FWD_1_MEM;
      end
      // WB hazard (pipe 2 precedence)
      else if (dcd_rs1_ID_cb[0] == dcd_rd_ID_EX_MEM[1]
               && wb_state_2_ID_EX_MEM.rd_data_src != RD_NONE) begin
        forward_1_A = FWD_2_WB;
      end else if (dcd_rs1_ID_cb[0] == dcd_rd_ID_EX_MEM[0]
                   && wb_state_1_ID_EX_MEM.rd_data_src != RD_NONE) begin
        forward_1_A = FWD_1_WB;
      end
    end

    // Operand B forwarding
    if (dcd_rs2_ID_cb[0] != REG_ZERO) begin
      // EX hazard with jump
      if (dcd_rs2_ID_cb[0] == dcd_rd_ID[0]
          && wb_state_1_ID.rd_data_src == RD_PC_PLUS_4) begin
        forward_1_B = FWD_JMP;
      end
      // EX hazard (pipe 2 precedence)
      else if (dcd_rs2_ID_cb[0] == dcd_rd_ID[1]
          && wb_state_2_ID.rd_data_src != RD_NONE) begin
        forward_1_B = FWD_2_EX;
      end else if (dcd_rs2_ID_cb[0] == dcd_rd_ID[0]
          && wb_state_1_ID.rd_data_src != RD_NONE) begin
        forward_1_B = FWD_1_EX;
      end
      // MEM hazard (pipe 2 precedence)
      else if (dcd_rs2_ID_cb[0] == dcd_rd_ID_EX[1]
               && wb_state_2_ID_EX.rd_data_src != RD_NONE) begin
        forward_1_B = FWD_2_MEM;
      end else if (dcd_rs2_ID_cb[0] == dcd_rd_ID_EX[0]
               && wb_state_1_ID_EX.rd_data_src != RD_NONE) begin
        forward_1_B = FWD_1_MEM;
      end
      // WB hazard
      else if (dcd_rs2_ID_cb[0] == dcd_rd_ID_EX_MEM[1]
               && wb_state_2_ID_EX_MEM.rd_data_src != RD_NONE) begin
        forward_1_B = FWD_2_WB;
      end else if (dcd_rs2_ID_cb[0] == dcd_rd_ID_EX_MEM[0]
               && wb_state_1_ID_EX_MEM.rd_data_src != RD_NONE) begin
        forward_1_B = FWD_1_WB;
      end
    end
  end

  // Pipe 2 destination forwarding
  always_comb begin
    forward_2_A = FWD_NONE;
    forward_2_B = FWD_NONE;

    // Operand A forwarding
    if (dcd_rs1_ID_cb[1] != REG_ZERO) begin
      // EX hazard (pipe 2 precedence)
      if (dcd_rs1_ID_cb[1] == dcd_rd_ID[1]
          && wb_state_2_ID.rd_data_src != RD_NONE) begin
        forward_2_A = FWD_2_EX;
      end else if (dcd_rs1_ID_cb[1] == dcd_rd_ID[0]
                   && wb_state_1_ID.rd_data_src != RD_NONE) begin
        forward_2_A = FWD_1_EX;
      end
      // MEM hazard (pipe 2 precedence)
      else if (dcd_rs1_ID_cb[1] == dcd_rd_ID_EX[1]
               && wb_state_2_ID_EX.rd_data_src != RD_NONE) begin
        forward_2_A = FWD_2_MEM;
      end else if (dcd_rs1_ID_cb[1] == dcd_rd_ID_EX[0]
                   && wb_state_1_ID_EX.rd_data_src != RD_NONE) begin
        forward_2_A = FWD_1_MEM;
      end
      // WB hazard (pipe 2 precedence)
      else if (dcd_rs1_ID_cb[1] == dcd_rd_ID_EX_MEM[1]
               && wb_state_2_ID_EX_MEM.rd_data_src != RD_NONE) begin
        forward_2_A = FWD_2_WB;
      end else if (dcd_rs1_ID_cb[1] == dcd_rd_ID_EX_MEM[0]
                   && wb_state_1_ID_EX_MEM.rd_data_src != RD_NONE) begin
        forward_2_A = FWD_1_WB;
      end
    end

    // Operand B forwarding
    if (dcd_rs2_ID_cb[1] != REG_ZERO) begin
      // EX hazard (pipe 2 precedence)
      if (dcd_rs2_ID_cb[1] == dcd_rd_ID[1]
          && wb_state_2_ID.rd_data_src != RD_NONE) begin
        forward_2_B = FWD_2_EX;
      end else if (dcd_rs2_ID_cb[1] == dcd_rd_ID[0]
                   && wb_state_1_ID.rd_data_src != RD_NONE) begin
        forward_2_B = FWD_1_EX;
      end
      // MEM hazard (pipe 2 precedence)
      else if (dcd_rs2_ID_cb[1] == dcd_rd_ID_EX[1]
               && wb_state_2_ID_EX.rd_data_src != RD_NONE) begin
        forward_2_B = FWD_2_MEM;
      end else if (dcd_rs2_ID_cb[1] == dcd_rd_ID_EX[0]
                   && wb_state_1_ID_EX.rd_data_src != RD_NONE) begin
        forward_2_B = FWD_1_MEM;
      end
      // WB hazard
      else if (dcd_rs2_ID_cb[1] == dcd_rd_ID_EX_MEM[1]
               && wb_state_2_ID_EX_MEM.rd_data_src != RD_NONE) begin
        forward_2_B = FWD_2_WB;
      end else if (dcd_rs2_ID_cb[1] == dcd_rd_ID_EX_MEM[0]
                   && wb_state_1_ID_EX_MEM.rd_data_src != RD_NONE) begin
        forward_2_B = FWD_1_WB;
      end
    end
  end
endmodule : riscv_Forward

////
//// riscv_PCManager: Calculates next_pc and pipeline flush signal
////
//// pc                 (input)  - Architectural PC
//// pc_plus_4          (input)  - Architectural PC+4
//// pc_plus_8          (input)  - Architectural PC+8
//// alu_out            (input)  - alu_out from inside EX for branch condition
////                               and indirect jump target
//// pipe_transfer      (input)  - Second pipeline cannot execute instruction
//// ex_state_1_ID      (input)  - Struct containing registered pc_src from
////                               end of ID
//// btb_null_IF_ID     (input)  - Registered BTB entry is null
//// btb_hit            (input)  - BTB hit
//// btb_hit_IF_ID      (input)  - Registered BTB hit
//// btb_rd_next_pc     (input)  - BTB predicted target
//// btb_rd_state       (input)  - BTB saturation counter
//// btb_rd_state_IF_ID (input)  - Registered BTB saturation counter
//// new_btb_state      (output) - New BTB saturation counter entry
//// update_btb         (output) - BTB entry needs to be updated
//// next_pc            (output) - Calculated next_pc
//// pipe_flush_if_id   (output) - Control-flow signal to flush IF and ID
module riscv_PCManager (
    input logic [31:0] pc, pc_plus_4, pc_plus_8, pc_plus_4_1_IF_ID,
    input logic [31:0] alu_out_EX_cb[1:0],
    input logic br_taken_1_EX_cb,
    input logic pipe_transfer,
    input ex_state_t ex_state_1_ID, ex_state_2_ID,
    input logic btb_null_IF_ID, btb_hit, btb_hit_IF_ID,
    input [29:0] btb_rd_next_pc,
    input btb_state_t btb_rd_state, btb_rd_state_IF_ID,
    output btb_state_t new_btb_state,
    output logic update_btb,
    output logic [31:0] next_pc,
    output logic pipe_flush_if_id, pipe_1_kill_id);

  always_comb begin
    new_btb_state = btb_rd_state_IF_ID;
    update_btb = 1'b0;
    next_pc = 32'hxx_xx_xx_xx;
    pipe_flush_if_id = 1'b0;
    pipe_1_kill_id = 1'b0;

    unique case (ex_state_1_ID.pc_src)
      PC_SRC_IMPLICIT: begin
        next_pc = pipe_transfer ? pc_plus_4 : pc_plus_8; // Architectural PC
      end

      PC_SRC_JALR: begin
        next_pc = {alu_out_EX_cb[0][31:1], 1'b0}; // Registered PC
        pipe_flush_if_id = 1'b1; // Full IF/ID flush
      end

      PC_SRC_JAL: begin
        if (btb_hit_IF_ID) begin
          next_pc = pipe_transfer ? pc_plus_4 : pc_plus_8; // Architectural PC
          pipe_1_kill_id = 1'b1; // Partial flush of simultaneously fetched instruction
        end else begin
          if (btb_null_IF_ID) begin
            new_btb_state = ST;
            update_btb = 1'b1;
          end

          next_pc = alu_out_EX_cb[0]; // Registered PC
          pipe_flush_if_id = 1'b1; // Full IF/ID flush
        end
      end

      PC_SRC_BXX: begin
        if (btb_hit_IF_ID) begin
          unique case (btb_rd_state_IF_ID)
            SNT: begin // Strongly not taken
              if (br_taken_1_EX_cb) begin // Mispredicted
                new_btb_state = WNT;
                update_btb = 1'b1;
                next_pc = alu_out_EX_cb[0]; // Registered PC
                pipe_flush_if_id = 1'b1; // Full IF/ID flush
              end else begin // Predicted
                next_pc = pipe_transfer ? pc_plus_4 : pc_plus_8; // Arch PC
              end
            end

            WNT: begin // Weakly not taken
              update_btb = 1'b1;
              if (br_taken_1_EX_cb) begin // Mispredicted
                new_btb_state = WT;
                next_pc = alu_out_EX_cb[0]; // Registered PC
                pipe_flush_if_id = 1'b1; // Full IF/ID flush
              end else begin // Predicted
                new_btb_state = SNT;
                next_pc = pipe_transfer ? pc_plus_4 : pc_plus_8; // Arch PC
              end
            end

            WT: begin // Weakly taken
              update_btb = 1'b1;
              if (br_taken_1_EX_cb) begin // Predicted
                new_btb_state = ST;
                next_pc = pipe_transfer ? pc_plus_4 : pc_plus_8; // Arch PC
                pipe_1_kill_id = 1'b1; // Partial flush of simultaneously fetched instruction
              end else begin // Mispredicted
                new_btb_state = WNT;
                next_pc = pc_plus_4_1_IF_ID; // Registered PC
                pipe_flush_if_id = 1'b1; // Full IF/ID flush
              end
            end

            ST: begin // Strongly taken
              if (br_taken_1_EX_cb) begin // Predicted
                next_pc = pipe_transfer ? pc_plus_4 : pc_plus_8; // Arch PC
                pipe_1_kill_id = 1'b1; // Partial flush of simultaneously fetched instruction
              end else begin // Mispredicted
                new_btb_state = WT;
                update_btb = 1'b1;
                next_pc = pc_plus_4_1_IF_ID; // Registered PC
                pipe_flush_if_id = 1'b1; // Full IF/ID flush
              end
            end
          endcase
        end else begin
          if (btb_null_IF_ID) begin
            new_btb_state = br_taken_1_EX_cb ? ST : SNT;
            update_btb = 1'b1;
          end

          if (br_taken_1_EX_cb) begin // Mispredicted (SNT) on invalid BTB entry
            next_pc = alu_out_EX_cb[0]; // Registered PC
            pipe_flush_if_id = 1'b1; // Full IF/ID flush
          end else begin // Predicted (SNT) on invalid BTB entry
            next_pc = pipe_transfer ? pc_plus_4 : pc_plus_8; // Architectural PC
          end
        end
      end
    endcase

    /* Branch prediction from IF takes least precedence, and should only happen */
    /* if the pipeline is not flushing due to JAL/R or a mispredicted branch *1/ */
    if (~pipe_flush_if_id & btb_hit) begin
      unique case (btb_rd_state)
        SNT, WNT: next_pc = pipe_transfer ? pc_plus_4 : pc_plus_8;
        ST, WT: next_pc = {btb_rd_next_pc, 2'b00};
      endcase
    end
  end
endmodule : riscv_PCManager

////
//// Statistical unit to record dynamic data
////
module riscv_Stats (
  input logic clk, rst_b,
  input logic pipe_stall, pipe_flush_if_id, pipe_1_kill_id, pipe_transfer,
  input logic pipe_2_noex,
  input logic is_syscall,
  input logic is_syscall_1_ID, is_syscall_1_ID_EX, is_syscall_1_ID_EX_MEM,
  input logic btb_null_IF_ID, btb_hit_IF_ID,
  input logic [31:0] ir_IF[1:0],
  input pc_src_t pc_src_1_ID_cb,
  input ex_state_t ex_state_1_ID,
  input wb_state_t wb_state_1_ID, wb_state_2_ID);

  logic [31:0] stat_cycles;
  logic [31:0] stat_fetched, stat_executed;
  logic [31:0] stat_id_2;
  logic [31:0] stat_id_1, stat_id_1_alu, stat_id_1_not_alu, stat_id_1_valid;
  logic [31:0] stat_id_0, stat_id_0_hazard, stat_id_0_valid;
  logic [31:0] stat_cf_executed, stat_cf_miss;

  always_ff @(posedge clk, negedge rst_b) begin
    if (~rst_b) begin
      stat_cycles <= '0; // total cycles elapsed
      stat_fetched <= '0; // total instructions fetched (no nops)
      stat_executed <= '0; // total instructions executed (no nops)

      stat_id_2 <= '0; // cycles ID issued 2 instructions

      stat_id_1 <= '0; // cycles ID issued 1 instruction
      stat_id_1_alu <= '0; // cycles ID issued 1 instruction due to ALU
      stat_id_1_not_alu <= '0; // cycles ID issued 1 instruction due to non-ALU
      stat_id_1_valid <= '0; // cycles ID issued 1 instruction due to invalid

      stat_id_0 <= '0; // cycles ID issued 0 instructions
      stat_id_0_hazard <= '0; // cycles ID issued 0 instructions due to hazard
      stat_id_0_valid <= '0; // cycles ID issued 0 instructions due to invalid

      stat_cf_executed <= '0; // control-flow instructions executed
      stat_cf_miss <= '0; // control-flow instructions mispredicted
    end else begin
      stat_cycles <= stat_cycles + 1;

      if (~pipe_stall & ~pipe_flush_if_id) begin
        if (pipe_transfer) stat_fetched <= stat_fetched + 1;
        else stat_fetched <= stat_fetched + 2;
      end

      if (~wb_state_1_ID.is_nop & ~wb_state_2_ID.is_nop) begin
        stat_executed <= stat_executed + 2;
      end else if (~wb_state_1_ID.is_nop | ~wb_state_2_ID.is_nop) begin
        stat_executed <= stat_executed + 1;
      end

      // ID issued 0 instructions
      if (pipe_flush_if_id
          | pipe_stall
          | ((ir_IF[0] == `INST_NOP) & (ir_IF[1] == `INST_NOP))) begin
        stat_id_0 <= stat_id_0 + 1;
        // If both instructions are invalid
        if (ir_IF[0] == `INST_NOP
            & ir_IF[1] == `INST_NOP) stat_id_0_valid <= stat_id_0_valid + 1;
        // If there is a data hazard (stall but no syscall)
        else if (pipe_stall &
                 ~(is_syscall_1_ID_EX_MEM
                   | is_syscall_1_ID_EX
                   | is_syscall_1_ID)) stat_id_0_hazard <= stat_id_0_hazard + 1;
      end
      // ID issued 1 instruction
      else if (pipe_transfer |
               pipe_1_kill_id |
               (ir_IF[1] == `INST_NOP)) begin
        stat_id_1 <= stat_id_1 + 1;
        // If pipeline 2 instruction is invalid
        if (ir_IF[1] == `INST_NOP) stat_id_1_valid <= stat_id_1_valid + 1;
        // If pipeline 2 instruction is not alu
        else if (pipe_2_noex) stat_id_1_not_alu <= stat_id_1_not_alu + 1;
        // If pipeline 2 instruction is alu and stalls (but no syscall)
        else if (pipe_transfer
                 & (pc_src_1_ID_cb == PC_SRC_IMPLICIT)
                 & ~is_syscall) stat_id_1_alu <= stat_id_1_alu + 1;
      end
      // ID issued 2 instructions
      else begin
        stat_id_2 <= stat_id_2 + 1;
      end

      unique case (ex_state_1_ID.pc_src)
        PC_SRC_BXX: begin
          stat_cf_executed <= stat_cf_executed + 1;
          if (btb_null_IF_ID | ~btb_hit_IF_ID) stat_cf_miss <= stat_cf_miss + 1;
        end
        PC_SRC_JAL: begin
          stat_cf_executed <= stat_cf_executed + 1;
          if (btb_null_IF_ID | ~btb_hit_IF_ID) stat_cf_miss <= stat_cf_miss + 1;
        end
        PC_SRC_JALR: begin
          stat_cf_executed <= stat_cf_executed + 1;
        end
        PC_SRC_IMPLICIT: ;
      endcase
    end
  end

endmodule : riscv_Stats

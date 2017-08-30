/**
 * riscv_decode.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains the implementation of the RISC-V decoder.
 *
 * This takes in information about the current RISC-V instruction and produces
 * the appropriate contol signals to get the processor to execute the current
 * instruction.
 **/

`include "riscvPkg.pkg"

////
//// The RISC-V instruction decoder module
////
////   inst           (input)  - The undecoded 32-bit instruction
////   dcd_rs1        (output) - Regfile source 1
////   dcd_rs2        (output) - Regfile source 2
////   dcd_rd         (output) - Regfile destination
////   dcd_imm        (output) - Multiplexed immediate for ALU or PCAdder
////   alu_op         (output) - Enumerated ALU operation
////   alu_signed     (output) - Signed-ness of ALU operation
////   is_alu_rtype     (output) - ALU reg-reg operation
////   is_syscall     (output) - Syscall with forwarded REG_A0
////   except_ri      (output) - Instruction decoder exception
////   rd_data_src    (output) - Regfile writeback source
////   pc_src         (output) - Next PC source
////   mem_signed     (output) - Signed-ness of load operation
////   mem_op         (output) - Memory operation (RD, WR, SLEEP)
////   mem_mask       (output) - Memory byte mask
////
module riscv_decode_first (
    input logic [31:0] inst,
    output abi_reg_t dcd_rs1, dcd_rs2, dcd_rd,
    output logic [31:0] dcd_imm,
    output alu_op_t alu_op,
    output br_type_t br_type,
    output logic alu_signed,
    output logic is_alu_rtype, is_alu_pctype, is_syscall, except_ri,
    output rd_data_src_t rd_data_src,
    output pc_src_t pc_src,
    output mem_op_t mem_op,
    output logic mem_signed,
    output mem_mask_t mem_mask);

  // Instruction decode signals
  // rtype, itype, stype, sbtype, utype, ujtype
  opcode_t opcode;
  logic [2:0] dcd_funct3;
  logic [6:0] dcd_funct7;
  logic [11:0] dcd_funct12;

  rtype_int_funct3_t rtype_int_funct3;
  rtype_alt_int_funct3_t rtype_alt_int_funct3;
  rtype_funct7_t rtype_funct7;

  itype_int_funct3_t itype_int_funct3;
  itype_load_funct3_t itype_load_funct3;
  itype_funct7_t itype_funct7; // SRLI or SRAI
  logic [31:0] itype_imm, itype_imm_shamt;

  stype_funct3_t stype_funct3;
  logic [31:0] stype_imm;

  sbtype_funct3_t sbtype_funct3;
  logic [31:0] sbtype_imm;

  logic [31:0] utype_imm;

  logic [31:0] ujtype_imm;

  itype_funct12_t itype_funct12; // System call code

  // Decode the various function codes for the instruction
  assign opcode = opcode_t'(inst[6:0]);
  assign dcd_funct3 = inst[14:12];
  assign dcd_funct7 = inst[31:25];
  assign dcd_funct12 = inst[31:20];

  // Interpret funct fields for different instruction types
  assign rtype_int_funct3 = rtype_int_funct3_t'(dcd_funct3);
  assign rtype_alt_int_funct3 = rtype_alt_int_funct3_t'(dcd_funct3);
  assign rtype_funct7 = rtype_funct7_t'(dcd_funct7);

  assign itype_int_funct3 = itype_int_funct3_t'(dcd_funct3);
  assign itype_load_funct3 = itype_load_funct3_t'(dcd_funct3);
  assign itype_funct7 = itype_funct7_t'(dcd_funct7);
  assign itype_imm = {{20{inst[31]}}, inst[31:20]};
  assign itype_imm_shamt = {27'h0_00_00_00, inst[24:20]};

  assign stype_funct3 = stype_funct3_t'(dcd_funct3);
  assign stype_imm = {{20{inst[31]}}, inst[31:25], inst[11:7]};

  assign sbtype_funct3 = sbtype_funct3_t'(dcd_funct3);
  assign sbtype_imm = {{19{inst[31]}}, inst[31], inst[7], inst[30:25],
                       inst[11:8], 1'b0};

  assign utype_imm = {inst[31:12], 12'b00_00_00};

  assign ujtype_imm = {{11{inst[31]}}, inst[31], inst[19:12], inst[20],
                       inst[30:21], 1'b0};

  assign itype_funct12 = itype_funct12_t'(dcd_funct12);

  // RISC-V instruction decoder primary datapath
  always_comb begin
    // Decode the opcode and registers for the instruction
    // dcd_rs2 will be forciby set to REG_A0 for syscall
    dcd_rs1 = inst[19:15];
    dcd_rs2 = inst[24:20];
    dcd_rd = inst[11:7];
    dcd_imm = 32'h00_00_00_00;

    alu_op = ALU_NOP; // Inject NOPs into the ALU instead of don't care
    br_type = NOBRANCH; // Default to no branching comparison
    alu_signed = 1'b0; // Default to performing unsigned ALU operations

    pc_src = PC_SRC_IMPLICIT; // Default to incrementing PC by 4

    is_alu_rtype = 1'b0; // Default to reg-immediate ALU operation
    is_alu_pctype = 1'b0; // Default to normal ALU operation
    is_syscall = 1'b0; // Default to no syscall
    rd_data_src = RD_NONE; // Default to no writeback operation
    except_ri = 1'b0; // Reserved instruction exception

    mem_signed = 1'b0; // Default to performing unsigned memory operations
    mem_op = MEM_SLEEP;
    mem_mask = MASK_NONE;

    unique case (opcode)
      OP_OP: begin
        unique case (rtype_funct7)
          // 7-bit function code for a general R-type integer operation
          FUNCT7_INT: begin
            unique case (rtype_int_funct3)
              FUNCT3_ADD: begin
                alu_op = ALU_ADD;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              FUNCT3_SLL: begin
                alu_op = ALU_SL;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              FUNCT3_SLT: begin
                alu_op = ALU_LT;
                alu_signed = 1'b1;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              FUNCT3_SLTU: begin
                alu_op = ALU_LT;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              FUNCT3_XOR: begin
                alu_op = ALU_XOR;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              FUNCT3_SRL: begin
                alu_op = ALU_SR;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              FUNCT3_OR: begin
                alu_op = ALU_OR;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              FUNCT3_AND: begin
                alu_op = ALU_AND;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              default: begin
                except_ri = 1'b1;
              end
            endcase
          end

          // 7-bit function code for an alternate R-type integer operation
          FUNCT7_ALT_INT: begin
            unique case (rtype_alt_int_funct3)
              FUNCT3_SUB: begin
                alu_op = ALU_SUB;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              FUNCT3_SRA: begin
                alu_op = ALU_SR;
                alu_signed = 1'b1;
                is_alu_rtype = 1'b1;
                rd_data_src = RD_ALU;
              end

              default: begin
                except_ri = 1'b1;
              end
            endcase
          end

          default: begin
            except_ri = 1'b1;
          end
        endcase
      end

      // General I-type arithmetic operation
      OP_IMM: begin
        unique case (itype_int_funct3)
          FUNCT3_ADDI: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_ADD;
            rd_data_src = RD_ALU;
          end

          FUNCT3_SLTI: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_LT;
            alu_signed = 1'b1;
            rd_data_src = RD_ALU;
          end

          FUNCT3_SLTIU: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_LT;
            rd_data_src = RD_ALU;
          end

          FUNCT3_XORI: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_XOR;
            rd_data_src = RD_ALU;
          end

          FUNCT3_ORI: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_OR;
            rd_data_src = RD_ALU;
          end

          FUNCT3_ANDI: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_AND;
            rd_data_src = RD_ALU;
          end

          FUNCT3_SLLI: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm_shamt;
            alu_op = ALU_SL;
            rd_data_src = RD_ALU;
          end

          FUNCT3_SRLI_SRAI: begin
            unique case (itype_funct7)
              FUNCT7_SRLI: begin
                dcd_rs2 = REG_ZERO; // Prevent forwarding
                dcd_imm = itype_imm_shamt;
                alu_op = ALU_SR;
                rd_data_src = RD_ALU;
              end

              FUNCT7_SRAI: begin
                dcd_rs2 = REG_ZERO; // Prevent forwarding
                dcd_imm = itype_imm_shamt;
                alu_op = ALU_SR;
                alu_signed = 1'b1;
                rd_data_src = RD_ALU;
              end

              default: begin
                except_ri = 1'b1;
              end
            endcase
          end

          default: begin
            except_ri = 1'b1;
          end
        endcase
      end

      // General I-type load
      OP_LOAD: begin
        unique case (itype_load_funct3)
          FUNCT3_LB: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_ADD;
            rd_data_src = RD_MEM;
            mem_op = MEM_RD;
            mem_signed = 1'b1;
            mem_mask = MASK_BYTE;
          end

          FUNCT3_LH: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_ADD;
            rd_data_src = RD_MEM;
            mem_op = MEM_RD;
            mem_signed = 1'b1;
            mem_mask = MASK_HALF;
          end

          FUNCT3_LW: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_ADD;
            rd_data_src = RD_MEM;
            mem_op = MEM_RD;
            mem_signed = 1'b1;
            mem_mask = MASK_WORD;
          end

          FUNCT3_LBU: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_ADD;
            rd_data_src = RD_MEM;
            mem_op = MEM_RD;
            mem_mask = MASK_BYTE;
          end

          FUNCT3_LHU: begin
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_imm = itype_imm;
            alu_op = ALU_ADD;
            rd_data_src = RD_MEM;
            mem_op = MEM_RD;
            mem_mask = MASK_HALF;
          end

          default: begin
            except_ri = 1'b1;
          end
        endcase
      end

      // General S-type store
      OP_STORE: begin
        unique case (stype_funct3)
          FUNCT3_SB: begin
            dcd_rd = REG_ZERO; // Prevent forwarding
            dcd_imm = stype_imm;
            alu_op = ALU_ADD;
            mem_op = MEM_WR;
            mem_mask = MASK_BYTE;
          end

          FUNCT3_SH: begin
            dcd_rd = REG_ZERO; // Prevent forwarding
            dcd_imm = stype_imm;
            alu_op = ALU_ADD;
            mem_op = MEM_WR;
            mem_mask = MASK_HALF;
          end

          FUNCT3_SW: begin
            dcd_rd = REG_ZERO; // Prevent forwarding
            dcd_imm = stype_imm;
            alu_op = ALU_ADD;
            mem_op = MEM_WR;
            mem_mask = MASK_WORD;
          end

          default: begin
            except_ri = 1'b1;
          end
        endcase
      end

      // General U-type load upper immediate
      OP_LUI: begin
        dcd_rs1 = REG_ZERO; // Prevent forwarding
        dcd_rs2 = REG_ZERO; // Prevent forwarding
        dcd_imm = utype_imm;
        alu_op = ALU_ADD;
        rd_data_src = RD_ALU;
      end

      // General U-type add upper immediate to PC
      OP_AUIPC: begin
        dcd_rs1 = REG_ZERO; // Prevent forwarding
        dcd_rs2 = REG_ZERO; // Prevent forwarding
        dcd_imm = utype_imm;
        alu_op = ALU_ADD;
        is_alu_pctype = 1'b1;
        rd_data_src = RD_ALU;
      end

      // General UJ-type jump and link
      OP_JAL: begin
        dcd_rs1 = REG_ZERO; // Prevent forwarding
        dcd_rs2 = REG_ZERO; // Prevent forwarding
        dcd_imm = ujtype_imm;
        alu_op = ALU_ADD;
        is_alu_pctype = 1'b1;
        rd_data_src = RD_PC_PLUS_4;
        pc_src = PC_SRC_JAL;
      end

      // General I-type jump and link register
      OP_JALR: begin
        dcd_rs2 = REG_ZERO; // Prevent forwarding
        dcd_imm = itype_imm;
        alu_op = ALU_ADD;
        rd_data_src = RD_PC_PLUS_4;
        pc_src = PC_SRC_JALR;
      end

      // General SB-type conditional branch
      OP_BRANCH: begin
        unique case (sbtype_funct3)
          FUNCT3_BEQ: begin
            dcd_rd = REG_ZERO; // Prevent forwarding
            dcd_imm = sbtype_imm;
            alu_op = ALU_ADD;
            br_type = BEQ;
            is_alu_pctype = 1'b1;
            pc_src = PC_SRC_BXX;
          end

          FUNCT3_BNE: begin
            dcd_rd = REG_ZERO; // Prevent forwarding
            dcd_imm = sbtype_imm;
            alu_op = ALU_ADD;
            br_type = BNE;
            is_alu_pctype = 1'b1;
            pc_src = PC_SRC_BXX;
          end

          FUNCT3_BLT: begin
            dcd_rd = REG_ZERO; // Prevent forwarding
            dcd_imm = sbtype_imm;
            alu_op = ALU_ADD;
            br_type = BLT;
            is_alu_pctype = 1'b1;
            pc_src = PC_SRC_BXX;
          end

          FUNCT3_BGE: begin
            dcd_rd = REG_ZERO; // Prevent forwarding
            dcd_imm = sbtype_imm;
            alu_op = ALU_ADD;
            br_type = BGE;
            is_alu_pctype = 1'b1;
            pc_src = PC_SRC_BXX;
          end

          FUNCT3_BLTU: begin
            dcd_rd = REG_ZERO; // Prevent forwarding
            dcd_imm = sbtype_imm;
            alu_op = ALU_ADD;
            br_type = BLTU;
            is_alu_pctype = 1'b1;
            pc_src = PC_SRC_BXX;
          end

          FUNCT3_BGEU: begin
            dcd_rd = REG_ZERO; // Prevent forwarding
            dcd_imm = sbtype_imm;
            alu_op = ALU_ADD;
            br_type = BGEU;
            is_alu_pctype = 1'b1;
            pc_src = PC_SRC_BXX;
          end

          default: begin
            except_ri = 1'b1;
          end
        endcase
      end

      // General system operation
      OP_SYSTEM: begin
        unique case (itype_funct12)
          FUNCT12_ECALL: begin
            dcd_rs1 = REG_A0;
            dcd_rs2 = REG_ZERO; // Prevent forwarding
            dcd_rd = REG_ZERO; // Prevent forwarding
            alu_op = ALU_ADD;
            is_syscall = 1'b1;
            rd_data_src = RD_SYS;
          end

          default: begin
            except_ri = 1'b1;
          end
        endcase
      end

      default: begin
        except_ri = 1'b1;
      end
    endcase
  end
endmodule : riscv_decode_first

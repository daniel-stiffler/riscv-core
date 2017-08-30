`ifndef INTERNAL_DEFINES_H
  `define INTERNAL_DEFINES_H

  // The sizes of opcodes and function codes in bits
  `define OPCODE_SIZE 7
  `define FUNCT3_SIZE 3
  `define FUNCT7_SIZE 7
  `define FUNCT12_SIZE 12

  // The number of registers in the register file, and the number of bits needed
  `define RISCV_NUM_REGS 32
  `define REG_BITS 5

  // Default entry in the BTB
  `define BTB_NULL 62'h00_00_00_00_00_00_00_00

  // The starting addresses of the user's data and text segments
  `define USER_TEXT_START 32'h00_40_00_00
  `define USER_DATA_START 32'h01_00_00_00

  // The instruction representing a NOP
  `define INST_NOP {12'h0_00, REG_ZERO, FUNCT3_ADDI, REG_ZERO, OP_IMM}
  `define RAS_JAL {REG_RA, OP_JAL}
  `define RAS_JALR_POP {REG_RA, 3'b000, REG_ZERO, OP_JALR}
  // JALR push conditon allows rs1 to be any register
  `define RAS_JALR_PUSH {3'b000, REG_RA, OP_JALR}
`endif

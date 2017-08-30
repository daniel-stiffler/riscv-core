/**
 *
 * Revision History (S17):
 * 
 * 1/27/17: corrected typo in rtype_alt_int_funct3_t typedef   
 * 
 * riscv_isa.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This contains the definitions for the RISC-V opcodes and function codes for
 * that must be implemented by the processor.
 *
 * Note that the names of the enumerations are based on the names given in the
 * RISC-V 2.1 ISA manual, sections 2 and 5.
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

`ifndef RISCV_ISA_VH_
`define RISCV_ISA_VH_

/*----------------------------------------------------------------------------
 * Definitions
 *----------------------------------------------------------------------------*/

// The sizes of opcodes and function codes in bits
`define OPCODE_SIZE         7
`define FUNCT3_SIZE         3
`define FUNCT7_SIZE         7
`define FUNCT12_SIZE        12

/*----------------------------------------------------------------------------
 * Opcodes (All Instruction Types)
 *----------------------------------------------------------------------------*/

// Opcodes for the RISC-V ISA, in the lowest 7 bits of the instruction
typedef enum logic [`OPCODE_SIZE-1:0] {
    // Opcode that indicates an integer R-type instruction (register)
    OP_OP                   = 'h33,

    // Opcode that indicates an integer I-type instruction (immediate)
    OP_IMM                  = 'h13,

    // Opcodes for load and store instructions (I-type and S-type)
    OP_LOAD                 = 'h03,
    OP_STORE                = 'h23,

    // Opcodes for U-type instructions (unsigned immediate)
    OP_LUI                  = 'h37,
    OP_AUIPC                = 'h17,

    // Opcodes for jump instructions (UJ-type and I-type)
    OP_JAL                  = 'h6F,
    OP_JALR                 = 'h67,

    // Opcode that indicates a general SB-type instruction (branch)
    OP_BRANCH               = 'h63,

    // Opcode that indicates a special system instruction (I-type)
    OP_SYSTEM               = 'h73
} opcode_t;

/*----------------------------------------------------------------------------
 * R-type Function Codes
 *----------------------------------------------------------------------------*/

// 7-bit function codes for different classes of R-type instructions
typedef enum logic [`FUNCT7_SIZE-1:0] {
    FUNCT7_INT              = 'h00,
    FUNCT7_ALT_INT          = 'h20,
    FUNCT7_MUL              = 'h01
} rtype_funct7_t;

// 3-bit function codes for integer R-type instructions
typedef enum logic [`FUNCT3_SIZE-1:0] {
    FUNCT3_ADD              = 'h0,
    FUNCT3_SLL              = 'h1,
    FUNCT3_SLT              = 'h2,
    FUNCT3_SLTU             = 'h3,
    FUNCT3_XOR              = 'h4,
    FUNCT3_SRL              = 'h5,
    FUNCT3_OR               = 'h6,
    FUNCT3_AND              = 'h7
} rtype_int_funct3_t;

// 3-bit function codes for alternate integer R-type instructions
typedef enum logic [`FUNCT3_SIZE-1:0] {
    FUNCT3_SUB              = 'h0,
    FUNCT3_SRA              = 'h5
} rtype_alt_int_funct3_t;

// 3-bit function codes for multiply R-type instructions
typedef enum logic [`FUNCT3_SIZE-1:0] {
    FUNCT3_MUL              = 'h0,
    FUNCT3_MULH             = 'h1,
    FUNCT3_MULHSU           = 'h2,
    FUNCT3_MULHU            = 'h3,
    FUNCT3_DIV              = 'h4,
    FUNCT3_DIVU             = 'h5,
    FUNCT3_REM              = 'h6,
    FUNCT3_REMU             = 'h7
} rtype_mul_funct3_t;

/*----------------------------------------------------------------------------
 * I-type Function Codes
 *----------------------------------------------------------------------------*/

// 3-bit function codes for integer I-type instructions
typedef enum logic [`FUNCT3_SIZE-1:0] {
    FUNCT3_ADDI             = 'h0,
    FUNCT3_SLTI             = 'h2,
    FUNCT3_SLTIU            = 'h3,
    FUNCT3_XORI             = 'h4,
    FUNCT3_ORI              = 'h6,
    FUNCT3_ANDI             = 'h7,
    FUNCT3_SLLI             = 'h1,
    FUNCT3_SRLI_SRAI        = 'h5
} itype_int_funct3_t;

// 3-bit function codes for load instructions (I-type)
typedef enum logic [`FUNCT3_SIZE-1:0] {
    FUNCT3_LB               = 'h0,
    FUNCT3_LH               = 'h1,
    FUNCT3_LW               = 'h2,
    FUNCT3_LBU              = 'h4,
    FUNCT3_LHU              = 'h5
} itype_load_funct3_t;

// 7-bit function codes for integer I-type instructions
typedef enum logic [`FUNCT7_SIZE-1:0] {
    FUNCT7_SRLI             = 'h00,
    FUNCT7_SRAI             = 'h20
} itype_funct7_t;

// 12-bit function codes for special system instructions (I-type)
typedef enum logic [`FUNCT12_SIZE-1:0] {
    FUNCT12_ECALL           = 'h000
} itype_funct12_t;

/*----------------------------------------------------------------------------
 * S-type Function Codes
 *----------------------------------------------------------------------------*/

// 3-bit function codes for S-type instructions (store)
typedef enum logic [`FUNCT3_SIZE-1:0] {
    FUNCT3_SB               = 'h0,
    FUNCT3_SH               = 'h1,
    FUNCT3_SW               = 'h2
} stype_funct3_t;

/*----------------------------------------------------------------------------
 * SB-type Function Codes
 *----------------------------------------------------------------------------*/

// 3-bit function codes for SB-type instructions (branch)
typedef enum logic [`FUNCT3_SIZE-1:0] {
    FUNCT3_BEQ              = 'h0,
    FUNCT3_BNE              = 'h1,
    FUNCT3_BLT              = 'h4,
    FUNCT3_BGE              = 'h5,
    FUNCT3_BLTU             = 'h6,
    FUNCT3_BGEU             = 'h7
} sbtype_funct3_t;

`endif /* RISCV_ISA_VH_ */

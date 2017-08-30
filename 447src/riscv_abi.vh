/**
 * riscv_abi.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains the definitions for the RISC-V application binary
 * interface (ABI), which are namely the register aliases for application
 * registers, such as temporaries, the stack pointer, etc.
 *
 * Note that the names of the enumerations are based on the names and
 * assignments in the RISC-V 2.1 ISA manual, section 20.2.
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

`ifndef RISCV_ABI_VH_
`define RISCV_ABI_VH_

/*----------------------------------------------------------------------------
 * Definitions
 *----------------------------------------------------------------------------*/

// The number of registers in the register file, and the number of bits needed
`define RISCV_NUM_REGS      32
`define REG_BITS            5

// The starting addresses of the user's data and text segments
`define USER_TEXT_START     'h00400000
`define USER_DATA_START     'h10000000

// The starting and ending addresses of the stack segment, and its size
`define STACK_END           'h7ff00000
`define STACK_SIZE          (1 * 1024 * 1024)
`define STACK_START         (`STACK_END - `STACK_SIZE)

// The starting addresses and sizes of the kernel's data, and text segments
`define KERNEL_TEXT_START   'h80000000
`define KERNEL_DATA_START   'h90000000

/* The value that must be passed in register a0 (x10) to the ECALL instruction
 * to halt the processor. */
`define ECALL_ARG_HALT      'ha

// Aliases for the register in the application binary interface (ABI)
typedef enum logic [`REG_BITS-1:0] {
    REG_ZERO    = 'd0,      // Zero register, hardwired to 0
    REG_RA      = 'd1,      // Return address register (caller-saved)
    REG_SP      = 'd2,      // Stack pointer register (callee-saved)
    REG_GP      = 'd3,      // Global pointer register (points to data section)
    REG_TP      = 'd4,      // Thread pointer (points to thread-local data)
    REG_T0      = 'd5,      // Temporary register 0 (caller-saved)
    REG_T1      = 'd6,      // Temporary register 0 (caller-saved)
    REG_T2      = 'd7,      // Temporary register 0 (caller-saved)
    REG_S0_FP   = 'd8,      // Saved 0/stack frame pointer (callee-saved)
    REG_S1      = 'd9,      // Saved register 1 (callee-saved)
    REG_A0      = 'd10,     // Function argument/return value 0 (caller-saved)
    REG_A1      = 'd11,     // Function argument/return value 1 (caller-saved)
    REG_A2      = 'd12,     // Function argument 2 (caller-saved)
    REG_A3      = 'd13,     // Function argument 3 (caller-saved)
    REG_A4      = 'd14,     // Function argument 4 (caller-saved)
    REG_A5      = 'd15,     // Function argument 5 (caller-saved)
    REG_A6      = 'd16,     // Function argument 6 (caller-saved)
    REG_A7      = 'd17,     // Function argument 7 (caller-saved)
    REG_S2      = 'd18,     // Saved register 2 (callee-saved)
    REG_S3      = 'd19,     // Saved register 3 (callee-saved)
    REG_S4      = 'd20,     // Saved register 4 (callee-saved)
    REG_S5      = 'd21,     // Saved register 5 (callee-saved)
    REG_S6      = 'd22,     // Saved register 6 (callee-saved)
    REG_S7      = 'd23,     // Saved register 7 (callee-saved)
    REG_S8      = 'd24,     // Saved register 8 (callee-saved)
    REG_S9      = 'd25,     // Saved register 9 (callee-saved)
    REG_S10     = 'd26,     // Saved register 10 (callee-saved)
    REG_S11     = 'd27,     // Saved register 11 (callee-saved)
    REG_T3      = 'd28,     // Temporary register 3 (caller-saved)
    REG_T4      = 'd29,     // Temporary register 4 (caller-saved)
    REG_T5      = 'd30,     // Temporary register 5 (caller-saved)
    REG_T6      = 'd31      // Temporary register 6 (caller-saved)
} abi_reg_t;

`endif /* RISCV_ABI_VH_ */

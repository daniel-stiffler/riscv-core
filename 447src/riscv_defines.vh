/*
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University.
 *
 * Copyright (c) 2004 by Babak Falsafi and James Hoe,
 * Computer Architecture Lab at Carnegie Mellon (CALCM),
 * Carnegie Mellon University.
 *
 * This source file was written and maintained by Jared Smolens
 * as part of the Two-Way In-Order Superscalar project for Carnegie Mellon's
 * Introduction to Computer Architecture course, 18-447. The source file
 * is in part derived from code originally written by Herman Schmit and
 * Diana Marculescu.
 *
 * You may not use the name "Carnegie Mellon University" or derivations
 * thereof to endorse or promote products derived from this software.
 *
 * If you modify the software you must place a notice on or within any
 * modified version provided or made available to any third party stating
 * that you have modified the software.  The notice shall include at least
 * your name, address, phone number, email address and the date and purpose
 * of the modification.
 *
 * THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO ANYWARRANTY
 * THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS OR BE ERROR-FREE AND ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
 * TITLE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY
 * BE LIABLE FOR ANY DAMAGES, INCLUDING BUT NOT LIMITED TO DIRECT, INDIRECT,
 * SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN
 * ANY WAY CONNECTED WITH THIS SOFTWARE (WHETHER OR NOT BASED UPON WARRANTY,
 * CONTRACT, TORT OR OTHERWISE).
 *
 */

//////
////// RISC-V Defines: Numerical parameters of the RISC-V processor
//////

////
//// USEFUL CONSTANTS
////

// Mask for the coprocessor number from the opcode
`define OP__ZMASK       6'h03

// Mask to force word-alignment of addresses
`define ADDR_ALIGN_MASK 32'hfffffffc

// Mask particular bytes
`define BYTE_0_MASK     32'h000000ff
`define BYTE_1_MASK     32'h0000ff00
`define BYTE_2_MASK     32'h00ff0000
`define BYTE_3_MASK     32'hff000000
`define BYTE_0_1_MASK   32'h0000ffff
`define BYTE_0_2_MASK   32'h00ff00ff
`define BYTE_0_3_MASK   32'hff0000ff
`define BYTE_1_2_MASK   32'h00ffff00
`define BYTE_1_3_MASK   32'hff00ff00
`define BYTE_2_3_MASK   32'hffff0000
`define BYTE_0_1_2_MASK 32'h00ffffff
`define BYTE_0_1_3_MASK 32'hff00ffff
`define BYTE_0_2_3_MASK 32'hffff00ff
`define BYTE_1_2_3_MASK 32'hffffff00

////
//// Exception Codes
////

// Left shift amount to align exception code from cause register
`define EX__SHIFT 'd2
// Mask to extract exception code from CAUSE register
`define EX__MASK  32'h0000007c

// Exception codes
`define EX_INT    5'd0
`define EX_MOD    5'd1
`define EX_TLBL   5'd2
`define EX_TLBS   5'd3
`define EX_ADEL   5'd4
`define EX_ADES   5'd5
`define EX_IBE    5'd6
`define EX_DBE    5'd7
`define EX_SYS    5'd8
`define EX_BP     5'd9
`define EX_RI     5'd10
`define EX_CPU    5'd11
`define EX_OV     5'd12
`define EX_TR     5'd13
`define EX_VCEI   5'd14
`define EX_FPE    5'd15
`define EX_WATCH  5'd23
`define EX_VCED   5'd31


////
//// RISC-V 447 constants
////

// System calls
`define SYS_EXIT         32'h0a

// Messages
`define MSG_UNCLASS     'h0
`define MSG_UNCLASS_S   "[0x%h]"
`define MSG_EOP         'h1
`define MSG_EOP_S       "[End of program at 0x%h]"
`define MSG_SWDUMP      'h3
`define MSG_SWDUMP_S    "[Program register dump at 0x%h]"

`define MSG_EXCPT       'h10
`define MSG_EXCPT_S     "[Exception at %h]"
`define MSG_ADEL        'h11
`define MSG_ADEL_S      "[Address error exception on load at 0x%h]"
`define MSG_ADES        'h12
`define MSG_ADES_S      "[Address error exception on store at 0x%h]"
`define MSG_IBE         'h13
`define MSG_IBE_S       "[Instruction bus error exception at 0x%h]"
`define MSG_DBE         'h14
`define MSG_DBE_S       "[Data bus error exception at 0x%h]"
`define MSG_BP          'h15
`define MSG_BP_S        "[Breakpoint exception at 0x%h]"
`define MSG_RI          'h16
`define MSG_RI_S        "[Reserved instruction exception at 0x%h]"
`define MSG_CPU         'h17
`define MSG_CPU_S       "[Coprocessor unusable exception at 0x%h]"
`define MSG_OV          'h18
`define MSG_OV_S        "[Arithmetic overflow exception at 0x%h]"

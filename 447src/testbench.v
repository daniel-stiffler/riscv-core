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


// Lab 4 version with 16-byte instruciton fetch

// Top module for the RISC-V processor core
// NOT synthesizable Verilog!
module testbench;

   reg [31:0] i;
   reg [29:0] addr;

   wire        clk, inst_excpt, mem_excpt, halted;
   wire [29:0] pc, mem_addr;
   wire [31:0] inst[3:0], mem_data_store, mem_data_load;
   wire [3:0]  mem_write_en;
   reg         rst_b;
   reg  [29:0] last_pc;
   int         instr_count;

   // The clock
   clock CLK(clk);

   // The RISC-V core
   riscv_core core(.clk(clk), .inst_addr(pc), .inst(inst),
                  .inst_excpt(inst_excpt), .mem_addr(mem_addr),
                  .mem_data_store(mem_data_store), .mem_data_load(mem_data_load),
                  .mem_write_en(mem_write_en), .mem_excpt(mem_excpt),
                  .halted(halted), .rst_b(rst_b));

   // Memory
   riscv_mem Memory(// Port 1 (instructions)
                   .addr1(pc), .data_in1(), .data_out1(inst), .we1(4'b0),
                   .excpt1(inst_excpt), .allow_kernel1(1'b1), .kernel1(),

                   // Port 2 (data)
                   .addr2(mem_addr), .data_in2(mem_data_store),
                   .data_out2(mem_data_load), .we2(mem_write_en),
                   .excpt2(mem_excpt), .allow_kernel2(1'b1), .kernel2(),
                   .rst_b(rst_b), .clk(clk));


   initial
     begin
        rst_b = 0;
        #75;
        rst_b <= 1;
     end

   always @(posedge clk) begin
        #0;     // Wait for all other tasks to finish
        if(halted)
          $finish;
   end

    // Count the number of instructions fetched from memory
    always @(posedge clk, negedge rst_b) begin
        if (!rst_b) begin
            instr_count = 1;
        end else if (pc != last_pc && !halted) begin
            instr_count += 1;
        end

        last_pc <= pc;
    end

endmodule

// Clock module for the RISC-V core
module clock(clockSignal);
   parameter start = 1, halfPeriod = 50;
   output    clockSignal;
   reg       clockSignal;

   initial
     clockSignal = start;

   always
     #halfPeriod clockSignal = ~clockSignal;

endmodule

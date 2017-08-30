/**
 * 
 * Revision History (S17)
 * 
 * 2/8/17: made memory ports externally visible to anchor retiming
 *  
 * riscv_core_timing.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is a synthesis wrapper to account for the combinational memory read delay
 **/

module fake_memory_delay(clk, addr, data);
   input clk;
   input [29:0] addr;
   output [31:0] data;

   parameter DELAYWIDTH = 7;

   reg [(DELAYWIDTH-1):0] dmem;
   reg [(DELAYWIDTH-1):0] dmem_next;
   reg            carry;
   reg            carry0;
   reg            carry1;
   reg            carry2;

   integer        i;

   always@(*) begin
      carry=(^addr);
      carry0=0;
      carry1=0;
      carry2=0;

      for(i=0;i<DELAYWIDTH;i++) begin
     dmem_next[i]=(dmem[i]^carry);
     carry2=carry1;
     carry1=carry0;
     carry0=carry;
     carry=(((dmem[i]^carry)^carry0)^carry1^carry2);
      end
   end

   assign data = {8{carry2, carry1, carry0, carry}} | addr;

   always @(posedge clk) begin
      dmem <= (dmem<<1) | addr;
   end

endmodule

module riscv_core_timing(
             // Outputs
	     inst_addr, mem_addr,
             inst, mem_data_load,  		 
             mem_data_store, mem_write_en,
             halted,
             // Inputs
             clk, inst_excpt, mem_excpt, rst_b
             );

   // Core Interface
   input         clk, inst_excpt, mem_excpt;
   output [29:0] inst_addr;
   output [29:0]   mem_addr;
   output [31:0]   inst[3:0], mem_data_load;
   output [31:0] mem_data_store;
   output [3:0]  mem_write_en;
   output        halted;
   input         rst_b;

   // The RISC-V core
   riscv_core core(.clk(clk), .inst_addr(inst_addr), .inst(inst),
                   .inst_excpt(inst_excpt), .mem_addr(mem_addr),
                   .mem_data_store(mem_data_store), .mem_data_load(mem_data_load),
                   .mem_write_en(mem_write_en), .mem_excpt(mem_excpt),
                   .halted(halted), .rst_b(rst_b));

   fake_memory_delay fake_imem_delay0(.clk(clk), .addr(inst_addr), .data(inst[0]));
   fake_memory_delay fake_imem_delay1(.clk(clk), .addr(inst_addr), .data(inst[1]));
   fake_memory_delay fake_imem_delay2(.clk(clk), .addr(inst_addr), .data(inst[2]));
   fake_memory_delay fake_imem_delay3(.clk(clk), .addr(inst_addr), .data(inst[3]));
   fake_memory_delay fake_dmem_delay(.clk(clk), .addr(mem_addr), .data(mem_data_load));

endmodule


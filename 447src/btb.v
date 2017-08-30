/*
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University. 
 *
 * Copyright (c) 2016 James C. Hoe,
 * Computer Architecture Lab at Carnegie Mellon (CALCM), 
 * Carnegie Mellon University.
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
 */

//
// SRAM of size WIDTHx(2^SIZE_LG2)
//
// 1 combinational read port, 1 synchronous write port
//
// (Yes, it really is just a synchronous SRAM with
//  interpretation applied to what is read and written.)
//

module btbsram #(parameter WIDTH=62, SIZE_LG2=7) 
   (
    // Outputs
    rd_data,
    // Inputs
    rd_idx, wr_idx, wr_data, wr_we, clk, rst_b
    );
   input [SIZE_LG2-1:0]  rd_idx, wr_idx; 
   input [WIDTH-1:0] 	 wr_data;
   input 		 wr_we, clk, rst_b ;
   
   output wire [WIDTH-1:0] rd_data ;
   
   reg [WIDTH-1:0] 	   mem[0:(2**SIZE_LG2)-1];
   integer 		   i;
   
   always @(posedge clk or negedge rst_b) begin 
      if (!rst_b) begin
	 for (i = 0; i < (2**SIZE_LG2); i = i+1) begin
	    mem[i] <= 0;
	 end
      end else if (wr_we) begin 
	 mem[wr_idx] <= wr_data; 
      end 
   end 
   
   assign rd_data = (!rst_b)? 0 : mem[rd_idx];
   
endmodule



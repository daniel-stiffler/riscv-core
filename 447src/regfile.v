/*
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University.
 *
 * Copyright (c) 2008 James C. Hoe,
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

`include "riscv_abi.vh"

/*----------------------------------------------------------------------------------------------------------------------
 * Internal Definitions
 *---------------------------------------------------------------------------------------------------------------------*/

// The file handle number for stdout
parameter STDOUT = 32'h8000_0002;

`ifdef SIMULATION_18447
// Structure representing the naming information about a register
typedef struct {
    string isa_name;        // The ISA name for a register (x0..x31)
    string abi_name;        // The ABI name for a register (sp, t0, etc.)
} register_name_t;

// An array of all the naming information for a register, and its ABI aliases
register_name_t RISCV_REGISTER_NAMES[0:`RISCV_NUM_REGS-1] = '{
    '{"x0", "zero"},
    '{"x1", "ra"},
    '{"x2", "sp"},
    '{"x3", "gp"},
    '{"x4", "tp"},
    '{"x5", "t0"},
    '{"x6", "t1"},
    '{"x7", "t2"},
    '{"x8", "s0/fp"},
    '{"x9", "s1"},
    '{"x10", "a0"},
    '{"x11", "a1"},
    '{"x12", "a2"},
    '{"x13", "a3"},
    '{"x14", "a4"},
    '{"x15", "a5"},
    '{"x16", "a6"},
    '{"x17", "a7"},
    '{"x18", "s2"},
    '{"x19", "s3"},
    '{"x20", "s4"},
    '{"x21", "s5"},
    '{"x22", "s6"},
    '{"x23", "s7"},
    '{"x24", "s8"},
    '{"x25", "s9"},
    '{"x26", "s10"},
    '{"x27", "s11"},
    '{"x28", "t3"},
    '{"x29", "t4"},
    '{"x30", "t5"},
    '{"x31", "t6"}
};
`endif

/*--------------------------------------------------------------------------------------------------------------------
 * Register Files
 *--------------------------------------------------------------------------------------------------------------------*/

`ifdef SIMULATION_18447
function void print_register(int fd, register_name_t reg_name, int i, reg [31:0] regs[0:31]);
    // Format the ABI alias name for the register
    string abi_name, reg_uint_value, reg_int_value;
    abi_name = {"(", reg_name.abi_name, ")"};

    // Format the signed and unsigned views of the register
    $sformat(reg_uint_value, "(%0d)", regs[i]);
    $sformat(reg_int_value, "(%0d)", signed'(regs[i]));

    // Print out the register's names and avlues
    $fdisplay(fd, "%-8s %-8s = 0x%08x %-12s %-13s", reg_name.isa_name, abi_name, regs[i], reg_uint_value,
            reg_int_value);
endfunction

function void print_cpu_state(int fd, reg [31:0] regs[0:31]);
    // Print out the instructions fetched and the current pc value
    $fdisplay(fd, "Current CPU State and Register Values:");
    $fdisplay(fd, "--------------------------------------");
    $fdisplay(fd, "%-20s = %0d", "Instruction Count", $root.testbench.instr_count);
    $fdisplay(fd, "%-20s = 0x%08x\n", "Program Counter (PC)", $root.testbench.pc << 2);

    // Display the header for the table of register values
    $fdisplay(fd, "%-8s %-8s   %-10s %-12s %-13s", "ISA Name", "ABI Name", "Hex Value", "Uint Value", "Int Value");
    $fdisplay(fd, {(8+1+8+3+10+1+12+1+13){"-"}});

    // Display the register and its values for each register
    for (int i = 0; i < `RISCV_NUM_REGS; i++) begin
        print_register(fd, RISCV_REGISTER_NAMES[i], i, regs);
    end
endfunction
`endif

module regfile (/*AUTOARG*/
   // Outputs
   rs1_data, rs2_data,
   // Inputs
   rs1_num, rs2_num, rd_num, rd_data, rd_we, clk, rst_b, halted
   );
    input       [4:0]  rs1_num, rs2_num, rd_num;
    input       [31:0] rd_data;
    input              rd_we, clk, rst_b, halted;
    output wire [31:0] rs1_data, rs2_data;

    reg         [31:0] mem[0:31];

    always @(posedge clk or negedge rst_b) begin
        if (!rst_b) begin
            for (int i = 0; i < 32; i++) begin
                mem[i] <= 32'b0;
            end

            // Set SP to the top of the stack, GP to the data section
            mem[REG_SP] <= `STACK_END;
            mem[REG_GP] <= `USER_DATA_START;
        end else if (rd_we && (rd_num != 0)) begin
            mem[rd_num] <= rd_data;
        end
    end

    assign rs1_data = (rs1_num == 0) ? 32'h0 : mem[rs1_num];
    assign rs2_data = (rs2_num == 0) ? 32'h0 : mem[rs2_num];

    // When simulation finishes, dump the register state to stdout and file
`ifdef SIMULATION_18447
    int fd;
    always @(posedge clk) begin
        if (halted) begin
            $display("\n18-447 Register File Dump at Cycle %0d", $time);
            $display("---------------------------------------------\n");
            print_cpu_state(STDOUT, mem);

            fd = $fopen("simulation.reg");
            print_cpu_state(fd, mem);
            $display();
            $fclose(fd);
        end
    end
`endif

endmodule


module regfile2 (/*AUTOARG*/
   // Outputs
   rs1_data, rs2_data,
   // Inputs
   rs1_num, rs2_num, rd_num, rd_data, rd_we, clk, rst_b, halted
   );
    input       [4:0]  rs1_num[1:0], rs2_num[1:0], rd_num[1:0];
    input       [31:0] rd_data[1:0];
    input              rd_we[1:0], clk, rst_b, halted;
    output wire [31:0] rs1_data[1:0], rs2_data[1:0];

    reg         [31:0] mem[0:31];

    always @(posedge clk or negedge rst_b) begin
        if (!rst_b) begin
            for (int i = 0; i < 32; i++) begin
                mem[i] <= 32'b0;
            end

            // Set SP to the top of the stack, GP to the data section
            mem[REG_SP] <= `STACK_END;
            mem[REG_GP] <= `USER_DATA_START;
        end else begin
	   if (rd_we[0] && (rd_num[0] != 0)) begin
              mem[rd_num[0]] <= rd_data[0];
	   end
	   if (rd_we[1] && (rd_num[1] != 0)) begin
              mem[rd_num[1]] <= rd_data[1];
	   end
        end
    end

    assign rs1_data[0] = (rs1_num[0] == 0) ? 32'h0 : mem[rs1_num[0]];
    assign rs1_data[1] = (rs1_num[1] == 0) ? 32'h0 : mem[rs1_num[1]];
    assign rs2_data[0] = (rs2_num[0] == 0) ? 32'h0 : mem[rs2_num[0]];
    assign rs2_data[1] = (rs2_num[1] == 0) ? 32'h0 : mem[rs2_num[1]];

    // When simulation finishes, dump the register state to stdout and file
`ifdef SIMULATION_18447
    int fd;
    always @(posedge clk) begin
        if (halted) begin
            $display("\n18-447 Register File Dump at Cycle %0d", $time);
            $display("---------------------------------------------\n");
            print_cpu_state(STDOUT, mem);

            fd = $fopen("simulation.reg");
            print_cpu_state(fd, mem);
            $display();
            $fclose(fd);
        end
    end
`endif

endmodule

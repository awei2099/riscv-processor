/*
  Group Members:
  Brandon O'Neill (71367607)
  Morgan Wang (85852961)
*/

`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`define ADDR_WIDTH 32
`define DATA_WIDTH 32

`ifndef DIVIDER_STAGES
`define DIVIDER_STAGES 8
`endif

`ifndef SYNTHESIS
  `include "../hw3-singlecycle/RvDisassembler.sv"
`endif
`include "../hw2b-cla/cla.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "../hw5-pipelined/cycle_status.sv"
`include "AxilCache_ref.sv"

module Disasm #(
    PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
`ifndef RISCV_FORMAL
`ifndef SYNTHESIS
  // this code is only for simulation, not synthesis
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic. Also,
  // string needs to be reversed to render correctly.
  genvar i;
  for (i = 3; i < 32; i = i + 1) begin : gen_disasm
    assign disasm[((i+1-3)*8)-1-:8] = disasm_string[31-i];
  end
  assign disasm[255-:8] = PREFIX;
  assign disasm[247-:8] = ":";
  assign disasm[239-:8] = " ";
`endif
`endif
endmodule

// TODO: copy over your RegFile and pipeline structs from HW5
module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,
    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;
  genvar i;
  logic [`REG_SIZE] regs[NumRegs];
  
  // The register file should handle giving the values of rs1 and rs2 as well as storing rd.

  // reg x0: should not be written to and is always 0.
  assign regs[0] = 32'b0;

  // Writing to registers:

  // Note we skip regs[0] because that should always be 0, and can't be written to.
  for (i = 1; i < NumRegs; i = i + 1) begin 
    always_ff @(posedge clk) begin
      if (rst) begin
        // If a reset signal is sent, we should clear each register and set to 0.
        regs[i] <= 32'b0;
      end else begin

        // Now we should, in each clk cycle, update the registers with its old value,
        // with the exception of the one currently written to. That should be an updated value.
        if (rd == i && we) begin 
          regs[i] <= rd_data;
        end else begin
          regs[i] <= regs[i];
        end
      end
    end
  end

  // Return the value of stored in the register file with the index specified.
  assign rs1_data = regs[rs1];
  assign rs2_data = regs[rs2];
endmodule

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
} stage_decode_t;

typedef struct packed {
  // RISC-V insn definitions
  logic [`INSN_SIZE] insn;
  logic [6:0] insn_funct7;
  logic [4:0] insn_rs2;
  logic [4:0] insn_rs1;
  logic [2:0] insn_funct3;
  logic [4:0] insn_rd;
  logic [`OPCODE_SIZE] insn_opcode;
  logic [`REG_SIZE] insn_rs1_data;
  logic [`REG_SIZE] insn_rs2_data;
  // Various formats of immediates
  logic [19:0] insn_imm_u;
  logic [11:0] insn_imm_i;
  logic [ 4:0] insn_imm_shamt;
  logic [11:0] insn_imm_s;
  logic [12:0] insn_imm_b;
  logic [20:0] insn_imm_j; 
  logic [`REG_SIZE] insn_imm_i_sext;
  logic [`REG_SIZE] insn_imm_s_sext;
  logic [`REG_SIZE] insn_imm_b_sext;
  logic [`REG_SIZE] insn_imm_j_sext;
  // Other stuff pipelining
  logic [`REG_SIZE] pc;
  cycle_status_e cycle_status;
} stage_execute_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  logic [`OPCODE_SIZE] insn_opcode;
  cycle_status_e cycle_status;
  logic we;
  logic [4:0] insn_rs1;
  logic [4:0] insn_rs2;
  logic [4:0] insn_rd;
  logic [`REG_SIZE] insn_rs1_data;
  logic [`REG_SIZE] insn_rs2_data;
  logic [`REG_SIZE] insn_rd_data;
  // Memory Stuff
  logic [`REG_SIZE] dmem_addr; // Address that the load or store insn is targetting in memory
  logic [`REG_SIZE] dmem_data; // data to store to memory (store only)
  logic [3:0] dmem_we;
  logic halt;
} stage_memory_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  logic [`OPCODE_SIZE] insn_opcode;
  cycle_status_e cycle_status;
  logic we;
  logic [`REG_SIZE] dmem_addr;
  logic [4:0] insn_rs1;
  logic [4:0] insn_rs2;
  logic [4:0] insn_rd;
  logic [`REG_SIZE] insn_rs1_data;
  logic [`REG_SIZE] insn_rs2_data;
  logic [`REG_SIZE] insn_rd_data;
  logic halt;
} stage_writeback_t;

module DatapathPipelinedCache (
    input wire clk,
    input wire rst,

    // AXIL interface to insn memory
    axi_if.manager icache,
    // AXIL interface to data memory/cache
    axi_if.manager dcache,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See the cycle_status.sv file for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

  localparam bit True = 1'b1;
  localparam bit False = 1'b0;

  // cycle counter
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  // TODO: copy in your HW5B datapath as a starting point
  logic illegal_insn;

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpcodeLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpcodeMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpcodeRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpcodeAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpcodeLui = 7'b01_101_11;

  /***************/
  /* FETCH STAGE */
  /***************/
  
  /*
  In the fetch stage, request isns from icache so that by decode, we have the insn
    Read Handshake:
      State: Normal
        icache.ARVALID = True, icache.ARADDR = pc_current, icache.RREADY = True
        (if using a I$) await icache.RVALID before moving on
      State: Stalled
        icache.ARVALID = False, icache.ARADDR = 0, icache.RREADY = False

    Our branch predition logic: pc_next = pc_current + 4 (by default)
  */

  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status;
  logic [`REG_SIZE] f_pc_current, f_pc_next;

  // The f_stall_div stalls when pc if we have a div in execute
  logic f_stall_div, f_stall_load2use, f_stall_memory, f_stall_md_load2use;
  // Need to latch a load2use until the data is written back to regfile
  logic f_reg_stall_load2use1;

  always_comb begin
    if (rst) begin
      icache.ARVALID = False;
      icache.ARADDR = 0;
      icache.RREADY = False;
    end else if (f_stall_memory || f_stall_div || f_stall_load2use || f_reg_stall_load2use1 || f_stall_md_load2use) begin
      icache.ARVALID = False;
      icache.ARADDR = 0;
      icache.RREADY = False;
    end else begin
      icache.ARVALID = True;
      icache.ARADDR = f_pc_current;
      icache.RREADY = True;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 0;
      f_cycle_status <= CYCLE_NO_STALL;
    end else if (f_stall_div) begin
      f_pc_current <= f_pc_current;
      f_cycle_status <= CYCLE_NO_STALL;
    end else if ( x_nop_branch ) begin
      f_pc_current   <= f_pc_next;
      f_cycle_status <= CYCLE_NO_STALL;
    end else if (f_stall_memory || f_stall_load2use || f_reg_stall_load2use1 || f_stall_md_load2use) begin
      f_pc_current <= f_pc_current;
      f_cycle_status <= f_cycle_status;
    end else begin
      f_pc_current <= f_pc_next;
      f_cycle_status <= CYCLE_NO_STALL;
    end
    // Latch to the stall
    f_reg_stall_load2use1 <= f_stall_load2use;
  end
  

  // Here's how to disassemble an insn into a string you can view in GtkWave.
  // Use PREFIX to provide a 1-character tag to identify which stage the insn comes from.
  wire [255:0] f_disasm;
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (f_insn),
      .disasm(f_disasm)
  );

  /****************/
  /* DECODE STAGE */
  /****************/

  stage_decode_t decode_state;
  logic d_stall_div, d_stall_load2use, d_stall_memory, d_nop_branch, d_reg_stall_load2use1, d_stall_md_load2use;

  logic [`INSN_SIZE] d_reg_insn;

  wire [`INSN_SIZE] d_insn;
  assign d_insn = icache.RDATA;
  
  always_ff @(posedge clk) begin
    if (rst) begin
      begin
        decode_state <= '{
          pc: 0,
          insn: 0,
          cycle_status: CYCLE_RESET
        };
      end
    end else if (d_stall_load2use || d_stall_md_load2use) begin
      begin
        decode_state <= '{
          pc: decode_state.pc,
          insn: decode_state.insn,
          cycle_status: decode_state.cycle_status
        };
      end
    end else if (d_reg_stall_load2use1) begin
      begin
        decode_state <= '{
          pc: decode_state.pc,
          insn: decode_state.insn,
          cycle_status: decode_state.cycle_status
        };
      end
      d_reg_stall_load2use1 <= False;
    end else if (d_nop_branch) begin
      begin
        decode_state <= '{
          pc: 0,
          insn: 32'b0,
          cycle_status: CYCLE_TAKEN_BRANCH
        };
      end
    end else if (d_stall_div) begin
      begin
        decode_state <= '{
          pc: decode_state.pc,
          insn: decode_state.insn,
          cycle_status: CYCLE_NO_STALL
        };     
      end
    end else if (d_stall_memory) begin
      begin
        decode_state <= '{
          pc: decode_state.pc,
          insn: decode_state.insn,
          cycle_status: CYCLE_NO_STALL
        };     
      end
    end else begin
      begin
        decode_state <= '{
          pc: f_pc_current,
          insn: d_insn,
          cycle_status: f_cycle_status
        };
      end
    end

    d_reg_stall_load2use1 <= d_stall_load2use;
  end


  wire [255:0] d_disasm;
  Disasm #(
      .PREFIX("D")
  ) disasm_1decode (
      .insn  (d_insn),
      .disasm(d_disasm)
  );

  // Inside the decode stage, we should break the insn into fun3, fun7, imm, etc
  wire [6:0] d_insn_funct7;
  wire [4:0] d_insn_rs2;
  wire [4:0] d_insn_rs1;
  wire [2:0] d_insn_funct3;
  wire [4:0] d_insn_rd;
  wire [`OPCODE_SIZE] d_insn_opcode;

  // split U-type instruction 
  wire [19:0] d_imm_u;
  assign d_imm_u = d_insn[31:12];

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {d_insn_funct7, d_insn_rs2, d_insn_rs1, d_insn_funct3, d_insn_rd, d_insn_opcode} = d_insn;

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] d_imm_i;
  assign d_imm_i = d_insn[31:20];
  wire [ 4:0] d_imm_shamt = d_insn[24:20];

  // S - stores
  wire [11:0] d_imm_s;
  assign d_imm_s[11:5] = d_insn_funct7, d_imm_s[4:0] = d_insn_rd;

  // B - conditionals
  wire [12:0] d_imm_b;
  assign {d_imm_b[12], d_imm_b[10:5]} = d_insn_funct7, {d_imm_b[4:1], d_imm_b[11]} = d_insn_rd, d_imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] d_imm_j;
  assign {d_imm_j[20], d_imm_j[10:1], d_imm_j[11], d_imm_j[19:12], d_imm_j[0]} = {d_insn[31:12], 1'b0};

  wire [`REG_SIZE] d_imm_i_sext = {{20{d_imm_i[11]}}, d_imm_i[11:0]};
  wire [`REG_SIZE] d_imm_s_sext = {{20{d_imm_s[11]}}, d_imm_s[11:0]};
  wire [`REG_SIZE] d_imm_b_sext = {{19{d_imm_b[12]}}, d_imm_b[12:0]};
  wire [`REG_SIZE] d_imm_j_sext = {{11{d_imm_j[20]}}, d_imm_j[20:0]};

  logic [`REG_SIZE] d_rs1_data, d_rs2_data;

  // Division to work
  wire d_insn_div    = d_insn_opcode == OpcodeRegReg && d_insn[31:25] == 7'd1 && d_insn[14:12] == 3'b100;
  wire d_insn_divu   = d_insn_opcode == OpcodeRegReg && d_insn[31:25] == 7'd1 && d_insn[14:12] == 3'b101;
  wire d_insn_rem    = d_insn_opcode == OpcodeRegReg && d_insn[31:25] == 7'd1 && d_insn[14:12] == 3'b110;
  wire d_insn_remu   = d_insn_opcode == OpcodeRegReg && d_insn[31:25] == 7'd1 && d_insn[14:12] == 3'b111;

  wire d_is_Div, d_Div_is_Independent, d_Div_is_Empty;

  // The is_Div tells whether the insn is a divide instruction
  assign d_is_Div = (d_insn_div || d_insn_divu || d_insn_rem || d_insn_remu);

  // Next we need to check whether the div is independent
  assign d_Div_is_Independent = ((d_insn_rs1 != divide_state0.insn_rd)
                              && (d_insn_rs1 != divide_state1.insn_rd)
                              && (d_insn_rs1 != divide_state2.insn_rd)
                              && (d_insn_rs1 != divide_state3.insn_rd)
                              && (d_insn_rs1 != divide_state4.insn_rd)
                              && (d_insn_rs1 != divide_state5.insn_rd)
                              && (d_insn_rs1 != divide_state6.insn_rd)
                              && (d_insn_rs2 != divide_state0.insn_rd)
                              && (d_insn_rs2 != divide_state1.insn_rd)
                              && (d_insn_rs2 != divide_state2.insn_rd)
                              && (d_insn_rs2 != divide_state3.insn_rd)
                              && (d_insn_rs2 != divide_state4.insn_rd)
                              && (d_insn_rs2 != divide_state5.insn_rd)
                              && (d_insn_rs2 != divide_state6.insn_rd));


  // Also want to check if the divide execute stages are empty. 
  assign d_Div_is_Empty = ((divide_state0.insn == 0) 
                        && (divide_state1.insn == 0) 
                        && (divide_state2.insn == 0) 
                        && (divide_state3.insn == 0)
                        && (divide_state4.insn == 0)
                        && (divide_state5.insn == 0)
                        && (divide_state6.insn == 0));


  /*****************/
  /* EXECUTE STAGE */
  /*****************/

  // In the execute stage, we should have all of the opcodes, fun7, fun3, and imms extracted from the insn. 
  // These extracted values should help us identify the type of insn. 
  // The idea behind letting the execute stage figure out the insn type from opcode, fun3, and fun7 is due to 
  // brevity, or a smaller packed struct passed between the D and X stages. 
  

  // Use a packed struct to store the decoded insn values
  stage_execute_t execute_state;

  // X0 - X6 Divider-specific execute stages
  stage_execute_t divide_state0;
  stage_execute_t divide_state1;
  stage_execute_t divide_state2;
  stage_execute_t divide_state3;
  stage_execute_t divide_state4;
  stage_execute_t divide_state5;
  stage_execute_t divide_state6;

  // Here we want to set up the divide-specific execute stages.
  /* 
    The idea is that divide takes 8 clock cycles to finish evaulating
    but our 5 stage pipeline only has 1 stage for execute,
    meaning that the result brought down to the memory stage is
    only after 1 cycle of the divide. 

    We can make it so that divide instructions take its 8 whole cycles
    by having it go through more cycles in the execute stage.

    A divide instruction is fetched from the imem

    In the decode stage, it acts as a normal instruction

    Once the divide instruction enters the execute stage, 
    we check if it is a divide instruction. If it is not, 
    it goes through the normal (X7) stage. 

    However, if it is a div insn, we perform some checks to it.
    If there are currently no div insn THAT is going to write to rs1 or rs2
    of the incoming div insn, the incoming div is allowed to enter 
    the div specific execute stages, X0 - X6.

                    | Decode
                    |
                    v
        (X0)   |         | X7 (for normal insns)
        (X1)   |         |
        (X2)   |         | 
        (X3)   |         |
        (X4)   |         |
        (X5)   |         |
        (X6)   |         |
              -------->  | (The final cycle of the div occurs inside the X7)
                         |
                         | (M)
    
    Independent Div:
      For independent div insns, we know that they won't conflict with each other
      so it is safe to have the div perform both (or more) divs at the same time

    Dependent Div:
      For dependent div insns, we need to stall it inside Decode (with CYCLE_DIV) 
      because we may need to wait for an MX bypass from the older div insn

    So at the decode stage, we decide whether to move the insn into 
      (X7) (non-div)
      (X0) (div and is independent of ones currently inside)
      (D) (hold in decode if it is not independent or it's a non-div and something is currently inside div)

    For instructions trying to enter the execute stage, it can have the following behavior
      - Stalled at Decode (CYCLE_DIV)
      - Move into X7 (Non-Div insn with an empty divider queue)
      - Move into X0 (div insn with an empty or non-conflicting divider queue)

    To determine whether we need to stall fetch and decode, we look at the condition of the decode stage.
      - If the decode can not move into execute, then fetch should be stalled and decode should be stalled
      - fetch is stalled by keeping the same pc, decode should push itself back into itself
  */


  /* X0 should be driven by decode
   X1-X6 is driven by the previous
   Pipelined Divider module should be driven by the X0 stage. 
   In X7, read from the divide

   Multiple conditions to check if something should enter X0
   1. is_Div
   2. is_independent 
  */ 

  logic x_nop_branch, x_nop_load2use, x_stall_memory, x_reg_nop_load2use1, x_nop_md_load2use, x_reg_nop_branch;
  
  always_ff @(posedge clk) begin
    if (rst) begin
      begin
        execute_state <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      end
      divide_state0 <= '{
        default: 0,
        cycle_status: CYCLE_RESET
      };
      divide_state1 <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      divide_state2 <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      divide_state3 <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      divide_state4 <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      divide_state5 <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      divide_state6 <= '{
        default: 0,
        cycle_status: CYCLE_RESET
      };

      x_div_rst <= 1'b1;
    end else if (x_nop_branch) begin
      begin
        execute_state <= '{
          default: 0,
          cycle_status: CYCLE_TAKEN_BRANCH
        };
      end

      divide_state0 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state1 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state2 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state3 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state4 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state5 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state6 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };

        x_div_rst <= 1'b1;
    end else if (x_reg_nop_branch) begin
      begin
        execute_state <= '{
          default: 0,
          cycle_status: CYCLE_TAKEN_BRANCH
        };
      end

      divide_state0 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state1 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state2 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state3 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state4 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state5 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state6 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };

        x_div_rst <= 1'b1;
        x_reg_nop_branch <= False;
    end else if (x_nop_load2use || x_reg_nop_load2use1 || x_nop_md_load2use) begin
      begin
        execute_state <= '{
          default: 0,
          cycle_status: CYCLE_LOAD2USE
        };
      end

      divide_state0 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state1 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state2 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state3 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state4 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state5 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      divide_state6 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      x_div_rst <= 1'b1;
    end else if (x_stall_memory) begin
      begin
        execute_state <= '{
          insn: execute_state.insn,
          insn_funct7: execute_state.insn_funct7,
          insn_rs2: (execute_state.insn_opcode == OpcodeRegImm || execute_state.insn_opcode == OpcodeLui || execute_state.insn_opcode == OpcodeAuipc || execute_state.insn_opcode == OpcodeJal) ? 5'b0 : execute_state.insn_rs2,
          insn_rs1: execute_state.insn_rs1,
          insn_funct3: execute_state.insn_funct3,
          insn_rd: (execute_state.insn_opcode == OpcodeStore || execute_state.insn_opcode == OpcodeBranch) ? 5'b0 : execute_state.insn_rd,
          insn_rs1_data: execute_state.insn_rs1_data,
          insn_rs2_data: execute_state.insn_rs2_data,
          insn_opcode: execute_state.insn_opcode,
          insn_imm_u: execute_state.insn_imm_u,
          insn_imm_i: execute_state.insn_imm_i,
          insn_imm_shamt: execute_state.insn_imm_shamt,
          insn_imm_s: execute_state.insn_imm_s,
          insn_imm_b: execute_state.insn_imm_b,
          insn_imm_j: execute_state.insn_imm_j,
          insn_imm_i_sext: execute_state.insn_imm_i_sext, 
          insn_imm_s_sext: execute_state.insn_imm_s_sext,
          insn_imm_b_sext: execute_state.insn_imm_b_sext,
          insn_imm_j_sext: execute_state.insn_imm_j_sext,
          pc: execute_state.pc,
          cycle_status: CYCLE_NO_STALL
        };
      end

      divide_state0 <= divide_state0;
      divide_state1 <= divide_state1;
      divide_state2 <= divide_state2;
      divide_state3 <= divide_state3;
      divide_state4 <= divide_state4;
      divide_state5 <= divide_state5;
      divide_state6 <= divide_state6;

      x_div_stall <= 1'b1;

    end else begin

      // The other X stages just take whatever was before it
      divide_state1 <= divide_state0;
      divide_state2 <= divide_state1;
      divide_state3 <= divide_state2;
      divide_state4 <= divide_state3;
      divide_state5 <= divide_state4;
      divide_state6 <= divide_state5; 
      x_div_rst <= 1'b0;

      if (d_is_Div && d_Div_is_Independent) begin
        // It's a divide instruction and does not conflict with the insn currently in divide
        begin
          divide_state0 <= '{
            insn: d_insn,
            insn_funct7: d_insn_funct7,
            insn_rs2: (d_insn_opcode == OpcodeRegImm || d_insn_opcode == OpcodeLui || d_insn_opcode == OpcodeAuipc || d_insn_opcode == OpcodeJal) ? 5'b0 : d_insn_rs2,
            insn_rs1: d_insn_rs1,
            insn_funct3: d_insn_funct3,
            insn_rd: (d_insn_opcode == OpcodeStore || d_insn_opcode == OpcodeBranch) ? 5'b0 : d_insn_rd,
            // This is trying to do a WX, MX bypass, but since we are using a non-blocking assignment, must use previous value
            insn_rs1_data: (d_insn_rs1 == 0) ? 32'b0 :
                          (d_insn_rs1 == execute_state.insn_rd) ? x_rd_data :
                          (d_insn_rs1 == memory_state.insn_rd) ? memory_state.insn_rd_data :
                          (d_insn_rs1 == writeback_state.insn_rd) ? writeback_state.insn_rd_data :
                            d_rs1_data,
            insn_rs2_data: (d_insn_rs2 == 0) ? 32'b0 :
                          (d_insn_rs2 == execute_state.insn_rd) ? x_rd_data :
                          (d_insn_rs2 == memory_state.insn_rd) ? memory_state.insn_rd_data :
                          (d_insn_rs2 == writeback_state.insn_rd) ? writeback_state.insn_rd_data :
                            d_rs2_data,
            insn_opcode: d_insn_opcode,
            insn_imm_u: d_imm_u,
            insn_imm_i: d_imm_i,
            insn_imm_shamt: d_imm_shamt,
            insn_imm_s: d_imm_s,
            insn_imm_b: d_imm_b,
            insn_imm_j: d_imm_j,
            insn_imm_i_sext: d_imm_i_sext, 
            insn_imm_s_sext: d_imm_s_sext,
            insn_imm_b_sext: d_imm_b_sext,
            insn_imm_j_sext: d_imm_j_sext,
            pc: decode_state.pc,
            cycle_status: decode_state.cycle_status
          }; 
        end 
        execute_state <= divide_state6;

      end else if (!d_is_Div && d_Div_is_Empty) begin
        // Not a divide instruction and is allowed to go into X
        begin
          execute_state <= '{
            insn: d_insn,
            insn_funct7: d_insn_funct7,
            insn_rs2: (d_insn_opcode == OpcodeRegImm || d_insn_opcode == OpcodeLui || d_insn_opcode == OpcodeAuipc || d_insn_opcode == OpcodeJal) ? 5'b0 : d_insn_rs2,
            insn_rs1: d_insn_rs1,
            insn_funct3: d_insn_funct3,
            insn_rd: (d_insn_opcode == OpcodeStore || d_insn_opcode == OpcodeBranch) ? 5'b0 : d_insn_rd,
            insn_rs1_data: wd_bypassed_rs1_data,
            insn_rs2_data: wd_bypassed_rs2_data,
            insn_opcode: d_insn_opcode,
            insn_imm_u: d_imm_u,
            insn_imm_i: d_imm_i,
            insn_imm_shamt: d_imm_shamt,
            insn_imm_s: d_imm_s,
            insn_imm_b: d_imm_b,
            insn_imm_j: d_imm_j,
            insn_imm_i_sext: d_imm_i_sext, 
            insn_imm_s_sext: d_imm_s_sext,
            insn_imm_b_sext: d_imm_b_sext,
            insn_imm_j_sext: d_imm_j_sext,
            pc: decode_state.pc,
            cycle_status: decode_state.cycle_status
          };
        end
        divide_state0 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };

      end else begin
        divide_state0 <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
        execute_state <= divide_state6; // Pass whatever is coming from the divide stackup
      end
    end

    x_reg_nop_load2use1 <= x_nop_load2use;
    x_reg_nop_branch <= x_nop_branch;
  end

  wire [255:0] x_disasm;
  Disasm #(
    .PREFIX("X")
  ) disasm_2execute (
    .insn  (execute_state.insn),
    .disasm(x_disasm)
  );
  wire [255:0] div0_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide0 (
    .insn  (divide_state0.insn),
    .disasm(div0_disasm)
  );
  wire [255:0] div1_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide1 (
    .insn  (divide_state1.insn),
    .disasm(div1_disasm)
  );
  wire [255:0] div2_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide2 (
    .insn  (divide_state2.insn),
    .disasm(div2_disasm)
  );
  wire [255:0] div3_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide3 (
    .insn  (divide_state3.insn),
    .disasm(div3_disasm)
  );
  wire [255:0] div4_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide4 (
    .insn  (divide_state4.insn),
    .disasm(div4_disasm)
  );
  wire [255:0] div5_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide5 (
    .insn  (divide_state5.insn),
    .disasm(div5_disasm)
  );
  wire [255:0] div6_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide6 (
    .insn  (divide_state6.insn),
    .disasm(div6_disasm)
  );
  
  // Interpreting instructions based on decoded values
  wire x_insn_lui   = execute_state.insn_opcode == OpcodeLui;
  wire x_insn_auipc = execute_state.insn_opcode == OpcodeAuipc;
  wire x_insn_jal   = execute_state.insn_opcode == OpcodeJal;
  wire x_insn_jalr  = execute_state.insn_opcode == OpcodeJalr;

  wire x_insn_beq  = execute_state.insn_opcode == OpcodeBranch && execute_state.insn[14:12] == 3'b000;
  wire x_insn_bne  = execute_state.insn_opcode == OpcodeBranch && execute_state.insn[14:12] == 3'b001;
  wire x_insn_blt  = execute_state.insn_opcode == OpcodeBranch && execute_state.insn[14:12] == 3'b100;
  wire x_insn_bge  = execute_state.insn_opcode == OpcodeBranch && execute_state.insn[14:12] == 3'b101;
  wire x_insn_bltu = execute_state.insn_opcode == OpcodeBranch && execute_state.insn[14:12] == 3'b110;
  wire x_insn_bgeu = execute_state.insn_opcode == OpcodeBranch && execute_state.insn[14:12] == 3'b111;

  wire x_insn_lb  = execute_state.insn_opcode == OpcodeLoad && execute_state.insn[14:12] == 3'b000;
  wire x_insn_lh  = execute_state.insn_opcode == OpcodeLoad && execute_state.insn[14:12] == 3'b001;
  wire x_insn_lw  = execute_state.insn_opcode == OpcodeLoad && execute_state.insn[14:12] == 3'b010;
  wire x_insn_lbu = execute_state.insn_opcode == OpcodeLoad && execute_state.insn[14:12] == 3'b100;
  wire x_insn_lhu = execute_state.insn_opcode == OpcodeLoad && execute_state.insn[14:12] == 3'b101;

  wire x_insn_sb = execute_state.insn_opcode == OpcodeStore && execute_state.insn[14:12] == 3'b000;
  wire x_insn_sh = execute_state.insn_opcode == OpcodeStore && execute_state.insn[14:12] == 3'b001;
  wire x_insn_sw = execute_state.insn_opcode == OpcodeStore && execute_state.insn[14:12] == 3'b010;

  wire x_insn_addi  = execute_state.insn_opcode == OpcodeRegImm && execute_state.insn[14:12] == 3'b000;
  wire x_insn_slti  = execute_state.insn_opcode == OpcodeRegImm && execute_state.insn[14:12] == 3'b010;
  wire x_insn_sltiu = execute_state.insn_opcode == OpcodeRegImm && execute_state.insn[14:12] == 3'b011;
  wire x_insn_xori  = execute_state.insn_opcode == OpcodeRegImm && execute_state.insn[14:12] == 3'b100;
  wire x_insn_ori   = execute_state.insn_opcode == OpcodeRegImm && execute_state.insn[14:12] == 3'b110;
  wire x_insn_andi  = execute_state.insn_opcode == OpcodeRegImm && execute_state.insn[14:12] == 3'b111;

  wire x_insn_slli = execute_state.insn_opcode == OpcodeRegImm && execute_state.insn[14:12] == 3'b001 && execute_state.insn[31:25] == 7'd0;
  wire x_insn_srli = execute_state.insn_opcode == OpcodeRegImm && execute_state.insn[14:12] == 3'b101 && execute_state.insn[31:25] == 7'd0;
  wire x_insn_srai = execute_state.insn_opcode == OpcodeRegImm && execute_state.insn[14:12] == 3'b101 && execute_state.insn[31:25] == 7'b0100000;

  wire x_insn_add  = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b000 && execute_state.insn[31:25] == 7'd0;
  wire x_insn_sub  = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b000 && execute_state.insn[31:25] == 7'b0100000;
  wire x_insn_sll  = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b001 && execute_state.insn[31:25] == 7'd0;
  wire x_insn_slt  = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b010 && execute_state.insn[31:25] == 7'd0;
  wire x_insn_sltu = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b011 && execute_state.insn[31:25] == 7'd0;
  wire x_insn_xor  = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b100 && execute_state.insn[31:25] == 7'd0;
  wire x_insn_srl  = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b101 && execute_state.insn[31:25] == 7'd0;
  wire x_insn_sra  = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b101 && execute_state.insn[31:25] == 7'b0100000;
  wire x_insn_or   = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b110 && execute_state.insn[31:25] == 7'd0;
  wire x_insn_and  = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[14:12] == 3'b111 && execute_state.insn[31:25] == 7'd0;

  wire x_insn_mul    = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b000;
  wire x_insn_mulh   = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b001;
  wire x_insn_mulhsu = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b010;
  wire x_insn_mulhu  = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b011;
  wire x_insn_div    = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b100;
  wire x_insn_divu   = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b101;
  wire x_insn_rem    = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b110;
  wire x_insn_remu   = execute_state.insn_opcode == OpcodeRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b111;

  wire x_insn_ecall = execute_state.insn_opcode == OpcodeEnviron && execute_state.insn[31:7] == 25'd0;
  wire x_insn_fence = execute_state.insn_opcode == OpcodeMiscMem;

  // Now some things we should define here for regFile, memData
  
  // Reg: we need to pass stuff to Writeback stage
  logic x_reg_we;
  logic [`REG_SIZE] x_rd_data;  
  
  // If the insn_cycle status is DIV, then we should stall fetch and decode. This is because we are currently within a division operation. 
  // Only the last insn (the one going from X6 -> X7) should have a cycle status that isn't DIV


  // Control Signals
  logic x_halt;

  // Memory temp logics
  
  // x_dmem_addr_pre_mask refers to the address to referred by the insn before memory alignment (load & store)
  // x_dmem_data refers to the data we want to store into memory (store)
  // x_dmem_we refers to which byte of the 4B memory we want to write to (store)

  logic [`REG_SIZE] x_dmem_addr_pre_mask, x_dmem_data;
  logic [3:0] x_dmem_we;

  // For multiply
  logic [63:0] x_temp_float;

  // Execution block of insn
  always_comb begin
    illegal_insn = 1'b0;
    x_halt = 1'b0;
    
    x_rd_data = 32'b0;
    x_reg_we = 1'b0;
    
    f_pc_next = (((!d_is_Div && d_Div_is_Empty) || (d_is_Div && d_Div_is_Independent)) || !((memory_state.insn_rd == d_insn_rs1) && (memory_state.insn_rd == d_insn_rs2))) ? f_pc_current + 4 : f_pc_current;
    
    // Default control signals
    f_stall_div = !((!d_is_Div && d_Div_is_Empty) || (d_is_Div && d_Div_is_Independent));
    f_stall_load2use = 1'b0;

    d_nop_branch = 1'b0;
    d_stall_load2use = 1'b0;
    d_stall_div = !((!d_is_Div && d_Div_is_Empty) || (d_is_Div && d_Div_is_Independent));
    
    x_nop_branch = 1'b0;
    x_nop_load2use = 1'b0;

    // Multiply stuff
    x_temp_float = bypassed_rs1_data * bypassed_rs2_data;
    x_sign_temp = 0;

    // default loading stuff
    x_dmem_we = 4'b0000; // No memory writes by default
    x_dmem_data = bypassed_rs2_data; // Default store data to rs2
    x_dmem_addr_pre_mask = 32'b0; // Default memory address for load/store

    case (execute_state.insn_opcode)
      OpcodeLui: begin
        if (x_insn_lui) begin
          x_reg_we = 1'b1;
          x_rd_data = execute_state.insn_imm_u << 12;
        end else begin
          illegal_insn = 1'b1;
        end
      end
      OpcodeRegImm: begin
        x_reg_we = 1'b1;
        case (1'b1) 
          x_insn_addi: begin
            x_rd_data = bypassed_rs1_data + execute_state.insn_imm_i_sext;
          end
          x_insn_slti: begin
            x_rd_data = $signed(bypassed_rs1_data) < $signed(execute_state.insn_imm_i_sext) ? 32'b1 : 32'b0;
          end
          x_insn_sltiu: begin
            x_rd_data = (bypassed_rs1_data < execute_state.insn_imm_i_sext) ? 32'b1 : 32'b0;
          end
          x_insn_xori: begin
            x_rd_data = bypassed_rs1_data ^ execute_state.insn_imm_i_sext;
          end
          x_insn_ori: begin
            x_rd_data = bypassed_rs1_data | execute_state.insn_imm_i_sext;
          end
          x_insn_andi: begin
            x_rd_data = bypassed_rs1_data & execute_state.insn_imm_i_sext;
          end
          x_insn_slli: begin
            x_rd_data = bypassed_rs1_data << execute_state.insn_imm_shamt;
          end
          x_insn_srli: begin
            x_rd_data = bypassed_rs1_data >> execute_state.insn_imm_shamt;
          end
          x_insn_srai: begin
            x_rd_data = $signed(bypassed_rs1_data) >>> execute_state.insn_imm_shamt;
          end
          default: begin
            x_rd_data = 32'b0;
          end
        endcase
      end
      OpcodeRegReg: begin
        x_reg_we = 1'b1;
        case (1'b1)
          x_insn_add: begin
            x_rd_data = bypassed_rs1_data + bypassed_rs2_data;
          end
          x_insn_sub: begin
            x_rd_data = bypassed_rs1_data - bypassed_rs2_data;
          end
          x_insn_sll: begin
            x_rd_data = bypassed_rs1_data << bypassed_rs2_data[4:0];
          end
          x_insn_slt: begin
            x_rd_data = $signed(bypassed_rs1_data) < $signed(bypassed_rs2_data) ? 32'b1 : 32'b0;
          end
          x_insn_sltu: begin
            x_rd_data = bypassed_rs1_data < bypassed_rs2_data ? 32'b1 : 32'b0;
          end
          x_insn_xor: begin
            x_rd_data = bypassed_rs1_data ^ bypassed_rs2_data;
          end
          x_insn_srl: begin
            x_rd_data = bypassed_rs1_data >> bypassed_rs2_data[4:0];
          end
          x_insn_sra: begin
            x_rd_data = $signed(bypassed_rs1_data) >>> bypassed_rs2_data[4:0];
          end
          x_insn_or: begin
            x_rd_data = bypassed_rs1_data | bypassed_rs2_data;
          end
          x_insn_and: begin
            x_rd_data = bypassed_rs1_data & bypassed_rs2_data;
          end
          x_insn_mul: begin
            x_temp_float = (bypassed_rs1_data * bypassed_rs2_data);
            x_rd_data = x_temp_float[31:0]; 
          end
          x_insn_mulh: begin
            x_temp_float = ($signed(bypassed_rs1_data) * $signed(bypassed_rs2_data));
            x_rd_data = x_temp_float[63:32];
          end
          x_insn_mulhsu: begin
            x_temp_float = {{32{bypassed_rs1_data[31]}}, bypassed_rs1_data} * {{32{1'b0}}, bypassed_rs2_data};
            x_rd_data = x_temp_float[63:32];
          end
          x_insn_mulhu: begin
            x_temp_float = (bypassed_rs1_data * bypassed_rs2_data);
            x_rd_data = x_temp_float[63:32];
          end
          x_insn_div: begin
            if (execute_state.insn_rs2_data == 0) begin
              x_rd_data = 32'hffffffff;
            end else begin
              x_sign_temp = execute_state.insn_rs1_data[31] ^ execute_state.insn_rs2_data[31];  
              x_rd_data = x_sign_temp ? (~x_divider_quotient + 1) : x_divider_quotient;
            end
          end
          x_insn_divu: begin 
            x_rd_data = x_divider_quotient;
          end
          x_insn_rem: begin 
            x_sign_temp = execute_state.insn_rs1_data[31];
            x_rd_data = x_sign_temp ? (~x_divider_remainder + 1) : x_divider_remainder;
          end
          x_insn_remu: begin 
            x_rd_data = x_divider_remainder;
          end
          default: begin
            x_rd_data = 32'b0;
          end
        endcase
      end
      OpcodeBranch: begin
        case (1'b1)
          (x_insn_bne && (bypassed_rs1_data != bypassed_rs2_data)): begin
            // If we take the branch, we have to bubble the current insn in F and D stage
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
            // Clear Fetch stage
            d_nop_branch = 1'b1;
            // Clear Decode stage
            x_nop_branch = 1'b1;
          end
          (x_insn_beq && (bypassed_rs1_data == bypassed_rs2_data)): begin
            // If we take the branch, we have to bubble the current insn in F and D stage
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
            // Clear Fetch stage
            d_nop_branch = 1'b1;
            // Clear Decode stage
            x_nop_branch = 1'b1;
          end
          (x_insn_blt && ($signed(bypassed_rs1_data) < $signed(bypassed_rs2_data))): begin
            // If we take the branch, we have to bubble the current insn in F and D stage
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
            // Clear Fetch stage
            d_nop_branch = 1'b1;
            // Clear Decode stage
            x_nop_branch = 1'b1;
          end
          (x_insn_bge && ($signed(bypassed_rs1_data) >= $signed(bypassed_rs2_data))): begin
            // If we take the branch, we have to bubble the current insn in F and D stage
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
            // Clear Fetch stage
            d_nop_branch = 1'b1;
            // Clear Decode stage
            x_nop_branch = 1'b1;
          end
          (x_insn_bltu && (bypassed_rs1_data < bypassed_rs2_data)): begin
            // If we take the branch, we have to bubble the current insn in F and D stage
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
            // Clear Fetch stage
            d_nop_branch = 1'b1;
            // Clear Decode stage
            x_nop_branch = 1'b1;
          end
          (x_insn_bgeu && (bypassed_rs1_data >= bypassed_rs2_data)): begin
            // If we take the branch, we have to bubble the current insn in F and D stage
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
            // Clear Fetch stage
            d_nop_branch = 1'b1;
            // Clear Decode stage
            x_nop_branch = 1'b1;
          end
          default: begin
            // pc += 4 automatically anyways.
          end
        endcase
      end
      OpcodeLoad: begin
        // Allows this insn to write to registers
        x_reg_we = 1'b1;
        // Sets up the address to load data from
        x_dmem_addr_pre_mask = bypassed_rs1_data + execute_state.insn_imm_i_sext;

        // This stalls the decode -> execute insn pipline under 2 conditions
        // Load to use, 
        // Or if a store needs to use a register modified by a load insn
        if ((execute_state.insn_rd == d_insn_rs1 && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc) || (d_insn_opcode != OpcodeRegImm && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc && d_insn_opcode != OpcodeJal && d_insn_opcode != OpcodeLoad && d_insn_opcode != OpcodeStore && execute_state.insn_rd == d_insn_rs2)) begin

          f_stall_load2use = 1'b1;
          d_stall_load2use = 1'b1;
          x_nop_load2use = 1'b1;
        end
      end
      OpcodeStore: begin
        f_pc_next = f_pc_current + 4;
        x_dmem_addr_pre_mask = bypassed_rs1_data + execute_state.insn_imm_s_sext;
        //adjusted_load_mem_temp = bypassed_rs2_data >> (8 * addr_pre_mask_temp[1:0]);
        x_dmem_data = bypassed_rs2_data << (8 * x_dmem_addr_pre_mask[1:0]);
        if (x_insn_sb) begin
          x_dmem_we = 4'b0001 << x_dmem_addr_pre_mask[1:0];
          //data_mem_temp = rs2_data; //{24'b0, {adjusted_load_mem_temp[7:0]}};
        end else if (x_insn_sh) begin
          x_dmem_we = 4'b0011 << x_dmem_addr_pre_mask[1:0];
          //data_mem_temp = rs2_data; //{16'b0, {adjusted_load_mem_temp[15:0]}};
        end else if (x_insn_sw) begin
          x_dmem_we = 4'b1111;
          //data_mem_temp = rs2_data;
        end else begin
          illegal_insn = 1'b1;
        end
      end
      OpcodeEnviron: begin
        if (x_insn_ecall) begin
          x_halt = 1'b1;
        end
      end
      OpcodeMiscMem: begin
         if (x_insn_fence) begin
          // Always increment the PC counter by 4
          f_pc_next = f_pc_current + 4;
        end else begin
          illegal_insn = 1'b1;
        end
      end
      OpcodeAuipc: begin
        case (1'b1)
        x_insn_auipc: begin
          x_reg_we = 1'b1;
          x_rd_data = execute_state.pc + (execute_state.insn_imm_u << 12);
        end
        default: begin
          illegal_insn = 1'b1;
        end
        endcase
      end
      OpcodeJal: begin
        case (1'b1)
          x_insn_jal: begin
            x_reg_we = 1'b1;
            x_rd_data = execute_state.pc + 4;
            f_pc_next = execute_state.pc + execute_state.insn_imm_j_sext;
            d_nop_branch = 1'b1;
            x_nop_branch = 1'b1;
          end
          default: begin
            illegal_insn = 1'b1;
          end
        endcase
      end
      OpcodeJalr: begin
        case (1'b1)
          x_insn_jalr: begin
            x_reg_we = 1'b1;
            x_rd_data = execute_state.pc + 4;
            f_pc_next = (bypassed_rs1_data + execute_state.insn_imm_i_sext) & ~32'b1;
            d_nop_branch = 1'b1;
            x_nop_branch = 1'b1;
          end
          default: begin
            illegal_insn = 1'b1;
          end
        endcase
      end
      default: begin
        illegal_insn = 1'b1;
      end
    endcase
  end
  wire div0_insn_div    = divide_state0.insn_opcode == OpcodeRegReg && divide_state0.insn[31:25] == 7'd1 && divide_state0.insn[14:12] == 3'b100;
  wire div0_insn_divu   = divide_state0.insn_opcode == OpcodeRegReg && divide_state0.insn[31:25] == 7'd1 && divide_state0.insn[14:12] == 3'b101;
  wire div0_insn_rem    = divide_state0.insn_opcode == OpcodeRegReg && divide_state0.insn[31:25] == 7'd1 && divide_state0.insn[14:12] == 3'b110;
  wire div0_insn_remu   = divide_state0.insn_opcode == OpcodeRegReg && divide_state0.insn[31:25] == 7'd1 && divide_state0.insn[14:12] == 3'b111;

  // Pipelined Divider
  logic [`REG_SIZE] x_divider_dividend, x_divider_divisor, x_divider_remainder, x_divider_quotient;  
  logic x_sign_temp;
  logic x_div_rst, x_div_stall;

  // Evaluate the dividend and divisor for values in divider_stage0
  always_comb begin
    case (1'b1) 
      (div0_insn_div || div0_insn_rem): begin
        x_divider_dividend = divide_state0.insn_rs1_data[31] ? (~divide_state0.insn_rs1_data + 1) : divide_state0.insn_rs1_data; 
        x_divider_divisor  = divide_state0.insn_rs2_data[31] ? (~divide_state0.insn_rs2_data + 1) : divide_state0.insn_rs2_data;
      end
      (div0_insn_divu || div0_insn_remu): begin
        x_divider_dividend = divide_state0.insn_rs1_data;
        x_divider_divisor = divide_state0.insn_rs2_data;
      end
      default: begin
        x_divider_dividend = 32'b0;
        x_divider_divisor = 32'b0;
      end
    endcase
  end
  
  DividerUnsignedPipelined divider(.clk(clk), .rst(x_div_rst), .i_dividend(x_divider_dividend), .i_divisor(x_divider_divisor), .o_quotient(x_divider_quotient), .o_remainder(x_divider_remainder));

    /****************/
   /* MEMORY STAGE */
  /****************/

  stage_memory_t memory_state;

  logic m_stall_memory;

  // Update the memory state registers with the execute stage values

  always_ff @(posedge clk) begin
    if (rst) begin
      begin
        memory_state <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
        m_reg_sent_data <= 1'b0;
      end
    end else if (m_stall_memory) begin
        memory_state <= memory_state;
    end else begin
      begin
        memory_state <= '{
          pc: execute_state.pc,
          insn: execute_state.insn,
          insn_opcode: execute_state.insn_opcode,
          cycle_status: execute_state.cycle_status,
          we: x_reg_we,
          insn_rs1: execute_state.insn_rs1,
          insn_rs2: execute_state.insn_rs2,
          insn_rd: execute_state.insn_rd,
          insn_rs1_data: bypassed_rs1_data,
          insn_rs2_data: bypassed_rs2_data,
          insn_rd_data: x_rd_data,
          // Memory Stuff
          dmem_addr: x_dmem_addr_pre_mask,
          dmem_data: x_dmem_data,
          dmem_we: x_dmem_we,
          //Control stuff
          halt: x_halt
        };
      end
      m_reg_sent_data <= (dcache.RVALID || dcache.BVALID) ? False : m_sent_data;
    end
  end

  // In the memory stage, we want to implement the load and stores
  
  // Store will set the write-enable and data stored
  //assign store_data_to_dmem = m_dmem_write_data; //6b: if we have a store, we need to check from memory(changes these assigns with the right axilmemory code)
  //assign store_we_to_dmem = memory_state.dmem_we;

  // Address to fetch or store data from
  //assign addr_to_dmem = memory_state.dmem_addr & ~32'b11; //2 byte alignment 6b: change with right axilmemory code
  //assign m_dmem_data_pre_mask = load_data_from_dmem; //6b: change with right axilmemory inputs
  
  logic m_sent_data, m_reg_sent_data;

  always_comb begin
    // Default values
    dcache.ARVALID = False; dcache.ARADDR = 0; dcache.RREADY = True;
    dcache.AWVALID = False; dcache.AWADDR = 0;
    dcache.WVALID  = False; dcache.WDATA  = 0; dcache.WSTRB = 4'b0000;
    dcache.BREADY  = True;

    m_sent_data = False;
    f_stall_md_load2use = False;
    d_stall_md_load2use = False;
    x_nop_md_load2use = False;

    case (memory_state.insn_opcode)
      OpcodeLoad: begin
        if (!m_sent_data) begin
          dcache.ARVALID = True;
          dcache.ARADDR = memory_state.dmem_addr & ~32'b11;
          m_sent_data = True;
        end else begin
          dcache.ARVALID = False;
          dcache.ARADDR  = 0;
        end

        f_stall_md_load2use = w_received_data && ((memory_state.insn_rd == d_insn_rs1 && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc) || (d_insn_opcode != OpcodeRegImm && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc && d_insn_opcode != OpcodeJal && d_insn_opcode != OpcodeLoad && d_insn_opcode != OpcodeStore && memory_state.insn_rd == d_insn_rs2));
        d_stall_md_load2use = w_received_data && ((memory_state.insn_rd == d_insn_rs1 && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc) || (d_insn_opcode != OpcodeRegImm && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc && d_insn_opcode != OpcodeJal && d_insn_opcode != OpcodeLoad && d_insn_opcode != OpcodeStore && memory_state.insn_rd == d_insn_rs2));
        x_nop_md_load2use = w_received_data && ((memory_state.insn_rd == d_insn_rs1 && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc) || (d_insn_opcode != OpcodeRegImm && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc && d_insn_opcode != OpcodeJal && d_insn_opcode != OpcodeLoad && d_insn_opcode != OpcodeStore && memory_state.insn_rd == d_insn_rs2));
      end
      OpcodeStore: begin
        if (!m_sent_data) begin
          dcache.AWVALID = True;
          dcache.AWADDR  = memory_state.dmem_addr & ~32'b11;
          dcache.WVALID  = True;
          dcache.WDATA   = wm_bypassed_rs2_data << (8 * memory_state.dmem_addr[1:0]);
          dcache.WSTRB   = memory_state.dmem_we;
          m_sent_data = True;
        end else begin
          dcache.AWVALID = False;
          dcache.AWADDR  = 0;
          dcache.WVALID  = False;
          dcache.WDATA   = 0;
          dcache.WSTRB   = 0;
        end
        /*
        f_stall_md_load2use = (memory_state.insn_rd == d_insn_rs1) || (memory_state.insn_rd == d_insn_rs2);
        d_stall_md_load2use = (memory_state.insn_rd == d_insn_rs1) || (memory_state.insn_rd == d_insn_rs2);
        x_nop_md_load2use = (memory_state.insn_rd == d_insn_rs1) || (memory_state.insn_rd == d_insn_rs2);
        */
      end
      default: begin
        m_sent_data = True;
      end
    endcase
  end

  //6b: if there is a load or store in the memory stage, stall prior stages
  // If the next instruction is dependent, then stall in decode registers
  // If the next instruction is independent, then stall in the execute registers
  //How do we know when we get the data from the caches? 
  //Basic logic: if the memory arrives,m_dmem_data_pre_mask is changed; else, wait
  //Also, while waiting, we need to have a stall signal that says we are stalling, maybe just load2use??

  wire [255:0] m_disasm;
  Disasm #(
    .PREFIX("M")
  ) disasm_3memory (
    .insn  (memory_state.insn),
    .disasm(m_disasm)
  );

    /*******************/
   /* WRITEBACK STAGE */
  /*******************/

  /*
    In the writeback stage, we write the data to the register file.
    If we are waiting for a load or store, we need to wait until the data arrives before writing to the register file.

    For most instructions:
      Immediately assign the memory_state to the writeback_state

    For a Load:
      Wait for RREADY signal from the cache. On the same cycle,
      read RDATA, use that as dmem_data, and assign it to the writeback_state.

    For a Store:
      Wait for BVALID signal from the cache, and then allow signal into the writeback stage.
  */
  
  stage_writeback_t writeback_state;

  logic w_received_data;

  logic w_insn_previous_was_load, w_reg_insn_previous_was_load;
  logic [`REG_SIZE] w_rd_data_previous, w_reg_rd_data_previous;
  logic [4:0] w_rd_previous, w_reg_rd_previous;

  always_ff @(posedge clk) begin
    if (rst) begin
      begin
        writeback_state <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      end
    end else if (w_received_data) begin
      begin
        writeback_state <= '{
          pc: memory_state.pc,
          insn: memory_state.insn,
          insn_opcode: memory_state.insn_opcode,
          cycle_status: memory_state.cycle_status,
          we: memory_state.we,
          dmem_addr: memory_state.dmem_addr,
          insn_rs1: memory_state.insn_rs1,
          insn_rs2: memory_state.insn_rs2,
          insn_rd: memory_state.insn_rd,
          insn_rs1_data: memory_state.insn_rs1_data,
          insn_rs2_data: memory_state.insn_rs2_data,
          insn_rd_data: memory_state.insn_rd_data,
          halt: memory_state.halt
        };
      end
    end else begin
      writeback_state <= writeback_state;
    end
    w_reg_insn_previous_was_load <= w_insn_previous_was_load;
    w_reg_rd_previous <= w_rd_previous;
    w_reg_rd_data_previous <= w_rd_data_previous;
  end
  
  always_comb begin

    f_stall_memory = False;
    d_stall_memory = False;
    x_stall_memory = False;
    m_stall_memory = False;
    // Default case for a non-store/load instruction
    w_received_data = True;  

    w_insn_previous_was_load = False; // Only true on sucessful load insns
    w_rd_data_previous = w_rd_data;
    w_rd_previous = writeback_state.insn_rd;

    case (writeback_state.insn_opcode) 
      OpcodeLoad: begin
        if(dcache.RVALID) begin
          w_received_data = True;
          w_insn_previous_was_load = True;
        end else begin
          // Waiting for the data to arrive
          w_received_data = False;
          // Start Stalling the pipeline
          f_stall_memory = True;
          d_stall_memory = True; 
          x_stall_memory = True;
          m_stall_memory = True;
        end
      end
      OpcodeStore: begin
        if (dcache.BVALID) begin
          w_received_data = True;
          w_insn_previous_was_load = True;
        end else begin          
          w_received_data = False;
          
          f_stall_memory = True;
          d_stall_memory = True; 
          x_stall_memory = True;
          m_stall_memory = True;          
        end
      end
      default: begin

      end
    endcase
  end

  logic [`REG_SIZE] w_dmem_data, w_rd_data;

  always_comb begin

    w_rd_data = writeback_state.insn_rd_data;

    if (writeback_state.insn_opcode == OpcodeLoad) begin

      w_dmem_data = dcache.RDATA >> (8 * writeback_state.dmem_addr[1:0]);
      
      case (1'b1)
        (writeback_state.insn[14:12] == 3'b000) : begin
          w_rd_data = {{24{w_dmem_data[7]}}, w_dmem_data[7:0]};    
        end
        (writeback_state.insn[14:12] == 3'b001) : begin
          w_rd_data = {{16{w_dmem_data[15]}}, w_dmem_data[15:0]};
        end
        (writeback_state.insn[14:12] == 3'b010) : begin
          w_rd_data = w_dmem_data[31:0];
        end
        (writeback_state.insn[14:12] == 3'b100) : begin
          w_rd_data = {24'b0, w_dmem_data[7:0]};
        end
        (writeback_state.insn[14:12] == 3'b101) : begin
          w_rd_data = {16'b0, w_dmem_data[15:0]};
        end
      endcase
    end
  end

  wire [255:0] w_disasm;
  Disasm #(
    .PREFIX("W")
  ) disasm_4writeback (
    .insn  (writeback_state.insn),
    .disasm(w_disasm)
  );

    /*******************/
   /* Bypassing Logic */
  /*******************/

  // These values were checked for potential bypasses and MAY contain an updated value.

  wire [`REG_SIZE] bypassed_rs1_data, bypassed_rs2_data, wd_bypassed_rs1_data, wd_bypassed_rs2_data, wm_bypassed_rs2_data;

  assign bypassed_rs1_data = (execute_state.insn_rs1 == 0) ? 32'b0 :
                             ((execute_state.insn_rs1 == memory_state.insn_rd) && (memory_state.insn_opcode != OpcodeLoad)) ? memory_state.insn_rd_data : //6b: we have to wait if this instruction is a load
                             (execute_state.insn_rs1 == writeback_state.insn_rd) ? writeback_state.insn_rd_data : 
                             (w_reg_insn_previous_was_load && execute_state.insn_rs1 == w_reg_rd_previous) ? w_reg_rd_data_previous : 
                             execute_state.insn_rs1_data;

  assign bypassed_rs2_data = (execute_state.insn_rs2 == 0) ? 32'b0 :
                             ((execute_state.insn_rs2 == memory_state.insn_rd) && (memory_state.insn_opcode != OpcodeLoad)) ? memory_state.insn_rd_data :
                             (execute_state.insn_rs2 == writeback_state.insn_rd) ? writeback_state.insn_rd_data : 
                             (w_reg_insn_previous_was_load && execute_state.insn_rs2 == w_reg_rd_previous) ? w_reg_rd_data_previous : 
                             execute_state.insn_rs2_data;
  
  assign wd_bypassed_rs1_data = (d_insn_rs1 == 0) ? 32'b0 :
                                (d_insn_rs1 == writeback_state.insn_rd) ? writeback_state.insn_rd_data : // This should be w_rd_data_previous
                                (w_reg_insn_previous_was_load && d_insn_rs1 == w_reg_rd_previous) ? w_reg_rd_data_previous : 
                                d_rs1_data;
  assign wd_bypassed_rs2_data = (d_insn_rs2 == 0) ? 32'b0 :
                                (d_insn_rs2 == writeback_state.insn_rd) ? writeback_state.insn_rd_data : // This should be w_rd_data_previous
                                (w_reg_insn_previous_was_load && d_insn_rs2 == w_reg_rd_previous) ? w_reg_rd_data_previous : 
                                d_rs2_data;

  /* For memory <- writeback bypassing. Only rs2 is needed. Assuming only Store insn needs a WM bypass
  assign wm_bypassed_rs2_data = (memory_state.insn_rs2 == 0) ? 32'b0 :
                                (memory_state.insn_rs2 == writeback_state.insn_rd) ? w_rd_data :
                                (w_reg_insn_previous_was_load && d_insn_rs2 == w_reg_rd_previous) ? w_reg_rd_data_previous : 
                                memory_state.insn_rs2_data;
*/
  // In your bypass section, replace the old assign with this:
  wire is_wb_write  = w_received_data && writeback_state.we && (writeback_state.insn_rd != 0);
  wire is_prev_load = w_reg_insn_previous_was_load && (w_reg_rd_previous    != 0);

  assign wm_bypassed_rs2_data = 
      (memory_state.insn_rs2 == 0)                     ? 32'b0 :         
      (is_wb_write && memory_state.insn_rs2 == writeback_state.insn_rd)
                                                      ? w_rd_data :     
      (is_prev_load && memory_state.insn_rs2 == w_reg_rd_previous)
                                                      ? w_reg_rd_data_previous : 
                                                        memory_state.insn_rs2_data;

  wire w_reg_file_we;
  wire [4:0] w_reg_file_rd;
  wire [`REG_SIZE] w_reg_file_rd_data;

  assign w_reg_file_we = w_received_data ? writeback_state.we : 0;
  assign w_reg_file_rd = w_received_data ? writeback_state.insn_rd : 0;
  assign w_reg_file_rd_data = w_received_data ? w_rd_data : 0;


  // Set up the RegFile. Read during the execute stage, and write during the writeback stage.
  RegFile rf (.clk(clk), 
              .rst(rst),
              .we(w_reg_file_we),
              .rd(w_reg_file_rd),
              .rd_data(w_reg_file_rd_data),
              .rs1(d_insn_rs1),
              .rs1_data(d_rs1_data),
              .rs2(d_insn_rs2),
              .rs2_data(d_rs2_data));

  // Setting the trace values for cycle specific tests
  assign trace_writeback_pc = w_received_data ? writeback_state.pc : 0;
  assign trace_writeback_insn = w_received_data ? writeback_state.insn : 0;
  assign trace_writeback_cycle_status = w_received_data ? writeback_state.cycle_status : CYCLE_DCACHE_MISS;

  // Setting the control signals
  assign halt = w_received_data ? writeback_state.halt : 0;

endmodule // DatapathPipelinedCache

module Processor (
    input wire                       clk,
    input wire                       rst,
    output logic                     halt,
    output wire [`REG_SIZE]          trace_writeback_pc,
    output wire [`INSN_SIZE]         trace_writeback_insn,
    output                           cycle_status_e trace_writeback_cycle_status
);

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  axi_if axi_data_cache ();
  axi_if axi_insn_cache ();
  // memory is dual-ported, to connect to both I$ and D$
  axi_if axi_mem_ro ();
  axi_if axi_mem_rw ();

AxilMemory #(.NUM_WORDS(8192)) memory (
  .ACLK(clk),
  .ARESETn(~rst),
  .port_ro(axi_mem_ro.subord),
  .port_rw(axi_mem_rw.subord)
);

`ifdef ENABLE_INSN_CACHE
  AxilCache #(
    .BLOCK_SIZE_BITS(32),
    .NUM_SETS(16)) icache (
    .ACLK(clk),
    .ARESETn(~rst),
    .proc(axi_insn_cache.subord),
    .mem(axi_mem_ro.manager)
  );
`endif
`ifdef ENABLE_DATA_CACHE
  AxilCache #(
    .BLOCK_SIZE_BITS(32),
    .NUM_SETS(16)) dcache (
    .ACLK(clk),
    .ARESETn(~rst),
    .proc(axi_data_cache.subord),
    .mem(axi_mem_rw.manager)
  );
`endif

  DatapathPipelinedCache datapath (
      .clk(clk),
      .rst(rst),
`ifdef ENABLE_INSN_CACHE
      .icache(axi_insn_cache.manager),
`else
      .icache(axi_mem_ro.manager),
`endif
`ifdef ENABLE_DATA_CACHE
      .dcache(axi_data_cache.manager),
`else
      .dcache(axi_mem_rw.manager),
`endif
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule

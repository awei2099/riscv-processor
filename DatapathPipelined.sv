`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`ifndef DIVIDER_STAGES
`define DIVIDER_STAGES 8
`endif

`ifndef SYNTHESIS
`include "../hw3-singlecycle/RvDisassembler.sv"
`endif
`include "../hw2b-cla/cla.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "cycle_status.sv"

module Disasm #(
    byte PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
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
endmodule

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

  assign regs[0] = 32'b0;

  for (i = 1; i < NumRegs; i = i + 1) begin 
    always_ff @(posedge clk) begin
      if (rst) begin
        regs[i] <= 32'b0;
      end else begin
        regs[i] <= (rd == i && we) ? rd_data : regs[i];
      end
    end
  end

  assign rs1_data = regs[rs1];
  assign rs2_data = regs[rs2];
endmodule

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
} stage_decode_t;

typedef struct packed {
  logic [`INSN_SIZE] insn;
  logic [6:0] insn_funct7;
  logic [4:0] insn_rs2;
  logic [4:0] insn_rs1;
  logic [2:0] insn_funct3;
  logic [4:0] insn_rd;
  logic [`OPCODE_SIZE] insn_opcode;
  logic [`REG_SIZE] insn_rs1_data;
  logic [`REG_SIZE] insn_rs2_data;

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

  // memory
  logic [`REG_SIZE] dmem_addr;
  logic [`REG_SIZE] dmem_data;
  logic [3:0] dmem_we;
  logic halt;
} stage_memory_t;

typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic we;

  logic [4:0] insn_rs1;
  logic [4:0] insn_rs2;
  logic [4:0] insn_rd;
  logic [`REG_SIZE] insn_rs1_data;
  logic [`REG_SIZE] insn_rs2_data;
  logic [`REG_SIZE] insn_rd_data;

  logic halt;
} stage_writeback_t;

module DatapathPipelined (
    input wire clk,
    input wire rst,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`INSN_SIZE] insn_from_imem,
    // dmem is read/write
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See the cycle_status.sv file for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

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

  // cycle counter, not really part of any stage but useful for orienting within GtkWave
  // do not rename this as the testbench uses this value
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  /***************/
  /* FETCH STAGE */
  /***************/
  logic [`REG_SIZE] f_pc_current;
  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status;

  assign pc_to_imem = f_pc_current;
  assign f_insn = insn_from_imem;

  logic [`REG_SIZE] f_pc_next;

  logic f_stall_div, f_stall_load2use;

  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 0;
      f_cycle_status <= CYCLE_NO_STALL;
    end else if (f_stall_div) begin
      f_pc_current <= f_pc_current;
      f_cycle_status <= CYCLE_DIV;
    end else if (f_stall_load2use) begin
      f_pc_current <= f_pc_current;
      f_cycle_status <= f_cycle_status;
    end else begin
      f_pc_current <= f_pc_next;
      f_cycle_status <= CYCLE_NO_STALL;
    end
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
  logic d_bubble, d_stall_load2use, d_stall_div;

  stage_decode_t decode_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      begin
        decode_state <= '{
          pc: 0,
          insn: 0,
          cycle_status: CYCLE_RESET
        };
      end
    end else if (d_bubble) begin
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
    end else if (d_stall_load2use) begin
      begin
        decode_state <= '{
          pc: decode_state.pc,
          insn: decode_state.insn,
          cycle_status: decode_state.cycle_status
        };     
      end
    end else begin
      begin
        decode_state <= '{
          pc: f_pc_current,
          insn: f_insn,
          cycle_status: f_cycle_status
        };
      end
    end
  end

  wire [255:0] d_disasm;
  Disasm #(
      .PREFIX("D")
  ) disasm_1decode (
      .insn  (decode_state.insn),
      .disasm(d_disasm)
  );

  wire [6:0] d_insn_funct7;
  wire [4:0] d_insn_rs2;
  wire [4:0] d_insn_rs1;
  wire [2:0] d_insn_funct3;
  wire [4:0] d_insn_rd;
  wire [`OPCODE_SIZE] d_insn_opcode;

  wire [19:0] d_imm_u;
  assign d_imm_u = decode_state.insn[31:12];

  assign {d_insn_funct7, d_insn_rs2, d_insn_rs1, d_insn_funct3, d_insn_rd, d_insn_opcode} = decode_state.insn;

  wire [11:0] d_imm_i;
  assign d_imm_i = decode_state.insn[31:20];
  wire [ 4:0] d_imm_shamt = decode_state.insn[24:20];

  wire [11:0] d_imm_s;
  assign d_imm_s[11:5] = d_insn_funct7, d_imm_s[4:0] = d_insn_rd;

  wire [12:0] d_imm_b;
  assign {d_imm_b[12], d_imm_b[10:5]} = d_insn_funct7, {d_imm_b[4:1], d_imm_b[11]} = d_insn_rd, d_imm_b[0] = 1'b0;

  wire [20:0] d_imm_j;
  assign {d_imm_j[20], d_imm_j[10:1], d_imm_j[11], d_imm_j[19:12], d_imm_j[0]} = {decode_state.insn[31:12], 1'b0};

  wire [`REG_SIZE] d_imm_i_sext = {{20{d_imm_i[11]}}, d_imm_i[11:0]};
  wire [`REG_SIZE] d_imm_s_sext = {{20{d_imm_s[11]}}, d_imm_s[11:0]};
  wire [`REG_SIZE] d_imm_b_sext = {{19{d_imm_b[12]}}, d_imm_b[12:0]};
  wire [`REG_SIZE] d_imm_j_sext = {{11{d_imm_j[20]}}, d_imm_j[20:0]};

  logic [`REG_SIZE] d_rs1_data, d_rs2_data;

  wire d_div_insn, d_div_independent, d_div_free;

  assign d_div_insn = (d_insn_div || d_insn_divu || d_insn_rem || d_insn_remu);

  wire d_insn_div    = d_insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b100;
  wire d_insn_divu   = d_insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b101;
  wire d_insn_rem    = d_insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b110;
  wire d_insn_remu   = d_insn_opcode == OpcodeRegReg && decode_state.insn[31:25] == 7'd1 && decode_state.insn[14:12] == 3'b111;

  assign d_div_independent = ((d_insn_rs1 != divide_state[0].insn_rd)
                              && (d_insn_rs1 != divide_state[1].insn_rd)
                              && (d_insn_rs1 != divide_state[2].insn_rd)
                              && (d_insn_rs1 != divide_state[3].insn_rd)
                              && (d_insn_rs1 != divide_state[4].insn_rd)
                              && (d_insn_rs1 != divide_state[5].insn_rd)
                              && (d_insn_rs1 != divide_state[6].insn_rd)
                              && (d_insn_rs2 != divide_state[0].insn_rd)
                              && (d_insn_rs2 != divide_state[1].insn_rd)
                              && (d_insn_rs2 != divide_state[2].insn_rd)
                              && (d_insn_rs2 != divide_state[3].insn_rd)
                              && (d_insn_rs2 != divide_state[4].insn_rd)
                              && (d_insn_rs2 != divide_state[5].insn_rd)
                              && (d_insn_rs2 != divide_state[6].insn_rd));

  assign d_div_free = ((divide_state[0].insn == 0) 
                        && (divide_state[1].insn == 0) 
                        && (divide_state[2].insn == 0) 
                        && (divide_state[3].insn == 0)
                        && (divide_state[4].insn == 0)
                        && (divide_state[5].insn == 0)
                        && (divide_state[6].insn == 0));


  /*****************/
  /* EXECUTE STAGE */
  /*****************/
  stage_execute_t execute_state;

  // divider execute blocks
  stage_execute_t [6:0] divide_state;

  logic x_bubble, x_bubble_load2use;

  always_ff @(posedge clk) begin
    if (rst) begin
      begin
        execute_state <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      end
      for (int i = 0; i < 7; i++) begin
        divide_state[i] <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      end
    end else if (x_bubble) begin
      begin
        execute_state <= '{
          default: 0,
          cycle_status: CYCLE_TAKEN_BRANCH
        };
      end
      for (int i = 0; i < 7; i++) begin
        divide_state[i] <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
      end
  end else if (x_bubble_load2use) begin
    begin
      execute_state <= '{
        default: 0,
        cycle_status: CYCLE_LOAD2USE
      };
    end
    for (int i = 0; i < 7; i++) begin
        divide_state[i] <= '{
          default: 0,
          cycle_status: CYCLE_DIV
        };
    end

  end else begin
    divide_state[1] <= divide_state[0];
    divide_state[2] <= divide_state[1];
    divide_state[3] <= divide_state[2];
    divide_state[4] <= divide_state[3];
    divide_state[5] <= divide_state[4];
    divide_state[6] <= divide_state[5];

    if (d_div_insn && d_div_independent) begin
      begin
        divide_state[0] <= '{
          insn: decode_state.insn,
          insn_funct7: d_insn_funct7,
          insn_rs2: (d_insn_opcode == OpcodeRegImm || d_insn_opcode == OpcodeLui || d_insn_opcode == OpcodeAuipc || d_insn_opcode == OpcodeJal) ? 5'b0 : d_insn_rs2,
          insn_rs1: d_insn_rs1,
          insn_funct3: d_insn_funct3,
          insn_rd: (d_insn_opcode == OpcodeStore || d_insn_opcode == OpcodeBranch) ? 5'b0 : d_insn_rd,
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
      execute_state <= divide_state[6];

    end else if (!d_div_insn && d_div_free) begin
      begin
        execute_state <= '{
          insn: decode_state.insn,
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
      divide_state[0] <= '{
        default: 0,
        cycle_status: CYCLE_DIV
      };

    end else begin
      divide_state[0] <= '{
        default: 0,
        cycle_status: CYCLE_DIV
      };
      execute_state <= divide_state[6];
    end
  end
  end

  wire [255:0] x_disasm;

  wire [255:0] div0_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide0 (
    .insn  (divide_state[0].insn),
    .disasm(div0_disasm)
  );
  wire [255:0] div1_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide1 (
    .insn  (divide_state[1].insn),
    .disasm(div1_disasm)
  );
  wire [255:0] div2_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide2 (
    .insn  (divide_state[2].insn),
    .disasm(div2_disasm)
  );
  wire [255:0] div3_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide3 (
    .insn  (divide_state[3].insn),
    .disasm(div3_disasm)
  );
  wire [255:0] div4_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide4 (
    .insn  (divide_state[4].insn),
    .disasm(div4_disasm)
  );
  wire [255:0] div5_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide5 (
    .insn  (divide_state[5].insn),
    .disasm(div5_disasm)
  );
  wire [255:0] div6_disasm;
  Disasm #(
    .PREFIX("V")
  ) disasm_divide6 (
    .insn  (divide_state[6].insn),
    .disasm(div6_disasm)
  );

  Disasm #(
    .PREFIX("X")
  ) disasm_2execute (
    .insn  (execute_state.insn),
    .disasm(x_disasm)
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

  logic x_reg_we;
  logic [`REG_SIZE] x_rd_data;

  logic x_halt;

  logic [`REG_SIZE] x_dmem_addr_pre_mask, x_dmem_data;
  logic [3:0] x_dmem_we;
  logic [63:0] x_multiply_long;

  // div stalls fetch and decode until divide execute 7(8)
  always_comb begin
    illegal_insn = 1'b0;
    x_halt = 1'b0;
    x_rd_data = 32'b0;
    x_reg_we = 1'b0;
    f_pc_next = ((!d_div_insn && d_div_free) || (d_div_insn && d_div_independent)) ? f_pc_current + 4 : f_pc_current;
    f_stall_div = 1'b0;
    f_stall_load2use = 1'b0;
    d_bubble = 1'b0;
    d_stall_load2use = 1'b0;
    d_stall_div = !((!d_div_insn && d_div_free) || (d_div_insn && d_div_independent));
    x_bubble = 1'b0;
    x_bubble_load2use = 1'b0;
    x_multiply_long = bypassed_rs1_data * bypassed_rs2_data;
    x_dmem_we = 4'b0000;
    x_dmem_data = bypassed_rs2_data;
    x_dmem_addr_pre_mask = 32'b0;

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
            x_multiply_long = (bypassed_rs1_data * bypassed_rs2_data);
            x_rd_data = x_multiply_long[31:0]; 
          end
          x_insn_mulh: begin
            x_multiply_long = ($signed(bypassed_rs1_data) * $signed(bypassed_rs2_data));
            x_rd_data = x_multiply_long[63:32];
          end
          x_insn_mulhsu: begin
            x_multiply_long = {{32{bypassed_rs1_data[31]}}, bypassed_rs1_data} * {{32{1'b0}}, bypassed_rs2_data};
            x_rd_data = x_multiply_long[63:32];
          end
          x_insn_mulhu: begin
            x_multiply_long = (bypassed_rs1_data * bypassed_rs2_data);
            x_rd_data = x_multiply_long[63:32];
          end
          x_insn_div: begin
            x_rd_data = (execute_state.insn_rs2_data == 0) ? {32{1'b1}} :
                        (execute_state.insn_rs1_data[31] ^ execute_state.insn_rs2_data[31]) ? 
                        (~div_quotient + 1) : div_quotient;
          end
          x_insn_divu: begin
            x_rd_data = div_quotient;
          end
          x_insn_rem: begin
            x_rd_data = (execute_state.insn_rs1_data[31]) ? (~div_remainder + 1) : div_remainder;
          end
          x_insn_remu: begin 
            x_rd_data = div_remainder;
          end
          default: begin
            x_rd_data = 32'b0;
          end
        endcase
      end
      OpcodeBranch: begin
        d_bubble = 1'b1;
        x_bubble = 1'b1;
        case (1'b1)
          (x_insn_bne && (bypassed_rs1_data != bypassed_rs2_data)): begin
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
          end
          (x_insn_beq && (bypassed_rs1_data == bypassed_rs2_data)): begin
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
          end
          (x_insn_blt && ($signed(bypassed_rs1_data) < $signed(bypassed_rs2_data))): begin
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
          end
          (x_insn_bge && ($signed(bypassed_rs1_data) >= $signed(bypassed_rs2_data))): begin
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
          end
          (x_insn_bltu && (bypassed_rs1_data < bypassed_rs2_data)): begin
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
          end
          (x_insn_bgeu && (bypassed_rs1_data >= bypassed_rs2_data)): begin
            f_pc_next = execute_state.pc + execute_state.insn_imm_b_sext;
          end
          default: begin
            d_bubble = 1'b0;
            x_bubble = 1'b0;
          end
        endcase
      end
      OpcodeLoad: begin
        x_reg_we = 1'b1;
        x_dmem_addr_pre_mask = bypassed_rs1_data + execute_state.insn_imm_i_sext;

        if ((execute_state.insn_rd == d_insn_rs1 && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc) || (d_insn_opcode != OpcodeRegImm && d_insn_opcode != OpcodeLui && d_insn_opcode != OpcodeAuipc && d_insn_opcode != OpcodeJal && d_insn_opcode != OpcodeLoad && d_insn_opcode != OpcodeStore && execute_state.insn_rd == d_insn_rs2)) begin

          f_stall_load2use = 1'b1;
          d_stall_load2use = 1'b1;
          x_bubble_load2use = 1'b1;
        end
      end
      OpcodeStore: begin
        f_pc_next = f_pc_current + 4;
        x_dmem_addr_pre_mask = bypassed_rs1_data + execute_state.insn_imm_s_sext;
        x_dmem_data = bypassed_rs2_data << (8 * x_dmem_addr_pre_mask[1:0]);
        if (x_insn_sb) begin
          x_dmem_we = 4'b0001 << x_dmem_addr_pre_mask[1:0];
        end else if (x_insn_sh) begin
          x_dmem_we = 4'b0011 << x_dmem_addr_pre_mask[1:0];
        end else if (x_insn_sw) begin
          x_dmem_we = 4'b1111;
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
            d_bubble = 1'b1;
            x_bubble = 1'b1;
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
            d_bubble = 1'b1;
            x_bubble = 1'b1;
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

  wire div0_insn_div    = divide_state[0].insn_opcode == OpcodeRegReg && divide_state[0].insn[31:25] == 7'd1 && divide_state[0].insn[14:12] == 3'b100;
  wire div0_insn_divu   = divide_state[0].insn_opcode == OpcodeRegReg && divide_state[0].insn[31:25] == 7'd1 && divide_state[0].insn[14:12] == 3'b101;
  wire div0_insn_rem    = divide_state[0].insn_opcode == OpcodeRegReg && divide_state[0].insn[31:25] == 7'd1 && divide_state[0].insn[14:12] == 3'b110;
  wire div0_insn_remu   = divide_state[0].insn_opcode == OpcodeRegReg && divide_state[0].insn[31:25] == 7'd1 && divide_state[0].insn[14:12] == 3'b111;

  // divider
  logic [`REG_SIZE] div_dividend, div_divisor, div_remainder, div_quotient;
  always_comb begin
    case (1'b1) 
      (div0_insn_div || div0_insn_rem): begin
        div_dividend = divide_state[0].insn_rs1_data[31] ? (~divide_state[0].insn_rs1_data + 1) : divide_state[0].insn_rs1_data; 
        div_divisor  = divide_state[0].insn_rs2_data[31] ? (~divide_state[0].insn_rs2_data + 1) : divide_state[0].insn_rs2_data;
      end
      (div0_insn_divu || div0_insn_remu): begin
        div_dividend = divide_state[0].insn_rs1_data;
        div_divisor = divide_state[0].insn_rs2_data;
      end
      default: begin
        div_dividend = 32'b0;
        div_divisor = 32'b0;
      end
    endcase
  end

  logic div_stall;
  DividerUnsignedPipelined divider(.stall(div_stall), .clk(clk), .rst(rst), .i_dividend(div_dividend), .i_divisor(div_divisor), .o_quotient(div_quotient), .o_remainder(div_remainder));

  /****************/
  /* MEMORY STAGE */
  /****************/
  
  stage_memory_t memory_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      begin
        memory_state <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      end
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

          dmem_addr: x_dmem_addr_pre_mask,
          dmem_data: x_dmem_data,
          dmem_we: x_dmem_we,
   
          halt: x_halt
        };
      end
    end
  end

  wire m_insn_lb  = memory_state.insn_opcode == OpcodeLoad && memory_state.insn[14:12] == 3'b000;
  wire m_insn_lh  = memory_state.insn_opcode == OpcodeLoad && memory_state.insn[14:12] == 3'b001;
  wire m_insn_lw  = memory_state.insn_opcode == OpcodeLoad && memory_state.insn[14:12] == 3'b010;
  wire m_insn_lbu = memory_state.insn_opcode == OpcodeLoad && memory_state.insn[14:12] == 3'b100;
  wire m_insn_lhu = memory_state.insn_opcode == OpcodeLoad && memory_state.insn[14:12] == 3'b101;

  wire m_insn_sb = memory_state.insn_opcode == OpcodeStore && memory_state.insn[14:12] == 3'b000;
  wire m_insn_sh = memory_state.insn_opcode == OpcodeStore && memory_state.insn[14:12] == 3'b001;
  wire m_insn_sw = memory_state.insn_opcode == OpcodeStore && memory_state.insn[14:12] == 3'b010;

  logic [`REG_SIZE] m_dmem_data_pre_mask, m_dmem_data, m_rd_data, m_dmem_write_data;

  assign store_data_to_dmem = m_dmem_write_data;
  assign store_we_to_dmem = memory_state.dmem_we;

  assign addr_to_dmem = memory_state.dmem_addr & ~32'b11;
  assign m_dmem_data_pre_mask = load_data_from_dmem;

  always_comb begin
    m_rd_data = memory_state.insn_rd_data;
    m_dmem_write_data = load_data_from_dmem;

    case (memory_state.insn_opcode)
      OpcodeLoad: begin
        m_dmem_data = m_dmem_data_pre_mask >> (8 * memory_state.dmem_addr[1:0]);
        case (1'b1)
          m_insn_lb: begin
            m_rd_data = {{24{m_dmem_data[7]}}, m_dmem_data[7:0]};
          end
          m_insn_lh: begin
            m_rd_data = {{16{m_dmem_data[15]}}, m_dmem_data[15:0]};
          end
          m_insn_lw: begin
            m_rd_data = m_dmem_data[31:0];
          end
          m_insn_lbu: begin
            m_rd_data = {24'b0, m_dmem_data[7:0]};
          end
          m_insn_lhu: begin
            m_rd_data = {16'b0, m_dmem_data[15:0]};
          end
          default: begin
          end
        endcase
      end
      OpcodeStore: begin
        m_dmem_write_data = wm_bypassed_rs2_data << (8 * memory_state.dmem_addr[1:0]);
      end
      default: begin
      end
    endcase
  end

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
  stage_writeback_t writeback_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      begin
        writeback_state <= '{
          default: 0,
          cycle_status: CYCLE_RESET
        };
      end
    end else begin
      writeback_state <= '{
        pc: memory_state.pc,
        insn: memory_state.insn,
        cycle_status: memory_state.cycle_status,
        we: memory_state.we,
        insn_rs1: memory_state.insn_rs1,
        insn_rs2: memory_state.insn_rs2,
        insn_rd: memory_state.insn_rd,
        insn_rs1_data: memory_state.insn_rs1_data,
        insn_rs2_data: memory_state.insn_rs2_data,
        insn_rd_data: m_rd_data,
        halt: memory_state.halt
      };
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

  wire [`REG_SIZE] bypassed_rs1_data, bypassed_rs2_data, wd_bypassed_rs1_data, wd_bypassed_rs2_data, wm_bypassed_rs1_data, wm_bypassed_rs2_data;

  assign bypassed_rs1_data = (execute_state.insn_rs1 == 0) ? 32'b0 :
                             (execute_state.insn_rs1 == memory_state.insn_rd)? memory_state.insn_rd_data :
                             (execute_state.insn_rs1 == writeback_state.insn_rd) ? writeback_state.insn_rd_data : 
                             execute_state.insn_rs1_data;

  assign bypassed_rs2_data = (execute_state.insn_rs2 == 0) ? 32'b0 :
                             (execute_state.insn_rs2 == memory_state.insn_rd) ? memory_state.insn_rd_data :
                             (execute_state.insn_rs2 == writeback_state.insn_rd) ? writeback_state.insn_rd_data : 
                             execute_state.insn_rs2_data;

  assign wd_bypassed_rs1_data = (d_insn_rs1 == 0) ? 32'b0 :
                                (d_insn_rs1 == writeback_state.insn_rd) ? writeback_state.insn_rd_data :
                                d_rs1_data;
  assign wd_bypassed_rs2_data = (d_insn_rs2 == 0) ? 32'b0 :
                                (d_insn_rs2 == writeback_state.insn_rd) ? writeback_state.insn_rd_data :
                                d_rs2_data;

  assign wm_bypassed_rs2_data = (memory_state.insn_rs2 == 0) ? 32'b0 :
                              (memory_state.insn_rs2 == writeback_state.insn_rd) ? writeback_state.insn_rd_data :
                              memory_state.insn_rs2_data;


  // Set up the RegFile. Read during the execute stage, and write during the writeback stage.
  RegFile rf (.clk(clk),
              .rst(rst),
              .we(writeback_state.we),
              .rd(writeback_state.insn_rd),
              .rd_data(writeback_state.insn_rd_data),
              .rs1(d_insn_rs1),
              .rs1_data(d_rs1_data),
              .rs2(d_insn_rs2),
              .rs2_data(d_rs2_data));

  // Setting the trace values for cycle specific tests
  assign trace_writeback_pc = writeback_state.pc;
  assign trace_writeback_insn = writeback_state.insn;
  assign trace_writeback_cycle_status = writeback_state.cycle_status;

  // Setting the control signals
  assign halt = writeback_state.halt;

endmodule

module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. The memory reads/writes on @(negedge clk)
    input wire clk,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] pc_to_imem,

    // the value at memory location pc_to_imem
    output logic [`REG_SIZE] insn_from_imem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] addr_to_dmem,

    // the value at memory location addr_to_dmem
    output logic [`REG_SIZE] load_data_from_dmem,

    // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    input wire [`REG_SIZE] store_data_to_dmem,

    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input wire [3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  logic [`REG_SIZE] mem_array[NUM_WORDS];

`ifdef SYNTHESIS
  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end
`endif

  always_comb begin
    // memory addresses should always be 4B-aligned
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      if (store_we_to_dmem[0]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
      end
      if (store_we_to_dmem[1]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
      end
      if (store_we_to_dmem[2]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
      end
      if (store_we_to_dmem[3]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
      end
      // dmem is "read-first": read returns value before the write
      load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
    end
  end
endmodule

/* This design has just one clock for both processor and memory. */
module Processor (
    input  wire  clk,
    input  wire  rst,
    output logic halt,
    output wire [`REG_SIZE] trace_writeback_pc,
    output wire [`INSN_SIZE] trace_writeback_insn,
    output cycle_status_e trace_writeback_cycle_status
);

  wire [`INSN_SIZE] insn_from_imem;
  wire [`REG_SIZE] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
      .rst                (rst),
      .clk                (clk),
      // imem is read-only
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
      // dmem is read-write
      .addr_to_dmem       (mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem   (mem_data_we)
  );

  DatapathPipelined datapath (
      .clk(clk),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule

`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`include "../hw2a-divider/divider_unsigned.sv"
`include "../hw2b-cla/cla.sv"

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
  logic [`REG_SIZE] regs[NumRegs];

  // read
  assign rs1_data = rs1 == 0 ? 32'b0 : regs[rs1];
  assign rs2_data = rs2 == 0 ? 32'b0 : regs[rs2];

  // write
  always_ff @(posedge clk) begin
      if (rst) begin
          for (int i = 1; i < NumRegs; i++) begin
              regs[i] <= 32'b0;
          end
      end else if (we && rd != 0) begin
          regs[rd] <= rd_data;
      end
  end

endmodule

module DatapathSingleCycle (
    input wire clk,
    input wire rst,
    output logic halt,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`REG_SIZE] insn_from_imem,
    // addr_to_dmem is a read-write port
    output logic [`REG_SIZE] addr_to_dmem,
    input logic [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem
);

  // components of the instruction
  wire [6:0] insn_funct7;
  wire [4:0] insn_rs2;
  wire [4:0] insn_rs1;
  wire [2:0] insn_funct3;
  wire [4:0] insn_rd;
  wire [`OPCODE_SIZE] insn_opcode;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;

  // U - upper immediate
  wire [31:0] imm_u;
  assign imm_u = {insn_from_imem[31:12], 12'b0};

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] imm_i;
  assign imm_i = insn_from_imem[31:20];
  wire [4:0] imm_shamt = insn_from_imem[24:20];

  // S - stores
  wire [11:0] imm_s;
  assign imm_s[11:5] = insn_funct7, imm_s[4:0] = insn_rd;

  // B - conditionals
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:5]} = insn_funct7, {imm_b[4:1], imm_b[11]} = insn_rd, imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {insn_from_imem[31:12], 1'b0};

  wire [`REG_SIZE] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpLui = 7'b01_101_11;

  wire insn_lui   = insn_opcode == OpLui;
  wire insn_auipc = insn_opcode == OpAuipc;
  wire insn_jal   = insn_opcode == OpJal;
  wire insn_jalr  = insn_opcode == OpJalr;

  wire insn_beq  = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b000;
  wire insn_bne  = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b001;
  wire insn_blt  = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b100;
  wire insn_bge  = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b101;
  wire insn_bltu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b110;
  wire insn_bgeu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b111;

  wire insn_lb  = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b000;
  wire insn_lh  = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b001;
  wire insn_lw  = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b010;
  wire insn_lbu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b100;
  wire insn_lhu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b101;

  wire insn_sb = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b000;
  wire insn_sh = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b001;
  wire insn_sw = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b010;

  wire insn_addi  = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b000;
  wire insn_slti  = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b010;
  wire insn_sltiu = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b011;
  wire insn_xori  = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b100;
  wire insn_ori   = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b110;
  wire insn_andi  = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b111;

  wire insn_slli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire insn_srli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire insn_srai = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;

  wire insn_add  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'd0;
  wire insn_sub  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'b0100000;
  wire insn_sll  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire insn_slt  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b010 && insn_from_imem[31:25] == 7'd0;
  wire insn_sltu = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b011 && insn_from_imem[31:25] == 7'd0;
  wire insn_xor  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b100 && insn_from_imem[31:25] == 7'd0;
  wire insn_srl  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire insn_sra  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;
  wire insn_or   = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b110 && insn_from_imem[31:25] == 7'd0;
  wire insn_and  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b111 && insn_from_imem[31:25] == 7'd0;

  wire insn_mul    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b000;
  wire insn_mulh   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b001;
  wire insn_mulhsu = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b010;
  wire insn_mulhu  = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b011;
  wire insn_div    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b100;
  wire insn_divu   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b101;
  wire insn_rem    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b110;
  wire insn_remu   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b111;

  wire insn_ecall = insn_opcode == OpEnviron && insn_from_imem[31:7] == 25'd0;
  wire insn_fence = insn_opcode == OpMiscMem;

  // this code is only for simulation, not synthesis
  `ifndef SYNTHESIS
  `include "RvDisassembler.sv"
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn_from_imem);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic...
  wire [(8*32)-1:0] disasm_wire;
  genvar i;
  for (i = 0; i < 32; i = i + 1) begin : gen_disasm
    assign disasm_wire[(((i+1))*8)-1:((i)*8)] = disasm_string[31-i];
  end
  `endif

  // branch
  logic take_branch;
  logic [`REG_SIZE] branch_target;

  // handle misaligned memory
  logic[`REG_SIZE] dmem_data, addr_mask, load_data;
  logic[3:0] dmem_we;

  assign addr_to_dmem = addr_mask & ~32'b11;
  assign store_data_to_dmem = dmem_data;
  assign store_we_to_dmem = dmem_we;

  // CLA adder
  logic [`REG_SIZE] cla_a, cla_b, cla_sum;
  cla alu_adder (.a(cla_a), .b(cla_b), .cin(1'b0), .sum(cla_sum));

  // divider
  logic [`REG_SIZE] div_dividend, div_divisor, div_remainder, div_quotient, quotient, remainder;
  divider_unsigned alu_divider (
    .i_dividend(div_dividend), .i_divisor(div_divisor),
    .o_remainder(div_remainder), .o_quotient(div_quotient)
    );
  logic [63:0] alu_product;
  assign quotient = div_quotient;
  assign remainder = div_remainder;

  // program counter
  logic [`REG_SIZE] pcNext, pcCurrent;
  always @(posedge clk) begin
    if (rst) begin
      pcCurrent <= 32'd0;
    end else begin
      pcCurrent <= pcNext;
    end
  end
  assign pc_to_imem = pcCurrent;

  // cycle/insn_from_imem counters
  logic [`REG_SIZE] cycles_current, num_insns_current;
  always @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
      num_insns_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
      if (!rst) begin
        num_insns_current <= num_insns_current + 1;
      end
    end
  end

  // NOTE: don't rename your RegFile instance as the tests expect it to be `rf`
  logic [`REG_SIZE] rs1_data, rs2_data, reg_wd;
  logic reg_we;
  RegFile rf (
    .clk(clk),
    .rst(rst),
    .we(reg_we),
    .rd(insn_rd),
    .rd_data(reg_wd),
    .rs1(insn_rs1),
    .rs2(insn_rs2),
    .rs1_data(rs1_data),
    .rs2_data(rs2_data));

  logic illegal_insn;

  // branch logic
  always_comb begin
    branch_target = pcCurrent + imm_b_sext;
    case (insn_funct3)
      3'b000: take_branch = insn_beq  & (rs1_data == rs2_data);  // beq
      3'b001: take_branch = insn_bne  & (rs1_data != rs2_data);  // bne
      3'b100: take_branch = insn_blt  & ($signed(rs1_data) < $signed(rs2_data));   // blt
      3'b101: take_branch = insn_bge  & ($signed(rs1_data) >= $signed(rs2_data));  // bge
      3'b110: take_branch = insn_bltu & (rs1_data < rs2_data);   // bltu
      3'b111: take_branch = insn_bgeu & (rs1_data >= rs2_data);  // bgeu
      default: take_branch = 1'b0;
    endcase
  end

  wire [63:0] rs1_ishift = {{32{rs1_data[31]}}, rs1_data} >> imm_shamt;
  wire [63:0] rs1_rshift = {{32{rs1_data[31]}}, rs1_data} >> rs2_data[4:0];

  // alu control logic
  always_comb begin
    // defaults
    pcNext = pcCurrent + 4;
    reg_we = 1'b0;
    reg_wd = 32'b0;
    cla_a = 32'b0;
    cla_b = 32'b0;
    div_dividend = 32'b0;
    div_divisor = 32'b0;
    halt = 1'b0;
    addr_mask = 32'b0;
    dmem_data = 32'b0;
    dmem_we = 4'b0000;
    illegal_insn = 1'b0;

    case (insn_opcode)
      OpEnviron: begin
        if (insn_ecall) begin
          halt = 1'b1;
        end else begin
          illegal_insn = 1'b0;
        end
      end

      OpLui: begin
        reg_we = 1'b1;
        reg_wd = imm_u;
      end

      OpAuipc: begin
        reg_we = 1'b1;
        reg_wd = pcCurrent + imm_u;
      end

      OpRegImm: begin
        reg_we = 1'b1;
        case (insn_funct3)
          3'b000: begin // addi
            cla_a = rs1_data;
            cla_b = imm_i_sext;
            reg_wd = cla_sum;
          end
          3'b010: begin // slti
            reg_wd = $signed(rs1_data) < $signed(imm_i_sext) ? 32'b1 : 32'b0;
          end
          3'b011: begin // sltiu
            reg_wd = rs1_data < imm_i_sext ? 32'b1 : 32'b0;
          end
          3'b100: begin // xori
            reg_wd = rs1_data ^ imm_i_sext;
          end
          3'b110: begin // ori
            reg_wd = rs1_data | imm_i_sext;
          end
          3'b111: begin // andi
            reg_wd = rs1_data & imm_i_sext;
          end
          3'b001: begin // slli
            reg_wd = rs1_data << imm_shamt;
          end
          3'b101: begin // srli/srai
            reg_wd = insn_srai ?
              rs1_ishift[31:0] : // srai
              // $signed(rs1_data) >>> imm_shamt :
              rs1_data >> imm_shamt;                                // srli
          end
          default: illegal_insn = 1'b1;
        endcase
      end

      OpRegReg: begin
        reg_we = 1'b1;
        case (insn_funct7)
          7'b0000001: begin
            case (insn_funct3)
              3'b000: begin // mul
                reg_wd = rs1_data * rs2_data;
              end
              3'b001: begin // mulh
                alu_product = $signed(rs1_data) * $signed(rs2_data);
                reg_wd = alu_product[63:32];
              end
              3'b010: begin // mulhsu
                alu_product = {{32{rs1_data[31]}}, rs1_data} * {{32{1'b0}}, rs2_data};
                reg_wd = alu_product[63:32];
              end
              3'b011: begin // mulhu
                alu_product = rs1_data * rs2_data;
                reg_wd = alu_product[63:32];
              end
              3'b100: begin // div
                div_dividend = rs1_data[31] ? (~rs1_data + 1) : rs1_data;
                div_divisor = rs2_data[31] ? (~rs2_data + 1) : rs2_data;
                reg_wd = (rs2_data == 0) ? {32{1'b1}} :
                  ((rs1_data[31] ^ rs2_data[31]) ? (~quotient + 1) : quotient);
              end
              3'b101: begin // divu
                div_dividend = rs1_data;
                div_divisor = rs2_data;
                reg_wd = quotient;
              end
              3'b110: begin // rem
                div_dividend = rs1_data[31] ? (~rs1_data + 1) : rs1_data;
                div_divisor = rs2_data[31] ? (~rs2_data + 1) : rs2_data;
                reg_wd = (rs2_data == 0) ? rs1_data :
                  ((rs1_data[31]) ? (~remainder + 1) : remainder); // sign only depends on rs1
              end
              3'b111: begin // remu
                div_dividend = rs1_data;
                div_divisor = rs2_data;
                reg_wd = remainder;
              end
              default: illegal_insn = 1'b1;
            endcase
          end
          default: begin
            case (insn_funct3)
              3'b000: begin // add/sub
                cla_a = rs1_data;
                cla_b = insn_add ? rs2_data : (~rs2_data) + 1;
                reg_wd = cla_sum;
              end
              3'b001: begin // sll
                reg_wd = rs1_data << rs2_data[4:0];
              end
              3'b010: begin // slt
                reg_wd = $signed(rs1_data) < $signed(rs2_data) ? 32'b1 : 32'b0;
              end
              3'b011: begin // sltu
                reg_wd = rs1_data < rs2_data ? 32'b1 : 32'b0;
              end
              3'b100: begin // xor
                reg_wd = rs1_data ^ rs2_data;
              end
              3'b101: begin // srl/sra
                reg_wd = insn_sra ?
                  rs1_rshift[31:0] : // sra
                  // $signed(rs1_data) >>> rs2_data[4:0] :
                  rs1_data >> rs2_data[4:0]; // srl
              end
              3'b110: begin // or
                reg_wd = rs1_data | rs2_data;
              end
              3'b111: begin // and
                reg_wd = rs1_data & rs2_data;
              end
              default: illegal_insn = 1'b1;
            endcase
          end
        endcase
      end

      OpBranch: begin
        if (take_branch) begin
          pcNext = branch_target;
        end
      end

      default: begin
        illegal_insn = 1'b1;
      end

      OpJal: begin
        reg_we = 1'b1;
        reg_wd = pcCurrent + 4;
        pcNext = pcCurrent + imm_j_sext;
      end

      OpJalr: begin
        reg_we = 1'b1;
        reg_wd = pcCurrent + 4;
        cla_a = rs1_data;
        cla_b = imm_i_sext; // insn[31:20] bruhbruhbruh
        pcNext = {cla_sum[31:1], {1'b0}}; // & ~0x1
      end

      OpLoad: begin
        reg_we = 1'b1;
        cla_a = rs1_data;
        cla_b = imm_i_sext;
        addr_mask = cla_sum;
        load_data = load_data_from_dmem >> (addr_mask[1:0] * 8);

        case (insn_funct3)
          3'b000: begin // lb
            reg_wd = {{24{load_data[7]}}, load_data[7:0]};
          end
          3'b001: begin // lh
            reg_wd = {{16{load_data[15]}}, load_data[15:0]};
          end
          3'b010: begin // lw
            reg_wd = load_data;
          end
          3'b100: begin // lbu
            reg_wd = {24'b0, load_data[7:0]};
          end
          3'b101: begin // lhu
            reg_wd = {16'b0, load_data[15:0]};
          end
          default: illegal_insn = 1'b1;
        endcase
      end

      OpStore: begin
        cla_a = rs1_data;
        cla_b = imm_s_sext;
        addr_mask = cla_sum;
        dmem_data = rs2_data << (addr_mask[1:0] * 8);

        case (insn_funct3)
          3'b000: begin // sb
            dmem_we = (4'b0001 << addr_mask[1:0]);
          end
          3'b001: begin // sh
            dmem_we = (4'b0011 << addr_mask[1:0]);
          end
          3'b010: begin // sw
            dmem_we = 4'b1111;
          end
          default: begin
            dmem_we = 4'b0000;
            illegal_insn = 1'b1;
          end
        endcase
      end
    endcase
  end

endmodule

/* A memory module that supports 1-cycle reads and writes, with one read-only port
 * and one read+write port.
 */
module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. See RiscvProcessor for clock details.
    input wire clock_mem,

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

  always @(posedge clock_mem) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clock_mem) begin
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

/*
This shows the relationship between clock_proc and clock_mem. The clock_mem is
phase-shifted 90° from clock_proc. You could think of one proc cycle being
broken down into 3 parts. During part 1 (which starts @posedge clock_proc)
the current PC is sent to the imem. In part 2 (starting @posedge clock_mem) we
read from imem. In part 3 (starting @negedge clock_mem) we read/write memory and
prepare register/PC updates, which occur at @posedge clock_proc.

        ____
 proc: |    |______
           ____
 mem:  ___|    |___
*/
module Processor (
    input  wire  clock_proc,
    input  wire  clock_mem,
    input  wire  rst,
    output logic halt
);

  wire [`REG_SIZE] pc_to_imem, insn_from_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
      .rst      (rst),
      .clock_mem (clock_mem),
      // imem is read-only
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      // dmem is read-write
      .addr_to_dmem(mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem  (mem_data_we)
  );

  DatapathSingleCycle datapath (
      .clk(clock_proc),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt)
  );

endmodule

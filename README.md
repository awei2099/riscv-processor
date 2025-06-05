# RISC-V Processor Implementations for CIS4710/5710

This repository contains SystemVerilog implementations of various RISC-V datapaths and helper modules for the University of Pennsylvania’s CIS 4710/5710 (Computer Organization & Design) coursework. It demonstrates single-cycle, multi-cycle, and pipelined RV32I processor designs—including a pipelined version with an AXI-Lite cache interface—alongside auxiliary components such as a divider, carry-lookahead adder, and a simple instruction “disassembler.”

## Overview

- **DatapathSingleCycle.sv**  
  A classic single-cycle RV32I CPU: all instruction stages complete in one clock.

- **DatapathMultiCycle.sv**  
  Breaks each instruction into multiple cycles (fetch, decode, execute, memory, write-back) using an FSM to reduce critical path length.

- **DatapathPipelined.sv**  
  Five-stage pipeline. Includes hazard detection, forwarding (bypass), and stall logic for data/control hazards.

- **DatapathPipelinedCache.sv**  
  Builds on the pipelined datapath by adding an AXI-Lite cache (AxilCache.sv). Models cache hits/misses and stalls the pipeline on misses.

- **AxilCache.sv / AxilCache_ref.sv**  
  Simple direct-mapped cache with an AXI-Lite interface. Handles read/write requests from the pipeline and forwards them to a memory model or testbench.

- **DividerUnsignedPipelined.sv / divider_unsigned.sv**  
  Two versions of an unsigned divider: one combinational (divider_unsigned.sv) and one pipelined over several cycles (DividerUnsignedPipelined.sv).

- **cla.sv**  
  A carry-lookahead adder (32-bit) used by the datapath ALU to speed up addition/subtraction.

- **RvDisassembler.sv**  
  Converts a 32-bit RISC-V instruction word into a text string (e.g., `add x1, x2, x3`). Useful for debugging/sim.



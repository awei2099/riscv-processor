`timescale 1ns / 1ns

`define ADDR_WIDTH 32
`define DATA_WIDTH 32

interface axi_if #(
      parameter int ADDR_WIDTH = 32
    , parameter int DATA_WIDTH = 32
);
  logic                      ARREADY;
  logic                      ARVALID;
  logic [    ADDR_WIDTH-1:0] ARADDR;
  logic [               2:0] ARPROT;

  logic                      RREADY;
  logic                      RVALID;
  logic [    DATA_WIDTH-1:0] RDATA;
  logic [               1:0] RRESP;

  logic                      AWREADY;
  logic                      AWVALID;
  logic [    ADDR_WIDTH-1:0] AWADDR;
  logic [               2:0] AWPROT;

  logic                      WREADY;
  logic                      WVALID;
  logic [    DATA_WIDTH-1:0] WDATA;
  logic [(DATA_WIDTH/8)-1:0] WSTRB;

  logic                      BREADY;
  logic                      BVALID;
  logic [               1:0] BRESP;

  modport manager(
      input ARREADY, RVALID, RDATA, RRESP, AWREADY, WREADY, BVALID, BRESP,
      output ARVALID, ARADDR, ARPROT, RREADY, AWVALID, AWADDR, AWPROT, WVALID, WDATA, WSTRB, BREADY
  );
  modport subord(
      input ARVALID, ARADDR, ARPROT, RREADY, AWVALID, AWADDR, AWPROT, WVALID, WDATA, WSTRB, BREADY,
      output ARREADY, RVALID, RDATA, RRESP, AWREADY, WREADY, BVALID, BRESP
  );
endinterface

// [BR]RESP codes, from Section A 3.4.4 of AXI4 spec
`define RESP_OK 2'b00
`define RESP_SUBORDINATE_ERROR 2'b10
`define RESP_DECODE_ERROR 2'b11

/** This is a simple memory that uses the AXI-Lite interface. */
module AxilMemory #(
    parameter int NUM_WORDS = 1024
) (
    input wire ACLK,
    input wire ARESETn,
    axi_if.subord port_ro,
    axi_if.subord port_rw
);
  localparam bit True = 1'b1;
  localparam bit False = 1'b0;
  localparam int AddrLsb = 2;  // since memory elements are 4B
  localparam int AddrMsb = $clog2(NUM_WORDS) + AddrLsb - 1;

  logic [31:0] mem_array[NUM_WORDS];
  logic [31:0] ro_araddr;
  logic ro_araddr_valid;

  initial begin
`ifdef SYNTHESIS
    $readmemh("mem_initial_contents.hex", mem_array);
`endif
  end

  assign port_ro.RRESP = `RESP_OK;
  assign port_ro.BRESP = `RESP_OK;
  assign port_rw.RRESP = `RESP_OK;
  assign port_rw.BRESP = `RESP_OK;

  always_ff @(posedge ACLK) begin
    if (!ARESETn) begin
      ro_araddr <= 0;
      ro_araddr_valid <= False;

      port_ro.ARREADY <= True;
      port_ro.AWREADY <= False;
      port_ro.WREADY <= False;
      port_ro.RVALID <= False;
      port_ro.RDATA <= 0;

      port_rw.ARREADY <= True;
      port_rw.AWREADY <= True;
      port_rw.WREADY <= True;
      port_rw.RVALID <= False;
      port_rw.RDATA <= 0;
    end else begin

      // port_ro is read-only

      if (ro_araddr_valid) begin
        // there is a buffered read request
        if (port_ro.RREADY) begin
          // manager accepted our response, we generate next response
          port_ro.RVALID <= True;
          port_ro.RDATA <= mem_array[ro_araddr[AddrMsb:AddrLsb]];
          ro_araddr <= 0;
          ro_araddr_valid <= False;
          port_ro.ARREADY <= True;
        end
      end else if (port_ro.ARVALID && port_ro.ARREADY) begin
        // we have accepted a read request
        if (port_ro.RVALID && !port_ro.RREADY) begin
          // We have sent a response but manager has not accepted it. Buffer the new read request.
          ro_araddr <= port_ro.ARADDR;
          ro_araddr_valid <= True;
          port_ro.ARREADY <= False;
        end else begin
          // We have sent a response and manager has accepted it. Or, we were not already sending a response.
          // Either way, send a response to the request we just accepted.
          port_ro.RVALID <= True;
          port_ro.RDATA  <= mem_array[port_ro.ARADDR[AddrMsb:AddrLsb]];
        end
      end else if (port_ro.RVALID && port_ro.RREADY) begin
        // No incoming request. We have sent a response and manager has accepted it
        port_ro.RVALID  <= False;
        port_ro.RDATA   <= 0;
        port_ro.ARREADY <= True;
      end

      // port_rw is read-write

      // NB: we take a shortcut on port_rw because the manager will always be RREADY/BREADY
      // as 1) the datapath never stalls in the W stage and 2) the cache is always ready
      if (port_rw.ARVALID && port_rw.ARREADY) begin
        port_rw.RVALID <= True;
        port_rw.RDATA  <= mem_array[port_rw.ARADDR[AddrMsb:AddrLsb]];
      end else if (port_rw.RVALID) begin
        port_rw.RVALID <= False;
        port_rw.RDATA  <= 0;
      end

      if (port_rw.AWVALID && port_rw.AWREADY && port_rw.WVALID && port_rw.WREADY) begin
        if (port_rw.WSTRB[0]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][7:0] <= port_rw.WDATA[7:0];
        end
        if (port_rw.WSTRB[1]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][15:8] <= port_rw.WDATA[15:8];
        end
        if (port_rw.WSTRB[2]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][23:16] <= port_rw.WDATA[23:16];
        end
        if (port_rw.WSTRB[3]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][31:24] <= port_rw.WDATA[31:24];
        end
        port_rw.BVALID <= True;
      end else if (port_rw.BVALID) begin
        port_rw.BVALID <= False;
      end
    end
  end

endmodule

// States for cache state machine
typedef enum logic [2:0] {
  // cache can respond to an incoming request
  CACHE_AVAILABLE = 3'd0,
  // cache miss, waiting for fill from memory
  CACHE_AWAIT_FILL_RESPONSE = 3'd1,
  // cache miss, waiting for writeback to memory
  CACHE_AWAIT_WRITEBACK_RESPONSE = 3'd2,
  // cache waiting for manager to accept response
  CACHE_AWAIT_MANAGER_READY = 3'd3,
  // cache miss with write, waiting for fill from memory
  CACHE_AWAIT_WRITE_FILL_RESPONSE = 3'd4
} cache_state_t;

// -----------------------------------------------------------------------------
//  AxilCache – Simple Direct‑Mapped AXI‑Lite Blocking Cache (Write‑back)
// -----------------------------------------------------------------------------
//  High‑level overview
//  ------------------
//  •  Direct‑mapped (1 way) cache with configurable number of sets (NUM_SETS)
//  •  Block/line size is fixed to one 32‑bit word (BLOCK_SIZE_BITS = 32)
//     so the byte offset is always 2 bits (word aligned).
//  •  Implements a **write‑back / write‑allocate** policy.
//  •  Serves a single AXI‑Lite **processor‑side** subordinate port (proc)
//     and a single AXI‑Lite **memory‑side** manager port (mem).
//  •  Finite‑state machine (cache_state_t) sequences the miss / writeback
//     protocol.  The states are:
//        – CACHE_AVAILABLE               : cache idle / hit handling
//        – CACHE_AWAIT_FILL_RESPONSE     : waiting for memory read data
//        – CACHE_AWAIT_WRITEBACK_RESPONSE: waiting for memory write response
//        – CACHE_AWAIT_WRITE_FILL_RESPONSE : write miss – wait for fill
//        – CACHE_AWAIT_MANAGER_READY     : not used in this stripped example

module AxilCache #(
    /** size of each cache block, in bits */
    parameter int BLOCK_SIZE_BITS = 32,
    /** number of blocks in each way of the cache */
    parameter int NUM_SETS = 4
) (
    input wire ACLK,
    input wire ARESETn,
    axi_if.subord proc,
    axi_if.manager mem
);

  // Calculate cache addressing bits
  localparam int BlockOffsetBits = 2;  // since memory elements are 4B
  localparam int IndexBits = $clog2(NUM_SETS);
  localparam int TagBits = 32 - IndexBits - BlockOffsetBits;

  // Cache state
  cache_state_t current_state;


  // Main cache structures
  logic [BLOCK_SIZE_BITS-1:0] data[NUM_SETS];
  logic [TagBits-1:0] tag[NUM_SETS];
  logic [0:0] valid[NUM_SETS];
  logic [0:0] dirty[NUM_SETS];

  // Initialize cache state to all zeroes
  genvar seti;
  for (seti = 0; seti < NUM_SETS; seti = seti + 1) begin : gen_cache_init
    initial begin
      valid[seti] = '0;
      dirty[seti] = '0;
      data[seti]  = 0;
      tag[seti]   = 0;
    end
  end

  assign proc.RRESP = `RESP_OK;
  assign proc.BRESP = `RESP_OK;

  // Address parsing for read and write operations
  logic [  TagBits-1:0] read_tag;
  logic [IndexBits-1:0] read_index;
  logic [  TagBits-1:0] write_tag;
  logic [IndexBits-1:0] write_index;

  assign read_tag = proc.ARADDR[31 : BlockOffsetBits+IndexBits];
  assign read_index = proc.ARADDR[BlockOffsetBits+IndexBits-1 : BlockOffsetBits];
  assign write_tag = proc.AWADDR[31 : BlockOffsetBits+IndexBits];
  assign write_index = proc.AWADDR[BlockOffsetBits+IndexBits-1 : BlockOffsetBits];

  // Control signals
  logic read_valid;
  logic write_valid;
  logic read_miss_buffer_flag;
  logic read_miss_buffer_empty_flag;
  logic write_writeback_flag;
  logic [31:0] write_data;
  cache_state_t next_state;

  // Memory side signals
  assign mem.WSTRB = 4'hF;  // Always write full words to memory

  // Cache hit data and control
  logic [31:0] rdata_hit;
  logic rvalid_hit;

  // Buffers for handling various operations
  logic [31:0] read_buffer;
  logic read_buffer_full;
  logic [31:0] read_miss_buffer_addr;
  logic read_miss_buffer_full;
  logic [31:0] mem_address;

  // Write buffers
  logic [3:0] wstrb_buffer;
  logic [31:0] wdata_buffer;
  logic [31:0] waddr_buffer;
  logic w_buffer_full;
  logic [31:0] wstrb_buffer_filter;

  // Ready/Valid signaling
  assign proc.ARREADY = (!ARESETn) ? 0 : 
  (current_state == CACHE_AVAILABLE && rvalid_hit == 1 && proc.RREADY == 0 && read_buffer_full) ? 0 : 1;
  assign proc.AWREADY = (!ARESETn) ? 0 : (current_state == CACHE_AVAILABLE) ? 1 : 0;
  assign proc.WREADY = (!ARESETn) ? 0 : (current_state == CACHE_AVAILABLE) ? 1 : 0;
  assign mem.BREADY = 1;
  assign mem.RREADY = 1;

  // Claude AI + Deepseek used here
  // Combinational logic for cache operations
  always_comb begin
    // Addresses should always be 4B-aligned
    assert (!proc.ARVALID || proc.ARADDR[1:0] == 2'b00);
    assert (proc.ARPROT == 3'd0);
    assert (!proc.AWVALID || proc.AWADDR[1:0] == 2'b00);
    assert (proc.AWPROT == 3'd0);
    // Cache is single-ported
    assert (!(proc.ARVALID && (proc.AWVALID || proc.WVALID)));

    // Default values
    next_state = CACHE_AVAILABLE;
    read_valid = 1'b0;
    write_valid = 1'b0;
    write_data = 32'b0;
    mem.ARVALID = 0;
    mem.ARADDR = 32'b0;
    mem.AWVALID = 0;
    mem.AWADDR = 0;
    mem.WVALID = 0;
    mem.WDATA = 0;
    proc.RDATA = rdata_hit;
    proc.RVALID = rvalid_hit;
    wstrb_buffer_filter = 0;
    read_miss_buffer_flag = 0;
    read_miss_buffer_empty_flag = 0;

    // Handle read requests and cache misses
    if (current_state == CACHE_AVAILABLE || (current_state == CACHE_AWAIT_WRITE_FILL_RESPONSE && proc.ARVALID)) begin
      if (proc.ARREADY && proc.ARVALID) begin
        if (read_tag == tag[read_index] && valid[read_index] == 1) begin
          // Cache hit
          read_valid = 1'b1;
        end else begin
          // Cache miss
          if (dirty[read_index]) begin
            // Need to writeback before fill
            next_state  = CACHE_AWAIT_WRITEBACK_RESPONSE;
            mem.AWVALID = 1;
            mem.AWADDR  = {tag[read_index], read_index, 2'b0};
            mem.WVALID  = 1;
            mem.WDATA   = data[read_index];
          end else begin
            if (proc.RREADY) begin
              // Can proceed with fill
              next_state  = CACHE_AWAIT_FILL_RESPONSE;
              mem.ARVALID = 1;
              mem.ARADDR  = proc.ARADDR;
            end else begin
              // Buffer the miss request
              read_miss_buffer_flag = 1;
            end
          end
        end
      end
    end  // Claude AI + Deepseek used here
         // Handle fill response state
    else if (current_state == CACHE_AWAIT_FILL_RESPONSE) begin
      if (!mem.RVALID) begin
        next_state = CACHE_AWAIT_FILL_RESPONSE;
      end
      if (proc.RREADY && proc.ARVALID) begin
        mem.ARVALID = 1;
        mem.ARADDR  = proc.ARADDR;
      end
    end  // Handle write fill response state
    else if (current_state == CACHE_AWAIT_WRITE_FILL_RESPONSE) begin
      if (!mem.RVALID) begin
        next_state = CACHE_AWAIT_WRITE_FILL_RESPONSE;
      end
      if (proc.AWVALID) begin
        mem.ARVALID = 1;
        mem.ARADDR  = proc.AWADDR;
      end
    end  // Handle writeback response state
    else if (current_state == CACHE_AWAIT_WRITEBACK_RESPONSE) begin
      if (write_writeback_flag) begin
        if (!mem.BVALID) begin
          next_state  = CACHE_AWAIT_WRITEBACK_RESPONSE;
          mem.AWVALID = 1;
          mem.AWADDR  = {tag[write_index], write_index, 2'b0};
          mem.WVALID  = 1;
          mem.WDATA   = data[write_index];
        end else begin
          next_state  = CACHE_AWAIT_WRITE_FILL_RESPONSE;
          mem.ARVALID = 1;
          mem.ARADDR  = proc.AWADDR;
        end
      end else begin
        if (!mem.BVALID) begin
          next_state  = CACHE_AWAIT_WRITEBACK_RESPONSE;
          mem.AWVALID = 1;
          mem.AWADDR  = {tag[read_index], read_index, 2'b0};
          mem.WVALID  = 1;
          mem.WDATA   = data[read_index];
        end else begin
          next_state  = CACHE_AWAIT_FILL_RESPONSE;
          mem.ARVALID = 1;
          mem.ARADDR  = proc.ARADDR;
        end
      end
    end
    // Claude AI + Deepseek used here
    // Handle write requests
    if ((proc.AWREADY && proc.AWVALID) && (proc.WVALID && proc.WREADY)) begin
      if (write_tag == tag[write_index] && valid[write_index] == 1) begin
        // Write hit - apply write strobe to data
        if (proc.WSTRB[3] == 1'b1) begin
          write_data = write_data | (proc.WDATA & 32'hFF000000);
        end
        if (proc.WSTRB[2] == 1'b1) begin
          write_data = write_data | (proc.WDATA & 32'h00FF0000);
        end
        if (proc.WSTRB[1] == 1'b1) begin
          write_data = write_data | (proc.WDATA & 32'h0000FF00);
        end
        if (proc.WSTRB[0] == 1'b1) begin
          write_data = write_data | (proc.WDATA & 32'h000000FF);
        end
        write_valid = 1'b1;
      end else begin
        // Write miss
        if (dirty[write_index]) begin
          // Need to writeback before fill
          next_state  = CACHE_AWAIT_WRITEBACK_RESPONSE;
          mem.AWVALID = 1;
          mem.AWADDR  = {tag[write_index], read_index, 2'b0};
          mem.WVALID  = 1;
          mem.WDATA   = data[write_index];
        end else begin
          // Can proceed with fill
          next_state  = CACHE_AWAIT_WRITE_FILL_RESPONSE;
          mem.ARVALID = 1;
          mem.ARADDR  = proc.AWADDR;
        end
      end
    end  // Process buffered write if available
    else if (w_buffer_full && next_state == CACHE_AVAILABLE) begin
      // Apply write strobe to data from buffer
      if (wstrb_buffer[3] == 1'b1) begin
        write_data = write_data | (wdata_buffer & 32'hFF000000);
        wstrb_buffer_filter = wstrb_buffer_filter | 32'hFF000000;
      end
      if (wstrb_buffer[2] == 1'b1) begin
        write_data = write_data | (wdata_buffer & 32'h00FF0000);
        wstrb_buffer_filter = wstrb_buffer_filter | 32'h00FF0000;
      end
      if (wstrb_buffer[1] == 1'b1) begin
        write_data = write_data | (wdata_buffer & 32'h0000FF00);
        wstrb_buffer_filter = wstrb_buffer_filter | 32'h0000FF00;
      end
      if (wstrb_buffer[0] == 1'b1) begin
        write_data = write_data | (wdata_buffer & 32'h000000FF);
        wstrb_buffer_filter = wstrb_buffer_filter | 32'h000000FF;
      end
      write_valid = 1'b1;
    end

    // Handle read data return paths
    if (proc.RREADY && current_state == CACHE_AVAILABLE) begin
      if (w_buffer_full) begin
        // Return merged data from buffer and memory
        proc.RDATA  = (mem.RDATA & (wstrb_buffer_filter ^ 32'hFFFFFFFF)) | write_data;
        proc.RVALID = 1;
      end
    end else if (mem.RVALID && !write_valid) begin
      // Return data from memory
      proc.RDATA  = mem.RDATA;
      proc.RVALID = 1;
    end

    // Handle memory read response for various states
    if (mem.RVALID && proc.RREADY && !proc.AWVALID && !w_buffer_full && 
        (current_state == CACHE_AWAIT_WRITEBACK_RESPONSE || proc.BVALID == 0)) begin
      proc.RDATA  = mem.RDATA;
      proc.RVALID = 1;
    end

    // Process buffered read miss if processor is ready
    if (proc.RREADY && read_miss_buffer_full) begin
      next_state = CACHE_AWAIT_FILL_RESPONSE;
      mem.ARVALID = 1;
      mem.ARADDR = read_miss_buffer_addr;
      read_miss_buffer_empty_flag = 1;
    end
  end

  // Sequential logic for state updates
  always_ff @(posedge ACLK) begin
    if (!ARESETn) begin  // Reset when ARESETn == 0
      rvalid_hit <= 1'b0;
      rdata_hit <= 32'b0;
      proc.BVALID <= 1'b0;
      current_state <= CACHE_AVAILABLE;
      read_buffer <= 0;
      read_buffer_full <= 0;
      read_miss_buffer_addr <= 0;
      read_miss_buffer_full <= 0;
      mem_address <= 0;
      wstrb_buffer <= 0;
      wdata_buffer <= 0;
      waddr_buffer <= 0;
      w_buffer_full <= 0;
      write_writeback_flag <= 0;
    end else begin
      current_state <= next_state;
      // Claude AI + Deepseek used here
      // Update read hit data and valid signals
      rvalid_hit <= (proc.RVALID && !proc.RREADY) ? proc.RVALID : 
                    (read_valid) ? 1'b1 : 
                    (read_buffer_full) ? 1'b1 : 1'b0;

      rdata_hit <= (proc.RVALID && !proc.RREADY) ? proc.RDATA : 
                   (read_valid) ? data[read_index] : 
                   (read_buffer_full) ? read_buffer : 32'b0;

      if (next_state == CACHE_AVAILABLE) begin
        // Buffer read data if needed
        if (read_valid && proc.ARVALID == 1 && rvalid_hit == 1 && proc.RREADY == 0) begin
          read_buffer <= data[read_index];
          read_buffer_full <= 1;
        end
        if (!read_valid && !(proc.RVALID && !proc.RREADY) && read_buffer_full) begin
          read_buffer_full <= 0;
        end

        // Handle write response
        proc.BVALID <= (write_valid) ? 1'b1 : 1'b0;
        if (write_valid && !w_buffer_full) begin
          data[write_index]  <= write_data;
          dirty[write_index] <= 1'b1;
        end

        // Handle cache fill from memory
        if (mem.RVALID) begin
          if (current_state == CACHE_AWAIT_FILL_RESPONSE) begin
            // Normal read cache fill
            data[mem_address[BlockOffsetBits+IndexBits-1 : BlockOffsetBits]] <= mem.RDATA;
            tag[mem_address[BlockOffsetBits + IndexBits - 1 : BlockOffsetBits]] <= mem.ARADDR[31 : BlockOffsetBits + IndexBits];
            valid[mem_address[BlockOffsetBits+IndexBits-1 : BlockOffsetBits]] <= 1;
            dirty[mem_address[BlockOffsetBits+IndexBits-1 : BlockOffsetBits]] <= 0;
          end else if (current_state == CACHE_AWAIT_WRITE_FILL_RESPONSE) begin
            // Write cache fill
            if (write_valid && w_buffer_full) begin
              // Merge memory data with pending write data
              data[waddr_buffer[BlockOffsetBits + IndexBits - 1 : BlockOffsetBits]] <= 
                (mem.RDATA & (wstrb_buffer_filter ^ 32'hFFFFFFFF)) | write_data;
              dirty[waddr_buffer[BlockOffsetBits+IndexBits-1 : BlockOffsetBits]] <= 1'b1;
            end else begin
              data[waddr_buffer[BlockOffsetBits+IndexBits-1 : BlockOffsetBits]]  <= mem.RDATA;
              dirty[waddr_buffer[BlockOffsetBits+IndexBits-1 : BlockOffsetBits]] <= 0'b1;
            end
            tag[waddr_buffer[BlockOffsetBits + IndexBits - 1 : BlockOffsetBits]] <= waddr_buffer[31 : BlockOffsetBits + IndexBits];
            valid[waddr_buffer[BlockOffsetBits+IndexBits-1 : BlockOffsetBits]] <= 1;
          end
        end

        // Handle read miss buffer
        if (read_miss_buffer_flag) begin
          read_miss_buffer_addr <= proc.ARADDR;
          read_miss_buffer_full <= 1;
        end
        if (read_miss_buffer_empty_flag) begin
          read_miss_buffer_full <= 0;
        end
      end

      // Track memory address for outstanding requests
      if (mem.ARVALID == 1) begin
        mem_address <= mem.ARADDR;
      end

      // Invalidate cache line for misses
      if ((!read_valid && proc.ARVALID) || (!write_valid && proc.AWVALID)) begin
        valid[read_index] <= 0;
      end

      // Buffer write data for cache misses
      if (((proc.AWREADY && proc.AWVALID) && (proc.WVALID && proc.WREADY)) && !write_valid) begin
        wstrb_buffer  <= proc.WSTRB;
        wdata_buffer  <= proc.WDATA;
        waddr_buffer  <= proc.AWADDR;
        w_buffer_full <= 1;
      end

      // Track writeback type (read vs write miss)
      if ((proc.AWREADY && proc.AWVALID) && (proc.WVALID && proc.WREADY)) begin
        write_writeback_flag <= 1;
      end else if (proc.ARREADY && proc.ARVALID) begin
        write_writeback_flag <= 0;
      end

      // Clear write buffer after processing
      if (w_buffer_full && write_valid) begin
        w_buffer_full <= 0;
      end

      // Update dirty bit after writeback
      if (mem.BVALID) begin
        dirty[mem.AWADDR[BlockOffsetBits+IndexBits-1 : BlockOffsetBits]] <= 0'b1;
      end
    end
  end

endmodule



`ifndef SYNTHESIS
/** This is used for testing AxilCache in simulation. Since Verilator doesn't allow
SV interfaces in a top-level module, we wrap the interfaces with plain wires. */
module AxilCacheTester #(
    // these parameters are for the AXIL interface
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    // these parameters are for the cache
    parameter int BLOCK_SIZE_BITS = 32,
    parameter int NUM_SETS = 4
) (
    input wire ACLK,
    input wire ARESETn,

    input  wire                       CACHE_ARVALID,
    output logic                      CACHE_ARREADY,
    input  wire  [    ADDR_WIDTH-1:0] CACHE_ARADDR,
    input  wire  [               2:0] CACHE_ARPROT,
    output logic                      CACHE_RVALID,
    input  wire                       CACHE_RREADY,
    output logic [    ADDR_WIDTH-1:0] CACHE_RDATA,
    output logic [               1:0] CACHE_RRESP,
    input  wire                       CACHE_AWVALID,
    output logic                      CACHE_AWREADY,
    input  wire  [    ADDR_WIDTH-1:0] CACHE_AWADDR,
    input  wire  [               2:0] CACHE_AWPROT,
    input  wire                       CACHE_WVALID,
    output logic                      CACHE_WREADY,
    input  wire  [    DATA_WIDTH-1:0] CACHE_WDATA,
    input  wire  [(DATA_WIDTH/8)-1:0] CACHE_WSTRB,
    output logic                      CACHE_BVALID,
    input  wire                       CACHE_BREADY,
    output logic [               1:0] CACHE_BRESP,

    output wire                       MEM_ARVALID,
    input  logic                      MEM_ARREADY,
    output wire  [    ADDR_WIDTH-1:0] MEM_ARADDR,
    output wire  [               2:0] MEM_ARPROT,
    input  logic                      MEM_RVALID,
    output wire                       MEM_RREADY,
    input  logic [    ADDR_WIDTH-1:0] MEM_RDATA,
    input  logic [               1:0] MEM_RRESP,
    output wire                       MEM_AWVALID,
    input  logic                      MEM_AWREADY,
    output wire  [    ADDR_WIDTH-1:0] MEM_AWADDR,
    output wire  [               2:0] MEM_AWPROT,
    output wire                       MEM_WVALID,
    input  logic                      MEM_WREADY,
    output wire  [    DATA_WIDTH-1:0] MEM_WDATA,
    output wire  [(DATA_WIDTH/8)-1:0] MEM_WSTRB,
    input  logic                      MEM_BVALID,
    output wire                       MEM_BREADY,
    input  logic [               1:0] MEM_BRESP
);

  axi_if #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) cache_axi ();
  assign cache_axi.manager.ARVALID = CACHE_ARVALID;
  assign CACHE_ARREADY = cache_axi.manager.ARREADY;
  assign cache_axi.manager.ARADDR = CACHE_ARADDR;
  assign cache_axi.manager.ARPROT = CACHE_ARPROT;
  assign CACHE_RVALID = cache_axi.manager.RVALID;
  assign cache_axi.manager.RREADY = CACHE_RREADY;
  assign CACHE_RRESP = cache_axi.manager.RRESP;
  assign CACHE_RDATA = cache_axi.manager.RDATA;
  assign cache_axi.manager.AWVALID = CACHE_AWVALID;
  assign CACHE_AWREADY = cache_axi.manager.AWREADY;
  assign cache_axi.manager.AWADDR = CACHE_AWADDR;
  assign cache_axi.manager.AWPROT = CACHE_AWPROT;
  assign cache_axi.manager.WVALID = CACHE_WVALID;
  assign CACHE_WREADY = cache_axi.manager.WREADY;
  assign cache_axi.manager.WDATA = CACHE_WDATA;
  assign cache_axi.manager.WSTRB = CACHE_WSTRB;
  assign CACHE_BVALID = cache_axi.manager.BVALID;
  assign cache_axi.manager.BREADY = CACHE_BREADY;
  assign CACHE_BRESP = cache_axi.manager.BRESP;

  axi_if #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) mem_axi ();
  assign MEM_ARVALID = mem_axi.subord.ARVALID;
  assign mem_axi.subord.ARREADY = MEM_ARREADY;
  assign MEM_ARADDR = mem_axi.subord.ARADDR;
  assign MEM_ARPROT = mem_axi.subord.ARPROT;
  assign mem_axi.subord.RVALID = MEM_RVALID;
  assign MEM_RREADY = mem_axi.subord.RREADY;
  assign mem_axi.subord.RRESP = MEM_RRESP;
  assign mem_axi.subord.RDATA = MEM_RDATA;
  assign MEM_AWVALID = mem_axi.subord.AWVALID;
  assign mem_axi.subord.AWREADY = MEM_AWREADY;
  assign MEM_AWADDR = mem_axi.subord.AWADDR;
  assign MEM_AWPROT = mem_axi.subord.AWPROT;
  assign MEM_WVALID = mem_axi.subord.WVALID;
  assign mem_axi.subord.WREADY = MEM_WREADY;
  assign MEM_WDATA = mem_axi.subord.WDATA;
  assign MEM_WSTRB = mem_axi.subord.WSTRB;
  assign mem_axi.subord.BVALID = MEM_BVALID;
  assign MEM_BREADY = mem_axi.subord.BREADY;
  assign mem_axi.subord.BRESP = MEM_BRESP;

  AxilCache #(
      .BLOCK_SIZE_BITS(BLOCK_SIZE_BITS),
      .NUM_SETS(NUM_SETS)
  ) cache (
      .ACLK(ACLK),
      .ARESETn(ARESETn),
      .proc(cache_axi.subord),
      .mem(mem_axi.manager)
  );
endmodule  // AxilCacheTester
`endif

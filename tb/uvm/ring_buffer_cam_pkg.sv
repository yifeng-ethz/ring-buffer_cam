// ring_buffer_cam_pkg.sv — UVM package for ring_buffer_cam testbench
// Contains: config, sequence items, all component class forwards
`ifndef RING_BUFFER_CAM_PKG_SV
`define RING_BUFFER_CAM_PKG_SV

package ring_buffer_cam_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"

  // ── DUT parameters (must match top-level generics) ────────────
  parameter int SEARCH_KEY_WIDTH    = 8;
  parameter int RING_BUFFER_N_ENTRY = 512;
  parameter int SIDE_DATA_BITS      = 31;
  parameter int INTERLEAVING_FACTOR = 4;
  parameter int INTERLEAVING_INDEX  = 0;
  parameter int N_PARTITIONS        = 2;   // v3.0
  parameter int ENCODER_LEAF_WIDTH  = 16;  // v3.0
  parameter int ENCODER_PIPE_STAGES = 4;   // delivered default

  // ── Hit type-1 bit-field positions ────────────────────────────
  parameter int ASIC_HI    = 38, ASIC_LO    = 35;
  parameter int CHANNEL_HI = 34, CHANNEL_LO = 30;
  parameter int TCC8N_HI   = 29, TCC8N_LO   = 17;
  parameter int TCC1N6_HI  = 16, TCC1N6_LO  = 14;
  parameter int TFINE_HI   = 13, TFINE_LO   = 9;
  parameter int ET1N6_HI   = 8,  ET1N6_LO   = 0;

  // ── Run-control command encodings ─────────────────────────────
  parameter logic [8:0] CTRL_IDLE          = 9'b0_0000_0001;
  parameter logic [8:0] CTRL_RUN_PREPARE   = 9'b0_0000_0010;
  parameter logic [8:0] CTRL_SYNC          = 9'b0_0000_0100;
  parameter logic [8:0] CTRL_RUNNING       = 9'b0_0000_1000;
  parameter logic [8:0] CTRL_TERMINATING   = 9'b0_0001_0000;

  // ── 8b10b constants ───────────────────────────────────────────
  parameter logic [7:0] K237 = 8'hF7;

  // ═══════════════════════════════════════════════════════════════
  // Configuration object
  // ═══════════════════════════════════════════════════════════════
  class ring_buffer_cam_cfg extends uvm_object;
    `uvm_object_utils(ring_buffer_cam_cfg)

    int unsigned expected_latency = 128;
    bit          filter_inerr     = 1;
    int unsigned n_partitions     = N_PARTITIONS;
    int unsigned encoder_leaf_width = ENCODER_LEAF_WIDTH;
    int unsigned encoder_pipe_stages = ENCODER_PIPE_STAGES;

    function new(string name = "ring_buffer_cam_cfg");
      super.new(name);
    endfunction
  endclass

  // ═══════════════════════════════════════════════════════════════
  // Sequence items
  // ═══════════════════════════════════════════════════════════════

  // ── ingress hit ───────────────────────────────────────────────
  class hit_seq_item extends uvm_sequence_item;
    `uvm_object_utils(hit_seq_item)

    rand bit [3:0]  asic;
    rand bit [4:0]  channel;
    rand bit [12:0] tcc8n;
    rand bit [2:0]  tcc1n6;
    rand bit [4:0]  tfine;
    rand bit [8:0]  et1n6;
    rand bit        has_error;

    // Convenience: search key is tcc8n[11:4] = tcc8n[12:4] >> 0
    // Actually search key = tcc8n(SK_RANGE_HI downto SK_RANGE_LO)
    // which is tcc8n[7:0] in the 8-bit search key config.
    function bit [7:0] search_key();
      return tcc8n[7:0];
    endfunction

    function logic [38:0] pack_hit();
      logic [38:0] d;
      d[38:35] = asic;
      d[34:30] = channel;
      d[29:17] = tcc8n;
      d[16:14] = tcc1n6;
      d[13:9]  = tfine;
      d[8:0]   = et1n6;
      return d;
    endfunction

    // Constraint: interleaving filter acceptance
    constraint c_interleaving {
      // ts8n mod INTERLEAVING_FACTOR == INTERLEAVING_INDEX
      // ts8n is tcc8n.  The interleaving check uses tcc8n[INTERLEAVING_BITS+3:4].
      // For FACTOR=4, INDEX=0: tcc8n[5:4] == 0 → tcc8n[5:4] == 0
      (tcc8n % INTERLEAVING_FACTOR) == INTERLEAVING_INDEX;
    }

    function new(string name = "hit_seq_item");
      super.new(name);
    endfunction
  endclass

  // ── egress hit (monitored) ────────────────────────────────────
  class out_seq_item extends uvm_sequence_item;
    `uvm_object_utils(out_seq_item)

    bit        is_subheader;
    bit [7:0]  search_key;     // subheader[31:24]
    bit [7:0]  hit_count;      // subheader[15:8]
    bit [35:0] raw_data;
    bit        sop;
    bit        eop;
    bit        error;

    // Extracted hit fields (valid when !is_subheader)
    bit [3:0]  ts_3_0;         // data[31:28]
    bit [3:0]  asic;           // data[25:22]
    bit [4:0]  channel;        // data[21:17]
    bit [7:0]  ts50p;          // data[16:9] = tcc1n6 & tfine
    bit [8:0]  et1n6;          // data[8:0]

    function new(string name = "out_seq_item");
      super.new(name);
    endfunction
  endclass

  // ── run control command ───────────────────────────────────────
  class ctrl_seq_item extends uvm_sequence_item;
    `uvm_object_utils(ctrl_seq_item)

    logic [8:0] cmd;

    function new(string name = "ctrl_seq_item");
      super.new(name);
    endfunction
  endclass

  // ── CSR access ────────────────────────────────────────────────
  class csr_seq_item extends uvm_sequence_item;
    `uvm_object_utils(csr_seq_item)

    bit          is_write;
    bit [4:0]    address;
    bit [31:0]   writedata;
    logic [31:0] readdata;  // filled by driver on read

    function new(string name = "csr_seq_item");
      super.new(name);
    endfunction
  endclass

endpackage

`endif

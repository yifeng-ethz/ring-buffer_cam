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

  typedef enum int unsigned {
    RUN_STATE_IDLE = 0,
    RUN_STATE_RUN_PREPARE,
    RUN_STATE_SYNC,
    RUN_STATE_RUNNING,
    RUN_STATE_TERMINATING,
    RUN_STATE_LINK_TEST,
    RUN_STATE_SYNC_TEST,
    RUN_STATE_RESET,
    RUN_STATE_OUT_OF_DAQ,
    RUN_STATE_ERROR
  } run_state_e;

  // ═══════════════════════════════════════════════════════════════
  // Configuration object
  // ═══════════════════════════════════════════════════════════════
  class ring_buffer_cam_cfg extends uvm_object;
    `uvm_object_utils(ring_buffer_cam_cfg)

    int unsigned expected_latency = 128;
    bit          filter_inerr     = 1;
    bit          go               = 1;
    bit          endofrun_seen    = 0;
    run_state_e  run_state        = RUN_STATE_IDLE;
    int unsigned interleaving_factor = INTERLEAVING_FACTOR;
    int unsigned interleaving_index  = INTERLEAVING_INDEX;
    int unsigned ring_buffer_n_entry = RING_BUFFER_N_ENTRY;
    int unsigned n_partitions     = N_PARTITIONS;
    int unsigned encoder_leaf_width = ENCODER_LEAF_WIDTH;
    int unsigned encoder_pipe_stages = ENCODER_PIPE_STAGES;
    int unsigned debug_level      = 1;

    function new(string name = "ring_buffer_cam_cfg");
      super.new(name);
    endfunction

    function void note_reset_defaults();
      expected_latency = 2000;
      filter_inerr     = 1;
      go               = 1;
      endofrun_seen    = 0;
      run_state        = RUN_STATE_IDLE;
    endfunction

    function int unsigned lane_key_ord_to_search_key(int unsigned key_ord);
      return (key_ord * interleaving_factor) + interleaving_index;
    endfunction

    function logic [12:0] make_tcc8n_for_lane_key(
      int unsigned key_ord,
      logic [3:0]  ts_low = 4'h0,
      bit          ts12   = 1'b0
    );
      logic [12:0] tcc8n;
      int unsigned sk;
      sk = lane_key_ord_to_search_key(key_ord);
      tcc8n = '0;
      tcc8n[11:4] = sk[7:0];
      tcc8n[12]   = ts12;
      tcc8n[3:0]  = ts_low;
      return tcc8n;
    endfunction

    function bit lane_matches_hit(
      logic [38:0] data,
      logic [3:0]  channel,
      bit          is_empty
    );
      int unsigned lane_bits;
      if (is_empty) begin
        return (channel[1:0] == interleaving_index[1:0]);
      end

      if (interleaving_factor <= 1) begin
        return 1'b1;
      end

      lane_bits = data[TCC8N_LO + 5 -: 2];
      return lane_bits == interleaving_index;
    endfunction

    function bit accepts_ingress(
      logic [38:0] data,
      logic [3:0]  channel,
      bit          is_empty,
      bit          valid,
      bit          ready,
      bit          err
    );
      if (!(valid && ready)) begin
        return 1'b0;
      end
      if (!go) begin
        return 1'b0;
      end
      if (!(
            (run_state == RUN_STATE_RUNNING) ||
            (run_state == RUN_STATE_TERMINATING && !endofrun_seen)
          )) begin
        return 1'b0;
      end
      if (is_empty) begin
        return 1'b0;
      end
      if (!lane_matches_hit(data, channel, is_empty)) begin
        return 1'b0;
      end
      if (filter_inerr && err) begin
        return 1'b0;
      end
      return 1'b1;
    endfunction

    function void note_ingress_beat(
      logic [38:0] data,
      logic [3:0]  channel,
      bit          is_empty,
      bit          valid
    );
      if (!valid) begin
        return;
      end
      if (is_empty && run_state == RUN_STATE_TERMINATING &&
          lane_matches_hit(data, channel, is_empty)) begin
        endofrun_seen = 1'b1;
      end
    endfunction

    function void note_ctrl_cmd(logic [8:0] cmd);
      unique case (cmd)
        CTRL_IDLE: begin
          run_state     = RUN_STATE_IDLE;
          endofrun_seen = 1'b0;
        end
        CTRL_RUN_PREPARE: begin
          run_state     = RUN_STATE_RUN_PREPARE;
          endofrun_seen = 1'b0;
        end
        CTRL_SYNC: begin
          run_state     = RUN_STATE_SYNC;
          endofrun_seen = 1'b0;
        end
        CTRL_RUNNING: begin
          run_state     = RUN_STATE_RUNNING;
          endofrun_seen = 1'b0;
        end
        CTRL_TERMINATING: begin
          run_state = RUN_STATE_TERMINATING;
        end
        default: begin
        end
      endcase
    endfunction

    function void note_csr_write(bit [4:0] address, bit [31:0] writedata);
      unique case (address)
        5'd2: begin
          go           = writedata[0];
          filter_inerr = writedata[4];
        end
        5'd3: begin
          expected_latency = writedata[15:0];
        end
        default: begin
        end
      endcase
    endfunction
  endclass

  // ═══════════════════════════════════════════════════════════════
  // Sequence items
  // ═══════════════════════════════════════════════════════════════

  // ── ingress hit ───────────────────────────────────────────────
  class hit_seq_item extends uvm_sequence_item;
    `uvm_object_utils(hit_seq_item)

    rand bit [3:0]  asic;
    rand bit [3:0]  ingress_channel;
    rand bit [4:0]  channel;
    rand bit [12:0] tcc8n;
    rand bit [2:0]  tcc1n6;
    rand bit [4:0]  tfine;
    rand bit [8:0]  et1n6;
    rand bit        has_error;
    rand bit        is_empty_marker;
    rand bit [63:0] metadata;
    rand bit        metadata_valid;

    // Search key is ts[11:4], which is the CAM compare value and the
    // low 8 bits of the emitted subheader search-key epoch.
    function bit [7:0] search_key();
      return tcc8n[11:4];
    endfunction

    function bit [8:0] search_epoch();
      return tcc8n[12:4];
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

    function logic [3:0] input_channel();
      if (is_empty_marker) begin
        return ingress_channel;
      end
      return asic;
    endfunction

    // Constraint: interleaving filter acceptance
    constraint c_interleaving {
      ((tcc8n >> 4) % INTERLEAVING_FACTOR) == INTERLEAVING_INDEX;
    }

    function new(string name = "hit_seq_item");
      super.new(name);
      ingress_channel = '0;
      is_empty_marker = 1'b0;
      metadata = '0;
      metadata_valid = 1'b0;
    endfunction
  endclass

  function automatic void fill_long_run_fingerprint(
    longint unsigned unique_idx,
    ref hit_seq_item hit
  );
    // Only consume fields that survive the DUT datapath. ingress_channel is not
    // stored in side RAM or emitted on egress for normal data hits, so using it
    // as part of the "unique" pattern aliases the observable fingerprint every
    // 16 beats in long profile runs.
    hit.asic            = unique_idx[3:0];
    hit.channel         = unique_idx[8:4];
    hit.tfine           = unique_idx[13:9];
    hit.tcc1n6          = unique_idx[16:14];
    hit.et1n6           = unique_idx[25:17];
    hit.ingress_channel = unique_idx[29:26];
  endfunction

  // ── egress hit (monitored) ────────────────────────────────────
  class out_seq_item extends uvm_sequence_item;
    `uvm_object_utils(out_seq_item)

    bit        is_subheader;
    bit [7:0]  search_key;     // subheader[31:24]
    bit [7:0]  hit_count;      // subheader[15:8]
    bit [7:0]  active_search_key;
    int unsigned hit_index_in_epoch;
    int unsigned hit_count_expected;
    bit [35:0] raw_data;
    bit        sop;
    bit        eop;
    bit        error;
    bit        cache_miss;
    bit [63:0] metadata;
    bit        metadata_valid;

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

  // ── internal push-write observation ───────────────────────────
  class debug_push_item extends uvm_sequence_item;
    `uvm_object_utils(debug_push_item)

    bit [15:0] slot_addr;
    bit [38:0] raw_hit;
    bit [63:0] push_count;
    bit [63:0] overwrite_count;

    function new(string name = "debug_push_item");
      super.new(name);
    endfunction

    function bit [12:0] tcc8n();
      return raw_hit[TCC8N_HI:TCC8N_LO];
    endfunction

    function bit [7:0] search_key();
      return tcc8n()[11:4];
    endfunction

    function bit [7:0] ts50p();
      return raw_hit[TCC1N6_HI:TFINE_LO];
    endfunction
  endclass

  class debug_pop_item extends uvm_sequence_item;
    `uvm_object_utils(debug_pop_item)

    bit [15:0] slot_addr;
    bit [38:0] raw_hit;
    bit        occupied;
    bit [63:0] pop_count;
    bit [7:0]  active_search_key;

    function new(string name = "debug_pop_item");
      super.new(name);
    endfunction

    function bit [12:0] tcc8n();
      return raw_hit[TCC8N_HI:TCC8N_LO];
    endfunction

    function bit [7:0] search_key();
      return tcc8n()[11:4];
    endfunction

    function bit [7:0] ts50p();
      return raw_hit[TCC1N6_HI:TFINE_LO];
    endfunction
  endclass

endpackage

`endif

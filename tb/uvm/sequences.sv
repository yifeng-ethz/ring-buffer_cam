// sequences.sv — UVM sequences for ring_buffer_cam test cases
// Provides both directed and constrained-random sequences.

`ifndef SEQUENCES_SV
`define SEQUENCES_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

// ═══════════════════════════════════════════════════════════════
// Helper: run-control startup sequence
// ═══════════════════════════════════════════════════════════════
class startup_seq extends uvm_sequence #(ring_buffer_cam_pkg::ctrl_seq_item);
  `uvm_object_utils(startup_seq)

  int unsigned expected_latency = 128;

  function new(string name = "startup_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::ctrl_seq_item ctrl;

    // Enter RUN_PREPARE first. The DUT's ready is registered from the
    // current state, so an initial IDLE->RUN_PREPARE command is acked by
    // IDLE immediately. Re-issuing RUN_PREPARE forces the driver to wait
    // for the actual flush-done acknowledgement in RUN_PREPARE.
    ctrl = ring_buffer_cam_pkg::ctrl_seq_item::type_id::create("ctrl");
    start_item(ctrl);
    ctrl.cmd = ring_buffer_cam_pkg::CTRL_RUN_PREPARE;
    finish_item(ctrl);

    ctrl = ring_buffer_cam_pkg::ctrl_seq_item::type_id::create("ctrl");
    start_item(ctrl);
    ctrl.cmd = ring_buffer_cam_pkg::CTRL_RUN_PREPARE;
    finish_item(ctrl);

    // SYNC
    ctrl = ring_buffer_cam_pkg::ctrl_seq_item::type_id::create("ctrl");
    start_item(ctrl);
    ctrl.cmd = ring_buffer_cam_pkg::CTRL_SYNC;
    finish_item(ctrl);

    // RUNNING
    ctrl = ring_buffer_cam_pkg::ctrl_seq_item::type_id::create("ctrl");
    start_item(ctrl);
    ctrl.cmd = ring_buffer_cam_pkg::CTRL_RUNNING;
    finish_item(ctrl);
  endtask
endclass

class ctrl_cmd_seq extends uvm_sequence #(ring_buffer_cam_pkg::ctrl_seq_item);
  `uvm_object_utils(ctrl_cmd_seq)

  logic [8:0] cmd;

  function new(string name = "ctrl_cmd_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::ctrl_seq_item ctrl;
    ctrl = ring_buffer_cam_pkg::ctrl_seq_item::type_id::create("ctrl");
    start_item(ctrl);
    ctrl.cmd = cmd;
    finish_item(ctrl);
  endtask
endclass

// ═══════════════════════════════════════════════════════════════
// Helper: CSR write sequence
// ═══════════════════════════════════════════════════════════════
class csr_write_seq extends uvm_sequence #(ring_buffer_cam_pkg::csr_seq_item);
  `uvm_object_utils(csr_write_seq)

  int unsigned addr;
  bit [31:0]   data;

  function new(string name = "csr_write_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::csr_seq_item item;
    item = ring_buffer_cam_pkg::csr_seq_item::type_id::create("csr_wr");
    start_item(item);
    item.is_write  = 1;
    item.address   = addr[4:0];
    item.writedata = data;
    finish_item(item);
  endtask
endclass

class csr_read_seq extends uvm_sequence #(ring_buffer_cam_pkg::csr_seq_item);
  `uvm_object_utils(csr_read_seq)

  int unsigned addr;
  logic [31:0] data;

  function new(string name = "csr_read_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::csr_seq_item item;
    item = ring_buffer_cam_pkg::csr_seq_item::type_id::create("csr_rd");
    start_item(item);
    item.is_write  = 0;
    item.address   = addr[4:0];
    item.writedata = '0;
    finish_item(item);
    data = item.readdata;
  endtask
endclass

// ═══════════════════════════════════════════════════════════════
// D001: single_push_pop
// ═══════════════════════════════════════════════════════════════
class single_push_pop_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(single_push_pop_seq)

  int unsigned search_key = 4;
  bit [3:0]    hit_asic = 0;
  bit [3:0]    ingress_channel = 0;
  bit [4:0]    hit_channel = 3;
  bit [3:0]    ts_low = 0;
  bit [2:0]    hit_tcc1n6 = 0;
  bit [4:0]    hit_tfine = 0;
  bit [8:0]    hit_et1n6 = 42;
  bit          hit_has_error = 0;
  bit          use_raw_tcc8n = 0;
  bit [12:0]   raw_tcc8n = '0;

  function new(string name = "single_push_pop_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit");
    start_item(hit);
    hit.asic      = hit_asic;
    hit.ingress_channel = ingress_channel;
    hit.channel   = hit_channel;
    hit.tcc8n     = use_raw_tcc8n ? raw_tcc8n : 13'((search_key * 16) + ts_low);
    hit.tcc1n6    = hit_tcc1n6;
    hit.tfine     = hit_tfine;
    hit.et1n6     = hit_et1n6;
    hit.has_error = hit_has_error;
    finish_item(hit);
  endtask
endclass

class single_error_hit_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(single_error_hit_seq)

  int unsigned search_key = 4;
  bit [3:0]    hit_asic = 0;
  bit [3:0]    ingress_channel = 0;
  bit [4:0]    hit_channel = 3;
  bit [3:0]    ts_low = 0;
  bit [2:0]    hit_tcc1n6 = 0;
  bit [4:0]    hit_tfine = 0;
  bit [8:0]    hit_et1n6 = 9'h001;
  bit          hit_has_error = 1;
  bit          hit_is_empty_marker = 0;
  bit          use_raw_tcc8n = 0;
  bit [12:0]   raw_tcc8n = '0;

  function new(string name = "single_error_hit_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit");
    start_item(hit);
    hit.asic      = hit_asic;
    hit.ingress_channel = ingress_channel;
    hit.channel   = hit_channel;
    hit.tcc8n     = use_raw_tcc8n ? raw_tcc8n : 13'((search_key * 16) + ts_low);
    hit.tcc1n6    = hit_tcc1n6;
    hit.tfine     = hit_tfine;
    hit.et1n6     = hit_et1n6;
    hit.has_error = hit_has_error;
    hit.is_empty_marker = hit_is_empty_marker;
    finish_item(hit);
  endtask
endclass

class error_burst_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(error_burst_seq)

  int unsigned num_hits = 16;
  int unsigned search_key = 4;
  bit [3:0]    hit_asic_base = 0;
  bit [3:0]    ingress_channel = 0;
  bit [4:0]    hit_channel = 3;
  bit [3:0]    ts_low_base = 0;
  bit [2:0]    hit_tcc1n6_base = 0;
  bit [4:0]    hit_tfine_base = 0;
  bit [8:0]    hit_et1n6_base = 9'h001;
  bit          use_raw_tcc8n = 0;
  bit [12:0]   raw_tcc8n = '0;

  function new(string name = "error_burst_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    for (int i = 0; i < num_hits; i++) begin
      hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit");
      start_item(hit);
      hit.asic      = hit_asic_base + i[3:0];
      hit.ingress_channel = ingress_channel;
      hit.channel   = hit_channel;
      hit.tcc8n     = use_raw_tcc8n ? raw_tcc8n : 13'((search_key * 16) + ((ts_low_base + i) & 4'hf));
      hit.tcc1n6    = hit_tcc1n6_base + i[2:0];
      hit.tfine     = hit_tfine_base + i[4:0];
      hit.et1n6     = hit_et1n6_base + i[8:0];
      hit.has_error = 1'b1;
      hit.is_empty_marker = 1'b0;
      finish_item(hit);
    end
  endtask
endclass

// ═══════════════════════════════════════════════════════════════
// D003: same-key burst (configurable count)
// ═══════════════════════════════════════════════════════════════
class same_key_burst_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(same_key_burst_seq)

  int unsigned num_hits   = 128;
  int unsigned search_key = 8;

  function new(string name = "same_key_burst_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    for (int i = 0; i < num_hits; i++) begin
      hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit");
      start_item(hit);
      hit.asic      = i[3:0];
      hit.ingress_channel = i[3:0];
      hit.channel   = 3;
      hit.tcc8n     = 13'(search_key * 16);
      hit.tcc1n6    = i[2:0];
      hit.tfine     = i[4:0];
      hit.et1n6     = i[8:0];
      hit.has_error = 0;
      finish_item(hit);
    end
  endtask
endclass

// ═══════════════════════════════════════════════════════════════
// D004: sequential keys
// ═══════════════════════════════════════════════════════════════
class sequential_keys_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(sequential_keys_seq)

  int unsigned num_keys     = 16;
  int unsigned hits_per_key = 4;
  int unsigned start_key    = 4;

  function new(string name = "sequential_keys_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    int unsigned actual_key;
    for (int k = 0; k < num_keys; k++) begin
      actual_key = ((start_key + k) * ring_buffer_cam_pkg::INTERLEAVING_FACTOR) +
                   ring_buffer_cam_pkg::INTERLEAVING_INDEX;
      for (int h = 0; h < hits_per_key; h++) begin
        hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit");
        start_item(hit);
        hit.asic      = h[3:0];
        hit.ingress_channel = h[3:0];
        hit.channel   = 3;
        hit.tcc8n     = 13'(actual_key * 16);
        hit.tcc1n6    = h[2:0];
        hit.tfine     = h[4:0];
        hit.et1n6     = 9'(k * hits_per_key + h);
        hit.has_error = 0;
        finish_item(hit);
      end
    end
  endtask
endclass

// ═══════════════════════════════════════════════════════════════
// D006: overwrite stress
// ═══════════════════════════════════════════════════════════════
class overwrite_stress_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(overwrite_stress_seq)

  int unsigned num_hits = 1024;  // > CAM depth to force overwrites

  function new(string name = "overwrite_stress_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    int unsigned actual_key;
    for (int i = 0; i < num_hits; i++) begin
      hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit");
      start_item(hit);
      // Use many different search keys so hits spread across the CAM
      actual_key = (((i / 4) % 64) * ring_buffer_cam_pkg::INTERLEAVING_FACTOR) +
                   ring_buffer_cam_pkg::INTERLEAVING_INDEX;
      hit.asic      = i[3:0];
      hit.ingress_channel = i[3:0];
      hit.channel   = 3;
      hit.tcc8n     = 13'(actual_key * 16);
      hit.tcc1n6    = i[2:0];
      hit.tfine     = i[4:0];
      hit.et1n6     = i[8:0];
      hit.has_error = 0;
      finish_item(hit);
    end
  endtask
endclass

class overwrite_profile_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(overwrite_profile_seq)

  int unsigned num_hits = 2048;
  int unsigned lane_key_start_ord = 4;
  int unsigned pool_keys = 1;
  int unsigned hits_per_key_switch = 1;
  int unsigned inter_hit_gap_cycles = 0;
  int unsigned burst_len = 0;
  int unsigned burst_idle_cycles = 0;
  int unsigned progress_stride = 0;
  string       progress_tag = "";

  function new(string name = "overwrite_profile_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    int unsigned actual_key;
    int unsigned lane_key_ord;

    for (int i = 0; i < num_hits; i++) begin
      lane_key_ord = lane_key_start_ord +
                     ((i / hits_per_key_switch) % (pool_keys == 0 ? 1 : pool_keys));
      actual_key = (lane_key_ord * ring_buffer_cam_pkg::INTERLEAVING_FACTOR) +
                   ring_buffer_cam_pkg::INTERLEAVING_INDEX;

      hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("pressure_hit");
      start_item(hit);
      ring_buffer_cam_pkg::fill_long_run_fingerprint(i, hit);
      hit.tcc8n     = 13'(actual_key * 16);
      hit.has_error = 0;
      hit.is_empty_marker = 1'b0;
      finish_item(hit);

      if (progress_stride > 0 && (((i + 1) % progress_stride) == 0)) begin
        `uvm_info("SEQ", $sformatf(
          "%s progress: sent=%0d/%0d",
          (progress_tag == "") ? get_name() : progress_tag, i + 1, num_hits), UVM_LOW)
      end

      if (inter_hit_gap_cycles > 0)
        #(inter_hit_gap_cycles * 8ns);
      if (burst_len > 0 && burst_idle_cycles > 0 && ((i + 1) % burst_len) == 0)
        #(burst_idle_cycles * 8ns);
    end

    `uvm_info("SEQ", $sformatf(
      "%s completed: sent=%0d/%0d",
      (progress_tag == "") ? get_name() : progress_tag, num_hits, num_hits), UVM_LOW)
  endtask
endclass

class profile_traffic_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(profile_traffic_seq)

  int unsigned num_hits = 2048;
  int unsigned lane_key_start_ord = 0;
  int unsigned lane_key_ord_list[$];
  int unsigned pool_keys = 1;
  int unsigned hits_per_key_switch = 1;
  bit          randomize_key_order = 0;
  int unsigned inter_hit_gap_cycles = 0;
  int unsigned random_gap_min_cycles = 0;
  int unsigned random_gap_max_cycles = 0;
  bit          use_bernoulli_arrival = 0;
  byte unsigned bernoulli_arrival_threshold = 8'd128;
  logic [31:0] lfsr_seed = 32'h1ace_b00c;
  int unsigned fingerprint_start_index = 0;
  bit          metadata_enable = 0;
  bit          metadata_valid_alternates = 0;
  logic [63:0] metadata_base = 64'h0000_0000_6600_0000;
  int unsigned burst_len = 0;
  int unsigned burst_idle_cycles = 0;
  int unsigned progress_stride = 0;
  string       progress_tag = "";

  function new(string name = "profile_traffic_seq");
    super.new(name);
  endfunction

  function automatic logic [31:0] next_lfsr(logic [31:0] state);
    logic feedback;
    feedback = state[31] ^ state[21] ^ state[1] ^ state[0];
    return {state[30:0], feedback};
  endfunction

  function automatic int unsigned choose_gap_cycles(ref logic [31:0] lfsr);
    int unsigned span;
    int unsigned gap_cycles;

    gap_cycles = inter_hit_gap_cycles;

    if (random_gap_max_cycles > random_gap_min_cycles) begin
      span = random_gap_max_cycles - random_gap_min_cycles + 1;
      lfsr = next_lfsr(lfsr);
      gap_cycles += random_gap_min_cycles + (lfsr % span);
    end else begin
      gap_cycles += random_gap_min_cycles;
    end

    if (use_bernoulli_arrival) begin
      gap_cycles = 0;
      do begin
        lfsr = next_lfsr(lfsr);
        if (lfsr[7:0] < bernoulli_arrival_threshold) begin
          break;
        end
        gap_cycles++;
      end while (gap_cycles < 4096);
    end

    return gap_cycles;
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    int unsigned actual_key;
    int unsigned lane_key_ord;
    int unsigned effective_pool_keys;
    int unsigned key_idx;
    int unsigned key_switch;
    int unsigned gap_cycles;
    logic [31:0] lfsr;

    effective_pool_keys = (lane_key_ord_list.size() > 0) ?
                          lane_key_ord_list.size() :
                          ((pool_keys == 0) ? 1 : pool_keys);
    key_switch = (hits_per_key_switch == 0) ? 1 : hits_per_key_switch;
    lfsr = (lfsr_seed == '0) ? 32'h1ace_b00c : lfsr_seed;

    for (int i = 0; i < num_hits; i++) begin
      if (randomize_key_order && effective_pool_keys > 1) begin
        lfsr = next_lfsr(lfsr);
        key_idx = lfsr % effective_pool_keys;
      end else begin
        key_idx = (i / key_switch) % effective_pool_keys;
      end
      if (lane_key_ord_list.size() > 0) begin
        lane_key_ord = lane_key_ord_list[key_idx];
      end else begin
        lane_key_ord = lane_key_start_ord + key_idx;
      end
      actual_key = (lane_key_ord * ring_buffer_cam_pkg::INTERLEAVING_FACTOR) +
                   ring_buffer_cam_pkg::INTERLEAVING_INDEX;

      hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("profile_hit");
      start_item(hit);
      ring_buffer_cam_pkg::fill_long_run_fingerprint(fingerprint_start_index + i, hit);
      hit.tcc8n     = 13'(actual_key * 16);
      hit.has_error = 1'b0;
      hit.is_empty_marker = 1'b0;
      if (metadata_enable) begin
        hit.metadata = metadata_base ^ 64'(fingerprint_start_index + i);
        hit.metadata_valid = metadata_valid_alternates ? ~i[0] : 1'b1;
      end
      finish_item(hit);

      if (progress_stride > 0 && (((i + 1) % progress_stride) == 0)) begin
        `uvm_info("SEQ", $sformatf(
          "%s progress: sent=%0d/%0d",
          (progress_tag == "") ? get_name() : progress_tag, i + 1, num_hits), UVM_LOW)
      end

      gap_cycles = choose_gap_cycles(lfsr);
      if (gap_cycles > 0)
        #(gap_cycles * 8ns);
      if (burst_len > 0 && burst_idle_cycles > 0 && ((i + 1) % burst_len) == 0)
        #(burst_idle_cycles * 8ns);
    end

    `uvm_info("SEQ", $sformatf(
      "%s completed: sent=%0d/%0d",
      (progress_tag == "") ? get_name() : progress_tag, num_hits, num_hits), UVM_LOW)
  endtask
endclass

class weighted_profile_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(weighted_profile_seq)

  int unsigned lane_key_start_ord = 0;
  int unsigned lane_key_ord_list[$];
  int unsigned key_hits_per_epoch[$];
  int unsigned num_epochs = 1;
  int unsigned inter_hit_gap_cycles = 0;
  int unsigned inter_epoch_gap_cycles = 0;
  int unsigned fingerprint_start_index = 0;
  int unsigned progress_stride = 0;
  string       progress_tag = "";

  function new(string name = "weighted_profile_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    int unsigned total_hits;
    int unsigned sent_hits;
    int unsigned actual_key;
    int unsigned lane_key_ord;

    total_hits = 0;
    foreach (key_hits_per_epoch[idx]) begin
      total_hits += key_hits_per_epoch[idx];
    end
    total_hits *= ((num_epochs == 0) ? 1 : num_epochs);
    sent_hits = 0;

    if (key_hits_per_epoch.size() == 0) begin
      `uvm_fatal("SEQ", $sformatf(
        "%s requires at least one weighted key slot",
        (progress_tag == "") ? get_name() : progress_tag))
    end
    if (lane_key_ord_list.size() > 0 &&
        lane_key_ord_list.size() != key_hits_per_epoch.size()) begin
      `uvm_fatal("SEQ", $sformatf(
        "%s key-list size (%0d) must match weight-vector size (%0d)",
        (progress_tag == "") ? get_name() : progress_tag,
        lane_key_ord_list.size(), key_hits_per_epoch.size()))
    end

    for (int epoch = 0; epoch < ((num_epochs == 0) ? 1 : num_epochs); epoch++) begin
      foreach (key_hits_per_epoch[idx]) begin
        if (lane_key_ord_list.size() > 0) begin
          lane_key_ord = lane_key_ord_list[idx];
        end else begin
          lane_key_ord = lane_key_start_ord + idx;
        end
        actual_key = (lane_key_ord * ring_buffer_cam_pkg::INTERLEAVING_FACTOR) +
                     ring_buffer_cam_pkg::INTERLEAVING_INDEX;
        for (int hit_idx = 0; hit_idx < key_hits_per_epoch[idx]; hit_idx++) begin
          hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("weighted_hit");
          start_item(hit);
          ring_buffer_cam_pkg::fill_long_run_fingerprint(fingerprint_start_index + sent_hits, hit);
          hit.tcc8n     = 13'(actual_key * 16);
          hit.has_error = 1'b0;
          hit.is_empty_marker = 1'b0;
          finish_item(hit);

          sent_hits++;
          if (progress_stride > 0 && ((sent_hits % progress_stride) == 0)) begin
            `uvm_info("SEQ", $sformatf(
              "%s progress: sent=%0d/%0d",
              (progress_tag == "") ? get_name() : progress_tag, sent_hits, total_hits), UVM_LOW)
          end
          if (inter_hit_gap_cycles > 0)
            #(inter_hit_gap_cycles * 8ns);
        end
      end
      if (inter_epoch_gap_cycles > 0)
        #(inter_epoch_gap_cycles * 8ns);
    end

    `uvm_info("SEQ", $sformatf(
      "%s completed: sent=%0d/%0d",
      (progress_tag == "") ? get_name() : progress_tag, sent_hits, total_hits), UVM_LOW)
  endtask
endclass

class endofrun_marker_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(endofrun_marker_seq)

  int unsigned lane_channel = ring_buffer_cam_pkg::INTERLEAVING_INDEX;

  function new(string name = "endofrun_marker_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("endofrun_marker");
    start_item(hit);
    hit.asic           = '0;
    hit.ingress_channel = lane_channel[3:0];
    hit.channel        = '0;
    hit.tcc8n          = '0;
    hit.tcc1n6         = '0;
    hit.tfine          = '0;
    hit.et1n6          = '0;
    hit.has_error      = 1'b0;
    hit.is_empty_marker = 1'b1;
    finish_item(hit);
  endtask
endclass

// ═══════════════════════════════════════════════════════════════
// R001: random push-pop (constrained random)
// ═══════════════════════════════════════════════════════════════
class random_push_pop_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(random_push_pop_seq)

  rand int unsigned num_hits;
  rand int unsigned search_key;
  rand int unsigned inter_hit_delay;

  constraint c_hits    { num_hits inside {[1:256]}; }
  constraint c_key     { search_key inside {[0:63]}; }
  constraint c_delay   { inter_hit_delay inside {[0:3]}; }

  function new(string name = "random_push_pop_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    int unsigned actual_key;
    actual_key = (search_key * ring_buffer_cam_pkg::INTERLEAVING_FACTOR) +
                 ring_buffer_cam_pkg::INTERLEAVING_INDEX;
    for (int i = 0; i < num_hits; i++) begin
      hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit");
      start_item(hit);
      assert(hit.randomize() with {
        tcc8n == 13'(actual_key * 16);
        has_error == 0;
        is_empty_marker == 0;
      });
      finish_item(hit);
      // Optional inter-hit delay
      if (inter_hit_delay > 0)
        #(inter_hit_delay * 8ns);
    end
  endtask
endclass

// ═══════════════════════════════════════════════════════════════
// R002: random multi-key
// ═══════════════════════════════════════════════════════════════
class random_multi_key_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(random_multi_key_seq)

  rand int unsigned num_keys;
  rand int unsigned hits_per_key;

  constraint c_keys { num_keys inside {[1:16]}; }
  constraint c_hits { hits_per_key inside {[1:64]}; }

  function new(string name = "random_multi_key_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    int unsigned base_key;
    base_key = $urandom_range(0, 63 - num_keys);

    for (int k = 0; k < num_keys; k++) begin
      for (int h = 0; h < hits_per_key; h++) begin
        hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit");
        start_item(hit);
        assert(hit.randomize() with {
          tcc8n == 13'((((base_key + k) * ring_buffer_cam_pkg::INTERLEAVING_FACTOR) +
                        ring_buffer_cam_pkg::INTERLEAVING_INDEX) * 16);
          has_error == 0;
          is_empty_marker == 0;
        });
        finish_item(hit);
      end
    end
  endtask
endclass

// ═══════════════════════════════════════════════════════════════
// R006: random throughput measurement
// ═══════════════════════════════════════════════════════════════
class random_throughput_seq extends uvm_sequence #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_object_utils(random_throughput_seq)

  rand int unsigned num_hits;
  rand int unsigned search_key;

  constraint c_hits { num_hits inside {[64:512]}; }
  constraint c_key  { search_key inside {[4:60]}; }

  function new(string name = "random_throughput_seq");
    super.new(name);
  endfunction

  task body();
    ring_buffer_cam_pkg::hit_seq_item hit;
    int unsigned actual_key;
    actual_key = (search_key * ring_buffer_cam_pkg::INTERLEAVING_FACTOR) +
                 ring_buffer_cam_pkg::INTERLEAVING_INDEX;
    for (int i = 0; i < num_hits; i++) begin
      hit = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit");
      start_item(hit);
      hit.asic      = i[3:0];
      hit.ingress_channel = i[3:0];
      hit.channel   = 3;
      hit.tcc8n     = 13'(actual_key * 16);
      hit.tcc1n6    = i[2:0];
      hit.tfine     = i[4:0];
      hit.et1n6     = i[8:0];
      hit.has_error = 0;
      hit.is_empty_marker = 1'b0;
      finish_item(hit);
    end
  endtask
endclass

`endif

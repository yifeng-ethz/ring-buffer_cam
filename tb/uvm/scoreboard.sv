// scoreboard.sv — Contract-derived resident model and egress checker

`ifndef SCOREBOARD_SV
`define SCOREBOARD_SV

import uvm_pkg::*;
`include "uvm_macros.svh"
`uvm_analysis_imp_decl(_out)
`uvm_analysis_imp_decl(_push)

class scoreboard extends uvm_scoreboard;
  `uvm_component_utils(scoreboard)

  typedef struct packed {
    bit [7:0] search_key;
    bit [8:0] et1n6;
    bit [7:0] ts50p;
    bit [3:0] asic;
    bit [4:0] channel;
  } hit_fingerprint_t;

  typedef struct {
    bit               valid;
    hit_fingerprint_t fp;
  } slot_model_t;

  uvm_analysis_imp #(ring_buffer_cam_pkg::hit_seq_item, scoreboard) accept_imp;
  uvm_analysis_imp_push #(ring_buffer_cam_pkg::debug_push_item, scoreboard) push_imp;
  uvm_analysis_imp_out #(ring_buffer_cam_pkg::out_seq_item, scoreboard) out_imp;

  ring_buffer_cam_pkg::ring_buffer_cam_cfg m_cfg;
  slot_model_t                             slot_model[];

  int unsigned total_ingress_accepted;
  int unsigned total_written;
  int unsigned total_drained;
  int unsigned total_expected_overwrites;
  int unsigned total_unexpected_outputs;
  int unsigned total_cache_miss_outputs;
  int unsigned total_subheaders;
  int unsigned total_zero_hit_subheaders;
  int unsigned total_subheader_mismatches;
  int unsigned current_remaining;
  int unsigned current_remaining_by_key[bit [7:0]];
  int unsigned max_remaining_seen;
  int unsigned max_remaining_seen_by_key[bit [7:0]];
  int unsigned overwrite_new_key_count[bit [7:0]];
  int unsigned overwrite_old_key_count[bit [7:0]];

  bit          epoch_active;
  bit [7:0]    active_search_key;
  int unsigned active_expected_hits;
  int unsigned active_seen_hits;

  function new(string name, uvm_component parent);
    super.new(name, parent);
    total_ingress_accepted = 0;
    total_written = 0;
    total_drained = 0;
    total_expected_overwrites = 0;
    total_unexpected_outputs = 0;
    total_cache_miss_outputs = 0;
    total_subheaders = 0;
    total_zero_hit_subheaders = 0;
    total_subheader_mismatches = 0;
    current_remaining = 0;
    max_remaining_seen = 0;
    epoch_active = 1'b0;
    active_search_key = '0;
    active_expected_hits = 0;
    active_seen_hits = 0;
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    accept_imp = new("accept_imp", this);
    push_imp = new("push_imp", this);
    out_imp = new("out_imp", this);
    if (!uvm_config_db#(ring_buffer_cam_pkg::ring_buffer_cam_cfg)::get(this, "", "cfg", m_cfg))
      `uvm_fatal("SCB", "Failed to get cfg from config_db")
    slot_model = new[m_cfg.ring_buffer_n_entry];
  endfunction

  function hit_fingerprint_t item_to_fp(ring_buffer_cam_pkg::hit_seq_item item);
    hit_fingerprint_t fp;
    fp.search_key = item.search_key();
    fp.et1n6      = item.et1n6;
    fp.ts50p      = {item.tcc1n6, item.tfine};
    fp.asic       = item.asic;
    fp.channel    = item.channel;
    return fp;
  endfunction

  function bit output_matches_fp(ring_buffer_cam_pkg::out_seq_item item, hit_fingerprint_t fp);
    return
      fp.search_key == item.active_search_key &&
      fp.et1n6      == item.et1n6 &&
      fp.ts50p      == item.ts50p &&
      fp.asic       == item.asic &&
      fp.channel    == item.channel;
  endfunction

  function int find_match_index(ring_buffer_cam_pkg::out_seq_item item);
    for (int i = 0; i < slot_model.size(); i++) begin
      if (slot_model[i].valid && output_matches_fp(item, slot_model[i].fp)) begin
        return i;
      end
    end
    return -1;
  endfunction

  function int unsigned remaining_entries();
    return current_remaining;
  endfunction

  function int unsigned remaining_entries_for_key(bit [7:0] search_key);
    if (!current_remaining_by_key.exists(search_key)) begin
      return 0;
    end
    return current_remaining_by_key[search_key];
  endfunction

  function void update_peak_residency();
    int unsigned remaining;
    bit [7:0] search_key;
    remaining = remaining_entries();
    if (remaining > max_remaining_seen) begin
      max_remaining_seen = remaining;
    end
    foreach (current_remaining_by_key[search_key]) begin
      remaining = current_remaining_by_key[search_key];
      if (!max_remaining_seen_by_key.exists(search_key) ||
          remaining > max_remaining_seen_by_key[search_key]) begin
        max_remaining_seen_by_key[search_key] = remaining;
      end
    end
  endfunction

  function int unsigned max_remaining_entries();
    return max_remaining_seen;
  endfunction

  function int unsigned max_remaining_entries_for_key(bit [7:0] search_key);
    if (!max_remaining_seen_by_key.exists(search_key)) begin
      return 0;
    end
    return max_remaining_seen_by_key[search_key];
  endfunction

  function int unsigned overwrite_events_for_new_key(bit [7:0] search_key);
    if (!overwrite_new_key_count.exists(search_key)) begin
      return 0;
    end
    return overwrite_new_key_count[search_key];
  endfunction

  function int unsigned overwrite_events_for_evicted_key(bit [7:0] search_key);
    if (!overwrite_old_key_count.exists(search_key)) begin
      return 0;
    end
    return overwrite_old_key_count[search_key];
  endfunction

  function bit epoch_idle();
    return !epoch_active;
  endfunction

  function void clear_epoch();
    epoch_active = 1'b0;
    active_search_key = '0;
    active_expected_hits = 0;
    active_seen_hits = 0;
  endfunction

  function void write(ring_buffer_cam_pkg::hit_seq_item item);
    if (item.is_empty_marker) begin
      return;
    end
    total_ingress_accepted++;
  endfunction

  function hit_fingerprint_t debug_item_to_fp(ring_buffer_cam_pkg::debug_push_item item);
    hit_fingerprint_t fp;
    fp.search_key = item.search_key();
    fp.et1n6      = item.raw_hit[ring_buffer_cam_pkg::ET1N6_HI:ring_buffer_cam_pkg::ET1N6_LO];
    fp.ts50p      = item.raw_hit[ring_buffer_cam_pkg::TCC1N6_HI:ring_buffer_cam_pkg::TFINE_LO];
    fp.asic       = item.raw_hit[ring_buffer_cam_pkg::ASIC_HI:ring_buffer_cam_pkg::ASIC_LO];
    fp.channel    = item.raw_hit[ring_buffer_cam_pkg::CHANNEL_HI:ring_buffer_cam_pkg::CHANNEL_LO];
    return fp;
  endfunction

  function void write_push(ring_buffer_cam_pkg::debug_push_item item);
    int unsigned slot_addr;
    hit_fingerprint_t fp;
    bit old_valid;
    bit [7:0] old_key;

    slot_addr = item.slot_addr;
    if (slot_addr >= slot_model.size()) begin
      `uvm_error("SCB", $sformatf(
        "Observed push_write to slot %0d outside modeled ring size %0d",
        slot_addr, slot_model.size()))
      return;
    end

    fp = debug_item_to_fp(item);
    old_valid = slot_model[slot_addr].valid;

    if (old_valid) begin
      old_key = slot_model[slot_addr].fp.search_key;
      total_expected_overwrites++;
      if (!overwrite_old_key_count.exists(old_key)) begin
        overwrite_old_key_count[old_key] = 0;
      end
      if (!overwrite_new_key_count.exists(fp.search_key)) begin
        overwrite_new_key_count[fp.search_key] = 0;
      end
      overwrite_old_key_count[old_key]++;
      overwrite_new_key_count[fp.search_key]++;
      if (old_key != fp.search_key) begin
        if (!current_remaining_by_key.exists(old_key)) begin
          current_remaining_by_key[old_key] = 0;
        end
        current_remaining_by_key[old_key]--;
      end
      `uvm_info("SCB", $sformatf(
        "Expected overwrite at slot %0d old_key=%0d new_key=%0d",
        slot_addr, slot_model[slot_addr].fp.search_key, fp.search_key), UVM_HIGH)
    end else begin
      current_remaining++;
    end

    slot_model[slot_addr].valid = 1'b1;
    slot_model[slot_addr].fp    = fp;
    if (!old_valid || old_key != fp.search_key) begin
      if (!current_remaining_by_key.exists(fp.search_key)) begin
        current_remaining_by_key[fp.search_key] = 0;
      end
      current_remaining_by_key[fp.search_key]++;
    end
    total_written++;
    update_peak_residency();
  endfunction

  function void write_out(ring_buffer_cam_pkg::out_seq_item item);
    int match_idx;

    if (item.is_subheader) begin
      if (epoch_active && active_seen_hits != active_expected_hits) begin
        total_subheader_mismatches++;
        `uvm_error("SCB", $sformatf(
          "New subheader arrived before previous epoch closed: key=%0d expected=%0d seen=%0d",
          active_search_key, active_expected_hits, active_seen_hits))
      end

      total_subheaders++;
      epoch_active = 1'b1;
      active_search_key = item.search_key;
      active_expected_hits = item.hit_count;
      active_seen_hits = 0;

      if (item.hit_count == 0 && item.eop) begin
        total_zero_hit_subheaders++;
        clear_epoch();
      end
      return;
    end

    total_drained++;
    if (!epoch_active) begin
      total_unexpected_outputs++;
      `uvm_error("SCB", $sformatf(
        "Observed hit with no active subheader context: key=%0d asic=%0d et1n6=%0d",
        item.active_search_key, item.asic, item.et1n6))
      return;
    end

    active_seen_hits++;
    if (item.cache_miss) begin
      total_cache_miss_outputs++;
      if (item.eop) begin
        if (active_seen_hits[7:0] != active_expected_hits[7:0]) begin
          total_subheader_mismatches++;
          `uvm_error("SCB", $sformatf(
            "Subheader count mismatch for key=%0d: field_expected=%0d seen=%0d seen_mod256=%0d remaining_for_key=%0d",
            active_search_key, active_expected_hits, active_seen_hits,
            active_seen_hits[7:0],
            remaining_entries_for_key(active_search_key)))
        end
        clear_epoch();
      end
      return;
    end

    match_idx = find_match_index(item);
    if (match_idx < 0) begin
      total_unexpected_outputs++;
      `uvm_error("SCB", $sformatf(
        "Unexpected drained hit: key=%0d asic=%0d channel=%0d ts50p=0x%0h et1n6=0x%0h",
        item.active_search_key, item.asic, item.channel, item.ts50p, item.et1n6))
    end else begin
      current_remaining--;
      if (!current_remaining_by_key.exists(slot_model[match_idx].fp.search_key)) begin
        current_remaining_by_key[slot_model[match_idx].fp.search_key] = 0;
      end
      current_remaining_by_key[slot_model[match_idx].fp.search_key]--;
      slot_model[match_idx].valid = 1'b0;
    end

    if (item.eop) begin
      if (active_seen_hits[7:0] != active_expected_hits[7:0]) begin
        total_subheader_mismatches++;
        `uvm_error("SCB", $sformatf(
          "Subheader count mismatch for key=%0d: field_expected=%0d seen=%0d seen_mod256=%0d remaining_for_key=%0d",
          active_search_key, active_expected_hits, active_seen_hits,
          active_seen_hits[7:0],
          remaining_entries_for_key(active_search_key)))
      end
      clear_epoch();
    end
  endfunction

  function void check_phase(uvm_phase phase);
    int unsigned remaining;
    super.check_phase(phase);
    remaining = remaining_entries();

    if (epoch_active) begin
      `uvm_error("SCB", $sformatf(
        "Active drain epoch still open at end of test: key=%0d expected=%0d seen=%0d",
        active_search_key, active_expected_hits, active_seen_hits))
    end

    if (remaining > 0) begin
      `uvm_error("SCB", $sformatf(
        "Undrained residents remain at end of test: accepted=%0d written=%0d drained=%0d remaining=%0d expected_overwrites=%0d unexpected_outputs=%0d",
        total_ingress_accepted, total_written, total_drained, remaining,
        total_expected_overwrites, total_unexpected_outputs))
    end
  endfunction

  function void report_phase(uvm_phase phase);
    `uvm_info("SCB", $sformatf(
      "Summary: pushed=%0d popped=%0d remaining=%0d overwrites=%0d unexpected=%0d subheaders=%0d zero_hit_subheaders=%0d accepted=%0d cache_miss_outputs=%0d",
      total_written, total_drained, remaining_entries(),
      total_expected_overwrites, total_unexpected_outputs,
      total_subheaders, total_zero_hit_subheaders, total_ingress_accepted,
      total_cache_miss_outputs), UVM_LOW)
    `uvm_info("SCB", $sformatf(
      "Pressure: max_remaining=%0d ring_depth=%0d",
      max_remaining_seen, m_cfg.ring_buffer_n_entry), UVM_LOW)
  endfunction

endclass

`endif

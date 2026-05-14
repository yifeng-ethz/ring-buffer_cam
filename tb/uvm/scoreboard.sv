// scoreboard.sv — Contract-derived resident model and egress checker

`ifndef SCOREBOARD_SV
`define SCOREBOARD_SV

import uvm_pkg::*;
`include "uvm_macros.svh"
`uvm_analysis_imp_decl(_out)
`uvm_analysis_imp_decl(_push)
`uvm_analysis_imp_decl(_pop)

class scoreboard extends uvm_scoreboard;
  `uvm_component_utils(scoreboard)

  typedef struct packed {
    bit [7:0] search_key;
    bit [7:0] subheader_key;
    bit [3:0] ts_3_0;
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
  uvm_analysis_imp_pop #(ring_buffer_cam_pkg::debug_pop_item, scoreboard) pop_imp;
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
  int unsigned total_packet_format_mismatches;
  int unsigned total_subframe_frames;
  int unsigned total_subframe_frame_mismatches;
  int unsigned current_remaining;
  bit          allow_nonempty_end;
  int unsigned allowed_remaining_at_end;
  int unsigned total_ingress_accepted_by_key[bit [7:0]];
  int unsigned total_written_by_key[bit [7:0]];
  int unsigned total_drained_by_key[bit [7:0]];
  int unsigned total_data_subheaders_by_key[bit [7:0]];
  int unsigned current_remaining_by_key[bit [7:0]];
  int unsigned max_remaining_seen;
  int unsigned max_remaining_seen_by_key[bit [7:0]];
  int unsigned overwrite_new_key_count[bit [7:0]];
  int unsigned overwrite_old_key_count[bit [7:0]];
  hit_fingerprint_t pending_drain_q[$];
  hit_fingerprint_t overlap_evicted_q[$];
  int unsigned total_overlap_fallback_hits;
  int unsigned total_pop_observations;
  bit          subframe_sequence_active;
  bit [7:0]    subframe_expected_key;
  int unsigned subframe_subheaders_seen;

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
    total_packet_format_mismatches = 0;
    total_subframe_frames = 0;
    total_subframe_frame_mismatches = 0;
    current_remaining = 0;
    allow_nonempty_end = 1'b0;
    allowed_remaining_at_end = 0;
    max_remaining_seen = 0;
    total_overlap_fallback_hits = 0;
    total_pop_observations = 0;
    subframe_sequence_active = 1'b0;
    subframe_expected_key = '0;
    subframe_subheaders_seen = 0;
    epoch_active = 1'b0;
    active_search_key = '0;
    active_expected_hits = 0;
    active_seen_hits = 0;
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    accept_imp = new("accept_imp", this);
    push_imp = new("push_imp", this);
    pop_imp = new("pop_imp", this);
    out_imp = new("out_imp", this);
    if (!uvm_config_db#(ring_buffer_cam_pkg::ring_buffer_cam_cfg)::get(this, "", "cfg", m_cfg))
      `uvm_fatal("SCB", "Failed to get cfg from config_db")
    slot_model = new[m_cfg.ring_buffer_n_entry];
  endfunction

  function hit_fingerprint_t item_to_fp(ring_buffer_cam_pkg::hit_seq_item item);
    hit_fingerprint_t fp;
    fp.search_key = m_cfg.search_key_from_tcc8n(item.tcc8n);
    fp.subheader_key = m_cfg.subheader_key_from_tcc8n(item.tcc8n);
    fp.ts_3_0     = item.tcc8n[3:0];
    fp.et1n6      = item.et1n6;
    fp.ts50p      = {item.tcc1n6, item.tfine};
    fp.asic       = item.asic;
    fp.channel    = item.channel;
    return fp;
  endfunction

  function bit output_matches_fp(ring_buffer_cam_pkg::out_seq_item item, hit_fingerprint_t fp);
    return
      fp.subheader_key == item.active_search_key &&
      fp.ts_3_0     == item.ts_3_0 &&
      fp.et1n6      == item.et1n6 &&
      fp.ts50p      == item.ts50p &&
      fp.asic       == item.asic &&
      fp.channel    == item.channel;
  endfunction

  function bit fp_matches(hit_fingerprint_t lhs, hit_fingerprint_t rhs);
    return
      lhs.search_key == rhs.search_key &&
      lhs.subheader_key == rhs.subheader_key &&
      lhs.ts_3_0     == rhs.ts_3_0 &&
      lhs.et1n6      == rhs.et1n6 &&
      lhs.ts50p      == rhs.ts50p &&
      lhs.asic       == rhs.asic &&
      lhs.channel    == rhs.channel;
  endfunction

  function int find_match_index(ring_buffer_cam_pkg::out_seq_item item);
    for (int i = 0; i < slot_model.size(); i++) begin
      if (slot_model[i].valid && output_matches_fp(item, slot_model[i].fp)) begin
        return i;
      end
    end
    return -1;
  endfunction

  function int find_model_fp_index(hit_fingerprint_t fp);
    for (int i = 0; i < slot_model.size(); i++) begin
      if (slot_model[i].valid && fp_matches(slot_model[i].fp, fp)) begin
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

  function int unsigned accepted_hits_for_key(bit [7:0] search_key);
    if (!total_ingress_accepted_by_key.exists(search_key)) begin
      return 0;
    end
    return total_ingress_accepted_by_key[search_key];
  endfunction

  function int unsigned written_hits_for_key(bit [7:0] search_key);
    if (!total_written_by_key.exists(search_key)) begin
      return 0;
    end
    return total_written_by_key[search_key];
  endfunction

  function int unsigned drained_hits_for_key(bit [7:0] search_key);
    if (!total_drained_by_key.exists(search_key)) begin
      return 0;
    end
    return total_drained_by_key[search_key];
  endfunction

  function int unsigned data_subheaders_for_key(bit [7:0] search_key);
    if (!total_data_subheaders_by_key.exists(search_key)) begin
      return 0;
    end
    return total_data_subheaders_by_key[search_key];
  endfunction

  function automatic bit is_data_subheader(ring_buffer_cam_pkg::out_seq_item item);
    return item.is_subheader && ((item.hit_count != 0) || !item.eop);
  endfunction

  function automatic int unsigned lane_subheaders_per_frame();
    if (m_cfg.interleaving_factor == 0) begin
      return 0;
    end
    return m_cfg.n_shd / m_cfg.interleaving_factor;
  endfunction

  function automatic bit [7:0] next_lane_search_key(bit [7:0] search_key);
    bit [8:0] next_key;

    next_key = {1'b0, search_key} + m_cfg.interleaving_factor[7:0];
    return next_key[7:0];
  endfunction

  function automatic bit [3:0] expected_avst_channel();
    return m_cfg.interleaving_index & 4'hf;
  endfunction

  function void note_subframe_subheader(ring_buffer_cam_pkg::out_seq_item item);
    int unsigned expected_count;

    expected_count = lane_subheaders_per_frame();
    if ($test$plusargs("RBCAM_TRACE_SUBFRAME")) begin
      `uvm_info("SCB_FRAME_TRACE", $sformatf(
        "subheader raw=0x%09h key=%0d hit_count=%0d sop=%0b eop=%0b expected_key=%0d seen_in_lane_frame=%0d completed_lane_frames=%0d n_shd=%0d",
        item.raw_data, item.search_key, item.hit_count, item.sop, item.eop,
        subframe_expected_key, subframe_subheaders_seen,
        total_subframe_frames, m_cfg.n_shd), UVM_LOW)
    end
    if (expected_count == 0) begin
      total_subframe_frame_mismatches++;
      `uvm_error("SCB_FRAME", $sformatf(
        "Illegal N_SHD/interleaving configuration: n_shd=%0d interleaving_factor=%0d",
        m_cfg.n_shd, m_cfg.interleaving_factor))
      return;
    end

    if ((item.search_key % m_cfg.interleaving_factor) != m_cfg.interleaving_index) begin
      total_subframe_frame_mismatches++;
      `uvm_error("SCB_FRAME", $sformatf(
        "Subheader key is not lane-matched: key=%0d interleaving_index=%0d interleaving_factor=%0d",
        item.search_key, m_cfg.interleaving_index, m_cfg.interleaving_factor))
    end

    if (!subframe_sequence_active) begin
      subframe_sequence_active = 1'b1;
      subframe_expected_key = item.search_key;
      subframe_subheaders_seen = 0;
    end

    if (item.search_key != subframe_expected_key) begin
      total_subframe_frame_mismatches++;
      `uvm_error("SCB_FRAME", $sformatf(
        "Subheader byte sequence mismatch: observed_key=%0d expected_key=%0d seen_in_lane_frame=%0d expected_lane_frame_count=%0d n_shd=%0d",
        item.search_key, subframe_expected_key, subframe_subheaders_seen,
        expected_count, m_cfg.n_shd))
      subframe_expected_key = item.search_key;
      subframe_subheaders_seen = 0;
    end

    subframe_subheaders_seen++;
    subframe_expected_key = next_lane_search_key(item.search_key);
    if (subframe_subheaders_seen == expected_count) begin
      total_subframe_frames++;
      subframe_subheaders_seen = 0;
    end else if (subframe_subheaders_seen > expected_count) begin
      total_subframe_frame_mismatches++;
      `uvm_error("SCB_FRAME", $sformatf(
        "Too many lane subheaders in frame: observed=%0d expected=%0d n_shd=%0d",
        subframe_subheaders_seen, expected_count, m_cfg.n_shd))
      subframe_subheaders_seen = 0;
    end
  endfunction

  function void note_packet_format_error(string msg);
    total_packet_format_mismatches++;
    `uvm_error("SCB_PKT", msg)
  endfunction

  function void check_subheader_packet_format(ring_buffer_cam_pkg::out_seq_item item);
    if (!item.sop) begin
      note_packet_format_error($sformatf(
        "Subheader missing SOP: raw=0x%09h key=%0d hit_count=%0d eop=%0b",
        item.raw_data, item.search_key, item.hit_count, item.eop));
    end
    if (item.raw_data[35:32] != 4'h1) begin
      note_packet_format_error($sformatf(
        "Subheader byte_is_k/type mismatch: raw=0x%09h observed=0x%0h expected=0x1",
        item.raw_data, item.raw_data[35:32]));
    end
    if (item.raw_data[23:16] != 8'h00) begin
      note_packet_format_error($sformatf(
        "Subheader reserved byte is nonzero: raw=0x%09h reserved=0x%02h",
        item.raw_data, item.raw_data[23:16]));
    end
    if (item.raw_data[7:0] != ring_buffer_cam_pkg::K237) begin
      note_packet_format_error($sformatf(
        "Subheader marker mismatch: raw=0x%09h marker=0x%02h expected=0x%02h",
        item.raw_data, item.raw_data[7:0], ring_buffer_cam_pkg::K237));
    end
    if (item.avst_channel != expected_avst_channel()) begin
      note_packet_format_error($sformatf(
        "Subheader AVST channel mismatch: raw=0x%09h channel=%0d expected=%0d",
        item.raw_data, item.avst_channel, expected_avst_channel()));
    end
    if (item.eop && item.hit_count != 0) begin
      note_packet_format_error($sformatf(
        "Subheader asserts EOP with nonzero hit_count: raw=0x%09h hit_count=%0d",
        item.raw_data, item.hit_count));
    end
  endfunction

  function void check_hit_packet_format(ring_buffer_cam_pkg::out_seq_item item);
    if (item.sop) begin
      note_packet_format_error($sformatf(
        "Data hit illegally asserts SOP: raw=0x%09h active_key=%0d",
        item.raw_data, item.active_search_key));
    end
    if (item.raw_data[35:32] != 4'h0) begin
      note_packet_format_error($sformatf(
        "Data hit byte_is_k/type mismatch: raw=0x%09h observed=0x%0h expected=0x0",
        item.raw_data, item.raw_data[35:32]));
    end
    if (item.raw_data[27:26] != 2'b00) begin
      note_packet_format_error($sformatf(
        "Data hit reserved bits [27:26] are nonzero: raw=0x%09h reserved=0x%0h",
        item.raw_data, item.raw_data[27:26]));
    end
    if (item.avst_channel != expected_avst_channel()) begin
      note_packet_format_error($sformatf(
        "Data hit AVST channel mismatch: raw=0x%09h channel=%0d expected=%0d",
        item.raw_data, item.avst_channel, expected_avst_channel()));
    end
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

  function int unsigned pending_drain_entries();
    return pending_drain_q.size();
  endfunction

  function int unsigned overlap_evicted_entries();
    return overlap_evicted_q.size();
  endfunction

  function bit epoch_idle();
    return !epoch_active;
  endfunction

  function int unsigned completed_lane_frames();
    return total_subframe_frames;
  endfunction

  function int unsigned active_lane_frame_subheaders();
    return subframe_subheaders_seen;
  endfunction

  function void note_flush_reset();
    for (int i = 0; i < slot_model.size(); i++) begin
      slot_model[i].valid = 1'b0;
    end
    total_ingress_accepted = 0;
    total_written = 0;
    total_drained = 0;
    total_expected_overwrites = 0;
    total_unexpected_outputs = 0;
    total_cache_miss_outputs = 0;
    total_subheaders = 0;
    total_zero_hit_subheaders = 0;
    total_subheader_mismatches = 0;
    total_packet_format_mismatches = 0;
    total_subframe_frames = 0;
    total_subframe_frame_mismatches = 0;
    max_remaining_seen = 0;
    current_remaining = 0;
    allow_nonempty_end = 1'b0;
    allowed_remaining_at_end = 0;
    total_ingress_accepted_by_key.delete();
    total_written_by_key.delete();
    total_drained_by_key.delete();
    total_data_subheaders_by_key.delete();
    current_remaining_by_key.delete();
    max_remaining_seen_by_key.delete();
    overwrite_new_key_count.delete();
    overwrite_old_key_count.delete();
    pending_drain_q.delete();
    overlap_evicted_q.delete();
    total_overlap_fallback_hits = 0;
    total_pop_observations = 0;
    subframe_sequence_active = 1'b0;
    subframe_expected_key = '0;
    subframe_subheaders_seen = 0;
    clear_epoch();
  endfunction

  function void note_intentional_nonempty_end(int unsigned expected_remaining);
    allow_nonempty_end = 1'b1;
    allowed_remaining_at_end = expected_remaining;
  endfunction

  function void clear_epoch();
    epoch_active = 1'b0;
    active_search_key = '0;
    active_expected_hits = 0;
    active_seen_hits = 0;
  endfunction

  function void write(ring_buffer_cam_pkg::hit_seq_item item);
    bit [7:0] key;
    if (item.is_empty_marker) begin
      return;
    end
    total_ingress_accepted++;
    key = m_cfg.search_key_from_tcc8n(item.tcc8n);
    if (!total_ingress_accepted_by_key.exists(key)) begin
      total_ingress_accepted_by_key[key] = 0;
    end
    total_ingress_accepted_by_key[key]++;
  endfunction

  function hit_fingerprint_t debug_item_to_fp(ring_buffer_cam_pkg::debug_push_item item);
    hit_fingerprint_t fp;
    fp.search_key = m_cfg.search_key_from_tcc8n(item.tcc8n());
    fp.subheader_key = m_cfg.subheader_key_from_tcc8n(item.tcc8n());
    fp.ts_3_0     = item.tcc8n()[3:0];
    fp.et1n6      = item.raw_hit[ring_buffer_cam_pkg::ET1N6_HI:ring_buffer_cam_pkg::ET1N6_LO];
    fp.ts50p      = item.raw_hit[ring_buffer_cam_pkg::TCC1N6_HI:ring_buffer_cam_pkg::TFINE_LO];
    fp.asic       = item.raw_hit[ring_buffer_cam_pkg::ASIC_HI:ring_buffer_cam_pkg::ASIC_LO];
    fp.channel    = item.raw_hit[ring_buffer_cam_pkg::CHANNEL_HI:ring_buffer_cam_pkg::CHANNEL_LO];
    return fp;
  endfunction

  function hit_fingerprint_t debug_pop_item_to_fp(ring_buffer_cam_pkg::debug_pop_item item);
    hit_fingerprint_t fp;
    fp.search_key = m_cfg.search_key_from_tcc8n(
      item.raw_hit[ring_buffer_cam_pkg::TCC8N_HI:ring_buffer_cam_pkg::TCC8N_LO]);
    fp.subheader_key = m_cfg.subheader_key_from_tcc8n(
      item.raw_hit[ring_buffer_cam_pkg::TCC8N_HI:ring_buffer_cam_pkg::TCC8N_LO]);
    fp.ts_3_0     = item.raw_hit[ring_buffer_cam_pkg::TCC8N_LO + 3:ring_buffer_cam_pkg::TCC8N_LO];
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
      overlap_evicted_q.push_back(slot_model[slot_addr].fp);
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
    if (!total_written_by_key.exists(fp.search_key)) begin
      total_written_by_key[fp.search_key] = 0;
    end
    total_written_by_key[fp.search_key]++;
    if (!old_valid || old_key != fp.search_key) begin
      if (!current_remaining_by_key.exists(fp.search_key)) begin
        current_remaining_by_key[fp.search_key] = 0;
      end
      current_remaining_by_key[fp.search_key]++;
    end
    total_written++;
    update_peak_residency();
  endfunction

  function int find_pending_match_index(ring_buffer_cam_pkg::out_seq_item item);
    foreach (pending_drain_q[idx]) begin
      if (output_matches_fp(item, pending_drain_q[idx])) begin
        return idx;
      end
    end
    return -1;
  endfunction

  function int find_overlap_match_index(ring_buffer_cam_pkg::out_seq_item item);
    foreach (overlap_evicted_q[idx]) begin
      if (output_matches_fp(item, overlap_evicted_q[idx])) begin
        return idx;
      end
    end
    return -1;
  endfunction

  function void write_pop(ring_buffer_cam_pkg::debug_pop_item item);
    int unsigned slot_addr;
    bit [7:0] key;
    hit_fingerprint_t pop_fp;
    int model_match_idx;

    slot_addr = item.slot_addr;
    if (slot_addr >= slot_model.size()) begin
      `uvm_error("SCB", $sformatf(
        "Observed pop_erase from slot %0d outside modeled ring size %0d",
        slot_addr, slot_model.size()))
      return;
    end

    pop_fp = debug_pop_item_to_fp(item);
    if (item.active_search_key != pop_fp.subheader_key) begin
      `uvm_error("SCB", $sformatf(
        "Pop-side subheader byte mismatch at slot %0d: pop_current_sk=%0d raw_hit_subheader=%0d cam_key=%0d occupied=%0b pop_count=%0d",
        slot_addr, item.active_search_key, pop_fp.subheader_key, pop_fp.search_key,
        item.occupied, item.pop_count))
    end
    model_match_idx = -1;
    if (slot_model[slot_addr].valid &&
        fp_matches(slot_model[slot_addr].fp, pop_fp)) begin
      model_match_idx = slot_addr;
    end else begin
      model_match_idx = find_model_fp_index(pop_fp);
    end

    if (!item.occupied) begin
      if (model_match_idx < 0) begin
        return;
      end
      `uvm_info("SCB", $sformatf(
        "Retiring cache-miss pop_erase from slot %0d using raw side-ram payload with occupied=0",
        slot_addr), UVM_LOW)
    end

    total_pop_observations++;

    if (model_match_idx < 0) begin
      pending_drain_q.push_back(pop_fp);
      `uvm_info("SCB", $sformatf(
        "Recovered pop_erase from slot %0d using raw side-ram payload without a matching live-slot fingerprint",
        slot_addr), UVM_LOW)
      return;
    end

    if (model_match_idx != slot_addr) begin
      `uvm_info("SCB", $sformatf(
        "Recovered pop_erase payload from slot %0d by matching live fingerprint in slot %0d",
        slot_addr, model_match_idx), UVM_LOW)
    end

    pending_drain_q.push_back(slot_model[model_match_idx].fp);
    key = slot_model[model_match_idx].fp.search_key;
    if (!current_remaining_by_key.exists(key)) begin
      current_remaining_by_key[key] = 0;
    end
    if (current_remaining > 0) begin
      current_remaining--;
    end
    if (current_remaining_by_key[key] > 0) begin
      current_remaining_by_key[key]--;
    end
    slot_model[model_match_idx].valid = 1'b0;
  endfunction

  function void write_out(ring_buffer_cam_pkg::out_seq_item item);
    int pending_match_idx;
    int live_match_idx;
    int overlap_match_idx;
    string pending_head_s;
    string overlap_head_s;

    if (item.is_subheader) begin
      if (epoch_active && active_seen_hits != active_expected_hits) begin
        total_subheader_mismatches++;
        `uvm_error("SCB", $sformatf(
          "New subheader arrived before previous epoch closed: key=%0d expected=%0d seen=%0d",
          active_search_key, active_expected_hits, active_seen_hits))
      end

      total_subheaders++;
      check_subheader_packet_format(item);
      if (m_cfg.enable_subframe_frame_check) begin
        note_subframe_subheader(item);
      end
      epoch_active = 1'b1;
      active_search_key = item.search_key;
      active_expected_hits = item.hit_count;
      active_seen_hits = 0;
      if (is_data_subheader(item)) begin
        if (!total_data_subheaders_by_key.exists(item.search_key)) begin
          total_data_subheaders_by_key[item.search_key] = 0;
        end
        total_data_subheaders_by_key[item.search_key]++;
      end

      if (item.hit_count == 0 && item.eop) begin
        total_zero_hit_subheaders++;
        clear_epoch();
      end
      return;
    end

    check_hit_packet_format(item);
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
      pending_match_idx = find_pending_match_index(item);
      if (pending_match_idx >= 0) begin
        pending_drain_q.delete(pending_match_idx);
      end
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

    if (!total_drained_by_key.exists(item.active_search_key)) begin
      total_drained_by_key[item.active_search_key] = 0;
    end
    total_drained_by_key[item.active_search_key]++;

    pending_match_idx = find_pending_match_index(item);
    live_match_idx = -1;
    overlap_match_idx = -1;

    if (pending_match_idx >= 0) begin
      pending_drain_q.delete(pending_match_idx);
    end else begin
      live_match_idx = find_match_index(item);
      if (live_match_idx < 0) begin
        overlap_match_idx = find_overlap_match_index(item);
        if (overlap_match_idx >= 0) begin
          overlap_evicted_q.delete(overlap_match_idx);
          total_overlap_fallback_hits++;
        end
      end
    end

    if (pending_match_idx < 0 && live_match_idx < 0 && overlap_match_idx < 0) begin
      if (pending_drain_q.size() > 0) begin
        pending_head_s = $sformatf(
          "head_pending={key=%0d asic=%0d channel=%0d ts50p=0x%0h et1n6=0x%0h}",
          pending_drain_q[0].search_key, pending_drain_q[0].asic,
          pending_drain_q[0].channel, pending_drain_q[0].ts50p,
          pending_drain_q[0].et1n6);
      end else begin
        pending_head_s = "head_pending=<none>";
      end
      if (overlap_evicted_q.size() > 0) begin
        overlap_head_s = $sformatf(
          "head_overlap={key=%0d asic=%0d channel=%0d ts50p=0x%0h et1n6=0x%0h}",
          overlap_evicted_q[0].search_key, overlap_evicted_q[0].asic,
          overlap_evicted_q[0].channel, overlap_evicted_q[0].ts50p,
          overlap_evicted_q[0].et1n6);
      end else begin
        overlap_head_s = "head_overlap=<none>";
      end
      total_unexpected_outputs++;
      `uvm_error("SCB", $sformatf(
        "Unexpected drained hit: key=%0d asic=%0d channel=%0d ts50p=0x%0h et1n6=0x%0h pending=%0d overlap=%0d remaining=%0d remaining_key=%0d accepted_key=%0d written_key=%0d drained_key=%0d pop_obs=%0d %s %s",
        item.active_search_key, item.asic, item.channel, item.ts50p, item.et1n6,
        pending_drain_entries(), overlap_evicted_entries(),
        remaining_entries(), remaining_entries_for_key(item.active_search_key),
        accepted_hits_for_key(item.active_search_key),
        written_hits_for_key(item.active_search_key),
        drained_hits_for_key(item.active_search_key),
        total_pop_observations,
        pending_head_s, overlap_head_s))
    end else if (live_match_idx >= 0) begin
      if (slot_model[live_match_idx].valid) begin
        if (current_remaining > 0) begin
          current_remaining--;
        end
        if (!current_remaining_by_key.exists(slot_model[live_match_idx].fp.search_key)) begin
          current_remaining_by_key[slot_model[live_match_idx].fp.search_key] = 0;
        end
        if (current_remaining_by_key[slot_model[live_match_idx].fp.search_key] > 0) begin
          current_remaining_by_key[slot_model[live_match_idx].fp.search_key]--;
        end
        slot_model[live_match_idx].valid = 1'b0;
      end
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
      if (allow_nonempty_end) begin
        if (remaining != allowed_remaining_at_end) begin
          `uvm_error("SCB", $sformatf(
            "Intentional non-empty end drifted: remaining=%0d expected=%0d accepted=%0d written=%0d drained=%0d expected_overwrites=%0d unexpected_outputs=%0d",
            remaining, allowed_remaining_at_end,
            total_ingress_accepted, total_written, total_drained,
            total_expected_overwrites, total_unexpected_outputs))
        end
      end else begin
        `uvm_error("SCB", $sformatf(
          "Undrained residents remain at end of test: accepted=%0d written=%0d drained=%0d remaining=%0d expected_overwrites=%0d unexpected_outputs=%0d",
          total_ingress_accepted, total_written, total_drained, remaining,
          total_expected_overwrites, total_unexpected_outputs))
      end
    end

    if (pending_drain_entries() > 0) begin
      `uvm_error("SCB", $sformatf(
        "Pending drain queue not empty at end of test: pending=%0d accepted=%0d written=%0d drained=%0d unexpected_outputs=%0d",
        pending_drain_entries(), total_ingress_accepted, total_written,
        total_drained, total_unexpected_outputs))
    end

    if (m_cfg.enable_subframe_frame_check &&
        total_subframe_frame_mismatches != 0) begin
      `uvm_error("SCB_FRAME", $sformatf(
        "Observed %0d subheader frame/sequence mismatches",
        total_subframe_frame_mismatches))
    end

    if (total_packet_format_mismatches != 0) begin
      `uvm_error("SCB_PKT", $sformatf(
        "Observed %0d packet-format mismatches",
        total_packet_format_mismatches))
    end

  endfunction

  function void report_phase(uvm_phase phase);
      `uvm_info("SCB", $sformatf(
      "Summary: pushed=%0d popped=%0d remaining=%0d pending_drain=%0d overlap_evicted=%0d overlap_fallback=%0d overwrites=%0d unexpected=%0d subheaders=%0d zero_hit_subheaders=%0d packet_format_mismatches=%0d completed_lane_frames=%0d active_lane_frame_subheaders=%0d accepted=%0d cache_miss_outputs=%0d",
      total_written, total_drained, remaining_entries(),
      pending_drain_entries(), overlap_evicted_entries(),
      total_overlap_fallback_hits,
      total_expected_overwrites, total_unexpected_outputs,
      total_subheaders, total_zero_hit_subheaders,
      total_packet_format_mismatches, total_subframe_frames,
      subframe_subheaders_seen,
      total_ingress_accepted,
      total_cache_miss_outputs), UVM_LOW)
    `uvm_info("SCB", $sformatf(
      "Pressure: max_remaining=%0d ring_depth=%0d",
      max_remaining_seen, m_cfg.ring_buffer_n_entry), UVM_LOW)
  endfunction

endclass

`endif

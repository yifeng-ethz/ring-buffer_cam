// hit_driver.sv — Drives Avalon-ST hit type-1 ingress interface

`ifndef HIT_DRIVER_SV
`define HIT_DRIVER_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class hit_driver extends uvm_driver #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_component_utils(hit_driver)

  virtual avst_hit_if.drv vif;
  virtual dut_debug_if.mon debug_vif;
  ring_buffer_cam_pkg::hit_seq_item pending_q[$];
  bit manual_override;

  int unsigned offered_total;
  int unsigned offered_payload_total;
  int unsigned offered_error_total;
  int unsigned offered_marker_total;
  int unsigned accepted_total;
  int unsigned accepted_payload_total;
  int unsigned accepted_error_total;
  int unsigned accepted_marker_total;
  int unsigned current_source_backlog;
  int unsigned max_source_backlog;
  int unsigned cycles_with_backlog;
  int unsigned ready_low_cycles;
  int unsigned current_ready_low_streak;
  int unsigned max_ready_low_streak;
  int unsigned offered_payload_at_first_pop;
  int unsigned accepted_payload_at_first_pop;
  int unsigned backlog_at_first_pop;

  longint unsigned sampled_cycles;
  longint unsigned first_offer_cycle;
  longint unsigned first_accept_cycle;
  longint unsigned first_backlog_cycle;
  longint unsigned first_ready_low_cycle;
  longint unsigned first_pop_cycle_seen;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual avst_hit_if.drv)::get(this, "", "vif", vif))
      `uvm_fatal("HIT_DRV", "Failed to get avst_hit_if.drv from config_db")
    void'(uvm_config_db#(virtual dut_debug_if.mon)::get(this, "", "debug_vif", debug_vif));
  endfunction

  function int unsigned pending_source_items();
    return pending_q.size();
  endfunction

  function bit head_pending_item_is_empty_marker();
    return (pending_q.size() > 0) && pending_q[0].is_empty_marker;
  endfunction

  function int unsigned pending_source_payload_items();
    int unsigned count;
    count = 0;
    foreach (pending_q[idx]) begin
      if (!pending_q[idx].is_empty_marker) begin
        count++;
      end
    end
    return count;
  endfunction

  function int unsigned pending_source_error_items();
    int unsigned count;
    count = 0;
    foreach (pending_q[idx]) begin
      if (!pending_q[idx].is_empty_marker && pending_q[idx].has_error) begin
        count++;
      end
    end
    return count;
  endfunction

  function int unsigned pending_source_marker_items();
    int unsigned count;
    count = 0;
    foreach (pending_q[idx]) begin
      if (pending_q[idx].is_empty_marker) begin
        count++;
      end
    end
    return count;
  endfunction

  function void discard_pending_items();
    pending_q.delete();
    current_source_backlog = 0;
    current_ready_low_streak = 0;
  endfunction

  function void reset_stats();
    pending_q.delete();
    manual_override = 1'b0;
    offered_total = 0;
    offered_payload_total = 0;
    offered_error_total = 0;
    offered_marker_total = 0;
    accepted_total = 0;
    accepted_payload_total = 0;
    accepted_error_total = 0;
    accepted_marker_total = 0;
    current_source_backlog = 0;
    max_source_backlog = 0;
    cycles_with_backlog = 0;
    ready_low_cycles = 0;
    current_ready_low_streak = 0;
    max_ready_low_streak = 0;
    offered_payload_at_first_pop = 0;
    accepted_payload_at_first_pop = 0;
    backlog_at_first_pop = 0;
    sampled_cycles = 0;
    first_offer_cycle = 0;
    first_accept_cycle = 0;
    first_backlog_cycle = 0;
    first_ready_low_cycle = 0;
    first_pop_cycle_seen = 0;
  endfunction

  function void note_offered(ring_buffer_cam_pkg::hit_seq_item item);
    offered_total++;
    if (item.is_empty_marker) begin
      offered_marker_total++;
    end else begin
      offered_payload_total++;
      if (item.has_error) begin
        offered_error_total++;
      end
    end
    if (first_offer_cycle == 0) begin
      first_offer_cycle = sampled_cycles;
    end
  endfunction

  function void copy_item_fields(
    ring_buffer_cam_pkg::hit_seq_item src,
    ring_buffer_cam_pkg::hit_seq_item dst
  );
    dst.asic = src.asic;
    dst.ingress_channel = src.ingress_channel;
    dst.channel = src.channel;
    dst.tcc8n = src.tcc8n;
    dst.tcc1n6 = src.tcc1n6;
    dst.tfine = src.tfine;
    dst.et1n6 = src.et1n6;
    dst.has_error = src.has_error;
    dst.is_empty_marker = src.is_empty_marker;
    dst.metadata = src.metadata;
    dst.metadata_valid = src.metadata_valid;
  endfunction

  function void note_accepted(ring_buffer_cam_pkg::hit_seq_item item);
    accepted_total++;
    if (item.is_empty_marker) begin
      accepted_marker_total++;
    end else begin
      accepted_payload_total++;
      if (item.has_error) begin
        accepted_error_total++;
      end
    end
    if (first_accept_cycle == 0) begin
      first_accept_cycle = sampled_cycles;
    end
  endfunction

  function void update_backlog_stats();
    current_source_backlog = pending_q.size();
    if (current_source_backlog > 0) begin
      cycles_with_backlog++;
      if (first_backlog_cycle == 0) begin
        first_backlog_cycle = sampled_cycles;
      end
      if (current_source_backlog > max_source_backlog) begin
        max_source_backlog = current_source_backlog;
      end
    end
  endfunction

  task automatic drive_outputs_for_next_cycle();
    if (pending_q.size() > 0) begin
      vif.data    <= pending_q[0].is_empty_marker ? '0 : pending_q[0].pack_hit();
      vif.valid   <= 1'b1;
      vif.channel <= pending_q[0].input_channel();
      vif.startofpacket <= pending_q[0].is_empty_marker;
      vif.endofpacket   <= pending_q[0].is_empty_marker;
      vif.empty   <= pending_q[0].is_empty_marker;
      vif.error   <= {pending_q[0].is_empty_marker ? 1'b0 : pending_q[0].has_error};
      vif.metadata <= pending_q[0].metadata;
      vif.metadata_valid <= pending_q[0].metadata_valid;
    end else begin
      vif.data    <= '0;
      vif.valid   <= 1'b0;
      vif.channel <= '0;
      vif.startofpacket <= 1'b0;
      vif.endofpacket   <= 1'b0;
      vif.empty   <= 1'b0;
      vif.error   <= 1'b0;
      vif.metadata <= '0;
      vif.metadata_valid <= 1'b0;
    end
  endtask

  task automatic enqueue_one_item_per_cycle();
    ring_buffer_cam_pkg::hit_seq_item item;
    ring_buffer_cam_pkg::hit_seq_item queued_item;

    item = null;
    seq_item_port.try_next_item(item);
    if (item == null) begin
      return;
    end

    queued_item = ring_buffer_cam_pkg::hit_seq_item::type_id::create("queued_hit");
    copy_item_fields(item, queued_item);
    pending_q.push_back(queued_item);
    note_offered(queued_item);
    seq_item_port.item_done();
  endtask

  task automatic pulse_manual_hit(
    ring_buffer_cam_pkg::hit_seq_item item,
    int unsigned hold_cycles = 1
  );
    @(negedge vif.clk);
    manual_override = 1'b1;
    vif.data    = item.is_empty_marker ? '0 : item.pack_hit();
    vif.valid   = 1'b1;
    vif.channel = item.input_channel();
    vif.startofpacket = item.is_empty_marker;
    vif.endofpacket   = item.is_empty_marker;
    vif.empty   = item.is_empty_marker;
    vif.error   = item.has_error;
    vif.metadata = item.metadata;
    vif.metadata_valid = item.metadata_valid;
    repeat (hold_cycles) @(posedge vif.clk);
    @(negedge vif.clk);
    manual_override = 1'b0;
    vif.data    = '0;
    vif.valid   = 1'b0;
    vif.channel = '0;
    vif.startofpacket = 1'b0;
    vif.endofpacket   = 1'b0;
    vif.empty   = 1'b0;
    vif.error   = 1'b0;
    vif.metadata = '0;
    vif.metadata_valid = 1'b0;
  endtask

  task automatic sample_first_pop_snapshot();
    if (debug_vif == null) begin
      return;
    end
    if (first_pop_cycle_seen != 0) begin
      return;
    end
    if (debug_vif.pop_erase_grant === 1'b1) begin
      first_pop_cycle_seen = sampled_cycles;
      offered_payload_at_first_pop = offered_payload_total;
      accepted_payload_at_first_pop = accepted_payload_total;
      backlog_at_first_pop = pending_q.size();
    end
  endtask

  task run_phase(uvm_phase phase);
    reset_stats();

    // Idle outputs
    vif.data    <= '0;
    vif.valid   <= 1'b0;
    vif.channel <= '0;
    vif.startofpacket <= 1'b0;
    vif.endofpacket   <= 1'b0;
    vif.empty   <= 1'b0;
    vif.error   <= 1'b0;
    vif.metadata <= '0;
    vif.metadata_valid <= 1'b0;

    forever begin
      @(posedge vif.clk);
      sampled_cycles++;

      if (manual_override) begin
        sample_first_pop_snapshot();
        update_backlog_stats();
        continue;
      end

      if (pending_q.size() > 0) begin
        if (vif.ready === 1'b1) begin
          note_accepted(pending_q[0]);
          void'(pending_q.pop_front());
          current_ready_low_streak = 0;
        end else begin
          ready_low_cycles++;
          current_ready_low_streak++;
          if (current_ready_low_streak > max_ready_low_streak) begin
            max_ready_low_streak = current_ready_low_streak;
          end
          if (first_ready_low_cycle == 0) begin
            first_ready_low_cycle = sampled_cycles;
          end
          if ((current_ready_low_streak % 50_000) == 0) begin
            if (debug_vif != null) begin
              `uvm_info("HIT_DRV", $sformatf(
                "Backpressured source on sk=%0d after %0d cycles: ready=%0d deassm_full=%0d deassm_usedw=%0d push=%0d pop=%0d overwrite=%0d offered_payload=%0d accepted_payload=%0d backlog=%0d",
                pending_q[0].search_key(), current_ready_low_streak, vif.ready,
                debug_vif.deassembly_fifo_full, debug_vif.deassembly_fifo_usedw,
                debug_vif.dbg_push_cnt[31:0], debug_vif.dbg_pop_cnt[31:0],
                debug_vif.dbg_overwrite_cnt[31:0],
                offered_payload_total, accepted_payload_total, pending_q.size()), UVM_LOW)
            end else begin
              `uvm_info("HIT_DRV", $sformatf(
                "Backpressured source on sk=%0d after %0d cycles: ready=%0d offered_payload=%0d accepted_payload=%0d backlog=%0d",
                pending_q[0].search_key(), current_ready_low_streak, vif.ready,
                offered_payload_total, accepted_payload_total, pending_q.size()), UVM_LOW)
            end
          end
        end
      end else begin
        current_ready_low_streak = 0;
      end

      sample_first_pop_snapshot();
      enqueue_one_item_per_cycle();
      update_backlog_stats();
      drive_outputs_for_next_cycle();
    end
  endtask

endclass

`endif

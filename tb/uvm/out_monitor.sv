// out_monitor.sv — Monitors egress Avalon-ST output and extracts hits

`ifndef OUT_MONITOR_SV
`define OUT_MONITOR_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class out_monitor extends uvm_monitor;
  `uvm_component_utils(out_monitor)

  virtual avst_out_if.mon vif;
  virtual dut_debug_if.mon dbg_vif;
  uvm_analysis_port #(ring_buffer_cam_pkg::out_seq_item) ap;

  // Throughput instrumentation
  int unsigned drain_cycle_count;
  int unsigned drain_hit_count;
  bit          drain_measuring;
  bit [7:0]    active_search_key;
  int unsigned active_hit_count_expected;
  int unsigned active_hit_index;
  int unsigned total_items_seen;
  int unsigned total_subheaders_seen;
  int unsigned total_hits_seen;
  ring_buffer_cam_pkg::out_seq_item recent_items[$];
  ring_buffer_cam_pkg::out_seq_item recent_subheaders[$];
  ring_buffer_cam_pkg::out_seq_item recent_hits[$];

  function new(string name, uvm_component parent);
    super.new(name, parent);
    drain_cycle_count = 0;
    drain_hit_count   = 0;
    drain_measuring   = 0;
    active_search_key = '0;
    active_hit_count_expected = 0;
    active_hit_index = 0;
    total_items_seen = 0;
    total_subheaders_seen = 0;
    total_hits_seen = 0;
  endfunction

  function void remember_item(ring_buffer_cam_pkg::out_seq_item item);
    ring_buffer_cam_pkg::out_seq_item clone;
    clone = ring_buffer_cam_pkg::out_seq_item::type_id::create("out_hist_item");
    clone.is_subheader       = item.is_subheader;
    clone.search_key         = item.search_key;
    clone.hit_count          = item.hit_count;
    clone.active_search_key  = item.active_search_key;
    clone.hit_index_in_epoch = item.hit_index_in_epoch;
    clone.hit_count_expected = item.hit_count_expected;
    clone.raw_data           = item.raw_data;
    clone.sop                = item.sop;
    clone.eop                = item.eop;
    clone.error              = item.error;
    clone.cache_miss         = item.cache_miss;
    clone.ts_3_0            = item.ts_3_0;
    clone.asic              = item.asic;
    clone.channel           = item.channel;
    clone.ts50p             = item.ts50p;
    clone.et1n6             = item.et1n6;
    recent_items.push_back(clone);
    if (recent_items.size() > 128)
      void'(recent_items.pop_front());
    if (item.is_subheader) begin
      recent_subheaders.push_back(clone);
      if (recent_subheaders.size() > 64)
        void'(recent_subheaders.pop_front());
    end else begin
      recent_hits.push_back(clone);
      if (recent_hits.size() > 128)
        void'(recent_hits.pop_front());
    end
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
    if (!uvm_config_db#(virtual avst_out_if.mon)::get(this, "", "vif", vif))
      `uvm_fatal("OUT_MON", "Failed to get avst_out_if.mon from config_db")
    if (!uvm_config_db#(virtual dut_debug_if.mon)::get(this, "", "debug_vif", dbg_vif))
      `uvm_fatal("OUT_MON", "Failed to get dut_debug_if.mon from config_db")
  endfunction

  task run_phase(uvm_phase phase);
    ring_buffer_cam_pkg::out_seq_item item;
    forever begin
      @(posedge vif.clk);

      if (drain_measuring)
        drain_cycle_count++;

      if (vif.valid === 1'b1) begin
        item = ring_buffer_cam_pkg::out_seq_item::type_id::create("out_item");
        item.raw_data = vif.data;
        item.sop      = vif.startofpacket;
        item.eop      = vif.endofpacket;
        item.error    = vif.error[0];
        item.cache_miss = 1'b0;

        if (vif.data[35:32] == 4'b0001) begin
          // Subheader
          item.is_subheader = 1;
          item.search_key   = vif.data[31:24];
          item.hit_count    = vif.data[15:8];
          active_search_key = item.search_key;
          active_hit_count_expected = item.hit_count;
          active_hit_index = 0;
          if (!drain_measuring && item.hit_count > 0) begin
            drain_measuring   = 1;
            drain_cycle_count = 0;
            drain_hit_count   = 0;
          end
          total_subheaders_seen++;
        end else begin
          // Hit data
          item.is_subheader = 0;
          item.cache_miss = (dbg_vif.pop_cache_miss_pulse === 1'b1);
          item.ts_3_0  = vif.data[31:28];
          item.asic    = vif.data[25:22];
          item.channel = vif.data[21:17];
          item.ts50p   = vif.data[16:9];
          item.et1n6   = vif.data[8:0];
          item.active_search_key = active_search_key;
          item.hit_count_expected = active_hit_count_expected;
          item.hit_index_in_epoch = active_hit_index;
          active_hit_index++;
          if (drain_measuring)
            drain_hit_count++;
          if (item.eop && drain_measuring) begin
            drain_measuring = 0;
            `uvm_info("OUT_MON", $sformatf(
              "Throughput: %0d hits in %0d cycles (%.1f%%)",
              drain_hit_count, drain_cycle_count,
              100.0 * drain_hit_count / drain_cycle_count), UVM_MEDIUM)
          end
          total_hits_seen++;
        end

        total_items_seen++;
        remember_item(item);
        ap.write(item);
      end
    end
  endtask

endclass

`endif

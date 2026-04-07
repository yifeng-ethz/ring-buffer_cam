// out_monitor.sv — Monitors egress Avalon-ST output and extracts hits

`ifndef OUT_MONITOR_SV
`define OUT_MONITOR_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class out_monitor extends uvm_monitor;
  `uvm_component_utils(out_monitor)

  virtual avst_out_if.mon vif;
  uvm_analysis_port #(ring_buffer_cam_pkg::out_seq_item) ap;

  // Throughput instrumentation
  int unsigned drain_cycle_count;
  int unsigned drain_hit_count;
  bit          drain_measuring;

  function new(string name, uvm_component parent);
    super.new(name, parent);
    drain_cycle_count = 0;
    drain_hit_count   = 0;
    drain_measuring   = 0;
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
    if (!uvm_config_db#(virtual avst_out_if.mon)::get(this, "", "vif", vif))
      `uvm_fatal("OUT_MON", "Failed to get avst_out_if.mon from config_db")
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

        if (vif.data[35:32] == 4'b0001) begin
          // Subheader
          item.is_subheader = 1;
          item.search_key   = vif.data[31:24];
          item.hit_count    = vif.data[15:8];
          if (!drain_measuring && item.hit_count > 0) begin
            drain_measuring   = 1;
            drain_cycle_count = 0;
            drain_hit_count   = 0;
          end
        end else begin
          // Hit data
          item.is_subheader = 0;
          item.ts_3_0  = vif.data[31:28];
          item.asic    = vif.data[25:22];
          item.channel = vif.data[21:17];
          item.ts50p   = vif.data[16:9];
          item.et1n6   = vif.data[8:0];
          if (drain_measuring)
            drain_hit_count++;
          if (item.eop && drain_measuring) begin
            drain_measuring = 0;
            `uvm_info("OUT_MON", $sformatf(
              "Throughput: %0d hits in %0d cycles (%.1f%%)",
              drain_hit_count, drain_cycle_count,
              100.0 * drain_hit_count / drain_cycle_count), UVM_MEDIUM)
          end
        end

        ap.write(item);
      end
    end
  endtask

endclass

`endif

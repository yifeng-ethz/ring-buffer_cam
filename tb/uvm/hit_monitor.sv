// hit_monitor.sv — Monitors ingress hits for scoreboard reference

`ifndef HIT_MONITOR_SV
`define HIT_MONITOR_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class hit_monitor extends uvm_monitor;
  `uvm_component_utils(hit_monitor)

  virtual avst_hit_if.mon vif;
  uvm_analysis_port #(ring_buffer_cam_pkg::hit_seq_item) ap;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
    if (!uvm_config_db#(virtual avst_hit_if.mon)::get(this, "", "vif", vif))
      `uvm_fatal("HIT_MON", "Failed to get avst_hit_if.mon from config_db")
  endfunction

  task run_phase(uvm_phase phase);
    ring_buffer_cam_pkg::hit_seq_item item;
    forever begin
      @(posedge vif.clk);
      if (vif.valid === 1'b1 && vif.ready === 1'b1) begin
        item = ring_buffer_cam_pkg::hit_seq_item::type_id::create("hit_item");
        item.asic    = vif.data[38:35];
        item.channel = vif.data[34:30];
        item.tcc8n   = vif.data[29:17];
        item.tcc1n6  = vif.data[16:14];
        item.tfine   = vif.data[13:9];
        item.et1n6   = vif.data[8:0];
        item.has_error = vif.error[0];
        ap.write(item);
      end
    end
  endtask

endclass

`endif

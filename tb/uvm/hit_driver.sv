// hit_driver.sv — Drives Avalon-ST hit type-1 ingress interface

`ifndef HIT_DRIVER_SV
`define HIT_DRIVER_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class hit_driver extends uvm_driver #(ring_buffer_cam_pkg::hit_seq_item);
  `uvm_component_utils(hit_driver)

  virtual avst_hit_if.drv vif;
  virtual dut_debug_if.mon debug_vif;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual avst_hit_if.drv)::get(this, "", "vif", vif))
      `uvm_fatal("HIT_DRV", "Failed to get avst_hit_if.drv from config_db")
    void'(uvm_config_db#(virtual dut_debug_if.mon)::get(this, "", "debug_vif", debug_vif));
  endfunction

  task run_phase(uvm_phase phase);
    ring_buffer_cam_pkg::hit_seq_item item;
    // Idle outputs
    vif.data    <= '0;
    vif.valid   <= 1'b0;
    vif.channel <= '0;
    vif.startofpacket <= 1'b0;
    vif.endofpacket   <= 1'b0;
    vif.empty   <= 1'b0;
    vif.error   <= 1'b0;

    forever begin
      seq_item_port.get_next_item(item);
      drive_hit(item);
      seq_item_port.item_done();
    end
  endtask

  task drive_hit(ring_buffer_cam_pkg::hit_seq_item item);
    int unsigned stall_cycles;
    vif.data    <= item.is_empty_marker ? '0 : item.pack_hit();
    vif.valid   <= 1'b1;
    vif.channel <= item.input_channel();
    vif.empty   <= item.is_empty_marker;
    vif.error   <= {item.is_empty_marker ? 1'b0 : item.has_error};

    // Wait for ready handshake
    stall_cycles = 0;
    do begin
      @(posedge vif.clk);
      stall_cycles++;
      if ((stall_cycles % 50_000) == 0) begin
        if (debug_vif != null) begin
          `uvm_info("HIT_DRV", $sformatf(
            "Waiting for ready on hit sk=%0d after %0d cycles: ready=%0d deassm_full=%0d deassm_usedw=%0d push=%0d pop=%0d overwrite=%0d fill=%0d",
            item.search_key(), stall_cycles, vif.ready,
            debug_vif.deassembly_fifo_full, debug_vif.deassembly_fifo_usedw,
            debug_vif.dbg_push_cnt[31:0], debug_vif.dbg_pop_cnt[31:0],
            debug_vif.dbg_overwrite_cnt[31:0],
            debug_vif.dbg_push_cnt[31:0] - debug_vif.dbg_pop_cnt[31:0] - debug_vif.dbg_overwrite_cnt[31:0]), UVM_LOW)
        end else begin
          `uvm_info("HIT_DRV", $sformatf(
            "Waiting for ready on hit sk=%0d after %0d cycles: ready=%0d",
            item.search_key(), stall_cycles, vif.ready), UVM_LOW)
        end
      end
    end while (vif.ready !== 1'b1);

    // De-assert after accepted
    vif.valid   <= 1'b0;
    vif.data    <= '0;
    vif.channel <= '0;
    vif.empty   <= 1'b0;
    vif.error   <= 1'b0;
  endtask

endclass

`endif

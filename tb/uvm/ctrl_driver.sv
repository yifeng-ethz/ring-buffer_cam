// ctrl_driver.sv — Drives Avalon-ST run-control interface

`ifndef CTRL_DRIVER_SV
`define CTRL_DRIVER_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class ctrl_driver extends uvm_driver #(ring_buffer_cam_pkg::ctrl_seq_item);
  `uvm_component_utils(ctrl_driver)

  virtual avst_ctrl_if.drv vif;
  virtual dut_debug_if.mon debug_vif;
  ring_buffer_cam_pkg::ring_buffer_cam_cfg m_cfg;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual avst_ctrl_if.drv)::get(this, "", "vif", vif))
      `uvm_fatal("CTRL_DRV", "Failed to get avst_ctrl_if.drv from config_db")
    void'(uvm_config_db#(virtual dut_debug_if.mon)::get(this, "", "debug_vif", debug_vif));
    if (!uvm_config_db#(ring_buffer_cam_pkg::ring_buffer_cam_cfg)::get(this, "", "cfg", m_cfg))
      `uvm_fatal("CTRL_DRV", "Failed to get cfg from config_db")
  endfunction

  task run_phase(uvm_phase phase);
    ring_buffer_cam_pkg::ctrl_seq_item item;
    vif.data  <= '0;
    vif.valid <= 1'b0;

    forever begin
      seq_item_port.get_next_item(item);
      drive_ctrl(item);
      seq_item_port.item_done();
    end
  endtask

  task drive_ctrl(ring_buffer_cam_pkg::ctrl_seq_item item);
    int unsigned wait_cycles;
    vif.data  <= item.cmd;
    vif.valid <= 1'b1;
    if (item.cmd == ring_buffer_cam_pkg::CTRL_RUN_PREPARE &&
        m_cfg.run_state != ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE) begin
      // Like TERMINATING, the DUT latches the state transition from valid
      // alone. Pulse the first RUN_PREPARE command to enter the state, then
      // use the second RUN_PREPARE command to wait for flush completion.
      @(posedge vif.clk);
      m_cfg.note_ctrl_cmd(item.cmd);
      vif.valid <= 1'b0;
      vif.data  <= '0;
      @(posedge vif.clk);
      return;
    end
    if (item.cmd == ring_buffer_cam_pkg::CTRL_TERMINATING &&
        m_cfg.run_state != ring_buffer_cam_pkg::RUN_STATE_TERMINATING) begin
      // The DUT samples the state transition on valid alone and only raises
      // ready in TERMINATING after drain-complete. Pulse the entry command so
      // the test can inject the end-of-run marker, then use a second
      // TERMINATING command when it actually wants to wait on drain-done.
      @(posedge vif.clk);
      m_cfg.note_ctrl_cmd(item.cmd);
      vif.valid <= 1'b0;
      vif.data  <= '0;
      @(posedge vif.clk);
      return;
    end
    wait_cycles = 0;
    do begin
      @(posedge vif.clk);
      wait_cycles++;
      if ((wait_cycles % 20_000) == 0) begin
        if (debug_vif != null) begin
          `uvm_info("CTRL_DRV", $sformatf(
            "Waiting for ctrl ready cmd=0x%0h after %0d cycles: ready=%0d endofrun_seen=%0d term_done=%0d deassm_empty=%0d deassm_usedw=%0d pop_cmd_empty=%0d pop_cmd_usedw=%0d push=%0d pop=%0d overwrite=%0d",
            item.cmd, wait_cycles, vif.ready,
            debug_vif.endofrun_seen, debug_vif.terminating_drain_done,
            debug_vif.deassembly_fifo_empty, debug_vif.deassembly_fifo_usedw,
            debug_vif.pop_cmd_fifo_empty, debug_vif.pop_cmd_fifo_usedw,
            debug_vif.dbg_push_cnt[31:0], debug_vif.dbg_pop_cnt[31:0],
            debug_vif.dbg_overwrite_cnt[31:0]), UVM_LOW)
        end else begin
          `uvm_info("CTRL_DRV", $sformatf(
            "Waiting for ctrl ready cmd=0x%0h after %0d cycles: ready=%0d",
            item.cmd, wait_cycles, vif.ready), UVM_LOW)
        end
      end
    end while (vif.ready !== 1'b1);
    m_cfg.note_ctrl_cmd(item.cmd);
    vif.valid <= 1'b0;
    vif.data  <= '0;
    @(posedge vif.clk);
  endtask

endclass

`endif

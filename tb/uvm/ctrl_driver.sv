// ctrl_driver.sv — Drives Avalon-ST run-control interface

`ifndef CTRL_DRIVER_SV
`define CTRL_DRIVER_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class ctrl_driver extends uvm_driver #(ring_buffer_cam_pkg::ctrl_seq_item);
  `uvm_component_utils(ctrl_driver)

  virtual avst_ctrl_if.drv vif;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual avst_ctrl_if.drv)::get(this, "", "vif", vif))
      `uvm_fatal("CTRL_DRV", "Failed to get avst_ctrl_if.drv from config_db")
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
    vif.data  <= item.cmd;
    vif.valid <= 1'b1;
    do begin
      @(posedge vif.clk);
    end while (vif.ready !== 1'b1);
    vif.valid <= 1'b0;
    vif.data  <= '0;
    @(posedge vif.clk);
  endtask

endclass

`endif

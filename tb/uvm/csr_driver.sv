// csr_driver.sv — Drives Avalon-MM CSR interface

`ifndef CSR_DRIVER_SV
`define CSR_DRIVER_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class csr_driver extends uvm_driver #(ring_buffer_cam_pkg::csr_seq_item);
  `uvm_component_utils(csr_driver)

  virtual avmm_csr_if.drv vif;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual avmm_csr_if.drv)::get(this, "", "vif", vif))
      `uvm_fatal("CSR_DRV", "Failed to get avmm_csr_if.drv from config_db")
  endfunction

  task run_phase(uvm_phase phase);
    ring_buffer_cam_pkg::csr_seq_item item;
    vif.address   <= '0;
    vif.read      <= 1'b0;
    vif.write     <= 1'b0;
    vif.writedata <= '0;

    forever begin
      seq_item_port.get_next_item(item);
      if (item.is_write)
        drive_write(item);
      else
        drive_read(item);
      seq_item_port.item_done();
    end
  endtask

  task drive_write(ring_buffer_cam_pkg::csr_seq_item item);
    vif.address   <= item.address;
    vif.writedata <= item.writedata;
    vif.write     <= 1'b1;
    do begin
      @(posedge vif.clk);
    end while (vif.waitrequest === 1'b1);
    vif.write     <= 1'b0;
    vif.address   <= '0;
    vif.writedata <= '0;
    @(posedge vif.clk);
  endtask

  task drive_read(ring_buffer_cam_pkg::csr_seq_item item);
    vif.address <= item.address;
    vif.read    <= 1'b1;
    do begin
      @(posedge vif.clk);
    end while (vif.waitrequest === 1'b1);
    #1step;
    item.readdata = vif.readdata;
    vif.read    <= 1'b0;
    vif.address <= '0;
    @(posedge vif.clk);
  endtask

endclass

`endif

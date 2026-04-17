// env.sv — UVM environment assembling all agents, scoreboard, and coverage

`ifndef ENV_SV
`define ENV_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class ring_buffer_cam_env extends uvm_env;
  `uvm_component_utils(ring_buffer_cam_env)

  uvm_sequencer #(ring_buffer_cam_pkg::hit_seq_item)  m_hit_seqr;
  uvm_sequencer #(ring_buffer_cam_pkg::ctrl_seq_item) m_ctrl_seqr;
  uvm_sequencer #(ring_buffer_cam_pkg::csr_seq_item)  m_csr_seqr;
  hit_driver                  m_hit_drv;
  hit_monitor                 m_hit_mon;
  ctrl_driver                 m_ctrl_drv;
  csr_driver                  m_csr_drv;
  out_monitor                 m_out_mon;
  debug_monitor               m_dbg_mon;
  scoreboard                  m_scb;
  ring_buffer_cam_coverage    m_cov;

  ring_buffer_cam_pkg::ring_buffer_cam_cfg m_cfg;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);

    if (!uvm_config_db#(ring_buffer_cam_pkg::ring_buffer_cam_cfg)::get(
          this, "", "cfg", m_cfg))
    begin
      m_cfg = ring_buffer_cam_pkg::ring_buffer_cam_cfg::type_id::create("cfg");
      `uvm_info("ENV", "Using default configuration", UVM_MEDIUM)
    end
    uvm_config_db#(ring_buffer_cam_pkg::ring_buffer_cam_cfg)::set(this, "*", "cfg", m_cfg);

    m_hit_seqr = uvm_sequencer#(ring_buffer_cam_pkg::hit_seq_item)::type_id::create("m_hit_seqr", this);
    m_ctrl_seqr = uvm_sequencer#(ring_buffer_cam_pkg::ctrl_seq_item)::type_id::create("m_ctrl_seqr", this);
    m_csr_seqr = uvm_sequencer#(ring_buffer_cam_pkg::csr_seq_item)::type_id::create("m_csr_seqr", this);
    m_hit_drv  = hit_driver::type_id::create("m_hit_drv", this);
    m_hit_mon  = hit_monitor::type_id::create("m_hit_mon", this);
    m_ctrl_drv = ctrl_driver::type_id::create("m_ctrl_drv", this);
    m_csr_drv  = csr_driver::type_id::create("m_csr_drv", this);
    m_out_mon  = out_monitor::type_id::create("m_out_mon", this);
    m_dbg_mon  = debug_monitor::type_id::create("m_dbg_mon", this);
    m_scb      = scoreboard::type_id::create("m_scb", this);
    m_cov      = ring_buffer_cam_coverage::type_id::create("m_cov", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    m_hit_drv.seq_item_port.connect(m_hit_seqr.seq_item_export);
    m_ctrl_drv.seq_item_port.connect(m_ctrl_seqr.seq_item_export);
    m_csr_drv.seq_item_port.connect(m_csr_seqr.seq_item_export);
    m_hit_mon.ap.connect(m_scb.accept_imp);
    m_dbg_mon.push_ap.connect(m_scb.push_imp);
    m_dbg_mon.pop_ap.connect(m_scb.pop_imp);
    m_out_mon.ap.connect(m_scb.out_imp);
    m_out_mon.ap.connect(m_cov.analysis_export);
  endfunction

endclass

`endif

// coverage.sv — Functional coverage collector for ring_buffer_cam

`ifndef COVERAGE_SV
`define COVERAGE_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class ring_buffer_cam_coverage extends uvm_subscriber #(ring_buffer_cam_pkg::out_seq_item);
  `uvm_component_utils(ring_buffer_cam_coverage)

  ring_buffer_cam_pkg::out_seq_item txn;

  // ── Coverage groups ────────────────────────────────────────────

  covergroup cg_output;
    cp_is_subheader : coverpoint txn.is_subheader {
      bins subheader = {1};
      bins hit_data  = {0};
    }

    cp_sop : coverpoint txn.sop {
      bins asserted    = {1};
      bins deasserted  = {0};
    }

    cp_eop : coverpoint txn.eop {
      bins asserted    = {1};
      bins deasserted  = {0};
    }

    cp_error : coverpoint txn.error {
      bins no_error = {0};
      bins error    = {1};
    }

    cp_hit_count : coverpoint txn.hit_count iff (txn.is_subheader) {
      bins zero_count   = {0};
      bins single_count = {1};
      bins small_count  = {[2:8]};
      bins medium_count = {[9:64]};
      bins large_count  = {[65:255]};
    }

    cp_search_key : coverpoint txn.search_key iff (txn.is_subheader) {
      bins keys[16] = {[0:255]};  // 16 bins spanning full range
    }

    // Cross: subheader with SOP
    cx_subhdr_sop : cross cp_is_subheader, cp_sop {
      bins valid_subhdr = binsof(cp_is_subheader.subheader) && binsof(cp_sop.asserted);
    }

    // Cross: last hit with EOP
    cx_hit_eop : cross cp_is_subheader, cp_eop {
      bins last_hit = binsof(cp_is_subheader.hit_data) && binsof(cp_eop.asserted);
    }
  endgroup

  covergroup cg_hit_data;
    cp_asic : coverpoint txn.asic iff (!txn.is_subheader) {
      bins asics[16] = {[0:15]};
    }

    cp_channel : coverpoint txn.channel iff (!txn.is_subheader) {
      bins channels[8] = {[0:31]};
    }

    cp_ts_3_0 : coverpoint txn.ts_3_0 iff (!txn.is_subheader) {
      bins ts_bins[16] = {[0:15]};
    }
  endgroup

  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg_output   = new();
    cg_hit_data = new();
  endfunction

  function void write(ring_buffer_cam_pkg::out_seq_item t);
    txn = t;
    cg_output.sample();
    if (!t.is_subheader)
      cg_hit_data.sample();
  endfunction

  function void report_phase(uvm_phase phase);
    `uvm_info("COV", $sformatf(
      "cg_output=%.1f%% cg_hit_data=%.1f%%",
      cg_output.get_coverage(), cg_hit_data.get_coverage()), UVM_LOW)
  endfunction

endclass

`endif

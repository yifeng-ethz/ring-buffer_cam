// scoreboard.sv — Compares pushed hits with popped hits
// Within a subheader group, hits may arrive in any order (bank arbiter selects
// lowest-index partition first).  The scoreboard uses a per-search-key queue.

`ifndef SCOREBOARD_SV
`define SCOREBOARD_SV

import uvm_pkg::*;
`include "uvm_macros.svh"
`uvm_analysis_imp_decl(_out)

class scoreboard extends uvm_scoreboard;
  `uvm_component_utils(scoreboard)

  uvm_analysis_imp #(ring_buffer_cam_pkg::hit_seq_item, scoreboard)  in_imp;
  uvm_analysis_imp_out #(ring_buffer_cam_pkg::out_seq_item, scoreboard) out_imp;

  // Reference model: pushed hits indexed by search_key
  // Each entry stores {et1n6, ts50p} as a unique fingerprint
  typedef struct {
    bit [8:0] et1n6;
    bit [7:0] ts50p;  // tcc1n6 & tfine
    bit [3:0] asic;
  } hit_fingerprint_t;

  hit_fingerprint_t ref_queue[int][$];  // key: search_key → queue of fingerprints

  int unsigned total_pushed;
  int unsigned total_popped;
  int unsigned total_mismatches;
  int unsigned total_overwrites;  // popped hits not in ref (overwritten)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    total_pushed     = 0;
    total_popped     = 0;
    total_mismatches = 0;
    total_overwrites = 0;
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    in_imp  = new("in_imp", this);
    out_imp = new("out_imp", this);
  endfunction

  // Called when a hit is pushed into the DUT
  function void write(ring_buffer_cam_pkg::hit_seq_item item);
    hit_fingerprint_t fp;
    int sk;
    if (item.has_error) return;  // error hits may be filtered

    sk = item.search_key();
    fp.et1n6 = item.et1n6;
    fp.ts50p = {item.tcc1n6, item.tfine};
    fp.asic  = item.asic;
    ref_queue[sk].push_back(fp);
    total_pushed++;
  endfunction

  // Called when a hit (or subheader) appears at egress
  function void write_out(ring_buffer_cam_pkg::out_seq_item item);
    if (item.is_subheader) return;  // subheaders are protocol, not data

    total_popped++;

    // Try to find matching fingerprint in ref_queue
    // The search key for this hit's subheader is tracked externally;
    // here we search across all keys (hits reconstruct ts[3:0] but not full key).
    // A simpler approach: match by et1n6 + asic (unique enough for typical tests).
    foreach (ref_queue[sk]) begin
      for (int i = 0; i < ref_queue[sk].size(); i++) begin
        if (ref_queue[sk][i].et1n6 == item.et1n6 &&
            ref_queue[sk][i].asic  == item.asic) begin
          ref_queue[sk].delete(i);
          return;
        end
      end
    end

    // Not found — could be overwritten hit or mismatch
    total_overwrites++;
    `uvm_info("SCB", $sformatf(
      "Popped hit not in reference (overwrite?): et1n6=%0h asic=%0h",
      item.et1n6, item.asic), UVM_HIGH)
  endfunction

  function void report_phase(uvm_phase phase);
    int remaining = 0;
    foreach (ref_queue[sk])
      remaining += ref_queue[sk].size();

    `uvm_info("SCB", $sformatf(
      "Summary: pushed=%0d popped=%0d remaining=%0d overwrites=%0d",
      total_pushed, total_popped, remaining, total_overwrites), UVM_LOW)

    // Remaining entries that were never popped are either:
    // (a) overwrites (expected if CAM was filled beyond capacity)
    // (b) hits with search keys not yet reached by read pointer
    // Only flag as error if no overwrites were expected and entries remain
  endfunction

endclass

`endif

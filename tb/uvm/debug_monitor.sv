// debug_monitor.sv — Samples internal DUT debug taps for scoreboard and case checks

`ifndef DEBUG_MONITOR_SV
`define DEBUG_MONITOR_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class debug_monitor extends uvm_monitor;
  `uvm_component_utils(debug_monitor)

  virtual dut_debug_if.mon vif;
  uvm_analysis_port #(ring_buffer_cam_pkg::debug_push_item) push_ap;

  int unsigned pop_cmd_wrreq_count;
  int unsigned pop_cmd_rdack_count;
  int unsigned push_write_grant_count;
  int unsigned push_erase_grant_count;
  int unsigned pop_erase_grant_count;
  int unsigned pop_flush_grant_count;
  int unsigned cache_miss_pulse_count;
  int unsigned current_live_fill;
  int unsigned max_live_fill;
  int unsigned push_count_at_first_pop;
  int unsigned overwrite_count_at_first_pop;
  int unsigned live_fill_at_first_pop;

  longint unsigned sampled_cycles;
  longint unsigned first_push_cycle;
  longint unsigned first_pop_cycle;
  longint unsigned first_overwrite_cycle;
  longint unsigned last_push_cycle;
  longint unsigned last_pop_cycle;
  longint unsigned last_overwrite_cycle;

  logic [47:0] dbg_inerr_cnt;
  logic [47:0] dbg_push_cnt;
  logic [47:0] dbg_pop_cnt;
  logic [47:0] dbg_overwrite_cnt;
  logic [47:0] dbg_cache_miss_cnt;
  logic [3:0]  run_state_code;
  logic [2:0]  pop_engine_state_code;
  logic        push_state_code;
  logic        endofrun_seen;
  logic        run_mgmt_flushed;
  logic        gts_counter_rst;
  logic        run_mgmt_flush_memory_start;
  logic        run_mgmt_flush_memory_done;
  logic        pop_flush_ram_done;
  logic        pop_flush_cam_done;
  logic        cam_clean;
  logic [15:0] pop_hits_count;
  logic [1:0]  pop_rr_idx;
  logic [1:0]  pop_issue_partition_idx;
  logic [1:0]  pop_count_partition_idx;
  logic [2:0]  pop_search_wait_cnt;
  logic [3:0]  pop_partition_pending;
  logic [3:0]  pop_partition_load;
  logic [3:0]  pop_partition_advance;
  logic [3:0]  pop_partition_result_valid;
  logic [3:0]  pop_partition_flag;
  logic [3:0]  pop_partition_has_more;
  logic        pop_last_hit_pending;
  logic        pop_cmd_fifo_sclr;
  logic        deassembly_fifo_sclr;
  logic [47:0] expected_latency_48b;
  logic [47:0] read_time_ptr;
  logic [47:0] gts_8n;
  logic [47:0] gts_end_of_run;
  logic [3:0]  pop_cmd_fifo_usedw;
  logic [7:0]  deassembly_fifo_usedw;
  logic        terminating_drain_done;

  function new(string name, uvm_component parent);
    super.new(name, parent);
    pop_cmd_wrreq_count = 0;
    pop_cmd_rdack_count = 0;
    push_write_grant_count = 0;
    push_erase_grant_count = 0;
    pop_erase_grant_count = 0;
    pop_flush_grant_count = 0;
    cache_miss_pulse_count = 0;
    current_live_fill = 0;
    max_live_fill = 0;
    push_count_at_first_pop = 0;
    overwrite_count_at_first_pop = 0;
    live_fill_at_first_pop = 0;
    sampled_cycles = 0;
    first_push_cycle = 0;
    first_pop_cycle = 0;
    first_overwrite_cycle = 0;
    last_push_cycle = 0;
    last_pop_cycle = 0;
    last_overwrite_cycle = 0;
    dbg_inerr_cnt = '0;
    dbg_push_cnt = '0;
    dbg_pop_cnt = '0;
    dbg_overwrite_cnt = '0;
    dbg_cache_miss_cnt = '0;
    run_state_code = '0;
    pop_engine_state_code = '0;
    push_state_code = '0;
    endofrun_seen = '0;
    run_mgmt_flushed = '0;
    gts_counter_rst = '0;
    run_mgmt_flush_memory_start = '0;
    run_mgmt_flush_memory_done = '0;
    pop_flush_ram_done = '0;
    pop_flush_cam_done = '0;
    cam_clean = '0;
    pop_hits_count = '0;
    pop_rr_idx = '0;
    pop_issue_partition_idx = '0;
    pop_count_partition_idx = '0;
    pop_search_wait_cnt = '0;
    pop_partition_pending = '0;
    pop_partition_load = '0;
    pop_partition_advance = '0;
    pop_partition_result_valid = '0;
    pop_partition_flag = '0;
    pop_partition_has_more = '0;
    pop_last_hit_pending = '0;
    pop_cmd_fifo_sclr = '0;
    deassembly_fifo_sclr = '0;
    expected_latency_48b = '0;
    read_time_ptr = '0;
    gts_8n = '0;
    gts_end_of_run = '0;
    pop_cmd_fifo_usedw = '0;
    deassembly_fifo_usedw = '0;
    terminating_drain_done = 1'b0;
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    push_ap = new("push_ap", this);
    if (!uvm_config_db#(virtual dut_debug_if.mon)::get(this, "", "vif", vif))
      `uvm_fatal("DBG_MON", "Failed to get dut_debug_if.mon from config_db")
  endfunction

  task run_phase(uvm_phase phase);
    ring_buffer_cam_pkg::debug_push_item item;
    logic [47:0] prev_push_cnt;
    logic [47:0] prev_pop_cnt;
    logic [47:0] prev_overwrite_cnt;
    prev_push_cnt = '0;
    prev_pop_cnt = '0;
    prev_overwrite_cnt = '0;
    forever begin
      @(posedge vif.clk);
      sampled_cycles++;

      dbg_inerr_cnt       = vif.dbg_inerr_cnt;
      dbg_push_cnt        = vif.dbg_push_cnt;
      dbg_pop_cnt         = vif.dbg_pop_cnt;
      dbg_overwrite_cnt   = vif.dbg_overwrite_cnt;
      dbg_cache_miss_cnt  = vif.dbg_cache_miss_cnt;
      run_state_code      = vif.run_state_code;
      pop_engine_state_code = vif.pop_engine_state_code;
      push_state_code     = vif.push_state_code;
      endofrun_seen       = vif.endofrun_seen;
      run_mgmt_flushed    = vif.run_mgmt_flushed;
      gts_counter_rst     = vif.gts_counter_rst;
      run_mgmt_flush_memory_start = vif.run_mgmt_flush_memory_start;
      run_mgmt_flush_memory_done = vif.run_mgmt_flush_memory_done;
      pop_flush_ram_done  = vif.pop_flush_ram_done;
      pop_flush_cam_done  = vif.pop_flush_cam_done;
      cam_clean           = vif.cam_clean;
      pop_hits_count      = vif.pop_hits_count;
      pop_rr_idx          = vif.pop_rr_idx;
      pop_issue_partition_idx = vif.pop_issue_partition_idx;
      pop_count_partition_idx = vif.pop_count_partition_idx;
      pop_search_wait_cnt = vif.pop_search_wait_cnt;
      pop_partition_pending = vif.pop_partition_pending;
      pop_partition_load  = vif.pop_partition_load;
      pop_partition_advance = vif.pop_partition_advance;
      pop_partition_result_valid = vif.pop_partition_result_valid;
      pop_partition_flag  = vif.pop_partition_flag;
      pop_partition_has_more = vif.pop_partition_has_more;
      pop_last_hit_pending = vif.pop_last_hit_pending;
      pop_cmd_fifo_sclr   = vif.pop_cmd_fifo_sclr;
      deassembly_fifo_sclr = vif.deassembly_fifo_sclr;
      expected_latency_48b = vif.expected_latency_48b;
      read_time_ptr       = vif.read_time_ptr;
      gts_8n              = vif.gts_8n;
      gts_end_of_run      = vif.gts_end_of_run;
      pop_cmd_fifo_usedw  = vif.pop_cmd_fifo_usedw;
      deassembly_fifo_usedw = vif.deassembly_fifo_usedw;
      terminating_drain_done = vif.terminating_drain_done;

      current_live_fill = dbg_push_cnt[31:0] - dbg_pop_cnt[31:0] - dbg_overwrite_cnt[31:0];
      if (current_live_fill > max_live_fill) begin
        max_live_fill = current_live_fill;
      end

      if (dbg_push_cnt != prev_push_cnt) begin
        if (first_push_cycle == 0) begin
          first_push_cycle = sampled_cycles;
        end
        last_push_cycle = sampled_cycles;
      end
      if (dbg_pop_cnt != prev_pop_cnt) begin
        if (first_pop_cycle == 0) begin
          first_pop_cycle = sampled_cycles;
          push_count_at_first_pop = dbg_push_cnt[31:0];
          overwrite_count_at_first_pop = dbg_overwrite_cnt[31:0];
          live_fill_at_first_pop = current_live_fill;
        end
        last_pop_cycle = sampled_cycles;
      end
      if (dbg_overwrite_cnt != prev_overwrite_cnt) begin
        if (first_overwrite_cycle == 0) begin
          first_overwrite_cycle = sampled_cycles;
        end
        last_overwrite_cycle = sampled_cycles;
      end

      if (vif.pop_cmd_fifo_wrreq === 1'b1)
        pop_cmd_wrreq_count++;
      if (vif.pop_cmd_fifo_rdack === 1'b1)
        pop_cmd_rdack_count++;
      if (vif.push_write_grant === 1'b1)
        push_write_grant_count++;
      if (vif.push_erase_grant === 1'b1)
        push_erase_grant_count++;
      if (vif.pop_erase_grant === 1'b1)
        pop_erase_grant_count++;
      if (vif.pop_flush_grant === 1'b1)
        pop_flush_grant_count++;
      if (vif.pop_cache_miss_pulse === 1'b1)
        cache_miss_pulse_count++;

      if (vif.push_write_grant === 1'b1 &&
          vif.side_ram_we === 1'b1) begin
        item = ring_buffer_cam_pkg::debug_push_item::type_id::create("push_evt");
        item.slot_addr = vif.side_ram_waddr;
        item.raw_hit = vif.side_ram_din[38:0];
        item.push_count = vif.dbg_push_cnt;
        item.overwrite_count = vif.dbg_overwrite_cnt;
        push_ap.write(item);
      end

      prev_push_cnt = dbg_push_cnt;
      prev_pop_cnt = dbg_pop_cnt;
      prev_overwrite_cnt = dbg_overwrite_cnt;
    end
  endtask

endclass

`endif

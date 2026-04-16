// base_test.sv — Base UVM test with common setup and teardown

`ifndef BASE_TEST_SV
`define BASE_TEST_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class base_test extends uvm_test;
  `uvm_component_utils(base_test)

  localparam int unsigned CSR_UID_ADDR             = 0;
  localparam int unsigned CSR_META_ADDR            = 1;
  localparam int unsigned CSR_CTRL_ADDR            = 2;
  localparam int unsigned CSR_EXPECTED_LAT_ADDR    = 3;
  localparam int unsigned CSR_FILL_LEVEL_ADDR      = 4;
  localparam int unsigned CSR_INERR_COUNT_ADDR     = 5;
  localparam int unsigned CSR_PUSH_COUNT_ADDR      = 6;
  localparam int unsigned CSR_POP_COUNT_ADDR       = 7;
  localparam int unsigned CSR_OVERWRITE_ADDR       = 8;
  localparam int unsigned CSR_CACHE_MISS_ADDR      = 9;

  ring_buffer_cam_env                       m_env;
  ring_buffer_cam_pkg::ring_buffer_cam_cfg  m_cfg;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    m_cfg = ring_buffer_cam_pkg::ring_buffer_cam_cfg::type_id::create("cfg");
    void'(uvm_config_db#(int unsigned)::get(this, "", "ring_buffer_n_entry", m_cfg.ring_buffer_n_entry));
    void'(uvm_config_db#(int unsigned)::get(this, "", "interleaving_factor", m_cfg.interleaving_factor));
    void'(uvm_config_db#(int unsigned)::get(this, "", "interleaving_index", m_cfg.interleaving_index));
    void'(uvm_config_db#(int unsigned)::get(this, "", "n_partitions", m_cfg.n_partitions));
    void'(uvm_config_db#(int unsigned)::get(this, "", "encoder_leaf_width", m_cfg.encoder_leaf_width));
    void'(uvm_config_db#(int unsigned)::get(this, "", "encoder_pipe_stages", m_cfg.encoder_pipe_stages));
    uvm_config_db#(ring_buffer_cam_pkg::ring_buffer_cam_cfg)::set(
      this, "m_env", "cfg", m_cfg);
    m_env = ring_buffer_cam_env::type_id::create("m_env", this);
  endfunction

  task automatic wait_clocks(int unsigned cycles);
    repeat (cycles) @(posedge m_env.m_csr_drv.vif.clk);
  endtask

  task automatic wait_for_reset_release();
    do begin
      @(posedge m_env.m_csr_drv.vif.clk);
    end while (m_env.m_csr_drv.vif.rst === 1'b1);
    m_cfg.note_reset_defaults();
    wait_clocks(2);
  endtask

  task automatic csr_write(int unsigned addr, logic [31:0] data);
    csr_write_seq csr_seq;
    csr_seq = csr_write_seq::type_id::create($sformatf("csr_wr_%0d", addr));
    csr_seq.addr = addr;
    csr_seq.data = data;
    csr_seq.start(m_env.m_csr_seqr);
  endtask

  task automatic csr_read(int unsigned addr, output logic [31:0] data);
    csr_read_seq csr_seq;
    csr_seq = csr_read_seq::type_id::create($sformatf("csr_rd_%0d", addr));
    csr_seq.addr = addr;
    csr_seq.start(m_env.m_csr_seqr);
    data = csr_seq.data;
  endtask

  task automatic read_counter_u32(int unsigned addr, output int unsigned value);
    logic [31:0] data;
    csr_read(addr, data);
    value = data;
  endtask

  task automatic ctrl_send(logic [8:0] cmd);
    ctrl_cmd_seq ctrl_seq;
    ctrl_seq = ctrl_cmd_seq::type_id::create($sformatf("ctrl_%0h", cmd));
    ctrl_seq.cmd = cmd;
    ctrl_seq.start(m_env.m_ctrl_seqr);
  endtask

  task automatic enter_run_prepare();
    // RUN_PREPARE is acked immediately from IDLE/RUNNING. Re-issue it so the
    // driver waits on the DUT's real flush-complete handshake instead of a
    // fixed simulation delay.
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
  endtask

  task automatic csr_expect_mask(
    int unsigned addr,
    logic [31:0] mask,
    logic [31:0] expected,
    string       what
  );
    logic [31:0] data;
    csr_read(addr, data);
    if ((data & mask) !== expected) begin
      `uvm_error("CSR_CHK", $sformatf(
        "%s mismatch at addr %0d: got 0x%08x expected 0x%08x mask 0x%08x",
        what, addr, data, expected, mask))
    end
  endtask

  // Common startup: configure CSR and send run-control commands
  task configure_and_start(int unsigned latency = 128);
    enter_run_prepare();
    csr_write(CSR_EXPECTED_LAT_ADDR, latency);
    ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
    wait_clocks(2);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
    wait_clocks(20);
  endtask

  task automatic send_endofrun_marker();
    endofrun_marker_seq eor_seq;
    eor_seq = endofrun_marker_seq::type_id::create("eor_seq");
    eor_seq.lane_channel = m_cfg.interleaving_index;
    eor_seq.start(m_env.m_hit_seqr);
  endtask

  task automatic wait_for_scoreboard_idle(
    int unsigned max_cycles = 50_000,
    string what = "scoreboard drain"
  );
    int unsigned cycles;
    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_scb.remaining_entries() == 0 && m_env.m_scb.epoch_idle()) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_info("TIMEOUT", $sformatf(
      "%s debug: remaining=%0d max_remaining=%0d live_fill=%0d max_live_fill=%0d push=%0d pop=%0d overwrite=%0d term_done=%0d endofrun_seen=%0d pop_cmd_empty=%0d pop_cmd_usedw=%0d deassm_empty=%0d deassm_full=%0d deassm_usedw=%0d first_push_cycle=%0d first_pop_cycle=%0d first_overwrite_cycle=%0d",
      what,
      m_env.m_scb.remaining_entries(),
      m_env.m_scb.max_remaining_entries(),
      m_env.m_dbg_mon.current_live_fill,
      m_env.m_dbg_mon.max_live_fill,
      m_env.m_dbg_mon.dbg_push_cnt[31:0],
      m_env.m_dbg_mon.dbg_pop_cnt[31:0],
      m_env.m_dbg_mon.dbg_overwrite_cnt[31:0],
      m_env.m_dbg_mon.terminating_drain_done,
      m_env.m_dbg_mon.vif.endofrun_seen,
      m_env.m_dbg_mon.vif.pop_cmd_fifo_empty,
      m_env.m_dbg_mon.pop_cmd_fifo_usedw,
      m_env.m_dbg_mon.vif.deassembly_fifo_empty,
      m_env.m_dbg_mon.vif.deassembly_fifo_full,
      m_env.m_dbg_mon.deassembly_fifo_usedw,
      m_env.m_dbg_mon.first_push_cycle,
      m_env.m_dbg_mon.first_pop_cycle,
      m_env.m_dbg_mon.first_overwrite_cycle), UVM_LOW)
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: remaining=%0d epoch_idle=%0d",
      what, max_cycles, m_env.m_scb.remaining_entries(), m_env.m_scb.epoch_idle()))
  endtask

  task automatic run_b003_activity_case();
    same_key_burst_seq burst_seq;
    single_error_hit_seq err_seq;
    logic [31:0] inerr_count;
    logic [31:0] push_count;
    logic [31:0] pop_count;
    logic [31:0] fill_level;

    configure_and_start();

    err_seq = single_error_hit_seq::type_id::create("err_seq");
    err_seq.search_key = 8;
    err_seq.start(m_env.m_hit_seqr);
    wait_clocks(32);
    csr_read(CSR_INERR_COUNT_ADDR, inerr_count);
    if (inerr_count < 1) begin
      `uvm_error("CFG", $sformatf("INERR_COUNT did not increment after error hit: %0d", inerr_count))
    end

    burst_seq = same_key_burst_seq::type_id::create("burst_seq");
    burst_seq.num_hits = 32;
    burst_seq.search_key = 12;
    burst_seq.start(m_env.m_hit_seqr);

    wait_clocks(400);
    csr_read(CSR_PUSH_COUNT_ADDR, push_count);
    if (push_count < 32) begin
      `uvm_error("CFG", $sformatf("PUSH_COUNT too small after burst: %0d", push_count))
    end

    wait_for_scoreboard_idle(40_000, "B003 activity drain");
    csr_read(CSR_POP_COUNT_ADDR, pop_count);
    if (pop_count == 0) begin
      `uvm_error("CFG", "POP_COUNT did not advance after activity")
    end

    csr_read(CSR_FILL_LEVEL_ADDR, fill_level);
    if (fill_level > push_count) begin
      `uvm_error("CFG", $sformatf(
        "FILL_LEVEL larger than PUSH_COUNT: fill=%0d push=%0d",
        fill_level, push_count))
    end
  endtask

  task automatic run_x001_filter_inerr_case();
    single_error_hit_seq err_seq;
    same_key_burst_seq   good_seq;
    logic [31:0] inerr_count;
    logic [31:0] push_count;

    configure_and_start();

    err_seq = single_error_hit_seq::type_id::create("err_seq_filter_on");
    err_seq.search_key = 8;
    err_seq.start(m_env.m_hit_seqr);
    wait_clocks(32);
    csr_read(CSR_INERR_COUNT_ADDR, inerr_count);
    if (inerr_count < 1) begin
      `uvm_error("X001", $sformatf("INERR_COUNT did not increment with filter_inerr=1: %0d", inerr_count))
    end

    csr_write(CSR_CTRL_ADDR, 32'h0000_0001); // go=1, filter_inerr=0
    wait_clocks(2);
    err_seq = single_error_hit_seq::type_id::create("err_seq_filter_off");
    err_seq.search_key = 12;
    err_seq.start(m_env.m_hit_seqr);
    wait_clocks(64);
    csr_read(CSR_PUSH_COUNT_ADDR, push_count);
    if (push_count < 1) begin
      `uvm_error("X001", "Errored hit with filter disabled was not accepted")
    end

    good_seq = same_key_burst_seq::type_id::create("good_seq");
    good_seq.num_hits = 16;
    good_seq.search_key = 16;
    good_seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(60_000, "X001 mixed good/error drain");
  endtask

  task automatic run_x003_good_terminate_case();
    same_key_burst_seq seq;
    logic [31:0] pop_count;
    logic [31:0] push_count;

    configure_and_start();
    seq = same_key_burst_seq::type_id::create("x003_seq");
    seq.num_hits = 32;
    seq.search_key = 8;
    seq.start(m_env.m_hit_seqr);
    wait_clocks(64);
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    wait_for_scoreboard_idle(60_000, "X003 terminate drain");
    csr_read(CSR_PUSH_COUNT_ADDR, push_count);
    csr_read(CSR_POP_COUNT_ADDR, pop_count);
    if (pop_count > push_count) begin
      `uvm_error("X003", $sformatf("POP_COUNT exceeded PUSH_COUNT after terminate: push=%0d pop=%0d", push_count, pop_count))
    end
  endtask

  task automatic terminate_and_drain(
    int unsigned max_cycles = 250_000,
    string what = "terminate drain"
  );
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    // Re-issue TERMINATING so the driver waits on the DUT's explicit
    // terminating_drain_done/ready handshake rather than relying on a fixed
    // post-marker delay.
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_clocks(4);
    wait_for_scoreboard_idle(max_cycles, what);
  endtask

  task automatic expect_service_model_accounting(
    string what,
    bit expect_quiescent = 0,
    int unsigned min_overwrites = 0
  );
    int unsigned fill_level;
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned cache_miss_count;
    int unsigned modeled_remaining;
    int unsigned expected_fill;

    if (expect_quiescent) begin
      // The frontdoor CSR fill-level path trails the debug counters by a small
      // number of clocks after the last pop/overwrite event.
      wait_clocks(4);
    end

    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);

    expected_fill = push_count - pop_count - overwrite_count;
    modeled_remaining = m_env.m_scb.remaining_entries();

    if (fill_level != expected_fill) begin
      `uvm_error("ACCOUNTING", $sformatf(
        "%s fill-level invariant failed: fill=%0d expected(push-pop-overwrite)=%0d push=%0d pop=%0d overwrite=%0d",
        what, fill_level, expected_fill, push_count, pop_count, overwrite_count))
    end

    if (fill_level != modeled_remaining) begin
      `uvm_error("ACCOUNTING", $sformatf(
        "%s scoreboard/CSR fill mismatch: fill=%0d modeled_remaining=%0d",
        what, fill_level, modeled_remaining))
    end

    if (overwrite_count < min_overwrites) begin
      `uvm_error("ACCOUNTING", $sformatf(
        "%s overwrite threshold not met: observed=%0d expected_at_least=%0d",
        what, overwrite_count, min_overwrites))
    end

    if (expect_quiescent) begin
      expect_backdoor_counter_matches(CSR_PUSH_COUNT_ADDR, m_env.m_dbg_mon.dbg_push_cnt,
        {what, " PUSH_COUNT"});
      expect_backdoor_counter_matches(CSR_POP_COUNT_ADDR, m_env.m_dbg_mon.dbg_pop_cnt,
        {what, " POP_COUNT"});
      expect_backdoor_counter_matches(CSR_OVERWRITE_ADDR, m_env.m_dbg_mon.dbg_overwrite_cnt,
        {what, " OVERWRITE_COUNT"});
      expect_backdoor_counter_matches(CSR_CACHE_MISS_ADDR, m_env.m_dbg_mon.dbg_cache_miss_cnt,
        {what, " CACHE_MISS_COUNT"});

      if (fill_level != 0) begin
        `uvm_error("ACCOUNTING", $sformatf(
          "%s expected quiescent fill_level=0, observed %0d",
          what, fill_level))
      end
      if (push_count != (pop_count + overwrite_count)) begin
        `uvm_error("ACCOUNTING", $sformatf(
          "%s terminate accounting failed: push=%0d pop=%0d overwrite=%0d cache_miss=%0d",
          what, push_count, pop_count, overwrite_count, cache_miss_count))
      end
    end
  endtask

  task automatic run_single_key_overwrite_window(
    string what,
    int unsigned num_hits
  );
    overwrite_profile_seq pressure_seq;
    int unsigned push_count;
    int unsigned overwrite_count;
    int unsigned pre_service_pushes;
    int unsigned min_window_overwrites;
    bit [7:0] focus_search_key;

    configure_and_start(2000);

    pressure_seq = overwrite_profile_seq::type_id::create({what, "_pressure"});
    pressure_seq.num_hits = num_hits;
    pressure_seq.lane_key_start_ord = 2;
    pressure_seq.pool_keys = 1;
    pressure_seq.hits_per_key_switch = 1;
    pressure_seq.progress_stride = 64;
    pressure_seq.progress_tag = what;
    pressure_seq.start(m_env.m_hit_seqr);
    terminate_and_drain(80_000, {what, " terminate drain"});
    expect_service_model_accounting({what, " post-terminate"}, 1, 1);

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
    pre_service_pushes = m_env.m_dbg_mon.push_count_at_first_pop;

    if (push_count != num_hits) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s final push_count mismatch: observed=%0d expected=%0d",
        what, push_count, num_hits))
    end
    if (m_env.m_scb.max_remaining_entries() < m_cfg.ring_buffer_n_entry) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s never saturated the rbCAM: max_remaining=%0d ring_depth=%0d",
        what, m_env.m_scb.max_remaining_entries(), m_cfg.ring_buffer_n_entry))
    end
    if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < m_cfg.ring_buffer_n_entry) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s same-ts hotspot never saturated the full sector depth: key=%0d max_for_key=%0d ring_depth=%0d",
        what, focus_search_key,
        m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
        m_cfg.ring_buffer_n_entry))
    end
    if (m_env.m_dbg_mon.first_pop_cycle == 0) begin
      `uvm_error("PRESSURE", $sformatf("%s never observed a pop event", what))
    end
    if (m_env.m_dbg_mon.first_overwrite_cycle == 0) begin
      `uvm_error("PRESSURE", $sformatf("%s never observed an overwrite event", what))
    end
    if (m_env.m_dbg_mon.first_overwrite_cycle != 0 &&
        m_env.m_dbg_mon.first_pop_cycle != 0 &&
        m_env.m_dbg_mon.first_overwrite_cycle >= m_env.m_dbg_mon.first_pop_cycle) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s overwrite started only after pop service: first_overwrite_cycle=%0d first_pop_cycle=%0d",
        what, m_env.m_dbg_mon.first_overwrite_cycle, m_env.m_dbg_mon.first_pop_cycle))
    end
    if (pre_service_pushes <= m_cfg.ring_buffer_n_entry) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s never exceeded the 512-entry sector depth before first pop service: pushes_before_first_pop=%0d ring_depth=%0d",
        what, pre_service_pushes, m_cfg.ring_buffer_n_entry))
    end else begin
      min_window_overwrites = pre_service_pushes - m_cfg.ring_buffer_n_entry;
      if (overwrite_count < min_window_overwrites) begin
        `uvm_error("PRESSURE", $sformatf(
          "%s violated the pre-service burst lower bound: overwrite=%0d pushes_before_first_pop=%0d required_at_least=%0d",
          what, overwrite_count, pre_service_pushes, min_window_overwrites))
      end
    end
    `uvm_info("PRESSURE", $sformatf(
      "%s network-calculus summary: push=%0d overwrite=%0d pushes_before_first_pop=%0d max_remaining=%0d max_hotspot=%0d first_push_cycle=%0d first_overwrite_cycle=%0d first_pop_cycle=%0d",
      what, push_count, overwrite_count, pre_service_pushes,
      m_env.m_scb.max_remaining_entries(),
      m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
      m_env.m_dbg_mon.first_push_cycle,
      m_env.m_dbg_mon.first_overwrite_cycle,
      m_env.m_dbg_mon.first_pop_cycle), UVM_LOW)
  endtask

  function automatic logic [31:0] expected_meta_version();
    return 32'h1A01_4192;
  endfunction

  task automatic set_meta_sel(logic [1:0] sel);
    csr_write(CSR_META_ADDR, {30'd0, sel});
    wait_clocks(2);
  endtask

  task automatic expect_backdoor_counter_matches(
    int unsigned   addr,
    logic [47:0]   dbg_value,
    string         what
  );
    logic [31:0] data;
    csr_read(addr, data);
    if (data !== dbg_value[31:0]) begin
      `uvm_error("DBG_CSR", $sformatf(
        "%s mismatch: frontdoor=0x%08x backdoor=0x%08x",
        what, data, dbg_value[31:0]))
    end
  endtask

  task automatic wait_for_subheader_count(
    int unsigned target_count,
    int unsigned max_cycles = 50_000,
    string what = "subheader emission"
  );
    int unsigned cycles;
    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_out_mon.total_subheaders_seen >= target_count) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: seen=%0d target=%0d",
      what, max_cycles, m_env.m_out_mon.total_subheaders_seen, target_count))
  endtask

  task automatic run_case_by_id(string case_id);
    single_push_pop_seq   single_seq;
    same_key_burst_seq    burst_seq;
    sequential_keys_seq   seq_keys;
    overwrite_stress_seq  ow_seq;
    overwrite_profile_seq pressure_seq;
    random_push_pop_seq   rand_same;
    random_multi_key_seq  rand_multi;
    random_throughput_seq rand_tp;
    single_error_hit_seq  err_seq;
    logic [31:0]          data_a;
    logic [31:0]          data_b;
    int unsigned          subhdr_before;
    int unsigned          ow16;
    int unsigned          ow20;
    int unsigned          ow24;
    int unsigned          ow28;
    int unsigned          ow_min;
    int unsigned          ow_max;

    if (case_id == "B001") begin
      csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID reset default");
      csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0013, 32'h0000_0011, "CTRL reset defaults");
      csr_expect_mask(CSR_EXPECTED_LAT_ADDR, 32'h0000_FFFF, 32'd2000, "EXPECTED_LATENCY reset default");
      csr_expect_mask(CSR_FILL_LEVEL_ADDR, 32'hFFFF_FFFF, 32'd0, "FILL_LEVEL reset default");
      for (int addr = CSR_INERR_COUNT_ADDR; addr <= CSR_CACHE_MISS_ADDR; addr++) begin
        csr_expect_mask(addr, 32'hFFFF_FFFF, 32'd0,
          $sformatf("Debug counter reset default addr=%0d", addr));
      end
    end else if (case_id == "B002") begin
      csr_write(CSR_CTRL_ADDR, 32'h0000_0000);
      wait_clocks(2);
      csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0013, 32'h0000_0000, "CTRL writable bits clear");
      csr_write(CSR_CTRL_ADDR, 32'h0000_0011);
      wait_clocks(2);
      csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0013, 32'h0000_0011, "CTRL writable bits set");
      csr_write(CSR_CTRL_ADDR, 32'h0000_0013);
      wait_clocks(2);
      csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0013, 32'h0000_0011, "CTRL soft_reset self-clears");
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0123);
      wait_clocks(2);
      csr_expect_mask(CSR_EXPECTED_LAT_ADDR, 32'h0000_FFFF, 32'h0000_0123, "EXPECTED_LATENCY writable");
    end else if (case_id == "B003") begin
      run_b003_activity_case();
    end else if (case_id == "B004") begin
      configure_and_start();
      single_seq = single_push_pop_seq::type_id::create("single_seq");
      single_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(20_000, "B004 single_push_pop");
    end else if (case_id == "B005") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("burst_seq_128");
      burst_seq.num_hits = 128;
      burst_seq.search_key = 8;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(40_000, "B005 same_key_burst_128");
    end else if (case_id == "B006") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("burst_seq_256");
      burst_seq.num_hits = 256;
      burst_seq.search_key = 12;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(60_000, "B006 same_key_burst_256");
    end else if (case_id == "B007") begin
      configure_and_start();
      seq_keys = sequential_keys_seq::type_id::create("seq_keys");
      seq_keys.num_keys = 16;
      seq_keys.hits_per_key = 8;
      seq_keys.start_key = 4;
      seq_keys.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(80_000, "B007 sequential_keys");
    end else if (case_id == "B008") begin
      configure_and_start();
      if (m_cfg.run_state != ring_buffer_cam_pkg::RUN_STATE_RUNNING) begin
        `uvm_error("B008", $sformatf("Run-control startup spine failed: state=%0d", m_cfg.run_state))
      end
    end else if (case_id == "B009") begin
      csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID reset-time readback");
      configure_and_start();
      csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID RUNNING readback");
      ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
      send_endofrun_marker();
      wait_clocks(8);
      csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID TERMINATING readback");
    end else if (case_id == "B010") begin
      set_meta_sel(2'b00);
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, expected_meta_version(), "META VERSION readback");
    end else if (case_id == "B011") begin
      set_meta_sel(2'b01);
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd20260402, "META DATE readback");
    end else if (case_id == "B012") begin
      set_meta_sel(2'b10);
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd0, "META GIT readback");
    end else if (case_id == "B013") begin
      set_meta_sel(2'b11);
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd0, "META INSTANCE readback");
    end else if (case_id == "B014") begin
      csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0001, 32'h0000_0001, "CTRL go reset default");
    end else if (case_id == "B015") begin
      csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
      wait_clocks(2);
      csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0002, 32'h0000_0000, "CTRL soft_reset self-clears");
    end else if (case_id == "B016") begin
      csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0010, 32'h0000_0010, "CTRL filter_inerr reset default");
    end else if (case_id == "B017") begin
      csr_write(CSR_CTRL_ADDR, 32'hFFFF_FFFF);
      wait_clocks(2);
      csr_expect_mask(CSR_CTRL_ADDR, 32'hFFFF_FFFF, 32'h0000_0011, "CTRL spare bits stay inert");
    end else if (case_id == "B018") begin
      csr_expect_mask(CSR_EXPECTED_LAT_ADDR, 32'h0000_FFFF, 32'd2000, "EXPECTED_LATENCY reset default");
    end else if (case_id == "B019") begin
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(2);
      csr_expect_mask(CSR_EXPECTED_LAT_ADDR, 32'h0000_FFFF, 32'h0000_0000, "EXPECTED_LATENCY min write");
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_FFFF);
      wait_clocks(2);
      csr_expect_mask(CSR_EXPECTED_LAT_ADDR, 32'h0000_FFFF, 32'h0000_FFFF, "EXPECTED_LATENCY max write");
    end else if (case_id == "B021") begin
      configure_and_start(2000);
      burst_seq = same_key_burst_seq::type_id::create("b021_burst");
      burst_seq.num_hits = 8;
      burst_seq.search_key = 8;
      burst_seq.start(m_env.m_hit_seqr);
      wait_clocks(64);
      csr_expect_mask(CSR_FILL_LEVEL_ADDR, 32'hFFFF_FFFF, 32'd8, "FILL_LEVEL push-pop-overwrite identity");
    end else if (case_id == "B022") begin
      configure_and_start(2000);
      burst_seq = same_key_burst_seq::type_id::create("b022_burst");
      burst_seq.num_hits = 4;
      burst_seq.search_key = 12;
      burst_seq.start(m_env.m_hit_seqr);
      wait_clocks(48);
      csr_read(CSR_FILL_LEVEL_ADDR, data_a);
      csr_write(CSR_FILL_LEVEL_ADDR, 32'hDEAD_BEEF);
      wait_clocks(2);
      csr_read(CSR_FILL_LEVEL_ADDR, data_b);
      if (data_b !== data_a) begin
        `uvm_error("B022", $sformatf(
          "FILL_LEVEL write-ignore violated: before=0x%08x after=0x%08x",
          data_a, data_b))
      end
    end else if (case_id == "B023") begin
      configure_and_start(2000);
      err_seq = single_error_hit_seq::type_id::create("b023_err_filter_on");
      err_seq.search_key = 8;
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(32);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "INERR_COUNT increments when filter_inerr=1");
      csr_write(CSR_CTRL_ADDR, 32'h0000_0001);
      wait_clocks(2);
      err_seq = single_error_hit_seq::type_id::create("b023_err_filter_off");
      err_seq.search_key = 12;
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(32);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "INERR_COUNT stays stable when filter_inerr=0");
      expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
        "INERR frontdoor/backdoor match");
    end else if (case_id == "B024") begin
      configure_and_start(2000);
      err_seq = single_error_hit_seq::type_id::create("b024_err");
      err_seq.search_key = 8;
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(32);
      enter_run_prepare();
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "INERR_COUNT clears in RUN_PREPARE");
    end else if (case_id == "B025") begin
      configure_and_start(2000);
      burst_seq = same_key_burst_seq::type_id::create("b025_burst");
      burst_seq.num_hits = 8;
      burst_seq.search_key = 16;
      burst_seq.start(m_env.m_hit_seqr);
      wait_clocks(64);
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd8, "PUSH_COUNT matches push grants");
      expect_backdoor_counter_matches(CSR_PUSH_COUNT_ADDR, m_env.m_dbg_mon.dbg_push_cnt,
        "PUSH_COUNT frontdoor/backdoor match");
    end else if (case_id == "B031") begin
      set_meta_sel(2'b01);
      csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID stable across META sel write");
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd20260402, "META DATE after selector write");
      csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID stable after META read");
    end else if (case_id == "B051") begin
      configure_and_start(2000);
      csr_write(CSR_CTRL_ADDR, 32'h0000_0010);
      wait_clocks(2);
      burst_seq = same_key_burst_seq::type_id::create("b051_burst");
      burst_seq.num_hits = 8;
      burst_seq.search_key = 20;
      burst_seq.start(m_env.m_hit_seqr);
      wait_clocks(64);
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "go=0 blocks push");
      if (m_env.m_scb.remaining_entries() != 0) begin
        `uvm_error("B051", $sformatf(
          "go=0 case left modeled residents: %0d",
          m_env.m_scb.remaining_entries()))
      end
    end else if (case_id == "B052") begin
      configure_and_start(2000);
      burst_seq = same_key_burst_seq::type_id::create("b052_good_lane");
      burst_seq.num_hits = 4;
      burst_seq.search_key = 8;
      burst_seq.start(m_env.m_hit_seqr);
      burst_seq = same_key_burst_seq::type_id::create("b052_bad_lane");
      burst_seq.num_hits = 4;
      burst_seq.search_key = 9;
      burst_seq.start(m_env.m_hit_seqr);
      wait_clocks(96);
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd4, "interleaving filter accepts only matching lane");
    end else if (case_id == "B053") begin
      configure_and_start(2000);
      ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
      send_endofrun_marker();
      wait_clocks(8);
      if (m_env.m_dbg_mon.vif.endofrun_seen !== 1'b1) begin
        `uvm_error("B053", "endofrun_seen did not assert on matching empty marker")
      end
    end else if (case_id == "B054") begin
      configure_and_start(2000);
      single_seq = single_push_pop_seq::type_id::create("b054_single");
      single_seq.search_key = 4;
      single_seq.start(m_env.m_hit_seqr);
      wait_clocks(32);
      expect_backdoor_counter_matches(CSR_PUSH_COUNT_ADDR, m_env.m_dbg_mon.dbg_push_cnt,
        "single-hit push_count");
      if (m_env.m_dbg_mon.dbg_overwrite_cnt != 0) begin
        `uvm_error("B054", $sformatf(
          "Unexpected overwrite during single-hit push: %0d",
          m_env.m_dbg_mon.dbg_overwrite_cnt))
      end
    end else if (case_id == "B060") begin
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      configure_and_start();
      wait_for_subheader_count(subhdr_before + 1, 40_000, "zero-hit subheader");
      if (m_env.m_out_mon.recent_subheaders.size() == 0) begin
        `uvm_error("B060", "No subheader captured for zero-hit framing check")
      end else begin
        if (!(m_env.m_out_mon.recent_subheaders[$].sop && m_env.m_out_mon.recent_subheaders[$].eop)) begin
          `uvm_error("B060", "Zero-hit subheader did not assert SOP+EOP")
        end
        if (m_env.m_out_mon.recent_subheaders[$].hit_count != 0) begin
          `uvm_error("B060", $sformatf(
            "Zero-hit subheader hit_count was %0d",
            m_env.m_out_mon.recent_subheaders[$].hit_count))
        end
        if (m_env.m_out_mon.recent_subheaders[$].raw_data[7:0] != ring_buffer_cam_pkg::K237) begin
          `uvm_error("B060", "Zero-hit subheader K237 marker mismatch")
        end
      end
    end else if (case_id == "B061") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("b061_burst");
      burst_seq.num_hits = 1;
      burst_seq.search_key = 8;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(30_000, "B061 single-hit drain");
      if (m_env.m_out_mon.recent_subheaders.size() == 0 || m_env.m_out_mon.recent_hits.size() == 0) begin
        `uvm_error("B061", "Did not capture both subheader and hit beats")
      end else begin
        if (!(m_env.m_out_mon.recent_subheaders[$].sop && !m_env.m_out_mon.recent_subheaders[$].eop)) begin
          `uvm_error("B061", "1-hit subheader framing was not SOP-only")
        end
        if (!m_env.m_out_mon.recent_hits[$].eop) begin
          `uvm_error("B061", "1-hit data beat did not assert EOP")
        end
      end
    end else if (case_id == "B062") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("b062_burst");
      burst_seq.num_hits = 1;
      burst_seq.search_key = 8'h2A;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(30_000, "B062 search-key framing");
      if (m_env.m_out_mon.recent_subheaders.size() == 0 ||
          m_env.m_out_mon.recent_subheaders[$].search_key != 8'h2A) begin
        `uvm_error("B062", "Subheader search_key did not mirror ts[11:4]")
      end
    end else if (case_id == "B063") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("b063_burst");
      burst_seq.num_hits = 5;
      burst_seq.search_key = 12;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(40_000, "B063 hit_count framing");
      if (m_env.m_out_mon.recent_subheaders.size() == 0 ||
          m_env.m_out_mon.recent_subheaders[$].hit_count != 5) begin
        `uvm_error("B063", "Subheader hit_count field did not report 5")
      end
    end else if (case_id == "B064") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("b064_burst");
      burst_seq.num_hits = 2;
      burst_seq.search_key = 16;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(40_000, "B064 K237 framing");
      if (m_env.m_out_mon.recent_subheaders.size() == 0 ||
          m_env.m_out_mon.recent_subheaders[$].raw_data[7:0] != ring_buffer_cam_pkg::K237) begin
        `uvm_error("B064", "Subheader K237 identifier mismatch")
      end
    end else if (case_id == "B065") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("b065_burst");
      burst_seq.num_hits = 2;
      burst_seq.search_key = 20;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(40_000, "B065 byte_is_k framing");
      if (m_env.m_out_mon.recent_subheaders.size() == 0 || m_env.m_out_mon.recent_hits.size() == 0) begin
        `uvm_error("B065", "Missing subheader/hit history for byte_is_k check")
      end else begin
        if (m_env.m_out_mon.recent_subheaders[$].raw_data[35:32] != 4'b0001) begin
          `uvm_error("B065", "Subheader byte_is_k field was not 0001")
        end
        if (m_env.m_out_mon.recent_hits[$].raw_data[35:32] != 4'b0000) begin
          `uvm_error("B065", "Hit byte_is_k field was not 0000")
        end
      end
    end else if (case_id == "B066") begin
      configure_and_start();
      single_seq = single_push_pop_seq::type_id::create("b066_single");
      single_seq.search_key = 8'h2C;
      single_seq.hit_asic = 4'hA;
      single_seq.hit_channel = 5'd17;
      single_seq.ts_low = 4'hC;
      single_seq.hit_tcc1n6 = 3'h5;
      single_seq.hit_tfine = 5'h12;
      single_seq.hit_et1n6 = 9'h101;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(30_000, "B066 hit bit layout");
      if (m_env.m_out_mon.recent_hits.size() == 0) begin
        `uvm_error("B066", "No hit captured for egress bit-layout check")
      end else begin
        if (m_env.m_out_mon.recent_hits[$].ts_3_0 != 4'hC ||
            m_env.m_out_mon.recent_hits[$].asic != 4'hA ||
            m_env.m_out_mon.recent_hits[$].channel != 5'd17 ||
            m_env.m_out_mon.recent_hits[$].ts50p != {3'h5, 5'h12} ||
            m_env.m_out_mon.recent_hits[$].et1n6 != 9'h101) begin
          `uvm_error("B066", "Hit egress bit slices did not match the injected payload")
        end
      end
    end else if (case_id == "E001") begin
      configure_and_start();
      wait_clocks(512);
      if (m_env.m_scb.total_zero_hit_subheaders == 0) begin
        `uvm_error("E001", "No zero-hit subheader was observed")
      end
    end else if (case_id == "E003") begin
      run_single_key_overwrite_window("E003 overwrite_stress same-ts hotspot", 520);
    end else if (case_id == "P001") begin
      configure_and_start();
      rand_same = random_push_pop_seq::type_id::create("rand_same");
      assert(rand_same.randomize());
      rand_same.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(80_000, "P001 random_push_pop");
    end else if (case_id == "P002") begin
      configure_and_start();
      rand_multi = random_multi_key_seq::type_id::create("rand_multi");
      assert(rand_multi.randomize());
      rand_multi.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(100_000, "P002 random_multi_key");
    end else if (case_id == "P003") begin
      configure_and_start();
      rand_tp = random_throughput_seq::type_id::create("rand_tp");
      assert(rand_tp.randomize());
      rand_tp.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(120_000, "P003 random_throughput");
    end else if (case_id == "P004") begin
      configure_and_start(2000);
      pressure_seq = overwrite_profile_seq::type_id::create("p004_pressure");
      pressure_seq.num_hits = 4096;
      pressure_seq.lane_key_start_ord = 4;
      pressure_seq.pool_keys = 64;
      pressure_seq.hits_per_key_switch = 4;
      pressure_seq.start(m_env.m_hit_seqr);
      terminate_and_drain(350_000, "P004 overwrite soak terminate drain");
      expect_service_model_accounting("P004 overwrite soak post-terminate", 1, 1);
    end else if (case_id == "P111") begin
      run_single_key_overwrite_window("P111 single-key overwrite 10pct-ish", 576);
    end else if (case_id == "P112") begin
      run_single_key_overwrite_window("P112 single-key overwrite 30pct-ish", 736);
    end else if (case_id == "P113") begin
      run_single_key_overwrite_window("P113 single-key overwrite 50pct-ish", 1024);
    end else if (case_id == "P116") begin
      configure_and_start(2000);
      pressure_seq = overwrite_profile_seq::type_id::create("p116_pressure");
      pressure_seq.num_hits = 640;
      pressure_seq.lane_key_start_ord = 4;
      pressure_seq.pool_keys = 4;
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.start(m_env.m_hit_seqr);
      terminate_and_drain(100_000, "P116 balanced overwrite terminate drain");
      expect_service_model_accounting("P116 balanced overwrite post-terminate", 1, 128);
      if (m_env.m_scb.max_remaining_entries() < m_cfg.ring_buffer_n_entry) begin
        `uvm_error("P116", $sformatf(
          "Balanced pressure never saturated the rbCAM: max_remaining=%0d ring_depth=%0d",
          m_env.m_scb.max_remaining_entries(), m_cfg.ring_buffer_n_entry))
      end
      ow16 = m_env.m_scb.overwrite_events_for_new_key(8'd16);
      ow20 = m_env.m_scb.overwrite_events_for_new_key(8'd20);
      ow24 = m_env.m_scb.overwrite_events_for_new_key(8'd24);
      ow28 = m_env.m_scb.overwrite_events_for_new_key(8'd28);
      ow_min = ow16;
      ow_max = ow16;
      if (ow20 < ow_min) ow_min = ow20;
      if (ow24 < ow_min) ow_min = ow24;
      if (ow28 < ow_min) ow_min = ow28;
      if (ow20 > ow_max) ow_max = ow20;
      if (ow24 > ow_max) ow_max = ow24;
      if (ow28 > ow_max) ow_max = ow28;
      if ((ow_max - ow_min) > 1) begin
        `uvm_error("P116", $sformatf(
          "Balanced overwrite fairness mismatch: key16=%0d key20=%0d key24=%0d key28=%0d",
          ow16, ow20, ow24, ow28))
      end
    end else if (case_id == "P119") begin
      configure_and_start(2000);
      pressure_seq = overwrite_profile_seq::type_id::create("p119_burst_fast");
      pressure_seq.num_hits = 512;
      pressure_seq.lane_key_start_ord = 3;
      pressure_seq.pool_keys = 1;
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.start(m_env.m_hit_seqr);
      pressure_seq = overwrite_profile_seq::type_id::create("p119_burst_slow");
      pressure_seq.num_hits = 256;
      pressure_seq.lane_key_start_ord = 3;
      pressure_seq.pool_keys = 1;
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.inter_hit_gap_cycles = 1;
      pressure_seq.start(m_env.m_hit_seqr);
      terminate_and_drain(100_000, "P119 burst overwrite terminate drain");
      expect_service_model_accounting("P119 burst overwrite post-terminate", 1, 1);
      if (m_env.m_scb.max_remaining_entries() < m_cfg.ring_buffer_n_entry) begin
        `uvm_error("P119", $sformatf(
          "Burst overwrite case never saturated the rbCAM: max_remaining=%0d ring_depth=%0d",
          m_env.m_scb.max_remaining_entries(), m_cfg.ring_buffer_n_entry))
      end
    end else if (case_id == "X001") begin
      run_x001_filter_inerr_case();
    end else if (case_id == "X007") begin
      configure_and_start(2000);
      err_seq = single_error_hit_seq::type_id::create("x007_err");
      err_seq.search_key = 8;
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(32);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "X007 INERR_COUNT increments");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "X007 PUSH_COUNT unchanged");
      csr_expect_mask(CSR_FILL_LEVEL_ADDR, 32'hFFFF_FFFF, 32'd0, "X007 FILL_LEVEL unchanged");
      if (m_env.m_scb.total_ingress_accepted != 0) begin
        `uvm_error("X007", $sformatf(
          "Filtered bad hit should not be accepted, observed %0d accepted ingress beats",
          m_env.m_scb.total_ingress_accepted))
      end
    end else if (case_id == "X008") begin
      configure_and_start(2000);
      csr_write(CSR_CTRL_ADDR, 32'h0000_0001);
      wait_clocks(2);
      err_seq = single_error_hit_seq::type_id::create("x008_err");
      err_seq.search_key = 8;
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(32);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "X008 INERR_COUNT unchanged");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "X008 PUSH_COUNT increments");
      wait_for_scoreboard_idle(40_000, "X008 bad-hit pass-through drain");
      expect_service_model_accounting("X008 bad-hit pass-through", 1, 0);
    end else if (case_id == "X013") begin
      configure_and_start(2000);
      err_seq = single_error_hit_seq::type_id::create("x013_lane_miss");
      err_seq.use_raw_tcc8n = 1'b1;
      err_seq.raw_tcc8n = 13'(9 * 16);
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(32);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "X013 raw bad-hit still increments INERR_COUNT");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "X013 lane-miss does not push");
      if (m_env.m_scb.total_ingress_accepted != 0) begin
        `uvm_error("X013", $sformatf(
          "Lane-mismatched bad hit should not be accepted, observed %0d accepted ingress beats",
          m_env.m_scb.total_ingress_accepted))
      end
    end else if (case_id == "X003") begin
      run_x003_good_terminate_case();
    end else begin
      `uvm_fatal("CASE", $sformatf("Case %s is not implemented in the case engine", case_id))
    end
  endtask

  task automatic run_basic_bucket_frame();
    same_key_burst_seq   burst_seq;
    single_push_pop_seq  single_seq;
    sequential_keys_seq  seq_keys;
    single_error_hit_seq err_seq;
    logic [31:0] inerr_count;

    `uvm_info("CASE", "CASE_BEGIN bucket=BASIC", UVM_LOW)
    run_case_by_id("B001");
    run_case_by_id("B002");
    configure_and_start();
    if (m_cfg.run_state != ring_buffer_cam_pkg::RUN_STATE_RUNNING) begin
      `uvm_error("BASIC_FRAME", $sformatf("Startup spine failed inside BASIC frame: state=%0d", m_cfg.run_state))
    end

    err_seq = single_error_hit_seq::type_id::create("basic_frame_err_seq");
    err_seq.search_key = 8;
    err_seq.start(m_env.m_hit_seqr);
    wait_clocks(32);
    csr_read(CSR_INERR_COUNT_ADDR, inerr_count);
    if (inerr_count < 1) begin
      `uvm_error("BASIC_FRAME", "B003 inerr check failed inside bucket_frame")
    end

    single_seq = single_push_pop_seq::type_id::create("basic_frame_single");
    single_seq.start(m_env.m_hit_seqr);

    burst_seq = same_key_burst_seq::type_id::create("basic_frame_burst_128");
    burst_seq.num_hits = 128;
    burst_seq.search_key = 8;
    burst_seq.start(m_env.m_hit_seqr);

    burst_seq = same_key_burst_seq::type_id::create("basic_frame_burst_256");
    burst_seq.num_hits = 256;
    burst_seq.search_key = 12;
    burst_seq.start(m_env.m_hit_seqr);

    seq_keys = sequential_keys_seq::type_id::create("basic_frame_seq_keys");
    seq_keys.num_keys = 8;
    seq_keys.hits_per_key = 4;
    seq_keys.start_key = 4;
    seq_keys.start(m_env.m_hit_seqr);

    wait_for_scoreboard_idle(150_000, "BASIC bucket_frame");
    `uvm_info("CASE", "CASE_END bucket=BASIC", UVM_LOW)
  endtask

  task automatic run_cross_good_error_good();
    same_key_burst_seq   burst_seq;
    single_error_hit_seq err_seq;
    sequential_keys_seq  seq_keys;

    `uvm_info("CASE", "CASE_BEGIN run=CROSS-007", UVM_LOW)
    configure_and_start();

    burst_seq = same_key_burst_seq::type_id::create("cross_good_a");
    burst_seq.num_hits = 32;
    burst_seq.search_key = 8;
    burst_seq.start(m_env.m_hit_seqr);
    wait_clocks(64);

    err_seq = single_error_hit_seq::type_id::create("cross_error");
    err_seq.search_key = 12;
    err_seq.start(m_env.m_hit_seqr);
    wait_clocks(32);

    seq_keys = sequential_keys_seq::type_id::create("cross_good_b");
    seq_keys.num_keys = 4;
    seq_keys.hits_per_key = 4;
    seq_keys.start_key = 8;
    seq_keys.start(m_env.m_hit_seqr);

    wait_for_scoreboard_idle(100_000, "CROSS-007 GOOD-ERROR-GOOD");
    `uvm_info("CASE", "CASE_END run=CROSS-007", UVM_LOW)
  endtask

  task automatic run_cross_good_error_good_overwrite();
    same_key_burst_seq   burst_seq;
    single_error_hit_seq err_seq;
    overwrite_profile_seq pressure_seq;
    int unsigned          overwrite_count;
    int unsigned          pre_service_pushes;
    bit [7:0]            focus_search_key;

    `uvm_info("CASE", "CASE_BEGIN run=CROSS-010", UVM_LOW)
    configure_and_start(2000);

    burst_seq = same_key_burst_seq::type_id::create("cross010_good_a");
    burst_seq.num_hits = 32;
    burst_seq.search_key = 8;
    burst_seq.start(m_env.m_hit_seqr);
    wait_clocks(32);

    err_seq = single_error_hit_seq::type_id::create("cross010_err");
    err_seq.search_key = 12;
    err_seq.start(m_env.m_hit_seqr);
    wait_clocks(16);

    pressure_seq = overwrite_profile_seq::type_id::create("cross010_pressure");
    pressure_seq.num_hits = 736;
    pressure_seq.lane_key_start_ord = 2;
    pressure_seq.pool_keys = 1;
    pressure_seq.hits_per_key_switch = 1;
    pressure_seq.progress_stride = 64;
    pressure_seq.progress_tag = "CROSS-010 overwrite window";
    pressure_seq.start(m_env.m_hit_seqr);

    burst_seq = same_key_burst_seq::type_id::create("cross010_good_b");
    burst_seq.num_hits = 16;
    burst_seq.search_key = 16;
    burst_seq.start(m_env.m_hit_seqr);

    terminate_and_drain(120_000, "CROSS-010 GOOD-ERROR-GOOD overwrite");
    expect_service_model_accounting("CROSS-010 GOOD-ERROR-GOOD overwrite", 1, 1);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    pre_service_pushes = m_env.m_dbg_mon.push_count_at_first_pop;
    if (m_env.m_scb.max_remaining_entries() < m_cfg.ring_buffer_n_entry) begin
      `uvm_error("CROSS-010", $sformatf(
        "Overwrite window never saturated the rbCAM: max_remaining=%0d ring_depth=%0d",
        m_env.m_scb.max_remaining_entries(), m_cfg.ring_buffer_n_entry))
    end
    if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < m_cfg.ring_buffer_n_entry) begin
      `uvm_error("CROSS-010", $sformatf(
        "Hotspot key did not saturate the full sector depth: key=%0d max_for_key=%0d ring_depth=%0d",
        focus_search_key,
        m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
        m_cfg.ring_buffer_n_entry))
    end
    if (pre_service_pushes <= m_cfg.ring_buffer_n_entry) begin
      `uvm_error("CROSS-010", $sformatf(
        "Overwrite window did not exceed sector depth before first pop service: pushes_before_first_pop=%0d ring_depth=%0d",
        pre_service_pushes, m_cfg.ring_buffer_n_entry))
    end
    `uvm_info("CROSS-010", $sformatf(
      "network-calculus summary: overwrite=%0d pushes_before_first_pop=%0d max_remaining=%0d hotspot_max=%0d",
      overwrite_count, pre_service_pushes,
      m_env.m_scb.max_remaining_entries(),
      m_env.m_scb.max_remaining_entries_for_key(focus_search_key)), UVM_LOW)
    csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "CROSS-010 INERR isolated to error window");
    `uvm_info("CASE", "CASE_END run=CROSS-010", UVM_LOW)
  endtask

  function void report_phase(uvm_phase phase);
    uvm_report_server srv = uvm_report_server::get_server();
    if (srv.get_severity_count(UVM_ERROR) > 0 ||
        srv.get_severity_count(UVM_FATAL) > 0)
      `uvm_info("TEST", "*** TEST FAILED ***", UVM_NONE)
    else
      `uvm_info("TEST", "*** TEST PASSED ***", UVM_NONE)
  endfunction
endclass

// ═══════════════════════════════════════════════════════════════
// Directed tests
// ═══════════════════════════════════════════════════════════════

// D001: single push-pop
class test_single_push_pop extends base_test;
  `uvm_component_utils(test_single_push_pop)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    single_push_pop_seq seq;
    phase.raise_objection(this);
    wait_for_reset_release();
    configure_and_start();
    seq = single_push_pop_seq::type_id::create("seq");
    seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(20_000, "test_single_push_pop");
    phase.drop_objection(this);
  endtask
endclass

// D003: same-key burst 128
class test_same_key_burst_128 extends base_test;
  `uvm_component_utils(test_same_key_burst_128)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    same_key_burst_seq seq;
    phase.raise_objection(this);
    wait_for_reset_release();
    configure_and_start();
    seq = same_key_burst_seq::type_id::create("seq");
    seq.num_hits   = 128;
    seq.search_key = 8;
    seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(40_000, "test_same_key_burst_128");
    phase.drop_objection(this);
  endtask
endclass

// D003: same-key burst 256
class test_same_key_burst_256 extends base_test;
  `uvm_component_utils(test_same_key_burst_256)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    same_key_burst_seq seq;
    phase.raise_objection(this);
    wait_for_reset_release();
    configure_and_start();
    seq = same_key_burst_seq::type_id::create("seq");
    seq.num_hits   = 256;
    seq.search_key = 12;
    seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(60_000, "test_same_key_burst_256");
    phase.drop_objection(this);
  endtask
endclass

// D004: sequential keys
class test_sequential_keys extends base_test;
  `uvm_component_utils(test_sequential_keys)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    sequential_keys_seq seq;
    phase.raise_objection(this);
    wait_for_reset_release();
    configure_and_start();
    seq = sequential_keys_seq::type_id::create("seq");
    seq.num_keys     = 16;
    seq.hits_per_key = 8;
    seq.start_key    = 4;
    seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(80_000, "test_sequential_keys");
    phase.drop_objection(this);
  endtask
endclass

// D006: overwrite stress
class test_overwrite_stress extends base_test;
  `uvm_component_utils(test_overwrite_stress)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    overwrite_stress_seq seq;
    phase.raise_objection(this);
    wait_for_reset_release();
    configure_and_start();
    seq = overwrite_stress_seq::type_id::create("seq");
    seq.num_hits = 1024;
    seq.start(m_env.m_hit_seqr);
    terminate_and_drain(100_000, "test_overwrite_stress");
    expect_service_model_accounting("test_overwrite_stress", 1, 1);
    phase.drop_objection(this);
  endtask
endclass

// ═══════════════════════════════════════════════════════════════
// Constrained-random tests
// ═══════════════════════════════════════════════════════════════

// R001: random push-pop
class test_random_push_pop extends base_test;
  `uvm_component_utils(test_random_push_pop)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    random_push_pop_seq seq;
    phase.raise_objection(this);
    wait_for_reset_release();
    configure_and_start();
    seq = random_push_pop_seq::type_id::create("seq");
    assert(seq.randomize());
    `uvm_info("TEST", $sformatf("R001: hits=%0d key=%0d delay=%0d",
      seq.num_hits, seq.search_key, seq.inter_hit_delay), UVM_LOW)
    seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(80_000, "test_random_push_pop");
    phase.drop_objection(this);
  endtask
endclass

// R002: random multi-key
class test_random_multi_key extends base_test;
  `uvm_component_utils(test_random_multi_key)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    random_multi_key_seq seq;
    phase.raise_objection(this);
    wait_for_reset_release();
    configure_and_start();
    seq = random_multi_key_seq::type_id::create("seq");
    assert(seq.randomize());
    `uvm_info("TEST", $sformatf("R002: keys=%0d hits_per_key=%0d",
      seq.num_keys, seq.hits_per_key), UVM_LOW)
    seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(100_000, "test_random_multi_key");
    phase.drop_objection(this);
  endtask
endclass

// R006: random throughput
class test_random_throughput extends base_test;
  `uvm_component_utils(test_random_throughput)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    random_throughput_seq seq;
    phase.raise_objection(this);
    wait_for_reset_release();
    configure_and_start();
    seq = random_throughput_seq::type_id::create("seq");
    assert(seq.randomize());
    `uvm_info("TEST", $sformatf("R006: hits=%0d key=%0d",
      seq.num_hits, seq.search_key), UVM_LOW)
    seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(120_000, "test_random_throughput");
    phase.drop_objection(this);
  endtask
endclass

class test_cfg_reset_defaults extends base_test;
  `uvm_component_utils(test_cfg_reset_defaults)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    wait_for_reset_release();
    csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID reset default");
    csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0013, 32'h0000_0011, "CTRL reset defaults");
    csr_expect_mask(CSR_EXPECTED_LAT_ADDR, 32'h0000_FFFF, 32'd2000, "EXPECTED_LATENCY reset default");
    csr_expect_mask(CSR_FILL_LEVEL_ADDR, 32'hFFFF_FFFF, 32'd0, "FILL_LEVEL reset default");
    for (int addr = CSR_INERR_COUNT_ADDR; addr <= CSR_CACHE_MISS_ADDR; addr++) begin
      csr_expect_mask(addr, 32'hFFFF_FFFF, 32'd0,
        $sformatf("Debug counter reset default addr=%0d", addr));
    end
    phase.drop_objection(this);
  endtask
endclass

class test_cfg_rw_semantics extends base_test;
  `uvm_component_utils(test_cfg_rw_semantics)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    wait_for_reset_release();

    csr_write(CSR_CTRL_ADDR, 32'h0000_0000);
    wait_clocks(2);
    csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0013, 32'h0000_0000, "CTRL writable bits clear");

    csr_write(CSR_CTRL_ADDR, 32'h0000_0011);
    wait_clocks(2);
    csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0013, 32'h0000_0011, "CTRL writable bits set");

    csr_write(CSR_CTRL_ADDR, 32'h0000_0013);
    wait_clocks(2);
    csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0013, 32'h0000_0011, "CTRL soft_reset self-clears");

    csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0123);
    wait_clocks(2);
    csr_expect_mask(CSR_EXPECTED_LAT_ADDR, 32'h0000_FFFF, 32'h0000_0123, "EXPECTED_LATENCY writable");

    for (int addr = CSR_FILL_LEVEL_ADDR; addr <= CSR_CACHE_MISS_ADDR; addr++) begin
      logic [31:0] before_data;
      csr_read(addr, before_data);
      csr_write(addr, 32'hCAFE_0000 | addr);
      wait_clocks(2);
      csr_expect_mask(addr, 32'hFFFF_FFFF, before_data,
        $sformatf("RO CSR unchanged after write addr=%0d", addr));
    end

    phase.drop_objection(this);
  endtask
endclass

class test_cfg_activity_counters extends base_test;
  `uvm_component_utils(test_cfg_activity_counters)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    wait_for_reset_release();
    run_b003_activity_case();
    phase.drop_objection(this);
  endtask
endclass

class test_case_engine extends base_test;
  `uvm_component_utils(test_case_engine)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    string case_id;
    string exec_mode;
    string bucket_name;
    string run_id;

    phase.raise_objection(this);
    wait_for_reset_release();

    if (!$value$plusargs("DV_EXEC_MODE=%s", exec_mode)) begin
      exec_mode = "isolated";
    end

    if (exec_mode == "isolated") begin
      if (!$value$plusargs("DV_CASE_ID=%s", case_id)) begin
        `uvm_fatal("CASE_ENGINE", "DV_CASE_ID=<case_id> is required in isolated mode")
      end
      `uvm_info("CASE", $sformatf("CASE_BEGIN id=%s mode=isolated", case_id), UVM_LOW)
      run_case_by_id(case_id);
      `uvm_info("CASE", $sformatf("CASE_END id=%s mode=isolated", case_id), UVM_LOW)
    end else if (exec_mode == "bucket_frame") begin
      if (!$value$plusargs("DV_BUCKET=%s", bucket_name)) begin
        `uvm_fatal("CASE_ENGINE", "DV_BUCKET=<bucket> is required in bucket_frame mode")
      end
      if (bucket_name == "BASIC") begin
        run_basic_bucket_frame();
      end else begin
        `uvm_fatal("CASE_ENGINE", $sformatf("bucket_frame for %s is not implemented yet", bucket_name))
      end
    end else if (exec_mode == "cross") begin
      if (!$value$plusargs("DV_RUN_ID=%s", run_id)) begin
        `uvm_fatal("CASE_ENGINE", "DV_RUN_ID=<run_id> is required in cross mode")
      end
      if (run_id == "CROSS-007") begin
        run_cross_good_error_good();
      end else if (run_id == "CROSS-010") begin
        run_cross_good_error_good_overwrite();
      end else begin
        `uvm_fatal("CASE_ENGINE", $sformatf("cross run %s is not implemented yet", run_id))
      end
    end else begin
      `uvm_fatal("CASE_ENGINE", $sformatf("Unsupported DV_EXEC_MODE=%s", exec_mode))
    end

    phase.drop_objection(this);
  endtask
endclass

`endif

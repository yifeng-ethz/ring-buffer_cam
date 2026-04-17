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
  virtual avst_out_if.drv                   m_out_vif;

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
    void'(uvm_config_db#(virtual avst_out_if.drv)::get(this, "", "out_vif", m_out_vif));
    uvm_config_db#(ring_buffer_cam_pkg::ring_buffer_cam_cfg)::set(
      this, "m_env", "cfg", m_cfg);
    m_env = ring_buffer_cam_env::type_id::create("m_env", this);
  endfunction

  task automatic wait_clocks(int unsigned cycles);
    repeat (cycles) @(posedge m_env.m_csr_drv.vif.clk);
  endtask

  task automatic set_sink_ready(bit ready);
    if (m_out_vif != null) begin
      m_out_vif.ready <= ready;
    end
  endtask

  task automatic wait_for_reset_release();
    do begin
      @(posedge m_env.m_csr_drv.vif.clk);
    end while (m_env.m_csr_drv.vif.rst === 1'b1);
    m_cfg.note_reset_defaults();
    wait_clocks(2);
    set_sink_ready(1'b1);
  endtask

  task automatic wait_for_run_state(
    logic [3:0] target_state,
    int unsigned max_cycles = 20_000,
    string what = "run_state"
  );
    int unsigned cycles;
    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_dbg_mon.run_state_code == target_state) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: observed=%0d expected=%0d",
      what, max_cycles, m_env.m_dbg_mon.run_state_code, target_state))
  endtask

  task automatic wait_for_pop_engine_state(
    logic [2:0] target_state,
    int unsigned max_cycles = 20_000,
    string what = "pop_engine_state"
  );
    int unsigned cycles;
    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_dbg_mon.pop_engine_state_code == target_state) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: observed=%0d expected=%0d",
      what, max_cycles, m_env.m_dbg_mon.pop_engine_state_code, target_state))
  endtask

  task automatic wait_for_ctrl_ready(
    bit expected_ready,
    int unsigned max_cycles = 20_000,
    string what = "ctrl ready"
  );
    int unsigned cycles;
    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_ctrl_drv.vif.ready === expected_ready) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: observed=%0d expected=%0d",
      what, max_cycles, m_env.m_ctrl_drv.vif.ready, expected_ready))
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

  task automatic ctrl_pulse_raw(logic [8:0] cmd, int unsigned hold_cycles = 1);
    m_env.m_ctrl_drv.vif.data <= cmd;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    repeat (hold_cycles) @(posedge m_env.m_ctrl_drv.vif.clk);
    m_cfg.note_ctrl_cmd(cmd);
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_ctrl_drv.vif.clk);
  endtask

  task automatic enter_run_prepare();
    // RUN_PREPARE is acked immediately from IDLE/RUNNING. Re-issue it so the
    // driver waits on the DUT's real flush-complete handshake instead of a
    // fixed simulation delay.
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    m_env.m_scb.note_flush_reset();
  endtask

  task automatic configure_and_start_full(int unsigned latency = 128);
    enter_run_prepare();
    csr_write(CSR_EXPECTED_LAT_ADDR, latency);
    ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
    wait_clocks(2);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
    wait_clocks(20);
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
    configure_and_start_full(latency);
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
      if (m_env.m_scb.remaining_entries() == 0 &&
          m_env.m_scb.pending_drain_entries() == 0 &&
          m_env.m_scb.epoch_idle() &&
          m_env.m_scb.total_written == m_env.m_scb.total_ingress_accepted &&
          m_env.m_hit_drv.pending_source_items() == 0 &&
          m_env.m_dbg_mon.vif.deassembly_fifo_empty === 1'b1 &&
          m_env.m_dbg_mon.vif.pop_cmd_fifo_empty === 1'b1) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_info("TIMEOUT", $sformatf(
      "%s debug: remaining=%0d pending_drain=%0d max_remaining=%0d source_backlog=%0d source_backlog_max=%0d source_offered=%0d source_accepted=%0d live_fill=%0d max_live_fill=%0d push=%0d pop=%0d overwrite=%0d term_done=%0d endofrun_seen=%0d pop_cmd_empty=%0d pop_cmd_usedw=%0d deassm_empty=%0d deassm_full=%0d deassm_usedw=%0d first_push_cycle=%0d first_pop_cycle=%0d first_overwrite_cycle=%0d",
      what,
      m_env.m_scb.remaining_entries(),
      m_env.m_scb.pending_drain_entries(),
      m_env.m_scb.max_remaining_entries(),
      m_env.m_hit_drv.pending_source_items(),
      m_env.m_hit_drv.max_source_backlog,
      m_env.m_hit_drv.offered_payload_total,
      m_env.m_hit_drv.accepted_payload_total,
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
      "Timed out waiting for %s after %0d cycles: remaining=%0d pending_drain=%0d epoch_idle=%0d accepted=%0d written=%0d source_backlog=%0d",
      what, max_cycles, m_env.m_scb.remaining_entries(),
      m_env.m_scb.pending_drain_entries(), m_env.m_scb.epoch_idle(),
      m_env.m_scb.total_ingress_accepted, m_env.m_scb.total_written,
      m_env.m_hit_drv.pending_source_items()))
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

  task automatic restart_after_flush(int unsigned latency = 2000);
    enter_run_prepare();
    csr_write(CSR_EXPECTED_LAT_ADDR, latency);
    ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
    wait_clocks(2);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
    wait_clocks(20);
  endtask

  task automatic return_to_idle();
    ctrl_send(ring_buffer_cam_pkg::CTRL_IDLE);
    wait_clocks(4);
  endtask

  task automatic run_x116_good_error_good_case(
    string       what,
    int unsigned good_a_hits = 32,
    int unsigned bad_hits = 8,
    int unsigned good_b_hits = 32,
    int unsigned latency = 2000
  );
    same_key_burst_seq burst_seq;
    error_burst_seq    err_burst;
    int unsigned       push_count;
    int unsigned       pop_count;
    int unsigned       inerr_count;
    int unsigned       fill_level;

    configure_and_start(latency);

    burst_seq = same_key_burst_seq::type_id::create({what, "_good_a"});
    burst_seq.num_hits = good_a_hits;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    err_burst = error_burst_seq::type_id::create({what, "_bad"});
    err_burst.num_hits = bad_hits;
    err_burst.search_key = m_cfg.lane_key_ord_to_search_key(3);
    err_burst.start(m_env.m_hit_seqr);
    wait_clocks(32);

    burst_seq = same_key_burst_seq::type_id::create({what, "_good_b"});
    burst_seq.num_hits = good_b_hits;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(4);
    burst_seq.start(m_env.m_hit_seqr);

    terminate_and_drain(180_000, {what, " terminate drain"});
    expect_service_model_accounting({what, " terminate drain"}, 1, 0);

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (push_count != (good_a_hits + good_b_hits) ||
        pop_count != (good_a_hits + good_b_hits) ||
        inerr_count != bad_hits ||
        fill_level != 0) begin
      `uvm_error("RECOVERY", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d inerr=%0d fill=%0d expected_push_pop=%0d expected_inerr=%0d",
        what, push_count, pop_count, inerr_count, fill_level, good_a_hits + good_b_hits, bad_hits))
    end
  endtask

  task automatic run_x117_good_error_flush_good_case(
    string       what,
    int unsigned good_a_hits = 32,
    int unsigned bad_hits = 8,
    int unsigned good_b_hits = 32,
    int unsigned latency = 2000
  );
    same_key_burst_seq burst_seq;
    error_burst_seq    err_burst;
    int unsigned       push_count;
    int unsigned       pop_count;
    int unsigned       inerr_count;
    int unsigned       overwrite_count;
    int unsigned       cache_miss_count;
    int unsigned       fill_level;

    configure_and_start(latency);

    burst_seq = same_key_burst_seq::type_id::create({what, "_good_a"});
    burst_seq.num_hits = good_a_hits;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    err_burst = error_burst_seq::type_id::create({what, "_bad"});
    err_burst.num_hits = bad_hits;
    err_burst.search_key = m_cfg.lane_key_ord_to_search_key(3);
    err_burst.start(m_env.m_hit_seqr);
    wait_clocks(32);
    restart_after_flush(latency);

    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    if (inerr_count != 0 || overwrite_count != 0 || cache_miss_count != 0) begin
      `uvm_error("RECOVERY", $sformatf(
        "%s flush did not clear transient error counters: inerr=%0d overwrite=%0d cache_miss=%0d",
        what, inerr_count, overwrite_count, cache_miss_count))
    end

    burst_seq = same_key_burst_seq::type_id::create({what, "_good_b"});
    burst_seq.num_hits = good_b_hits;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(4);
    burst_seq.start(m_env.m_hit_seqr);

    terminate_and_drain(180_000, {what, " restart drain"});
    expect_service_model_accounting({what, " restart drain"}, 1, 0);

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (push_count != good_b_hits || pop_count != good_b_hits || fill_level != 0) begin
      `uvm_error("RECOVERY", $sformatf(
        "%s restart accounting mismatch: push=%0d pop=%0d fill=%0d expected=%0d",
        what, push_count, pop_count, fill_level, good_b_hits))
    end
  endtask

  task automatic run_x118_good_terminate_restart_case(
    string       what,
    int unsigned first_hits = 96,
    int unsigned second_hits = 32,
    int unsigned latency = 2000
  );
    same_key_burst_seq burst_seq;
    int unsigned       push_count;
    int unsigned       pop_count;
    int unsigned       fill_level;

    configure_and_start(latency);

    burst_seq = same_key_burst_seq::type_id::create({what, "_first"});
    burst_seq.num_hits = first_hits;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_clocks(4);

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    wait_for_ctrl_ready(1'b1, 40_000, {what, " first terminate ack"});
    wait_for_scoreboard_idle(180_000, {what, " first terminate drain"});

    return_to_idle();
    restart_after_flush(latency);
    if (m_env.m_dbg_mon.vif.endofrun_seen !== 1'b0) begin
      `uvm_error("RECOVERY", $sformatf("%s restart inherited endofrun_seen", what))
    end

    burst_seq = same_key_burst_seq::type_id::create({what, "_second"});
    burst_seq.num_hits = second_hits;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(4);
    burst_seq.start(m_env.m_hit_seqr);

    terminate_and_drain(180_000, {what, " second terminate drain"});
    expect_service_model_accounting({what, " second terminate drain"}, 1, 0);

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (push_count != second_hits || pop_count != second_hits || fill_level != 0) begin
      `uvm_error("RECOVERY", $sformatf(
        "%s second-run accounting mismatch: push=%0d pop=%0d fill=%0d expected=%0d",
        what, push_count, pop_count, fill_level, second_hits))
    end
  endtask

  task automatic run_cross_curated_all_bucket_mix();
    same_key_burst_seq   burst_seq;
    random_push_pop_seq  rand_same;
    overwrite_profile_seq pressure_seq;
    error_burst_seq      err_burst;
    int unsigned         inerr_count;
    int unsigned         push_count;
    int unsigned         pop_count;

    `uvm_info("CASE", "CASE_BEGIN run=CROSS-015", UVM_LOW)
    configure_and_start(2000);

    for (int round = 0; round < 3; round++) begin
      burst_seq = same_key_burst_seq::type_id::create($sformatf("cross015_basic_%0d", round));
      burst_seq.num_hits = (round == 0) ? 128 : 256;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2 + round);
      burst_seq.start(m_env.m_hit_seqr);
      wait_clocks($urandom_range(0, 64));

      burst_seq = same_key_burst_seq::type_id::create($sformatf("cross015_edge_%0d", round));
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key((4 + round) % 8);
      burst_seq.start(m_env.m_hit_seqr);
      wait_clocks($urandom_range(0, 128));

      rand_same = random_push_pop_seq::type_id::create($sformatf("cross015_prof_%0d", round));
      rand_same.num_hits = 4096;
      rand_same.search_key = (8 + (round * 7)) % 60;
      rand_same.inter_hit_delay = round[1:0];
      rand_same.start(m_env.m_hit_seqr);
      wait_clocks($urandom_range(0, 200));

      err_burst = error_burst_seq::type_id::create($sformatf("cross015_err_%0d", round));
      err_burst.num_hits = 32;
      err_burst.search_key = m_cfg.lane_key_ord_to_search_key((6 + round) % 8);
      err_burst.start(m_env.m_hit_seqr);
      wait_clocks(16);

      restart_after_flush(2000);

      pressure_seq = overwrite_profile_seq::type_id::create($sformatf("cross015_pressure_%0d", round));
      pressure_seq.num_hits = 2048;
      pressure_seq.lane_key_start_ord = 2 + round;
      pressure_seq.pool_keys = 1 + (round % 2);
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.progress_stride = 256;
      pressure_seq.progress_tag = $sformatf("CROSS-015 round %0d", round);
      pressure_seq.start(m_env.m_hit_seqr);
      wait_clocks($urandom_range(0, 200));
    end

    terminate_and_drain(400_000, "CROSS-015 curated all-bucket mix");
    expect_service_model_accounting("CROSS-015 curated all-bucket mix", 1, 1);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    `uvm_info("CROSS-015", $sformatf(
      "summary: inerr=%0d push=%0d pop=%0d overwrites=%0d max_remaining=%0d cache_miss_outputs=%0d",
      inerr_count, push_count, pop_count, m_env.m_dbg_mon.dbg_overwrite_cnt[31:0],
      m_env.m_scb.max_remaining_entries(), m_env.m_scb.total_cache_miss_outputs), UVM_LOW)
    `uvm_info("CASE", "CASE_END run=CROSS-015", UVM_LOW)
  endtask

  task automatic run_cross_inerr_toggle_longrun();
    error_burst_seq err_burst;
    int unsigned    inerr_count;

    `uvm_info("CASE", "CASE_BEGIN run=CROSS-091", UVM_LOW)
    configure_and_start(2000);

    err_burst = error_burst_seq::type_id::create("cross091_inerr_burst");
    err_burst.num_hits = 131072;
    err_burst.search_key = m_cfg.lane_key_ord_to_search_key(2);
    err_burst.start(m_env.m_hit_seqr);
    wait_clocks(64);

    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "CROSS-091 INERR frontdoor/backdoor match");
    if (inerr_count != 131072) begin
      `uvm_error("CROSS-091", $sformatf(
        "Long-run INERR counter mismatch: observed=%0d expected=%0d",
        inerr_count, 131072))
    end
    wait_for_scoreboard_idle(60_000, "CROSS-091 inerr long run");
    `uvm_info("CROSS-091", $sformatf(
      "summary: inerr=%0d cache_miss=%0d push=%0d pop=%0d",
      inerr_count,
      m_env.m_dbg_mon.dbg_cache_miss_cnt[31:0],
      m_env.m_dbg_mon.dbg_push_cnt[31:0],
      m_env.m_dbg_mon.dbg_pop_cnt[31:0]), UVM_LOW)
    `uvm_info("CASE", "CASE_END run=CROSS-091", UVM_LOW)
  endtask

  task automatic run_cross_overwrite_toggle_longrun();
    `uvm_info("CASE", "CASE_BEGIN run=CROSS-076", UVM_LOW)
    run_single_key_overwrite_window("CROSS-076 overwrite toggle soak", 131072);
    `uvm_info("CASE", "CASE_END run=CROSS-076", UVM_LOW)
  endtask

  task automatic terminate_and_drain(
    int unsigned max_cycles = 250_000,
    string what = "terminate drain"
  );
    int unsigned source_wait_cycles;
    int unsigned term_wait_cycles;
    source_wait_cycles = 0;
    while (m_env.m_hit_drv.pending_source_items() != 0 && source_wait_cycles < max_cycles) begin
      @(posedge m_env.m_csr_drv.vif.clk);
      source_wait_cycles++;
    end
    if (m_env.m_hit_drv.pending_source_items() != 0) begin
      `uvm_error("TERM", $sformatf(
        "%s: source backlog did not drain before TERMINATING: backlog=%0d after %0d cycles",
        what, m_env.m_hit_drv.pending_source_items(), source_wait_cycles))
    end else begin
      `uvm_info("TERM", $sformatf(
        "%s: source backlog drained before TERMINATING after %0d cycles (offered=%0d accepted=%0d)",
        what, source_wait_cycles,
        m_env.m_hit_drv.offered_payload_total, m_env.m_hit_drv.accepted_payload_total), UVM_LOW)
    end
    `uvm_info("TERM", $sformatf("%s: issue TERMINATING entry pulse", what), UVM_LOW)
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    `uvm_info("TERM", $sformatf("%s: send end-of-run marker", what), UVM_LOW)
    send_endofrun_marker();
    `uvm_info("TERM", $sformatf("%s: poll TERMINATING drain-done", what), UVM_LOW)
    term_wait_cycles = 0;
    while (m_env.m_dbg_mon.terminating_drain_done !== 1'b1 && term_wait_cycles < max_cycles) begin
      @(posedge m_env.m_csr_drv.vif.clk);
      term_wait_cycles++;
    end
    if (m_env.m_dbg_mon.terminating_drain_done !== 1'b1) begin
      `uvm_error("TERM", $sformatf(
        "%s: terminating_drain_done did not assert after %0d cycles: endofrun_seen=%0d deassm_empty=%0d pop_cmd_empty=%0d push=%0d pop=%0d overwrite=%0d",
        what, term_wait_cycles,
        m_env.m_dbg_mon.vif.endofrun_seen,
        m_env.m_dbg_mon.vif.deassembly_fifo_empty,
        m_env.m_dbg_mon.vif.pop_cmd_fifo_empty,
        m_env.m_dbg_mon.dbg_push_cnt[31:0],
        m_env.m_dbg_mon.dbg_pop_cnt[31:0],
        m_env.m_dbg_mon.dbg_overwrite_cnt[31:0]))
    end else begin
      `uvm_info("TERM", $sformatf(
        "%s: terminating_drain_done asserted after %0d cycles",
        what, term_wait_cycles), UVM_LOW)
    end
    wait_clocks(4);
    `uvm_info("TERM", $sformatf("%s: waiting for scoreboard/source idle", what), UVM_LOW)
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
    int unsigned offered_before_first_pop;
    int unsigned accepted_before_first_pop;
    int unsigned pre_service_pushes;
    int unsigned backlog_before_first_pop;
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
    `uvm_info("PRESSURE", $sformatf("%s: start pressure source num_hits=%0d", what, num_hits), UVM_LOW)
    pressure_seq.start(m_env.m_hit_seqr);
    `uvm_info("PRESSURE", $sformatf("%s: pressure source completed offered=%0d accepted=%0d backlog=%0d",
      what, m_env.m_hit_drv.offered_payload_total, m_env.m_hit_drv.accepted_payload_total,
      m_env.m_hit_drv.pending_source_items()), UVM_LOW)
    terminate_and_drain(80_000, {what, " terminate drain"});
    expect_service_model_accounting({what, " post-terminate"}, 1, 1);

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
    offered_before_first_pop = m_env.m_hit_drv.offered_payload_at_first_pop;
    accepted_before_first_pop = m_env.m_hit_drv.accepted_payload_at_first_pop;
    pre_service_pushes = m_env.m_dbg_mon.push_count_at_first_pop;
    backlog_before_first_pop = m_env.m_hit_drv.backlog_at_first_pop;

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
    if (offered_before_first_pop <= m_cfg.ring_buffer_n_entry) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s source did not offer more than the 512-entry sector depth before first pop: offered_before_first_pop=%0d ring_depth=%0d",
        what, offered_before_first_pop, m_cfg.ring_buffer_n_entry))
    end
    if (accepted_before_first_pop <= m_cfg.ring_buffer_n_entry) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s frontdoor did not admit more than the 512-entry sector depth before first pop: accepted_before_first_pop=%0d ring_depth=%0d backlog_before_first_pop=%0d",
        what, accepted_before_first_pop, m_cfg.ring_buffer_n_entry, backlog_before_first_pop))
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
      "%s network-calculus summary: offered_before_first_pop=%0d accepted_before_first_pop=%0d pushes_before_first_pop=%0d backlog_before_first_pop=%0d push=%0d overwrite=%0d max_remaining=%0d max_hotspot=%0d max_source_backlog=%0d first_offer_cycle=%0d first_push_cycle=%0d first_overwrite_cycle=%0d first_pop_cycle=%0d",
      what, offered_before_first_pop, accepted_before_first_pop,
      pre_service_pushes, backlog_before_first_pop,
      push_count, overwrite_count,
      m_env.m_scb.max_remaining_entries(),
      m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
      m_env.m_hit_drv.max_source_backlog,
      m_env.m_hit_drv.first_offer_cycle,
      m_env.m_dbg_mon.first_push_cycle,
      m_env.m_dbg_mon.first_overwrite_cycle,
      m_env.m_dbg_mon.first_pop_cycle), UVM_LOW)
  endtask

  task automatic run_same_key_epoch_count_case(
    string what,
    int unsigned num_hits,
    int unsigned key_ord,
    int unsigned expected_hit_count_field,
    int unsigned latency = 2000,
    int unsigned timeout_cycles = 200_000,
    bit check_pre_drain_fill = 0,
    int unsigned expected_pre_drain_fill = 0
  );
    same_key_burst_seq burst_seq;
    ring_buffer_cam_pkg::out_seq_item matched_subheader;
    int unsigned focus_search_key;
    int unsigned fill_level;

    configure_and_start(latency);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(key_ord);
    burst_seq = same_key_burst_seq::type_id::create({what, "_burst"});
    burst_seq.num_hits = num_hits;
    burst_seq.search_key = focus_search_key;
    burst_seq.start(m_env.m_hit_seqr);

    if (check_pre_drain_fill) begin
      wait_for_push_count(num_hits, timeout_cycles / 4, {what, " fill pre-check"});
      wait_clocks(4);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (fill_level != expected_pre_drain_fill) begin
        `uvm_error("SAME_KEY", $sformatf(
          "%s pre-drain fill mismatch: observed=%0d expected=%0d",
          what, fill_level, expected_pre_drain_fill))
      end
    end

    wait_for_subheader_search_key(
      focus_search_key[7:0], timeout_cycles, {what, " target subheader"}, matched_subheader);
    if (matched_subheader == null ||
        matched_subheader.hit_count != expected_hit_count_field[7:0]) begin
      `uvm_error("SAME_KEY", $sformatf(
        "%s subheader hit_count mismatch: observed=%s expected=0x%02x (%0d hits)",
        what,
        (matched_subheader == null) ? "none" : $sformatf("0x%02x", matched_subheader.hit_count),
        expected_hit_count_field[7:0], num_hits))
    end

    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
  endtask

  function automatic logic [31:0] expected_meta_version();
    return 32'h1A01_51A2;
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

  task automatic wait_for_hit_output_count(
    int unsigned target_count,
    int unsigned max_cycles = 50_000,
    string what = "hit emission"
  );
    int unsigned cycles;
    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_out_mon.total_hits_seen >= target_count) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: seen=%0d target=%0d",
      what, max_cycles, m_env.m_out_mon.total_hits_seen, target_count))
  endtask

  task automatic wait_for_subheader_search_key(
    bit [7:0] expected_search_key,
    int unsigned max_cycles = 50_000,
    string what = "matching subheader",
    output ring_buffer_cam_pkg::out_seq_item matched_subheader
  );
    int unsigned cycles;
    int unsigned seen_before;
    matched_subheader = null;
    cycles = 0;
    seen_before = m_env.m_out_mon.total_subheaders_seen;
    while (cycles < max_cycles) begin
      if (m_env.m_out_mon.total_subheaders_seen > seen_before &&
          m_env.m_out_mon.recent_subheaders.size() > 0) begin
        matched_subheader = m_env.m_out_mon.recent_subheaders[$];
        seen_before = m_env.m_out_mon.total_subheaders_seen;
        if (matched_subheader.search_key == expected_search_key) begin
          return;
        end
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: expected_search_key=0x%02x total_seen=%0d",
      what, max_cycles, expected_search_key, m_env.m_out_mon.total_subheaders_seen))
  endtask

  task automatic wait_for_subheader_match(
    input bit [7:0] expected_search_key,
    input bit [7:0] expected_hit_count,
    input bit check_hit_count = 0,
    input bit require_nonzero_hit_count = 0,
    input int unsigned max_cycles = 50_000,
    input string what = "matching subheader",
    output ring_buffer_cam_pkg::out_seq_item matched_subheader
  );
    int unsigned cycles;
    int unsigned seen_before;
    matched_subheader = null;
    cycles = 0;
    seen_before = require_nonzero_hit_count ?
      m_env.m_out_mon.total_data_subheaders_seen :
      m_env.m_out_mon.total_subheaders_seen;
    while (cycles < max_cycles) begin
      if (require_nonzero_hit_count &&
          m_env.m_out_mon.total_data_subheaders_seen > seen_before &&
          m_env.m_out_mon.recent_data_subheaders.size() > 0) begin
        matched_subheader = m_env.m_out_mon.recent_data_subheaders[$];
        seen_before = m_env.m_out_mon.total_data_subheaders_seen;
      end else if (!require_nonzero_hit_count &&
          m_env.m_out_mon.total_subheaders_seen > seen_before &&
          m_env.m_out_mon.recent_subheaders.size() > 0) begin
        matched_subheader = m_env.m_out_mon.recent_subheaders[$];
        seen_before = m_env.m_out_mon.total_subheaders_seen;
      end else begin
        @(posedge m_env.m_csr_drv.vif.clk);
        cycles++;
        continue;
      end
        if (matched_subheader.search_key != expected_search_key) begin
          @(posedge m_env.m_csr_drv.vif.clk);
          cycles++;
          continue;
        end
        if (require_nonzero_hit_count && matched_subheader.hit_count == 0) begin
          @(posedge m_env.m_csr_drv.vif.clk);
          cycles++;
          continue;
        end
        if (check_hit_count && matched_subheader.hit_count != expected_hit_count) begin
          @(posedge m_env.m_csr_drv.vif.clk);
          cycles++;
          continue;
        end
        return;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: expected_search_key=0x%02x expected_hit_count=%0d total_seen=%0d",
      what, max_cycles, expected_search_key, expected_hit_count,
      require_nonzero_hit_count ? m_env.m_out_mon.total_data_subheaders_seen :
        m_env.m_out_mon.total_subheaders_seen))
  endtask

  task automatic wait_for_push_count(
    int unsigned target_count,
    int unsigned max_cycles = 50_000,
    string what = "push-count target"
  );
    int unsigned cycles;
    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_dbg_mon.dbg_push_cnt[31:0] >= target_count) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: push=%0d target=%0d overwrite=%0d fill=%0d",
      what, max_cycles, m_env.m_dbg_mon.dbg_push_cnt[31:0], target_count,
      m_env.m_dbg_mon.dbg_overwrite_cnt[31:0], m_env.m_dbg_mon.current_live_fill))
  endtask

  task automatic wait_for_partition_issue_mask(
    input logic [3:0] expected_mask,
    output logic [3:0] observed_mask,
    input int unsigned max_cycles = 50_000,
    input string what = "partition issue mask"
  );
    int unsigned cycles;
    observed_mask = '0;
    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
        observed_mask[m_env.m_dbg_mon.pop_issue_partition_idx] = 1'b1;
        if ((observed_mask & expected_mask) == expected_mask) begin
          return;
        end
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: observed_mask=0x%0h expected_mask=0x%0h",
      what, max_cycles, observed_mask, expected_mask))
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
    error_burst_seq       err_burst;
    ring_buffer_cam_pkg::out_seq_item matched_subheader;
    logic [31:0]          data_a;
    logic [31:0]          data_b;
    logic [47:0]          dbg48_a;
    logic [47:0]          dbg48_b;
    int unsigned          subhdr_before;
    int unsigned          hit_before;
    int unsigned          ow16;
    int unsigned          ow20;
    int unsigned          ow24;
    int unsigned          ow28;
    int unsigned          ow_min;
    int unsigned          ow_max;
    int unsigned          push_count;
    int unsigned          pop_count;
    int unsigned          overwrite_count;
    int unsigned          cache_miss_count;
    int unsigned          fill_level;
    int unsigned          focus_search_key;
    int unsigned          cmd_before;
    int unsigned          cmd_after;
    int unsigned          expected_cmds;
    int unsigned          observed_cmds;
    int unsigned          load_pulses;
    int unsigned          expected_mask;
    int unsigned          search_cycles;
    int unsigned          wait_min;
    int unsigned          wait_max;
    logic [3:0]           load_mask;
    logic [1:0]           rr_before;
    bit                   saw_load;
    bit                   saw_overlap;
    bit                   saw_rr_advance;
    bit                   saw_preempt;
    bit                   traffic_done;
    longint unsigned      cycle_a;
    longint unsigned      cycle_b;

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
      configure_and_start_full();
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
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd20260417, "META DATE readback");
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
    end else if (case_id == "B020") begin
      configure_and_start(2000);
      wait_clocks(17_000);
      dbg48_a = m_env.m_dbg_mon.gts_8n - m_env.m_dbg_mon.read_time_ptr;
      if ((dbg48_a < 48'd1999) || (dbg48_a > 48'd2001)) begin
        `uvm_error("B020", $sformatf(
          "read_time_ptr delta before latency rewrite mismatch: observed=%0d expected~=2000",
          dbg48_a))
      end
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_4000);
      wait_clocks(4);
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'h4000) begin
        `uvm_error("B020", $sformatf(
          "expected_latency_48b did not reflect 0x4000 write: observed=0x%04x",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end
      dbg48_b = m_env.m_dbg_mon.gts_8n - m_env.m_dbg_mon.read_time_ptr;
      if ((dbg48_b < 48'd16383) || (dbg48_b > 48'd16385)) begin
        `uvm_error("B020", $sformatf(
          "read_time_ptr delta after latency rewrite mismatch: observed=%0d expected~=16384",
          dbg48_b))
      end
    end else if (case_id == "B021") begin
      configure_and_start(2000);
      burst_seq = same_key_burst_seq::type_id::create("b021_burst");
      burst_seq.num_hits = 8;
      burst_seq.search_key = 8;
      burst_seq.start(m_env.m_hit_seqr);
      wait_clocks(64);
      csr_expect_mask(CSR_FILL_LEVEL_ADDR, 32'hFFFF_FFFF, 32'd8, "FILL_LEVEL push-pop-overwrite identity");
      terminate_and_drain(80_000, "B021 post-check drain");
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
      terminate_and_drain(80_000, "B022 post-check drain");
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
      terminate_and_drain(80_000, "B023 post-check drain");
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
      terminate_and_drain(80_000, "B025 post-check drain");
    end else if (case_id == "B026") begin
      configure_and_start(2000);
      single_seq = single_push_pop_seq::type_id::create("b026_single");
      single_seq.search_key = 8;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(80_000, "B026 real hit plus zero-hit windows");
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      if (pop_count != 1) begin
        `uvm_error("B026", $sformatf(
          "POP_COUNT should only reflect the real drained hit: observed=%0d expected=1",
          pop_count))
      end
      if (cache_miss_count != 0) begin
        `uvm_error("B026", $sformatf(
          "CACHE_MISS_COUNT should remain 0 for zero-hit searches: observed=%0d",
          cache_miss_count))
      end
    end else if (case_id == "B027") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b027_wrap");
      burst_seq.num_hits = 513;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(513, 25_000, "B027 overwrite event");
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (overwrite_count != 1) begin
        `uvm_error("B027", $sformatf(
          "OVERWRITE_COUNT did not increment on decision_reg=1 wrap: observed=%0d expected=1",
          overwrite_count))
      end
      if (m_env.m_dbg_mon.first_overwrite_cycle == 0) begin
        `uvm_error("B027", "No overwrite event was observed in the debug monitor")
      end
      m_env.m_scb.note_intentional_nonempty_end(m_cfg.ring_buffer_n_entry);
    end else if (case_id == "B028") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b028_wrap");
      burst_seq.num_hits = 513;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(513, 25_000, "B028 overwrite event");
      csr_read(CSR_OVERWRITE_ADDR, data_a);
      csr_write(CSR_OVERWRITE_ADDR, 32'hFFFF_FFFF);
      wait_clocks(2);
      csr_read(CSR_OVERWRITE_ADDR, data_b);
      if (data_b !== data_a) begin
        `uvm_error("B028", $sformatf(
          "OVERWRITE_COUNT write-ignore violated: before=0x%08x after=0x%08x",
          data_a, data_b))
      end
      m_env.m_scb.note_intentional_nonempty_end(m_cfg.ring_buffer_n_entry);
    end else if (case_id == "B031") begin
      set_meta_sel(2'b01);
      csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID stable across META sel write");
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd20260417, "META DATE after selector write");
      csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID stable after META read");
    end else if (case_id == "B032") begin
      m_env.m_csr_drv.vif.address <= CSR_UID_ADDR[4:0];
      m_env.m_csr_drv.vif.writedata <= '0;
      m_env.m_csr_drv.vif.write <= 1'b0;
      m_env.m_csr_drv.vif.read <= 1'b0;
      @(posedge m_env.m_csr_drv.vif.clk);
      if (m_env.m_csr_drv.vif.waitrequest !== 1'b1) begin
        `uvm_error("B032", $sformatf(
          "CSR waitrequest should be high in idle, observed=%0d",
          m_env.m_csr_drv.vif.waitrequest))
      end
      m_env.m_csr_drv.vif.read <= 1'b1;
      @(posedge m_env.m_csr_drv.vif.clk);
      m_env.m_csr_drv.vif.read <= 1'b0;
      @(posedge m_env.m_csr_drv.vif.clk);
      if (m_env.m_csr_drv.vif.waitrequest !== 1'b0) begin
        `uvm_error("B032", $sformatf(
          "CSR waitrequest did not drop after read: observed=%0d",
          m_env.m_csr_drv.vif.waitrequest))
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      if (m_env.m_csr_drv.vif.waitrequest !== 1'b1) begin
        `uvm_error("B032", $sformatf(
          "CSR waitrequest did not return high in idle: observed=%0d",
          m_env.m_csr_drv.vif.waitrequest))
      end
    end else if (case_id == "B033") begin
      data_a = 0;
      m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_IDLE;
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (2) begin
        @(posedge m_env.m_ctrl_drv.vif.clk);
        if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
          data_a = 1;
        end
      end
      if (data_a == 0) begin
        `uvm_error("B033", "IDLE command was not acknowledged immediately from IDLE")
      end
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      @(posedge m_env.m_ctrl_drv.vif.clk);
      if (m_env.m_dbg_mon.run_state_code != 4'd0) begin
        `uvm_error("B033", $sformatf(
          "IDLE command did not leave DUT in IDLE: observed=%0d",
          m_env.m_dbg_mon.run_state_code))
      end
    end else if (case_id == "B034") begin
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_run_state(4'd1, 2_000, "B034 RUN_PREPARE entry");
      if (m_env.m_dbg_mon.pop_cmd_fifo_sclr !== 1'b1 ||
          m_env.m_dbg_mon.deassembly_fifo_sclr !== 1'b1) begin
        `uvm_error("B034", "RUN_PREPARE entry did not assert both FIFO sclr signals")
      end
    end else if (case_id == "B035") begin
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_run_state(4'd1, 2_000, "B035 RUN_PREPARE entry");
      if (m_env.m_ctrl_drv.vif.ready !== 1'b0) begin
        `uvm_error("B035", "RUN_PREPARE acknowledged before flush prerequisites completed")
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b1) begin
        `uvm_error("B035", "run_mgmt_flushed did not latch before PREP ready")
      end
    end else if (case_id == "B036") begin
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_run_state(4'd1, 2_000, "B036 RUN_PREPARE entry");
      wait_clocks(4);
      if (m_env.m_dbg_mon.run_mgmt_flush_memory_start !== 1'b1 &&
          m_env.m_dbg_mon.run_mgmt_flushed !== 1'b1) begin
        `uvm_error("B036", "run_mgmt_flush_memory_start never asserted on PREP entry")
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      if (m_env.m_dbg_mon.run_mgmt_flush_memory_start !== 1'b0 ||
          m_env.m_dbg_mon.run_mgmt_flushed !== 1'b1) begin
        `uvm_error("B036", "Flush handshake did not settle to flushed=1/start=0")
      end
    end else if (case_id == "B037") begin
      enter_run_prepare();
      ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
      wait_for_run_state(4'd2, 2_000, "B037 SYNC entry");
      if (m_env.m_dbg_mon.gts_counter_rst !== 1'b1 || m_env.m_dbg_mon.gts_8n != 0) begin
        `uvm_error("B037", $sformatf(
          "SYNC did not assert gts_counter_rst / hold gts_8n at 0: rst=%0d gts=%0d",
          m_env.m_dbg_mon.gts_counter_rst, m_env.m_dbg_mon.gts_8n))
      end
    end else if (case_id == "B038") begin
      enter_run_prepare();
      ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
      wait_for_run_state(4'd3, 2_000, "B038 RUNNING entry");
      wait_clocks(8);
      if (m_env.m_dbg_mon.gts_counter_rst !== 1'b0 || m_env.m_dbg_mon.gts_8n == 0) begin
        `uvm_error("B038", $sformatf(
          "RUNNING did not release gts counter reset: rst=%0d gts=%0d",
          m_env.m_dbg_mon.gts_counter_rst, m_env.m_dbg_mon.gts_8n))
      end
      if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b0) begin
        `uvm_error("B038", "RUNNING did not clear run_mgmt_flushed")
      end
    end else if (case_id == "B039") begin
      configure_and_start(2000);
      wait_clocks(32);
      dbg48_a = m_env.m_dbg_mon.gts_8n;
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
      wait_for_run_state(4'd4, 2_000, "B039 TERMINATING entry");
      dbg48_b = m_env.m_dbg_mon.gts_end_of_run;
      if ((dbg48_b < dbg48_a - 2) || (dbg48_b > dbg48_a + 2)) begin
        `uvm_error("B039", $sformatf(
          "gts_end_of_run snapshot mismatch: gts_before=%0d captured=%0d",
          dbg48_a, dbg48_b))
      end
    end else if (case_id == "B040") begin
      configure_and_start(2000);
      single_seq = single_push_pop_seq::type_id::create("b040_single");
      single_seq.search_key = 8;
      single_seq.start(m_env.m_hit_seqr);
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
      wait_for_run_state(4'd4, 2_000, "B040 TERMINATING entry");
      wait_clocks(8);
      if (m_env.m_ctrl_drv.vif.ready !== 1'b0) begin
        `uvm_error("B040", "TERMINATING ready asserted before endofrun/drain completion")
      end
      send_endofrun_marker();
      ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
      wait_for_scoreboard_idle(60_000, "B040 terminate drain");
    end else if (case_id == "B041") begin
      configure_and_start(2000);
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
      send_endofrun_marker();
      wait_clocks(8);
      if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
        `uvm_error("B041", "endofrun_seen did not latch before IDLE clear")
      end
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_IDLE);
      wait_clocks(2);
      if (m_env.m_dbg_mon.endofrun_seen !== 1'b0) begin
        `uvm_error("B041", "IDLE transition did not clear endofrun_seen")
      end
    end else if (case_id == "B042") begin
      data_a = 0;
      m_env.m_ctrl_drv.vif.data <= 9'b010000000;
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (2) begin
        @(posedge m_env.m_ctrl_drv.vif.clk);
        if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
          data_a = 1;
        end
      end
      if (data_a == 0) begin
        `uvm_error("B042", "RESET command was not acknowledged immediately")
      end
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      if (m_env.m_dbg_mon.run_state_code != 4'd7) begin
        `uvm_error("B042", $sformatf(
          "RESET command did not move DUT into RESET: observed=%0d",
          m_env.m_dbg_mon.run_state_code))
      end
    end else if (case_id == "B043") begin
      m_env.m_ctrl_drv.vif.data <= 9'b000000011;
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      if (m_env.m_dbg_mon.run_state_code != 4'd9) begin
        `uvm_error("B043", $sformatf(
          "Illegal multi-hot run-control payload did not enter ERROR: observed=%0d",
          m_env.m_dbg_mon.run_state_code))
      end
      if (m_env.m_ctrl_drv.vif.ready !== 1'b0) begin
        `uvm_error("B043", "Illegal multi-hot run-control payload should not ack")
      end
    end else if (case_id == "B044") begin
      enter_run_prepare();
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_clocks(2);
      if (m_env.m_dbg_mon.run_mgmt_flush_memory_start !== 1'b0 ||
          m_env.m_dbg_mon.run_mgmt_flushed !== 1'b1) begin
        `uvm_error("B044", "Duplicate RUN_PREPARE retriggered flush after flush completion")
      end
    end else if (case_id == "B045") begin
      enter_run_prepare();
      ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
      wait_clocks(2);
      dbg48_a = m_env.m_dbg_mon.gts_8n;
      if (m_env.m_dbg_mon.gts_counter_rst !== 1'b1 || dbg48_a != 0) begin
        `uvm_error("B045", "First SYNC did not hold gts reset active")
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
      wait_clocks(2);
      dbg48_b = m_env.m_dbg_mon.gts_8n;
      if (m_env.m_dbg_mon.gts_counter_rst !== 1'b1 || dbg48_b != 0) begin
        `uvm_error("B045", "Duplicate SYNC glitched gts_counter_rst")
      end
    end else if (case_id == "B046") begin
      enter_run_prepare();
      ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
      wait_clocks(4);
      if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b0) begin
        `uvm_error("B046", "RUNNING did not clear run_mgmt_flushed on first transition")
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
      wait_clocks(4);
      if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b0) begin
        `uvm_error("B046", "Duplicate RUNNING glitched run_mgmt_flushed")
      end
    end else if (case_id == "B049") begin
      m_env.m_ctrl_drv.vif.data <= 9'b000100000;
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      if (m_env.m_dbg_mon.run_state_code != 4'd5 || m_env.m_ctrl_drv.vif.ready !== 1'b0) begin
        `uvm_error("B049", $sformatf(
          "LINK_TEST should transition to unsupported state with ready low: state=%0d ready=%0d",
          m_env.m_dbg_mon.run_state_code, m_env.m_ctrl_drv.vif.ready))
      end
    end else if (case_id == "B050") begin
      m_env.m_ctrl_drv.vif.data <= 9'b100000000;
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      if (m_env.m_dbg_mon.run_state_code != 4'd8 || m_env.m_ctrl_drv.vif.ready !== 1'b0) begin
        `uvm_error("B050", $sformatf(
          "OUT_OF_DAQ should transition to unsupported state with ready low: state=%0d ready=%0d",
          m_env.m_dbg_mon.run_state_code, m_env.m_ctrl_drv.vif.ready))
      end
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
      terminate_and_drain(80_000, "B052 post-check drain");
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
      terminate_and_drain(80_000, "B054 post-check drain");
    end else if (case_id == "B055") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b055_wrap");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry + 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry + 1, 30_000, "B055 first overwrite");
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (overwrite_count != 1) begin
        `uvm_error("B055", $sformatf(
          "Expected exactly one overwrite at depth+1, observed=%0d",
          overwrite_count))
      end
      if (m_env.m_dbg_mon.push_erase_grant_count == 0 ||
          m_env.m_dbg_mon.first_overwrite_cycle == 0) begin
        `uvm_error("B055", $sformatf(
          "Did not observe the push ERASE path: erase_grants=%0d first_overwrite_cycle=%0d",
          m_env.m_dbg_mon.push_erase_grant_count,
          m_env.m_dbg_mon.first_overwrite_cycle))
      end
      m_env.m_scb.note_intentional_nonempty_end(m_cfg.ring_buffer_n_entry);
    end else if (case_id == "B056") begin
      configure_and_start();
      single_seq = single_push_pop_seq::type_id::create("b056_single");
      single_seq.search_key = 8;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd1, 40_000, "B056 SEARCH entry");
      search_cycles = 0;
      wait_min = 32'hffff_ffff;
      wait_max = 0;
      saw_load = 1'b0;
      while (search_cycles < 32 && !saw_load) begin
        if (m_env.m_dbg_mon.pop_engine_state_code == 3'd1) begin
          search_cycles++;
          if (m_env.m_dbg_mon.pop_search_wait_cnt < wait_min)
            wait_min = m_env.m_dbg_mon.pop_search_wait_cnt;
          if (m_env.m_dbg_mon.pop_search_wait_cnt > wait_max)
            wait_max = m_env.m_dbg_mon.pop_search_wait_cnt;
        end
        if (m_env.m_dbg_mon.pop_partition_load != 0)
          saw_load = 1'b1;
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      if (!saw_load || search_cycles != 6 || wait_min != 0 || wait_max != 5) begin
        `uvm_error("B056", $sformatf(
          "SEARCH latency mismatch: saw_load=%0d search_cycles=%0d wait_min=%0d wait_max=%0d",
          saw_load, search_cycles, wait_min, wait_max))
      end
      wait_for_scoreboard_idle(40_000, "B056 pop search latency drain");
    end else if (case_id == "B057") begin
      configure_and_start();
      single_seq = single_push_pop_seq::type_id::create("b057_single");
      single_seq.search_key = 12;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd2, 40_000, "B057 LOAD entry");
      load_mask = '0;
      load_pulses = 0;
      while (load_pulses < 16) begin
        if (m_env.m_dbg_mon.pop_partition_load != 0) begin
          load_mask |= m_env.m_dbg_mon.pop_partition_load;
          load_pulses += $countones(m_env.m_dbg_mon.pop_partition_load);
        end else if (m_env.m_dbg_mon.pop_engine_state_code != 3'd2 && load_mask != 0) begin
          break;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      expected_mask = (1 << m_cfg.n_partitions) - 1;
      if (load_mask !== expected_mask[3:0]) begin
        `uvm_error("B057", $sformatf(
          "Partition load walk mismatch: load_mask=0x%0h expected=0x%0h pulses=%0d",
          load_mask, expected_mask, load_pulses))
      end
      wait_for_scoreboard_idle(40_000, "B057 LOAD walk drain");
    end else if (case_id == "B058") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("b058_partition_count");
      burst_seq.num_hits = 17;
      burst_seq.search_key = 16;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_search_key(burst_seq.search_key[7:0], 40_000, "B058 target subheader", matched_subheader);
      if (matched_subheader == null || matched_subheader.hit_count != 17) begin
        `uvm_error("B058", $sformatf(
          "Partition count mismatch: expected 17 observed=%s",
          (matched_subheader == null) ? "none" : $sformatf("%0d", matched_subheader.hit_count)))
      end
      wait_for_scoreboard_idle(60_000, "B058 17-hit partition drain");
    end else if (case_id == "B059") begin
      configure_and_start();
      cmd_before = m_env.m_dbg_mon.pop_cmd_rdack_count;
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;
      observed_cmds = 0;
      while (observed_cmds < 1) begin
        if (m_env.m_dbg_mon.pop_cmd_rdack_count > cmd_before) begin
          observed_cmds = m_env.m_dbg_mon.pop_cmd_rdack_count - cmd_before;
        end
        if (observed_cmds >= 1)
          break;
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      wait_clocks(4);
      if (m_env.m_out_mon.total_hits_seen != hit_before) begin
        `uvm_error("B059", $sformatf(
          "Zero-hit fast path emitted hit data: hits_before=%0d hits_after=%0d",
          hit_before, m_env.m_out_mon.total_hits_seen))
      end
      if (m_env.m_out_mon.total_subheaders_seen <= subhdr_before ||
          m_env.m_out_mon.recent_subheaders.size() == 0 ||
          m_env.m_out_mon.recent_subheaders[$].hit_count != 0) begin
        `uvm_error("B059", "Zero-hit fast path did not emit a zero-hit subheader before rdack")
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
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq = same_key_burst_seq::type_id::create("b061_burst");
      burst_seq.num_hits = 1;
      burst_seq.search_key = 8;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_search_key(burst_seq.search_key[7:0], 40_000, "B061 target subheader", matched_subheader);
      wait_for_hit_output_count(hit_before + 1, 40_000, "B061 first hit");
      if (matched_subheader == null || m_env.m_out_mon.recent_hits.size() == 0) begin
        `uvm_error("B061", "Did not capture both subheader and hit beats")
      end else begin
        if (!(matched_subheader.sop && !matched_subheader.eop)) begin
          `uvm_error("B061", "1-hit subheader framing was not SOP-only")
        end
        if (!m_env.m_out_mon.recent_hits[$].eop) begin
          `uvm_error("B061", "1-hit data beat did not assert EOP")
        end
      end
      wait_for_scoreboard_idle(40_000, "B061 single-hit drain");
    end else if (case_id == "B062") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("b062_burst");
      burst_seq.num_hits = 1;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(11);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_search_key(burst_seq.search_key[7:0], 40_000, "B062 target subheader", matched_subheader);
      if (matched_subheader == null ||
          matched_subheader.search_key != burst_seq.search_key[7:0]) begin
        `uvm_error("B062", $sformatf(
          "Subheader search_key did not mirror ts[11:4]: expected=0x%02x observed=%s",
          burst_seq.search_key[7:0],
          (matched_subheader == null) ? "none" : $sformatf("0x%02x", matched_subheader.search_key)))
      end
      wait_for_scoreboard_idle(40_000, "B062 search-key framing");
    end else if (case_id == "B063") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("b063_burst");
      burst_seq.num_hits = 5;
      burst_seq.search_key = 12;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_search_key(burst_seq.search_key[7:0], 40_000, "B063 target subheader", matched_subheader);
      if (matched_subheader == null ||
          matched_subheader.hit_count != 5) begin
        `uvm_error("B063", $sformatf(
          "Subheader hit_count field did not report 5: expected=5 observed=%s",
          (matched_subheader == null) ? "none" : $sformatf("%0d", matched_subheader.hit_count)))
      end
      wait_for_scoreboard_idle(40_000, "B063 hit_count framing");
    end else if (case_id == "B064") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("b064_burst");
      burst_seq.num_hits = 2;
      burst_seq.search_key = 16;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_search_key(burst_seq.search_key[7:0], 40_000, "B064 target subheader", matched_subheader);
      if (matched_subheader == null ||
          matched_subheader.raw_data[7:0] != ring_buffer_cam_pkg::K237) begin
        `uvm_error("B064", "Subheader K237 identifier mismatch")
      end
      wait_for_scoreboard_idle(40_000, "B064 K237 framing");
    end else if (case_id == "B065") begin
      configure_and_start();
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq = same_key_burst_seq::type_id::create("b065_burst");
      burst_seq.num_hits = 2;
      burst_seq.search_key = 20;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_search_key(burst_seq.search_key[7:0], 40_000, "B065 target subheader", matched_subheader);
      wait_for_hit_output_count(hit_before + 1, 40_000, "B065 first hit");
      if (matched_subheader == null || m_env.m_out_mon.recent_hits.size() == 0) begin
        `uvm_error("B065", "Missing subheader/hit history for byte_is_k check")
      end else begin
        if (matched_subheader.raw_data[35:32] != 4'b0001) begin
          `uvm_error("B065", "Subheader byte_is_k field was not 0001")
        end
        if (m_env.m_out_mon.recent_hits[$].raw_data[35:32] != 4'b0000) begin
          `uvm_error("B065", "Hit byte_is_k field was not 0000")
        end
      end
      wait_for_scoreboard_idle(40_000, "B065 byte_is_k framing");
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
    end else if (case_id == "B069") begin
      configure_and_start();
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      wait_clocks(256);
      cmd_after = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      observed_cmds = cmd_after - cmd_before;
      expected_cmds = 256 / (16 * m_cfg.interleaving_factor);
      if (observed_cmds < (expected_cmds - 1) || observed_cmds > (expected_cmds + 1)) begin
        `uvm_error("B069", $sformatf(
          "Pop descriptor cadence mismatch: observed=%0d expected=%0d interleaving_factor=%0d",
          observed_cmds, expected_cmds, m_cfg.interleaving_factor))
      end
    end else if (case_id == "B070") begin
      configure_and_start(2000);
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      single_seq = single_push_pop_seq::type_id::create("b070_first_epoch");
      single_seq.search_key = m_cfg.lane_key_ord_to_search_key(0);
      single_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_count(subhdr_before + 1, 60_000, "B070 first pop descriptor");
      if (m_env.m_out_mon.recent_subheaders.size() == 0 ||
          m_env.m_out_mon.recent_subheaders[$].search_key != 8'h00) begin
        `uvm_error("B070", $sformatf(
          "First subheader search key mismatch: observed=%s expected=0x00",
          (m_env.m_out_mon.recent_subheaders.size() == 0) ? "none" :
            $sformatf("0x%02x", m_env.m_out_mon.recent_subheaders[$].search_key)))
      end
      wait_for_scoreboard_idle(80_000, "B070 first-epoch drain");
      expect_service_model_accounting("B070 first-epoch drain", 1, 0);
    end else if (case_id == "B073") begin
      run_same_key_epoch_count_case("B073 occupancy-1", 1, 2, 1, 2000, 80_000, 1, 1);
    end else if (case_id == "B074") begin
      run_same_key_epoch_count_case("B074 occupancy-2", 2, 2, 2);
    end else if (case_id == "B076") begin
      run_same_key_epoch_count_case("B076 occupancy-16", 16, 2, 16);
    end else if (case_id == "B077") begin
      run_same_key_epoch_count_case("B077 occupancy-17", 17, 2, 17);
    end else if (case_id == "B081") begin
      run_same_key_epoch_count_case("B081 occupancy-511", 511, 2, 8'hFF, 2000, 300_000);
    end else if (case_id == "B082") begin
      run_single_key_overwrite_window("B082 same-key wraparound", 1024);
    end else if (case_id == "B083") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      single_seq = single_push_pop_seq::type_id::create("b083_single");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "B083 immediate-latency push");
      cycle_a = m_env.m_dbg_mon.first_push_cycle;
      search_cycles = 0;
      while (search_cycles < 64 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("B083", "EXPECTED_LATENCY=0 did not trigger an immediate pop descriptor")
      end
      wait_clocks(4);
      dbg48_a = m_env.m_dbg_mon.gts_8n - m_env.m_dbg_mon.read_time_ptr;
      if (dbg48_a > 48'd1) begin
        `uvm_error("B083", $sformatf(
          "EXPECTED_LATENCY=0 did not collapse read_time_ptr delta: observed=%0d",
          dbg48_a))
      end
      wait_for_subheader_search_key(focus_search_key[7:0], 80_000, "B083 immediate subheader", matched_subheader);
      wait_for_scoreboard_idle(80_000, "B083 immediate-latency drain");
      expect_service_model_accounting("B083 immediate-latency drain", 1, 0);
    end else if (case_id == "B084") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      single_seq = single_push_pop_seq::type_id::create("b084_pending_single");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "B084 pending hit");
      wait_clocks(64);
      if (m_env.m_out_mon.total_subheaders_seen != subhdr_before) begin
        `uvm_error("B084", "Pending hit drained before the latency rewrite took effect")
      end
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'h0000) begin
        `uvm_error("B084", $sformatf(
          "Latency rewrite to zero did not reach the DUT: observed=0x%04x",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end
      wait_for_subheader_search_key(focus_search_key[7:0], 80_000, "B084 latency-shifted drain", matched_subheader);
      wait_for_scoreboard_idle(80_000, "B084 latency rewrite drain");
      expect_service_model_accounting("B084 latency rewrite drain", 1, 0);
    end else if (case_id == "B085") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      for (int i = 0; i < 4; i++) begin
        err_seq = single_error_hit_seq::type_id::create($sformatf("b085_filtered_%0d", i));
        err_seq.search_key = focus_search_key;
        err_seq.start(m_env.m_hit_seqr);
      end
      wait_clocks(32);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd4, "B085 filtered half increments INERR");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "B085 filtered half does not push");
      csr_write(CSR_CTRL_ADDR, 32'h0000_0001);
      wait_clocks(2);
      for (int i = 0; i < 4; i++) begin
        err_seq = single_error_hit_seq::type_id::create($sformatf("b085_passthrough_%0d", i));
        err_seq.search_key = focus_search_key;
        err_seq.start(m_env.m_hit_seqr);
      end
      wait_for_subheader_search_key(focus_search_key[7:0], 80_000, "B085 accepted second-half drain", matched_subheader);
      if (matched_subheader == null || matched_subheader.hit_count != 4) begin
        `uvm_error("B085", $sformatf(
          "Filter-toggle drain count mismatch: observed=%s expected=4",
          (matched_subheader == null) ? "none" : $sformatf("%0d", matched_subheader.hit_count)))
      end
      wait_for_scoreboard_idle(80_000, "B085 filter-toggle drain");
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd4, "B085 INERR_COUNT holds filtered half only");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd4, "B085 PUSH_COUNT reflects second half only");
      expect_service_model_accounting("B085 filter-toggle drain", 1, 0);
    end else if (case_id == "B086") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b086_same_key");
      burst_seq.num_hits = 2;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd4, 40_000, "B086 DRAIN entry");
      search_cycles = 0;
      while (search_cycles < 128 &&
             m_env.m_dbg_mon.pop_engine_state_code != 3'd5) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_engine_state_code != 3'd5) begin
        `uvm_error("B086", "Drain did not reach RESET after the idle-partition round-robin handoff")
      end
      wait_for_scoreboard_idle(80_000, "B086 round-robin drain");
      expect_service_model_accounting("B086 round-robin drain", 1, 0);
    end else if (case_id == "B087") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b087_same_key");
      burst_seq.num_hits = 2;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd4, 40_000, "B087 DRAIN entry");
      saw_overlap = 1'b0;
      search_cycles = 0;
      while (search_cycles < 256 && !saw_overlap) begin
        if (m_env.m_dbg_mon.pop_partition_advance[m_env.m_dbg_mon.pop_issue_partition_idx] === 1'b1 &&
            m_env.m_dbg_mon.pop_partition_has_more[m_env.m_dbg_mon.pop_issue_partition_idx] === 1'b1) begin
          saw_overlap = 1'b1;
          @(posedge m_env.m_csr_drv.vif.clk);
          if (m_env.m_dbg_mon.pop_partition_advance[m_env.m_dbg_mon.pop_issue_partition_idx] !== 1'b0) begin
            `uvm_error("B087", "pop_partition_advance stayed high for more than one cycle")
          end
        end else begin
          @(posedge m_env.m_csr_drv.vif.clk);
          search_cycles++;
        end
      end
      if (!saw_overlap) begin
        `uvm_error("B087", "Did not observe a one-cycle pop_partition_advance pulse while has_more=1")
      end
      wait_for_scoreboard_idle(80_000, "B087 advance handshake drain");
      expect_service_model_accounting("B087 advance handshake drain", 1, 0);
    end else if (case_id == "B088") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq = same_key_burst_seq::type_id::create("b088_same_key");
      burst_seq.num_hits = 2;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd4, 40_000, "B088 DRAIN entry");
      saw_overlap = 1'b0;
      search_cycles = 0;
      while (search_cycles < 256 && !saw_overlap) begin
        if (m_env.m_dbg_mon.pop_last_hit_pending === 1'b1) begin
          saw_overlap = 1'b1;
          break;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_overlap) begin
        `uvm_error("B088", "Did not observe pop_last_hit_pending before the final hit")
      end
      wait_for_hit_output_count(hit_before + 2, 80_000, "B088 last hit output");
      if (m_env.m_out_mon.recent_hits.size() == 0 ||
          !m_env.m_out_mon.recent_hits[$].eop) begin
        `uvm_error("B088", "Final hit did not assert EOP after pop_last_hit_pending")
      end
      wait_for_scoreboard_idle(80_000, "B088 last-hit drain");
      expect_service_model_accounting("B088 last-hit drain", 1, 0);
    end else if (case_id == "B089") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b089_same_key");
      burst_seq.num_hits = 2;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd5, 40_000, "B089 RESET entry");
      if (m_env.m_dbg_mon.pop_partition_pending != 0 ||
          m_env.m_dbg_mon.pop_hits_count != 0 ||
          m_env.m_dbg_mon.pop_partition_flag != 0 ||
          m_env.m_dbg_mon.pop_partition_has_more != 0) begin
        `uvm_error("B089", $sformatf(
          "RESET state left stale partition context: pending=0x%0h hits=%0d flag=0x%0h has_more=0x%0h result_valid=0x%0h",
          m_env.m_dbg_mon.pop_partition_pending,
          m_env.m_dbg_mon.pop_hits_count,
          m_env.m_dbg_mon.pop_partition_flag,
          m_env.m_dbg_mon.pop_partition_has_more,
          m_env.m_dbg_mon.pop_partition_result_valid))
      end
      wait_for_scoreboard_idle(80_000, "B089 reset cleanup drain");
      expect_service_model_accounting("B089 reset cleanup drain", 1, 0);
    end else if (case_id == "B090") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b090_overlap");
      burst_seq.num_hits = 64;
      burst_seq.search_key = focus_search_key;
      saw_overlap = 1'b0;
      fork
        begin
          burst_seq.start(m_env.m_hit_seqr);
        end
        begin
          search_cycles = 0;
          while (search_cycles < 1_024 && !saw_overlap) begin
            if (m_env.m_dbg_mon.vif.push_write_grant === 1'b1 &&
                (m_env.m_dbg_mon.pop_engine_state_code inside {3'd1, 3'd2, 3'd3})) begin
              saw_overlap = 1'b1;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
        end
      join
      if (!saw_overlap) begin
        `uvm_error("B090", "Did not observe push_write_grant during SEARCH/LOAD/COUNT")
      end
      wait_for_scoreboard_idle(120_000, "B090 push/pop overlap drain");
      expect_service_model_accounting("B090 push/pop overlap drain", 1, 0);
    end else if (case_id == "B091") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b091_drain_priority");
      burst_seq.num_hits = 256;
      burst_seq.search_key = focus_search_key;
      saw_overlap = 1'b0;
      traffic_done = 1'b0;
      fork
        begin
          burst_seq.start(m_env.m_hit_seqr);
          traffic_done = 1'b1;
        end
        begin
          search_cycles = 0;
          while (search_cycles < 1_024) begin
            if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1 &&
                !traffic_done) begin
              saw_overlap = 1'b1;
              if (m_env.m_dbg_mon.vif.push_write_grant === 1'b1) begin
                `uvm_error("B091", "push_write_grant remained high while pop_erase_grant owned DRAIN")
              end
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
        end
      join
      if (!saw_overlap) begin
        `uvm_error("B091", "Did not observe pop_erase_grant under ingress backpressure")
      end
      wait_for_scoreboard_idle(120_000, "B091 drain-priority overlap");
      expect_service_model_accounting("B091 drain-priority overlap", 1, 0);
    end else if (case_id == "B093") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b093_preempt");
      burst_seq.num_hits = 32;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      while (m_env.m_dbg_mon.vif.pop_erase_grant !== 1'b1) begin
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      saw_preempt = 1'b0;
      search_cycles = 0;
      while (search_cycles < 512 && !saw_preempt) begin
        if (m_env.m_dbg_mon.vif.decision_reg == 3'b011 &&
            m_env.m_dbg_mon.run_mgmt_flush_memory_start === 1'b1) begin
          saw_preempt = 1'b1;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_preempt) begin
        `uvm_error("B093", "Flush did not preempt DRAIN with decision_reg=3")
      end
      repeat (8) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
          `uvm_error("B093", "pop_erase_grant remained active after flush preemption")
        end
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      m_env.m_scb.note_flush_reset();
      wait_for_scoreboard_idle(120_000, "B093 flush preemption cleanup");
    end else if (case_id == "E001") begin
      configure_and_start();
      wait_clocks(512);
      if (m_env.m_scb.total_zero_hit_subheaders == 0) begin
        `uvm_error("E001", "No zero-hit subheader was observed")
      end
    end else if (case_id == "E002") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e002_fill_512");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry, 25_000, "E002 boundary fill");
      terminate_and_drain(120_000, "E002 boundary drain");
      expect_service_model_accounting("E002 boundary drain", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (push_count != m_cfg.ring_buffer_n_entry ||
          pop_count != m_cfg.ring_buffer_n_entry ||
          overwrite_count != 0) begin
        `uvm_error("E002", $sformatf(
          "Boundary drain mismatch: push=%0d pop=%0d overwrite=%0d depth=%0d",
          push_count, pop_count, overwrite_count, m_cfg.ring_buffer_n_entry))
      end
    end else if (case_id == "E003") begin
      run_single_key_overwrite_window("E003 overwrite_stress same-ts hotspot", 520);
    end else if (case_id == "E004") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("e004_push_a");
      burst_seq.num_hits = 64;
      burst_seq.search_key = 8;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_search_key(8, 40_000, "E004 first drain epoch", matched_subheader);
      burst_seq = same_key_burst_seq::type_id::create("e004_push_b");
      burst_seq.num_hits = 32;
      burst_seq.search_key = 12;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(120_000, "E004 concurrent push/pop arbitration");
      expect_service_model_accounting("E004 concurrent push/pop arbitration", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      if (push_count != 96 || pop_count != 96) begin
        `uvm_error("E004", $sformatf(
          "Concurrent push/pop accounting mismatch: push=%0d pop=%0d",
          push_count, pop_count))
      end
    end else if (case_id == "E005") begin
      configure_and_start(0);
      single_seq = single_push_pop_seq::type_id::create("e005_first");
      single_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      single_seq.start(m_env.m_hit_seqr);
      single_seq = single_push_pop_seq::type_id::create("e005_second");
      single_seq.search_key = m_cfg.lane_key_ord_to_search_key(3);
      single_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_match(
        m_cfg.lane_key_ord_to_search_key(2)[7:0], 8'd1, 1'b1, 1'b1, 200_000,
        "E005 first data-bearing subheader", matched_subheader);
      wait_for_subheader_match(
        m_cfg.lane_key_ord_to_search_key(3)[7:0], 8'd1, 1'b1, 1'b1, 200_000,
        "E005 second data-bearing subheader", matched_subheader);
      wait_for_scoreboard_idle(120_000, "E005 back-to-back drain");
      expect_service_model_accounting("E005 back-to-back drain", 1, 0);
    end else if (case_id == "E006") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e006_partition_balance");
      burst_seq.num_hits = (m_cfg.ring_buffer_n_entry / m_cfg.n_partitions) + 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(burst_seq.num_hits, 40_000, "E006 partition-span fill");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != burst_seq.num_hits || overwrite_count != 0 || fill_level != burst_seq.num_hits) begin
        `uvm_error("E006", $sformatf(
          "Partition-span fill mismatch: push=%0d overwrite=%0d fill=%0d expected=%0d",
          push_count, overwrite_count, fill_level, burst_seq.num_hits))
      end
      if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < burst_seq.num_hits) begin
        `uvm_error("E006", $sformatf(
          "Partition-span occupancy never reached target: key=%0d max_for_key=%0d expected=%0d",
          focus_search_key,
          m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
          burst_seq.num_hits))
      end
      terminate_and_drain(250_000, "E006 partition-span terminate");
      expect_service_model_accounting("E006 partition-span terminate", 1, 0);
    end else if (case_id == "E007") begin
      configure_and_start();
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e007_partition_handoff");
      burst_seq.num_hits = (m_cfg.ring_buffer_n_entry / m_cfg.n_partitions) + 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_hit_output_count(burst_seq.num_hits - 1, 2_500_000, "E007 first partition drain");
      hit_before = m_env.m_out_mon.total_hits_seen;
      search_cycles = 0;
      while (search_cycles < 200_000 &&
             m_env.m_out_mon.total_hits_seen == hit_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_out_mon.total_hits_seen == hit_before) begin
        `uvm_error("E007", "Drain stalled after the first partition emptied while one hit still remained")
      end
      wait_for_hit_output_count(burst_seq.num_hits, 200_000, "E007 final spill hit");
      wait_for_scoreboard_idle(200_000, "E007 partition handoff drain");
    end else if (case_id == "E008") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      single_seq = single_push_pop_seq::type_id::create("e008_latency_min");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "E008 min-latency push");
      search_cycles = 0;
      while (search_cycles < 64 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E008", "Minimum expected latency did not emit an immediate pop descriptor")
      end
      wait_for_scoreboard_idle(80_000, "E008 minimum-latency drain");
      restart_after_flush(16'hffff);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(3);
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      single_seq = single_push_pop_seq::type_id::create("e008_latency_max");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "E008 maximum-latency push");
      wait_clocks(4096);
      if (m_env.m_out_mon.total_subheaders_seen != subhdr_before) begin
        `uvm_error("E008", "Maximum expected latency drained too early")
      end
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_subheader_search_key(focus_search_key[7:0], 120_000, "E008 maximum-latency release", matched_subheader);
      wait_for_scoreboard_idle(120_000, "E008 latency-extreme drain");
    end else if (case_id == "E009") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e009_fill_511");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry - 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry - 1, 25_000, "E009 fill to depth-1");
      terminate_and_drain(120_000, "E009 boundary-1 drain");
      expect_service_model_accounting("E009 boundary-1 drain", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (push_count != (m_cfg.ring_buffer_n_entry - 1) ||
          pop_count != (m_cfg.ring_buffer_n_entry - 1) ||
          overwrite_count != 0) begin
        `uvm_error("E009", $sformatf(
          "Boundary-1 drain mismatch: push=%0d pop=%0d overwrite=%0d depth=%0d",
          push_count, pop_count, overwrite_count, m_cfg.ring_buffer_n_entry))
      end
    end else if (case_id == "E010") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e010_fill_512");
      burst_seq.num_hits = 512;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(512, 20_000, "E010 fill to exact depth");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 512) begin
        `uvm_error("E010", $sformatf("Exact-depth push count mismatch: observed=%0d expected=512", push_count))
      end
      if (overwrite_count != 0) begin
        `uvm_error("E010", $sformatf("Exact-depth fill should not overwrite: observed=%0d", overwrite_count))
      end
      if (fill_level != m_cfg.ring_buffer_n_entry) begin
        `uvm_error("E010", $sformatf(
          "Exact-depth fill level mismatch: observed=%0d expected=%0d",
          fill_level, m_cfg.ring_buffer_n_entry))
      end
      if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < m_cfg.ring_buffer_n_entry) begin
        `uvm_error("E010", $sformatf(
          "Exact-depth case never reached full hotspot occupancy: key=%0d max_for_key=%0d ring_depth=%0d",
          focus_search_key,
          m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
          m_cfg.ring_buffer_n_entry))
      end
      m_env.m_scb.note_intentional_nonempty_end(m_cfg.ring_buffer_n_entry);
    end else if (case_id == "E011") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e011_fill_513");
      burst_seq.num_hits = 513;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(513, 25_000, "E011 first wrap overwrite");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 513) begin
        `uvm_error("E011", $sformatf("Wrap-overwrite push count mismatch: observed=%0d expected=513", push_count))
      end
      if (overwrite_count != 1) begin
        `uvm_error("E011", $sformatf("Wrap-overwrite count mismatch: observed=%0d expected=1", overwrite_count))
      end
      if (fill_level != m_cfg.ring_buffer_n_entry) begin
        `uvm_error("E011", $sformatf(
          "Wrap-overwrite fill level mismatch: observed=%0d expected=%0d",
          fill_level, m_cfg.ring_buffer_n_entry))
      end
      m_env.m_scb.note_intentional_nonempty_end(m_cfg.ring_buffer_n_entry);
    end else if (case_id == "E012") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e012_fill_1024");
      burst_seq.num_hits = 1024;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1024, 40_000, "E012 double wrap overwrite");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 1024) begin
        `uvm_error("E012", $sformatf("Double-wrap push count mismatch: observed=%0d expected=1024", push_count))
      end
      if (overwrite_count != 512) begin
        `uvm_error("E012", $sformatf("Double-wrap overwrite mismatch: observed=%0d expected=512", overwrite_count))
      end
      if (fill_level != m_cfg.ring_buffer_n_entry) begin
        `uvm_error("E012", $sformatf(
          "Double-wrap fill level mismatch: observed=%0d expected=%0d",
          fill_level, m_cfg.ring_buffer_n_entry))
      end
      m_env.m_scb.note_intentional_nonempty_end(m_cfg.ring_buffer_n_entry);
    end else if (case_id == "E013") begin
      configure_and_start();
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e013_boundary_127");
      burst_seq.num_hits = 127;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'(burst_seq.num_hits), 1'b0, 1'b1, 500_000,
        "E013 127-hit subheader", matched_subheader);
      if (matched_subheader == null || matched_subheader.hit_count != 127) begin
        `uvm_error("E013", $sformatf(
          "127-hit partition boundary mismatch: observed=%s",
          (matched_subheader == null) ? "none" : $sformatf("%0d", matched_subheader.hit_count)))
      end
      wait_for_scoreboard_idle(120_000, "E013 127-hit drain");
    end else if (case_id == "E014") begin
      configure_and_start();
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e014_boundary_128");
      burst_seq.num_hits = 128;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'(burst_seq.num_hits), 1'b0, 1'b1, 500_000,
        "E014 128-hit subheader", matched_subheader);
      if (matched_subheader == null || matched_subheader.hit_count != 128) begin
        `uvm_error("E014", $sformatf(
          "128-hit partition boundary mismatch: observed=%s",
          (matched_subheader == null) ? "none" : $sformatf("%0d", matched_subheader.hit_count)))
      end
      wait_for_scoreboard_idle(120_000, "E014 128-hit drain");
    end else if (case_id == "E015") begin
      configure_and_start();
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      burst_seq = same_key_burst_seq::type_id::create("e015_boundary_129");
      burst_seq.num_hits = (m_cfg.ring_buffer_n_entry / m_cfg.n_partitions) + 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_hit_output_count(burst_seq.num_hits, 2_500_000, "E015 full wrapped drain");
      wait_for_scoreboard_idle(180_000, "E015 partition-span drain");
      if (m_env.m_out_mon.total_data_subheaders_seen < subhdr_before + 2) begin
        `uvm_error("E015", $sformatf(
          "Partition-span packetization emitted too few data-bearing subheaders: observed_total=%0d baseline=%0d",
          m_env.m_out_mon.total_data_subheaders_seen, subhdr_before))
      end
      if (m_env.m_out_mon.recent_data_subheaders.size() < 2) begin
        `uvm_error("E015", "Partition-span packetization did not retain two data-bearing subheaders")
      end else begin
        if (m_env.m_out_mon.recent_data_subheaders[m_env.m_out_mon.recent_data_subheaders.size()-2].search_key !=
            focus_search_key[7:0] ||
            m_env.m_out_mon.recent_data_subheaders[$].search_key != focus_search_key[7:0]) begin
          `uvm_error("E015", $sformatf(
            "Partition-span packetization search-key mismatch: first=0x%02x last=0x%02x expected=0x%02x",
            m_env.m_out_mon.recent_data_subheaders[m_env.m_out_mon.recent_data_subheaders.size()-2].search_key,
            m_env.m_out_mon.recent_data_subheaders[$].search_key,
            focus_search_key[7:0]))
        end
      end
    end else if (case_id == "E016") begin
      configure_and_start();
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      burst_seq = same_key_burst_seq::type_id::create("e016_full_partition_span");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry, 40_000, "E016 exact-depth full-partition fill");
      expected_mask = (1 << m_cfg.n_partitions) - 1;
      wait_for_partition_issue_mask(expected_mask[3:0], load_mask, 180_000, "E016 full-partition issue mask");
      wait_for_hit_output_count(m_cfg.ring_buffer_n_entry, 2_500_000, "E016 full-depth drain");
      wait_for_scoreboard_idle(300_000, "E016 full-partition drain");
      if (m_env.m_out_mon.total_data_subheaders_seen < subhdr_before + 2) begin
        `uvm_error("E016", $sformatf(
          "Full-depth packetization emitted too few data-bearing subheaders: observed_total=%0d baseline=%0d",
          m_env.m_out_mon.total_data_subheaders_seen, subhdr_before))
      end
      if (m_env.m_out_mon.recent_data_subheaders.size() < 2) begin
        `uvm_error("E016", "Full-depth packetization did not retain two data-bearing subheaders")
      end else begin
        if (m_env.m_out_mon.recent_data_subheaders[m_env.m_out_mon.recent_data_subheaders.size()-2].search_key !=
            focus_search_key[7:0] ||
            m_env.m_out_mon.recent_data_subheaders[$].search_key != focus_search_key[7:0]) begin
          `uvm_error("E016", $sformatf(
            "Full-depth packetization search-key mismatch: first=0x%02x last=0x%02x expected=0x%02x",
            m_env.m_out_mon.recent_data_subheaders[m_env.m_out_mon.recent_data_subheaders.size()-2].search_key,
            m_env.m_out_mon.recent_data_subheaders[$].search_key,
            focus_search_key[7:0]))
        end
      end
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
    end else if (case_id == "X002") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      single_seq = single_push_pop_seq::type_id::create("x002_tsglitch");
      single_seq.use_raw_tcc8n = 1'b1;
      single_seq.raw_tcc8n = m_cfg.make_tcc8n_for_lane_key(2, 4'h0, 1'b1);
      single_seq.hit_has_error = 1'b0;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_search_key(focus_search_key[7:0], 80_000, "X002 target subheader", matched_subheader);
      wait_for_hit_output_count(hit_before + 1, 80_000, "X002 glitch-hit drain");
      if (m_env.m_out_mon.recent_hits.size() == 0 ||
          m_env.m_out_mon.recent_hits[$].error !== 1'b1) begin
        `uvm_error("X002", "Timestamp-glitch hit did not assert the egress error bit")
      end
      wait_for_scoreboard_idle(80_000, "X002 glitch-hit drain");
      expect_service_model_accounting("X002 glitch-hit drain", 1, 0);
    end else if (case_id == "X017") begin
      configure_and_start(2000);
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_clocks(2);
      while (m_env.m_dbg_mon.vif.decision_reg != 3'b011) begin
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      err_seq = single_error_hit_seq::type_id::create("x017_flush_bad");
      err_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(8);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "X017 flush clears concurrent bad-hit count");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "X017 flush swallows the bad hit");
      if (m_env.m_scb.total_ingress_accepted != 0) begin
        `uvm_error("X017", $sformatf(
          "Flushing bad hit should not be accepted, observed accepted ingress=%0d",
          m_env.m_scb.total_ingress_accepted))
      end
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
    end else if (case_id == "X004") begin
      configure_and_start();
      burst_seq = same_key_burst_seq::type_id::create("x004_preflush");
      burst_seq.num_hits = 16;
      burst_seq.search_key = 8;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(16, 10_000, "X004 preflush activity");
      enter_run_prepare();
      configure_and_start();
      single_seq = single_push_pop_seq::type_id::create("x004_postrestart");
      single_seq.search_key = 12;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(60_000, "X004 flush and restart");
      expect_service_model_accounting("X004 flush and restart", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      if (push_count != 1) begin
        `uvm_error("X004", $sformatf(
          "Restart should only retain post-flush traffic, observed push_count=%0d",
          push_count))
      end
    end else if (case_id == "X006") begin
      configure_and_start(2000);
      data_a = 0;
      m_env.m_ctrl_drv.vif.data <= (ring_buffer_cam_pkg::CTRL_RUNNING | ring_buffer_cam_pkg::CTRL_SYNC);
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (2) begin
        @(posedge m_env.m_ctrl_drv.vif.clk);
        if (m_env.m_ctrl_drv.vif.ready === 1'b1)
          data_a = 1;
      end
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      if (data_a != 0 || m_env.m_dbg_mon.run_state_code != 4'd9) begin
        `uvm_error("X006", $sformatf(
          "Illegal active command was not contained: ack=%0d state=%0d",
          data_a, m_env.m_dbg_mon.run_state_code))
      end
    end else if (case_id == "X009") begin
      configure_and_start(2000);
      err_seq = single_error_hit_seq::type_id::create("x009_bad");
      err_seq.search_key = 8;
      err_seq.start(m_env.m_hit_seqr);
      single_seq = single_push_pop_seq::type_id::create("x009_good");
      single_seq.search_key = 8;
      single_seq.start(m_env.m_hit_seqr);
      wait_clocks(64);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "X009 INERR_COUNT");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "X009 PUSH_COUNT");
      wait_for_subheader_search_key(8, 40_000, "X009 one-good drain", matched_subheader);
      if (matched_subheader == null || matched_subheader.hit_count != 1) begin
        `uvm_error("X009", "Bad+good same-key case did not drain exactly one good hit")
      end
      wait_for_scoreboard_idle(60_000, "X009 good after bad drain");
    end else if (case_id == "X010") begin
      configure_and_start(2000);
      err_seq = single_error_hit_seq::type_id::create("x010_bad_a");
      err_seq.search_key = 8;
      err_seq.start(m_env.m_hit_seqr);
      err_seq = single_error_hit_seq::type_id::create("x010_bad_b");
      err_seq.search_key = 8;
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(32);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd2, "X010 INERR_COUNT");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "X010 PUSH_COUNT");
      if (m_env.m_scb.total_ingress_accepted != 0) begin
        `uvm_error("X010", $sformatf(
          "Back-to-back filtered bad hits should not be accepted, observed=%0d",
          m_env.m_scb.total_ingress_accepted))
      end
    end else if (case_id == "X011") begin
      configure_and_start(2000);
      csr_write(CSR_CTRL_ADDR, 32'h0000_0001);
      wait_clocks(2);
      err_seq = single_error_hit_seq::type_id::create("x011_bad_a");
      err_seq.search_key = 8;
      err_seq.start(m_env.m_hit_seqr);
      err_seq = single_error_hit_seq::type_id::create("x011_bad_b");
      err_seq.search_key = 8;
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(64);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "X011 INERR_COUNT");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd2, "X011 PUSH_COUNT");
      wait_for_subheader_search_key(8, 40_000, "X011 two bad hits forwarded", matched_subheader);
      if (matched_subheader == null || matched_subheader.hit_count != 2) begin
        `uvm_error("X011", "Filter-disabled bad hits did not both reach the CAM/drain")
      end
      wait_for_scoreboard_idle(60_000, "X011 bad-hit pass-through drain");
    end else if (case_id == "X012") begin
      configure_and_start(2000);
      for (int i = 0; i < 3; i++) begin
        single_seq = single_push_pop_seq::type_id::create($sformatf("x012_good_%0d", i));
        single_seq.search_key = 12;
        single_seq.start(m_env.m_hit_seqr);
        err_seq = single_error_hit_seq::type_id::create($sformatf("x012_bad_%0d", i));
        err_seq.search_key = 12;
        err_seq.start(m_env.m_hit_seqr);
      end
      wait_clocks(64);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd3, "X012 INERR_COUNT");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd3, "X012 PUSH_COUNT");
      wait_for_subheader_search_key(12, 40_000, "X012 alternating drain", matched_subheader);
      if (matched_subheader == null || matched_subheader.hit_count != 3) begin
        `uvm_error("X012", "Alternating good/bad case did not drain exactly the 3 good hits")
      end
      wait_for_scoreboard_idle(80_000, "X012 alternating good/bad drain");
    end else if (case_id == "X014") begin
      configure_and_start(2000);
      err_seq = single_error_hit_seq::type_id::create("x014_empty_bad");
      err_seq.hit_is_empty_marker = 1'b1;
      err_seq.hit_has_error = 1'b1;
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(8);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "X014 empty marker should not bump INERR_COUNT");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "X014 empty marker should not push");
      if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
        `uvm_error("X014", "Errored empty marker did not still latch endofrun_seen")
      end
    end else if (case_id == "X015") begin
      configure_and_start(2000);
      single_seq = single_push_pop_seq::type_id::create("x015_good");
      single_seq.search_key = 16;
      single_seq.start(m_env.m_hit_seqr);
      err_seq = single_error_hit_seq::type_id::create("x015_bad");
      err_seq.search_key = 16;
      err_seq.start(m_env.m_hit_seqr);
      wait_clocks(64);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "X015 INERR_COUNT");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd1, "X015 PUSH_COUNT");
      csr_expect_mask(CSR_OVERWRITE_ADDR, 32'hFFFF_FFFF, 32'd0, "X015 OVERWRITE_COUNT");
      wait_for_subheader_search_key(16, 40_000, "X015 good then filtered bad", matched_subheader);
      if (matched_subheader == null || matched_subheader.hit_count != 1) begin
        `uvm_error("X015", "Good hit was corrupted by adjacent filtered bad hit")
      end
      wait_for_scoreboard_idle(60_000, "X015 adjacent good/bad drain");
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
    end else if (case_id == "X116") begin
      run_x116_good_error_good_case("X116 GOOD-ERROR-GOOD");
    end else if (case_id == "X117") begin
      run_x117_good_error_flush_good_case("X117 GOOD-ERROR-FLUSH-GOOD");
    end else if (case_id == "X118") begin
      run_x118_good_terminate_restart_case("X118 GOOD-TERM-RESTART-GOOD");
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
      end else if (run_id == "CROSS-008") begin
        run_x117_good_error_flush_good_case("CROSS-008 GOOD-ERROR-FLUSH-GOOD", 2048, 64, 2048, 2000);
      end else if (run_id == "CROSS-009") begin
        run_x118_good_terminate_restart_case("CROSS-009 GOOD-TERM-RESTART-GOOD", 2048, 2048, 2000);
      end else if (run_id == "CROSS-010") begin
        run_cross_good_error_good_overwrite();
      end else if (run_id == "CROSS-015") begin
        run_cross_curated_all_bucket_mix();
      end else if (run_id == "CROSS-076") begin
        run_cross_overwrite_toggle_longrun();
      end else if (run_id == "CROSS-091") begin
        run_cross_inerr_toggle_longrun();
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

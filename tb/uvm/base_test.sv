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

  function automatic int unsigned flush_cam_linear_progress();
    return (int'(m_env.m_dbg_mon.flush_cam_wraddr) * 256) +
           int'(m_env.m_dbg_mon.flush_cam_wrdata);
  endfunction

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

  task automatic force_raw_hit(
    ring_buffer_cam_pkg::hit_seq_item hit_item,
    int unsigned hold_cycles = 1
  );
    m_env.m_hit_drv.manual_override = 1'b1;
    m_env.m_hit_drv.vif.data    <= hit_item.is_empty_marker ? '0 : hit_item.pack_hit();
    m_env.m_hit_drv.vif.valid   <= 1'b1;
    m_env.m_hit_drv.vif.channel <= hit_item.input_channel();
    m_env.m_hit_drv.vif.startofpacket <= hit_item.is_empty_marker;
    m_env.m_hit_drv.vif.endofpacket   <= hit_item.is_empty_marker;
    m_env.m_hit_drv.vif.empty   <= hit_item.is_empty_marker;
    m_env.m_hit_drv.vif.error   <= (hit_item.has_error ? 1'b1 : 1'b0);
    repeat (hold_cycles) @(posedge m_env.m_hit_drv.vif.clk);
    m_env.m_hit_drv.vif.data    <= '0;
    m_env.m_hit_drv.vif.valid   <= 1'b0;
    m_env.m_hit_drv.vif.channel <= '0;
    m_env.m_hit_drv.vif.startofpacket <= 1'b0;
    m_env.m_hit_drv.vif.endofpacket   <= 1'b0;
    m_env.m_hit_drv.vif.empty   <= 1'b0;
    m_env.m_hit_drv.vif.error   <= 1'b0;
    m_env.m_hit_drv.manual_override = 1'b0;
  endtask

  task automatic enter_run_prepare();
    // RUN_PREPARE is acked immediately from IDLE/RUNNING. Re-issue it so the
    // driver waits on the DUT's real flush-complete handshake instead of a
    // fixed simulation delay.
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    m_env.m_scb.note_flush_reset();
  endtask

  task automatic expect_soft_reset_abort(
    string       what,
    logic [31:0] ctrl_mask = 32'h0000_0002,
    logic [31:0] ctrl_expected = 32'h0000_0000,
    int unsigned settle_clocks = 8,
    bit          reset_scoreboard = 1'b1
  );
    int unsigned inerr_count;
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned cache_miss_count;
    int unsigned fill_level;

    wait_clocks(settle_clocks);
    csr_expect_mask(CSR_CTRL_ADDR, ctrl_mask, ctrl_expected, {what, " CTRL soft_reset readback"});
    if (m_env.m_dbg_mon.run_state_code != ring_buffer_cam_pkg::RUN_STATE_IDLE) begin
      `uvm_error("SOFT_RST", $sformatf(
        "%s did not return the DUT to IDLE: observed_run_state=%0d",
        what, m_env.m_dbg_mon.run_state_code))
    end
    if (m_env.m_dbg_mon.pop_engine_state_code != 3'd0) begin
      `uvm_error("SOFT_RST", $sformatf(
        "%s left the pop engine active: observed_pop_state=%0d",
        what, m_env.m_dbg_mon.pop_engine_state_code))
    end

    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (inerr_count != 0 || push_count != 0 || pop_count != 0 ||
        overwrite_count != 0 || cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("SOFT_RST", $sformatf(
        "%s did not clear the live accounting state: inerr=%0d push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d",
        what, inerr_count, push_count, pop_count, overwrite_count, cache_miss_count, fill_level))
    end
    if (m_env.m_dbg_mon.vif.pop_cmd_fifo_empty !== 1'b1 ||
        m_env.m_dbg_mon.vif.deassembly_fifo_empty !== 1'b1) begin
      `uvm_error("SOFT_RST", $sformatf(
        "%s did not leave the datapath FIFOs idle: pop_cmd_empty=%0d deassembly_empty=%0d",
        what, m_env.m_dbg_mon.vif.pop_cmd_fifo_empty, m_env.m_dbg_mon.vif.deassembly_fifo_empty))
    end

    if (reset_scoreboard) begin
      m_env.m_scb.note_flush_reset();
    end
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

  task automatic discard_pending_source_backlog(string what);
    int unsigned pending_total;
    int unsigned pending_payload;
    int unsigned pending_error;
    int unsigned pending_markers;

    pending_total = m_env.m_hit_drv.pending_source_items();
    if (pending_total == 0) begin
      return;
    end

    pending_payload = m_env.m_hit_drv.pending_source_payload_items();
    pending_error = m_env.m_hit_drv.pending_source_error_items();
    pending_markers = m_env.m_hit_drv.pending_source_marker_items();
    `uvm_info("HIT_DRV", $sformatf(
      "%s: discarding pending source backlog=%0d payload=%0d error=%0d marker=%0d offered=%0d accepted=%0d",
      what,
      pending_total,
      pending_payload,
      pending_error,
      pending_markers,
      m_env.m_hit_drv.offered_payload_total,
      m_env.m_hit_drv.accepted_payload_total), UVM_LOW)
    m_env.m_hit_drv.discard_pending_items();
    wait_clocks(2);
  endtask

  task automatic poison_side_ram_occ_zero_on_push_write(
    int unsigned target_push_ordinal = 1,
    int unsigned max_cycles = 40_000,
    string       what = "stale-slot poison"
  );
    int unsigned push_count;
    int unsigned cycles;
    int unsigned slot_addr;
    logic [39:0] poisoned_word;

    push_count = 0;
    cycles = 0;
    while (cycles < max_cycles) begin
      @(posedge m_env.m_csr_drv.vif.clk);
      if (m_env.m_dbg_mon.vif.push_write_grant === 1'b1) begin
        push_count++;
        if (push_count == target_push_ordinal) begin
          slot_addr = m_env.m_dbg_mon.vif.side_ram_waddr;
          poisoned_word = m_env.m_dbg_mon.vif.side_ram_din;
          poisoned_word[39] = 1'b0;
          $root.tb_top.tb_poison_side_ram_word(slot_addr, poisoned_word);
          return;
        end
      end
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s push write #%0d after %0d cycles",
      what, target_push_ordinal, max_cycles))
  endtask

  task automatic wait_for_cache_miss_count(
    int unsigned expected_count,
    int unsigned max_cycles = 40_000,
    string       what = "cache_miss_count"
  );
    int unsigned cycles;

    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_dbg_mon.dbg_cache_miss_cnt[31:0] >= expected_count) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s=%0d after %0d cycles: observed=%0d",
      what, expected_count, max_cycles, m_env.m_dbg_mon.dbg_cache_miss_cnt[31:0]))
  endtask

  task automatic expect_main_activity_counters_zero(string what);
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned cache_miss_count;

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    if (push_count != 0 || pop_count != 0 ||
        overwrite_count != 0 || cache_miss_count != 0) begin
      `uvm_error("COUNTERS", $sformatf(
        "%s did not clear PUSH/POP/OVERWRITE/CACHE_MISS: push=%0d pop=%0d overwrite=%0d cache_miss=%0d",
        what, push_count, pop_count, overwrite_count, cache_miss_count))
    end
  endtask

  function automatic int unsigned get_dv_seed();
    int unsigned dv_seed;

    if (!$value$plusargs("DV_SEED=%d", dv_seed)) begin
      dv_seed = 1;
    end
    return dv_seed;
  endfunction

  function automatic string get_dv_tb_dir();
    string tb_dir;

    if (!$value$plusargs("DV_TB_DIR=%s", tb_dir)) begin
      tb_dir = "";
    end
    return tb_dir;
  endfunction

  function automatic int unsigned get_long_txn_override();
    int unsigned override_hits;

    if (!$value$plusargs("DV_LONG_TXN_OVERRIDE=%d", override_hits)) begin
      override_hits = 0;
    end
    return override_hits;
  endfunction

  task automatic build_log_spaced_checkpoint_targets(
    int unsigned final_txn,
    ref int unsigned checkpoint_targets[$]
  );
    int unsigned next_txn;

    checkpoint_targets.delete();
    if (final_txn == 0) begin
      return;
    end

    next_txn = 1;
    while (next_txn < final_txn) begin
      checkpoint_targets.push_back(next_txn);
      if (next_txn > (32'h7fff_ffff >> 1)) begin
        break;
      end
      next_txn <<= 1;
    end
    if (checkpoint_targets.size() == 0 ||
        checkpoint_targets[$] != final_txn) begin
      checkpoint_targets.push_back(final_txn);
    end
  endtask

  task automatic wait_for_accepted_payload_count(
    int unsigned expected_count,
    int unsigned max_cycles,
    string       what
  );
    int unsigned cycles;

    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_hit_drv.accepted_payload_total >= expected_count) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s accepted_payload_total=%0d after %0d cycles: observed=%0d",
      what, expected_count, max_cycles, m_env.m_hit_drv.accepted_payload_total))
  endtask

  task automatic save_txn_growth_checkpoints(
    string       case_id,
    int unsigned checkpoint_targets[$],
    int unsigned max_cycles_per_checkpoint,
    string       what
  );
    string checkpoint_dir;
    string checkpoint_path;
    string tb_dir;
    int unsigned dv_seed;
    int unsigned idx;

    tb_dir = get_dv_tb_dir();
    if (tb_dir == "") begin
      `uvm_fatal("COV", $sformatf(
        "%s is missing +DV_TB_DIR; cannot save checkpoint UCDBs",
        what))
    end

    dv_seed = get_dv_seed();
    checkpoint_dir = {tb_dir, "/uvm/cov_after/txn_growth"};
    void'($system({"mkdir -p ", checkpoint_dir}));

    for (idx = 0; idx < checkpoint_targets.size(); idx++) begin
      wait_for_accepted_payload_count(
        checkpoint_targets[idx],
        max_cycles_per_checkpoint,
        {what, " checkpoint"});
      checkpoint_path = $sformatf(
        "%s/%s_txn%0d_s%0d.ucdb",
        checkpoint_dir, case_id, checkpoint_targets[idx], dv_seed);
      void'($coverage_save(checkpoint_path));
      `uvm_info("COV", $sformatf(
        "%s saved checkpoint UCDB txn=%0d path=%s",
        what, checkpoint_targets[idx], checkpoint_path), UVM_LOW)
    end
  endtask

  task automatic run_weighted_overwrite_checkpoint_case(
    string       what,
    string       case_id,
    int unsigned latency,
    int unsigned default_num_epochs,
    int unsigned key_hits_per_epoch[$],
    int unsigned lane_key_ord_list[$],
    int unsigned inter_hit_gap_cycles,
    int unsigned inter_epoch_gap_cycles,
    int unsigned sink_ready_mode,
    int unsigned timeout_cycles,
    int unsigned min_overwrites,
    output int unsigned total_hits,
    output int unsigned push_count,
    output int unsigned pop_count,
    output int unsigned overwrite_count,
    output int unsigned cache_miss_count,
    output int unsigned fill_level,
    output int unsigned sink_low_cycles
  );
    weighted_profile_seq prof_seq;
    int unsigned         checkpoint_targets[$];
    int unsigned         hits_per_epoch_total;
    int unsigned         num_epochs;
    int unsigned         override_hits;
    int unsigned         sink_cycles;
    logic [31:0]         ready_lfsr;
    bit                  ready_now;
    bit                  traffic_done;

    hits_per_epoch_total = 0;
    foreach (key_hits_per_epoch[idx]) begin
      hits_per_epoch_total += key_hits_per_epoch[idx];
    end
    if (hits_per_epoch_total == 0) begin
      `uvm_fatal("PROF", $sformatf(
        "%s requires at least one weighted traffic slot",
        what))
    end

    num_epochs = default_num_epochs;
    override_hits = get_long_txn_override();
    if (override_hits > 0) begin
      num_epochs = (override_hits + hits_per_epoch_total - 1) / hits_per_epoch_total;
      if (num_epochs == 0) begin
        num_epochs = 1;
      end
      `uvm_info("PROF", $sformatf(
        "%s applying DV_LONG_TXN_OVERRIDE=%0d -> epochs=%0d",
        what, override_hits, num_epochs), UVM_LOW)
    end

    total_hits = hits_per_epoch_total * num_epochs;
    build_log_spaced_checkpoint_targets(total_hits, checkpoint_targets);

    sink_low_cycles = 0;
    traffic_done = 1'b0;
    ready_lfsr = 32'h1bad_cafe;

    configure_and_start(latency);

    fork
      begin : sink_ready_thread
        if (sink_ready_mode != 0) begin
          sink_cycles = 0;
          while (sink_cycles < timeout_cycles &&
                 (!traffic_done ||
                  m_env.m_hit_drv.pending_source_items() != 0 ||
                  m_env.m_scb.remaining_entries() != 0)) begin
            case (sink_ready_mode)
              1: ready_now = ((sink_cycles & 2'b11) != 2'b11); // 75% ready
              2: ready_now = ((sink_cycles & 1'b1) == 1'b0);   // 50% ready
              3: begin
                ready_lfsr = {ready_lfsr[30:0],
                              ready_lfsr[31] ^ ready_lfsr[21] ^ ready_lfsr[1] ^ ready_lfsr[0]};
                ready_now = (ready_lfsr[7:0] < 8'd179);        // ~70% ready
              end
              4: begin
                ready_lfsr = {ready_lfsr[30:0],
                              ready_lfsr[31] ^ ready_lfsr[21] ^ ready_lfsr[1] ^ ready_lfsr[0]};
                ready_now = (ready_lfsr[7:0] < 8'd77);         // ~30% ready
              end
              default: ready_now = 1'b1;
            endcase
            set_sink_ready(ready_now);
            if (!ready_now) begin
              sink_low_cycles++;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            sink_cycles++;
          end
          set_sink_ready(1'b1);
        end
      end
      begin : checkpoint_thread
        save_txn_growth_checkpoints(
          case_id, checkpoint_targets, timeout_cycles, what);
      end
      begin : traffic_thread
        prof_seq = weighted_profile_seq::type_id::create({what, "_weighted"});
        prof_seq.num_epochs = num_epochs;
        prof_seq.inter_hit_gap_cycles = inter_hit_gap_cycles;
        prof_seq.inter_epoch_gap_cycles = inter_epoch_gap_cycles;
        prof_seq.progress_stride = (total_hits >= 4096) ? (total_hits / 4) :
                                   ((total_hits >= 4) ? (total_hits / 2) : 1);
        prof_seq.progress_tag = what;
        foreach (key_hits_per_epoch[idx]) begin
          prof_seq.key_hits_per_epoch.push_back(key_hits_per_epoch[idx]);
        end
        foreach (lane_key_ord_list[idx]) begin
          prof_seq.lane_key_ord_list.push_back(lane_key_ord_list[idx]);
        end
        prof_seq.start(m_env.m_hit_seqr);
        traffic_done = 1'b1;
      end
    join

    set_sink_ready(1'b1);
    terminate_and_drain(timeout_cycles, {what, " terminate drain"});
    expect_service_model_accounting({what, " post-terminate"}, 1, min_overwrites);

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (push_count != total_hits || cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected_push=%0d",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level, total_hits))
    end
    if (sink_ready_mode != 0 && sink_low_cycles == 0) begin
      `uvm_error("PROF", $sformatf(
        "%s never forced sink-ready low in the backpressure-enabled soak",
        what))
    end

    `uvm_info("PROF", $sformatf(
      "%s overwrite-soak summary: total_hits=%0d push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d sink_low_cycles=%0d max_live_fill=%0d",
      what, total_hits, push_count, pop_count, overwrite_count, cache_miss_count,
      fill_level, sink_low_cycles, m_env.m_dbg_mon.max_live_fill), UVM_LOW)
  endtask

  task automatic wait_for_scoreboard_idle(
    int unsigned max_cycles = 50_000,
    string what = "scoreboard drain"
  );
    int unsigned cycles;
    bit          lone_pending_eor_marker;
    cycles = 0;
    while (cycles < max_cycles) begin
      lone_pending_eor_marker =
          (m_env.m_hit_drv.pending_source_items() == 1) &&
          m_env.m_hit_drv.head_pending_item_is_empty_marker() &&
          (m_env.m_dbg_mon.vif.endofrun_seen === 1'b1) &&
          (m_env.m_dbg_mon.terminating_drain_done === 1'b1);
      if (m_env.m_scb.remaining_entries() == 0 &&
          m_env.m_scb.pending_drain_entries() == 0 &&
          m_env.m_scb.epoch_idle() &&
          m_env.m_scb.total_written == m_env.m_scb.total_ingress_accepted &&
          (m_env.m_hit_drv.pending_source_items() == 0 || lone_pending_eor_marker) &&
          m_env.m_dbg_mon.vif.deassembly_fifo_empty === 1'b1 &&
          m_env.m_dbg_mon.vif.pop_cmd_fifo_empty === 1'b1) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_info("TIMEOUT", $sformatf(
      "%s debug: remaining=%0d pending_drain=%0d max_remaining=%0d source_backlog=%0d source_head_empty_marker=%0d source_markers_offered=%0d source_markers_accepted=%0d source_backlog_max=%0d source_offered=%0d source_accepted=%0d live_fill=%0d max_live_fill=%0d push=%0d pop=%0d overwrite=%0d term_done=%0d endofrun_seen=%0d pop_cmd_empty=%0d pop_cmd_usedw=%0d deassm_empty=%0d deassm_full=%0d deassm_usedw=%0d first_push_cycle=%0d first_pop_cycle=%0d first_overwrite_cycle=%0d",
      what,
      m_env.m_scb.remaining_entries(),
      m_env.m_scb.pending_drain_entries(),
      m_env.m_scb.max_remaining_entries(),
      m_env.m_hit_drv.pending_source_items(),
      m_env.m_hit_drv.head_pending_item_is_empty_marker(),
      m_env.m_hit_drv.offered_marker_total,
      m_env.m_hit_drv.accepted_marker_total,
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

  task automatic run_x031_terminate_empty_case();
    int unsigned push_count_before;
    int unsigned pop_count_before;
    int unsigned fill_level_before;
    int unsigned pop_count_after;
    int unsigned fill_level_after;

    configure_and_start(2000);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count_before);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count_before);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level_before);
    if (fill_level_before != 0 || push_count_before != 0 || pop_count_before != 0) begin
      `uvm_error("X031", $sformatf(
        "Precondition for empty terminate was not met: fill=%0d push=%0d pop=%0d",
        fill_level_before, push_count_before, pop_count_before))
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    if (m_env.m_dbg_mon.terminating_drain_done !== 1'b1) begin
      `uvm_error("X031", "terminating_drain_done did not assert after terminate+endofrun")
    end

    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count_after);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level_after);
    if (pop_count_after != pop_count_before) begin
      `uvm_error("X031", $sformatf(
        "POP_COUNT changed across empty terminate: before=%0d after=%0d",
        pop_count_before, pop_count_after))
    end
    if (fill_level_after != 0) begin
      `uvm_error("X031", $sformatf(
        "X031 empty terminate left residual FILL_LEVEL=%0d", fill_level_after))
    end
    if (m_env.m_ctrl_drv.vif.ready !== 1'b1) begin
      `uvm_error("X031", "asi_ctrl_ready did not assert after terminate handshake")
    end
    expect_service_model_accounting("X031 empty terminate", 1, 0);
  endtask

  task automatic run_x032_terminate_during_burst_case();
    same_key_burst_seq burst_seq;
    int unsigned       focus_search_key;
    int unsigned       accepted_at_endofrun;
    int unsigned       push_after_drain;
    int unsigned       pop_after_drain;
    int unsigned       offered_at_endofrun;
    int unsigned       cycles;
    bit                traffic_done;
    bit                saw_post_endofrun_offers;

    configure_and_start(2000);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
    traffic_done = 1'b0;
    burst_seq = same_key_burst_seq::type_id::create("x032_same_key_burst");
    burst_seq.num_hits = 16;
    burst_seq.search_key = focus_search_key;
    fork
      begin
        burst_seq.start(m_env.m_hit_seqr);
        traffic_done = 1'b1;
      end
    join_none

    while (m_env.m_hit_drv.offered_payload_total < 4) begin
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 2_000, "X032 TERMINATING entry");
    send_endofrun_marker();
    wait_clocks(2);
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
      `uvm_error("X032", "Matching empty marker did not latch endofrun_seen during terminate burst")
    end

    accepted_at_endofrun = m_env.m_hit_drv.accepted_payload_total;
    offered_at_endofrun = m_env.m_hit_drv.offered_payload_total;
    saw_post_endofrun_offers = 1'b0;
    cycles = 0;
    while (cycles < 128 &&
           (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
      if (m_env.m_hit_drv.offered_payload_total > offered_at_endofrun) begin
        saw_post_endofrun_offers = 1'b1;
      end
      if (m_env.m_hit_drv.accepted_payload_total > accepted_at_endofrun) begin
        `uvm_error("X032", $sformatf(
          "Accepted payload advanced after endofrun_seen latched: accepted_before=%0d accepted_now=%0d offered_now=%0d pending_source=%0d",
          accepted_at_endofrun,
          m_env.m_hit_drv.accepted_payload_total,
          m_env.m_hit_drv.offered_payload_total,
          m_env.m_hit_drv.pending_source_items()))
        break;
      end
    end
    if (!saw_post_endofrun_offers) begin
      `uvm_error("X032", $sformatf(
        "Terminate-burst case never observed additional offered traffic after endofrun_seen: offered_at_eor=%0d final_offered=%0d",
        offered_at_endofrun, m_env.m_hit_drv.offered_payload_total))
    end

    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    discard_pending_source_backlog("X032 dropped post-endofrun backlog");
    wait_for_scoreboard_idle(120_000, "X032 terminate burst drain");
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_after_drain);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_after_drain);
    if (push_after_drain != m_env.m_hit_drv.accepted_payload_total) begin
      `uvm_error("X032", $sformatf(
        "PUSH_COUNT disagreed with the accepted terminate-burst payload total: push=%0d accepted=%0d pop=%0d",
        push_after_drain,
        m_env.m_hit_drv.accepted_payload_total,
        pop_after_drain))
    end
    if (pop_after_drain != push_after_drain) begin
      `uvm_error("X032", $sformatf(
        "Drain did not retire exactly the accepted terminate-burst residents: push=%0d pop=%0d",
        push_after_drain, pop_after_drain))
    end
    expect_service_model_accounting("X032 terminate during burst", 1, 0);
  endtask

  task automatic run_x033_terminate_during_drain_case();
    same_key_burst_seq               burst_seq;
    ring_buffer_cam_pkg::out_seq_item last_hit;
    int unsigned                     focus_search_key;
    int unsigned                     pop_count;
    int unsigned                     cycles;
    bit                              saw_mid_epoch_hit;
    bit                              ready_seen;
    bit                              early_ready;

    configure_and_start(0);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq = same_key_burst_seq::type_id::create("x033_drain_burst");
    burst_seq.num_hits = 32;
    burst_seq.search_key = focus_search_key;
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_pop_engine_state(3'd4, 40_000, "X033 DRAIN entry");
    saw_mid_epoch_hit = 1'b0;
    cycles = 0;
    while (cycles < 60_000 && !saw_mid_epoch_hit) begin
      if (m_env.m_out_mon.recent_hits.size() > 0) begin
        last_hit = m_env.m_out_mon.recent_hits[$];
        if (last_hit.active_search_key == focus_search_key[7:0] &&
            last_hit.eop !== 1'b1) begin
          saw_mid_epoch_hit = 1'b1;
        end
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    if (!saw_mid_epoch_hit) begin
      `uvm_error("X033", "Did not observe a non-EOP drain beat before issuing TERMINATE")
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_TERMINATING;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    ready_seen = 1'b0;
    early_ready = 1'b0;
    cycles = 0;
    while (cycles < 120_000 && !ready_seen) begin
      @(posedge m_env.m_ctrl_drv.vif.clk);
      cycles++;
      if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
        ready_seen = 1'b1;
        if (m_env.m_out_mon.recent_hits.size() == 0 ||
            m_env.m_out_mon.recent_hits[$].eop !== 1'b1) begin
          early_ready = 1'b1;
        end
      end
    end
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_ctrl_drv.vif.clk);

    if (!ready_seen) begin
      `uvm_error("X033", "TERMINATING wait-for-ready never completed during active drain")
    end
    if (early_ready) begin
      `uvm_error("X033", "asi_ctrl_ready asserted before the final drained hit carried EOP")
    end
    wait_for_scoreboard_idle(120_000, "X033 in-flight drain completion");
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    if (pop_count != 32) begin
      `uvm_error("X033", $sformatf(
        "Active-drain TERMINATE did not retire all hits: pop=%0d expected=32",
        pop_count))
    end
    expect_service_model_accounting("X033 terminate during drain", 1, 0);
  endtask

  task automatic run_x034_terminate_mid_subheader_case();
    same_key_burst_seq burst_seq;
    ring_buffer_cam_pkg::out_seq_item matched_subheader;
    int unsigned       focus_search_key;
    int unsigned       cycles;
    bit                saw_packetization_window;

    configure_and_start(0);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq = same_key_burst_seq::type_id::create("x034_subheader_burst");
    burst_seq.num_hits = 16;
    burst_seq.search_key = focus_search_key;
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_pop_engine_state(3'd4, 40_000, "X034 DRAIN entry");
    saw_packetization_window = 1'b0;
    cycles = 0;
    while (cycles < 40_000 && !saw_packetization_window) begin
      if (m_env.m_dbg_mon.vif.subheader_gen_done === 1'b0 &&
          (m_env.m_dbg_mon.vif.deassembly_fifo_empty !== 1'b1 ||
           m_env.m_dbg_mon.vif.pop_cmd_fifo_empty !== 1'b1)) begin
        saw_packetization_window = 1'b1;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    if (!saw_packetization_window) begin
      `uvm_error("X034", "Did not observe active packetization before issuing TERMINATE")
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    wait_for_subheader_match(
      focus_search_key[7:0], 8'h00, 1'b0, 1'b1, 80_000,
      "X034 completed data-bearing subheader", matched_subheader);
    if (matched_subheader == null ||
        matched_subheader.raw_data[7:0] != ring_buffer_cam_pkg::K237) begin
      `uvm_error("X034", $sformatf(
        "Completed data-bearing subheader was malformed after mid-subheader TERMINATE: raw=0x%09x expected_k237=0x%02x",
        (matched_subheader == null) ? '0 : matched_subheader.raw_data,
        ring_buffer_cam_pkg::K237))
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_for_scoreboard_idle(120_000, "X034 completed subheader drain");
    expect_service_model_accounting("X034 terminate mid-subheader", 1, 0);
  endtask

  task automatic run_x035_terminate_before_soft_reset_case();
    same_key_burst_seq burst_seq;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x035_terminate_soft_reset");
    burst_seq.num_hits = 16;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(16, 20_000, "X035 resident precondition");

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 2_000, "X035 TERMINATING entry");
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X035 terminate then soft_reset");
  endtask

  task automatic run_x036_terminate_after_soft_reset_case();
    same_key_burst_seq burst_seq;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x036_soft_reset_then_terminate");
    burst_seq.num_hits = 16;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(16, 20_000, "X036 resident precondition");

    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X036 soft_reset before terminate");

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 2_000, "X036 TERMINATING entry");
    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
  endtask

  task automatic run_x041_terminate_with_valid_hit_case();
    ring_buffer_cam_pkg::hit_seq_item hit_item;
    int unsigned                      push_before;
    int unsigned                      push_after;
    int unsigned                      inerr_before;
    int unsigned                      inerr_after;
    int unsigned                      accepted_before;
    int unsigned                      accepted_after;

    configure_and_start(2000);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_before);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_before);
    accepted_before = m_env.m_scb.total_ingress_accepted;

    hit_item = ring_buffer_cam_pkg::hit_seq_item::type_id::create("x041_raw_hit");
    hit_item.asic = 4'h1;
    hit_item.ingress_channel = m_cfg.interleaving_index[3:0];
    hit_item.channel = 5'd3;
    hit_item.tcc8n = m_cfg.make_tcc8n_for_lane_key(2, 4'h1, 1'b0);
    hit_item.tcc1n6 = 3'd0;
    hit_item.tfine = 5'd8;
    hit_item.et1n6 = 9'd12;
    hit_item.has_error = 1'b0;
    hit_item.is_empty_marker = 1'b0;

    m_env.m_hit_drv.vif.data <= hit_item.pack_hit();
    m_env.m_hit_drv.vif.channel <= hit_item.input_channel();
    m_env.m_hit_drv.vif.error <= 1'b0;
    m_env.m_hit_drv.vif.valid <= 1'b1;
    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_TERMINATING;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    m_cfg.note_ctrl_cmd(ring_buffer_cam_pkg::CTRL_TERMINATING);
    @(posedge m_env.m_csr_drv.vif.clk);
    m_env.m_hit_drv.vif.valid <= 1'b0;
    m_env.m_hit_drv.vif.data <= '0;
    m_env.m_hit_drv.vif.channel <= '0;
    m_env.m_hit_drv.vif.error <= '0;
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_csr_drv.vif.clk);

    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_for_scoreboard_idle(80_000, "X041 terminate-vs-valid cleanup");

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_after);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_after);
    accepted_after = m_env.m_scb.total_ingress_accepted;
    if (push_after != push_before) begin
      `uvm_error("X041", $sformatf(
        "Good hit concurrent with TERMINATING was unexpectedly accepted: push_before=%0d push_after=%0d",
        push_before, push_after))
    end
    if (inerr_after != inerr_before) begin
      `uvm_error("X041", $sformatf(
        "Good hit concurrent with TERMINATING unexpectedly affected INERR_COUNT: before=%0d after=%0d",
        inerr_before, inerr_after))
    end
    if (accepted_after != accepted_before) begin
      `uvm_error("X041", $sformatf(
        "Scoreboard recorded an accepted ingress beat during same-cycle TERMINATING: before=%0d after=%0d",
        accepted_before, accepted_after))
    end
  endtask

  task automatic run_x053_flush_preempts_drain_case();
    same_key_burst_seq   burst_seq;
    int unsigned         pop_count_before;
    int unsigned         pop_count_after;
    int unsigned         pop_cmd_before;
    bit                  saw_sclr;

    configure_and_start(0);
    burst_seq = same_key_burst_seq::type_id::create("x053_drain_burst");
    burst_seq.num_hits = 64;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_pop_engine_state(3'd4, 40_000, "X053 DRAIN entry");
    pop_cmd_before = m_env.m_dbg_mon.pop_cmd_fifo_usedw;
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count_before);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);

    saw_sclr = 1'b0;
    for (int unsigned i = 0; i < 200; i++) begin
      if (m_env.m_dbg_mon.pop_cmd_fifo_sclr === 1'b1) begin
        saw_sclr = 1'b1;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      if (saw_sclr) begin
        break;
      end
    end
    if (!saw_sclr) begin
      `uvm_error("X053", "RUN_PREPARE did not assert pop_cmd_fifo_sclr during flush entry")
    end

    wait_for_pop_engine_state(3'd6, 40_000, "X053 FLUSHING entry");
    if (m_env.m_dbg_mon.pop_cmd_fifo_usedw > pop_cmd_before) begin
      `uvm_error("X053", $sformatf(
        "pop_cmd_fifo_usedw grew during flush preemption: before=%0d after=%0d",
        pop_cmd_before, m_env.m_dbg_mon.pop_cmd_fifo_usedw))
    end
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count_after);
    if (pop_count_after > pop_count_before) begin
      `uvm_error("X053", $sformatf(
        "POP_COUNT advanced during DRAIN preemption flush: before=%0d after=%0d",
        pop_count_before, pop_count_after))
    end
    m_env.m_scb.note_flush_reset();
    wait_for_scoreboard_idle(120_000, "X053 flush preemption cleanup");
  endtask

  task automatic run_x054_flush_drops_incoming_hits_case();
    same_key_burst_seq burst_seq;
    int unsigned       push_before;
    int unsigned       push_after;
    int unsigned       inerr_after;

    configure_and_start(2000);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_before);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X054 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();

    burst_seq = same_key_burst_seq::type_id::create("x054_flush_burst");
    burst_seq.num_hits = 16;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_clocks(16);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    discard_pending_source_backlog("X054 dropped flush-time ingress backlog");
    wait_for_scoreboard_idle(80_000, "X054 flush drops incoming hits");

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_after);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_after);
    if (push_after != push_before) begin
      `uvm_error("X054", $sformatf(
        "Incoming hits during FLUSHING unexpectedly incremented PUSH_COUNT: before=%0d after=%0d",
        push_before, push_after))
    end
    if (inerr_after != 0) begin
      `uvm_error("X054", $sformatf(
        "Incoming good hits during FLUSHING unexpectedly affected INERR_COUNT=%0d",
        inerr_after))
    end
  endtask

  task automatic run_x055_flush_error_reset_case();
    same_key_burst_seq  burst_seq;
    single_error_hit_seq pre_err;
    single_error_hit_seq flush_err;
    int unsigned        inerr_before;
    int unsigned        inerr_after;

    configure_and_start(0);
    burst_seq = same_key_burst_seq::type_id::create("x055_drain_burst");
    burst_seq.num_hits = 64;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    pre_err = single_error_hit_seq::type_id::create("x055_pre_err");
    pre_err.search_key = m_cfg.lane_key_ord_to_search_key(2);
    pre_err.start(m_env.m_hit_seqr);
    wait_clocks(8);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_before);

    wait_for_pop_engine_state(3'd4, 60_000, "X055 DRAIN entry");
    fork
      begin
        ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      end
      begin
        while (m_env.m_dbg_mon.vif.decision_reg != 3'b011) begin
          @(posedge m_env.m_csr_drv.vif.clk);
        end
        flush_err = single_error_hit_seq::type_id::create("x055_flush_err");
        flush_err.search_key = m_cfg.lane_key_ord_to_search_key(2);
        flush_err.start(m_env.m_hit_seqr);
      end
    join
    wait_for_pop_engine_state(3'd6, 60_000, "X055 FLUSHING entry");

    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_after);
    if (inerr_after != 0) begin
      `uvm_error("X055", $sformatf(
        "INERR_COUNT was not wiped by FLUSHING: before=%0d after=%0d",
        inerr_before, inerr_after))
    end
    m_env.m_scb.note_flush_reset();
    discard_pending_source_backlog("X055 dropped flush-time error backlog");
    wait_for_scoreboard_idle(80_000, "X055 inerr clear under flush");
  endtask

  task automatic run_x061_flush_backlog_popcmd_case();
    same_key_burst_seq   burst_seq;
    int unsigned         pop_count_before_flush;
    int unsigned         pop_count_after;
    int unsigned         cmd_usedw_before;
    bit                  saw_sclr;

    configure_and_start(0);
    burst_seq = same_key_burst_seq::type_id::create("x061_backlog_burst");
    burst_seq.num_hits = 128;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_pop_engine_state(3'd4, 60_000, "X061 DRAIN entry");
    wait_for_pop_cmd_fifo_usedw_at_least(4, 120_000, "X061 backlog in pop_cmd_fifo");
    cmd_usedw_before = m_env.m_dbg_mon.pop_cmd_fifo_usedw;
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);

    saw_sclr = 1'b0;
    wait_for_pop_engine_state(3'd6, 120_000, "X061 FLUSHING entry");
    for (int unsigned i = 0; i < 200; i++) begin
      if (m_env.m_dbg_mon.pop_cmd_fifo_sclr === 1'b1) begin
        saw_sclr = 1'b1;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (!saw_sclr) begin
      `uvm_error("X061", "RUN_PREPARE flush did not assert pop_cmd_fifo_sclr with fifo backlog")
    end
    if (m_env.m_dbg_mon.pop_cmd_fifo_usedw > cmd_usedw_before) begin
      `uvm_error("X061", $sformatf(
        "pop_cmd_fifo_usedw increased before clear: before=%0d after=%0d",
        cmd_usedw_before, m_env.m_dbg_mon.pop_cmd_fifo_usedw))
    end
    if (m_env.m_dbg_mon.pop_cmd_fifo_usedw != 0 &&
        m_env.m_dbg_mon.vif.pop_cmd_fifo_empty !== 1'b1) begin
      `uvm_error("X061", $sformatf(
        "pop_cmd_fifo was not cleared during FLUSHING: usedw=%0d empty=%0d",
        m_env.m_dbg_mon.pop_cmd_fifo_usedw,
        m_env.m_dbg_mon.vif.pop_cmd_fifo_empty))
    end

    wait_for_pop_engine_state(3'd6, 120_000, "X061 FLUSHING entry");
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count_before_flush);
    repeat (64) begin
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count_after);
    if (pop_count_after > pop_count_before_flush) begin
      `uvm_error("X061", $sformatf(
        "POP_COUNT changed during FLUSHING with pop_cmd backlog: before=%0d after=%0d",
        pop_count_before_flush, pop_count_after))
    end
    m_env.m_scb.note_flush_reset();
    wait_for_scoreboard_idle(120_000, "X061 pop_cmd backlog clear");
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

  task automatic run_x038_double_terminate_case();
    same_key_burst_seq burst_seq;
    int unsigned       pop_count_after_first;
    int unsigned       pop_count_after_second;
    int unsigned       fill_level_after_second;

    configure_and_start(0);
    burst_seq = same_key_burst_seq::type_id::create("x038_double_term");
    burst_seq.num_hits = 16;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(16, 20_000, "X038 resident precondition");

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_for_scoreboard_idle(120_000, "X038 first terminate drain");
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count_after_first);
    if (pop_count_after_first != 16) begin
      `uvm_error("X038", $sformatf(
        "First TERMINATE did not drain the expected residents: pop=%0d expected=16",
        pop_count_after_first))
    end

    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_clocks(2);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count_after_second);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level_after_second);
    if (pop_count_after_second != pop_count_after_first) begin
      `uvm_error("X038", $sformatf(
        "Redundant TERMINATE changed POP_COUNT: first=%0d second=%0d",
        pop_count_after_first, pop_count_after_second))
    end
    if (fill_level_after_second != 0) begin
      `uvm_error("X038", $sformatf(
        "Redundant TERMINATE left non-zero fill level: fill=%0d",
        fill_level_after_second))
    end
    if (m_env.m_dbg_mon.run_state_code != ring_buffer_cam_pkg::RUN_STATE_TERMINATING) begin
      `uvm_error("X038", $sformatf(
        "Redundant TERMINATE left the DUT in the wrong state: observed=%0d",
        m_env.m_dbg_mon.run_state_code))
    end
    expect_service_model_accounting("X038 double terminate", 1, 0);
  endtask

  task automatic run_x049_terminate_terminate_idle_case();
    int unsigned fill_level;

    configure_and_start(2000);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    ctrl_send(ring_buffer_cam_pkg::CTRL_IDLE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_IDLE, 2_000, "X049 return to IDLE");
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (fill_level != 0) begin
      `uvm_error("X049", $sformatf(
        "TERMINATE->TERMINATE->IDLE left residual fill_level=%0d",
        fill_level))
    end
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b0) begin
      `uvm_error("X049", "endofrun_seen did not clear after IDLE")
    end
  endtask

  task automatic run_x126_recovery_empty_terminate_case();
    error_burst_seq err_burst;
    int unsigned    fill_level;
    int unsigned    pop_count;

    configure_and_start(2000);
    err_burst = error_burst_seq::type_id::create("x126_error_window");
    err_burst.num_hits = 8;
    err_burst.search_key = m_cfg.lane_key_ord_to_search_key(3);
    err_burst.start(m_env.m_hit_seqr);
    wait_clocks(32);
    restart_after_flush(2000);

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    if (m_env.m_dbg_mon.terminating_drain_done !== 1'b1) begin
      `uvm_error("X126", "Recovered empty terminate did not assert terminating_drain_done")
    end
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    if (fill_level != 0 || pop_count != 0) begin
      `uvm_error("X126", $sformatf(
        "Recovered empty terminate changed counters unexpectedly: fill=%0d pop=%0d",
        fill_level, pop_count))
    end
  endtask

  task automatic run_x127_endofrun_clear_case();
    same_key_burst_seq burst_seq;
    int unsigned       push_count;
    int unsigned       pop_count;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x127_first_run");
    burst_seq.num_hits = 4;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_clocks(16);

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
      `uvm_error("X127", "First terminate did not latch endofrun_seen")
    end

    return_to_idle();
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b0) begin
      `uvm_error("X127", "endofrun_seen did not clear on the IDLE boundary")
    end

    restart_after_flush(2000);
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b0) begin
      `uvm_error("X127", "Second RUN inherited endofrun_seen")
    end

    burst_seq = same_key_burst_seq::type_id::create("x127_second_run");
    burst_seq.num_hits = 4;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(4);
    burst_seq.start(m_env.m_hit_seqr);
    terminate_and_drain(120_000, "X127 second-run terminate");
    expect_service_model_accounting("X127 second-run terminate", 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    if (push_count != 4 || pop_count != 4) begin
      `uvm_error("X127", $sformatf(
        "Second run accounting mismatch after endofrun clear: push=%0d pop=%0d",
        push_count, pop_count))
    end
  endtask

  task automatic run_x128_gts_end_of_run_refresh_case();
    logic [47:0] gts_end_a;
    logic [47:0] gts_end_b;

    configure_and_start(2000);
    wait_clocks(64);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_clocks(1);
    gts_end_a = m_env.m_dbg_mon.gts_end_of_run;
    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);

    return_to_idle();
    wait_clocks(64);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_clocks(1);
    gts_end_b = m_env.m_dbg_mon.gts_end_of_run;
    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);

    if (gts_end_a == 48'd0 || gts_end_b == 48'd0) begin
      `uvm_error("X128", $sformatf(
        "gts_end_of_run snapshots did not capture non-zero values: first=%0d second=%0d",
        gts_end_a, gts_end_b))
    end
    if (gts_end_b <= gts_end_a) begin
      `uvm_error("X128", $sformatf(
        "gts_end_of_run did not refresh on the second TERMINATE: first=%0d second=%0d",
        gts_end_a, gts_end_b))
    end
  endtask

  task automatic run_x039_terminate_then_prep_case();
    int unsigned fill_level;

    configure_and_start(2000);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X039 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b0) begin
      `uvm_error("X039", "RUN_PREPARE did not clear endofrun_seen after TERMINATING")
    end
    if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b1) begin
      `uvm_error("X039", "RUN_PREPARE did not complete the flush handshake after TERMINATING")
    end
    if (fill_level != 0) begin
      `uvm_error("X039", $sformatf(
        "TERMINATE->RUN_PREPARE left non-zero fill_level=%0d",
        fill_level))
    end
  endtask

  task automatic run_x040_terminate_then_running_case();
    same_key_burst_seq burst_seq;
    int unsigned       fill_before;
    int unsigned       fill_after;
    int unsigned       push_before;
    int unsigned       push_after;
    int unsigned       pop_after;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x040_prefill");
    burst_seq.num_hits = 16;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(16, 20_000, "X040 prefill");
    wait_clocks(4);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_before);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_before);

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 2_000, "X040 TERMINATING entry");
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUNNING, 2_000, "X040 RUNNING re-entry");
    wait_clocks(4);

    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_after);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_after);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_after);
    if (fill_after != fill_before) begin
      `uvm_error("X040", $sformatf(
        "TERMINATE->RUNNING boundary unexpectedly changed FILL_LEVEL without FLUSH: before=%0d after=%0d",
        fill_before, fill_after))
    end
    if (push_after != push_before || pop_after != 0) begin
      `uvm_error("X040", $sformatf(
        "TERMINATE->RUNNING boundary unexpectedly changed accounting: push_before=%0d push_after=%0d pop_after=%0d",
        push_before, push_after, pop_after))
    end
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b0) begin
      `uvm_error("X040", "RUNNING re-entry inherited endofrun_seen without an intervening FLUSH")
    end
    m_env.m_scb.note_intentional_nonempty_end(fill_after);
  endtask

  task automatic run_x042_prelatched_endofrun_case();
    int unsigned fill_level;

    configure_and_start(2000);
    send_endofrun_marker();
    wait_clocks(4);
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
      `uvm_error("X042", "Precondition failed: endofrun_seen did not latch before TERMINATING")
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    if (m_env.m_dbg_mon.terminating_drain_done !== 1'b1) begin
      `uvm_error("X042", "Pre-latched endofrun TERMINATE did not reach drain_done promptly")
    end
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (fill_level != 0) begin
      `uvm_error("X042", $sformatf(
        "Pre-latched endofrun TERMINATE left residual fill_level=%0d",
        fill_level))
    end
  endtask

  task automatic run_x044_bad_hits_survive_terminate_case();
    error_burst_seq err_burst;
    int unsigned    inerr_before;
    int unsigned    inerr_after;

    configure_and_start(2000);
    err_burst = error_burst_seq::type_id::create("x044_bad_burst");
    err_burst.num_hits = 8;
    err_burst.search_key = m_cfg.lane_key_ord_to_search_key(2);
    err_burst.start(m_env.m_hit_seqr);
    wait_clocks(32);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_before);

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_after);
    if (inerr_before != 8 || inerr_after != inerr_before) begin
      `uvm_error("X044", $sformatf(
        "INERR_COUNT did not survive TERMINATING: before=%0d after=%0d expected=8",
        inerr_before, inerr_after))
    end
  endtask

  task automatic run_x045_terminate_ready_with_backlog_case();
    same_key_burst_seq burst_seq;
    int unsigned       cycles;
    bit                saw_ready;
    bit                saw_backlog_during_terminate;
    bit                saw_nonempty_at_ready;
    bit                ready_pop_cmd_empty;
    bit                ready_deassembly_empty;
    bit                ready_term_done;

    configure_and_start(0);
    set_sink_ready(1'b0);
    burst_seq = same_key_burst_seq::type_id::create("x045_backlog_burst");
    burst_seq.num_hits = 256;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_pop_engine_state(3'd4, 80_000, "X045 DRAIN entry");
    wait_for_pop_cmd_fifo_usedw_at_least(1, 80_000, "X045 pop_cmd backlog");

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_TERMINATING;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    saw_ready = 1'b0;
    saw_backlog_during_terminate = 1'b0;
    saw_nonempty_at_ready = 1'b0;
    ready_pop_cmd_empty = 1'b1;
    ready_deassembly_empty = 1'b1;
    ready_term_done = 1'b0;
    for (cycles = 0; cycles < 40_000; cycles++) begin
      @(posedge m_env.m_ctrl_drv.vif.clk);
      if (m_env.m_dbg_mon.vif.pop_cmd_fifo_empty !== 1'b1 ||
          m_env.m_dbg_mon.vif.deassembly_fifo_empty !== 1'b1) begin
        saw_backlog_during_terminate = 1'b1;
      end
      if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
        saw_ready = 1'b1;
        ready_pop_cmd_empty = m_env.m_dbg_mon.vif.pop_cmd_fifo_empty;
        ready_deassembly_empty = m_env.m_dbg_mon.vif.deassembly_fifo_empty;
        ready_term_done = m_env.m_dbg_mon.terminating_drain_done;
        if (ready_pop_cmd_empty !== 1'b1 || ready_deassembly_empty !== 1'b1) begin
          saw_nonempty_at_ready = 1'b1;
        end
        break;
      end
    end
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_ctrl_drv.vif.clk);

    if (!saw_ready) begin
      `uvm_error("X045", "Held TERMINATING command never re-acknowledged under local backlog")
    end
    if (!saw_backlog_during_terminate) begin
      `uvm_error("X045", "Terminate-ready audit never saw any local backlog while the held command was pending")
    end
    if (!saw_nonempty_at_ready) begin
      `uvm_error("X045", $sformatf(
        "asi_ctrl_ready stayed masked until local buffers emptied: pop_cmd_empty=%0d deassembly_empty=%0d term_done=%0d",
        ready_pop_cmd_empty,
        ready_deassembly_empty,
        ready_term_done))
    end
    if (ready_term_done !== 1'b1) begin
      `uvm_error("X045", $sformatf(
        "asi_ctrl_ready re-acknowledged before terminating_drain_done asserted: pop_cmd_empty=%0d deassembly_empty=%0d term_done=%0d",
        ready_pop_cmd_empty,
        ready_deassembly_empty,
        ready_term_done))
    end

    set_sink_ready(1'b1);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X045 cleanup RUN_PREPARE");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_scoreboard_idle(120_000, "X045 cleanup flush");
  endtask

  task automatic run_x043_full_ring_terminate_case();
    sequential_keys_seq key_seq;
    int unsigned        partition_size;
    int unsigned        visit_idx;
    logic [3:0]         visit_mask;
    logic [3:0]         expected_mask;
    int unsigned        hit_before;
    int unsigned        search_cycles;
    int unsigned        pop_count;
    int unsigned        fill_level;

    configure_and_start(2000);
    partition_size = (m_cfg.n_partitions == 0) ? m_cfg.ring_buffer_n_entry :
                     (m_cfg.ring_buffer_n_entry / m_cfg.n_partitions);
    key_seq = sequential_keys_seq::type_id::create("x043_full_ring");
    key_seq.num_keys = m_cfg.ring_buffer_n_entry / 4;
    key_seq.hits_per_key = 4;
    key_seq.start_key = 4;
    key_seq.start(m_env.m_hit_seqr);

    wait_for_push_count(m_cfg.ring_buffer_n_entry, 160_000, "X043 full-ring fill");
    hit_before = m_env.m_out_mon.total_hits_seen;
    expected_mask = (1 << m_cfg.n_partitions) - 1;
    visit_mask = '0;

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 2_000, "X043 TERMINATING entry");
    search_cycles = 0;
    while (search_cycles < 400_000 &&
           (m_env.m_dbg_mon.terminating_drain_done !== 1'b1 ||
            m_env.m_out_mon.total_hits_seen < (hit_before + m_cfg.ring_buffer_n_entry))) begin
      if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
        visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
        if (visit_idx >= m_cfg.n_partitions) begin
          `uvm_error("X043", $sformatf(
            "Drain issued from partition outside active build range: partition=%0d n_partitions=%0d addr=%0d",
            visit_idx, m_cfg.n_partitions, m_env.m_dbg_mon.vif.pop_issue_addr))
        end else begin
          visit_mask[visit_idx[1:0]] = 1'b1;
        end
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      search_cycles++;
    end
    if (m_env.m_dbg_mon.terminating_drain_done !== 1'b1) begin
      `uvm_error("X043", "Full-ring TERMINATING never reached drain_done")
    end
    wait_for_ctrl_ready(1'b1, 40_000, "X043 terminate ack");
    wait_for_hit_output_count(hit_before + m_cfg.ring_buffer_n_entry, 3_000_000, "X043 full-ring hit drain");
    wait_for_scoreboard_idle(240_000, "X043 full-ring terminate drain");
    expect_service_model_accounting("X043 full-ring terminate drain", 1, 0);

    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (pop_count != m_cfg.ring_buffer_n_entry || fill_level != 0) begin
      `uvm_error("X043", $sformatf(
        "Full-ring terminate accounting mismatch: pop=%0d fill=%0d expected_pop=%0d",
        pop_count, fill_level, m_cfg.ring_buffer_n_entry))
    end
    if ((visit_mask & expected_mask) != expected_mask) begin
      `uvm_error("X043", $sformatf(
        "Full-ring terminate did not visit all active partitions: observed_mask=0x%0h expected=0x%0h",
        visit_mask, expected_mask))
    end
  endtask

  task automatic run_x046_filllevel_read_during_terminate_case();
    same_key_burst_seq burst_seq;
    int unsigned       fill_prev;
    int unsigned       fill_now;
    int unsigned       push_snapshot;
    int unsigned       source_wait_cycles;
    int unsigned       stable_cycles;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x046_terminate_filllevel");
    burst_seq.num_hits = 64;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_push_count(64, 40_000, "X046 pre-terminate fill");
    source_wait_cycles = 0;
    while (m_env.m_hit_drv.pending_source_items() != 0 && source_wait_cycles < 40_000) begin
      @(posedge m_env.m_csr_drv.vif.clk);
      source_wait_cycles++;
    end
    if (m_env.m_hit_drv.pending_source_items() != 0) begin
      `uvm_error("X046", $sformatf(
        "Source backlog did not drain before TERMINATING: backlog=%0d after %0d cycles",
        m_env.m_hit_drv.pending_source_items(), source_wait_cycles))
    end
    push_snapshot = m_env.m_dbg_mon.dbg_push_cnt[31:0];
    stable_cycles = 0;
    while (stable_cycles < 8 && source_wait_cycles < 60_000) begin
      @(posedge m_env.m_csr_drv.vif.clk);
      source_wait_cycles++;
      if (m_env.m_dbg_mon.dbg_push_cnt[31:0] == push_snapshot) begin
        stable_cycles++;
      end else begin
        push_snapshot = m_env.m_dbg_mon.dbg_push_cnt[31:0];
        stable_cycles = 0;
      end
    end
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 2_000, "X046 TERMINATING entry");
    stable_cycles = 0;
    while (stable_cycles < 2_000 && m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
      @(posedge m_env.m_csr_drv.vif.clk);
      stable_cycles++;
    end
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
      `uvm_error("X046", "endofrun_seen did not latch after TERMINATING marker")
    end
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_prev);
    for (int unsigned sample = 0; sample < 128; sample++) begin
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_now);
      if (fill_now > fill_prev) begin
        `uvm_error("X046", $sformatf(
          "FILL_LEVEL increased during TERMINATING: prev=%0d now=%0d sample=%0d",
          fill_prev, fill_now, sample))
      end
      if (fill_now == 32'hffff_ffff) begin
        `uvm_error("X046", "FILL_LEVEL read all-ones during TERMINATING")
      end
      fill_prev = fill_now;
      if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
        break;
      end
      wait_clocks(2);
    end
    wait_for_ctrl_ready(1'b1, 40_000, "X046 terminate ack");
    wait_for_scoreboard_idle(120_000, "X046 terminate fill-level drain");
    expect_service_model_accounting("X046 terminate fill-level drain", 1, 0);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_now);
    if (fill_now != 0) begin
      `uvm_error("X046", $sformatf(
        "TERMINATING fill-level read case left residual fill_level=%0d",
        fill_now))
    end
  endtask

  task automatic run_x048_multihot_terminate_case();
    int unsigned fill_level;

    configure_and_start(2000);
    m_env.m_ctrl_drv.vif.data <= (ring_buffer_cam_pkg::CTRL_TERMINATING |
                                  ring_buffer_cam_pkg::CTRL_RUNNING);
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
    if (m_env.m_dbg_mon.run_state_code != ring_buffer_cam_pkg::RUN_STATE_ERROR) begin
      `uvm_error("X048", $sformatf(
        "Illegal TERMINATING|RUNNING payload did not enter ERROR: observed=%0d",
        m_env.m_dbg_mon.run_state_code))
    end
    if (m_env.m_ctrl_drv.vif.ready !== 1'b0) begin
      `uvm_error("X048", "Illegal TERMINATING|RUNNING payload should not acknowledge")
    end
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (fill_level != 0) begin
      `uvm_error("X048", $sformatf(
        "Illegal TERMINATING|RUNNING payload corrupted FILL_LEVEL=%0d",
        fill_level))
    end
  endtask

  task automatic run_x047_terminate_push_count_write_case();
    same_key_burst_seq burst_seq;
    int unsigned       push_before;
    int unsigned       push_after;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x047_preterminate_fill");
    burst_seq.num_hits = 8;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(8, 20_000, "X047 pre-terminate push count");
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_before);

    fork
      begin
        csr_write(CSR_PUSH_COUNT_ADDR, 32'hdead_beef);
      end
      begin
        ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
      end
    join
    send_endofrun_marker();
    wait_for_ctrl_ready(1'b1, 40_000, "X047 terminate ack");
    wait_for_scoreboard_idle(120_000, "X047 terminate after push-count write");
    expect_service_model_accounting("X047 terminate after push-count write", 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_after);
    if (push_after != push_before) begin
      `uvm_error("X047", $sformatf(
        "PUSH_COUNT write in flight changed the counter: before=%0d after=%0d",
        push_before, push_after))
    end
    if (m_env.m_dbg_mon.run_state_code != ring_buffer_cam_pkg::RUN_STATE_TERMINATING) begin
      `uvm_error("X047", $sformatf(
        "Concurrent PUSH_COUNT write disturbed run_state_cmd: observed=%0d",
        m_env.m_dbg_mon.run_state_code))
    end
  endtask

  task automatic run_x050_filllevel_underflow_guard_case();
    single_push_pop_seq single_seq;
    int unsigned        fill_now;

    configure_and_start(0);
    single_seq = single_push_pop_seq::type_id::create("x050_one_hit");
    single_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    single_seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(80_000, "X050 exact-balance precondition");
    wait_clocks(4);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_now);
    if (fill_now != 0) begin
      `uvm_error("X050", $sformatf(
        "Exact-balance precondition left non-zero FILL_LEVEL=%0d",
        fill_now))
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    for (int unsigned sample = 0; sample < 64; sample++) begin
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_now);
      if (fill_now == 32'hffff_ffff) begin
        `uvm_error("X050", "FILL_LEVEL underflowed to all-ones during exact-balance TERMINATING")
      end
      if (fill_now != 0) begin
        `uvm_error("X050", $sformatf(
          "Exact-balance TERMINATING should keep FILL_LEVEL at 0, observed=%0d sample=%0d",
          fill_now, sample))
      end
      if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
        break;
      end
      wait_clocks(1);
    end
    wait_for_ctrl_ready(1'b1, 40_000, "X050 terminate ack");
    wait_for_scoreboard_idle(40_000, "X050 exact-balance terminate");
  endtask

  task automatic run_x052_flush_full_ring_case();
    sequential_keys_seq key_seq;
    int unsigned        hit_before;
    int unsigned        fill_level;
    int unsigned        push_count;
    int unsigned        pop_count;
    int unsigned        overwrite_count;

    configure_and_start(2000);
    key_seq = sequential_keys_seq::type_id::create("x052_full_ring");
    key_seq.num_keys = m_cfg.ring_buffer_n_entry / 4;
    key_seq.hits_per_key = 4;
    key_seq.start_key = 4;
    key_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(m_cfg.ring_buffer_n_entry, 160_000, "X052 full-ring fill");
    hit_before = m_env.m_out_mon.total_hits_seen;
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X052 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_scoreboard_idle(120_000, "X052 full-ring flush");

    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    if (fill_level != 0 || push_count != 0 || pop_count != 0 || overwrite_count != 0) begin
      `uvm_error("X052", $sformatf(
        "Full-ring FLUSH did not clear accounting: fill=%0d push=%0d pop=%0d overwrite=%0d",
        fill_level, push_count, pop_count, overwrite_count))
    end
    if (m_env.m_out_mon.total_hits_seen != hit_before) begin
      `uvm_error("X052", $sformatf(
        "Full-ring FLUSH emitted residual outputs: before=%0d after=%0d",
        hit_before, m_env.m_out_mon.total_hits_seen))
    end
  endtask

  task automatic run_x051_flush_empty_case();
    int unsigned fill_level;
    int unsigned inerr_count;
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned cache_miss_count;

    configure_and_start(2000);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X051 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b1) begin
      `uvm_error("X051", "Empty RUN_PREPARE flush never asserted run_mgmt_flushed")
    end
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    if (fill_level != 0 || inerr_count != 0 || push_count != 0 ||
        pop_count != 0 || overwrite_count != 0 || cache_miss_count != 0) begin
      `uvm_error("X051", $sformatf(
        "Empty RUN_PREPARE flush counters not clean: fill=%0d inerr=%0d push=%0d pop=%0d overwrite=%0d cache_miss=%0d",
        fill_level, inerr_count, push_count, pop_count, overwrite_count, cache_miss_count))
    end
    if (m_env.m_ctrl_drv.vif.ready !== 1'b1) begin
      `uvm_error("X051", "Empty RUN_PREPARE flush did not acknowledge")
    end
    wait_for_scoreboard_idle(40_000, "X051 empty flush");
  endtask

  task automatic run_x057_flush_cross_partition_case();
    same_key_burst_seq burst_seq;
    int unsigned       partition_boundary;
    bit                saw_boundary;
    bit                saw_ram_done;
    bit                saw_cam_done;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x057_prefill");
    burst_seq.num_hits = 64;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(64, 20_000, "X057 pre-flush fill");

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X057 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();

    partition_boundary = (m_cfg.n_partitions == 0) ? m_cfg.ring_buffer_n_entry :
                         (m_cfg.ring_buffer_n_entry / m_cfg.n_partitions);
    saw_boundary = 1'b0;
    saw_ram_done = 1'b0;
    saw_cam_done = 1'b0;
    for (int unsigned cycles = 0; cycles < 180_000; cycles++) begin
      if (int'(m_env.m_dbg_mon.flush_cam_wraddr) >= partition_boundary) begin
        saw_boundary = 1'b1;
      end
      if (m_env.m_dbg_mon.pop_flush_ram_done === 1'b1) begin
        saw_ram_done = 1'b1;
      end
      if (m_env.m_dbg_mon.pop_flush_cam_done === 1'b1) begin
        saw_cam_done = 1'b1;
      end
      if (m_env.m_dbg_mon.run_mgmt_flushed === 1'b1 &&
          m_env.m_ctrl_drv.vif.ready === 1'b1) begin
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b1 ||
        m_env.m_ctrl_drv.vif.ready !== 1'b1) begin
      `uvm_error("X057", $sformatf(
        "Cross-partition FLUSH did not reach the flushed/ready state: flushed=%0d ready=%0d cam_addr=%0d ram_addr=%0d",
        m_env.m_dbg_mon.run_mgmt_flushed,
        m_env.m_ctrl_drv.vif.ready,
        m_env.m_dbg_mon.flush_cam_wraddr,
        m_env.m_dbg_mon.flush_ram_wraddr))
    end
    if (!saw_boundary) begin
      `uvm_error("X057", $sformatf(
        "FLUSH never crossed the partition boundary: boundary=%0d final_cam_addr=%0d",
        partition_boundary,
        m_env.m_dbg_mon.flush_cam_wraddr))
    end
    if (!saw_ram_done || !saw_cam_done) begin
      `uvm_error("X057", $sformatf(
        "FLUSH did not observe both RAM and CAM completion pulses: ram_done=%0d cam_done=%0d",
        saw_ram_done, saw_cam_done))
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_scoreboard_idle(120_000, "X057 cross-partition flush");
  endtask

  task automatic run_x059_flush_clears_stale_epoch_case();
    same_key_burst_seq burst_seq;
    bit [7:0]          focus_search_key;
    int unsigned       overwrite_count;
    int unsigned       cache_miss_count;
    int unsigned       fill_level;
    int unsigned       push_count;
    int unsigned       pop_count;

    configure_and_start(2000);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq = same_key_burst_seq::type_id::create("x059_prefill");
    burst_seq.num_hits = 32;
    burst_seq.search_key = focus_search_key;
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(32, 20_000, "X059 pre-flush fill");

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X059 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);

    csr_write(CSR_EXPECTED_LAT_ADDR, 32'd2000);
    ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
    wait_clocks(2);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
    wait_clocks(20);

    burst_seq = same_key_burst_seq::type_id::create("x059_postflush_same_key");
    burst_seq.num_hits = 32;
    burst_seq.search_key = focus_search_key;
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(32, 20_000, "X059 post-flush same-key refill");

    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (overwrite_count != 0 || cache_miss_count != 0 || fill_level != 32) begin
      `uvm_error("X059", $sformatf(
        "Post-flush same-key refill did not start from a clean epoch: overwrite=%0d cache_miss=%0d fill=%0d expected_fill=32",
        overwrite_count, cache_miss_count, fill_level))
    end

    terminate_and_drain(120_000, "X059 post-flush same-key drain");
    expect_service_model_accounting("X059 post-flush same-key drain", 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    if (push_count != 32 || pop_count != 32) begin
      `uvm_error("X059", $sformatf(
        "Post-flush same-key accounting mismatch: push=%0d pop=%0d expected=32",
        push_count, pop_count))
    end
  endtask

  task automatic run_x065_flush_then_terminate_case();
    int unsigned fill_level;
    int unsigned hit_before;

    configure_and_start(2000);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X065 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    hit_before = m_env.m_out_mon.total_hits_seen;
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 2_000, "X065 TERMINATING entry");
    wait_for_ctrl_ready(1'b1, 2_000, "X065 terminate ack after empty flush");
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (fill_level != 0) begin
      `uvm_error("X065", $sformatf(
        "Post-flush TERMINATING left residual fill_level=%0d",
        fill_level))
    end
    wait_for_scoreboard_idle(40_000, "X065 flush then terminate");
    if (m_env.m_out_mon.total_hits_seen != hit_before) begin
      `uvm_error("X065", $sformatf(
        "Post-flush TERMINATING emitted stray output beats: before=%0d after=%0d",
        hit_before, m_env.m_out_mon.total_hits_seen))
    end
  endtask

  task automatic run_x058_flush_from_running_case();
    same_key_burst_seq burst_seq;
    int unsigned       hit_before;
    int unsigned       fill_level;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x058_flush_running");
    burst_seq.num_hits = 16;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(16, 20_000, "X058 pre-flush fill");
    hit_before = m_env.m_out_mon.total_hits_seen;
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X058 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_scoreboard_idle(80_000, "X058 flush from RUNNING");
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (fill_level != 0) begin
      `uvm_error("X058", $sformatf(
        "RUNNING->RUN_PREPARE flush left residual fill_level=%0d",
        fill_level))
    end
    if (m_env.m_out_mon.total_hits_seen != hit_before) begin
      `uvm_error("X058", $sformatf(
        "RUNNING->RUN_PREPARE flush emitted residual outputs: before=%0d after=%0d",
        hit_before, m_env.m_out_mon.total_hits_seen))
    end
  endtask

  task automatic run_x056_flush_restart_good_push_case();
    same_key_burst_seq  burst_seq;
    single_push_pop_seq single_seq;
    int unsigned        fill_level;
    int unsigned        push_count;
    int unsigned        pop_count;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x056_preflush_fill");
    burst_seq.num_hits = 16;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(16, 20_000, "X056 pre-flush fill");
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X056 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    restart_after_flush(2000);

    single_seq = single_push_pop_seq::type_id::create("x056_postflush_good");
    single_seq.search_key = m_cfg.lane_key_ord_to_search_key(4);
    single_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(1, 20_000, "X056 first good push after restart");
    wait_clocks(2);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    if (!((fill_level == 1 && pop_count == 0) ||
          (fill_level == 0 && pop_count == 1))) begin
      `uvm_error("X056", $sformatf(
        "First good push after flush did not see a clean post-restart trajectory: fill=%0d pop=%0d expected=(1,0) or (0,1)",
        fill_level,
        pop_count))
    end

    terminate_and_drain(80_000, "X056 post-flush restart drain");
    expect_service_model_accounting("X056 post-flush restart drain", 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    if (push_count != 1 || pop_count != 1) begin
      `uvm_error("X056", $sformatf(
        "Post-flush restart counters mismatch: push=%0d pop=%0d expected=1",
        push_count, pop_count))
    end
  endtask

  task automatic run_x066_duplicate_prep_noop_case();
    int unsigned flush_grants_before;
    bit          saw_restart;

    configure_and_start(2000);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X066 first RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    flush_grants_before = m_env.m_dbg_mon.pop_flush_grant_count;
    saw_restart = 1'b0;

    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    for (int unsigned cycles = 0; cycles < 256; cycles++) begin
      if (m_env.m_dbg_mon.run_mgmt_flush_memory_start === 1'b1 ||
          m_env.m_dbg_mon.pop_flush_grant_count > flush_grants_before) begin
        saw_restart = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (saw_restart) begin
      `uvm_error("X066", "Duplicate RUN_PREPARE retriggered the flush walk after already reaching flushed state")
    end
    if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b1 || m_env.m_ctrl_drv.vif.ready !== 1'b1) begin
      `uvm_error("X066", $sformatf(
        "Duplicate RUN_PREPARE did not leave PREP in the flushed/ready state: flushed=%0d ready=%0d",
        m_env.m_dbg_mon.run_mgmt_flushed, m_env.m_ctrl_drv.vif.ready))
    end
  endtask

  task automatic run_x062_flush_start_latch_case();
    bit          prev_flush_start;
    int unsigned flush_start_rises;
    bit          saw_prep;
    bit          saw_flush_grant;
    bit          saw_ready;

    configure_and_start(2000);
    saw_prep = 1'b0;
    saw_flush_grant = 1'b0;
    saw_ready = 1'b0;
    prev_flush_start = 1'b0;
    flush_start_rises = 0;

    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_RUN_PREPARE;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    for (int unsigned cycles = 0; cycles < 180_000; cycles++) begin
      @(posedge m_env.m_ctrl_drv.vif.clk);
      if (!saw_prep &&
          m_env.m_dbg_mon.run_state_code == ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE) begin
        saw_prep = 1'b1;
        m_env.m_scb.note_flush_reset();
      end
      if (m_env.m_dbg_mon.run_mgmt_flush_memory_start === 1'b1 &&
          prev_flush_start === 1'b0) begin
        flush_start_rises++;
      end
      prev_flush_start = m_env.m_dbg_mon.run_mgmt_flush_memory_start;
      if (m_env.m_dbg_mon.pop_flush_grant_count != 0) begin
        saw_flush_grant = 1'b1;
      end
      if (flush_start_rises > 1) begin
        `uvm_error("X062", $sformatf(
          "Held RUN_PREPARE retriggered flush start more than once: rises=%0d",
          flush_start_rises))
      end
      if (m_env.m_ctrl_drv.vif.ready === 1'b1 &&
          m_env.m_dbg_mon.run_mgmt_flushed === 1'b1) begin
        saw_ready = 1'b1;
        break;
      end
    end
    m_cfg.note_ctrl_cmd(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_ctrl_drv.vif.clk);

    if (!saw_prep) begin
      `uvm_error("X062", "Held RUN_PREPARE did not move the DUT into RUN_PREPARE")
    end
    if (flush_start_rises != 1 || !saw_flush_grant) begin
      `uvm_error("X062", $sformatf(
        "Held RUN_PREPARE did not produce exactly one sustained flush start: rises=%0d flush_grants_seen=%0d",
        flush_start_rises, saw_flush_grant))
    end
    if (!saw_ready) begin
      `uvm_error("X062", "Held RUN_PREPARE never reached the flushed/ready acknowledgement point")
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_clocks(4);
  endtask

  task automatic run_x063_flush_beats_push_erase_case();
    same_key_burst_seq burst_seq;
    int unsigned       push_write_grants_before_flush;
    int unsigned       push_erase_grants_before_flush;
    int unsigned       flush_grants_before_flush;
    bit                saw_active_erase;
    bit                saw_flush_progress;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x063_overwrite_burst");
    burst_seq.num_hits = m_cfg.ring_buffer_n_entry + 64;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    saw_active_erase = 1'b0;
    for (int unsigned cycles = 0; cycles < 60_000; cycles++) begin
      if (m_env.m_dbg_mon.push_state_code === 1'b1 &&
          m_env.m_dbg_mon.dbg_overwrite_cnt[31:0] != 0) begin
        saw_active_erase = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (!saw_active_erase) begin
      `uvm_error("X063", "Precondition failed: did not observe an active push-erase phase before FLUSHING")
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X063 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    wait_for_pop_engine_state(3'd6, 120_000, "X063 FLUSHING entry");

    push_write_grants_before_flush = m_env.m_dbg_mon.push_write_grant_count;
    push_erase_grants_before_flush = m_env.m_dbg_mon.push_erase_grant_count;
    flush_grants_before_flush = m_env.m_dbg_mon.pop_flush_grant_count;
    saw_flush_progress = 1'b0;
    for (int unsigned cycles = 0; cycles < 256; cycles++) begin
      if (m_env.m_dbg_mon.pop_flush_grant_count > flush_grants_before_flush) begin
        saw_flush_progress = 1'b1;
      end
      if (m_env.m_dbg_mon.push_write_grant_count > push_write_grants_before_flush ||
          m_env.m_dbg_mon.push_erase_grant_count > push_erase_grants_before_flush ||
          m_env.m_dbg_mon.vif.push_write_grant === 1'b1 ||
          m_env.m_dbg_mon.vif.push_erase_grant === 1'b1) begin
        `uvm_error("X063", $sformatf(
          "FLUSH lost arbiter priority to push traffic: push_write_count %0d->%0d push_erase_count %0d->%0d live_write=%0d live_erase=%0d flush_grant=%0d",
          push_write_grants_before_flush, m_env.m_dbg_mon.push_write_grant_count,
          push_erase_grants_before_flush, m_env.m_dbg_mon.push_erase_grant_count,
          m_env.m_dbg_mon.vif.push_write_grant,
          m_env.m_dbg_mon.vif.push_erase_grant,
          m_env.m_dbg_mon.pop_flush_grant))
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (!saw_flush_progress) begin
      `uvm_error("X063", "FLUSH never consumed arbiter grants after entering FLUSHING")
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_clocks(4);
  endtask

  task automatic run_x060_flush_backlog_deassembly_case();
    same_key_burst_seq burst_seq;
    int unsigned       push_before;
    int unsigned       push_after;
    bit                saw_backlog;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x060_deassembly_backlog");
    burst_seq.num_hits = 64;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    saw_backlog = 1'b0;
    for (int unsigned cycles = 0; cycles < 20_000; cycles++) begin
      if (m_env.m_dbg_mon.deassembly_fifo_usedw != 0) begin
        saw_backlog = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (!saw_backlog) begin
      `uvm_error("X060", "Did not observe any queued data in deassembly_fifo before FLUSHING")
    end

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_before);
    if (push_before == 0) begin
      `uvm_error("X060", "Precondition failed: deassembly backlog never translated into pre-flush PUSH_COUNT activity")
    end
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X060 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_scoreboard_idle(80_000, "X060 deassembly backlog flush");
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_after);
    if (push_after != 0) begin
      `uvm_error("X060", $sformatf(
        "FLUSH with deassembly backlog did not clear PUSH_COUNT: before=%0d after=%0d",
        push_before, push_after))
    end
    if (m_env.m_dbg_mon.vif.deassembly_fifo_empty !== 1'b1) begin
      `uvm_error("X060", $sformatf(
        "deassembly_fifo was not empty after FLUSHING: usedw=%0d empty=%0d",
        m_env.m_dbg_mon.deassembly_fifo_usedw,
        m_env.m_dbg_mon.vif.deassembly_fifo_empty))
    end
  endtask

  task automatic run_x067_flush_complete_sync_case();
    logic [47:0] gts_before;
    bit          saw_sync_ack;

    configure_and_start(2000);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X067 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    gts_before = m_env.m_dbg_mon.gts_end_of_run;
    saw_sync_ack = 1'b0;
    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_SYNC;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    @(posedge m_env.m_ctrl_drv.vif.clk);
    saw_sync_ack = (m_env.m_ctrl_drv.vif.ready === 1'b1);
    m_cfg.note_ctrl_cmd(ring_buffer_cam_pkg::CTRL_SYNC);
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_ctrl_drv.vif.clk);
    if (!saw_sync_ack) begin
      `uvm_error("X067", "SYNC did not acknowledge immediately after flush completion")
    end
    if (m_env.m_dbg_mon.gts_end_of_run != gts_before) begin
      `uvm_error("X067", $sformatf(
        "SYNC after flush unexpectedly changed gts_end_of_run: before=%0d after=%0d",
        gts_before, m_env.m_dbg_mon.gts_end_of_run))
    end
  endtask

  task automatic run_x068_prep_clears_endofrun_case();
    configure_and_start(2000);
    send_endofrun_marker();
    wait_clocks(8);
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
      `uvm_error("X068", "Precondition failed: matching empty marker did not latch endofrun_seen")
    end
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X068 RUN_PREPARE entry");
    if (m_env.m_dbg_mon.endofrun_seen !== 1'b0) begin
      `uvm_error("X068", "RUN_PREPARE entry did not clear endofrun_seen")
    end
    m_env.m_scb.note_flush_reset();
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
  endtask

  task automatic run_x069_flush_ready_latch_case();
    same_key_burst_seq burst_seq;
    bit                saw_prep;
    bit                saw_flushed;
    bit                saw_flushed_waiting_on_other_qualifier;
    bit                saw_ready;
    bit                saw_ready_drop;
    int unsigned       ready_rises;
    int unsigned       steady_ready_cycles;

    configure_and_start(0);
    burst_seq = same_key_burst_seq::type_id::create("x069_backlog_burst");
    burst_seq.num_hits = 128;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_pop_cmd_fifo_usedw_at_least(1, 40_000, "X069 pop_cmd backlog precondition");

    saw_prep = 1'b0;
    saw_flushed = 1'b0;
    saw_flushed_waiting_on_other_qualifier = 1'b0;
    saw_ready = 1'b0;
    saw_ready_drop = 1'b0;
    ready_rises = 0;
    steady_ready_cycles = 0;

    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_RUN_PREPARE;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    m_env.m_scb.note_flush_reset();
    for (int unsigned cycles = 0; cycles < 180_000; cycles++) begin
      @(posedge m_env.m_ctrl_drv.vif.clk);
      if (!saw_prep &&
          m_env.m_dbg_mon.run_state_code == ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE) begin
        saw_prep = 1'b1;
      end
      if (m_env.m_dbg_mon.run_mgmt_flushed === 1'b1) begin
        saw_flushed = 1'b1;
        if (m_env.m_ctrl_drv.vif.ready !== 1'b1) begin
          saw_flushed_waiting_on_other_qualifier = 1'b1;
        end
      end else if (saw_flushed && !saw_ready) begin
        `uvm_error("X069", "run_mgmt_flushed deasserted before RUN_PREPARE ready became visible")
      end
      if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
        if (!saw_ready) begin
          ready_rises++;
        end
        saw_ready = 1'b1;
        steady_ready_cycles++;
        if (steady_ready_cycles >= 8) begin
          break;
        end
      end else if (saw_ready) begin
        saw_ready_drop = 1'b1;
      end
    end
    m_cfg.note_ctrl_cmd(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_ctrl_drv.vif.clk);

    if (!saw_prep) begin
      `uvm_error("X069", "Held RUN_PREPARE never entered RUN_PREPARE")
    end
    if (!saw_flushed || !saw_ready) begin
      `uvm_error("X069", $sformatf(
        "RUN_PREPARE latch test did not reach the flushed/ready point: flushed_seen=%0d ready_seen=%0d",
        saw_flushed, saw_ready))
    end
    if (!saw_flushed_waiting_on_other_qualifier) begin
      `uvm_error("X069", "run_mgmt_flushed never needed to stay latched while another PREP qualifier was still pending")
    end
    if (ready_rises != 1 || saw_ready_drop) begin
      `uvm_error("X069", $sformatf(
        "RUN_PREPARE ready glitched instead of staying latched high: rises=%0d ready_drop=%0d",
        ready_rises, saw_ready_drop))
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_clocks(4);
  endtask

  task automatic run_x072_soft_reset_in_prep_case();
    configure_and_start(2000);
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X072 RUN_PREPARE entry");
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X072 soft_reset during RUN_PREPARE");
    enter_run_prepare();
  endtask

  task automatic run_x073_soft_reset_in_terminate_case();
    same_key_burst_seq burst_seq;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x073_terminate_burst");
    burst_seq.num_hits = 16;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(16, 20_000, "X073 pre-terminate fill");
    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 2_000, "X073 TERMINATING entry");
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X073 soft_reset during TERMINATING");
  endtask

  task automatic run_x075_soft_reset_with_latency_write_case();
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    csr_write(CSR_EXPECTED_LAT_ADDR, 32'd1536);
    wait_clocks(2);
    csr_expect_mask(
      CSR_CTRL_ADDR, 32'h0000_0002, 32'h0000_0000,
      "X075 soft_reset self-clears with adjacent EXPECTED_LAT write");
    csr_expect_mask(
      CSR_EXPECTED_LAT_ADDR, 32'h0000_FFFF, 32'd1536,
      "X075 EXPECTED_LAT write survives adjacent soft_reset write");
  endtask

  task automatic run_x085_soft_reset_uid_read_case();
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    wait_clocks(1);
    csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "X085 UID readback under soft_reset activity");
    wait_clocks(1);
    csr_expect_mask(
      CSR_CTRL_ADDR, 32'h0000_0002, 32'h0000_0000,
      "X085 soft_reset self-clears around UID read");
  endtask

  task automatic run_x086_reserved_ctrl_bit2_case();
    csr_write(CSR_CTRL_ADDR, 32'h0000_0015);
    wait_clocks(2);
    csr_expect_mask(
      CSR_CTRL_ADDR, 32'h0000_001F, 32'h0000_0011,
      "X086 reserved CTRL bit2 is inert");
  endtask

  task automatic run_x087_reserved_ctrl_bit3_case();
    csr_write(CSR_CTRL_ADDR, 32'h0000_0019);
    wait_clocks(2);
    csr_expect_mask(
      CSR_CTRL_ADDR, 32'h0000_001F, 32'h0000_0011,
      "X087 reserved CTRL bit3 is inert");
  endtask

  task automatic run_x088_reserved_ctrl_upper_bits_case();
    csr_write(CSR_CTRL_ADDR, 32'hFFFF_FFFF);
    wait_clocks(2);
    csr_expect_mask(
      CSR_CTRL_ADDR, 32'hFFFF_FFFF, 32'h0000_0011,
      "X088 upper CTRL bits stay inert and soft_reset self-clears");
  endtask

  task automatic run_x071_soft_reset_running_case();
    same_key_burst_seq burst_seq;
    int unsigned       push_before;
    int unsigned       fill_before;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x071_running_burst");
    burst_seq.num_hits = 16;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(16, 20_000, "X071 RUNNING traffic");
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_before);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_before);
    if (push_before != 16 || fill_before == 0) begin
      `uvm_error("X071", $sformatf(
        "Precondition failed before RUNNING soft_reset: push=%0d fill=%0d",
        push_before, fill_before))
    end
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X071 soft_reset during RUNNING");
  endtask

  task automatic run_x074_soft_reset_during_drain_case();
    same_key_burst_seq burst_seq;

    configure_and_start(0);
    burst_seq = same_key_burst_seq::type_id::create("x074_drain_burst");
    burst_seq.num_hits = 32;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_pop_engine_state(3'd4, 40_000, "X074 active DRAIN");
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X074 soft_reset during DRAIN");
  endtask

  task automatic run_x090_meta_selector_case();
    logic [31:0] version_word;
    logic [31:0] date_word;

    set_meta_sel(2'b00);
    csr_read(CSR_META_ADDR, version_word);
    set_meta_sel(2'b01);
    csr_read(CSR_META_ADDR, date_word);
    if (version_word != expected_meta_version() || date_word != 32'd20260421) begin
      `uvm_error("X090", $sformatf(
        "META selector write/read mismatch: version=0x%08x date=0x%08x",
        version_word, date_word))
    end
    if (version_word == date_word) begin
      `uvm_error("X090", "META selector did not change the returned bank")
    end
  endtask

  task automatic run_x105_pop_counter_before_terminate_ready_case();
    same_key_burst_seq                burst_seq;
    ring_buffer_cam_pkg::out_seq_item last_hit;
    longint unsigned                  ready_cycle;
    int unsigned                      pop_count;
    int unsigned                      focus_search_key;
    int unsigned                      cycles;
    bit                               saw_mid_epoch_hit;

    configure_and_start(0);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq = same_key_burst_seq::type_id::create("x105_drain_burst");
    burst_seq.num_hits = 32;
    burst_seq.search_key = focus_search_key;
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_pop_engine_state(3'd4, 40_000, "X105 DRAIN entry");
    saw_mid_epoch_hit = 1'b0;
    cycles = 0;
    while (cycles < 60_000 && !saw_mid_epoch_hit) begin
      if (m_env.m_out_mon.recent_hits.size() > 0) begin
        last_hit = m_env.m_out_mon.recent_hits[$];
        if (last_hit.active_search_key == focus_search_key[7:0] &&
            last_hit.eop !== 1'b1) begin
          saw_mid_epoch_hit = 1'b1;
        end
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    if (!saw_mid_epoch_hit) begin
      `uvm_error("X105", "Did not reach an in-flight drain window before TERMINATING")
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_TERMINATING;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    ready_cycle = 0;
    for (cycles = 0; cycles < 120_000; cycles++) begin
      @(posedge m_env.m_ctrl_drv.vif.clk);
      if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
        ready_cycle = m_env.m_dbg_mon.sampled_cycles;
        break;
      end
    end
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_ctrl_drv.vif.clk);

    if (ready_cycle == 0) begin
      `uvm_error("X105", "Held TERMINATING command never re-acknowledged")
    end
    if (m_env.m_dbg_mon.last_pop_cycle == 0) begin
      `uvm_error("X105", "No backdoor pop counter update was observed")
    end
    if (ready_cycle != 0 &&
        m_env.m_dbg_mon.last_pop_cycle != 0 &&
        ready_cycle <= m_env.m_dbg_mon.last_pop_cycle) begin
      `uvm_error("X105", $sformatf(
        "asi_ctrl_ready asserted before the final POP_COUNT update was safely retired: ready_cycle=%0d last_pop_cycle=%0d",
        ready_cycle, m_env.m_dbg_mon.last_pop_cycle))
    end
    wait_for_scoreboard_idle(120_000, "X105 terminate drain completion");
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    if (pop_count != 32) begin
      `uvm_error("X105", $sformatf(
        "Final POP_COUNT mismatch after terminate drain: observed=%0d expected=32",
        pop_count))
    end
    expect_backdoor_counter_matches(CSR_POP_COUNT_ADDR, m_env.m_dbg_mon.dbg_pop_cnt,
      "X105 POP_COUNT backdoor/frontdoor agreement");
    expect_service_model_accounting("X105 pop counter before terminate ready", 1, 0);
  endtask

  task automatic run_x108_inerr_near_flush_case();
    single_error_hit_seq err_seq;
    int unsigned         inerr_count;
    bit                  saw_inerr_increment;
    bit                  saw_inerr_clear;

    configure_and_start(2000);
    err_seq = single_error_hit_seq::type_id::create("x108_bad_before_flush");
    err_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    err_seq.start(m_env.m_hit_seqr);

    saw_inerr_increment = 1'b0;
    for (int unsigned cycles = 0; cycles < 8; cycles++) begin
      if (m_env.m_dbg_mon.dbg_inerr_cnt[31:0] != 0) begin
        saw_inerr_increment = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (!saw_inerr_increment) begin
      `uvm_error("X108", "Backdoor INERR counter never incremented before FLUSH")
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X108 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    saw_inerr_clear = 1'b0;
    for (int unsigned cycles = 0; cycles < 120_000; cycles++) begin
      if (m_env.m_dbg_mon.dbg_inerr_cnt[31:0] == 0 &&
          m_env.m_dbg_mon.run_mgmt_flush_memory_start === 1'b1) begin
        saw_inerr_clear = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    if (!saw_inerr_clear) begin
      `uvm_error("X108", "Backdoor INERR counter never cleared during FLUSHING")
    end
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    if (inerr_count != 0) begin
      `uvm_error("X108", $sformatf(
        "Frontdoor INERR_COUNT did not settle to zero after FLUSH: observed=%0d",
        inerr_count))
    end
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X108 INERR_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x101_single_bad_hit_counter_observer_case();
    single_error_hit_seq err_seq;
    int unsigned         inerr_count;
    int unsigned         first_seen_cycle;

    configure_and_start(2000);
    err_seq = single_error_hit_seq::type_id::create("x101_single_bad");
    err_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    err_seq.start(m_env.m_hit_seqr);

    first_seen_cycle = 0;
    for (int unsigned cycles = 0; cycles < 16; cycles++) begin
      if (m_env.m_dbg_mon.dbg_inerr_cnt[31:0] == 1) begin
        first_seen_cycle = cycles;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (first_seen_cycle == 0 && m_env.m_dbg_mon.dbg_inerr_cnt[31:0] != 1) begin
      `uvm_error("X101", "Backdoor INERR counter never incremented after a single bad hit")
    end

    wait_clocks(4);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    if (inerr_count != 1) begin
      `uvm_error("X101", $sformatf(
        "Frontdoor INERR_COUNT mismatch after single bad hit: observed=%0d expected=1",
        inerr_count))
    end
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X101 INERR_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x102_bad_hit_burst_counter_observer_case();
    error_burst_seq err_burst;
    int unsigned    inerr_count;
    bit             saw_target_count;

    configure_and_start(2000);
    err_burst = error_burst_seq::type_id::create("x102_bad_burst");
    err_burst.num_hits = 16;
    err_burst.search_key = m_cfg.lane_key_ord_to_search_key(2);
    err_burst.start(m_env.m_hit_seqr);

    saw_target_count = 1'b0;
    for (int unsigned cycles = 0; cycles < 64; cycles++) begin
      if (m_env.m_dbg_mon.dbg_inerr_cnt[31:0] == 16) begin
        saw_target_count = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (!saw_target_count) begin
      `uvm_error("X102", $sformatf(
        "Backdoor INERR counter did not reach 16 after the bad-hit burst: observed=%0d",
        m_env.m_dbg_mon.dbg_inerr_cnt[31:0]))
    end

    wait_clocks(4);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    if (inerr_count != 16) begin
      `uvm_error("X102", $sformatf(
        "Frontdoor INERR_COUNT mismatch after 16 consecutive bad hits: observed=%0d expected=16",
        inerr_count))
    end
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X102 INERR_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x104_push_counter_before_terminate_ready_case();
    same_key_burst_seq burst_seq;
    int unsigned       ready_cycle;
    int unsigned       push_count;
    bit                saw_push_activity;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x104_terminate_burst");
    burst_seq.num_hits = 32;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    saw_push_activity = 1'b0;
    for (int unsigned cycles = 0; cycles < 20_000; cycles++) begin
      if (m_env.m_dbg_mon.dbg_push_cnt[31:0] != 0) begin
        saw_push_activity = 1'b1;
        break;
      end
      @(posedge m_env.m_ctrl_drv.vif.clk);
    end
    if (!saw_push_activity) begin
      `uvm_error("X104", "Backdoor PUSH counter never incremented before TERMINATING")
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_TERMINATING;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    ready_cycle = 0;
    for (int unsigned cycles = 0; cycles < 120_000; cycles++) begin
      @(posedge m_env.m_ctrl_drv.vif.clk);
      if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
        ready_cycle = m_env.m_dbg_mon.sampled_cycles;
        break;
      end
    end
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_ctrl_drv.vif.clk);

    if (ready_cycle == 0) begin
      `uvm_error("X104", "Held TERMINATING command never re-acknowledged")
    end
    if (m_env.m_dbg_mon.last_push_cycle == 0) begin
      `uvm_error("X104", "No backdoor push counter update was observed")
    end
    if (ready_cycle != 0 &&
        m_env.m_dbg_mon.last_push_cycle != 0 &&
        ready_cycle <= m_env.m_dbg_mon.last_push_cycle) begin
      `uvm_error("X104", $sformatf(
        "asi_ctrl_ready asserted before the final PUSH_COUNT update was safely retired: ready_cycle=%0d last_push_cycle=%0d",
        ready_cycle, m_env.m_dbg_mon.last_push_cycle))
    end
    wait_for_scoreboard_idle(120_000, "X104 terminate drain completion");
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    if (push_count != 32) begin
      `uvm_error("X104", $sformatf(
        "Final PUSH_COUNT mismatch after terminate drain: observed=%0d expected=32",
        push_count))
    end
    expect_backdoor_counter_matches(CSR_PUSH_COUNT_ADDR, m_env.m_dbg_mon.dbg_push_cnt,
      "X104 PUSH_COUNT backdoor/frontdoor agreement");
    expect_service_model_accounting("X104 push counter before terminate ready", 1, 0);
  endtask

  task automatic run_x111_push_near_flush_case();
    single_push_pop_seq single_seq;
    int unsigned        push_count_before;
    int unsigned        push_count_after;
    bit                 saw_push_clear;

    configure_and_start(2000);
    single_seq = single_push_pop_seq::type_id::create("x111_push_before_flush");
    single_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    single_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(1, 10_000, "X111 pre-flush push");
    push_count_before = m_env.m_dbg_mon.dbg_push_cnt[31:0];

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X111 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    saw_push_clear = 1'b0;
    for (int unsigned cycles = 0; cycles < 120_000; cycles++) begin
      if (m_env.m_dbg_mon.dbg_push_cnt[31:0] == 0 &&
          m_env.m_dbg_mon.run_mgmt_flush_memory_start === 1'b1) begin
        saw_push_clear = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    if (!saw_push_clear) begin
      `uvm_error("X111", "Backdoor PUSH counter never cleared during FLUSHING")
    end
    push_count_after = m_env.m_dbg_mon.dbg_push_cnt[31:0];
    if (push_count_after != 0) begin
      `uvm_error("X111", $sformatf(
        "Backdoor PUSH counter did not clear across FLUSH: before=%0d after=%0d",
        push_count_before, push_count_after))
    end
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count_after);
    if (push_count_after != 0) begin
      `uvm_error("X111", $sformatf(
        "Frontdoor PUSH_COUNT did not clear across FLUSH: before=%0d after=%0d",
        push_count_before, push_count_after))
    end
    expect_backdoor_counter_matches(CSR_PUSH_COUNT_ADDR, m_env.m_dbg_mon.dbg_push_cnt,
      "X111 PUSH_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x109_overwrite_near_flush_case();
    same_key_burst_seq burst_seq;
    int unsigned       overwrite_count;
    bit                saw_overwrite_increment;
    bit                saw_overwrite_clear;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x109_overwrite_before_flush");
    burst_seq.num_hits = m_cfg.ring_buffer_n_entry + 1;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    saw_overwrite_increment = 1'b0;
    for (int unsigned cycles = 0; cycles < 80_000; cycles++) begin
      if (m_env.m_dbg_mon.dbg_overwrite_cnt[31:0] != 0) begin
        saw_overwrite_increment = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (!saw_overwrite_increment) begin
      `uvm_error("X109", "Backdoor OVERWRITE counter never incremented before FLUSH")
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X109 RUN_PREPARE entry");
    m_env.m_scb.note_flush_reset();
    saw_overwrite_clear = 1'b0;
    for (int unsigned cycles = 0; cycles < 120_000; cycles++) begin
      if (m_env.m_dbg_mon.dbg_overwrite_cnt[31:0] == 0 &&
          m_env.m_dbg_mon.run_mgmt_flush_memory_start === 1'b1) begin
        saw_overwrite_clear = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    if (!saw_overwrite_clear) begin
      `uvm_error("X109", "Backdoor OVERWRITE counter never cleared during FLUSHING")
    end
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    if (overwrite_count != 0) begin
      `uvm_error("X109", $sformatf(
        "Frontdoor OVERWRITE_COUNT did not settle to zero after FLUSH: observed=%0d",
        overwrite_count))
    end
    expect_backdoor_counter_matches(CSR_OVERWRITE_ADDR, m_env.m_dbg_mon.dbg_overwrite_cnt,
      "X109 OVERWRITE_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x112_pop_near_flush_case();
    same_key_burst_seq burst_seq;
    int unsigned       pop_count_before;
    int unsigned       pop_count_after;
    bit                saw_pop_activity;
    bit                saw_pop_clear;

    configure_and_start(16);
    burst_seq = same_key_burst_seq::type_id::create("x112_pop_before_flush");
    burst_seq.num_hits = 32;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    saw_pop_activity = 1'b0;
    for (int unsigned cycles = 0; cycles < 400_000; cycles++) begin
      if (m_env.m_dbg_mon.dbg_pop_cnt[31:0] != 0) begin
        saw_pop_activity = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    if (!saw_pop_activity) begin
      `uvm_error("X112", "Backdoor POP counter never incremented before FLUSH")
    end
    pop_count_before = m_env.m_dbg_mon.dbg_pop_cnt[31:0];

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X112 RUN_PREPARE entry");
    saw_pop_clear = 1'b0;
    for (int unsigned cycles = 0; cycles < 120_000; cycles++) begin
      if (m_env.m_dbg_mon.dbg_pop_cnt[31:0] == 0 &&
          m_env.m_dbg_mon.run_mgmt_flush_memory_start === 1'b1) begin
        saw_pop_clear = 1'b1;
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    if (!saw_pop_clear) begin
      `uvm_error("X112", "Backdoor POP counter never cleared during FLUSHING")
    end
    pop_count_after = m_env.m_dbg_mon.dbg_pop_cnt[31:0];
    if (pop_count_after != 0) begin
      `uvm_error("X112", $sformatf(
        "Backdoor POP counter did not clear across FLUSH: before=%0d after=%0d",
        pop_count_before, pop_count_after))
    end
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count_after);
    if (pop_count_after != 0) begin
      `uvm_error("X112", $sformatf(
        "Frontdoor POP_COUNT did not clear across FLUSH: before=%0d after=%0d",
        pop_count_before, pop_count_after))
    end
    expect_backdoor_counter_matches(CSR_POP_COUNT_ADDR, m_env.m_dbg_mon.dbg_pop_cnt,
      "X112 POP_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x106_overwrite_counter_before_terminate_ready_case();
    same_key_burst_seq burst_seq;
    int unsigned       ready_cycle;
    int unsigned       overwrite_count;
    bit                saw_overwrite_activity;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x106_terminate_overwrite_burst");
    burst_seq.num_hits = m_cfg.ring_buffer_n_entry + 32;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);

    saw_overwrite_activity = 1'b0;
    for (int unsigned cycles = 0; cycles < 80_000; cycles++) begin
      if (m_env.m_dbg_mon.dbg_overwrite_cnt[31:0] != 0) begin
        saw_overwrite_activity = 1'b1;
        break;
      end
      @(posedge m_env.m_ctrl_drv.vif.clk);
    end
    if (!saw_overwrite_activity) begin
      `uvm_error("X106", "Backdoor OVERWRITE counter never incremented before TERMINATING")
    end

    ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
    send_endofrun_marker();
    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_TERMINATING;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    ready_cycle = 0;
    for (int unsigned cycles = 0; cycles < 120_000; cycles++) begin
      @(posedge m_env.m_ctrl_drv.vif.clk);
      if (m_env.m_ctrl_drv.vif.ready === 1'b1) begin
        ready_cycle = m_env.m_dbg_mon.sampled_cycles;
        break;
      end
    end
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_ctrl_drv.vif.clk);

    if (ready_cycle == 0) begin
      `uvm_error("X106", "Held TERMINATING command never re-acknowledged")
    end
    if (m_env.m_dbg_mon.last_overwrite_cycle == 0) begin
      `uvm_error("X106", "No backdoor overwrite counter update was observed")
    end
    if (ready_cycle != 0 &&
        m_env.m_dbg_mon.last_overwrite_cycle != 0 &&
        ready_cycle <= m_env.m_dbg_mon.last_overwrite_cycle) begin
      `uvm_error("X106", $sformatf(
        "asi_ctrl_ready asserted before the final OVERWRITE_COUNT update was safely retired: ready_cycle=%0d last_overwrite_cycle=%0d",
        ready_cycle, m_env.m_dbg_mon.last_overwrite_cycle))
    end
    wait_for_scoreboard_idle(120_000, "X106 terminate drain completion");
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    if (overwrite_count == 0) begin
      `uvm_error("X106", "Final OVERWRITE_COUNT stayed zero after an overwrite-bearing terminate run")
    end
    expect_backdoor_counter_matches(CSR_OVERWRITE_ADDR, m_env.m_dbg_mon.dbg_overwrite_cnt,
      "X106 OVERWRITE_COUNT backdoor/frontdoor agreement");
    expect_service_model_accounting("X106 overwrite counter before terminate ready", 1, 1);
  endtask

  task automatic run_x113_inerr_coincident_soft_reset_case();
    single_error_hit_seq err_seq;

    configure_and_start(2000);
    fork
      begin
        err_seq = single_error_hit_seq::type_id::create("x113_bad_hit");
        err_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
        err_seq.start(m_env.m_hit_seqr);
      end
      begin
        csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
      end
    join

    expect_soft_reset_abort("X113 coincident bad-hit soft_reset");
    if (m_env.m_dbg_mon.dbg_inerr_cnt[31:0] != 0) begin
      `uvm_error("X113", $sformatf(
        "INERR_COUNT backdoor was not cleared by coincident soft-reset: backdoor=%0d",
        m_env.m_dbg_mon.dbg_inerr_cnt[31:0]))
    end
  endtask

  task automatic run_x114_push_count_read_race_case();
    single_push_pop_seq single_seq;
    logic [31:0]        sampled_read;
    int unsigned        push_after;

    configure_and_start(2000);
    fork
      begin
        single_seq = single_push_pop_seq::type_id::create("x114_good_hit");
        single_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
        single_seq.start(m_env.m_hit_seqr);
      end
      begin
        csr_read(CSR_PUSH_COUNT_ADDR, sampled_read);
      end
    join

    wait_for_push_count(1, 10_000, "X114 post-race push");
    if (!(sampled_read == 32'd0 || sampled_read == 32'd1)) begin
      `uvm_error("X114", $sformatf(
        "PUSH_COUNT read returned a non-atomic value during the push race: observed=%0d",
        sampled_read))
    end
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_after);
    if (push_after != 1) begin
      `uvm_error("X114", $sformatf(
        "Final PUSH_COUNT mismatch after the read race: observed=%0d expected=1",
        push_after))
    end
    expect_backdoor_counter_matches(CSR_PUSH_COUNT_ADDR, m_env.m_dbg_mon.dbg_push_cnt,
      "X114 PUSH_COUNT backdoor/frontdoor agreement");
    terminate_and_drain(120_000, "X114 terminate drain");
    expect_service_model_accounting("X114 terminate drain", 1, 0);
  endtask

  task automatic run_x022_inerr_with_expected_latency_read_case();
    single_error_hit_seq err_seq;
    logic [31:0]         latency_read;
    int unsigned         inerr_count;
    int unsigned         push_count;

    configure_and_start(2000);
    fork
      begin
        err_seq = single_error_hit_seq::type_id::create("x022_bad_hit");
        err_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
        err_seq.start(m_env.m_hit_seqr);
      end
      begin
        csr_read(CSR_EXPECTED_LAT_ADDR, latency_read);
      end
    join

    wait_clocks(4);
    if (latency_read[15:0] != 16'd2000) begin
      `uvm_error("X022", $sformatf(
        "EXPECTED_LATENCY read was corrupted during the inerr race: observed=0x%08x expected=0x%08x",
        latency_read, 32'd2000))
    end
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    if (inerr_count != 1 || push_count != 0) begin
      `uvm_error("X022", $sformatf(
        "Bad-hit read race accounting mismatch: inerr=%0d push=%0d expected_inerr=1 expected_push=0",
        inerr_count, push_count))
    end
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X022 INERR_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x023_inerr_with_filter_disable_write_case();
    single_error_hit_seq err_seq;
    int unsigned         inerr_count;
    int unsigned         push_count;

    configure_and_start(2000);
    fork
      begin
        err_seq = single_error_hit_seq::type_id::create("x023_bad_hit");
        err_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
        err_seq.start(m_env.m_hit_seqr);
      end
      begin
        csr_write(CSR_CTRL_ADDR, 32'h0000_0000);
      end
    join

    wait_clocks(4);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    if (inerr_count != 1 || push_count != 0) begin
      `uvm_error("X023", $sformatf(
        "Boundary write 1->0 filter disable corrupted the current bad hit: inerr=%0d push=%0d expected_inerr=1 expected_push=0",
        inerr_count, push_count))
    end
    csr_expect_mask(CSR_CTRL_ADDR, 32'h0000_0013, 32'h0000_0000,
      "X023 CTRL write settles to go=0/filter_inerr=0");
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X023 INERR_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x024_inerr_with_inerr_write_case();
    single_error_hit_seq err_seq;
    int unsigned         inerr_count;
    int unsigned         push_count;

    configure_and_start(2000);
    fork
      begin
        err_seq = single_error_hit_seq::type_id::create("x024_bad_hit");
        err_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
        err_seq.start(m_env.m_hit_seqr);
      end
      begin
        csr_write(CSR_INERR_COUNT_ADDR, 32'hDEAD_BEEF);
      end
    join

    wait_clocks(4);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    if (inerr_count != 1 || push_count != 0) begin
      `uvm_error("X024", $sformatf(
        "Concurrent INERR_COUNT write disturbed bad-hit accounting: inerr=%0d push=%0d expected_inerr=1 expected_push=0",
        inerr_count, push_count))
    end
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X024 INERR_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x025_bad_hit_window_case();
    single_error_hit_seq err_seq;
    int unsigned         inerr_count;
    int unsigned         push_count;
    int unsigned         fill_level;

    configure_and_start(2000);
    for (int unsigned idx = 0; idx < 8; idx++) begin
      err_seq = single_error_hit_seq::type_id::create($sformatf("x025_bad_%0d", idx));
      err_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      err_seq.ts_low = idx[3:0];
      err_seq.start(m_env.m_hit_seqr);
      if (idx == 3) begin
        wait_clocks(2);
      end
    end

    for (int unsigned cycles = 0; cycles < 32; cycles++) begin
      if (m_env.m_dbg_mon.dbg_inerr_cnt[31:0] == 8) begin
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (inerr_count != 8 || push_count != 0 || fill_level != 0) begin
      `uvm_error("X025", $sformatf(
        "8-in-10 bad-hit window accounting mismatch: inerr=%0d push=%0d fill=%0d expected=8/0/0",
        inerr_count, push_count, fill_level))
    end
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X025 INERR_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x030_sustained_bad_hit_rate_case();
    single_error_hit_seq err_seq;
    int unsigned         inerr_count;
    int unsigned         push_count;
    int unsigned         fill_level;

    configure_and_start(2000);
    for (int unsigned idx = 0; idx < 64; idx++) begin
      err_seq = single_error_hit_seq::type_id::create($sformatf("x030_bad_%0d", idx));
      err_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      err_seq.ts_low = idx[3:0];
      err_seq.start(m_env.m_hit_seqr);
      if (idx != 63) begin
        wait_clocks(3);
      end
    end

    for (int unsigned cycles = 0; cycles < 320; cycles++) begin
      if (m_env.m_dbg_mon.dbg_inerr_cnt[31:0] == 64) begin
        break;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
    end
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (inerr_count != 64 || push_count != 0 || fill_level != 0) begin
      `uvm_error("X030", $sformatf(
        "1-in-4 bad-hit soak accounting mismatch: inerr=%0d push=%0d fill=%0d expected=64/0/0",
        inerr_count, push_count, fill_level))
    end
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X030 INERR_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x018_bad_hit_idle_to_prep_boundary_case();
    ring_buffer_cam_pkg::hit_seq_item hit_item;
    int unsigned         inerr_count;
    int unsigned         push_count;

    hit_item = ring_buffer_cam_pkg::hit_seq_item::type_id::create("x018_bad_hit");
    hit_item.asic = 4'h1;
    hit_item.ingress_channel = m_cfg.interleaving_index[3:0];
    hit_item.channel = 5'd3;
    hit_item.tcc8n = m_cfg.make_tcc8n_for_lane_key(2, 4'h1, 1'b0);
    hit_item.tcc1n6 = 3'd0;
    hit_item.tfine = 5'd4;
    hit_item.et1n6 = 9'd7;
    hit_item.has_error = 1'b1;
    hit_item.is_empty_marker = 1'b0;

    m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_RUN_PREPARE;
    m_env.m_ctrl_drv.vif.valid <= 1'b1;
    force_raw_hit(hit_item, 1);
    m_cfg.note_ctrl_cmd(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    m_env.m_ctrl_drv.vif.valid <= 1'b0;
    m_env.m_ctrl_drv.vif.data <= '0;
    @(posedge m_env.m_csr_drv.vif.clk);

    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUN_PREPARE, 2_000, "X018 RUN_PREPARE entry");
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_clocks(4);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    if (inerr_count != 0 || push_count != 0) begin
      `uvm_error("X018", $sformatf(
        "IDLE->RUN_PREPARE boundary bad hit mismatch: inerr=%0d push=%0d expected_inerr=0 expected_push=0",
        inerr_count, push_count))
    end
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X018 INERR_COUNT clear-on-RUN_PREPARE agreement");
  endtask

  task automatic run_x019_bad_hit_running_entry_boundary_case();
    ring_buffer_cam_pkg::hit_seq_item err_item;
    int unsigned         inerr_count;
    int unsigned         push_count;
    int unsigned         ingress_valid_pulses_before;
    int unsigned         ingress_err_pulses_before;
    int unsigned         ingress_err_accepts_before;
    int unsigned         boundary_valid_pulses;
    int unsigned         boundary_err_pulses;
    int unsigned         boundary_err_accepts;

    enter_run_prepare();
    csr_write(CSR_EXPECTED_LAT_ADDR, 32'd2000);
    ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
    wait_clocks(2);

    err_item = ring_buffer_cam_pkg::hit_seq_item::type_id::create("x019_bad_hit_item");
    err_item.asic = '0;
    err_item.ingress_channel = m_cfg.interleaving_index[3:0];
    err_item.channel = 5'd3;
    err_item.tcc8n = m_cfg.make_tcc8n_for_lane_key(2, 4'h2, 1'b0);
    err_item.tcc1n6 = 3'd1;
    err_item.tfine = 5'd6;
    err_item.et1n6 = 9'd9;
    err_item.has_error = 1'b1;
    err_item.is_empty_marker = 1'b0;

    ingress_valid_pulses_before = m_env.m_dbg_mon.ingress_valid_pulse_count;
    ingress_err_pulses_before = m_env.m_dbg_mon.ingress_error_pulse_count;
    ingress_err_accepts_before = m_env.m_dbg_mon.ingress_error_accept_count;
    fork
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUNNING);
      m_env.m_hit_drv.pulse_manual_hit(err_item, 1);
    join
    wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_RUNNING, 2_000, "X019 RUNNING entry");
    wait_clocks(4);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    boundary_valid_pulses = m_env.m_dbg_mon.ingress_valid_pulse_count - ingress_valid_pulses_before;
    boundary_err_pulses = m_env.m_dbg_mon.ingress_error_pulse_count - ingress_err_pulses_before;
    boundary_err_accepts = m_env.m_dbg_mon.ingress_error_accept_count - ingress_err_accepts_before;
    if (inerr_count != 1 || push_count != 0 ||
        boundary_valid_pulses != 1 || boundary_err_pulses != 1 ||
        boundary_err_accepts != 1) begin
      `uvm_error("X019", $sformatf(
        "RUNNING-entry boundary bad hit mismatch: inerr=%0d push=%0d expected_inerr=1 expected_push=0 boundary_valid=%0d boundary_pulse=%0d boundary_accept=%0d run_state=%0d ready=%0d",
        inerr_count, push_count, boundary_valid_pulses, boundary_err_pulses, boundary_err_accepts,
        m_env.m_dbg_mon.run_state_code, m_env.m_dbg_mon.ingress_ready))
    end
    expect_backdoor_counter_matches(CSR_INERR_COUNT_ADDR, m_env.m_dbg_mon.dbg_inerr_cnt,
      "X019 INERR_COUNT backdoor/frontdoor agreement");
  endtask

  task automatic run_x076_soft_reset_retains_inerr_case();
    error_burst_seq err_burst;
    int unsigned    inerr_before;

    configure_and_start(2000);
    err_burst = error_burst_seq::type_id::create("x076_inerr_burst");
    err_burst.num_hits = 4;
    err_burst.search_key = m_cfg.lane_key_ord_to_search_key(2);
    err_burst.start(m_env.m_hit_seqr);
    wait_clocks(32);
    read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_before);
    if (inerr_before != 4) begin
      `uvm_error("X076", $sformatf(
        "Precondition failed: expected INERR_COUNT=4 before soft_reset, observed %0d",
        inerr_before))
    end
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X076 soft_reset clears INERR_COUNT");
  endtask

  task automatic run_x077_soft_reset_retains_push_case();
    same_key_burst_seq burst_seq;
    int unsigned       push_before;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x077_push_burst");
    burst_seq.num_hits = 8;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(8, 20_000, "X077 RUNNING traffic");
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_before);
    if (push_before != 8) begin
      `uvm_error("X077", $sformatf(
        "Precondition failed: expected PUSH_COUNT=8 before soft_reset, observed %0d",
        push_before))
    end
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X077 soft_reset clears PUSH_COUNT");
  endtask

  task automatic run_x078_soft_reset_retains_pop_case();
    same_key_burst_seq burst_seq;
    int unsigned       pop_before;

    configure_and_start(0);
    burst_seq = same_key_burst_seq::type_id::create("x078_pop_burst");
    burst_seq.num_hits = 8;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_scoreboard_idle(80_000, "X078 pre-soft-reset drain");
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_before);
    if (pop_before != 8) begin
      `uvm_error("X078", $sformatf(
        "Precondition failed: expected POP_COUNT=8 before soft_reset, observed %0d",
        pop_before))
    end
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X078 soft_reset clears POP_COUNT");
  endtask

  task automatic run_x079_soft_reset_retains_overwrite_case();
    same_key_burst_seq burst_seq;
    int unsigned       overwrite_before;
    int unsigned       cycles;

    configure_and_start(2000);
    burst_seq = same_key_burst_seq::type_id::create("x079_overwrite_burst");
    burst_seq.num_hits = m_cfg.ring_buffer_n_entry + 32;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(m_cfg.ring_buffer_n_entry + 1, 40_000, "X079 overwrite start");
    cycles = 0;
    while (cycles < 40_000 && m_env.m_dbg_mon.dbg_overwrite_cnt[31:0] == 0) begin
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_before);
    if (overwrite_before == 0) begin
      `uvm_error("X079", "Precondition failed: overwrite counter never became non-zero before soft_reset")
    end
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X079 soft_reset clears OVERWRITE_COUNT");
  endtask

  task automatic run_x081_soft_reset_retains_fill_case();
    same_key_burst_seq burst_seq;
    int unsigned       fill_before;

    configure_and_start(16'hffff);
    set_sink_ready(1'b0);
    burst_seq = same_key_burst_seq::type_id::create("x081_fill_burst");
    burst_seq.num_hits = 12;
    burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
    burst_seq.start(m_env.m_hit_seqr);
    wait_for_push_count(12, 20_000, "X081 fill precondition");
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_before);
    if (fill_before < 11 || fill_before > 12) begin
      `uvm_error("X081", $sformatf(
        "Precondition failed: expected FILL_LEVEL to remain near the 12-hit preload before soft_reset, observed %0d",
        fill_before))
    end
    csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
    expect_soft_reset_abort("X081 soft_reset clears FILL_LEVEL");
    set_sink_ready(1'b1);
  endtask

  task automatic run_cross_curated_all_bucket_mix();
    same_key_burst_seq   burst_seq;
    random_push_pop_seq  rand_same;
    overwrite_profile_seq pressure_seq;
    error_burst_seq      err_burst;
    int unsigned         inerr_count;
    int unsigned         push_count;
    int unsigned         pop_count;
    int unsigned         accepted_checkpoint;
    int unsigned         accepted_delta;

    `uvm_info("CASE", "CASE_BEGIN run=CROSS-015", UVM_LOW)
    configure_and_start(2000);
    accepted_checkpoint = m_env.m_hit_drv.accepted_payload_total;

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
      accepted_checkpoint = m_env.m_hit_drv.accepted_payload_total;

      pressure_seq = overwrite_profile_seq::type_id::create($sformatf("cross015_pressure_%0d", round));
      pressure_seq.num_hits = 2048;
      pressure_seq.lane_key_start_ord = 2 + round;
      pressure_seq.pool_keys = 1 + (round % 2);
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.progress_stride = 256;
      pressure_seq.progress_tag = $sformatf("CROSS-015 round %0d", round);
      pressure_seq.start(m_env.m_hit_seqr);
      accepted_delta = m_env.m_hit_drv.accepted_payload_total - accepted_checkpoint;
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      `uvm_info("CROSS-015", $sformatf(
        "round=%0d post-pressure epoch: driver_accepted_delta=%0d push=%0d pop=%0d overwrite=%0d remaining=%0d pending_drain=%0d",
        round, accepted_delta, push_count, pop_count,
        m_env.m_dbg_mon.dbg_overwrite_cnt[31:0],
        m_env.m_scb.remaining_entries(), m_env.m_scb.pending_drain_entries()), UVM_LOW)
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
    int unsigned deassembly_wait_cycles;
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
    deassembly_wait_cycles = 0;
    while (m_env.m_dbg_mon.vif.deassembly_fifo_empty !== 1'b1 &&
           deassembly_wait_cycles < max_cycles) begin
      @(posedge m_env.m_csr_drv.vif.clk);
      deassembly_wait_cycles++;
    end
    if (m_env.m_dbg_mon.vif.deassembly_fifo_empty !== 1'b1) begin
      `uvm_error("TERM", $sformatf(
        "%s: deassembly fifo did not drain before TERMINATING: deassm_empty=%0d deassm_usedw=%0d after %0d cycles",
        what, m_env.m_dbg_mon.vif.deassembly_fifo_empty,
        m_env.m_dbg_mon.vif.deassembly_fifo_usedw, deassembly_wait_cycles))
    end else begin
      `uvm_info("TERM", $sformatf(
        "%s: deassembly fifo drained before TERMINATING after %0d cycles (usedw=%0d)",
        what, deassembly_wait_cycles, m_env.m_dbg_mon.vif.deassembly_fifo_usedw), UVM_LOW)
    end
    wait_clocks(4);
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

  task automatic run_single_key_latency_extreme_case(
    string       what,
    int unsigned latency,
    int unsigned num_hits,
    int unsigned min_pre_service_pushes,
    int unsigned max_pre_service_pushes,
    int unsigned min_overwrites,
    int unsigned max_overwrites,
    bit          require_full_hotspot,
    int unsigned min_accepted_before_first_pop = 0,
    int unsigned max_accepted_before_first_pop = 0
  );
    overwrite_profile_seq pressure_seq;
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned fill_level;
    int unsigned pre_service_pushes;
    int unsigned accepted_before_first_pop;
    int unsigned focus_search_key;

    configure_and_start(latency);

    pressure_seq = overwrite_profile_seq::type_id::create({what, "_pressure"});
    pressure_seq.num_hits = num_hits;
    pressure_seq.lane_key_start_ord = 2;
    pressure_seq.pool_keys = 1;
    pressure_seq.hits_per_key_switch = 1;
    pressure_seq.progress_stride = 128;
    pressure_seq.progress_tag = what;
    pressure_seq.start(m_env.m_hit_seqr);

    terminate_and_drain((latency >= 16'h8000) ? 1_000_000 : 200_000, {what, " terminate drain"});
    expect_service_model_accounting({what, " post-terminate"}, 1, 1);

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    pre_service_pushes = m_env.m_dbg_mon.push_count_at_first_pop;
    accepted_before_first_pop = m_env.m_hit_drv.accepted_payload_at_first_pop;
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);

    if (push_count != num_hits) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s final push_count mismatch: observed=%0d expected=%0d",
        what, push_count, num_hits))
    end
    if (fill_level != 0) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s expected a drained fill_level=0 after terminate, observed %0d",
        what, fill_level))
    end
    if (m_env.m_dbg_mon.first_pop_cycle == 0) begin
      `uvm_error("PRESSURE", $sformatf("%s never observed a pop event", what))
    end
    if (pre_service_pushes < min_pre_service_pushes ||
        pre_service_pushes > max_pre_service_pushes) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s pushes_before_first_pop out of range: observed=%0d expected=[%0d,%0d]",
        what, pre_service_pushes, min_pre_service_pushes, max_pre_service_pushes))
    end
    if (min_accepted_before_first_pop == 0 && max_accepted_before_first_pop == 0) begin
      min_accepted_before_first_pop = min_pre_service_pushes;
      max_accepted_before_first_pop = max_pre_service_pushes;
    end
    if (accepted_before_first_pop < min_accepted_before_first_pop ||
        accepted_before_first_pop > max_accepted_before_first_pop) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s accepted_before_first_pop out of range: observed=%0d expected=[%0d,%0d]",
        what, accepted_before_first_pop,
        min_accepted_before_first_pop,
        max_accepted_before_first_pop))
    end
    if (overwrite_count < min_overwrites || overwrite_count > max_overwrites) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s overwrite_count out of range: observed=%0d expected=[%0d,%0d]",
        what, overwrite_count, min_overwrites, max_overwrites))
    end
    if (require_full_hotspot &&
        m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < m_cfg.ring_buffer_n_entry) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s hotspot never saturated the full sector depth: key=%0d max_for_key=%0d ring_depth=%0d",
        what, focus_search_key,
        m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
        m_cfg.ring_buffer_n_entry))
    end
    `uvm_info("PRESSURE", $sformatf(
      "%s latency summary: latency=%0d pushes_before_first_pop=%0d accepted_before_first_pop=%0d push=%0d pop=%0d overwrite=%0d max_hotspot=%0d",
      what, latency, pre_service_pushes, accepted_before_first_pop,
      push_count, pop_count, overwrite_count,
      m_env.m_scb.max_remaining_entries_for_key(focus_search_key)), UVM_LOW)
  endtask

  task automatic emit_weighted_overwrite_epochs(
    string       what,
    int unsigned heavy_search_key,
    int unsigned light_a_search_key,
    int unsigned light_b_search_key,
    int unsigned light_c_search_key,
    int unsigned epochs
  );
    same_key_burst_seq burst_seq;

    for (int epoch = 0; epoch < epochs; epoch++) begin
      burst_seq = same_key_burst_seq::type_id::create($sformatf("%s_heavy_%0d", what, epoch));
      burst_seq.num_hits = 7;
      burst_seq.search_key = heavy_search_key;
      burst_seq.start(m_env.m_hit_seqr);

      burst_seq = same_key_burst_seq::type_id::create($sformatf("%s_light_a_%0d", what, epoch));
      burst_seq.num_hits = 1;
      burst_seq.search_key = light_a_search_key;
      burst_seq.start(m_env.m_hit_seqr);

      burst_seq = same_key_burst_seq::type_id::create($sformatf("%s_light_b_%0d", what, epoch));
      burst_seq.num_hits = 1;
      burst_seq.search_key = light_b_search_key;
      burst_seq.start(m_env.m_hit_seqr);

      burst_seq = same_key_burst_seq::type_id::create($sformatf("%s_light_c_%0d", what, epoch));
      burst_seq.num_hits = 1;
      burst_seq.search_key = light_c_search_key;
      burst_seq.start(m_env.m_hit_seqr);
    end
  endtask

  task automatic run_single_key_slow_creep_case(
    string       what,
    int unsigned latency,
    int unsigned prefill_hits,
    int unsigned slow_hits,
    int unsigned slow_inter_hit_gap_cycles,
    int unsigned min_overwrites,
    int unsigned max_overwrite_pct_x100
  );
    overwrite_profile_seq pressure_seq;
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned fill_level;
    int unsigned total_hits;
    int unsigned focus_search_key;

    configure_and_start(latency);

    pressure_seq = overwrite_profile_seq::type_id::create({what, "_prefill"});
    pressure_seq.num_hits = prefill_hits;
    pressure_seq.lane_key_start_ord = 2;
    pressure_seq.pool_keys = 1;
    pressure_seq.hits_per_key_switch = 1;
    pressure_seq.progress_stride = 128;
    pressure_seq.progress_tag = {what, " prefill"};
    pressure_seq.start(m_env.m_hit_seqr);

    pressure_seq = overwrite_profile_seq::type_id::create({what, "_slow"});
    pressure_seq.num_hits = slow_hits;
    pressure_seq.lane_key_start_ord = 2;
    pressure_seq.pool_keys = 1;
    pressure_seq.hits_per_key_switch = 1;
    pressure_seq.inter_hit_gap_cycles = slow_inter_hit_gap_cycles;
    pressure_seq.progress_stride = 1024;
    pressure_seq.progress_tag = {what, " slow"};
    pressure_seq.start(m_env.m_hit_seqr);

    terminate_and_drain(1_000_000, {what, " terminate drain"});
    expect_service_model_accounting({what, " post-terminate"}, 1, min_overwrites);

    total_hits = prefill_hits + slow_hits;
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(2);

    if (push_count != total_hits) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s final push_count mismatch: observed=%0d expected=%0d",
        what, push_count, total_hits))
    end
    if (fill_level != 0) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s expected a drained fill_level=0 after terminate, observed %0d",
        what, fill_level))
    end
    if (m_env.m_dbg_mon.first_pop_cycle == 0 || m_env.m_dbg_mon.first_overwrite_cycle == 0) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s did not observe both first pop and first overwrite cycles: first_pop=%0d first_overwrite=%0d",
        what, m_env.m_dbg_mon.first_pop_cycle, m_env.m_dbg_mon.first_overwrite_cycle))
    end
    if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < m_cfg.ring_buffer_n_entry) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s hotspot never saturated the full sector depth: key=%0d max_for_key=%0d ring_depth=%0d",
        what, focus_search_key,
        m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
        m_cfg.ring_buffer_n_entry))
    end
    if ((overwrite_count * 10_000) > (push_count * max_overwrite_pct_x100)) begin
      `uvm_error("PRESSURE", $sformatf(
        "%s overwrite rate escaped the slow-creep envelope: push=%0d overwrite=%0d pct_x100=%0d limit_x100=%0d",
        what, push_count, overwrite_count,
        (push_count == 0) ? 0 : ((overwrite_count * 10_000) / push_count),
        max_overwrite_pct_x100))
    end
    `uvm_info("PRESSURE", $sformatf(
      "%s slow-creep summary: latency=%0d prefill_hits=%0d slow_hits=%0d slow_gap=%0d push=%0d pop=%0d overwrite=%0d overwrite_pct_x100=%0d max_hotspot=%0d",
      what, latency, prefill_hits, slow_hits, slow_inter_hit_gap_cycles,
      push_count, pop_count, overwrite_count,
      (push_count == 0) ? 0 : ((overwrite_count * 10_000) / push_count),
      m_env.m_scb.max_remaining_entries_for_key(focus_search_key)), UVM_LOW)
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
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;

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
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (push_count != num_hits ||
        pop_count != num_hits ||
        overwrite_count != 0 ||
        fill_level != 0) begin
      `uvm_error("SAME_KEY", $sformatf(
        "%s accounting mismatch after drain: push=%0d pop=%0d overwrite=%0d fill=%0d expected=%0d/%0d/0/0",
        what, push_count, pop_count, overwrite_count, fill_level, num_hits, num_hits))
    end
  endtask

  task automatic run_pipe_stage_latency_smoke_case(
    string       what,
    string       case_tag,
    int unsigned expected_pipe_stages,
    int unsigned key_ord = 2,
    int unsigned timeout_cycles = 120_000
  );
    same_key_burst_seq burst_seq;
    int unsigned       focus_search_key;
    int unsigned       search_cycles;
    int unsigned       observed_wait_cycles;
    int unsigned       expected_wait_cycles;
    int                active_partition_idx;
    logic [3:0]        load_mask;
    logic [3:0]        active_load_mask;
    longint unsigned   cycle_a;
    longint unsigned   cycle_b;
    bit                saw_load;
    bit                saw_flagged_result;

    if (m_cfg.encoder_pipe_stages != expected_pipe_stages) begin
      `uvm_fatal(case_tag, $sformatf(
        "%s requires ENCODER_PIPE_STAGES=%0d, observed=%0d",
        what, expected_pipe_stages, m_cfg.encoder_pipe_stages))
    end

    configure_and_start(16'd2000);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(key_ord);
    burst_seq = same_key_burst_seq::type_id::create({case_tag, "_pipe_stage_smoke"});
    burst_seq.num_hits = 1;
    burst_seq.search_key = focus_search_key;
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_push_count(1, timeout_cycles / 8, {what, " single-hit fill"});
    cycle_a = 0;
    cycle_b = 0;
    saw_load = 1'b0;
    saw_flagged_result = 1'b0;
    active_partition_idx = -1;
    expected_wait_cycles = (expected_pipe_stages <= 2) ? 2 : expected_pipe_stages;
    search_cycles = 0;
    while (search_cycles < timeout_cycles && cycle_b == 0) begin
      load_mask = m_env.m_dbg_mon.pop_partition_load;
      active_load_mask = load_mask & m_env.m_dbg_mon.pop_partition_pending;
      if (m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0] &&
          active_load_mask != 0) begin
        saw_load = 1'b1;
        if (cycle_a == 0) begin
          cycle_a = m_env.m_dbg_mon.sampled_cycles;
          for (int idx = 0; idx < m_cfg.n_partitions; idx++) begin
            if (active_load_mask[idx] === 1'b1) begin
              active_partition_idx = idx;
              break;
            end
          end
          if ($countones(active_load_mask) != 1 || active_partition_idx < 0) begin
            `uvm_error(case_tag, $sformatf(
              "%s expected exactly one pending partition at load, observed load=0x%0h pending=0x%0h",
              what, load_mask, m_env.m_dbg_mon.pop_partition_pending))
          end
        end
      end
      if (cycle_a != 0 &&
          active_partition_idx >= 0 &&
          m_env.m_dbg_mon.pop_partition_result_valid[active_partition_idx] === 1'b1 &&
          m_env.m_dbg_mon.pop_partition_flag[active_partition_idx] === 1'b1 &&
          m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0]) begin
        cycle_b = m_env.m_dbg_mon.sampled_cycles;
        saw_flagged_result = 1'b1;
        if ((m_env.m_dbg_mon.pop_partition_flag & ~(4'b0001 << active_partition_idx)) != 0) begin
          `uvm_error(case_tag, $sformatf(
            "%s flagged an inactive partition: active=%0d flags=0x%0h result_valid=0x%0h",
            what, active_partition_idx, m_env.m_dbg_mon.pop_partition_flag,
            m_env.m_dbg_mon.pop_partition_result_valid))
        end
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      search_cycles++;
    end

    if (!saw_load) begin
      `uvm_error(case_tag, $sformatf(
        "%s never observed the partition load pulse",
        what))
    end
    if (!saw_flagged_result) begin
      `uvm_error(case_tag, $sformatf(
        "%s never observed a flagged result_valid pulse on the active partition",
        what))
    end
    if (cycle_b == 0) begin
      `uvm_error(case_tag, $sformatf(
        "%s never observed a flagged result_valid pulse",
        what))
    end else begin
      observed_wait_cycles = int'(cycle_b - cycle_a);
      if (observed_wait_cycles != int'(expected_wait_cycles)) begin
        `uvm_error(case_tag, $sformatf(
          "%s result latency mismatch: observed=%0d expected=%0d anchor_cycle=%0d result_cycle=%0d",
          what, observed_wait_cycles, expected_wait_cycles,
          cycle_a, cycle_b))
      end
    end

    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
  endtask

  task automatic run_profile_integrity_case(
    string        what,
    int unsigned  latency,
    int unsigned  num_hits,
    int unsigned  lane_key_start_ord,
    int unsigned  pool_keys,
    int unsigned  hits_per_key_switch,
    int unsigned  inter_hit_gap_cycles,
    int unsigned  random_gap_min_cycles,
    int unsigned  random_gap_max_cycles,
    bit           randomize_key_order,
    bit           use_bernoulli_arrival,
    byte unsigned bernoulli_arrival_threshold,
    int unsigned  burst_len,
    int unsigned  burst_idle_cycles,
    int unsigned  sink_ready_mode,
    int unsigned  timeout_cycles,
    int unsigned  min_max_fill,
    int unsigned  max_max_fill,
    int unsigned  min_data_subheaders,
    bit           check_latency_readback = 1'b0,
    logic [31:0]  lfsr_seed = 32'h1ace_b00c,
    logic [3:0]   expected_partition_mask = 4'b0000,
    int unsigned  max_overwrites = 0
  );
    profile_traffic_seq prof_seq;
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned cache_miss_count;
    int unsigned fill_level;
    int unsigned subhdr_before;
    int unsigned subhdr_delta;
    int unsigned sink_low_cycles;
    int unsigned sink_cycles;
    logic [31:0] latency_readback;
    logic [31:0] ready_lfsr;
    bit          traffic_done;
    bit          ready_now;
    logic [3:0]  observed_partition_mask;
    int unsigned observed_partition_idx;
    int unsigned search_cycles;

    subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
    sink_low_cycles = 0;
    traffic_done = 1'b0;
    ready_lfsr = 32'h1bad_cafe;
    observed_partition_mask = '0;

    configure_and_start(latency);
    if (check_latency_readback) begin
      csr_read(CSR_EXPECTED_LAT_ADDR, latency_readback);
      if (latency_readback != latency[31:0]) begin
        `uvm_error("PROF", $sformatf(
          "%s latency readback mismatch: observed=%0d expected=%0d",
          what, latency_readback, latency))
      end
    end

    fork
      begin : sink_ready_thread
        if (sink_ready_mode != 0) begin
          sink_cycles = 0;
          while (sink_cycles < timeout_cycles &&
                 (!traffic_done ||
                  m_env.m_hit_drv.pending_source_items() != 0 ||
                  m_env.m_scb.remaining_entries() != 0)) begin
            case (sink_ready_mode)
              1: ready_now = ((sink_cycles & 2'b11) != 2'b11); // 75% ready
              2: ready_now = ((sink_cycles & 1'b1) == 1'b0);   // 50% ready
              3: begin
                ready_lfsr = {ready_lfsr[30:0],
                              ready_lfsr[31] ^ ready_lfsr[21] ^ ready_lfsr[1] ^ ready_lfsr[0]};
                ready_now = (ready_lfsr[7:0] < 8'd179);        // ~70% ready
              end
              4: begin
                ready_lfsr = {ready_lfsr[30:0],
                              ready_lfsr[31] ^ ready_lfsr[21] ^ ready_lfsr[1] ^ ready_lfsr[0]};
                ready_now = (ready_lfsr[7:0] < 8'd77);         // ~30% ready
              end
              5: begin
                case ((sink_cycles / 128) % 4)
                  0: ready_now = 1'b1;
                  1: ready_now = ((sink_cycles & 1'b1) == 1'b0);   // 50% ready
                  2: ready_now = ((sink_cycles & 2'b11) != 2'b11); // 75% ready
                  default: ready_now = 1'b0;                       // hard stall window
                endcase
              end
              6: ready_now = (sink_cycles >= 500);
              default: ready_now = 1'b1;
            endcase
            set_sink_ready(ready_now);
            if (!ready_now) begin
              sink_low_cycles++;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            sink_cycles++;
          end
          set_sink_ready(1'b1);
        end
      end
      begin : traffic_thread
        prof_seq = profile_traffic_seq::type_id::create({what, "_profile"});
        prof_seq.num_hits = num_hits;
        prof_seq.lane_key_start_ord = lane_key_start_ord;
        prof_seq.pool_keys = pool_keys;
        prof_seq.hits_per_key_switch = hits_per_key_switch;
        prof_seq.randomize_key_order = randomize_key_order;
        prof_seq.inter_hit_gap_cycles = inter_hit_gap_cycles;
        prof_seq.random_gap_min_cycles = random_gap_min_cycles;
        prof_seq.random_gap_max_cycles = random_gap_max_cycles;
        prof_seq.use_bernoulli_arrival = use_bernoulli_arrival;
        prof_seq.bernoulli_arrival_threshold = bernoulli_arrival_threshold;
        prof_seq.lfsr_seed = lfsr_seed;
        prof_seq.burst_len = burst_len;
        prof_seq.burst_idle_cycles = burst_idle_cycles;
        prof_seq.progress_stride = (num_hits >= 4096) ? (num_hits / 4) : (num_hits / 2);
        prof_seq.progress_tag = what;
        prof_seq.start(m_env.m_hit_seqr);
        traffic_done = 1'b1;
      end
      begin : partition_watch_thread
        search_cycles = 0;
        while (search_cycles < timeout_cycles &&
               (!traffic_done ||
                m_env.m_hit_drv.pending_source_items() != 0 ||
                m_env.m_scb.remaining_entries() != 0)) begin
          if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
            observed_partition_idx = m_env.m_dbg_mon.pop_issue_partition_idx;
            if (observed_partition_idx < 4) begin
              observed_partition_mask[observed_partition_idx[1:0]] = 1'b1;
            end
          end
          @(posedge m_env.m_csr_drv.vif.clk);
          search_cycles++;
        end
      end
    join

    set_sink_ready(1'b1);
    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    subhdr_delta = m_env.m_out_mon.total_data_subheaders_seen - subhdr_before;

    if (push_count != num_hits ||
        (pop_count + overwrite_count) != num_hits ||
        overwrite_count > max_overwrites ||
        cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected_push=%0d expected_pop_plus_overwrite=%0d max_overwrite=%0d",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        num_hits, num_hits, max_overwrites))
    end
    if (sink_ready_mode != 0 && sink_low_cycles == 0) begin
      `uvm_error("PROF", $sformatf(
        "%s never forced sink-ready low despite a backpressure-enabled case",
        what))
    end
    if (min_data_subheaders > 0 && subhdr_delta < min_data_subheaders) begin
      `uvm_error("PROF", $sformatf(
        "%s produced too few data-bearing subheaders: observed=%0d expected_at_least=%0d",
        what, subhdr_delta, min_data_subheaders))
    end
    if (min_max_fill > 0 && m_env.m_dbg_mon.max_live_fill < min_max_fill) begin
      `uvm_error("PROF", $sformatf(
        "%s fill envelope stayed too low: max_live_fill=%0d expected_at_least=%0d",
        what, m_env.m_dbg_mon.max_live_fill, min_max_fill))
    end
    if (max_max_fill > 0 && m_env.m_dbg_mon.max_live_fill > max_max_fill) begin
      `uvm_error("PROF", $sformatf(
        "%s fill envelope exceeded the allowed bound: max_live_fill=%0d expected_at_most=%0d",
        what, m_env.m_dbg_mon.max_live_fill, max_max_fill))
    end
    if (expected_partition_mask != 4'b0000 &&
        observed_partition_mask != expected_partition_mask) begin
      `uvm_error("PROF", $sformatf(
        "%s partition issue mask mismatch: observed=0x%0h expected=0x%0h",
        what, observed_partition_mask, expected_partition_mask))
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: latency=%0d num_hits=%0d pool_keys=%0d gap=%0d rand_gap=[%0d,%0d] bernoulli=%0d/%0d burst=%0d/%0d sink_mode=%0d push=%0d pop=%0d overwrite=%0d cache_miss=%0d max_live_fill=%0d max_remaining=%0d data_subheaders=%0d pushes_before_first_pop=%0d sink_low_cycles=%0d observed_partition_mask=0x%0h",
      what, latency, num_hits, pool_keys, inter_hit_gap_cycles,
      random_gap_min_cycles, random_gap_max_cycles,
      use_bernoulli_arrival, bernoulli_arrival_threshold,
      burst_len, burst_idle_cycles, sink_ready_mode,
      push_count, pop_count, overwrite_count, cache_miss_count,
      m_env.m_dbg_mon.max_live_fill, m_env.m_scb.max_remaining_entries(),
      subhdr_delta, m_env.m_dbg_mon.push_count_at_first_pop, sink_low_cycles,
      observed_partition_mask), UVM_LOW)
  endtask

  task automatic check_window_service_rate(
    string           what,
    int unsigned     push_before,
    int unsigned     pop_before,
    int unsigned     overwrite_before,
    longint unsigned traffic_start_cycle,
    int unsigned     max_cycles_per_accepted_hit,
    int unsigned     min_accepted_hits = 1
  );
    int unsigned     push_after;
    int unsigned     pop_after;
    int unsigned     overwrite_after;
    int unsigned     push_delta;
    int unsigned     pop_delta;
    int unsigned     overwrite_delta;
    longint unsigned effective_start_cycle;
    longint unsigned active_span_cycles;
    longint unsigned max_allowed_span_cycles;

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_after);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_after);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_after);

    push_delta = push_after - push_before;
    pop_delta = pop_after - pop_before;
    overwrite_delta = overwrite_after - overwrite_before;

    if (push_delta < min_accepted_hits) begin
      `uvm_error("PROF", $sformatf(
        "%s accepted too little traffic for a throughput audit: push_delta=%0d expected_at_least=%0d",
        what, push_delta, min_accepted_hits))
    end
    if (push_delta != (pop_delta + overwrite_delta)) begin
      `uvm_error("PROF", $sformatf(
        "%s post-window accounting mismatch: push_delta=%0d pop_delta=%0d overwrite_delta=%0d",
        what, push_delta, pop_delta, overwrite_delta))
    end
    if (push_delta == 0) begin
      return;
    end
    effective_start_cycle = traffic_start_cycle;
    if (effective_start_cycle == 0) begin
      effective_start_cycle = m_env.m_hit_drv.first_accept_cycle;
    end
    if (effective_start_cycle == 0) begin
      `uvm_error("PROF", $sformatf(
        "%s never observed an accepted hit before the throughput audit",
        what))
      return;
    end
    if (m_env.m_dbg_mon.last_pop_cycle < effective_start_cycle) begin
      `uvm_error("PROF", $sformatf(
        "%s never observed a post-start pop event: traffic_start=%0d last_pop=%0d",
        what, effective_start_cycle, m_env.m_dbg_mon.last_pop_cycle))
      return;
    end

    active_span_cycles = m_env.m_dbg_mon.last_pop_cycle - effective_start_cycle + 1;
    max_allowed_span_cycles = longint'(push_delta) * max_cycles_per_accepted_hit;
    if (active_span_cycles > max_allowed_span_cycles) begin
      `uvm_error("PROF", $sformatf(
        "%s service span exceeded the conservative throughput floor: span=%0d max_allowed=%0d push_delta=%0d cycles_per_hit_limit=%0d",
        what, active_span_cycles, max_allowed_span_cycles, push_delta, max_cycles_per_accepted_hit))
    end

    `uvm_info("PROF", $sformatf(
      "%s service-rate summary: push_delta=%0d pop_delta=%0d overwrite_delta=%0d span_cycles=%0d cycles_per_hit_x100=%0d limit=%0d",
      what, push_delta, pop_delta, overwrite_delta, active_span_cycles,
      (push_delta == 0) ? 0 : int'((active_span_cycles * 100) / push_delta),
      max_cycles_per_accepted_hit), UVM_LOW)
  endtask

  task automatic run_frequent_terminate_mixed_profile_case(
    string       what,
    int unsigned windows,
    int unsigned timeout_cycles
  );
    same_key_burst_seq burst_seq;
    single_push_pop_seq single_seq;
    int unsigned hot_key_ord;
    int unsigned single_key_ord;
    int unsigned latency;
    int unsigned rounds;
    int unsigned burst_hits;
    int unsigned singles_per_round;
    int unsigned push_before;
    int unsigned pop_before;
    int unsigned overwrite_before;
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned cache_miss_count;
    int unsigned fill_level;
    int unsigned total_push_delta;
    int unsigned total_pop_delta;
    int unsigned total_overwrite_delta;
    longint unsigned traffic_start_cycle;

    total_push_delta = 0;
    total_pop_delta = 0;
    total_overwrite_delta = 0;

    for (int window = 0; window < windows; window++) begin
      case (window % 3)
        0: latency = 16'd128;
        1: latency = 16'd2000;
        default: latency = 16'd1;
      endcase

      configure_and_start(latency);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_before);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_before);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_before);
      traffic_start_cycle = m_env.m_dbg_mon.sampled_cycles;

      hot_key_ord = 2 + ((window * 5) % 16);
      rounds = 4 + $urandom_range(0, 2);
      for (int round = 0; round < rounds; round++) begin
        burst_hits = 16 + $urandom_range(0, 24);
        burst_seq = same_key_burst_seq::type_id::create($sformatf("%s_hot_%0d_%0d", what, window, round));
        burst_seq.num_hits = burst_hits;
        burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(hot_key_ord);
        burst_seq.start(m_env.m_hit_seqr);

        singles_per_round = 1 + $urandom_range(0, 2);
        for (int single_idx = 0; single_idx < singles_per_round; single_idx++) begin
          single_key_ord = 18 + ((window + round + single_idx) % 12);
          if (single_key_ord == hot_key_ord) begin
            single_key_ord++;
          end
          single_seq = single_push_pop_seq::type_id::create($sformatf("%s_single_%0d_%0d_%0d", what, window, round, single_idx));
          single_seq.search_key = m_cfg.lane_key_ord_to_search_key(single_key_ord);
          single_seq.hit_asic = (window + round + single_idx) & 4'hf;
          single_seq.ingress_channel = (window + single_idx) & 4'hf;
          single_seq.hit_channel = 5'd3;
          single_seq.ts_low = (round + single_idx) & 4'hf;
          single_seq.hit_tcc1n6 = (window + round + single_idx) & 3'h7;
          single_seq.hit_tfine = ((window * 3) + round + single_idx) & 5'h1f;
          single_seq.hit_et1n6 = 9'(32 + (window * 16) + (round * 4) + single_idx);
          single_seq.start(m_env.m_hit_seqr);
        end

        wait_clocks($urandom_range(0, m_cfg.encoder_pipe_stages + 2));
      end

      terminate_and_drain(timeout_cycles, $sformatf("%s window_%0d terminate drain", what, window));
      expect_service_model_accounting($sformatf("%s window_%0d post-terminate", what, window), 1, 0);
      check_window_service_rate(
        $sformatf("%s window_%0d", what, window),
        push_before,
        pop_before,
        overwrite_before,
        traffic_start_cycle,
        96,
        80);

      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      total_push_delta += (push_count - push_before);
      total_pop_delta += (pop_count - pop_before);
      total_overwrite_delta += (overwrite_count - overwrite_before);
    end

    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (total_push_delta < (windows * 80)) begin
      `uvm_error("PROF", $sformatf(
        "%s total accepted traffic stayed below the planned floor: cumulative_push=%0d expected_at_least=%0d",
        what, total_push_delta, windows * 80))
    end
    if (total_push_delta != total_pop_delta + total_overwrite_delta) begin
      `uvm_error("PROF", $sformatf(
        "%s cumulative accounting mismatch: cumulative_push=%0d cumulative_pop=%0d cumulative_overwrite=%0d",
        what, total_push_delta, total_pop_delta, total_overwrite_delta))
    end
    if (push_count != pop_count || overwrite_count != 0 || cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s final window accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level))
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: windows=%0d cumulative_push=%0d cumulative_pop=%0d cumulative_overwrite=%0d final_cache_miss=%0d max_live_fill=%0d max_source_backlog=%0d",
      what, windows, total_push_delta, total_pop_delta, total_overwrite_delta, cache_miss_count,
      m_env.m_dbg_mon.max_live_fill, m_env.m_hit_drv.max_source_backlog), UVM_LOW)
  endtask

  task automatic run_config_aware_encoder_stress_case(
    string       what,
    int unsigned timeout_cycles
  );
    int unsigned    active_pool_keys;
    int unsigned    num_hits;
    int unsigned    inter_hit_gap_cycles;
    int unsigned    min_data_subheaders;
    int unsigned    min_max_fill;
    int unsigned    max_cycles_per_hit;
    logic [3:0]     expected_partition_mask;
    longint unsigned traffic_start_cycle;

    active_pool_keys = (m_cfg.n_partitions < 2) ? 4 : (m_cfg.n_partitions * 4);
    if (active_pool_keys > 16) begin
      active_pool_keys = 16;
    end
    num_hits = active_pool_keys * 128;
    if (num_hits < 512) begin
      num_hits = 512;
    end
    inter_hit_gap_cycles = 9 + m_cfg.encoder_pipe_stages;
    min_data_subheaders = (active_pool_keys < 4) ? active_pool_keys : 4;
    min_max_fill = m_cfg.ring_buffer_n_entry / 4;
    max_cycles_per_hit = 32 + (m_cfg.encoder_pipe_stages * 8);
    expected_partition_mask = (1 << m_cfg.n_partitions) - 1;
    traffic_start_cycle = m_env.m_dbg_mon.sampled_cycles;

    run_profile_integrity_case(
      what,
      16'd128,
      num_hits,
      2,
      active_pool_keys,
      1,
      inter_hit_gap_cycles,
      0,
      0,
      1'b1,
      1'b0,
      8'd0,
      0,
      0,
      0,
      timeout_cycles,
      min_max_fill,
      m_cfg.ring_buffer_n_entry,
      min_data_subheaders,
      1'b1,
      32'h0060_0a11,
      expected_partition_mask);

    check_window_service_rate(
      what,
      0,
      0,
      0,
      0,
      max_cycles_per_hit,
      num_hits);

    `uvm_info("PROF", $sformatf(
      "%s config summary: n_partitions=%0d encoder_pipe_stages=%0d pool_keys=%0d num_hits=%0d gap=%0d cycles_per_hit_limit=%0d",
      what, m_cfg.n_partitions, m_cfg.encoder_pipe_stages,
      active_pool_keys, num_hits, inter_hit_gap_cycles, max_cycles_per_hit), UVM_LOW)
  endtask

  task automatic run_profile_exact_depth_boundary_case(
    string       what,
    int unsigned latency,
    int unsigned key_ord
  );
    same_key_burst_seq burst_seq;
    single_push_pop_seq single_seq;
    int unsigned focus_search_key;
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned fill_level;

    configure_and_start(latency);
    focus_search_key = m_cfg.lane_key_ord_to_search_key(key_ord);

    burst_seq = same_key_burst_seq::type_id::create({what, "_prefill"});
    burst_seq.num_hits = m_cfg.ring_buffer_n_entry;
    burst_seq.search_key = focus_search_key;
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_push_count(m_cfg.ring_buffer_n_entry, 80_000, {what, " exact-depth fill"});
    wait_clocks(4);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (push_count != m_cfg.ring_buffer_n_entry || overwrite_count != 0 ||
        fill_level != m_cfg.ring_buffer_n_entry) begin
      `uvm_error("PROF", $sformatf(
        "%s exact-depth precondition mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d expected_push_fill=%0d",
        what, push_count, pop_count, overwrite_count, fill_level, m_cfg.ring_buffer_n_entry))
    end

    single_seq = single_push_pop_seq::type_id::create({what, "_overflow"});
    single_seq.search_key = focus_search_key;
    single_seq.start(m_env.m_hit_seqr);

    wait_for_push_count(m_cfg.ring_buffer_n_entry + 1, 80_000, {what, " depth-plus-one push"});
    wait_clocks(4);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (overwrite_count != 1 || fill_level != m_cfg.ring_buffer_n_entry) begin
      `uvm_error("PROF", $sformatf(
        "%s depth+1 overwrite mismatch: overwrite=%0d fill=%0d expected=1/%0d",
        what, overwrite_count, fill_level, m_cfg.ring_buffer_n_entry))
    end

    terminate_and_drain(200_000, {what, " terminate drain"});
    expect_service_model_accounting({what, " terminate drain"}, 1, 1);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    if (push_count != (m_cfg.ring_buffer_n_entry + 1) ||
        pop_count != m_cfg.ring_buffer_n_entry ||
        overwrite_count != 1 ||
        fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s final accounting mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d expected=%0d/%0d/1/0",
        what, push_count, pop_count, overwrite_count, fill_level,
        m_cfg.ring_buffer_n_entry + 1, m_cfg.ring_buffer_n_entry))
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: push=%0d pop=%0d overwrite=%0d fill=%0d max_live_fill=%0d max_remaining=%0d",
      what, push_count, pop_count, overwrite_count, fill_level,
      m_env.m_dbg_mon.max_live_fill, m_env.m_scb.max_remaining_entries()), UVM_LOW)
  endtask

  task automatic check_per_key_drain_conservation(
    string       what,
    int unsigned lane_key_start_ord,
    int unsigned pool_keys,
    int unsigned min_active_keys = 1,
    bit          allow_overwrites = 1'b0
  );
    int unsigned active_keys;
    int unsigned search_key;
    int unsigned accepted_hits;
    int unsigned written_hits;
    int unsigned drained_hits;
    int unsigned evicted_hits;
    int unsigned data_subheaders;

    active_keys = 0;
    for (int idx = 0; idx < pool_keys; idx++) begin
      search_key = m_cfg.lane_key_ord_to_search_key(lane_key_start_ord + idx);
      accepted_hits = m_env.m_scb.accepted_hits_for_key(search_key[7:0]);
      written_hits = m_env.m_scb.written_hits_for_key(search_key[7:0]);
      drained_hits = m_env.m_scb.drained_hits_for_key(search_key[7:0]);
      evicted_hits = m_env.m_scb.overwrite_events_for_evicted_key(search_key[7:0]);
      data_subheaders = m_env.m_scb.data_subheaders_for_key(search_key[7:0]);
      if (accepted_hits != 0) begin
        active_keys++;
        if (!allow_overwrites && data_subheaders == 0) begin
          `uvm_error("PROF", $sformatf(
            "%s accepted hits for key=0x%02x but never emitted a data-bearing subheader",
            what, search_key[7:0]))
        end
      end
      if (drained_hits != 0 && data_subheaders == 0) begin
        `uvm_error("PROF", $sformatf(
          "%s drained hits for key=0x%02x but never emitted a data-bearing subheader",
          what, search_key[7:0]))
      end
      if (accepted_hits != written_hits ||
          (!allow_overwrites && written_hits != drained_hits) ||
          (allow_overwrites && written_hits != (drained_hits + evicted_hits))) begin
        `uvm_error("PROF", $sformatf(
          "%s per-key conservation mismatch key=0x%02x accepted=%0d written=%0d drained=%0d evicted=%0d subheaders=%0d allow_overwrites=%0d",
          what, search_key[7:0], accepted_hits, written_hits, drained_hits,
          evicted_hits, data_subheaders, allow_overwrites))
      end
    end
    if (active_keys < min_active_keys) begin
      `uvm_error("PROF", $sformatf(
        "%s activated too few keys: active=%0d expected_at_least=%0d",
        what, active_keys, min_active_keys))
    end
  endtask

  task automatic check_exact_key_counts(
    string       what,
    int unsigned lane_key_ords[$],
    int unsigned expected_hits[$],
    int unsigned min_active_keys = 1,
    bit          allow_overwrites = 1'b0
  );
    int unsigned active_keys;
    int unsigned search_key;
    int unsigned accepted_hits;
    int unsigned written_hits;
    int unsigned drained_hits;
    int unsigned evicted_hits;
    int unsigned data_subheaders;
    int unsigned total_expected;
    int unsigned total_accounted;

    if (lane_key_ords.size() != expected_hits.size()) begin
      `uvm_error("PROF", $sformatf(
        "%s exact-key-count check size mismatch: keys=%0d expected=%0d",
        what, lane_key_ords.size(), expected_hits.size()))
      return;
    end

    active_keys = 0;
    total_expected = 0;
    total_accounted = 0;
    foreach (lane_key_ords[idx]) begin
      search_key = m_cfg.lane_key_ord_to_search_key(lane_key_ords[idx]);
      accepted_hits = m_env.m_scb.accepted_hits_for_key(search_key[7:0]);
      written_hits = m_env.m_scb.written_hits_for_key(search_key[7:0]);
      drained_hits = m_env.m_scb.drained_hits_for_key(search_key[7:0]);
      evicted_hits = m_env.m_scb.overwrite_events_for_evicted_key(search_key[7:0]);
      data_subheaders = m_env.m_scb.data_subheaders_for_key(search_key[7:0]);
      total_expected += expected_hits[idx];
      total_accounted += allow_overwrites ? (drained_hits + evicted_hits) : drained_hits;
      if (expected_hits[idx] != accepted_hits ||
          expected_hits[idx] != written_hits ||
          (!allow_overwrites && expected_hits[idx] != drained_hits) ||
          (allow_overwrites && expected_hits[idx] != (drained_hits + evicted_hits))) begin
        `uvm_error("PROF", $sformatf(
          "%s exact-count mismatch key=0x%02x accepted=%0d written=%0d drained=%0d evicted=%0d expected=%0d subheaders=%0d allow_overwrites=%0d",
          what, search_key[7:0], accepted_hits, written_hits, drained_hits,
          evicted_hits, expected_hits[idx], data_subheaders, allow_overwrites))
      end
      if (expected_hits[idx] != 0) begin
        active_keys++;
        if (!allow_overwrites && data_subheaders == 0) begin
          `uvm_error("PROF", $sformatf(
            "%s key=0x%02x drained %0d hits but emitted no data-bearing subheader",
            what, search_key[7:0], expected_hits[idx]))
        end
      end
      if (drained_hits != 0 && data_subheaders == 0) begin
        `uvm_error("PROF", $sformatf(
          "%s key=0x%02x drained %0d hits but emitted no data-bearing subheader",
          what, search_key[7:0], drained_hits))
      end
    end

    if (active_keys < min_active_keys) begin
      `uvm_error("PROF", $sformatf(
        "%s activated too few exact-count keys: active=%0d expected_at_least=%0d",
        what, active_keys, min_active_keys))
    end
    if (total_accounted != total_expected) begin
      `uvm_error("PROF", $sformatf(
        "%s exact-count total mismatch: accounted=%0d expected=%0d allow_overwrites=%0d",
        what, total_accounted, total_expected, allow_overwrites))
    end
  endtask

  task automatic run_dynamic_weighted_multi_key_case(
    string       what,
    int unsigned latency,
    int unsigned lane_key_ords[$],
    int unsigned key_hits_per_epoch[$],
    int unsigned num_epochs,
    int unsigned inter_hit_gap_cycles,
    int unsigned inter_epoch_gap_cycles,
    int unsigned timeout_cycles,
    int unsigned max_overwrites = 0
  );
    weighted_profile_seq weighted_seq;
    int unsigned         expected_hits[$];
    int unsigned         total_hits;
    int unsigned         push_count;
    int unsigned         pop_count;
    int unsigned         overwrite_count;
    int unsigned         cache_miss_count;
    int unsigned         fill_level;
    int unsigned         observed_min_hits;
    int unsigned         observed_max_hits;
    int unsigned         search_key;
    int unsigned         observed_hits;

    if (lane_key_ords.size() == 0 || lane_key_ords.size() != key_hits_per_epoch.size()) begin
      `uvm_fatal("PROF", $sformatf(
        "%s requires a non-empty key list that matches the weight vector size",
        what))
    end

    total_hits = 0;
    expected_hits.delete();
    foreach (key_hits_per_epoch[idx]) begin
      expected_hits.push_back(key_hits_per_epoch[idx] * num_epochs);
      total_hits += key_hits_per_epoch[idx] * num_epochs;
    end

    configure_and_start(latency);
    weighted_seq = weighted_profile_seq::type_id::create({what, "_weighted"});
    weighted_seq.lane_key_ord_list.delete();
    weighted_seq.key_hits_per_epoch.delete();
    foreach (lane_key_ords[idx]) begin
      weighted_seq.lane_key_ord_list.push_back(lane_key_ords[idx]);
      weighted_seq.key_hits_per_epoch.push_back(key_hits_per_epoch[idx]);
    end
    weighted_seq.num_epochs = num_epochs;
    weighted_seq.inter_hit_gap_cycles = inter_hit_gap_cycles;
    weighted_seq.inter_epoch_gap_cycles = inter_epoch_gap_cycles;
    weighted_seq.progress_stride = (total_hits >= 4096) ? (total_hits / 4) : ((total_hits == 0) ? 0 : total_hits);
    weighted_seq.progress_tag = what;
    weighted_seq.start(m_env.m_hit_seqr);

    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (push_count != total_hits ||
        (pop_count + overwrite_count) != total_hits ||
        overwrite_count > max_overwrites ||
        cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected_push=%0d expected_pop_plus_overwrite=%0d max_overwrite=%0d",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        total_hits, total_hits, max_overwrites))
    end

    check_exact_key_counts(
      what,
      lane_key_ords,
      expected_hits,
      lane_key_ords.size(),
      (max_overwrites != 0));
    observed_min_hits = total_hits;
    observed_max_hits = 0;
    foreach (lane_key_ords[idx]) begin
      search_key = m_cfg.lane_key_ord_to_search_key(lane_key_ords[idx]);
      observed_hits = m_env.m_scb.drained_hits_for_key(search_key[7:0]);
      if (observed_hits < observed_min_hits) observed_min_hits = observed_hits;
      if (observed_hits > observed_max_hits) observed_max_hits = observed_hits;
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: total_hits=%0d keys=%0d min_hits_per_key=%0d max_hits_per_key=%0d max_live_fill=%0d max_remaining=%0d",
      what, total_hits, lane_key_ords.size(), observed_min_hits, observed_max_hits,
      m_env.m_dbg_mon.max_live_fill, m_env.m_scb.max_remaining_entries()), UVM_LOW)
  endtask

  task automatic run_weighted_multi_key_case(
    string       what,
    int unsigned latency,
    int unsigned lane_key_start_ord,
    int unsigned key0_hits_per_epoch,
    int unsigned key1_hits_per_epoch,
    int unsigned key2_hits_per_epoch,
    int unsigned key3_hits_per_epoch,
    int unsigned num_epochs,
    int unsigned inter_hit_gap_cycles,
    int unsigned inter_epoch_gap_cycles,
    int unsigned timeout_cycles,
    int unsigned max_overwrites = 0
  );
    int unsigned lane_key_ords[$];
    int unsigned key_hits_per_epoch[$];

    for (int idx = 0; idx < 4; idx++) begin
      lane_key_ords.push_back(lane_key_start_ord + idx);
    end
    key_hits_per_epoch.push_back(key0_hits_per_epoch);
    key_hits_per_epoch.push_back(key1_hits_per_epoch);
    key_hits_per_epoch.push_back(key2_hits_per_epoch);
    key_hits_per_epoch.push_back(key3_hits_per_epoch);

    run_dynamic_weighted_multi_key_case(
      what, latency, lane_key_ords, key_hits_per_epoch,
      num_epochs, inter_hit_gap_cycles, inter_epoch_gap_cycles, timeout_cycles, max_overwrites);
  endtask

  task automatic run_single_key_fixed_point_profile_case(
    string       what,
    int unsigned latency,
    int unsigned num_hits,
    int unsigned lane_key_ord,
    int unsigned inter_hit_gap_cycles,
    int unsigned timeout_cycles,
    int unsigned min_max_fill,
    int unsigned max_max_fill
  );
    run_profile_integrity_case(
      what,
      latency,
      num_hits,
      lane_key_ord,
      1,
      1,
      inter_hit_gap_cycles,
      0,
      0,
      1'b0,
      1'b0,
      8'd0,
      0,
      0,
      0,
      timeout_cycles,
      min_max_fill,
      max_max_fill,
      32,
      1'b1);
  endtask

  task automatic run_adversarial_overlap_profile_case(
    string       what,
    int unsigned latency,
    int unsigned lane_key_ord_a,
    int unsigned lane_key_ord_b,
    int unsigned num_epochs,
    int unsigned inter_hit_gap_cycles,
    int unsigned timeout_cycles
  );
    weighted_profile_seq weighted_seq;
    int unsigned         lane_key_ords[$];
    int unsigned         key_hits_per_epoch[$];
    int unsigned         expected_hits[$];
    int unsigned         push_count;
    int unsigned         pop_count;
    int unsigned         overwrite_count;
    int unsigned         cache_miss_count;
    int unsigned         fill_level;
    bit                  saw_search_overlap;
    bit                  saw_drain_overlap;
    bit                  traffic_done;
    int unsigned         search_cycles;

    lane_key_ords.push_back(lane_key_ord_a);
    lane_key_ords.push_back(lane_key_ord_b);
    key_hits_per_epoch.push_back(1);
    key_hits_per_epoch.push_back(1);
    expected_hits.push_back(num_epochs);
    expected_hits.push_back(num_epochs);
    saw_search_overlap = 1'b0;
    saw_drain_overlap = 1'b0;
    traffic_done = 1'b0;

    configure_and_start(latency);

    fork
      begin : overlap_traffic_thread
        weighted_seq = weighted_profile_seq::type_id::create({what, "_weighted"});
        weighted_seq.lane_key_ord_list.delete();
        weighted_seq.key_hits_per_epoch.delete();
        foreach (lane_key_ords[idx]) begin
          weighted_seq.lane_key_ord_list.push_back(lane_key_ords[idx]);
          weighted_seq.key_hits_per_epoch.push_back(key_hits_per_epoch[idx]);
        end
        weighted_seq.num_epochs = num_epochs;
        weighted_seq.inter_hit_gap_cycles = inter_hit_gap_cycles;
        weighted_seq.progress_stride = ((num_epochs * 2) >= 4096) ?
          ((num_epochs * 2) / 4) : (num_epochs * 2);
        weighted_seq.progress_tag = what;
        weighted_seq.start(m_env.m_hit_seqr);
        traffic_done = 1'b1;
      end
      begin : overlap_watch_thread
        search_cycles = 0;
        while (search_cycles < timeout_cycles &&
               (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
          if (m_env.m_dbg_mon.vif.push_write_grant === 1'b1 &&
              (m_env.m_dbg_mon.pop_engine_state_code inside {3'd1, 3'd2, 3'd3})) begin
            saw_search_overlap = 1'b1;
          end
          if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1 &&
              (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
            saw_drain_overlap = 1'b1;
          end
          @(posedge m_env.m_csr_drv.vif.clk);
          search_cycles++;
        end
      end
    join

    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (push_count != (num_epochs * 2) ||
        pop_count != (num_epochs * 2) ||
        overwrite_count != 0 ||
        cache_miss_count != 0 ||
        fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected=%0d/%0d/0/0/0",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        num_epochs * 2, num_epochs * 2))
    end

    check_exact_key_counts(what, lane_key_ords, expected_hits, 2);
    if (!saw_search_overlap) begin
      `uvm_error("PROF", $sformatf(
        "%s never observed push_write_grant during SEARCH/LOAD/COUNT",
        what))
    end
    if (!saw_drain_overlap) begin
      `uvm_error("PROF", $sformatf(
        "%s never observed pop_erase_grant while ingress traffic was still active",
        what))
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: total_hits=%0d per_key=%0d gap=%0d saw_search_overlap=%0d saw_drain_overlap=%0d max_live_fill=%0d max_remaining=%0d",
      what, num_epochs * 2, num_epochs, inter_hit_gap_cycles,
      saw_search_overlap, saw_drain_overlap,
      m_env.m_dbg_mon.max_live_fill, m_env.m_scb.max_remaining_entries()), UVM_LOW)
  endtask

  task automatic run_partition_local_profile_case(
    string       what,
    int unsigned latency,
    int unsigned prefill_lane_key_ord,
    int unsigned prefill_hits,
    int unsigned target_lane_key_ord,
    int unsigned target_hits,
    logic [3:0] expected_partition_mask,
    int unsigned timeout_cycles,
    int unsigned min_target_occupancy = 32'hffff_ffff
  );
    same_key_burst_seq burst_seq;
    int unsigned       prefill_search_key;
    int unsigned       target_search_key;
    int unsigned       required_target_occupancy;
    int unsigned       hit_before;
    int unsigned       push_count;
    int unsigned       pop_count;
    int unsigned       overwrite_count;
    int unsigned       cache_miss_count;
    int unsigned       fill_level;
    int unsigned       search_cycles;
    logic [3:0]        observed_partition_mask;

    configure_and_start(latency);

    if (prefill_hits > 0) begin
      prefill_search_key = m_cfg.lane_key_ord_to_search_key(prefill_lane_key_ord);
      burst_seq = same_key_burst_seq::type_id::create({what, "_prefill"});
      burst_seq.num_hits = prefill_hits;
      burst_seq.search_key = prefill_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(timeout_cycles, {what, " pointer-advance prefill"});
      expect_service_model_accounting({what, " pointer-advance prefill"}, 1, 0);
    end

    target_search_key = m_cfg.lane_key_ord_to_search_key(target_lane_key_ord);
    required_target_occupancy =
      (min_target_occupancy == 32'hffff_ffff) ? target_hits : min_target_occupancy;
    hit_before = m_env.m_out_mon.total_hits_seen;
    burst_seq = same_key_burst_seq::type_id::create({what, "_target"});
    burst_seq.num_hits = target_hits;
    burst_seq.search_key = target_search_key;
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_push_count(prefill_hits + target_hits, timeout_cycles / 4, {what, " fill"});
    if (required_target_occupancy != 0 &&
        m_env.m_scb.max_remaining_entries_for_key(target_search_key) < required_target_occupancy) begin
      `uvm_error("PROF", $sformatf(
        "%s never accumulated the requested target occupancy: key=0x%02x max_for_key=%0d expected=%0d",
        what, target_search_key[7:0],
        m_env.m_scb.max_remaining_entries_for_key(target_search_key), required_target_occupancy))
    end

    csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
    wait_clocks(4);
    wait_for_pop_engine_state(3'd4, 80_000, {what, " DRAIN entry"});

    observed_partition_mask = '0;
    search_cycles = 0;
    while (search_cycles < timeout_cycles &&
           (m_env.m_out_mon.total_hits_seen - hit_before) < target_hits) begin
      if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
        observed_partition_mask[m_env.m_dbg_mon.pop_issue_partition_idx] = 1'b1;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      search_cycles++;
    end

    wait_for_hit_output_count(hit_before + target_hits, timeout_cycles, {what, " hit drain"});
    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (push_count != (prefill_hits + target_hits) ||
        pop_count != (prefill_hits + target_hits) ||
        overwrite_count != 0 ||
        cache_miss_count != 0 ||
        fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected=%0d/%0d/0/0/0",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        prefill_hits + target_hits, prefill_hits + target_hits))
    end
    if (observed_partition_mask != expected_partition_mask) begin
      `uvm_error("PROF", $sformatf(
        "%s partition issue mask mismatch: observed=0x%0h expected=0x%0h",
        what, observed_partition_mask, expected_partition_mask))
    end
    if (m_env.m_scb.drained_hits_for_key(target_search_key[7:0]) != target_hits) begin
      `uvm_error("PROF", $sformatf(
        "%s target-key drain mismatch: key=0x%02x drained=%0d expected=%0d",
        what, target_search_key[7:0],
        m_env.m_scb.drained_hits_for_key(target_search_key[7:0]), target_hits))
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: prefill_hits=%0d target_hits=%0d observed_partition_mask=0x%0h max_live_fill=%0d max_remaining=%0d",
      what, prefill_hits, target_hits, observed_partition_mask,
      m_env.m_dbg_mon.max_live_fill, m_env.m_scb.max_remaining_entries()), UVM_LOW)
  endtask

  task automatic run_partition_share_profile_case(
    string       what,
    int unsigned latency,
    int unsigned target_lane_key_ord,
    int unsigned target_hits,
    input int unsigned expected_partition_hits[$],
    int unsigned timeout_cycles,
    int unsigned expected_early_partition_idx = 32'hffff_ffff,
    int unsigned max_grants_before_early_partition = 0
  );
    same_key_burst_seq burst_seq;
    int unsigned       partition_size;
    int unsigned       target_search_key;
    int unsigned       expected_total_hits;
    int unsigned       partition_grants[4];
    int unsigned       first_grant_idx[4];
    int unsigned       total_grants;
    int unsigned       visit_idx;
    int unsigned       hit_before;
    int unsigned       push_count;
    int unsigned       pop_count;
    int unsigned       overwrite_count;
    int unsigned       cache_miss_count;
    int unsigned       fill_level;
    int unsigned       search_cycles;
    logic [3:0]        expected_partition_mask;
    logic [3:0]        observed_partition_mask;

    if (expected_partition_hits.size() != m_cfg.n_partitions) begin
      `uvm_fatal("PROF", $sformatf(
        "%s expected_partition_hits size mismatch: observed=%0d expected=%0d",
        what, expected_partition_hits.size(), m_cfg.n_partitions))
    end

    partition_size = (m_cfg.n_partitions == 0) ? m_cfg.ring_buffer_n_entry :
                     (m_cfg.ring_buffer_n_entry / m_cfg.n_partitions);
    expected_total_hits = 0;
    expected_partition_mask = '0;
    observed_partition_mask = '0;
    total_grants = 0;
    for (int idx = 0; idx < 4; idx++) begin
      partition_grants[idx] = 0;
      first_grant_idx[idx] = 0;
    end
    foreach (expected_partition_hits[idx]) begin
      expected_total_hits += expected_partition_hits[idx];
      if (expected_partition_hits[idx] > partition_size) begin
        `uvm_fatal("PROF", $sformatf(
          "%s expected partition occupancy exceeds partition size: partition=%0d hits=%0d partition_size=%0d",
          what, idx, expected_partition_hits[idx], partition_size))
      end
      if (expected_partition_hits[idx] != 0) begin
        expected_partition_mask[idx[1:0]] = 1'b1;
      end
    end
    if (target_hits != expected_total_hits) begin
      `uvm_fatal("PROF", $sformatf(
        "%s target hit mismatch: target_hits=%0d expected_total=%0d",
        what, target_hits, expected_total_hits))
    end

    configure_and_start(latency);
    target_search_key = m_cfg.lane_key_ord_to_search_key(target_lane_key_ord);
    burst_seq = same_key_burst_seq::type_id::create({what, "_partition_share"});
    burst_seq.num_hits = target_hits;
    burst_seq.search_key = target_search_key;
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_push_count(target_hits, timeout_cycles / 4, {what, " fill"});
    if (m_env.m_scb.max_remaining_entries_for_key(target_search_key) < target_hits) begin
      `uvm_error("PROF", $sformatf(
        "%s never accumulated the requested occupancy: key=0x%02x max_for_key=%0d expected=%0d",
        what, target_search_key[7:0],
        m_env.m_scb.max_remaining_entries_for_key(target_search_key), target_hits))
    end

    csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
    wait_clocks(4);
    wait_for_pop_engine_state(3'd4, 80_000, {what, " DRAIN entry"});

    hit_before = m_env.m_out_mon.total_hits_seen;
    search_cycles = 0;
    while (search_cycles < timeout_cycles &&
           (m_env.m_out_mon.total_hits_seen - hit_before) < target_hits) begin
      if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
        visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
        if (visit_idx >= m_cfg.n_partitions) begin
          `uvm_error("PROF", $sformatf(
            "%s issued from partition outside active build range: partition=%0d n_partitions=%0d addr=%0d",
            what, visit_idx, m_cfg.n_partitions, m_env.m_dbg_mon.vif.pop_issue_addr))
        end else begin
          observed_partition_mask[visit_idx[1:0]] = 1'b1;
          if (partition_grants[visit_idx] == 0) begin
            first_grant_idx[visit_idx] = total_grants + 1;
          end
          partition_grants[visit_idx]++;
          total_grants++;
        end
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      search_cycles++;
    end

    wait_for_hit_output_count(hit_before + target_hits, timeout_cycles, {what, " hit drain"});
    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (push_count != target_hits || pop_count != target_hits ||
        overwrite_count != 0 || cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected=%0d/%0d/0/0/0",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        target_hits, target_hits))
    end
    if ((observed_partition_mask & expected_partition_mask) != expected_partition_mask) begin
      `uvm_error("PROF", $sformatf(
        "%s partition visit mask mismatch: observed=0x%0h expected=0x%0h",
        what, observed_partition_mask, expected_partition_mask))
    end
    foreach (expected_partition_hits[idx]) begin
      if (partition_grants[idx] != expected_partition_hits[idx]) begin
        `uvm_error("PROF", $sformatf(
          "%s partition grant mismatch: partition=%0d observed=%0d expected=%0d",
          what, idx, partition_grants[idx], expected_partition_hits[idx]))
      end
    end
    if (expected_early_partition_idx < m_cfg.n_partitions &&
        max_grants_before_early_partition != 0 &&
        expected_partition_hits[expected_early_partition_idx] != 0) begin
      if (first_grant_idx[expected_early_partition_idx] == 0 ||
          first_grant_idx[expected_early_partition_idx] > max_grants_before_early_partition) begin
        `uvm_error("PROF", $sformatf(
          "%s early-service partition did not appear soon enough: partition=%0d first_grant=%0d required_at_or_before=%0d",
          what,
          expected_early_partition_idx,
          first_grant_idx[expected_early_partition_idx],
          max_grants_before_early_partition))
      end
    end
    if (m_env.m_scb.drained_hits_for_key(target_search_key[7:0]) != target_hits) begin
      `uvm_error("PROF", $sformatf(
        "%s target-key drain mismatch: key=0x%02x drained=%0d expected=%0d",
        what, target_search_key[7:0],
        m_env.m_scb.drained_hits_for_key(target_search_key[7:0]), target_hits))
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: target_hits=%0d observed_partition_mask=0x%0h grants=[%0d,%0d,%0d,%0d] first_grant=[%0d,%0d,%0d,%0d] max_live_fill=%0d max_remaining=%0d",
      what, target_hits, observed_partition_mask,
      partition_grants[0], partition_grants[1], partition_grants[2], partition_grants[3],
      first_grant_idx[0], first_grant_idx[1], first_grant_idx[2], first_grant_idx[3],
      m_env.m_dbg_mon.max_live_fill, m_env.m_scb.max_remaining_entries()), UVM_LOW)
  endtask

  task automatic run_staged_partition_share_profile_case(
    string       what,
    int unsigned latency,
    int unsigned prefill_lane_key_ord,
    int unsigned prefill_hits,
    int unsigned target_lane_key_ord,
    int unsigned target_hits,
    input int unsigned expected_partition_hits[$],
    int unsigned timeout_cycles,
    logic [3:0]  required_pending_mask = 4'b0000,
    int unsigned handoff_progress_hits = 0,
    int unsigned handoff_progress_timeout = 200_000
  );
    same_key_burst_seq burst_seq;
    int unsigned       partition_size;
    int unsigned       prefill_search_key;
    int unsigned       target_search_key;
    int unsigned       expected_total_hits;
    int unsigned       partition_grants[4];
    int unsigned       total_grants;
    int unsigned       visit_idx;
    int unsigned       hit_before;
    int unsigned       push_count;
    int unsigned       pop_count;
    int unsigned       overwrite_count;
    int unsigned       cache_miss_count;
    int unsigned       fill_level;
    int unsigned       search_cycles;
    int unsigned       handoff_seen;
    logic [3:0]        expected_partition_mask;
    logic [3:0]        observed_partition_mask;
    bit                saw_required_pending_mask;

    if (expected_partition_hits.size() != m_cfg.n_partitions) begin
      `uvm_fatal("PROF", $sformatf(
        "%s expected_partition_hits size mismatch: observed=%0d expected=%0d",
        what, expected_partition_hits.size(), m_cfg.n_partitions))
    end

    partition_size = (m_cfg.n_partitions == 0) ? m_cfg.ring_buffer_n_entry :
                     (m_cfg.ring_buffer_n_entry / m_cfg.n_partitions);
    expected_total_hits = 0;
    expected_partition_mask = '0;
    observed_partition_mask = '0;
    total_grants = 0;
    saw_required_pending_mask = (required_pending_mask == 4'b0000);
    for (int idx = 0; idx < 4; idx++) begin
      partition_grants[idx] = 0;
    end
    foreach (expected_partition_hits[idx]) begin
      expected_total_hits += expected_partition_hits[idx];
      if (expected_partition_hits[idx] > partition_size) begin
        `uvm_fatal("PROF", $sformatf(
          "%s expected partition occupancy exceeds partition size: partition=%0d hits=%0d partition_size=%0d",
          what, idx, expected_partition_hits[idx], partition_size))
      end
      if (expected_partition_hits[idx] != 0) begin
        expected_partition_mask[idx[1:0]] = 1'b1;
      end
    end
    if (target_hits != expected_total_hits) begin
      `uvm_fatal("PROF", $sformatf(
        "%s target hit mismatch: target_hits=%0d expected_total=%0d",
        what, target_hits, expected_total_hits))
    end
    if (handoff_progress_hits >= target_hits && handoff_progress_hits != 0) begin
      `uvm_fatal("PROF", $sformatf(
        "%s handoff_progress_hits must stay below target_hits: progress=%0d target=%0d",
        what, handoff_progress_hits, target_hits))
    end

    configure_and_start(latency);

    if (prefill_hits > 0) begin
      prefill_search_key = m_cfg.lane_key_ord_to_search_key(prefill_lane_key_ord);
      burst_seq = same_key_burst_seq::type_id::create({what, "_prefill"});
      burst_seq.num_hits = prefill_hits;
      burst_seq.search_key = prefill_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(timeout_cycles, {what, " staged prefill drain"});
      expect_service_model_accounting({what, " staged prefill drain"}, 1, 0);
    end

    target_search_key = m_cfg.lane_key_ord_to_search_key(target_lane_key_ord);
    hit_before = m_env.m_out_mon.total_hits_seen;
    burst_seq = same_key_burst_seq::type_id::create({what, "_target"});
    burst_seq.num_hits = target_hits;
    burst_seq.search_key = target_search_key;
    burst_seq.start(m_env.m_hit_seqr);

    wait_for_push_count(prefill_hits + target_hits, timeout_cycles / 4, {what, " fill"});
    if (m_env.m_scb.max_remaining_entries_for_key(target_search_key) < target_hits) begin
      `uvm_error("PROF", $sformatf(
        "%s never accumulated the requested target occupancy: key=0x%02x max_for_key=%0d expected=%0d",
        what, target_search_key[7:0],
        m_env.m_scb.max_remaining_entries_for_key(target_search_key), target_hits))
    end

    csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
    wait_clocks(4);
    wait_for_pop_engine_state(3'd4, 80_000, {what, " DRAIN entry"});

    search_cycles = 0;
    while (search_cycles < timeout_cycles &&
           (m_env.m_out_mon.total_hits_seen - hit_before) < target_hits) begin
      if ((m_env.m_dbg_mon.pop_partition_pending & required_pending_mask) == required_pending_mask) begin
        saw_required_pending_mask = 1'b1;
      end
      if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
        visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
        if (visit_idx >= m_cfg.n_partitions) begin
          `uvm_error("PROF", $sformatf(
            "%s issued from partition outside active build range: partition=%0d n_partitions=%0d addr=%0d",
            what, visit_idx, m_cfg.n_partitions, m_env.m_dbg_mon.vif.pop_issue_addr))
        end else begin
          observed_partition_mask[visit_idx[1:0]] = 1'b1;
          partition_grants[visit_idx]++;
          total_grants++;
        end
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      search_cycles++;
    end

    if (handoff_progress_hits > 0) begin
      wait_for_hit_output_count(hit_before + handoff_progress_hits, timeout_cycles, {what, " first-stage drain"});
      if (m_env.m_out_mon.total_hits_seen < (hit_before + target_hits)) begin
        handoff_seen = m_env.m_out_mon.total_hits_seen;
        search_cycles = 0;
        while (search_cycles < handoff_progress_timeout &&
               m_env.m_out_mon.total_hits_seen == handoff_seen) begin
          @(posedge m_env.m_csr_drv.vif.clk);
          search_cycles++;
        end
        if (m_env.m_out_mon.total_hits_seen == handoff_seen) begin
          `uvm_error("PROF", $sformatf(
            "%s stalled after the first staged partition window drained: observed_hits=%0d expected_total=%0d",
            what, handoff_seen - hit_before, target_hits))
        end
      end
    end

    wait_for_hit_output_count(hit_before + target_hits, timeout_cycles, {what, " hit drain"});
    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (push_count != (prefill_hits + target_hits) ||
        pop_count != (prefill_hits + target_hits) ||
        overwrite_count != 0 || cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected=%0d/%0d/0/0/0",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        prefill_hits + target_hits, prefill_hits + target_hits))
    end
    if ((observed_partition_mask & expected_partition_mask) != expected_partition_mask) begin
      `uvm_error("PROF", $sformatf(
        "%s partition visit mask mismatch: observed=0x%0h expected=0x%0h",
        what, observed_partition_mask, expected_partition_mask))
    end
    if (!saw_required_pending_mask) begin
      `uvm_error("PROF", $sformatf(
        "%s never observed the required concurrent pending mask: required=0x%0h",
        what, required_pending_mask))
    end
    foreach (expected_partition_hits[idx]) begin
      if (partition_grants[idx] != expected_partition_hits[idx]) begin
        `uvm_error("PROF", $sformatf(
          "%s partition grant mismatch: partition=%0d observed=%0d expected=%0d",
          what, idx, partition_grants[idx], expected_partition_hits[idx]))
      end
    end
    if (m_env.m_scb.drained_hits_for_key(target_search_key[7:0]) != target_hits) begin
      `uvm_error("PROF", $sformatf(
        "%s target-key drain mismatch: key=0x%02x drained=%0d expected=%0d",
        what, target_search_key[7:0],
        m_env.m_scb.drained_hits_for_key(target_search_key[7:0]), target_hits))
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: prefill_hits=%0d target_hits=%0d observed_partition_mask=0x%0h required_pending_mask=0x%0h grants=[%0d,%0d,%0d,%0d] max_live_fill=%0d max_remaining=%0d",
      what, prefill_hits, target_hits, observed_partition_mask, required_pending_mask,
      partition_grants[0], partition_grants[1], partition_grants[2], partition_grants[3],
      m_env.m_dbg_mon.max_live_fill, m_env.m_scb.max_remaining_entries()), UVM_LOW)
  endtask

  task automatic run_multi_key_profile_case(
    string       what,
    int unsigned latency,
    int unsigned num_hits,
    int unsigned lane_key_start_ord,
    int unsigned pool_keys,
    int unsigned hits_per_key_switch,
    bit          randomize_key_order,
    int unsigned inter_hit_gap_cycles,
    int unsigned timeout_cycles,
    logic [31:0] lfsr_seed = 32'h1ace_b00c,
    int unsigned max_overwrites = 0
  );
    profile_traffic_seq prof_seq;
    int unsigned push_count;
    int unsigned pop_count;
    int unsigned overwrite_count;
    int unsigned cache_miss_count;
    int unsigned fill_level;
    int unsigned min_hits;
    int unsigned max_hits;
    int unsigned search_key;
    int unsigned observed_hits;

    configure_and_start(latency);
    prof_seq = profile_traffic_seq::type_id::create({what, "_profile"});
    prof_seq.num_hits = num_hits;
    prof_seq.lane_key_start_ord = lane_key_start_ord;
    prof_seq.pool_keys = pool_keys;
    prof_seq.hits_per_key_switch = hits_per_key_switch;
    prof_seq.randomize_key_order = randomize_key_order;
    prof_seq.inter_hit_gap_cycles = inter_hit_gap_cycles;
    prof_seq.lfsr_seed = lfsr_seed;
    prof_seq.progress_stride = (num_hits >= 4096) ? (num_hits / 4) : (num_hits / 2);
    prof_seq.progress_tag = what;
    prof_seq.start(m_env.m_hit_seqr);

    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (push_count != num_hits ||
        (pop_count + overwrite_count) != num_hits ||
        overwrite_count > max_overwrites ||
        cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected_push=%0d expected_pop_plus_overwrite=%0d max_overwrite=%0d",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        num_hits, num_hits, max_overwrites))
    end

    check_per_key_drain_conservation(
      what,
      lane_key_start_ord,
      pool_keys,
      pool_keys,
      (max_overwrites != 0));
    min_hits = num_hits;
    max_hits = 0;
    for (int idx = 0; idx < pool_keys; idx++) begin
      search_key = m_cfg.lane_key_ord_to_search_key(lane_key_start_ord + idx);
      observed_hits = m_env.m_scb.drained_hits_for_key(search_key[7:0]);
      if (observed_hits < min_hits) min_hits = observed_hits;
      if (observed_hits > max_hits) max_hits = observed_hits;
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: num_hits=%0d pool_keys=%0d min_hits_per_key=%0d max_hits_per_key=%0d seed=0x%08x max_live_fill=%0d",
      what, num_hits, pool_keys, min_hits, max_hits, lfsr_seed,
      m_env.m_dbg_mon.max_live_fill), UVM_LOW)
  endtask

  task automatic run_staggered_multi_key_profile_case(
    string       what,
    int unsigned latency,
    int unsigned early_num_hits,
    int unsigned late_num_hits,
    int unsigned early_lane_key_start_ord,
    int unsigned early_pool_keys,
    int unsigned late_lane_key_start_ord,
    int unsigned late_pool_keys,
    int unsigned inter_hit_gap_cycles,
    int unsigned late_start_min_fill,
    int unsigned timeout_cycles,
    logic [31:0] early_lfsr_seed = 32'h0420_a11e,
    logic [31:0] late_lfsr_seed = 32'h0420_b22e,
    int unsigned max_overwrites = 0
  );
    profile_traffic_seq early_seq;
    profile_traffic_seq late_seq;
    int unsigned        push_count;
    int unsigned        pop_count;
    int unsigned        overwrite_count;
    int unsigned        cache_miss_count;
    int unsigned        fill_level;
    int unsigned        late_start_fill;
    int unsigned        combined_lane_key_ords[$];

    configure_and_start(latency);
    late_start_fill = 0;

    early_seq = profile_traffic_seq::type_id::create({what, "_early_profile"});
    early_seq.num_hits = early_num_hits;
    early_seq.lane_key_start_ord = early_lane_key_start_ord;
    early_seq.pool_keys = early_pool_keys;
    early_seq.hits_per_key_switch = 1;
    early_seq.randomize_key_order = 1'b1;
    early_seq.inter_hit_gap_cycles = inter_hit_gap_cycles;
    early_seq.lfsr_seed = early_lfsr_seed;
    early_seq.fingerprint_start_index = 0;
    early_seq.progress_stride = (early_num_hits >= 4096) ? (early_num_hits / 4) : early_num_hits;
    early_seq.progress_tag = {what, " early"};
    early_seq.start(m_env.m_hit_seqr);

    late_start_fill = m_env.m_scb.remaining_entries();
    if (late_start_fill < late_start_min_fill) begin
      `uvm_error("PROF", $sformatf(
        "%s never reached the staged-fill precondition before launching late keys: observed_fill=%0d expected_at_least=%0d",
        what, late_start_fill, late_start_min_fill))
    end

    combined_lane_key_ords.delete();
    for (int idx = 0; idx < early_pool_keys; idx++) begin
      combined_lane_key_ords.push_back(early_lane_key_start_ord + idx);
    end
    for (int idx = 0; idx < late_pool_keys; idx++) begin
      combined_lane_key_ords.push_back(late_lane_key_start_ord + idx);
    end

    late_seq = profile_traffic_seq::type_id::create({what, "_late_profile"});
    late_seq.num_hits = late_num_hits;
    late_seq.lane_key_ord_list.delete();
    foreach (combined_lane_key_ords[idx]) begin
      late_seq.lane_key_ord_list.push_back(combined_lane_key_ords[idx]);
    end
    late_seq.pool_keys = combined_lane_key_ords.size();
    late_seq.hits_per_key_switch = 1;
    late_seq.randomize_key_order = 1'b1;
    late_seq.inter_hit_gap_cycles = inter_hit_gap_cycles;
    late_seq.lfsr_seed = late_lfsr_seed;
    late_seq.fingerprint_start_index = early_num_hits;
    late_seq.progress_stride = (late_num_hits >= 4096) ? (late_num_hits / 4) : late_num_hits;
    late_seq.progress_tag = {what, " late"};
    late_seq.start(m_env.m_hit_seqr);

    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (push_count != (early_num_hits + late_num_hits) ||
        (pop_count + overwrite_count) != (early_num_hits + late_num_hits) ||
        overwrite_count > max_overwrites ||
        cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected_push=%0d expected_pop_plus_overwrite=%0d max_overwrite=%0d",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        early_num_hits + late_num_hits, early_num_hits + late_num_hits, max_overwrites))
    end

    check_per_key_drain_conservation(
      {what, " early-key conservation"},
      early_lane_key_start_ord,
      early_pool_keys,
      early_pool_keys,
      (max_overwrites != 0));
    check_per_key_drain_conservation(
      {what, " late-key conservation"},
      late_lane_key_start_ord,
      late_pool_keys,
      late_pool_keys,
      (max_overwrites != 0));

    `uvm_info("PROF", $sformatf(
      "%s summary: early_hits=%0d late_hits=%0d late_start_fill=%0d max_live_fill=%0d max_remaining=%0d",
      what, early_num_hits, late_num_hits, late_start_fill,
      m_env.m_dbg_mon.max_live_fill, m_env.m_scb.max_remaining_entries()), UVM_LOW)
  endtask

  task automatic run_latency_reprogrammed_multi_key_case(
    string       what,
    int unsigned initial_latency,
    int unsigned num_hits,
    int unsigned lane_key_start_ord,
    int unsigned pool_keys,
    int unsigned inter_hit_gap_cycles,
    int unsigned timeout_cycles,
    logic [31:0] lfsr_seed = 32'h0430_c33f,
    int unsigned max_overwrites = 0
  );
    profile_traffic_seq prof_seq;
    int unsigned        push_count;
    int unsigned        pop_count;
    int unsigned        overwrite_count;
    int unsigned        cache_miss_count;
    int unsigned        fill_level;
    int unsigned        push_milestones[3];
    int unsigned        latency_values[3];
    logic [31:0]        latency_readback;
    bit                 saw_active_reprogram;

    push_milestones[0] = 2048;
    push_milestones[1] = 12_000;
    push_milestones[2] = 22_000;
    latency_values[0] = 16'd1;
    latency_values[1] = 16'd2000;
    latency_values[2] = 16'd128;
    saw_active_reprogram = 1'b0;

    configure_and_start(initial_latency);

    fork
      begin : latency_profile_thread
        prof_seq = profile_traffic_seq::type_id::create({what, "_profile"});
        prof_seq.num_hits = num_hits;
        prof_seq.lane_key_start_ord = lane_key_start_ord;
        prof_seq.pool_keys = pool_keys;
        prof_seq.hits_per_key_switch = 1;
        prof_seq.randomize_key_order = 1'b1;
        prof_seq.inter_hit_gap_cycles = inter_hit_gap_cycles;
        prof_seq.lfsr_seed = lfsr_seed;
        prof_seq.progress_stride = (num_hits >= 4096) ? (num_hits / 4) : num_hits;
        prof_seq.progress_tag = what;
        prof_seq.start(m_env.m_hit_seqr);
      end
      begin : latency_reprogram_thread
        for (int idx = 0; idx < 3; idx++) begin
          wait_for_push_count(
            push_milestones[idx],
            timeout_cycles,
            $sformatf("%s push milestone %0d", what, idx));
          if (m_env.m_scb.remaining_entries() != 0 ||
              m_env.m_hit_drv.pending_source_items() != 0) begin
            saw_active_reprogram = 1'b1;
          end
          csr_write(CSR_EXPECTED_LAT_ADDR, latency_values[idx]);
          wait_clocks(2);
          csr_read(CSR_EXPECTED_LAT_ADDR, latency_readback);
          if (latency_readback != latency_values[idx][31:0]) begin
            `uvm_error("PROF", $sformatf(
              "%s latency reprogram readback mismatch at step=%0d: observed=%0d expected=%0d",
              what, idx, latency_readback, latency_values[idx]))
          end
        end
      end
    join

    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
    csr_read(CSR_EXPECTED_LAT_ADDR, latency_readback);

    if (push_count != num_hits ||
        (pop_count + overwrite_count) != num_hits ||
        overwrite_count > max_overwrites ||
        cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected_push=%0d expected_pop_plus_overwrite=%0d max_overwrite=%0d",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        num_hits, num_hits, max_overwrites))
    end
    if (!saw_active_reprogram) begin
      `uvm_error("PROF", $sformatf(
        "%s never observed an EXPECTED_LATENCY rewrite while traffic or residency was still active",
        what))
    end
    if (latency_readback != latency_values[2][31:0]) begin
      `uvm_error("PROF", $sformatf(
        "%s final latency readback mismatch: observed=%0d expected=%0d",
        what, latency_readback, latency_values[2]))
    end

    check_per_key_drain_conservation(
      what,
      lane_key_start_ord,
      pool_keys,
      pool_keys,
      (max_overwrites != 0));

    `uvm_info("PROF", $sformatf(
      "%s summary: num_hits=%0d pool_keys=%0d final_latency=%0d max_live_fill=%0d max_remaining=%0d",
      what, num_hits, pool_keys, latency_readback,
      m_env.m_dbg_mon.max_live_fill, m_env.m_scb.max_remaining_entries()), UVM_LOW)
  endtask

  task automatic run_randomized_multi_key_integrity_case(
    string       what,
    int unsigned latency,
    int unsigned num_hits,
    int unsigned lane_key_start_ord,
    int unsigned pool_keys,
    int unsigned inter_hit_gap_cycles,
    int unsigned timeout_cycles,
    int unsigned max_overwrites = 0
  );
    run_multi_key_profile_case(
      what,
      latency,
      num_hits,
      lane_key_start_ord,
      pool_keys,
      1,
      1'b1,
      inter_hit_gap_cycles,
      timeout_cycles,
      32'h1ace_b00c,
      max_overwrites);
  endtask

  task automatic run_silent_key_profile_case(
    string       what,
    int unsigned latency,
    int unsigned num_hits,
    int unsigned inter_hit_gap_cycles,
    int unsigned active_lane_key_ords[$],
    int unsigned silent_lane_key_ords[$],
    int unsigned timeout_cycles
  );
    profile_traffic_seq              prof_seq;
    ring_buffer_cam_pkg::out_seq_item matched_subheader;
    int unsigned                     push_count;
    int unsigned                     pop_count;
    int unsigned                     overwrite_count;
    int unsigned                     cache_miss_count;
    int unsigned                     fill_level;
    int unsigned                     search_key;
    int unsigned                     active_total;
    int unsigned                     accepted_hits;
    int unsigned                     written_hits;
    int unsigned                     drained_hits;

    configure_and_start(latency);
    prof_seq = profile_traffic_seq::type_id::create({what, "_silent"});
    prof_seq.num_hits = num_hits;
    prof_seq.lane_key_ord_list.delete();
    foreach (active_lane_key_ords[idx]) begin
      prof_seq.lane_key_ord_list.push_back(active_lane_key_ords[idx]);
    end
    prof_seq.pool_keys = active_lane_key_ords.size();
    prof_seq.hits_per_key_switch = 1;
    prof_seq.randomize_key_order = 1'b0;
    prof_seq.inter_hit_gap_cycles = inter_hit_gap_cycles;
    prof_seq.progress_stride = (num_hits >= 4096) ? (num_hits / 4) : ((num_hits == 0) ? 0 : num_hits);
    prof_seq.progress_tag = what;
    prof_seq.start(m_env.m_hit_seqr);

    foreach (silent_lane_key_ords[idx]) begin
      search_key = m_cfg.lane_key_ord_to_search_key(silent_lane_key_ords[idx]);
      wait_for_subheader_search_key(
        search_key[7:0],
        timeout_cycles,
        {what, " silent-key search"},
        matched_subheader);
      if (matched_subheader == null) begin
        `uvm_error("PROF", $sformatf(
          "%s did not observe a subheader for silent key=0x%02x",
          what, search_key[7:0]))
      end else begin
        if (matched_subheader.hit_count != 0) begin
          `uvm_error("PROF", $sformatf(
            "%s silent key=0x%02x emitted non-zero hit_count=%0d",
            what, search_key[7:0], matched_subheader.hit_count))
        end
        if (!(matched_subheader.sop && matched_subheader.eop)) begin
          `uvm_error("PROF", $sformatf(
            "%s silent key=0x%02x did not emit a single-beat zero-hit subheader",
            what, search_key[7:0]))
        end
      end
    end

    wait_for_scoreboard_idle(timeout_cycles, {what, " drain"});
    expect_service_model_accounting({what, " drain"}, 1, 0);
    read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
    read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
    read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
    read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
    read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);

    if (push_count != num_hits || pop_count != num_hits ||
        overwrite_count != 0 || cache_miss_count != 0 || fill_level != 0) begin
      `uvm_error("PROF", $sformatf(
        "%s accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected=%0d/%0d/0/0/0",
        what, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
        num_hits, num_hits))
    end

    active_total = 0;
    foreach (active_lane_key_ords[idx]) begin
      search_key = m_cfg.lane_key_ord_to_search_key(active_lane_key_ords[idx]);
      accepted_hits = m_env.m_scb.accepted_hits_for_key(search_key[7:0]);
      written_hits = m_env.m_scb.written_hits_for_key(search_key[7:0]);
      drained_hits = m_env.m_scb.drained_hits_for_key(search_key[7:0]);
      active_total += drained_hits;
      if (accepted_hits == 0 || accepted_hits != written_hits || written_hits != drained_hits) begin
        `uvm_error("PROF", $sformatf(
          "%s active key conservation mismatch key=0x%02x accepted=%0d written=%0d drained=%0d",
          what, search_key[7:0], accepted_hits, written_hits, drained_hits))
      end
    end

    foreach (silent_lane_key_ords[idx]) begin
      search_key = m_cfg.lane_key_ord_to_search_key(silent_lane_key_ords[idx]);
      accepted_hits = m_env.m_scb.accepted_hits_for_key(search_key[7:0]);
      written_hits = m_env.m_scb.written_hits_for_key(search_key[7:0]);
      drained_hits = m_env.m_scb.drained_hits_for_key(search_key[7:0]);
      if (accepted_hits != 0 || written_hits != 0 || drained_hits != 0) begin
        `uvm_error("PROF", $sformatf(
          "%s silent key accounting mismatch key=0x%02x accepted=%0d written=%0d drained=%0d",
          what, search_key[7:0], accepted_hits, written_hits, drained_hits))
      end
    end

    if (active_total != num_hits) begin
      `uvm_error("PROF", $sformatf(
        "%s drained total across active keys did not close: drained=%0d expected=%0d",
        what, active_total, num_hits))
    end

    `uvm_info("PROF", $sformatf(
      "%s summary: num_hits=%0d active_keys=%0d silent_keys=%0d zero_hit_subheaders=%0d cache_miss=%0d max_live_fill=%0d",
      what, num_hits, active_lane_key_ords.size(), silent_lane_key_ords.size(),
      m_env.m_scb.total_zero_hit_subheaders, cache_miss_count,
      m_env.m_dbg_mon.max_live_fill), UVM_LOW)
  endtask

  function automatic logic [31:0] expected_meta_version();
    logic [31:0] version_word;
    version_word = '0;
    version_word[31:24] = 8'd26;
    version_word[23:16] = 8'd2;
    version_word[15:12] = 4'd4;
    version_word[11:0]  = 12'd421;
    return version_word;
  endfunction

  task automatic set_meta_sel(logic [1:0] sel);
    csr_write(CSR_META_ADDR, {30'd0, sel});
    wait_clocks(2);
  endtask

  task automatic expect_csr_write_no_effect(
    int unsigned addr,
    logic [31:0] write_data,
    logic [31:0] mask,
    string       what
  );
    logic [31:0] before_data;
    logic [31:0] after_data;

    csr_read(addr, before_data);
    csr_write(addr, write_data);
    wait_clocks(2);
    csr_read(addr, after_data);
    if ((before_data & mask) !== (after_data & mask)) begin
      `uvm_error("CSR_RO", $sformatf(
        "%s changed under write: before=0x%08x after=0x%08x write=0x%08x mask=0x%08x",
        what, before_data, after_data, write_data, mask))
    end
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

  task automatic wait_for_pop_cmd_fifo_usedw_at_least(
    input int unsigned target_usedw,
    input int unsigned max_cycles = 50_000,
    input string what = "pop_cmd_fifo_usedw threshold"
  );
    int unsigned cycles;
    cycles = 0;
    while (cycles < max_cycles) begin
      if (m_env.m_dbg_mon.pop_cmd_fifo_usedw >= target_usedw[3:0]) begin
        return;
      end
      @(posedge m_env.m_csr_drv.vif.clk);
      cycles++;
    end
    `uvm_error("TIMEOUT", $sformatf(
      "Timed out waiting for %s after %0d cycles: observed_usedw=%0d target_usedw=%0d",
      what, max_cycles, m_env.m_dbg_mon.pop_cmd_fifo_usedw, target_usedw))
  endtask

  task automatic run_case_by_id(string case_id);
    single_push_pop_seq   single_seq;
    same_key_burst_seq    burst_seq;
    sequential_keys_seq   seq_keys;
    overwrite_stress_seq  ow_seq;
    overwrite_profile_seq pressure_seq;
    weighted_profile_seq  weighted_seq;
    random_push_pop_seq   rand_same;
    random_multi_key_seq  rand_multi;
    random_throughput_seq rand_tp;
    single_error_hit_seq  err_seq;
    error_burst_seq       err_burst;
    endofrun_marker_seq   eor_seq;
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
    int unsigned          ow16_b;
    int unsigned          ow20_b;
    int unsigned          ow24_b;
    int unsigned          ow28_b;
    int unsigned          ow_min;
    int unsigned          ow_max;
    int unsigned          push_count;
    int unsigned          pop_count;
    int unsigned          overwrite_count;
    int unsigned          inerr_count;
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
    int unsigned          error_hit_count;
    int unsigned          partition_size;
    int unsigned          prefill_hits;
    int unsigned          latency_choice;
    logic [3:0]           load_mask;
    logic [1:0]           rr_before;
    int unsigned          last_partition_idx;
    bit [3:0]             expected_partition_mask;
    bit                   saw_load;
    bit                   saw_overlap;
    bit                   saw_rr_step1;
    bit                   saw_rr_step2;
    bit                   saw_last_partition;
    bit                   saw_rr_advance;
    bit                   saw_ram_done;
    bit                   saw_cam_done;
    bit                   saw_done_pulse;
    bit                   saw_subheader_channel;
    bit                   saw_hit_channel;
    bit                   saw_idle_bubble;
    int unsigned          visit_idx;
    bit                   saw_push_511;
    bit                   saw_push_000;
    bit                   saw_pop_510;
    int unsigned          grant_count;
    bit                   saw_preempt;
    bit                   saw_full;
    bit                   saw_ready_recover;
    bit                   saw_stage0;
    bit                   saw_cache_miss;
    bit                   saw_term_deassm_block;
    bit                   saw_term_popcmd_block;
    bit                   saw_term_busy_block;
    bit                   saw_term_accounting_block;
    int unsigned          reads_needed;
    int unsigned          max_cmd_usedw;
    int unsigned          prev_flush_ram_addr;
    int unsigned          prev_flush_cam_addr;
    int unsigned          prev_flush_cam_data;
    bit                   traffic_done;
    longint unsigned      cycle_a;
    longint unsigned      cycle_b;

    partition_size = (m_cfg.n_partitions == 0) ? m_cfg.ring_buffer_n_entry :
                     (m_cfg.ring_buffer_n_entry / m_cfg.n_partitions);
    prefill_hits = 0;

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
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd20260421, "META DATE readback");
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
    end else if (case_id == "B029") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      saw_cache_miss = 1'b0;
      burst_seq = same_key_burst_seq::type_id::create("b029_forced_cache_miss");
      burst_seq.num_hits = 1;
      burst_seq.search_key = focus_search_key;
      fork
        begin
          poison_side_ram_occ_zero_on_push_write(1, 40_000, "B029 forced cache miss");
        end
        begin
          burst_seq.start(m_env.m_hit_seqr);
        end
      join
      wait_for_hit_output_count(hit_before + 1, 80_000, "B029 forced cache-miss output");
      wait_for_cache_miss_count(1, 80_000, "B029 cache-miss counter");
      wait_for_scoreboard_idle(80_000, "B029 forced cache-miss drain");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (m_env.m_out_mon.recent_hits.size() > 0 &&
          m_env.m_out_mon.recent_hits[$].cache_miss) begin
        saw_cache_miss = 1'b1;
      end
      if (push_count != 1 || pop_count != 1 ||
          cache_miss_count != 1 || fill_level != 0) begin
        `uvm_error("B029", $sformatf(
          "Forced cache-miss accounting mismatch: push=%0d pop=%0d cache_miss=%0d fill=%0d",
          push_count, pop_count, cache_miss_count, fill_level))
      end
      if (m_env.m_dbg_mon.cache_miss_pulse_count == 0) begin
        `uvm_error("B029", "Forced cache miss never raised pop_cache_miss_pulse")
      end
      if (m_env.m_scb.total_cache_miss_outputs != 1) begin
        `uvm_error("B029", $sformatf(
          "Forced cache miss did not produce exactly one cache-miss data beat: observed=%0d",
          m_env.m_scb.total_cache_miss_outputs))
      end
      if (!saw_cache_miss) begin
        `uvm_error("B029", "Forced cache miss did not tag the drained hit as cache_miss")
      end
    end else if (case_id == "B030") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      single_seq = single_push_pop_seq::type_id::create("b030_cache_miss_seed");
      single_seq.search_key = focus_search_key;
      fork
        begin
          poison_side_ram_occ_zero_on_push_write(1, 40_000, "B030 cache-miss seed");
        end
        begin
          single_seq.start(m_env.m_hit_seqr);
        end
      join
      wait_for_cache_miss_count(1, 80_000, "B030 cache-miss seed");
      wait_for_scoreboard_idle(80_000, "B030 cache-miss seed drain");
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd2000);
      wait_clocks(4);
      burst_seq = same_key_burst_seq::type_id::create("b030_overwrite_fill");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry + 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry + 2, 120_000, "B030 overwrite precondition");
      search_cycles = 0;
      overwrite_count = 0;
      while (search_cycles < 1_024 && overwrite_count == 0) begin
        read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
        if (overwrite_count != 0) begin
          break;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      if (push_count == 0 || pop_count == 0 ||
          overwrite_count == 0 || cache_miss_count == 0) begin
        `uvm_error("B030", $sformatf(
          "FLUSHING precondition did not accumulate non-zero counters: push=%0d pop=%0d overwrite=%0d cache_miss=%0d",
          push_count, pop_count, overwrite_count, cache_miss_count))
      end
      enter_run_prepare();
      expect_main_activity_counters_zero("B030 FLUSHING counter clear");
    end else if (case_id == "B031") begin
      set_meta_sel(2'b01);
      csr_expect_mask(CSR_UID_ADDR, 32'hFFFF_FFFF, 32'h5242_434d, "UID stable across META sel write");
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd20260421, "META DATE after selector write");
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
    end else if (case_id == "B047") begin
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd16);
      wait_clocks(2);
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
      wait_for_run_state(4'd3, 2_000, "B047 direct RUNNING entry");
      if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b0) begin
        `uvm_error("B047", "Direct IDLE->RUNNING unexpectedly left run_mgmt_flushed asserted")
      end
      burst_seq = same_key_burst_seq::type_id::create("b047_direct_running");
      burst_seq.num_hits = 4;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(4, 20_000, "B047 direct RUNNING accepts ingress");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      if (push_count != 4) begin
        `uvm_error("B047", $sformatf(
          "Direct RUNNING did not accept the 4-hit burst: push_count=%0d",
          push_count))
      end
      enter_run_prepare();
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "B047 PREP flush clears PUSH_COUNT");
      csr_expect_mask(CSR_POP_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "B047 PREP flush clears POP_COUNT");
      csr_expect_mask(CSR_OVERWRITE_ADDR, 32'hFFFF_FFFF, 32'd0, "B047 PREP flush clears OVERWRITE_COUNT");
      csr_expect_mask(CSR_FILL_LEVEL_ADDR, 32'hFFFF_FFFF, 32'd0, "B047 PREP flush clears FILL_LEVEL");
      if (m_env.m_scb.remaining_entries() != 0 || m_env.m_scb.pending_drain_entries() != 0) begin
        `uvm_error("B047", $sformatf(
          "PREP flush did not clear the scoreboard model after direct RUNNING: remaining=%0d pending_drain=%0d",
          m_env.m_scb.remaining_entries(), m_env.m_scb.pending_drain_entries()))
      end
    end else if (case_id == "B048") begin
      enter_run_prepare();
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd16);
      wait_clocks(2);
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
      wait_for_run_state(4'd3, 2_000, "B048 PREP->RUNNING entry");
      if (m_env.m_dbg_mon.gts_counter_rst !== 1'b0) begin
        `uvm_error("B048", "Skipping SYNC should leave gts_counter_rst deasserted in RUNNING")
      end
      if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b0) begin
        `uvm_error("B048", "PREP->RUNNING did not clear run_mgmt_flushed")
      end
      burst_seq = same_key_burst_seq::type_id::create("b048_presync_hits");
      burst_seq.num_hits = 4;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(3);
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(4, 20_000, "B048 PREP->RUNNING accepts ingress");
      ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
      wait_clocks(2);
      wait_for_run_state(4'd2, 2_000, "B048 late SYNC entry");
      if (m_env.m_dbg_mon.gts_counter_rst !== 1'b1 || m_env.m_dbg_mon.gts_8n != 0) begin
        `uvm_error("B048", $sformatf(
          "Late SYNC did not reset gts_8n cleanly: rst=%0d gts=%0d",
          m_env.m_dbg_mon.gts_counter_rst, m_env.m_dbg_mon.gts_8n))
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
      wait_clocks(4);
      wait_for_run_state(4'd3, 2_000, "B048 post-SYNC RUNNING re-entry");
      wait_for_scoreboard_idle(80_000, "B048 late SYNC recovery drain");
      expect_service_model_accounting("B048 late SYNC recovery drain", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      if (push_count != 4 || pop_count != 4) begin
        `uvm_error("B048", $sformatf(
          "Late-SYNC recovery did not drain the preserved backlog: push=%0d pop=%0d",
          push_count, pop_count))
      end
      if (m_env.m_out_mon.total_subheaders_seen <= subhdr_before ||
          m_env.m_out_mon.total_hits_seen < (hit_before + 4)) begin
        `uvm_error("B048", $sformatf(
          "Late-SYNC recovery did not emit the expected output: subheaders_before=%0d subheaders_after=%0d hits_before=%0d hits_after=%0d",
          subhdr_before, m_env.m_out_mon.total_subheaders_seen,
          hit_before, m_env.m_out_mon.total_hits_seen))
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
    end else if (case_id == "B067") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq = same_key_burst_seq::type_id::create("b067_partial_cache_miss");
      burst_seq.num_hits = 2;
      burst_seq.search_key = focus_search_key;
      fork
        begin
          poison_side_ram_occ_zero_on_push_write(2, 40_000, "B067 second-hit cache miss");
        end
        begin
          burst_seq.start(m_env.m_hit_seqr);
        end
      join
      wait_for_hit_output_count(hit_before + 2, 80_000, "B067 two-hit cache-miss drain");
      wait_for_cache_miss_count(1, 80_000, "B067 cache-miss counter");
      wait_for_scoreboard_idle(80_000, "B067 partial cache-miss drain");
      matched_subheader = null;
      if (m_env.m_out_mon.total_data_subheaders_seen <= subhdr_before ||
          m_env.m_out_mon.recent_data_subheaders.size() == 0) begin
        `uvm_error("B067", "Forced cache-miss epoch did not publish a data-bearing subheader")
      end else begin
        matched_subheader = m_env.m_out_mon.recent_data_subheaders[$];
      end
      if (matched_subheader == null ||
          matched_subheader.search_key != focus_search_key[7:0] ||
          matched_subheader.hit_count != 2) begin
        `uvm_error("B067", $sformatf(
          "Forced cache-miss epoch did not preserve subheader key/hit_count: expected_key=0x%02x observed=%s",
          focus_search_key[7:0],
          (matched_subheader == null) ? "none" :
            $sformatf("key=0x%02x hit_count=%0d",
              matched_subheader.search_key, matched_subheader.hit_count)))
      end
      if (m_env.m_out_mon.recent_hits.size() < 2) begin
        `uvm_error("B067", "Forced cache-miss epoch did not capture both hit beats")
      end else begin
        if (m_env.m_out_mon.recent_hits[$-1].cache_miss) begin
          `uvm_error("B067", "First beat of the two-hit epoch was incorrectly marked cache_miss")
        end
        if (m_env.m_out_mon.recent_hits[$-1].eop) begin
          `uvm_error("B067", "First beat of the two-hit epoch asserted EOP")
        end
        if (!m_env.m_out_mon.recent_hits[$].cache_miss) begin
          `uvm_error("B067", "Second beat of the two-hit epoch did not carry cache_miss")
        end
        if (!m_env.m_out_mon.recent_hits[$].eop) begin
          `uvm_error("B067", "Second beat of the two-hit epoch did not assert EOP")
        end
      end
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      if (pop_count != 2 || cache_miss_count != 1) begin
        `uvm_error("B067", $sformatf(
          "Partial cache-miss accounting mismatch: pop=%0d cache_miss=%0d",
          pop_count, cache_miss_count))
      end
      if (m_env.m_scb.total_cache_miss_outputs != 1) begin
        `uvm_error("B067", $sformatf(
          "Expected exactly one cache-miss output in the two-hit epoch: observed=%0d",
          m_env.m_scb.total_cache_miss_outputs))
      end
    end else if (case_id == "B068") begin
      configure_and_start(64);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      single_seq = single_push_pop_seq::type_id::create("b068_good_a");
      single_seq.search_key = focus_search_key;
      single_seq.ts_low = 4'h0;
      single_seq.start(m_env.m_hit_seqr);
      single_seq = single_push_pop_seq::type_id::create("b068_glitch");
      single_seq.use_raw_tcc8n = 1'b1;
      single_seq.raw_tcc8n = m_cfg.make_tcc8n_for_lane_key(2, 4'h1, 1'b1);
      single_seq.hit_has_error = 1'b0;
      single_seq.start(m_env.m_hit_seqr);
      single_seq = single_push_pop_seq::type_id::create("b068_good_b");
      single_seq.search_key = focus_search_key;
      single_seq.ts_low = 4'h2;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_search_key(focus_search_key[7:0], 80_000, "B068 target subheader", matched_subheader);
      wait_for_hit_output_count(hit_before + 3, 80_000, "B068 good-glitch-good drain");
      if (m_env.m_out_mon.recent_hits.size() < 3) begin
        `uvm_error("B068", "Did not capture three output hits for the good-glitch-good check")
      end else begin
        error_hit_count = 0;
        for (int idx = m_env.m_out_mon.recent_hits.size() - 3;
             idx < m_env.m_out_mon.recent_hits.size();
             idx++) begin
          if (m_env.m_out_mon.recent_hits[idx].active_search_key != focus_search_key[7:0]) begin
            `uvm_error("B068", $sformatf(
              "Observed hit from the wrong search key in the glitch window: expected=0x%02x observed=0x%02x",
              focus_search_key[7:0], m_env.m_out_mon.recent_hits[idx].active_search_key))
          end
          if (m_env.m_out_mon.recent_hits[idx].error)
            error_hit_count++;
        end
        if (error_hit_count != 1) begin
          `uvm_error("B068", $sformatf(
            "Expected exactly one tsglitcherr-marked hit in the three-hit window, observed=%0d",
            error_hit_count))
        end
      end
      wait_for_scoreboard_idle(80_000, "B068 good-glitch-good drain");
      expect_service_model_accounting("B068 good-glitch-good drain", 1, 0);
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
    end else if (case_id == "B071") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b071_prefill");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry, 80_000, "B071 ring-fill precondition");
      traffic_done = 1'b0;
      burst_seq = same_key_burst_seq::type_id::create("b071_terminate_overlap");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry;
      burst_seq.search_key = focus_search_key;
      fork
        begin
          burst_seq.start(m_env.m_hit_seqr);
          traffic_done = 1'b1;
        end
        begin
          wait_for_push_count(m_cfg.ring_buffer_n_entry + 64, 120_000, "B071 overwrite-pressure start");
          ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
        end
      join
      send_endofrun_marker();
      saw_term_deassm_block = 1'b0;
      saw_term_popcmd_block = 1'b0;
      saw_term_busy_block = 1'b0;
      saw_term_accounting_block = 1'b0;
      search_cycles = 0;
      while (search_cycles < 240_000 &&
             m_env.m_dbg_mon.terminating_drain_done !== 1'b1) begin
        if (m_env.m_dbg_mon.vif.endofrun_seen === 1'b1 &&
            m_env.m_dbg_mon.terminating_drain_done === 1'b0) begin
          if (m_env.m_dbg_mon.vif.deassembly_fifo_empty !== 1'b1)
            saw_term_deassm_block = 1'b1;
          if (m_env.m_dbg_mon.vif.pop_cmd_fifo_empty !== 1'b1)
            saw_term_popcmd_block = 1'b1;
          if (m_env.m_dbg_mon.pop_engine_state_code != 3'd0)
            saw_term_busy_block = 1'b1;
          if (m_env.m_dbg_mon.dbg_push_cnt[31:0] !=
              (m_env.m_dbg_mon.dbg_pop_cnt[31:0] + m_env.m_dbg_mon.dbg_overwrite_cnt[31:0]))
            saw_term_accounting_block = 1'b1;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.terminating_drain_done !== 1'b1) begin
        `uvm_error("B071", $sformatf(
          "terminating_drain_done did not assert after %0d cycles: endofrun=%0d deassm_empty=%0d pop_cmd_empty=%0d pop_state=%0d push=%0d pop=%0d overwrite=%0d",
          search_cycles,
          m_env.m_dbg_mon.vif.endofrun_seen,
          m_env.m_dbg_mon.vif.deassembly_fifo_empty,
          m_env.m_dbg_mon.vif.pop_cmd_fifo_empty,
          m_env.m_dbg_mon.pop_engine_state_code,
          m_env.m_dbg_mon.dbg_push_cnt[31:0],
          m_env.m_dbg_mon.dbg_pop_cnt[31:0],
          m_env.m_dbg_mon.dbg_overwrite_cnt[31:0]))
      end
      if (!(saw_term_deassm_block && saw_term_popcmd_block &&
            saw_term_busy_block && saw_term_accounting_block)) begin
        `uvm_error("B071", $sformatf(
          "Terminate drain did not prove all blocking conjuncts independently: deassm=%0d popcmd=%0d busy=%0d accounting=%0d",
          saw_term_deassm_block, saw_term_popcmd_block,
          saw_term_busy_block, saw_term_accounting_block))
      end
      wait_clocks(4);
      wait_for_scoreboard_idle(300_000, "B071 terminating condition audit");
      expect_service_model_accounting("B071 terminating condition audit", 1, 1);
    end else if (case_id == "B072") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b072_prefill");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry, 80_000, "B072 ring-fill precondition");
      saw_full = 1'b0;
      saw_ready_recover = 1'b0;
      traffic_done = 1'b0;
      burst_seq = same_key_burst_seq::type_id::create("b072_fifo_backpressure");
      burst_seq.num_hits = 1024;
      burst_seq.search_key = focus_search_key;
      fork
        begin
          burst_seq.start(m_env.m_hit_seqr);
          traffic_done = 1'b1;
        end
        begin
          search_cycles = 0;
          while (search_cycles < 240_000 && !traffic_done) begin
            if (m_env.m_hit_drv.vif.ready === 1'b0 &&
                m_env.m_dbg_mon.vif.deassembly_fifo_full !== 1'b1) begin
              `uvm_error("B072", $sformatf(
                "Ingress ready dropped before deassembly_fifo_full asserted: usedw=%0d full=%0d",
                m_env.m_dbg_mon.deassembly_fifo_usedw,
                m_env.m_dbg_mon.vif.deassembly_fifo_full))
            end
            if (m_env.m_dbg_mon.vif.deassembly_fifo_full === 1'b1) begin
              saw_full = 1'b1;
              if (m_env.m_hit_drv.vif.ready !== 1'b0) begin
                `uvm_error("B072", $sformatf(
                  "Ingress ready stayed high while deassembly_fifo_full asserted: usedw=%0d",
                  m_env.m_dbg_mon.deassembly_fifo_usedw))
              end
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
        end
      join
      if (!saw_full) begin
        `uvm_error("B072", $sformatf(
          "Overwrite-pressure run never filled the deassembly FIFO: max_ready_low_streak=%0d usedw=%0d",
          m_env.m_hit_drv.max_ready_low_streak,
          m_env.m_dbg_mon.deassembly_fifo_usedw))
      end
      search_cycles = 0;
      while (search_cycles < 40_000 && !saw_ready_recover) begin
        if (m_env.m_dbg_mon.vif.deassembly_fifo_full !== 1'b1 &&
            m_env.m_hit_drv.vif.ready === 1'b1) begin
          saw_ready_recover = 1'b1;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_ready_recover) begin
        `uvm_error("B072", "Ingress ready did not recover after the deassembly FIFO left full")
      end
      terminate_and_drain(350_000, "B072 deassembly backpressure drain");
      expect_service_model_accounting("B072 deassembly backpressure drain", 1, 1);
    end else if (case_id == "B073") begin
      run_same_key_epoch_count_case("B073 occupancy-1", 1, 2, 1, 2000, 80_000, 1, 1);
    end else if (case_id == "B074") begin
      run_same_key_epoch_count_case("B074 occupancy-2", 2, 2, 2);
    end else if (case_id == "B075") begin
      configure_and_start(0);
      prefill_hits = partition_size - 1;
      burst_seq = same_key_burst_seq::type_id::create("b075_prefill");
      burst_seq.num_hits = prefill_hits;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(2_500_000, "B075 pointer advance prefill");
      expect_service_model_accounting("B075 pointer advance prefill", 1, 0);
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd2000);
      wait_clocks(4);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      burst_seq = same_key_burst_seq::type_id::create("b075_target");
      burst_seq.num_hits = 3;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(prefill_hits + 3, 20_000, "B075 boundary fill");
      if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < 3) begin
        `uvm_error("B075", $sformatf(
          "Boundary-crossing case never accumulated 3 live hits: key=%0d max_for_key=%0d",
          focus_search_key, m_env.m_scb.max_remaining_entries_for_key(focus_search_key)))
      end
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_pop_engine_state(3'd4, 40_000, "B075 DRAIN entry");
      saw_overlap = 1'b0;
      search_cycles = 0;
      while (search_cycles < 1_024 && !saw_overlap) begin
        if ((m_env.m_dbg_mon.pop_partition_pending & 4'b0011) == 4'b0011) begin
          saw_overlap = 1'b1;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_overlap) begin
        `uvm_error("B075", $sformatf(
          "Boundary-crossing case never exposed both partitions pending: pending=0x%0h rr_idx=%0d issue_idx=%0d",
          m_env.m_dbg_mon.pop_partition_pending,
          m_env.m_dbg_mon.pop_rr_idx,
          m_env.m_dbg_mon.pop_issue_partition_idx))
      end
      load_mask = '0;
      search_cycles = 0;
      while (search_cycles < 1_024 &&
             ((m_env.m_out_mon.total_hits_seen - hit_before) < 3 || load_mask != 4'b0011)) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
          load_mask[m_env.m_dbg_mon.pop_issue_partition_idx] = 1'b1;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (load_mask != 4'b0011) begin
        `uvm_error("B075", $sformatf(
          "Boundary-crossing 3-hit drain did not visit partitions 0 and 1: observed_mask=0x%0h",
          load_mask))
      end
      wait_for_hit_output_count(hit_before + 3, 200_000, "B075 boundary hit drain");
      wait_for_scoreboard_idle(160_000, "B075 boundary-crossing drain");
      if (m_env.m_out_mon.total_data_subheaders_seen <= subhdr_before ||
          m_env.m_out_mon.recent_data_subheaders.size() == 0 ||
          m_env.m_out_mon.recent_data_subheaders[$].search_key != focus_search_key[7:0]) begin
        `uvm_error("B075", $sformatf(
          "Boundary-crossing drain did not publish a target data-bearing subheader: delta=%0d last_key=%s expected=0x%02x",
          m_env.m_out_mon.total_data_subheaders_seen - subhdr_before,
          (m_env.m_out_mon.recent_data_subheaders.size() == 0) ? "none" :
            $sformatf("0x%02x", m_env.m_out_mon.recent_data_subheaders[$].search_key),
          focus_search_key[7:0]))
      end
      expect_service_model_accounting("B075 boundary-crossing drain", 1, 0);
    end else if (case_id == "B076") begin
      run_same_key_epoch_count_case("B076 occupancy-16", 16, 2, 16);
    end else if (case_id == "B077") begin
      run_same_key_epoch_count_case("B077 occupancy-17", 17, 2, 17);
    end else if (case_id == "B078") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq = same_key_burst_seq::type_id::create("b078_partition_full");
      burst_seq.num_hits = partition_size;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(partition_size, 40_000, "B078 partition-full fill");
      if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < partition_size) begin
        `uvm_error("B078", $sformatf(
          "Partition-full case never accumulated one full partition: key=%0d max_for_key=%0d partition_size=%0d",
          focus_search_key,
          m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
          partition_size))
      end
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_pop_engine_state(3'd4, 40_000, "B078 DRAIN entry");
      load_mask = '0;
      wait_for_partition_issue_mask(4'b0001, load_mask, 200_000, "B078 single-partition issue mask");
      if (load_mask != 4'b0001) begin
        `uvm_error("B078", $sformatf(
          "Exactly-one-partition drain leaked beyond partition 0: observed_mask=0x%0h",
          load_mask))
      end
      wait_for_hit_output_count(hit_before + partition_size, 3_000_000, "B078 partition-full hit drain");
      wait_for_scoreboard_idle(200_000, "B078 single-partition drain");
      expect_service_model_accounting("B078 single-partition drain", 1, 0);
    end else if (case_id == "B079") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      burst_seq = same_key_burst_seq::type_id::create("b079_partition_plus_one");
      burst_seq.num_hits = partition_size + 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(partition_size + 1, 40_000, "B079 partition+1 fill");
      if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < (partition_size + 1)) begin
        `uvm_error("B079", $sformatf(
          "Partition+1 case never accumulated the target occupancy: key=%0d max_for_key=%0d expected=%0d",
          focus_search_key,
          m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
          partition_size + 1))
      end
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_pop_engine_state(3'd4, 40_000, "B079 DRAIN entry");
      saw_overlap = 1'b0;
      search_cycles = 0;
      while (search_cycles < 4_096 && !saw_overlap) begin
        if ((m_env.m_dbg_mon.pop_partition_pending & 4'b0011) == 4'b0011) begin
          saw_overlap = 1'b1;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_overlap) begin
        `uvm_error("B079", $sformatf(
          "Partition+1 drain never exposed both partitions pending: pending=0x%0h rr_idx=%0d issue_idx=%0d",
          m_env.m_dbg_mon.pop_partition_pending,
          m_env.m_dbg_mon.pop_rr_idx,
          m_env.m_dbg_mon.pop_issue_partition_idx))
      end
      wait_for_hit_output_count(hit_before + partition_size + 1, 3_000_000, "B079 partition+1 hit drain");
      wait_for_scoreboard_idle(200_000, "B079 partition+1 drain");
      expect_service_model_accounting("B079 partition+1 drain", 1, 0);
    end else if (case_id == "B080") begin
      configure_and_start(0);
      if (m_cfg.n_partitions == 2)
        prefill_hits = partition_size / 2;
      else
        prefill_hits = 0;
      if (prefill_hits != 0) begin
        burst_seq = same_key_burst_seq::type_id::create("b080_prefill");
        burst_seq.num_hits = prefill_hits;
        burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
        burst_seq.start(m_env.m_hit_seqr);
        wait_for_scoreboard_idle(2_500_000, "B080 pointer advance prefill");
        expect_service_model_accounting("B080 pointer advance prefill", 1, 0);
      end
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd2000);
      wait_clocks(4);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq = same_key_burst_seq::type_id::create("b080_half_ring");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry / 2;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(prefill_hits + (m_cfg.ring_buffer_n_entry / 2), 80_000, "B080 half-ring fill");
      if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < (m_cfg.ring_buffer_n_entry / 2)) begin
        `uvm_error("B080", $sformatf(
          "Half-ring case never accumulated the target occupancy: key=%0d max_for_key=%0d expected=%0d",
          focus_search_key,
          m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
          m_cfg.ring_buffer_n_entry / 2))
      end
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_pop_engine_state(3'd4, 40_000, "B080 DRAIN entry");
      load_mask = '0;
      wait_for_partition_issue_mask(4'b0011, load_mask, 240_000, "B080 two-full-partition issue mask");
      saw_rr_advance = 1'b0;
      search_cycles = 0;
      while (search_cycles < 4_096 && !saw_rr_advance) begin
        if (m_env.m_dbg_mon.pop_engine_state_code == 3'd4 &&
            m_env.m_dbg_mon.pop_rr_idx == 2'd1) begin
          saw_rr_advance = 1'b1;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_rr_advance) begin
        `uvm_error("B080", "Two-partition equal-load drain never advanced pop_rr_idx from partition 0 to 1")
      end
      wait_for_hit_output_count(hit_before + (m_cfg.ring_buffer_n_entry / 2), 3_000_000, "B080 half-ring hit drain");
      wait_for_scoreboard_idle(260_000, "B080 half-ring drain");
      expect_service_model_accounting("B080 half-ring drain", 1, 0);
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
    end else if (case_id == "B092") begin
      overwrite_profile_seq pressure_seq_local;
      bit saw_active_erase;
      bit saw_drain_activity;
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      pressure_seq_local = overwrite_profile_seq::type_id::create("b092_overlap_stream");
      pressure_seq_local.num_hits = m_cfg.ring_buffer_n_entry + 1024;
      pressure_seq_local.lane_key_start_ord = 2;
      pressure_seq_local.pool_keys = 4;
      pressure_seq_local.hits_per_key_switch = 1;
      saw_overlap = 1'b0;
      saw_active_erase = 1'b0;
      saw_drain_activity = 1'b0;
      traffic_done = 1'b0;
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd768);
      wait_clocks(4);
      fork
        begin
          pressure_seq_local.start(m_env.m_hit_seqr);
          traffic_done = 1'b1;
        end
      join_none
      wait_for_push_count(m_cfg.ring_buffer_n_entry + 16, 80_000, "B092 overwrite precondition");
      if (m_env.m_dbg_mon.dbg_overwrite_cnt[31:0] == 0) begin
        `uvm_error("B092", "Precondition failed: overwrite stream never became active before DRAIN")
      end else begin
        saw_active_erase = 1'b1;
      end
      wait_for_pop_engine_state(3'd4, 80_000, "B092 DRAIN entry");
      search_cycles = 0;
      while (search_cycles < 20_000) begin
        if (m_env.m_dbg_mon.pop_engine_state_code == 3'd4 &&
            m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
          saw_drain_activity = 1'b1;
        end
        if (saw_drain_activity &&
            m_env.m_dbg_mon.pop_engine_state_code == 3'd4 &&
            m_env.m_dbg_mon.vif.push_erase_grant === 1'b1) begin
          saw_overlap = 1'b1;
          `uvm_error("B092", $sformatf(
            "push_erase_grant leaked into active DRAIN after pop_erase ownership started: push_erase_count=%0d pop_erase_count=%0d pop_hits=%0d overwrites=%0d",
            m_env.m_dbg_mon.push_erase_grant_count,
            m_env.m_dbg_mon.pop_erase_grant_count,
            m_env.m_dbg_mon.vif.pop_hits_count,
            m_env.m_dbg_mon.dbg_overwrite_cnt[31:0]))
          break;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      wait (traffic_done == 1'b1);
      if (!saw_drain_activity) begin
        `uvm_error("B092", $sformatf(
          "Did not observe active DRAIN pop ownership after overwrite pressure: traffic_done=%0d active_erase=%0d push_erase_count=%0d pop_erase_count=%0d pop_hits=%0d overwrites=%0d",
          traffic_done,
          saw_active_erase,
          m_env.m_dbg_mon.push_erase_grant_count,
          m_env.m_dbg_mon.pop_erase_grant_count,
          m_env.m_dbg_mon.vif.pop_hits_count,
          m_env.m_dbg_mon.dbg_overwrite_cnt[31:0]))
      end
      terminate_and_drain(240_000, "B092 push-erase priority drain");
      expect_service_model_accounting("B092 push-erase priority drain", 1, 1);
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
    end else if (case_id == "B094") begin
      run_pipe_stage_latency_smoke_case(
        "B094 PIPE_STAGES=1 latency baseline",
        "B094",
        1,
        2,
        80_000);
    end else if (case_id == "B095") begin
      run_pipe_stage_latency_smoke_case(
        "B095 PIPE_STAGES=2 latency baseline",
        "B095",
        2,
        2,
        80_000);
    end else if (case_id == "B096") begin
      run_pipe_stage_latency_smoke_case(
        "B096 PIPE_STAGES=3 latency baseline",
        "B096",
        3,
        2,
        80_000);
    end else if (case_id == "B097") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b097_pipe4_latency");
      burst_seq.num_hits = 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "B097 single-hit fill");
      cycle_a = 0;
      cycle_b = 0;
      saw_stage0 = 1'b0;
      search_cycles = 0;
      while (search_cycles < 8_192 && cycle_b == 0) begin
        if (cycle_a == 0 &&
            m_env.m_dbg_mon.pop_partition_eval_stage0_valid[0] === 1'b1) begin
          cycle_a = m_env.m_dbg_mon.sampled_cycles;
          saw_stage0 = 1'b1;
        end
        if (m_env.m_dbg_mon.pop_partition_eval_stage0_valid[0] === 1'b1) begin
          saw_stage0 = 1'b1;
        end
        if (cycle_a != 0 &&
            m_env.m_dbg_mon.pop_partition_result_valid[0] === 1'b1 &&
            m_env.m_dbg_mon.pop_partition_flag[0] === 1'b1) begin
          cycle_b = m_env.m_dbg_mon.sampled_cycles;
          if ((m_env.m_dbg_mon.pop_partition_result_valid & 4'b1110) != 0) begin
            `uvm_error("B097", $sformatf(
              "Single-hit PIPE_STAGES=4 result_valid leaked into inactive partitions: result_valid=0x%0h",
              m_env.m_dbg_mon.pop_partition_result_valid))
          end
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (cycle_a == 0) begin
        `uvm_error("B097", "PIPE_STAGES=4 case never observed the non-zero stage0 eval bitmap")
      end
      if (cycle_b == 0) begin
        `uvm_error("B097", "PIPE_STAGES=4 case never observed a flagged result_valid pulse")
      end else begin
        wait_min = int'(cycle_b - cycle_a);
        if (wait_min != int'(m_cfg.encoder_pipe_stages - 1)) begin
          `uvm_error("B097", $sformatf(
            "PIPE_STAGES=4 result latency mismatch: observed=%0d expected=%0d load_cycle=%0d result_cycle=%0d",
            wait_min, m_cfg.encoder_pipe_stages - 1, cycle_a, cycle_b))
        end
      end
      if (!saw_stage0) begin
        `uvm_error("B097", "PIPE_STAGES=4 case never observed the stage0 eval bitmap")
      end
      wait_for_scoreboard_idle(80_000, "B097 pipe4 latency drain");
    end else if (case_id == "B098") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq = same_key_burst_seq::type_id::create("b098_one_partition_flag");
      burst_seq.num_hits = partition_size;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(partition_size, 40_000, "B098 partition-local fill");
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_pop_engine_state(3'd4, 40_000, "B098 DRAIN entry");
      saw_load = 1'b0;
      grant_count = 0;
      search_cycles = 0;
      while (search_cycles < 240_000 &&
             m_env.m_out_mon.total_hits_seen < (hit_before + partition_size)) begin
        if ((m_env.m_dbg_mon.pop_partition_flag & 4'b1110) != 0) begin
          `uvm_error("B098", $sformatf(
            "Inactive partition flagged a match: flags=0x%0h pending=0x%0h",
            m_env.m_dbg_mon.pop_partition_flag,
            m_env.m_dbg_mon.pop_partition_pending))
        end
        if (m_env.m_dbg_mon.pop_partition_flag[0] === 1'b1)
          saw_load = 1'b1;
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
          visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
          if (visit_idx != 0) begin
            `uvm_error("B098", $sformatf(
              "Exactly-one-partition case issued from unexpected partition=%0d addr=%0d",
              visit_idx, m_env.m_dbg_mon.vif.pop_issue_addr))
          end
          grant_count++;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_load) begin
        `uvm_error("B098", "Matching partition never asserted pop_partition_flag")
      end
      wait_for_hit_output_count(hit_before + partition_size, 3_000_000, "B098 one-partition hit drain");
      wait_for_scoreboard_idle(200_000, "B098 one-partition flag drain");
      expect_service_model_accounting("B098 one-partition flag drain", 1, 0);
    end else if (case_id == "B099") begin
      if (m_cfg.n_partitions != 2) begin
        `uvm_fatal("B099", "B099 currently targets the exactly-two-active-partition build")
      end
      configure_and_start(0);
      prefill_hits = partition_size - 4;
      burst_seq = same_key_burst_seq::type_id::create("b099_prefill");
      burst_seq.num_hits = prefill_hits;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(2_500_000, "B099 pointer advance prefill");
      expect_service_model_accounting("B099 pointer advance prefill", 1, 0);
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd2000);
      wait_clocks(4);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq = same_key_burst_seq::type_id::create("b099_two_partition_equal_load");
      burst_seq.num_hits = 8;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(prefill_hits + 8, 40_000, "B099 equal-load fill");
      if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < 8) begin
        `uvm_error("B099", $sformatf(
          "Exactly-two-partitions case never accumulated the target occupancy: key=%0d max_for_key=%0d expected=%0d",
          focus_search_key,
          m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
          8))
      end
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_pop_engine_state(3'd4, 40_000, "B099 DRAIN entry");
      load_mask = '0;
      grant_count = 0;
      search_cycles = 0;
      while (search_cycles < 40_000 && grant_count < 8) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
          visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
          if (visit_idx >= 2) begin
            `uvm_error("B099", $sformatf(
              "Exactly-two-partitions case issued from unexpected partition=%0d addr=%0d",
              visit_idx, m_env.m_dbg_mon.vif.pop_issue_addr))
          end else begin
            load_mask[visit_idx] = 1'b1;
            if (visit_idx != (grant_count % 2)) begin
              `uvm_error("B099", $sformatf(
                "Exactly-two-partitions equal-load drain lost 1:1 alternation: grant=%0d observed_partition=%0d expected_partition=%0d",
                grant_count + 1, visit_idx, (grant_count % 2)))
            end
          end
          grant_count++;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (grant_count != 8) begin
        `uvm_error("B099", $sformatf(
          "Did not observe all eight grants in the equal-load two-partition drain: grants=%0d",
          grant_count))
      end
      if ((load_mask & 4'b0011) != 4'b0011) begin
        `uvm_error("B099", $sformatf(
          "Exactly-two-partitions case did not visit both partitions: observed_mask=0x%0h",
          load_mask))
      end
      wait_for_hit_output_count(hit_before + 8, 120_000, "B099 equal-load hit drain");
      wait_for_scoreboard_idle(120_000, "B099 equal-load drain");
      expect_service_model_accounting("B099 equal-load drain", 1, 0);
      if (m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]) != 8) begin
        `uvm_error("B099", $sformatf(
          "Exactly-two-partitions case drained the wrong hit count: key=0x%02x drained=%0d expected=%0d",
          focus_search_key[7:0],
          m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]),
          8))
      end
    end else if (case_id == "B100") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b100_single_hot_partition");
      burst_seq.num_hits = 8;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd4, 40_000, "B100 DRAIN entry");
      grant_count = 0;
      search_cycles = 0;
      while (search_cycles < 8_192 && grant_count < 8) begin
        if (m_env.m_dbg_mon.pop_partition_pending[0] === 1'b1 &&
            m_env.m_dbg_mon.pop_rr_idx != 0) begin
          `uvm_error("B100", $sformatf(
            "pop_rr_idx left the active partition before exhaustion: rr_idx=%0d pending=0x%0h grants=%0d",
            m_env.m_dbg_mon.pop_rr_idx,
            m_env.m_dbg_mon.pop_partition_pending,
            grant_count))
        end
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
          visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
          if (visit_idx != 0) begin
            `uvm_error("B100", $sformatf(
              "Single-hot-partition case issued from unexpected partition=%0d addr=%0d",
              visit_idx, m_env.m_dbg_mon.vif.pop_issue_addr))
          end
          grant_count++;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (grant_count != 8) begin
        `uvm_error("B100", $sformatf(
          "Did not observe all eight grants from the hot partition: grants=%0d",
          grant_count))
      end
      wait_for_scoreboard_idle(120_000, "B100 single-hot-partition drain");
      expect_service_model_accounting("B100 single-hot-partition drain", 1, 0);
    end else if (case_id == "B101") begin
      configure_and_start(0);
      if (m_cfg.n_partitions < 2) begin
        `uvm_fatal("B101", "B101 requires at least 2 active partitions")
      end
      prefill_hits = partition_size - 1;
      burst_seq = same_key_burst_seq::type_id::create("b101_prefill");
      burst_seq.num_hits = prefill_hits;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(2_500_000, "B101 pointer advance prefill");
      expect_service_model_accounting("B101 pointer advance prefill", 1, 0);
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd2000);
      wait_clocks(4);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b101_active_partition_rotation");
      burst_seq.num_hits = m_cfg.n_partitions;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(prefill_hits + m_cfg.n_partitions, 20_000, "B101 active-partition fill");
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_pop_engine_state(3'd4, 40_000, "B101 DRAIN entry");
      expected_mask = (1 << m_cfg.n_partitions) - 1;
      load_mask = '0;
      grant_count = 0;
      search_cycles = 0;
      while (search_cycles < 80_000 && grant_count < m_cfg.n_partitions) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
          visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
          if (visit_idx >= m_cfg.n_partitions) begin
            `uvm_error("B101", $sformatf(
              "Issued from partition outside active build range: partition=%0d n_partitions=%0d",
              visit_idx, m_cfg.n_partitions))
          end else begin
            load_mask[visit_idx[1:0]] = 1'b1;
            if (visit_idx != grant_count) begin
              `uvm_error("B101", $sformatf(
                "Active-partition round-robin order mismatch: grant=%0d observed_partition=%0d expected_partition=%0d",
                grant_count + 1, visit_idx, grant_count))
            end
          end
          grant_count++;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if ((load_mask & expected_mask[3:0]) != expected_mask[3:0]) begin
        `uvm_error("B101", $sformatf(
          "Not all active partitions were visited in order: observed_mask=0x%0h expected=0x%0h",
          load_mask, expected_mask))
      end
      wait_for_scoreboard_idle(120_000, "B101 active-partition round-robin drain");
      expect_service_model_accounting("B101 active-partition round-robin drain", 1, 0);
    end else if (case_id == "B102") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b102_clear_onehot_bit");
      burst_seq.num_hits = 3;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd4, 40_000, "B102 DRAIN entry");
      grant_count = 0;
      data_b = 32'hffff_ffff;
      search_cycles = 0;
      while (search_cycles < 8_192 && grant_count < 3) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
          visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
          if (visit_idx != 0) begin
            `uvm_error("B102", $sformatf(
              "clear_onehot_bit case escaped the active partition: partition=%0d addr=%0d",
              visit_idx, m_env.m_dbg_mon.vif.pop_issue_addr))
          end
          if (grant_count > 0 &&
              m_env.m_dbg_mon.vif.pop_issue_addr == data_b) begin
            `uvm_error("B102", $sformatf(
              "clear_onehot_bit replayed the same address on consecutive grants: addr=%0d grant=%0d",
              m_env.m_dbg_mon.vif.pop_issue_addr, grant_count + 1))
          end
          data_b = m_env.m_dbg_mon.vif.pop_issue_addr;
          grant_count++;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (grant_count != 3) begin
        `uvm_error("B102", $sformatf(
          "Did not observe three distinct grants for clear_onehot_bit audit: grants=%0d",
          grant_count))
      end
      wait_for_scoreboard_idle(120_000, "B102 clear_onehot_bit drain");
      expect_service_model_accounting("B102 clear_onehot_bit drain", 1, 0);
    end else if (case_id == "B103") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b103_has_more_transition");
      burst_seq.num_hits = 2;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd4, 40_000, "B103 DRAIN entry");
      saw_rr_step1 = 1'b0;
      saw_rr_step2 = 1'b0;
      search_cycles = 0;
      while (search_cycles < 8_192 && !saw_rr_step2) begin
        if (m_env.m_dbg_mon.pop_partition_result_valid[0] === 1'b1 &&
            m_env.m_dbg_mon.pop_partition_flag[0] === 1'b1) begin
          if (!saw_rr_step1 &&
              m_env.m_dbg_mon.pop_partition_has_more[0] === 1'b1) begin
            saw_rr_step1 = 1'b1;
          end else if (saw_rr_step1 &&
                       m_env.m_dbg_mon.pop_partition_has_more[0] === 1'b0) begin
            saw_rr_step2 = 1'b1;
          end
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!(saw_rr_step1 && saw_rr_step2)) begin
        `uvm_error("B103", $sformatf(
          "Did not observe has_more transition 1->0 on the final match: saw_has_more=%0d saw_last=%0d",
          saw_rr_step1, saw_rr_step2))
      end
      wait_for_scoreboard_idle(120_000, "B103 has_more transition drain");
      expect_service_model_accounting("B103 has_more transition drain", 1, 0);
    end else if (case_id == "B104") begin
      int unsigned issue_addr_q[$];
      int unsigned first_search_key;
      int unsigned second_search_key;
      bit          saw_load_override;
      bit          saw_first_subheader;
      bit          saw_second_subheader;

      configure_and_start(0);
      first_search_key = m_cfg.lane_key_ord_to_search_key(2);
      second_search_key = m_cfg.lane_key_ord_to_search_key(3);

      burst_seq = same_key_burst_seq::type_id::create("b104_first_vector");
      burst_seq.num_hits = 2;
      burst_seq.search_key = first_search_key;
      burst_seq.start(m_env.m_hit_seqr);

      burst_seq = same_key_burst_seq::type_id::create("b104_second_vector");
      burst_seq.num_hits = 1;
      burst_seq.search_key = second_search_key;
      burst_seq.start(m_env.m_hit_seqr);

      wait_for_push_count(3, 20_000, "B104 dual-vector fill");
      saw_load_override = 1'b0;
      saw_first_subheader = 1'b0;
      saw_second_subheader = 1'b0;
      search_cycles = 0;
      while (search_cycles < 120_000 &&
             (issue_addr_q.size() < 3 || !(saw_first_subheader && saw_second_subheader))) begin
        if (m_env.m_dbg_mon.pop_partition_load != 0 &&
            (m_env.m_dbg_mon.pop_partition_has_more != 0 ||
             m_env.m_dbg_mon.pop_partition_result_valid != 0)) begin
          saw_load_override = 1'b1;
        end
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
          issue_addr_q.push_back(m_env.m_dbg_mon.vif.pop_issue_addr);
        end
        if (m_env.m_out_mon.recent_data_subheaders.size() != 0) begin
          if (m_env.m_out_mon.recent_data_subheaders[$].search_key == first_search_key[7:0] &&
              m_env.m_out_mon.recent_data_subheaders[$].hit_count == 2) begin
            saw_first_subheader = 1'b1;
          end
          if (m_env.m_out_mon.recent_data_subheaders[$].search_key == second_search_key[7:0] &&
              m_env.m_out_mon.recent_data_subheaders[$].hit_count == 1) begin
            saw_second_subheader = 1'b1;
          end
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end

      if (!saw_load_override) begin
        `uvm_error("B104", "Did not observe a fresh pop_partition_load overriding an active encoder context")
      end
      if (issue_addr_q.size() != 3) begin
        `uvm_error("B104", $sformatf(
          "Expected three pop issues across the load-override audit, observed=%0d",
          issue_addr_q.size()))
      end else if (issue_addr_q[0] != 0 || issue_addr_q[1] != 1 || issue_addr_q[2] != 2) begin
        `uvm_error("B104", $sformatf(
          "Load-override issue order mismatch: observed={%0d,%0d,%0d} expected={0,1,2}",
          issue_addr_q[0], issue_addr_q[1], issue_addr_q[2]))
      end
      if (!saw_first_subheader || !saw_second_subheader) begin
        `uvm_error("B104", $sformatf(
          "Did not observe both post-load subheaders cleanly: first=%0d second=%0d",
          saw_first_subheader, saw_second_subheader))
      end
      wait_for_scoreboard_idle(120_000, "B104 load-override drain");
      expect_service_model_accounting("B104 load-override drain", 1, 0);
    end else if (case_id == "B105") begin
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_run_state(4'd1, 2_000, "B105 RUN_PREPARE entry");
      wait_for_pop_engine_state(3'd6, 40_000, "B105 FLUSHING entry");
      prev_flush_cam_addr = m_env.m_dbg_mon.flush_cam_wraddr[8:0];
      prev_flush_cam_data = m_env.m_dbg_mon.flush_cam_wrdata[7:0];
      saw_load = 1'b0;
      search_cycles = 0;
      while (search_cycles < 300_000 &&
             m_env.m_dbg_mon.pop_flush_cam_done !== 1'b1) begin
        if (m_env.m_dbg_mon.pop_flush_grant === 1'b1 &&
            m_env.m_dbg_mon.pop_flush_cam_done !== 1'b1) begin
          if (m_env.m_dbg_mon.flush_cam_wraddr[8:0] != prev_flush_cam_addr[8:0]) begin
            saw_load = 1'b1;
            if (prev_flush_cam_data[7:0] != 8'hff) begin
              `uvm_error("B105", $sformatf(
                "flush_cam_wraddr advanced before flush_cam_wrdata rolled over: prev_addr=%0d prev_data=0x%02x curr_addr=%0d curr_data=0x%02x",
                prev_flush_cam_addr[8:0], prev_flush_cam_data[7:0],
                m_env.m_dbg_mon.flush_cam_wraddr[8:0],
                m_env.m_dbg_mon.flush_cam_wrdata[7:0]))
            end
          end
          prev_flush_cam_addr = m_env.m_dbg_mon.flush_cam_wraddr[8:0];
          prev_flush_cam_data = m_env.m_dbg_mon.flush_cam_wrdata[7:0];
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_load) begin
        `uvm_error("B105", "flush_cam_wraddr never advanced during FLUSHING")
      end
      if (m_env.m_dbg_mon.pop_flush_cam_done !== 1'b1 ||
          prev_flush_cam_addr[8:0] != 9'h1ff ||
          m_env.m_dbg_mon.flush_cam_wraddr[8:0] != 9'h000) begin
        `uvm_error("B105", $sformatf(
          "FLUSHING cam walk did not roll over on the terminal address: done=%0d prev_addr=%0d curr_addr=%0d",
          m_env.m_dbg_mon.pop_flush_cam_done,
          prev_flush_cam_addr[8:0],
          m_env.m_dbg_mon.flush_cam_wraddr[8:0]))
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_ctrl_ready(1'b1, 400_000, "B105 PREP ready");
    end else if (case_id == "B106") begin
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_run_state(4'd1, 2_000, "B106 RUN_PREPARE entry");
      wait_for_pop_engine_state(3'd6, 40_000, "B106 FLUSHING entry");
      prev_flush_ram_addr = m_env.m_dbg_mon.flush_ram_wraddr[8:0];
      saw_load = 1'b0;
      search_cycles = 0;
      while (search_cycles < 300_000 &&
             m_env.m_dbg_mon.pop_flush_ram_done !== 1'b1) begin
        if (m_env.m_dbg_mon.pop_flush_grant === 1'b1 &&
            m_env.m_dbg_mon.pop_flush_ram_done !== 1'b1) begin
          if (m_env.m_dbg_mon.flush_ram_wraddr[8:0] != prev_flush_ram_addr[8:0]) begin
            saw_load = 1'b1;
            if (m_env.m_dbg_mon.flush_ram_wraddr[8:0] != (prev_flush_ram_addr + 1)) begin
              `uvm_error("B106", $sformatf(
                "flush_ram_wraddr skipped or repeated during FLUSHING: prev=%0d curr=%0d",
                prev_flush_ram_addr,
                m_env.m_dbg_mon.flush_ram_wraddr[8:0]))
            end
          end
          prev_flush_ram_addr = m_env.m_dbg_mon.flush_ram_wraddr[8:0];
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_load) begin
        `uvm_error("B106", "flush_ram_wraddr never advanced during FLUSHING")
      end
      if (m_env.m_dbg_mon.pop_flush_ram_done !== 1'b1 ||
          prev_flush_ram_addr[8:0] != 9'h1ff ||
          m_env.m_dbg_mon.flush_ram_wraddr[8:0] != 9'h000) begin
        `uvm_error("B106", $sformatf(
          "FLUSHING ram walk did not roll over on the terminal address: done=%0d prev_addr=%0d curr_addr=%0d",
          m_env.m_dbg_mon.pop_flush_ram_done,
          prev_flush_ram_addr[8:0],
          m_env.m_dbg_mon.flush_ram_wraddr[8:0]))
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_ctrl_ready(1'b1, 400_000, "B106 PREP ready");
    end else if (case_id == "B107") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b107_flushing_rst");
      burst_seq.num_hits = 8;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd4, 40_000, "B107 DRAIN entry");
      while (m_env.m_dbg_mon.vif.pop_erase_grant !== 1'b1) begin
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_pop_engine_state(3'd6, 40_000, "B107 FLUSHING entry");
      m_env.m_scb.note_flush_reset();
      wait_for_pop_engine_state(3'd7, 400_000, "B107 FLUSHING_RST entry");
      wait_for_pop_engine_state(3'd0, 400_000, "B107 IDLE return");
      if (m_env.m_dbg_mon.vif.pop_total_hits != 0 ||
          m_env.m_dbg_mon.pop_hits_count != 0 ||
          m_env.m_dbg_mon.pop_partition_pending != 0 ||
          m_env.m_dbg_mon.pop_partition_has_more != 0 ||
          m_env.m_dbg_mon.vif.pop_pipeline_start != 1'b0) begin
        `uvm_error("B107", $sformatf(
          "FLUSHING_RST left stale reset-owned pop-engine context: total_hits=%0d hits=%0d pending=0x%0h has_more=0x%0h pipeline_start=%0d flag=0x%0h result_valid=0x%0h",
          m_env.m_dbg_mon.vif.pop_total_hits,
          m_env.m_dbg_mon.pop_hits_count,
          m_env.m_dbg_mon.pop_partition_pending,
          m_env.m_dbg_mon.pop_partition_has_more,
          m_env.m_dbg_mon.vif.pop_pipeline_start,
          m_env.m_dbg_mon.pop_partition_flag,
          m_env.m_dbg_mon.pop_partition_result_valid))
      end
      wait_for_ctrl_ready(1'b1, 400_000, "B107 PREP ready");
      wait_for_scoreboard_idle(120_000, "B107 FLUSHING_RST cleanup");
    end else if (case_id == "B108") begin
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_run_state(4'd1, 2_000, "B108 RUN_PREPARE entry");
      saw_ram_done = 1'b0;
      saw_cam_done = 1'b0;
      saw_done_pulse = 1'b0;
      search_cycles = 0;
      while (search_cycles < 400_000 && !saw_done_pulse) begin
        if (m_env.m_dbg_mon.pop_flush_ram_done === 1'b1)
          saw_ram_done = 1'b1;
        if (m_env.m_dbg_mon.pop_flush_cam_done === 1'b1)
          saw_cam_done = 1'b1;
        if (saw_ram_done && saw_cam_done) begin
          @(posedge m_env.m_csr_drv.vif.clk);
          search_cycles++;
          if (m_env.m_dbg_mon.run_mgmt_flush_memory_done !== 1'b1) begin
            `uvm_error("B108", $sformatf(
              "run_mgmt_flush_memory_done did not assert one cycle after both flush completions: ram_done=%0d cam_done=%0d done=%0d",
              m_env.m_dbg_mon.pop_flush_ram_done,
              m_env.m_dbg_mon.pop_flush_cam_done,
              m_env.m_dbg_mon.run_mgmt_flush_memory_done))
          end else begin
            saw_done_pulse = 1'b1;
          end
          break;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!(saw_ram_done && saw_cam_done && saw_done_pulse)) begin
        `uvm_error("B108", $sformatf(
          "Flush completion handshake incomplete after %0d cycles: ram_done=%0d cam_done=%0d done=%0d",
          search_cycles, saw_ram_done, saw_cam_done, saw_done_pulse))
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_ctrl_ready(1'b1, 400_000, "B108 PREP ready after done pulse");
    end else if (case_id == "B109") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      single_seq = single_push_pop_seq::type_id::create("b109_single");
      single_seq.search_key = focus_search_key;
      single_seq.hit_et1n6 = 9'h12a;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 80_000,
        "B109 one-hit data-bearing subheader", matched_subheader);
      wait_for_scoreboard_idle(80_000, "B109 one-hit drain");
      terminate_and_drain(80_000, "B109 end-to-end terminate");
      expect_service_model_accounting("B109 end-to-end terminate", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (push_count != 1 || pop_count != 1 || overwrite_count != 0) begin
        `uvm_error("B109", $sformatf(
          "One-hit end-to-end accounting mismatch: push=%0d pop=%0d overwrite=%0d",
          push_count, pop_count, overwrite_count))
      end
    end else if (case_id == "B110") begin
      configure_and_start(0);
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;
      seq_keys = sequential_keys_seq::type_id::create("b110_seq_keys");
      seq_keys.num_keys = 10;
      seq_keys.hits_per_key = 1;
      seq_keys.start_key = 2;
      seq_keys.start(m_env.m_hit_seqr);
      for (int k = 0; k < 10; k++) begin
        wait_for_subheader_match(
          m_cfg.lane_key_ord_to_search_key(2 + k)[7:0], 8'd1, 1'b1, 1'b1, 120_000,
          $sformatf("B110 subheader %0d", k), matched_subheader);
        if (matched_subheader.raw_data[7:0] != 8'hF7) begin
          `uvm_error("B110", $sformatf(
            "Subheader %0d lost the K237 marker: search_key=0x%02x marker=0x%02x",
            k, matched_subheader.search_key, matched_subheader.raw_data[7:0]))
        end
      end
      wait_for_hit_output_count(hit_before + 10, 200_000, "B110 ten single-hit drains");
      wait_for_scoreboard_idle(200_000, "B110 mixed-epoch drain");
      if ((m_env.m_out_mon.total_data_subheaders_seen - subhdr_before) < 10) begin
        `uvm_error("B110", $sformatf(
          "Expected ten data-bearing subheaders, observed delta=%0d",
          m_env.m_out_mon.total_data_subheaders_seen - subhdr_before))
      end
      expect_service_model_accounting("B110 mixed-epoch drain", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      if (push_count != 10 || pop_count != 10) begin
        `uvm_error("B110", $sformatf(
          "Ten-key end-to-end accounting mismatch: push=%0d pop=%0d",
          push_count, pop_count))
      end
    end else if (case_id == "B111") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      single_seq = single_push_pop_seq::type_id::create("b111_first_latency");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "B111 first-latency push");
      cycle_a = m_env.m_dbg_mon.sampled_cycles;
      search_cycles = 0;
      while (search_cycles < 160_000 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("B111", "First-latency hit never emitted a pop descriptor")
      end
      search_cycles = int'(m_env.m_dbg_mon.sampled_cycles - cycle_a);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 160_000,
        "B111 first-latency subheader", matched_subheader);
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd1000);
      wait_clocks(4);
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'd1000) begin
        `uvm_error("B111", $sformatf(
          "Latency rewrite to 1000 did not reach the DUT: observed=%0d",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      single_seq = single_push_pop_seq::type_id::create("b111_second_latency");
      single_seq.search_key = m_cfg.lane_key_ord_to_search_key(4);
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(2, 10_000, "B111 second-latency push");
      cycle_b = m_env.m_dbg_mon.sampled_cycles;
      wait_min = 0;
      while (wait_min < 120_000 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        wait_min++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("B111", "Second-latency hit never emitted a pop descriptor")
      end
      wait_min = int'(m_env.m_dbg_mon.sampled_cycles - cycle_b);
      if (wait_min >= search_cycles || (search_cycles - wait_min) < 200) begin
        `uvm_error("B111", $sformatf(
          "Latency ramp did not shorten the descriptor window enough: before=%0d after=%0d",
          search_cycles, wait_min))
      end
      wait_for_subheader_match(
        m_cfg.lane_key_ord_to_search_key(4)[7:0], 8'd1, 1'b1, 1'b1, 120_000,
        "B111 second-latency subheader", matched_subheader);
      wait_for_scoreboard_idle(120_000, "B111 latency-ramp drain");
      expect_service_model_accounting("B111 latency-ramp drain", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (push_count != 2 || pop_count != 2 || overwrite_count != 0) begin
        `uvm_error("B111", $sformatf(
          "Latency-ramp accounting mismatch: push=%0d pop=%0d overwrite=%0d",
          push_count, pop_count, overwrite_count))
      end
    end else if (case_id == "B112") begin
      configure_and_start(2000);
      err_burst = error_burst_seq::type_id::create("b112_filtered_half");
      err_burst.num_hits = 4;
      err_burst.search_key = m_cfg.lane_key_ord_to_search_key(2);
      err_burst.start(m_env.m_hit_seqr);
      wait_clocks(32);
      csr_expect_mask(CSR_INERR_COUNT_ADDR, 32'hFFFF_FFFF, 32'd4, "B112 filtered half increments INERR");
      csr_expect_mask(CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'd0, "B112 filtered half does not push");
      csr_write(CSR_CTRL_ADDR, 32'h0000_0001);
      wait_clocks(2);
      err_burst = error_burst_seq::type_id::create("b112_passthrough_half");
      err_burst.num_hits = 4;
      err_burst.search_key = m_cfg.lane_key_ord_to_search_key(4);
      err_burst.start(m_env.m_hit_seqr);
      wait_for_subheader_match(
        m_cfg.lane_key_ord_to_search_key(4)[7:0], 8'd4, 1'b1, 1'b1, 120_000,
        "B112 accepted error-half drain", matched_subheader);
      wait_for_scoreboard_idle(120_000, "B112 filter-toggle drain");
      expect_service_model_accounting("B112 filter-toggle drain", 1, 0);
      read_counter_u32(CSR_INERR_COUNT_ADDR, inerr_count);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (inerr_count != 4 || push_count != 4 || pop_count != 4 || fill_level != 0) begin
        `uvm_error("B112", $sformatf(
          "Filter-toggle end-to-end accounting mismatch: inerr=%0d push=%0d pop=%0d fill=%0d",
          inerr_count, push_count, pop_count, fill_level))
      end
    end else if (case_id == "B113") begin
      configure_and_start(2000);
      cycle_a = m_env.m_dbg_mon.sampled_cycles;
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
      send_endofrun_marker();
      wait_for_ctrl_ready(1'b1, 4_000, "B113 empty terminate ack");
      cycle_b = m_env.m_dbg_mon.sampled_cycles;
      if ((cycle_b - cycle_a) > 64) begin
        `uvm_error("B113", $sformatf(
          "Empty terminate ack took too long: observed=%0d cycles expected<=64",
          cycle_b - cycle_a))
      end
      wait_for_scoreboard_idle(40_000, "B113 empty terminate cleanup");
      expect_service_model_accounting("B113 empty terminate cleanup", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (push_count != 0 || pop_count != 0 || overwrite_count != 0) begin
        `uvm_error("B113", $sformatf(
          "Empty terminate changed counters: push=%0d pop=%0d overwrite=%0d",
          push_count, pop_count, overwrite_count))
      end
    end else if (case_id == "B114") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b114_pending_hits");
      burst_seq.num_hits = 16;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(16, 20_000, "B114 pending fill");
      cycle_a = m_env.m_dbg_mon.sampled_cycles;
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
      send_endofrun_marker();
      saw_term_accounting_block = 1'b0;
      search_cycles = 0;
      while (search_cycles < 120_000 &&
             m_env.m_dbg_mon.terminating_drain_done !== 1'b1) begin
        if (m_env.m_dbg_mon.vif.endofrun_seen === 1'b1 &&
            m_env.m_dbg_mon.dbg_push_cnt[31:0] !=
              (m_env.m_dbg_mon.dbg_pop_cnt[31:0] + m_env.m_dbg_mon.dbg_overwrite_cnt[31:0])) begin
          saw_term_accounting_block = 1'b1;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.terminating_drain_done !== 1'b1) begin
        `uvm_error("B114", $sformatf(
          "Pending-hit terminate never completed: push=%0d pop=%0d overwrite=%0d search_cycles=%0d",
          m_env.m_dbg_mon.dbg_push_cnt[31:0],
          m_env.m_dbg_mon.dbg_pop_cnt[31:0],
          m_env.m_dbg_mon.dbg_overwrite_cnt[31:0],
          search_cycles))
      end
      if (!saw_term_accounting_block) begin
        `uvm_error("B114", "TERMINATING never had to wait on pending hit accounting before ack")
      end
      cycle_b = m_env.m_dbg_mon.sampled_cycles;
      if ((cycle_b - cycle_a) <= 4) begin
        `uvm_error("B114", $sformatf(
          "Pending-hit terminate ack was unexpectedly immediate: observed=%0d cycles",
          cycle_b - cycle_a))
      end
      wait_for_scoreboard_idle(120_000, "B114 pending-hit terminate cleanup");
      expect_service_model_accounting("B114 pending-hit terminate cleanup", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (push_count != 16 || pop_count != 16 || overwrite_count != 0) begin
        `uvm_error("B114", $sformatf(
          "Pending-hit terminate accounting mismatch: push=%0d pop=%0d overwrite=%0d",
          push_count, pop_count, overwrite_count))
      end
    end else if (case_id == "B115") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b115_first_pass");
      burst_seq.num_hits = 16;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      terminate_and_drain(120_000, "B115 first terminate");
      expect_service_model_accounting("B115 first terminate", 1, 0);
      return_to_idle();
      restart_after_flush(0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      if (push_count != 0 || pop_count != 0 || overwrite_count != 0 || cache_miss_count != 0) begin
        `uvm_error("B115", $sformatf(
          "Second RUN inherited stale counters after restart: push=%0d pop=%0d overwrite=%0d cache_miss=%0d",
          push_count, pop_count, overwrite_count, cache_miss_count))
      end
      burst_seq = same_key_burst_seq::type_id::create("b115_second_pass");
      burst_seq.num_hits = 16;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(4);
      burst_seq.start(m_env.m_hit_seqr);
      terminate_and_drain(120_000, "B115 second terminate");
      expect_service_model_accounting("B115 second terminate", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 16 || pop_count != 16 || fill_level != 0) begin
        `uvm_error("B115", $sformatf(
          "Second RUN accounting mismatch after restart: push=%0d pop=%0d fill=%0d",
          push_count, pop_count, fill_level))
      end
    end else if (case_id == "B116") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      for (int idx = 1; idx <= 16; idx++) begin
        single_seq = single_push_pop_seq::type_id::create($sformatf("b116_single_%0d", idx));
        single_seq.search_key = focus_search_key;
        single_seq.ts_low = idx[3:0];
        single_seq.start(m_env.m_hit_seqr);
        if ((idx % 4) == 0) begin
          wait_for_push_count(idx, 20_000, $sformatf("B116 push grant %0d", idx));
          wait_clocks(4);
          read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
          if (fill_level != idx) begin
            `uvm_error("B116", $sformatf(
              "FILL_LEVEL trajectory mismatch after %0d pushes: observed=%0d expected=%0d",
              idx, fill_level, idx))
          end
        end
      end
      terminate_and_drain(120_000, "B116 fill trajectory terminate");
      expect_service_model_accounting("B116 fill trajectory terminate", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 16 || pop_count != 16 || fill_level != 0) begin
        `uvm_error("B116", $sformatf(
          "End-to-end fill trajectory accounting mismatch: push=%0d pop=%0d fill=%0d",
          push_count, pop_count, fill_level))
      end
    end else if (case_id == "B117") begin
      configure_and_start(16);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(3);
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      single_seq = single_push_pop_seq::type_id::create("b117_single");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 80_000,
        "B117 first data-bearing subheader", matched_subheader);
      wait_clocks(64);
      if (m_env.m_out_mon.total_data_subheaders_seen != subhdr_before + 1) begin
        `uvm_error("B117", $sformatf(
          "Expected exactly one data-bearing subheader over one latency window: before=%0d after=%0d",
          subhdr_before, m_env.m_out_mon.total_data_subheaders_seen))
      end
      if (matched_subheader == null || matched_subheader.search_key != focus_search_key[7:0]) begin
        `uvm_error("B117", $sformatf(
          "Latency-window subheader search key mismatch: observed=%s expected=0x%02x",
          (matched_subheader == null) ? "none" : $sformatf("0x%02x", matched_subheader.search_key),
          focus_search_key[7:0]))
      end
      wait_for_scoreboard_idle(80_000, "B117 single-window drain");
      expect_service_model_accounting("B117 single-window drain", 1, 0);
    end else if (case_id == "B118") begin
      configure_and_start();
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;
      wait_for_subheader_count(subhdr_before + 1, 40_000, "B118 zero-hit subheader");
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      if (m_env.m_out_mon.recent_subheaders.size() == 0) begin
        `uvm_error("B118", "No subheader captured for zero-hit end-to-end check")
      end else begin
        if (!(m_env.m_out_mon.recent_subheaders[$].sop && m_env.m_out_mon.recent_subheaders[$].eop)) begin
          `uvm_error("B118", "Zero-hit end-to-end subheader did not assert SOP+EOP")
        end
        if (m_env.m_out_mon.recent_subheaders[$].hit_count != 0) begin
          `uvm_error("B118", $sformatf(
            "Zero-hit end-to-end subheader carried hit_count=%0d",
            m_env.m_out_mon.recent_subheaders[$].hit_count))
        end
      end
      if (m_env.m_out_mon.total_hits_seen != hit_before) begin
        `uvm_error("B118", $sformatf(
          "Zero-hit end-to-end flow unexpectedly emitted hit data: hits_before=%0d hits_after=%0d",
          hit_before, m_env.m_out_mon.total_hits_seen))
      end
      if (cache_miss_count != 0) begin
        `uvm_error("B118", $sformatf(
          "Zero-hit end-to-end flow unexpectedly bumped CACHE_MISS_COUNT=%0d",
          cache_miss_count))
      end
    end else if (case_id == "B119") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(4);
      burst_seq = same_key_burst_seq::type_id::create("b119_plateau");
      burst_seq.num_hits = 128;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(128, 40_000, "B119 plateau fill");
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, push_count);
      wait_clocks(64);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, pop_count);
      if (overwrite_count != 0) begin
        `uvm_error("B119", $sformatf(
          "Plateau case unexpectedly overwrote entries: overwrite=%0d",
          overwrite_count))
      end
      if (push_count == 0 || pop_count != push_count) begin
        `uvm_error("B119", $sformatf(
          "Non-zero fill plateau was not stable before service: fill_a=%0d fill_b=%0d",
          push_count, pop_count))
      end
      terminate_and_drain(160_000, "B119 plateau terminate");
      expect_service_model_accounting("B119 plateau terminate", 1, 0);
    end else if (case_id == "B120") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(5);
      for (int idx = 0; idx < 8; idx++) begin
        single_seq = single_push_pop_seq::type_id::create($sformatf("b120_single_%0d", idx));
        single_seq.search_key = focus_search_key;
        single_seq.ts_low = idx[3:0];
        single_seq.start(m_env.m_hit_seqr);
        wait_for_scoreboard_idle(40_000, $sformatf("B120 immediate drain step %0d", idx));
      end
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 8 || pop_count != 8 || cache_miss_count != 0 || fill_level != 0) begin
        `uvm_error("B120", $sformatf(
          "Pop-dominant end-to-end accounting mismatch: push=%0d pop=%0d cache_miss=%0d fill=%0d",
          push_count, pop_count, cache_miss_count, fill_level))
      end
      expect_service_model_accounting("B120 immediate-drain accounting", 1, 0);
    end else if (case_id == "B121") begin
      configure_and_start(2000);
      eor_seq = endofrun_marker_seq::type_id::create("b121_wrong_lane_eor");
      eor_seq.lane_channel = (m_cfg.interleaving_index + 1) % m_cfg.interleaving_factor;
      eor_seq.start(m_env.m_hit_seqr);
      wait_clocks(8);
      if (m_env.m_dbg_mon.endofrun_seen !== 1'b0) begin
        `uvm_error("B121", "Non-matching empty marker incorrectly latched endofrun_seen")
      end
      send_endofrun_marker();
      wait_clocks(8);
      if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
        `uvm_error("B121", "Matching empty marker did not latch endofrun_seen")
      end
    end else if (case_id == "B122") begin
      configure_and_start(2000);
      terminate_and_drain(60_000, "B122 empty terminate");
      return_to_idle();
      wait_for_run_state(4'd0, 2_000, "B122 return to IDLE");
      grant_count = m_env.m_dbg_mon.pop_flush_grant_count;
      saw_load = 1'b0;
      ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      wait_for_run_state(4'd1, 2_000, "B122 second PREP entry");
      search_cycles = 0;
      while (search_cycles < 10_000) begin
        if (m_env.m_dbg_mon.run_mgmt_flush_memory_start === 1'b1 ||
            m_env.m_dbg_mon.pop_flush_grant_count > grant_count) begin
          saw_load = 1'b1;
          break;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_load) begin
        `uvm_error("B122", "Second PREP never reasserted the flush activity on a clean CAM")
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
      if (m_env.m_dbg_mon.run_mgmt_flushed !== 1'b1) begin
        `uvm_error("B122", "Second PREP did not settle to run_mgmt_flushed=1")
      end
    end else if (case_id == "B123") begin
      set_meta_sel(2'b00);
      csr_read(CSR_META_ADDR, data_a);
      set_meta_sel(2'b01);
      csr_read(CSR_META_ADDR, data_b);
      if (data_a != expected_meta_version() || data_b != 32'd20260421) begin
        `uvm_error("B123", $sformatf(
          "META walk mismatch in VERSION/DATE phase: version=0x%08x date=0x%08x",
          data_a, data_b))
      end
      set_meta_sel(2'b10);
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd0, "B123 META GIT readback");
      set_meta_sel(2'b11);
      csr_expect_mask(CSR_META_ADDR, 32'hFFFF_FFFF, 32'd0, "B123 META INSTANCE readback");
      if (data_a == data_b) begin
        `uvm_error("B123", "META walk returned identical VERSION and DATE values")
      end
    end else if (case_id == "B124") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b124_soft_reset_mid_drain");
      burst_seq.num_hits = 16;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_pop_engine_state(3'd4, 40_000, "B124 DRAIN entry");
      while (m_env.m_dbg_mon.vif.pop_erase_grant !== 1'b1) begin
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      csr_write(CSR_CTRL_ADDR, 32'h0000_0013);
      expect_soft_reset_abort(
        "B124 mid-drain soft_reset",
        32'h0000_0013,
        32'h0000_0011);
    end else if (case_id == "B125") begin
      configure_and_start(2000);
      for (int idx = 0; idx < 32; idx++) begin
        if (m_env.m_out_mon.vif.filllevel_valid !== 1'b1) begin
          `uvm_error("B125", $sformatf(
            "aso_filllevel_valid dropped low in idle sample %0d", idx))
        end
        if (m_env.m_out_mon.vif.filllevel_data !== m_env.m_dbg_mon.current_live_fill[15:0]) begin
          `uvm_error("B125", $sformatf(
            "Idle filllevel stream mismatch sample %0d: stream=%0d debug=%0d",
            idx, m_env.m_out_mon.vif.filllevel_data, m_env.m_dbg_mon.current_live_fill[15:0]))
        end
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      burst_seq = same_key_burst_seq::type_id::create("b125_filllevel_fill");
      burst_seq.num_hits = 8;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(8, 20_000, "B125 filllevel fill");
      wait_clocks(4);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (fill_level != 8) begin
        `uvm_error("B125", $sformatf(
          "B125 fill precondition mismatch: observed fill=%0d expected=8",
          fill_level))
      end
      for (int idx = 0; idx < 32; idx++) begin
        if (m_env.m_out_mon.vif.filllevel_valid !== 1'b1) begin
          `uvm_error("B125", $sformatf(
            "aso_filllevel_valid dropped low during non-zero fill sample %0d", idx))
        end
        if (m_env.m_out_mon.vif.filllevel_data !== fill_level[15:0]) begin
          `uvm_error("B125", $sformatf(
            "Non-zero filllevel stream mismatch sample %0d: stream=%0d expected=%0d",
            idx, m_env.m_out_mon.vif.filllevel_data, fill_level[15:0]))
        end
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      terminate_and_drain(120_000, "B125 filllevel terminate");
      expect_service_model_accounting("B125 filllevel terminate", 1, 0);
      for (int idx = 0; idx < 16; idx++) begin
        if (m_env.m_out_mon.vif.filllevel_valid !== 1'b1 ||
            m_env.m_out_mon.vif.filllevel_data !== 16'd0) begin
          `uvm_error("B125", $sformatf(
            "Post-drain filllevel stream mismatch sample %0d: valid=%0d data=%0d",
            idx, m_env.m_out_mon.vif.filllevel_valid, m_env.m_out_mon.vif.filllevel_data))
        end
        @(posedge m_env.m_csr_drv.vif.clk);
      end
    end else if (case_id == "B126") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      single_seq = single_push_pop_seq::type_id::create("b126_single");
      single_seq.search_key = focus_search_key;
      single_seq.ingress_channel = 4'h9;
      single_seq.hit_channel = 5'd17;
      single_seq.hit_asic = 4'ha;
      single_seq.hit_et1n6 = 9'h155;
      single_seq.start(m_env.m_hit_seqr);
      saw_subheader_channel = 1'b0;
      saw_hit_channel = 1'b0;
      search_cycles = 0;
      while (search_cycles < 80_000 && !(saw_subheader_channel && saw_hit_channel)) begin
        if (m_env.m_out_mon.vif.valid === 1'b1) begin
          if (m_env.m_out_mon.vif.channel !== m_cfg.interleaving_index[3:0]) begin
            `uvm_error("B126", $sformatf(
              "aso_hit_type2_channel mismatch: observed=%0d expected=%0d",
              m_env.m_out_mon.vif.channel, m_cfg.interleaving_index[3:0]))
          end
          if (m_env.m_out_mon.vif.data[35:32] == 4'b0001) begin
            saw_subheader_channel = 1'b1;
          end else begin
            saw_hit_channel = 1'b1;
            if (m_env.m_out_mon.vif.data[21:17] != 5'd17) begin
              `uvm_error("B126", $sformatf(
                "Hit payload channel mutated while egress bus channel stayed lane-indexed: payload=%0d expected=17",
                m_env.m_out_mon.vif.data[21:17]))
            end
          end
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!(saw_subheader_channel && saw_hit_channel)) begin
        `uvm_error("B126", $sformatf(
          "Did not observe both subheader and hit beats while auditing aso_hit_type2_channel: subheader=%0d hit=%0d",
          saw_subheader_channel, saw_hit_channel))
      end
      wait_for_scoreboard_idle(80_000, "B126 egress channel cleanup");
      expect_service_model_accounting("B126 egress channel cleanup", 1, 0);
    end else if (case_id == "B127") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      grant_count = m_env.m_dbg_mon.push_write_grant_count;
      single_seq = single_push_pop_seq::type_id::create("b127_single");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      search_cycles = 0;
      while (search_cycles < 10_000 &&
             m_env.m_dbg_mon.push_write_grant_count == grant_count) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.push_write_grant_count == grant_count) begin
        `uvm_error("B127", "Timed out waiting for the granted push_write event")
      end
      reads_needed = 0;
      push_count = 0;
      while (reads_needed < 3 && push_count < 1) begin
        reads_needed++;
        read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      end
      if (push_count != 1 || reads_needed > 2) begin
        `uvm_error("B127", $sformatf(
          "PUSH_COUNT visibility exceeded the 2-read bound: reads=%0d push_count=%0d",
          reads_needed, push_count))
      end
      terminate_and_drain(80_000, "B127 bounded-delay terminate");
      expect_service_model_accounting("B127 bounded-delay terminate", 1, 0);
    end else if (case_id == "B128") begin
      configure_and_start(2000);
      max_cmd_usedw = 0;
      traffic_done = 1'b0;
      pressure_seq = overwrite_profile_seq::type_id::create("b128_cmd_fifo_soak");
      pressure_seq.num_hits = 4096;
      pressure_seq.lane_key_start_ord = 2;
      pressure_seq.pool_keys = 8;
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.inter_hit_gap_cycles = 31;
      pressure_seq.progress_stride = 1024;
      pressure_seq.progress_tag = "B128";
      fork
        begin
          pressure_seq.start(m_env.m_hit_seqr);
          traffic_done = 1'b1;
        end
        begin
          search_cycles = 0;
          while (search_cycles < 400_000 &&
                 (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0 ||
                  m_env.m_dbg_mon.pop_cmd_fifo_usedw != 0)) begin
            if (m_env.m_dbg_mon.pop_cmd_fifo_usedw > max_cmd_usedw)
              max_cmd_usedw = m_env.m_dbg_mon.pop_cmd_fifo_usedw;
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
        end
      join
      if (max_cmd_usedw == 0) begin
        `uvm_error("B128", "pop_cmd_fifo_usedw never rose above 0 during the soak")
      end
      if (max_cmd_usedw > 11) begin
        `uvm_error("B128", $sformatf(
          "pop_cmd_fifo_usedw exceeded the bounded-soak ceiling: observed_peak=%0d expected<=11",
          max_cmd_usedw))
      end
      terminate_and_drain(250_000, "B128 pop_cmd soak terminate");
      expect_service_model_accounting("B128 pop_cmd soak terminate", 1, 0);
    end else if (case_id == "B129") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      saw_idle_bubble = 1'b0;
      traffic_done = 1'b0;
      burst_seq = same_key_burst_seq::type_id::create("b129_back_to_back_pushes");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry;
      burst_seq.search_key = focus_search_key;
      fork
        begin
          burst_seq.start(m_env.m_hit_seqr);
          traffic_done = 1'b1;
        end
        begin
          search_cycles = 0;
          while (search_cycles < 120_000 &&
                 (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0 ||
                  m_env.m_dbg_mon.deassembly_fifo_usedw != 0 ||
                  m_env.m_dbg_mon.push_write_grant_count < m_cfg.ring_buffer_n_entry)) begin
            if (m_env.m_dbg_mon.push_write_grant_count > 0 &&
                m_env.m_dbg_mon.push_write_grant_count < m_cfg.ring_buffer_n_entry &&
                m_env.m_dbg_mon.deassembly_fifo_usedw > 1 &&
                m_env.m_scb.max_remaining_entries() >= m_cfg.ring_buffer_n_entry &&
                m_env.m_dbg_mon.vif.decision_reg == 3'd4) begin
              saw_idle_bubble = 1'b1;
              break;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
        end
      join
      wait_for_push_count(m_cfg.ring_buffer_n_entry, 80_000, "B129 back-to-back push grants");
      if (saw_idle_bubble) begin
        `uvm_error("B129", "decision_reg=4 appeared between consecutive push_write grants under sustained pressure")
      end
      terminate_and_drain(250_000, "B129 no-bubble terminate");
      expect_service_model_accounting("B129 no-bubble terminate", 1, 0);
    end else if (case_id == "B130") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b130_drain_bubble_guard");
      burst_seq.num_hits = 512;
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
          while (search_cycles < 8_192 &&
                 (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
            if (m_env.m_dbg_mon.pop_engine_state_code == 3'd4 &&
                m_env.m_dbg_mon.vif.push_write_grant === 1'b1) begin
              saw_overlap = 1'b1;
              `uvm_error("B130", $sformatf(
                "push_write_grant entered DRAIN bubble at decision_reg=%0d pop_issue_addr=%0d",
                m_env.m_dbg_mon.vif.decision_reg,
                m_env.m_dbg_mon.vif.pop_issue_addr))
              break;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
        end
      join
      if (saw_overlap) begin
        `uvm_error("B130", "Observed push_write_grant while pop_engine_state stayed in DRAIN")
      end
      terminate_and_drain(250_000, "B130 no push-write in DRAIN");
      expect_service_model_accounting("B130 no push-write in DRAIN", 1, 0);
    end else if (case_id == "B131") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("b131_snapshot_freeze_guard");
      burst_seq.num_hits = 512;
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
          while (search_cycles < 8_192 &&
                 (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
            if ((m_env.m_dbg_mon.pop_engine_state_code inside {3'd2, 3'd3}) &&
                m_env.m_dbg_mon.vif.push_write_grant === 1'b1) begin
              saw_overlap = 1'b1;
              `uvm_error("B131", $sformatf(
                "push_write_grant overlapped a frozen pop snapshot: pop_state=%0d decision_reg=%0d",
                m_env.m_dbg_mon.pop_engine_state_code,
                m_env.m_dbg_mon.vif.decision_reg))
              break;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
        end
      join
      if (saw_overlap) begin
        `uvm_error("B131", "Observed push_write_grant while pop_engine_state stayed in LOAD/COUNT")
      end
      terminate_and_drain(250_000, "B131 no push-write in LOAD/COUNT");
      expect_service_model_accounting("B131 no push-write in LOAD/COUNT", 1, 0);
    end else if (case_id == "B132") begin
      profile_traffic_seq term_seq;
      int unsigned        push_at_endofrun;
      int unsigned        offered_at_endofrun;
      int unsigned        observe_cycles;
      bit                 saw_ready_high_after_endofrun;
      bit                 saw_push_after_endofrun;

      configure_and_start(16'd128);
      traffic_done = 1'b0;
      saw_ready_high_after_endofrun = 1'b0;
      saw_push_after_endofrun = 1'b0;
      fork
        begin
          term_seq = profile_traffic_seq::type_id::create("b132_term_ready_guard");
          term_seq.num_hits = 256;
          term_seq.lane_key_start_ord = 2;
          term_seq.pool_keys = 4;
          term_seq.hits_per_key_switch = 1;
          term_seq.inter_hit_gap_cycles = 13;
          term_seq.progress_stride = 128;
          term_seq.progress_tag = "B132 terminate ingress-ready guard";
          term_seq.start(m_env.m_hit_seqr);
          traffic_done = 1'b1;
        end
        begin
          wait_clocks(1_500);
          ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
          wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 20_000, "B132 TERMINATING entry");
          send_endofrun_marker();
          wait_clocks(2);
          if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
            `uvm_error("B132", "endofrun_seen did not latch after TERMINATING marker")
          end

          push_at_endofrun = m_env.m_dbg_mon.dbg_push_cnt[31:0];
          offered_at_endofrun = m_env.m_hit_drv.offered_payload_total;
          observe_cycles = 0;
          while (observe_cycles < 128 &&
                 (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
            @(posedge m_env.m_csr_drv.vif.clk);
            observe_cycles++;
            if (m_env.m_hit_drv.vif.ready === 1'b1) begin
              saw_ready_high_after_endofrun = 1'b1;
            end
            if (m_env.m_dbg_mon.dbg_push_cnt[31:0] != push_at_endofrun) begin
              saw_push_after_endofrun = 1'b1;
            end
          end
        end
      join

      if (m_env.m_hit_drv.offered_payload_total == offered_at_endofrun) begin
        `uvm_error("B132", $sformatf(
          "Terminate ingress-ready guard never observed additional offered traffic after endofrun_seen: offered_at_eor=%0d final_offered=%0d",
          offered_at_endofrun, m_env.m_hit_drv.offered_payload_total))
      end
      if (saw_ready_high_after_endofrun) begin
        `uvm_error("B132", "asi_hit_type1_ready stayed high after endofrun_seen latched in TERMINATING")
      end
      if (saw_push_after_endofrun) begin
        `uvm_error("B132", $sformatf(
          "PUSH_COUNT advanced after endofrun_seen latched: before=%0d after=%0d",
          push_at_endofrun, m_env.m_dbg_mon.dbg_push_cnt[31:0]))
      end
      ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
      m_env.m_hit_drv.pending_q.delete();
      wait_clocks(2);
      wait_for_scoreboard_idle(120_000, "B132 post-terminate cleanup");
      expect_service_model_accounting("B132 terminate ingress-ready guard", 1, 0);
    end else if (case_id == "B133") begin
      same_key_burst_seq focus_burst_seq;
      same_key_burst_seq cross_burst_seq;
      bit                saw_early_cross_key_overlap;
      bit                cross_injected;
      int unsigned       pushed_key;
      int unsigned       active_key;

      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      focus_burst_seq = same_key_burst_seq::type_id::create("b133_focus_burst");
      focus_burst_seq.num_hits = 16;
      focus_burst_seq.search_key = focus_search_key;
      focus_burst_seq.start(m_env.m_hit_seqr);

      wait_for_pop_engine_state(3'd1, 40_000, "B133 SEARCH entry");
      search_cycles = 0;
      while (search_cycles < 128 &&
             !(m_env.m_dbg_mon.pop_engine_state_code == 3'd1 &&
               m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0] &&
               m_env.m_dbg_mon.pop_search_wait_cnt <= 3'd1)) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!(m_env.m_dbg_mon.pop_engine_state_code == 3'd1 &&
            m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0] &&
            m_env.m_dbg_mon.pop_search_wait_cnt <= 3'd1)) begin
        `uvm_error("B133", $sformatf(
          "Did not observe the focus-key early SEARCH guard window before injection: pop_state=%0d active_key=%0d wait_cnt=%0d focus_key=%0d",
          m_env.m_dbg_mon.pop_engine_state_code,
          m_env.m_dbg_mon.vif.pop_current_sk[7:0],
          m_env.m_dbg_mon.pop_search_wait_cnt,
          focus_search_key[7:0]))
      end

      saw_early_cross_key_overlap = 1'b0;
      cross_injected = 1'b0;
      traffic_done = 1'b0;
      fork
        begin
          cross_burst_seq = same_key_burst_seq::type_id::create("b133_cross_burst");
          cross_burst_seq.num_hits = 8;
          cross_burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(3);
          cross_burst_seq.start(m_env.m_hit_seqr);
          cross_injected = 1'b1;
        end
        begin
          search_cycles = 0;
          while (search_cycles < 512 &&
                 (m_env.m_dbg_mon.pop_engine_state_code == 3'd1 ||
                  !cross_injected ||
                  m_env.m_hit_drv.pending_source_items() != 0)) begin
            if (m_env.m_dbg_mon.pop_engine_state_code == 3'd1 &&
                m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0] &&
                m_env.m_dbg_mon.pop_search_wait_cnt < 3'd5 &&
                m_env.m_dbg_mon.vif.push_write_grant === 1'b1) begin
              pushed_key = m_env.m_dbg_mon.vif.side_ram_din[
                             ring_buffer_cam_pkg::TCC8N_HI:ring_buffer_cam_pkg::TCC8N_LO] >> 4;
              active_key = m_env.m_dbg_mon.vif.pop_current_sk[7:0];
              if (pushed_key != active_key) begin
                saw_early_cross_key_overlap = 1'b1;
                `uvm_error("B133", $sformatf(
                  "Cross-key push_write_grant slipped into the unstable SEARCH window: pop_wait=%0d active_key=%0d pushed_key=%0d decision_reg=%0d",
                  m_env.m_dbg_mon.pop_search_wait_cnt,
                  active_key,
                  pushed_key,
                  m_env.m_dbg_mon.vif.decision_reg))
                break;
              end
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
          traffic_done = 1'b1;
        end
      join

      if (!cross_injected) begin
        `uvm_error("B133", "Cross-key SEARCH guard never injected the competing burst")
      end
      if (saw_early_cross_key_overlap) begin
        `uvm_error("B133", "Observed a cross-key push_write_grant before SEARCH match validity was guaranteed")
      end
      terminate_and_drain(120_000, "B133 early SEARCH cross-key guard");
      expect_service_model_accounting("B133 early SEARCH cross-key guard", 1, 0);
    end else if (case_id == "B134") begin
      same_key_burst_seq old_seq;
      same_key_burst_seq new_seq;
      int unsigned       wrap_slot;
      int unsigned       observed_erase_addr;
      int unsigned       watch_cycles;
      bit                saw_wrap_write;
      bit                saw_wrap_erase;
      bit                expect_wrap_erase;

      if (m_cfg.ring_buffer_n_entry < 2) begin
        `uvm_fatal("B134", $sformatf(
          "B134 requires ring_buffer_n_entry >= 2, observed=%0d",
          m_cfg.ring_buffer_n_entry))
      end
      if ((m_cfg.ring_buffer_n_entry & (m_cfg.ring_buffer_n_entry - 1)) == 0) begin
        `uvm_fatal("B134", $sformatf(
          "B134 targets the non-power-of-two wrap-overwrite path, but ring_buffer_n_entry=%0d is a power of two",
          m_cfg.ring_buffer_n_entry))
      end

      configure_and_start(16'hFFFF);
      wrap_slot = m_cfg.ring_buffer_n_entry - 1;

      old_seq = same_key_burst_seq::type_id::create("b134_first_lap_old_key");
      old_seq.num_hits = m_cfg.ring_buffer_n_entry;
      old_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      old_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry, 500_000, "B134 first-lap fill");

      saw_wrap_write = 1'b0;
      saw_wrap_erase = 1'b0;
      expect_wrap_erase = 1'b0;
      observed_erase_addr = 0;

      fork
        begin
          new_seq = same_key_burst_seq::type_id::create("b134_second_lap_new_key");
          new_seq.num_hits = m_cfg.ring_buffer_n_entry;
          new_seq.search_key = m_cfg.lane_key_ord_to_search_key(3);
          new_seq.start(m_env.m_hit_seqr);
        end
        begin
          watch_cycles = 0;
          while (watch_cycles < 2_000_000 && !saw_wrap_erase) begin
            if (m_env.m_dbg_mon.vif.push_write_grant === 1'b1 &&
                int'(m_env.m_dbg_mon.vif.cam_wr_addr) == wrap_slot) begin
              saw_wrap_write = 1'b1;
              expect_wrap_erase = 1'b1;
            end else if (expect_wrap_erase &&
                         m_env.m_dbg_mon.vif.push_erase_grant === 1'b1) begin
              saw_wrap_erase = 1'b1;
              observed_erase_addr = int'(m_env.m_dbg_mon.vif.cam_wr_addr);
              expect_wrap_erase = 1'b0;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            watch_cycles++;
          end
        end
      join

      wait_for_push_count(2 * m_cfg.ring_buffer_n_entry, 2_000_000, "B134 second-lap fill");
      if (!saw_wrap_write) begin
        `uvm_error("B134", $sformatf(
          "Did not observe the second-lap wrap write at slot%0d under non-power-of-two pressure",
          wrap_slot))
      end
      if (!saw_wrap_erase) begin
        `uvm_error("B134", "Did not observe the push_erase that should immediately follow the wrap write")
      end
      if (observed_erase_addr != wrap_slot) begin
        `uvm_error("B134", $sformatf(
          "Wrap-overwrite erased the wrong CAM slot: expected=%0d observed=%0d ring_depth=%0d",
          wrap_slot, observed_erase_addr, m_cfg.ring_buffer_n_entry))
      end

      m_env.m_scb.note_intentional_nonempty_end(m_cfg.ring_buffer_n_entry);
      return_to_idle();
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
    end else if (case_id == "E017") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e017_prefill_other");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry - 2;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry - 2, 80_000, "E017 near-wrap prefill");
      burst_seq = same_key_burst_seq::type_id::create("e017_target_wrap_slot510");
      burst_seq.num_hits = 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry - 1, 80_000, "E017 target at slot510");
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_subheader_search_key(
        focus_search_key[7:0], 120_000, "E017 target subheader", matched_subheader);
      wait_for_pop_engine_state(3'd4, 120_000, "E017 DRAIN entry");
      saw_push_511 = 1'b0;
      saw_push_000 = 1'b0;
      saw_pop_510 = 1'b0;
      saw_overlap = 1'b0;
      grant_count = 0;
      fork
        begin
          while (m_env.m_dbg_mon.pop_engine_state_code != 3'd4) begin
            @(posedge m_env.m_csr_drv.vif.clk);
          end
          burst_seq = same_key_burst_seq::type_id::create("e017_wrap_pushes");
          burst_seq.num_hits = 2;
          burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
          burst_seq.start(m_env.m_hit_seqr);
        end
        begin
          while (grant_count < 4 && !(saw_push_511 && saw_push_000)) begin
            if (m_env.m_dbg_mon.vif.push_write_grant === 1'b1) begin
              if (m_env.m_dbg_mon.vif.cam_wr_addr == (m_cfg.ring_buffer_n_entry - 1))
                saw_push_511 = 1'b1;
              if (m_env.m_dbg_mon.vif.cam_wr_addr == 16'h0000)
                saw_push_000 = 1'b1;
            end
            if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
              grant_count++;
              if (m_env.m_dbg_mon.vif.cam_wr_addr == (m_cfg.ring_buffer_n_entry - 2))
                saw_pop_510 = 1'b1;
              if (m_env.m_dbg_mon.vif.push_write_grant === 1'b1)
                saw_overlap = 1'b1;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
          end
        end
      join
      if (!saw_pop_510) begin
        `uvm_error("E017", "Target pop-erase at slot510 did not occur")
      end
      if (!saw_push_511 || !saw_push_000) begin
        `uvm_error("E017", $sformatf(
          "Did not observe full wrap-around push pressure during DRAIN: saw_push_511=%0d saw_push_000=%0d",
          saw_push_511, saw_push_000))
      end
      if (saw_overlap) begin
        `uvm_error("E017", "Arbitration selected non-pop path while pop_erase was active")
      end
      wait_for_hit_output_count(1, 120_000, "E017 target pop beat");
      wait_for_scoreboard_idle(200_000, "E017 wraparound conflict drain");
    end else if (case_id == "E018") begin
      int unsigned focus_issue_q[$];
      int unsigned focus_partition_q[$];

      if (m_cfg.n_partitions != 4) begin
        `uvm_fatal("E018", "E018 requires the 4-partition p4 build variant")
      end

      configure_and_start(16'hFFFF);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e018_prefill_wrap_boundary");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry - 2;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry - 2, 120_000, "E018 wrap prefill");

      burst_seq = same_key_burst_seq::type_id::create("e018_target_wrap_boundary");
      burst_seq.num_hits = 3;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry + 1, 120_000, "E018 wrap target fill");

      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd3, 1'b1, 1'b1, 300_000,
        "E018 wrap-boundary target subheader", matched_subheader);

      search_cycles = 0;
      while (search_cycles < 600_000 && focus_issue_q.size() < 3) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1 &&
            m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0]) begin
          focus_issue_q.push_back(m_env.m_dbg_mon.vif.pop_issue_addr);
          focus_partition_q.push_back(m_env.m_dbg_mon.vif.pop_issue_addr / partition_size);
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end

      if (focus_issue_q.size() != 3) begin
        `uvm_error("E018", $sformatf(
          "Did not observe all three focus-key pop issues across the wrap boundary: observed=%0d",
          focus_issue_q.size()))
      end else begin
        if (focus_partition_q[0] != 0 || focus_partition_q[1] != 3 || focus_partition_q[2] != 3) begin
          `uvm_error("E018", $sformatf(
            "Wrap-boundary partition order mismatch: observed={%0d,%0d,%0d} expected={0,3,3}",
            focus_partition_q[0], focus_partition_q[1], focus_partition_q[2]))
        end
        if (focus_issue_q[0] != 0 ||
            focus_issue_q[1] != (m_cfg.ring_buffer_n_entry - 2) ||
            focus_issue_q[2] != (m_cfg.ring_buffer_n_entry - 1)) begin
          `uvm_error("E018", $sformatf(
            "Wrap-boundary issue order mismatch: observed={%0d,%0d,%0d} expected={0,%0d,%0d}",
            focus_issue_q[0], focus_issue_q[1], focus_issue_q[2],
            m_cfg.ring_buffer_n_entry - 2, m_cfg.ring_buffer_n_entry - 1))
        end
      end

      wait_for_scoreboard_idle(1_500_000, "E018 wrap-boundary drain");
      expect_service_model_accounting("E018 wrap-boundary drain", 1, 1);
      if (m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]) != 3) begin
        `uvm_error("E018", $sformatf(
          "Wrap-boundary focus-key drain mismatch: key=0x%02x drained=%0d expected=%0d",
          focus_search_key[7:0],
          m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]), 3))
      end
    end else if (case_id == "E024") begin
      int unsigned focus_issue_addr;
      bit          saw_focus_issue;

      if (m_cfg.n_partitions != 4) begin
        `uvm_fatal("E024", "E024 requires the 4-partition p4 build variant")
      end

      configure_and_start(16'hFFFF);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e024_prefill_last_slot");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry - 1;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry - 1, 120_000, "E024 last-slot prefill");

      burst_seq = same_key_burst_seq::type_id::create("e024_target_last_slot");
      burst_seq.num_hits = 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry, 120_000, "E024 slot511 target fill");

      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 300_000,
        "E024 slot511 target subheader", matched_subheader);

      saw_focus_issue = 1'b0;
      focus_issue_addr = '0;
      search_cycles = 0;
      while (search_cycles < 300_000 && !saw_focus_issue) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1 &&
            m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0]) begin
          focus_issue_addr = m_env.m_dbg_mon.vif.pop_issue_addr;
          saw_focus_issue = 1'b1;
          if (m_env.m_dbg_mon.pop_issue_partition_idx != 2'd3) begin
            `uvm_error("E024", $sformatf(
              "Slot511 focus hit issued from the wrong partition: partition=%0d addr=%0d",
              m_env.m_dbg_mon.pop_issue_partition_idx, focus_issue_addr))
          end
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end

      if (!saw_focus_issue) begin
        `uvm_error("E024", "Did not observe the focus-key pop issue for the last-slot case")
      end else if (focus_issue_addr != (m_cfg.ring_buffer_n_entry - 1)) begin
        `uvm_error("E024", $sformatf(
          "Last-slot focus hit issued from the wrong address: observed=%0d expected=%0d",
          focus_issue_addr, m_cfg.ring_buffer_n_entry - 1))
      end

      wait_for_scoreboard_idle(1_500_000, "E024 last-slot drain");
      expect_service_model_accounting("E024 last-slot drain", 1, 0);
      if (m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]) != 1) begin
        `uvm_error("E024", $sformatf(
          "Last-slot focus-key drain mismatch: key=0x%02x drained=%0d expected=%0d",
          focus_search_key[7:0],
          m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]), 1))
      end
    end else if (case_id == "E027") begin
      bit [3:0] saw_zero_flag_mask;
      bit       saw_partition2_flag;
      bit [3:0] focus_issue_mask;

      if (m_cfg.n_partitions != 4) begin
        `uvm_fatal("E027", "E027 requires the 4-partition p4 build variant")
      end

      configure_and_start(16'hFFFF);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e027_prefill_partition2");
      burst_seq.num_hits = partition_size * 2;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(partition_size * 2, 120_000, "E027 partition2 prefill");

      burst_seq = same_key_burst_seq::type_id::create("e027_target_partition2");
      burst_seq.num_hits = 8;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count((partition_size * 2) + 8, 120_000, "E027 partition2 target fill");

      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd8, 1'b1, 1'b1, 300_000,
        "E027 partition2 target subheader", matched_subheader);

      saw_zero_flag_mask = '0;
      saw_partition2_flag = 1'b0;
      focus_issue_mask = '0;
      grant_count = 0;
      search_cycles = 0;
      while (search_cycles < 400_000 &&
             ((!saw_partition2_flag) ||
              ((saw_zero_flag_mask & 4'b1011) != 4'b1011) ||
              grant_count < 8)) begin
        if (m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0]) begin
          for (int idx = 0; idx < 4; idx++) begin
            if (m_env.m_dbg_mon.pop_partition_result_valid[idx] === 1'b1) begin
              if (idx == 2) begin
                if (m_env.m_dbg_mon.pop_partition_flag[idx] === 1'b1)
                  saw_partition2_flag = 1'b1;
              end else begin
                if (m_env.m_dbg_mon.pop_partition_flag[idx] === 1'b1) begin
                  `uvm_error("E027", $sformatf(
                    "Inactive partition asserted flag during the partition2-only case: partition=%0d flags=0x%0h",
                    idx, m_env.m_dbg_mon.pop_partition_flag))
                end else begin
                  saw_zero_flag_mask[idx] = 1'b1;
                end
              end
            end
          end
          if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1) begin
            visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
            if (visit_idx >= 4) begin
              `uvm_error("E027", $sformatf(
                "Partition2-only case issued from out-of-range partition=%0d addr=%0d",
                visit_idx, m_env.m_dbg_mon.vif.pop_issue_addr))
            end else begin
              focus_issue_mask[visit_idx[1:0]] = 1'b1;
              if (visit_idx != 2) begin
                `uvm_error("E027", $sformatf(
                  "Partition2-only case issued from the wrong partition=%0d addr=%0d",
                  visit_idx, m_env.m_dbg_mon.vif.pop_issue_addr))
              end
            end
            grant_count++;
          end
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end

      if (!saw_partition2_flag) begin
        `uvm_error("E027", "Partition2-only case never observed a flagged result on partition2")
      end
      if ((saw_zero_flag_mask & 4'b1011) != 4'b1011) begin
        `uvm_error("E027", $sformatf(
          "Partition2-only case did not observe zero-flag result_valid on partitions {0,1,3}: observed_mask=0x%0h",
          saw_zero_flag_mask))
      end
      if (grant_count != 8 || focus_issue_mask != 4'b0100) begin
        `uvm_error("E027", $sformatf(
          "Partition2-only focus drain mismatch: grants=%0d issue_mask=0x%0h expected_mask=0x4",
          grant_count, focus_issue_mask))
      end

      wait_for_scoreboard_idle(1_500_000, "E027 partition2-only drain");
      expect_service_model_accounting("E027 partition2-only drain", 1, 0);
      if (m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]) != 8) begin
        `uvm_error("E027", $sformatf(
          "Partition2-only focus-key drain mismatch: key=0x%02x drained=%0d expected=%0d",
          focus_search_key[7:0],
          m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]), 8))
      end
    end else if (case_id == "E034") begin
      int unsigned focus_issue_q[$];
      int unsigned focus_partition_q[$];

      if (m_cfg.n_partitions != 4) begin
        `uvm_fatal("E034", "E034 requires the 4-partition p4 build variant")
      end

      configure_and_start(16'hFFFF);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      hit_before = m_env.m_out_mon.total_hits_seen;

      burst_seq = same_key_burst_seq::type_id::create("e034_first_partition0_hit");
      burst_seq.num_hits = 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 20_000, "E034 first focus hit");

      burst_seq = same_key_burst_seq::type_id::create("e034_interstitial_prefill");
      burst_seq.num_hits = partition_size * 2 - 1;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(1);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(partition_size * 2, 120_000, "E034 interstitial prefill");

      burst_seq = same_key_burst_seq::type_id::create("e034_second_partition2_hit");
      burst_seq.num_hits = 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count((partition_size * 2) + 1, 40_000, "E034 second focus hit");

      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd2, 1'b1, 1'b1, 300_000,
        "E034 split-partition target subheader", matched_subheader);

      search_cycles = 0;
      while (search_cycles < 400_000 && focus_issue_q.size() < 2) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant === 1'b1 &&
            m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0]) begin
          focus_issue_q.push_back(m_env.m_dbg_mon.vif.pop_issue_addr);
          focus_partition_q.push_back(m_env.m_dbg_mon.vif.pop_issue_addr / partition_size);
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end

      if (focus_issue_q.size() != 2) begin
        `uvm_error("E034", $sformatf(
          "Did not observe the two focus-key pop issues for the split-partition case: observed=%0d",
          focus_issue_q.size()))
      end else begin
        if (focus_partition_q[0] != 0 || focus_partition_q[1] != 2) begin
          `uvm_error("E034", $sformatf(
            "Split-partition issue order mismatch: observed partitions={%0d,%0d} expected={0,2}",
            focus_partition_q[0], focus_partition_q[1]))
        end
        if (focus_issue_q[0] != 0 || focus_issue_q[1] != (partition_size * 2)) begin
          `uvm_error("E034", $sformatf(
            "Split-partition issue addresses mismatch: observed={%0d,%0d} expected={0,%0d}",
            focus_issue_q[0], focus_issue_q[1], partition_size * 2))
        end
      end

      wait_for_scoreboard_idle(1_500_000, "E034 split-partition drain");
      expect_service_model_accounting("E034 split-partition drain", 1, 0);
      if (m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]) != 2) begin
        `uvm_error("E034", $sformatf(
          "Split-partition focus-key drain mismatch: key=0x%02x drained=%0d expected=%0d",
          focus_search_key[7:0],
          m_env.m_scb.drained_hits_for_key(focus_search_key[7:0]), 2))
      end
    end else if (case_id == "E025") begin
      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e025_slot0_single");
      burst_seq.num_hits = 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 20_000, "E025 one-hit fill");
      wait_for_pop_engine_state(3'd4, 80_000, "E025 DRAIN entry");
      hit_before = m_env.m_out_mon.total_hits_seen;
      if (m_env.m_dbg_mon.pop_partition_pending != 4'b0001) begin
        `uvm_error("E025", $sformatf(
          "Partition snapshot leaked stale partitions: pending=0x%0h",
          m_env.m_dbg_mon.pop_partition_pending))
      end
      load_mask = '0;
      wait_for_partition_issue_mask(4'b0001, load_mask, 80_000, "E025 partition0 issue mask");
      if (load_mask != 4'b0001) begin
        `uvm_error("E025", $sformatf(
          "Single-slot0 hit escaped partition0 issue mask: observed=0x%0h",
          load_mask))
      end
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      wait_for_hit_output_count(hit_before + 1, 120_000, "E025 single-hit output");
      if (m_env.m_out_mon.total_hits_seen != (hit_before + 1)) begin
        `uvm_error("E025", "Exactly one hit beat was expected for the single-hit burst")
      end
      matched_subheader = null;
      for (int idx = int'(m_env.m_out_mon.recent_data_subheaders.size()) - 1; idx >= 0; idx--) begin
        if (m_env.m_out_mon.recent_data_subheaders[idx].search_key == focus_search_key[7:0] &&
            m_env.m_out_mon.recent_data_subheaders[idx].hit_count == 8'd1) begin
          matched_subheader = m_env.m_out_mon.recent_data_subheaders[idx];
          break;
        end
      end
      if (matched_subheader == null) begin
        wait_for_subheader_match(
          focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 40_000,
          "E025 target single-hit subheader", matched_subheader);
      end
      if (matched_subheader == null || m_env.m_out_mon.recent_hits.size() == 0) begin
        `uvm_error("E025", "Did not capture subheader and hit beat")
      end else begin
        if (matched_subheader.search_key != focus_search_key[7:0]) begin
          `uvm_error("E025", $sformatf(
            "Single-hit subheader search-key mismatch: observed=0x%02x expected=0x%02x",
            matched_subheader.search_key, focus_search_key[7:0]))
        end
        if (matched_subheader.hit_count != 1) begin
          `uvm_error("E025", $sformatf(
            "Expected one-hit framing but observed hit_count=%0d",
            matched_subheader.hit_count))
        end
        if (!(matched_subheader.sop && !matched_subheader.eop)) begin
          `uvm_error("E025", "One-hit framing was not SOP-only")
        end
        if (!m_env.m_out_mon.recent_hits[$].eop) begin
          `uvm_error("E025", "One-hit frame did not close on hit beat")
        end
      end
      wait_for_scoreboard_idle(80_000, "E025 slot0 single-hit drain");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 1 || pop_count != 1 || overwrite_count != 0 || fill_level != 0) begin
        `uvm_error("E025", $sformatf(
          "Single-hit accounting mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d",
          push_count, pop_count, overwrite_count, fill_level))
      end
    end else if (case_id == "E026") begin
      configure_and_start(16'hFFFF);
      last_partition_idx = (m_cfg.n_partitions > 0) ? (m_cfg.n_partitions - 1) : 0;
      expected_partition_mask = 4'b0001;
      if (last_partition_idx < 4) begin
        expected_partition_mask[last_partition_idx] = 1'b1;
      end
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e026_two_partition_fill");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry - partition_size + 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry - partition_size + 1, 120_000, "E026 two-partition fill");
      csr_write(CSR_EXPECTED_LAT_ADDR, 32'h0000_0000);
      wait_clocks(4);
      wait_for_pop_engine_state(3'd4, 80_000, "E026 DRAIN entry");
      hit_before = m_env.m_out_mon.total_hits_seen;
      search_cycles = 0;
      while (search_cycles < 80_000) begin
        if ((m_env.m_dbg_mon.pop_partition_pending & expected_partition_mask) == expected_partition_mask) begin
          break;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if ((m_env.m_dbg_mon.pop_partition_pending & expected_partition_mask) != expected_partition_mask) begin
        `uvm_error("E026", $sformatf(
          "Target partition snapshot mismatch after DRAIN stabilizes: observed=0x%0h expected=0x%0h",
          m_env.m_dbg_mon.pop_partition_pending, expected_partition_mask))
      end
      load_mask = '0;
      grant_count = 0;
      saw_rr_step1 = 1'b0;
      saw_rr_step2 = 1'b0;
      saw_last_partition = 1'b0;
      search_cycles = 0;
      while (search_cycles < 240_000 &&
             ((load_mask & expected_partition_mask) != expected_partition_mask ||
              grant_count < 1)) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant) begin
          visit_idx = m_env.m_dbg_mon.vif.pop_issue_addr / partition_size;
          if (visit_idx < 4) begin
            load_mask[visit_idx[1:0]] = 1'b1;
          end
          grant_count++;
          if (grant_count == 1 && visit_idx[1:0] != 2'd0) begin
            `uvm_error("E026", $sformatf(
              "First pop partition should be P0, observed=%0d", visit_idx))
          end
          if (visit_idx == last_partition_idx) begin
            saw_last_partition = 1'b1;
          end
        end else if (grant_count == 1) begin
          if (m_env.m_dbg_mon.pop_rr_idx == 2'd1) begin
            saw_rr_step1 = 1'b1;
          end
          if (m_env.m_dbg_mon.pop_rr_idx == 2'd2) begin
            saw_rr_step2 = 1'b1;
          end
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_cfg.n_partitions > 2 && !saw_rr_step1) begin
        `uvm_error("E026", "Did not observe round-robin advance to partition1 while scanning")
      end
      if (m_cfg.n_partitions > 3 && !saw_rr_step2) begin
        `uvm_error("E026", "Did not observe round-robin advance to partition2 while scanning")
      end
      if (!saw_last_partition) begin
        `uvm_error("E026", $sformatf(
          "Last partition P%0d was never visited after drain start", last_partition_idx[1:0]))
      end
      if (load_mask != expected_partition_mask) begin
        `uvm_error("E026", $sformatf(
          "Partition visits did not match expected mask 0x%0h: observed=0x%0h",
          expected_partition_mask, load_mask))
      end
      wait_for_hit_output_count(
        hit_before + (m_cfg.ring_buffer_n_entry - partition_size + 1), 200_000, "E026 two-partition drain");
      wait_for_scoreboard_idle(160_000, "E026 partition skip drain");
    end else if (case_id == "E031") begin
      configure_and_start();
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;
      saw_load = 1'b0;
      saw_done_pulse = 1'b0;
      search_cycles = 0;
      while (search_cycles < 40_000 && !saw_done_pulse) begin
        if (m_env.m_dbg_mon.pop_engine_state_code == 3'd3) begin
          saw_load = 1'b1;
          if (m_env.m_dbg_mon.vif.pop_total_hits != 0 ||
              m_env.m_dbg_mon.pop_hits_count != 0) begin
            `uvm_error("E031", $sformatf(
              "Zero-hit COUNT phase carried stale hit totals: pop_total_hits=%0d pop_hits_count=%0d",
              m_env.m_dbg_mon.vif.pop_total_hits,
              m_env.m_dbg_mon.pop_hits_count))
          end
        end
        if (m_env.m_out_mon.total_subheaders_seen > subhdr_before) begin
          saw_done_pulse = 1'b1;
          break;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (!saw_load) begin
        `uvm_error("E031", "Zero-hit drain never exposed the COUNT phase")
      end
      if (!saw_done_pulse) begin
        `uvm_error("E031", "Zero-hit drain never emitted a subheader")
      end
      wait_for_pop_engine_state(3'd5, 40_000, "E031 RESET entry");
      if (m_env.m_out_mon.recent_subheaders.size() == 0) begin
        `uvm_error("E031", "No subheader captured for zero-hit drain")
      end else begin
        matched_subheader = m_env.m_out_mon.recent_subheaders[$];
        if (!(matched_subheader.sop && matched_subheader.eop)) begin
          `uvm_error("E031", "Zero-hit drain subheader did not assert SOP+EOP")
        end
        if (matched_subheader.hit_count != 0) begin
          `uvm_error("E031", $sformatf(
            "Zero-hit drain subheader carried hit_count=%0d",
            matched_subheader.hit_count))
        end
        if (matched_subheader.raw_data[7:0] != ring_buffer_cam_pkg::K237) begin
          `uvm_error("E031", "Zero-hit drain K237 marker mismatch")
        end
      end
      if (m_env.m_out_mon.total_hits_seen != hit_before) begin
        `uvm_error("E031", $sformatf(
          "Zero-hit drain unexpectedly emitted hit data: hits_before=%0d hits_after=%0d",
          hit_before, m_env.m_out_mon.total_hits_seen))
      end
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      if (pop_count != 0 || cache_miss_count != 0) begin
        `uvm_error("E031", $sformatf(
          "Zero-hit drain accounting mismatch: pop=%0d cache_miss=%0d",
          pop_count, cache_miss_count))
      end
    end else if (case_id == "E032") begin
      configure_and_start();
      hit_before = m_env.m_out_mon.total_hits_seen;
      burst_seq = same_key_burst_seq::type_id::create("e032_single_hit");
      burst_seq.num_hits = 1;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_match(
        burst_seq.search_key[7:0], 1, 1'b1, 1'b1, 80_000,
        "E032 target single-hit subheader", matched_subheader);
      wait_for_hit_output_count(hit_before + 1, 120_000, "E032 single-hit output");
      if (matched_subheader == null || m_env.m_out_mon.recent_hits.size() == 0) begin
        `uvm_error("E032", "Did not capture both subheader and hit beats")
      end else begin
        if (!(matched_subheader.sop && !matched_subheader.eop)) begin
          `uvm_error("E032", "SOP-only subheader for N=1 hit was not observed")
        end
        if (!m_env.m_out_mon.recent_hits[$].eop) begin
          `uvm_error("E032", "Single hit did not assert EOP")
        end
      end
      wait_for_scoreboard_idle(80_000, "E032 single-hit drain");
    end else if (case_id == "E033") begin
      int unsigned issue_addr_a;
      int unsigned issue_addr_b;
      int unsigned issue_seen;

      configure_and_start();
      hit_before = m_env.m_out_mon.total_hits_seen;
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("e033_two_hit_adjacent");
      burst_seq.num_hits = 2;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_subheader_match(
        focus_search_key[7:0], 2, 1'b1, 1'b1, 80_000,
        "E033 target two-hit subheader", matched_subheader);
      issue_seen = 0;
      search_cycles = 0;
      while (search_cycles < 120_000 && issue_seen < 2) begin
        if (m_env.m_dbg_mon.vif.pop_erase_grant) begin
          if (issue_seen == 0) begin
            issue_addr_a = m_env.m_dbg_mon.vif.pop_issue_addr;
          end else begin
            issue_addr_b = m_env.m_dbg_mon.vif.pop_issue_addr;
          end
          issue_seen++;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      wait_for_hit_output_count(hit_before + 2, 120_000, "E033 two-hit output");
      if (matched_subheader == null || m_env.m_out_mon.recent_hits.size() < 2) begin
        `uvm_error("E033", "Did not capture the two-hit epoch history")
      end else begin
        if (!(matched_subheader.sop && !matched_subheader.eop)) begin
          `uvm_error("E033", "Two-hit subheader was not SOP-only")
        end
        if (m_env.m_out_mon.recent_hits[$-1].eop) begin
          `uvm_error("E033", "First data beat of two-hit epoch asserted EOP")
        end
        if (!m_env.m_out_mon.recent_hits[$].eop) begin
          `uvm_error("E033", "Second data beat of two-hit epoch did not assert EOP")
        end
      end
      if (issue_seen != 2) begin
        `uvm_error("E033", $sformatf(
          "Did not observe two pop issues for the two-hit epoch: observed=%0d",
          issue_seen))
      end else begin
        if (issue_addr_a >= partition_size || issue_addr_b >= partition_size) begin
          `uvm_error("E033", $sformatf(
            "Two-hit epoch escaped partition0: addr_a=%0d addr_b=%0d partition_size=%0d",
            issue_addr_a, issue_addr_b, partition_size))
        end
        if (issue_addr_b != (issue_addr_a + 1)) begin
          `uvm_error("E033", $sformatf(
            "Adjacent-slot issue check failed: addr_a=%0d addr_b=%0d",
            issue_addr_a, issue_addr_b))
        end
      end
      wait_for_scoreboard_idle(80_000, "E033 adjacent two-hit drain");
    end else if (case_id == "E035") begin
      run_same_key_epoch_count_case("E035 hit_count=255", 255, 2, 8'hFF, 2000, 300_000);
    end else if (case_id == "E036") begin
      run_same_key_epoch_count_case("E036 hit_count wraps at 256", 256, 2, 8'h00, 2000, 300_000);
    end else if (case_id == "E037") begin
      run_same_key_epoch_count_case("E037 hit_count wraps at 511", 511, 2, 8'hFF, 2000, 500_000);
    end else if (case_id == "E019") begin
      configure_and_start(1);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      burst_seq = same_key_burst_seq::type_id::create("e019_depthm1_latency1");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry - 1;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry - 1, 40_000, "E019 depth-1 fill");
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'h0001) begin
        `uvm_error("E019", $sformatf(
          "EXPECTED_LATENCY=1 did not reach the DUT: observed=0x%04x",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end
      search_cycles = 0;
      while (search_cycles < 256 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E019", "EXPECTED_LATENCY=1 did not trigger an early pop descriptor")
      end
      wait_for_subheader_match(
        focus_search_key[7:0], '0, 1'b0, 1'b1, 500_000,
        "E019 latency-1 data-bearing subheader", matched_subheader);
      wait_for_hit_output_count(m_cfg.ring_buffer_n_entry - 1, 3_000_000, "E019 full depth-1 drain");
      wait_for_scoreboard_idle(500_000, "E019 latency-1 full drain");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != (m_cfg.ring_buffer_n_entry - 1) ||
          pop_count != (m_cfg.ring_buffer_n_entry - 1) ||
          overwrite_count != 0 ||
          fill_level != 0) begin
        `uvm_error("E019", $sformatf(
          "Latency-1 depth-1 accounting mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d expected_push_pop=%0d",
          push_count, pop_count, overwrite_count, fill_level, m_cfg.ring_buffer_n_entry - 1))
      end
    end else if (case_id == "E020") begin
      configure_and_start(16'hFFFF);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      burst_seq = same_key_burst_seq::type_id::create("e020_depth_latency_max");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry, 40_000, "E020 exact-depth fill");
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'hFFFF) begin
        `uvm_error("E020", $sformatf(
          "EXPECTED_LATENCY=max did not reach the DUT: observed=0x%04x",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end
      wait_clocks(4_096);
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count != cmd_before ||
          m_env.m_out_mon.total_subheaders_seen != subhdr_before) begin
        `uvm_error("E020", $sformatf(
          "Maximum-latency run started draining too early: cmd_before=%0d cmd_after=%0d subhdr_before=%0d subhdr_after=%0d",
          cmd_before, m_env.m_dbg_mon.pop_cmd_wrreq_count,
          subhdr_before, m_env.m_out_mon.total_subheaders_seen))
      end
      search_cycles = 0;
      while (search_cycles < 120_000 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E020", "Maximum-latency run never emitted a delayed pop descriptor")
      end
      wait_for_hit_output_count(m_cfg.ring_buffer_n_entry, 6_000_000, "E020 delayed full-depth drain");
      wait_for_scoreboard_idle(1_000_000, "E020 delayed full-depth drain");
      if (m_env.m_dbg_mon.first_push_cycle == 0 || m_env.m_dbg_mon.first_pop_cycle == 0) begin
        `uvm_error("E020", "Maximum-latency run did not observe both first push and first pop")
      end else begin
        cycle_a = m_env.m_dbg_mon.first_pop_cycle - m_env.m_dbg_mon.first_push_cycle;
        if (cycle_a < 60_000) begin
          `uvm_error("E020", $sformatf(
            "Maximum-latency drain started too early: first_pop_minus_first_push=%0d cycles",
            cycle_a))
        end
      end
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != m_cfg.ring_buffer_n_entry ||
          pop_count != m_cfg.ring_buffer_n_entry ||
          overwrite_count != 0 ||
          fill_level != 0) begin
        `uvm_error("E020", $sformatf(
          "Maximum-latency accounting mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d expected_push_pop=%0d",
          push_count, pop_count, overwrite_count, fill_level, m_cfg.ring_buffer_n_entry))
      end
    end else if (case_id == "E021") begin
      configure_and_start(2000);
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      seq_keys = sequential_keys_seq::type_id::create("e021_lane_local_key_sweep");
      seq_keys.num_keys = 64;
      seq_keys.hits_per_key = 8;
      seq_keys.start_key = 0;
      seq_keys.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry, 40_000, "E021 lane-local key sweep");
      wait_for_hit_output_count(m_cfg.ring_buffer_n_entry, 4_000_000, "E021 lane-local full drain");
      wait_for_scoreboard_idle(600_000, "E021 lane-local key sweep drain");
      if (m_env.m_out_mon.total_data_subheaders_seen != subhdr_before + 64) begin
        `uvm_error("E021", $sformatf(
          "Lane-local key sweep emitted the wrong number of data-bearing subheaders: observed_delta=%0d expected=64",
          m_env.m_out_mon.total_data_subheaders_seen - subhdr_before))
      end
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != m_cfg.ring_buffer_n_entry ||
          pop_count != m_cfg.ring_buffer_n_entry ||
          overwrite_count != 0 ||
          fill_level != 0) begin
        `uvm_error("E021", $sformatf(
          "Lane-local key sweep accounting mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d expected_push_pop=%0d",
          push_count, pop_count, overwrite_count, fill_level, m_cfg.ring_buffer_n_entry))
      end
    end else if (case_id == "E022") begin
      profile_traffic_seq e022_seq;
      int unsigned        outstanding;
      int unsigned        max_outstanding;
      int unsigned        max_usedw;
      bit                 saw_saturation;

      configure_and_start(128);
      max_outstanding = 0;
      max_usedw = 0;
      saw_saturation = 1'b0;
      traffic_done = 1'b0;
      fork
        begin
          e022_seq = profile_traffic_seq::type_id::create("e022_fifo_fill");
          e022_seq.num_hits = 12_000;
          e022_seq.lane_key_start_ord = 2;
          e022_seq.pool_keys = 1;
          e022_seq.hits_per_key_switch = 1;
          e022_seq.inter_hit_gap_cycles = 16;
          e022_seq.progress_stride = 3_000;
          e022_seq.progress_tag = "E022";
          e022_seq.start(m_env.m_hit_seqr);
          traffic_done = 1'b1;
        end
        begin
          search_cycles = 0;
          while (search_cycles < 2_500_000 &&
                 (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0 ||
                  m_env.m_dbg_mon.pop_cmd_fifo_usedw != 0)) begin
            outstanding = m_env.m_dbg_mon.pop_cmd_wrreq_count -
                          m_env.m_dbg_mon.pop_cmd_rdack_count;
            if (outstanding > max_outstanding)
              max_outstanding = outstanding;
            if (int'(m_env.m_dbg_mon.pop_cmd_fifo_usedw) > max_usedw)
              max_usedw = int'(m_env.m_dbg_mon.pop_cmd_fifo_usedw);
            if (int'(m_env.m_dbg_mon.pop_cmd_fifo_usedw) >= 14)
              saw_saturation = 1'b1;
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
        end
      join

      wait_for_scoreboard_idle(2_500_000, "E022 descriptor fifo saturation drain");
      expect_service_model_accounting("E022 descriptor fifo saturation drain", 1, 0);
      if (!saw_saturation) begin
        `uvm_error("E022", $sformatf(
          "Descriptor saturation case never reached the pop_cmd fifo saturation band: max_usedw=%0d",
          max_usedw))
      end
      if (max_usedw > 15) begin
        `uvm_error("E022", $sformatf(
          "Descriptor saturation case exceeded the physical pop_cmd fifo depth: max_usedw=%0d expected<=15",
          max_usedw))
      end
      if (max_outstanding > 16) begin
        `uvm_error("E022", $sformatf(
          "Descriptor saturation case exceeded the outstanding descriptor budget: max_outstanding=%0d expected<=16",
          max_outstanding))
      end
    end else if (case_id == "E023") begin
      configure_and_start(2000);
      pressure_seq = overwrite_profile_seq::type_id::create("e023_steady_state");
      pressure_seq.num_hits = 2000;
      pressure_seq.lane_key_start_ord = 0;
      pressure_seq.pool_keys = 64;
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.inter_hit_gap_cycles = 31;
      pressure_seq.progress_stride = 500;
      pressure_seq.progress_tag = "E023";
      pressure_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(1_500_000, "E023 steady-state drain");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 2000 || pop_count != 2000 || overwrite_count != 0 || fill_level != 0) begin
        `uvm_error("E023", $sformatf(
          "Steady-state drain accounting mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d expected_push_pop=2000",
          push_count, pop_count, overwrite_count, fill_level))
      end
      if (m_env.m_scb.max_remaining_entries() < 32 ||
          m_env.m_scb.max_remaining_entries() > 128) begin
        `uvm_error("E023", $sformatf(
          "Steady-state backlog escaped the expected bounded window: max_remaining=%0d expected_range=[32,128]",
          m_env.m_scb.max_remaining_entries()))
      end
    end else if (case_id == "E039") begin
      int unsigned                     second_search_key;
      ring_buffer_cam_pkg::out_seq_item prev_subheader;

      configure_and_start(0);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      second_search_key = m_cfg.lane_key_ord_to_search_key(3);
      subhdr_before = m_env.m_out_mon.total_data_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;

      single_seq = single_push_pop_seq::type_id::create("e039_first_single");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_clocks(16);
      single_seq = single_push_pop_seq::type_id::create("e039_second_single");
      single_seq.search_key = second_search_key;
      single_seq.start(m_env.m_hit_seqr);

      wait_for_scoreboard_idle(200_000, "E039 one-descriptor-gap two-key drain");
      expect_service_model_accounting("E039 one-descriptor-gap two-key drain", 1, 0);
      if (m_env.m_out_mon.total_hits_seen != hit_before + 2) begin
        `uvm_error("E039", $sformatf(
          "One-descriptor-gap two-key case emitted the wrong hit count: hits_before=%0d hits_after=%0d",
          hit_before, m_env.m_out_mon.total_hits_seen))
      end
      if (m_env.m_out_mon.total_data_subheaders_seen != subhdr_before + 2) begin
        `uvm_error("E039", $sformatf(
          "One-descriptor-gap two-key case emitted the wrong number of data subheaders: subhdr_before=%0d subhdr_after=%0d",
          subhdr_before, m_env.m_out_mon.total_data_subheaders_seen))
      end else if (m_env.m_out_mon.recent_data_subheaders.size() < 2) begin
        `uvm_error("E039", "Did not retain the two expected data subheaders in monitor history")
      end else begin
        prev_subheader =
          m_env.m_out_mon.recent_data_subheaders[m_env.m_out_mon.recent_data_subheaders.size()-2];
        matched_subheader = m_env.m_out_mon.recent_data_subheaders[$];
        if (prev_subheader.search_key != focus_search_key[7:0] ||
            matched_subheader.search_key != second_search_key[7:0] ||
            prev_subheader.hit_count != 8'd1 ||
            matched_subheader.hit_count != 8'd1) begin
          `uvm_error("E039", $sformatf(
            "One-descriptor-gap key order/framing mismatch: first_key=0x%02x first_hits=%0d second_key=0x%02x second_hits=%0d expected=(0x%02x,1)->(0x%02x,1)",
            prev_subheader.search_key, prev_subheader.hit_count,
            matched_subheader.search_key, matched_subheader.hit_count,
            focus_search_key[7:0], second_search_key[7:0]))
        end
      end
    end else if (case_id == "E043") begin
      int unsigned max_subheader_valid_streak;
      int unsigned current_subheader_valid_streak;
      int unsigned zero_subheaders_seen;

      configure_and_start(0);
      hit_before = m_env.m_out_mon.total_hits_seen;
      max_subheader_valid_streak = 0;
      current_subheader_valid_streak = 0;
      zero_subheaders_seen = 0;
      search_cycles = 0;
      while (search_cycles < 120_000 && zero_subheaders_seen < 3) begin
        if (m_env.m_out_mon.vif.valid === 1'b1 &&
            m_env.m_out_mon.vif.data[35:32] == 4'b0001) begin
          current_subheader_valid_streak++;
          if (current_subheader_valid_streak > max_subheader_valid_streak)
            max_subheader_valid_streak = current_subheader_valid_streak;
          if (m_env.m_out_mon.vif.data[15:8] != 8'd0) begin
            `uvm_error("E043", $sformatf(
              "Subheader-only valid-pulse audit observed a non-zero hit_count=%0d",
              m_env.m_out_mon.vif.data[15:8]))
          end
          zero_subheaders_seen++;
        end else begin
          current_subheader_valid_streak = 0;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (zero_subheaders_seen < 3) begin
        `uvm_error("E043", $sformatf(
          "Did not observe enough zero-hit subheaders for the valid-pulse audit: observed=%0d expected_at_least=3",
          zero_subheaders_seen))
      end
      if (max_subheader_valid_streak != 1) begin
        `uvm_error("E043", $sformatf(
          "Subheader valid stayed asserted across multiple cycles: max_streak=%0d expected=1",
          max_subheader_valid_streak))
      end
      if (m_env.m_out_mon.total_hits_seen != hit_before) begin
        `uvm_error("E043", $sformatf(
          "Zero-hit subheader valid-pulse audit emitted unexpected hit beats: hits_before=%0d hits_after=%0d",
          hit_before, m_env.m_out_mon.total_hits_seen))
      end
    end else if (case_id == "E111") begin
      configure_and_start(1);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      single_seq = single_push_pop_seq::type_id::create("e111_min_latency_single");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "E111 minimum-latency push");
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'h0001) begin
        `uvm_error("E111", $sformatf(
          "EXPECTED_LATENCY=1 did not reach the DUT: observed=0x%04x",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end
      search_cycles = 0;
      while (search_cycles < 128 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E111", "EXPECTED_LATENCY=1 did not trigger an early pop descriptor")
      end
      wait_clocks(4);
      dbg48_a = m_env.m_dbg_mon.gts_8n - m_env.m_dbg_mon.read_time_ptr;
      if (dbg48_a > 48'd2) begin
        `uvm_error("E111", $sformatf(
          "EXPECTED_LATENCY=1 did not hold read_time_ptr within the minimum-latency window: observed_delta=%0d",
          dbg48_a))
      end
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 80_000,
        "E111 minimum-latency single-hit subheader", matched_subheader);
      wait_for_scoreboard_idle(80_000, "E111 minimum-latency drain");
      expect_service_model_accounting("E111 minimum-latency drain", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 1 || pop_count != 1 || overwrite_count != 0 || fill_level != 0) begin
        `uvm_error("E111", $sformatf(
          "Minimum-latency accounting mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d",
          push_count, pop_count, overwrite_count, fill_level))
      end
    end else if (case_id == "E112") begin
      configure_and_start(0);
      focus_search_key = m_cfg.interleaving_index;
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;
      wait_clocks(4);
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'h0000) begin
        `uvm_error("E112", $sformatf(
          "EXPECTED_LATENCY=0 did not reach the DUT: observed=0x%04x",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end
      dbg48_a = m_env.m_dbg_mon.gts_8n - m_env.m_dbg_mon.read_time_ptr;
      if (dbg48_a > 48'd1) begin
        `uvm_error("E112", $sformatf(
          "EXPECTED_LATENCY=0 did not collapse the read_time_ptr delta: observed=%0d",
          dbg48_a))
      end
      search_cycles = 0;
      while (search_cycles < 128 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E112", "EXPECTED_LATENCY=0 did not trigger the immediate pop descriptor")
      end
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd0, 1'b1, 1'b0, 80_000,
        "E112 immediate zero-hit subheader", matched_subheader);
      if (matched_subheader == null || !(matched_subheader.sop && matched_subheader.eop)) begin
        `uvm_error("E112", "Immediate zero-hit descriptor did not emit a single-beat SOP+EOP subheader")
      end
      if (m_env.m_out_mon.total_subheaders_seen <= subhdr_before) begin
        `uvm_error("E112", $sformatf(
          "Immediate zero-hit descriptor did not retire any subheader: before=%0d after=%0d",
          subhdr_before, m_env.m_out_mon.total_subheaders_seen))
      end
      if (m_env.m_out_mon.total_hits_seen != hit_before) begin
        `uvm_error("E112", $sformatf(
          "Immediate zero-hit descriptor unexpectedly emitted hit data: hits_before=%0d hits_after=%0d",
          hit_before, m_env.m_out_mon.total_hits_seen))
      end
    end else if (case_id == "E113") begin
      configure_and_start(16'hFFFF);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'hFFFF) begin
        `uvm_error("E113", $sformatf(
          "EXPECTED_LATENCY=max did not reach the DUT: observed=0x%04x",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end
      wait_clocks(60_000);
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count != cmd_before ||
          m_env.m_out_mon.total_subheaders_seen != subhdr_before) begin
        `uvm_error("E113", $sformatf(
          "Maximum-latency idle run started draining too early: cmd_before=%0d cmd_after=%0d subhdr_before=%0d subhdr_after=%0d",
          cmd_before, m_env.m_dbg_mon.pop_cmd_wrreq_count,
          subhdr_before, m_env.m_out_mon.total_subheaders_seen))
      end
      search_cycles = 0;
      while (search_cycles < 20_000 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E113", "Maximum-latency idle run never emitted the delayed pop descriptor")
      end
      dbg48_a = m_env.m_dbg_mon.gts_8n - m_env.m_dbg_mon.read_time_ptr;
      if ((dbg48_a < 48'd65534) || (dbg48_a > 48'd65536)) begin
        `uvm_error("E113", $sformatf(
          "Maximum-latency idle run observed the wrong read_time_ptr delta: observed=%0d expected~=65535",
          dbg48_a))
      end
      wait_for_subheader_count(subhdr_before + 1, 120_000, "E113 delayed zero-hit subheader");
      if (m_env.m_out_mon.recent_subheaders.size() == 0) begin
        `uvm_error("E113", "Maximum-latency idle run did not retain the delayed zero-hit subheader")
      end else begin
        matched_subheader = m_env.m_out_mon.recent_subheaders[$];
        if (!(matched_subheader.sop && matched_subheader.eop) ||
            matched_subheader.hit_count != 8'd0) begin
          `uvm_error("E113", $sformatf(
            "Maximum-latency idle run emitted malformed delayed zero-hit framing: sop=%0d eop=%0d hit_count=%0d",
            matched_subheader.sop, matched_subheader.eop, matched_subheader.hit_count))
        end
      end
      if (m_env.m_out_mon.total_hits_seen != hit_before) begin
        `uvm_error("E113", $sformatf(
          "Maximum-latency idle run unexpectedly emitted hit data: hits_before=%0d hits_after=%0d",
          hit_before, m_env.m_out_mon.total_hits_seen))
      end
    end else if (case_id == "E114") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      single_seq = single_push_pop_seq::type_id::create("e114_first_latency");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "E114 first-latency push");
      cycle_a = m_env.m_dbg_mon.sampled_cycles;
      search_cycles = 0;
      while (search_cycles < 160_000 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E114", "First-latency hit never emitted a descriptor before the rewrite")
      end
      search_cycles = int'(m_env.m_dbg_mon.sampled_cycles - cycle_a);
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 160_000,
        "E114 first-latency subheader", matched_subheader);

      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd1);
      wait_clocks(4);
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'd1) begin
        `uvm_error("E114", $sformatf(
          "Latency rewrite to 1 did not reach the DUT: observed=%0d",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end

      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      cmd_after = m_cfg.lane_key_ord_to_search_key(4);
      single_seq = single_push_pop_seq::type_id::create("e114_second_latency");
      single_seq.search_key = cmd_after;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(2, 10_000, "E114 second-latency push");
      cycle_b = m_env.m_dbg_mon.sampled_cycles;
      wait_min = 0;
      while (wait_min < 2_048 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        wait_min++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E114", "Latency rewrite to 1 did not accelerate the second descriptor window")
      end
      wait_min = int'(m_env.m_dbg_mon.sampled_cycles - cycle_b);
      if (wait_min >= search_cycles || wait_min > 256) begin
        `uvm_error("E114", $sformatf(
          "Latency rewrite did not shorten the descriptor window enough: before=%0d after=%0d",
          search_cycles, wait_min))
      end
      wait_for_subheader_match(
        cmd_after[7:0], 8'd1, 1'b1, 1'b1, 80_000,
        "E114 post-rewrite single-hit subheader", matched_subheader);
      wait_for_scoreboard_idle(120_000, "E114 latency rewrite drain");
      expect_service_model_accounting("E114 latency rewrite drain", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 2 || pop_count != 2 || overwrite_count != 0 || fill_level != 0) begin
        `uvm_error("E114", $sformatf(
          "Latency rewrite accounting mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d",
          push_count, pop_count, overwrite_count, fill_level))
      end
    end else if (case_id == "E115") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      single_seq = single_push_pop_seq::type_id::create("e115_focus_latency");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "E115 first-latency push");
      cycle_a = m_env.m_dbg_mon.sampled_cycles;
      search_cycles = 0;
      while (search_cycles < 160_000 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E115", "First-latency hit never emitted a descriptor before the mid-SEARCH rewrite")
      end
      search_cycles = int'(m_env.m_dbg_mon.sampled_cycles - cycle_a);

      wait_for_pop_engine_state(3'd1, 40_000, "E115 SEARCH entry");
      wait_min = 0;
      while (wait_min < 128 &&
             !(m_env.m_dbg_mon.pop_engine_state_code == 3'd1 &&
               m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0] &&
               m_env.m_dbg_mon.pop_search_wait_cnt <= 3'd1)) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        wait_min++;
      end
      if (!(m_env.m_dbg_mon.pop_engine_state_code == 3'd1 &&
            m_env.m_dbg_mon.vif.pop_current_sk[7:0] == focus_search_key[7:0] &&
            m_env.m_dbg_mon.pop_search_wait_cnt <= 3'd1)) begin
        `uvm_error("E115", $sformatf(
          "Did not observe the early SEARCH guard before rewriting EXPECTED_LATENCY: pop_state=%0d active_key=%0d wait_cnt=%0d focus_key=%0d",
          m_env.m_dbg_mon.pop_engine_state_code,
          m_env.m_dbg_mon.vif.pop_current_sk[7:0],
          m_env.m_dbg_mon.pop_search_wait_cnt,
          focus_search_key[7:0]))
      end

      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd5);
      wait_clocks(1);
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'd5) begin
        `uvm_error("E115", $sformatf(
          "Mid-SEARCH latency rewrite to 5 did not reach the DUT: observed=%0d",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end

      saw_overlap = 1'b0;
      saw_load = 1'b0;
      wait_max = m_env.m_dbg_mon.pop_search_wait_cnt;
      wait_min = 0;
      while (wait_min < 16 && !saw_load) begin
        if (m_env.m_dbg_mon.pop_engine_state_code == 3'd1) begin
          if (m_env.m_dbg_mon.vif.pop_current_sk[7:0] != focus_search_key[7:0]) begin
            saw_overlap = 1'b1;
          end
          if (m_env.m_dbg_mon.pop_search_wait_cnt > wait_max) begin
            wait_max = m_env.m_dbg_mon.pop_search_wait_cnt;
          end
        end
        if (m_env.m_dbg_mon.pop_partition_load != 0) begin
          saw_load = 1'b1;
        end
        @(posedge m_env.m_csr_drv.vif.clk);
        wait_min++;
      end
      if (saw_overlap) begin
        `uvm_error("E115", $sformatf(
          "SEARCH key changed while the mid-SEARCH latency rewrite was in flight: active_key=%0d expected_focus=%0d",
          m_env.m_dbg_mon.vif.pop_current_sk[7:0], focus_search_key[7:0]))
      end
      if (!saw_load || wait_max != 5) begin
        `uvm_error("E115", $sformatf(
          "SEARCH guard tail changed after the mid-SEARCH latency rewrite: saw_load=%0d wait_max=%0d",
          saw_load, wait_max))
      end

      wait_for_subheader_match(
        focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 160_000,
        "E115 pre-rewrite single-hit subheader", matched_subheader);

      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      cmd_after = m_cfg.lane_key_ord_to_search_key(4);
      single_seq = single_push_pop_seq::type_id::create("e115_postsearch_latency");
      single_seq.search_key = cmd_after;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(2, 10_000, "E115 post-rewrite push");
      cycle_b = m_env.m_dbg_mon.sampled_cycles;
      wait_min = 0;
      while (wait_min < 2_048 &&
             m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        wait_min++;
      end
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count == cmd_before) begin
        `uvm_error("E115", "Mid-SEARCH latency rewrite to 5 did not accelerate the next descriptor window")
      end
      wait_min = int'(m_env.m_dbg_mon.sampled_cycles - cycle_b);
      if (wait_min >= search_cycles || wait_min > 256) begin
        `uvm_error("E115", $sformatf(
          "Mid-SEARCH rewrite did not shorten the next descriptor window enough: before=%0d after=%0d",
          search_cycles, wait_min))
      end
      wait_for_subheader_match(
        cmd_after[7:0], 8'd1, 1'b1, 1'b1, 120_000,
        "E115 post-rewrite single-hit subheader", matched_subheader);
      wait_for_scoreboard_idle(120_000, "E115 mid-SEARCH latency rewrite drain");
      expect_service_model_accounting("E115 mid-SEARCH latency rewrite drain", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 2 || pop_count != 2 || overwrite_count != 0 || fill_level != 0) begin
        `uvm_error("E115", $sformatf(
          "Mid-SEARCH latency rewrite accounting mismatch: push=%0d pop=%0d overwrite=%0d fill=%0d",
          push_count, pop_count, overwrite_count, fill_level))
      end
    end else if (case_id == "E116") begin
      configure_and_start(2000);
      cmd_before = m_env.m_dbg_mon.pop_cmd_wrreq_count;
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      hit_before = m_env.m_out_mon.total_hits_seen;
      wait_clocks(64);
      if (m_env.m_dbg_mon.pop_cmd_wrreq_count != cmd_before ||
          m_env.m_out_mon.total_subheaders_seen != subhdr_before) begin
        `uvm_error("E116", $sformatf(
          "No-push generator case emitted traffic before the latency rewrite: cmd_before=%0d cmd_after=%0d subhdr_before=%0d subhdr_after=%0d",
          cmd_before, m_env.m_dbg_mon.pop_cmd_wrreq_count,
          subhdr_before, m_env.m_out_mon.total_subheaders_seen))
      end

      csr_write(CSR_EXPECTED_LAT_ADDR, 32'd128);
      wait_clocks(4);
      if (m_env.m_dbg_mon.expected_latency_48b[15:0] != 16'd128) begin
        `uvm_error("E116", $sformatf(
          "No-push latency rewrite to 128 did not reach the DUT: observed=%0d",
          m_env.m_dbg_mon.expected_latency_48b[15:0]))
      end

      cycle_a = m_env.m_dbg_mon.sampled_cycles;
      wait_for_subheader_count(subhdr_before + 1, 1_024, "E116 zero-hit post-rewrite subheader");
      wait_min = int'(m_env.m_dbg_mon.sampled_cycles - cycle_a);
      if (wait_min > 512) begin
        `uvm_error("E116", $sformatf(
          "No-push latency rewrite did not advance the zero-hit descriptor soon enough: wait=%0d cycles",
          wait_min))
      end
      if (m_env.m_out_mon.recent_subheaders.size() == 0) begin
        `uvm_error("E116", "No-push latency rewrite did not retain the zero-hit subheader")
      end else begin
        matched_subheader = m_env.m_out_mon.recent_subheaders[$];
        if (!(matched_subheader.sop && matched_subheader.eop) ||
            matched_subheader.hit_count != 8'd0) begin
          `uvm_error("E116", $sformatf(
            "No-push latency rewrite emitted malformed zero-hit framing: sop=%0d eop=%0d hit_count=%0d",
            matched_subheader.sop, matched_subheader.eop, matched_subheader.hit_count))
        end
      end
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      if (push_count != 0 || pop_count != 0 || overwrite_count != 0 || cache_miss_count != 0) begin
        `uvm_error("E116", $sformatf(
          "No-push latency rewrite changed the accounting counters: push=%0d pop=%0d overwrite=%0d cache_miss=%0d",
          push_count, pop_count, overwrite_count, cache_miss_count))
      end
      if (m_env.m_out_mon.total_hits_seen != hit_before) begin
        `uvm_error("E116", $sformatf(
          "No-push latency rewrite unexpectedly emitted hit data: hits_before=%0d hits_after=%0d",
          hit_before, m_env.m_out_mon.total_hits_seen))
      end
    end else if (case_id == "E120") begin
      configure_and_start(0);
      wait_for_pop_engine_state(3'd5, 80_000, "E120 sync RESET");
      while (m_env.m_dbg_mon.pop_engine_state_code == 3'd5) begin
        @(posedge m_env.m_csr_drv.vif.clk);
      end
      wait_for_pop_engine_state(3'd3, 40_000, "E120 fresh COUNT entry");
      subhdr_before = m_env.m_out_mon.total_subheaders_seen;
      reads_needed = (((partition_size + 15) / 16) * m_cfg.n_partitions) + 1;
      search_cycles = 0;
      while (search_cycles < (reads_needed + 4) &&
             m_env.m_dbg_mon.pop_engine_state_code == 3'd3) begin
        @(posedge m_env.m_csr_drv.vif.clk);
        search_cycles++;
      end
      if (m_env.m_dbg_mon.pop_engine_state_code == 3'd3) begin
        `uvm_error("E120", $sformatf(
          "Zero-hit COUNT phase did not complete near the expected budget: observed_at_least=%0d expected=%0d",
          search_cycles, reads_needed))
      end
      if (search_cycles < reads_needed || search_cycles > (reads_needed + 1)) begin
        `uvm_error("E120", $sformatf(
          "Zero-hit COUNT duration mismatch: observed=%0d expected=%0d..%0d",
          search_cycles, reads_needed, reads_needed + 1))
      end
      wait_for_subheader_count(subhdr_before + 1, 40_000, "E120 zero-hit subheader");
      if (m_env.m_out_mon.recent_subheaders.size() == 0) begin
        `uvm_error("E120", "Zero-hit COUNT timing case did not retain the subheader")
      end else begin
        matched_subheader = m_env.m_out_mon.recent_subheaders[$];
        if (matched_subheader.hit_count != 8'd0) begin
          `uvm_error("E120", $sformatf(
            "Zero-hit COUNT timing case emitted a non-zero subheader hit_count=%0d",
            matched_subheader.hit_count))
        end
      end
    end else if (case_id == "E119") begin
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);

      configure_and_start(16'd1);
      single_seq = single_push_pop_seq::type_id::create("e119_latency1");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "E119 latency=1 push");
      wait_for_pop_engine_state(3'd1, 20_000, "E119 SEARCH entry latency=1");
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
        `uvm_error("E119", $sformatf(
          "SEARCH guard mismatch at EXPECTED_LATENCY=1: saw_load=%0d search_cycles=%0d wait_min=%0d wait_max=%0d",
          saw_load, search_cycles, wait_min, wait_max))
      end
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 40_000,
        "E119 latency=1 subheader", matched_subheader);
      wait_for_scoreboard_idle(80_000, "E119 latency=1 drain");
      expect_service_model_accounting("E119 latency=1 drain", 1, 0);

      configure_and_start(16'd65535);
      single_seq = single_push_pop_seq::type_id::create("e119_latency65535");
      single_seq.search_key = focus_search_key;
      single_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(1, 10_000, "E119 latency=65535 push");
      wait_for_pop_engine_state(3'd1, 80_000, "E119 SEARCH entry latency=65535");
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
        `uvm_error("E119", $sformatf(
          "SEARCH guard mismatch at EXPECTED_LATENCY=65535: saw_load=%0d search_cycles=%0d wait_min=%0d wait_max=%0d",
          saw_load, search_cycles, wait_min, wait_max))
      end
      wait_for_subheader_match(
        focus_search_key[7:0], 8'd1, 1'b1, 1'b1, 120_000,
        "E119 latency=65535 subheader", matched_subheader);
      wait_for_scoreboard_idle(140_000, "E119 latency=65535 drain");
      expect_service_model_accounting("E119 latency=65535 drain", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 1 || pop_count != 1 || overwrite_count != 0 || fill_level != 0) begin
        `uvm_error("E119", $sformatf(
          "SEARCH guard audit left unexpected accounting after the rearmed max-latency phase: push=%0d pop=%0d overwrite=%0d fill=%0d",
          push_count, pop_count, overwrite_count, fill_level))
      end
    end else if (case_id == "E124") begin
      profile_traffic_seq e124_seq;
      int unsigned outstanding;
      int unsigned max_outstanding;
      int unsigned max_usedw;
      bit          saw_saturation;
      bit          traffic_done;

      configure_and_start(128);
      max_outstanding = 0;
      max_usedw = 0;
      saw_saturation = 1'b0;
      traffic_done = 1'b0;
      fork
        begin : e124_traffic
          e124_seq = profile_traffic_seq::type_id::create("e124_fixed_point");
          e124_seq.num_hits = 15_000;
          e124_seq.lane_key_start_ord = 2;
          e124_seq.pool_keys = 1;
          e124_seq.hits_per_key_switch = 1;
          e124_seq.inter_hit_gap_cycles = 16;
          e124_seq.progress_stride = 3_750;
          e124_seq.progress_tag = "E124";
          e124_seq.start(m_env.m_hit_seqr);
          traffic_done = 1'b1;
        end
        begin : e124_monitor
          search_cycles = 0;
          while (search_cycles < 2_500_000 &&
                 (!traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
            outstanding = m_env.m_dbg_mon.pop_cmd_wrreq_count -
                          m_env.m_dbg_mon.pop_cmd_rdack_count;
            if (outstanding > max_outstanding)
              max_outstanding = outstanding;
            if (int'(m_env.m_dbg_mon.pop_cmd_fifo_usedw) > max_usedw)
              max_usedw = int'(m_env.m_dbg_mon.pop_cmd_fifo_usedw);
            if (int'(m_env.m_dbg_mon.pop_cmd_fifo_usedw) >= 14)
              saw_saturation = 1'b1;
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
        end
      join

      wait_for_scoreboard_idle(2_500_000, "E124 fixed-point descriptor backlog drain");
      expect_service_model_accounting("E124 fixed-point descriptor backlog drain", 1, 0);
      if (!saw_saturation) begin
        `uvm_error("E124", $sformatf(
          "Low-latency descriptor stress never reached the fifo saturation band: max_usedw=%0d",
          max_usedw))
      end
      if (max_outstanding > 16) begin
        `uvm_error("E124", $sformatf(
          "Descriptor fifo outstanding count exceeded the 16-entry depth: max_outstanding=%0d max_usedw=%0d wrreq=%0d rdack=%0d",
          max_outstanding, max_usedw,
          m_env.m_dbg_mon.pop_cmd_wrreq_count,
          m_env.m_dbg_mon.pop_cmd_rdack_count))
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
    end else if (case_id == "P005") begin
      run_frequent_terminate_mixed_profile_case(
        "P005 mixed terminate/restart hotspot profile",
        4,
        250_000);
    end else if (case_id == "P006") begin
      run_config_aware_encoder_stress_case(
        "P006 configuration-aware encoder stress profile",
        3_000_000);
    end else if (case_id == "P007") begin
      run_profile_integrity_case(
        "P007 low-fill single-key",
        16'd128, 4_096, 2, 1, 1,
        31, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 500_000,
        0, 0, 1, 1'b1);
    end else if (case_id == "P008") begin
      run_profile_integrity_case(
        "P008 medium-fill single-key",
        16'd128, 4_096, 2, 1, 1,
        15, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 500_000,
        0, 0, 1, 1'b1);
    end else if (case_id == "P009") begin
      run_profile_integrity_case(
        "P009 high-fill single-key",
        16'd128, 4_096, 2, 1, 1,
        13, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 500_000,
        0, 0, 1, 1'b1);
    end else if (case_id == "P010") begin
      run_profile_integrity_case(
        "P010 two-key round-robin",
        16'd128, 4_096, 2, 2, 1,
        11, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 500_000,
        0, 0, 2, 1'b1);
    end else if (case_id == "P011") begin
      run_profile_integrity_case(
        "P011 four-key round-robin",
        16'd128, 4_096, 2, 4, 1,
        13, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 500_000,
        0, 0, 4, 1'b1);
    end else if (case_id == "P012") begin
      run_profile_integrity_case(
        "P012 eight-key round-robin",
        16'd128, 4_096, 2, 8, 1,
        13, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 500_000,
        0, 0, 8, 1'b1);
    end else if (case_id == "P013") begin
      run_profile_integrity_case(
        "P013 sixteen-key round-robin",
        16'd128, 4_096, 2, 16, 1,
        13, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 500_000,
        0, 0, 16, 1'b1, 32'h1ace_b00c, 4'b0000, 8);
    end else if (case_id == "P014") begin
      run_profile_integrity_case(
        "P014 full lane-local keyspace",
        16'd128, 4_096, 0, 64, 1,
        15, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 900_000,
        0, 0, 64, 1'b1);
    end else if (case_id == "P015") begin
      run_profile_integrity_case(
        "P015 burst-gap four-key",
        16'd128, 4_096, 2, 4, 1,
        0, 0, 0, 1'b0, 1'b0, 8'd0,
        16, 176, 0, 600_000,
        0, 0, 8, 1'b1);
    end else if (case_id == "P016") begin
      run_profile_integrity_case(
        "P016 bernoulli-lambda-0p5",
        16'd128, 4_096, 2, 4, 1,
        0, 0, 0, 1'b1, 1'b1, 8'd24,
        0, 0, 0, 600_000,
        0, 0, 8, 1'b1);
    end else if (case_id == "P017") begin
      run_profile_integrity_case(
        "P017 bernoulli-lambda-0p9",
        16'd128, 4_096, 2, 4, 1,
        0, 0, 0, 1'b1, 1'b1, 8'd28,
        0, 0, 0, 600_000,
        0, 0, 8, 1'b1);
    end else if (case_id == "P018") begin
      run_profile_integrity_case(
        "P018 fixed-75-ready",
        16'd128, 4_096, 2, 4, 1,
        15, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 1, 700_000,
        0, 0, 4, 1'b1);
    end else if (case_id == "P019") begin
      run_profile_integrity_case(
        "P019 fixed-50-ready",
        16'd128, 4_096, 2, 4, 1,
        23, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 2, 900_000,
        0, 0, 4, 1'b1);
    end else if (case_id == "P020") begin
      run_profile_integrity_case(
        "P020 prng-70-ready",
        16'd128, 4_096, 2, 4, 1,
        17, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 3, 800_000,
        0, 0, 8, 1'b1);
    end else if (case_id == "P021") begin
      latency_choice = $urandom_range(0, 2);
      case (latency_choice)
        0: latency_choice = 1;
        1: latency_choice = 128;
        default: latency_choice = 2000;
      endcase
      run_profile_integrity_case(
        $sformatf("P021 latency-choice-%0d", latency_choice),
        latency_choice, 4_096, 2, 4, 1,
        15, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 1_200_000,
        0, 0, 4, 1'b1);
      if (latency_choice == 1 && m_env.m_dbg_mon.push_count_at_first_pop > 4) begin
        `uvm_error("P021", $sformatf(
          "Min-latency selection admitted too many pushes before first pop: observed=%0d expected_at_most=4",
          m_env.m_dbg_mon.push_count_at_first_pop))
      end
      if (latency_choice == 128 &&
          (m_env.m_dbg_mon.push_count_at_first_pop < 8 ||
           m_env.m_dbg_mon.push_count_at_first_pop > 24)) begin
        `uvm_error("P021", $sformatf(
          "Default-latency selection escaped the expected pre-service window: observed=%0d expected=[8,24]",
          m_env.m_dbg_mon.push_count_at_first_pop))
      end
      if (latency_choice == 2000 &&
          (m_env.m_dbg_mon.push_count_at_first_pop < 96 ||
           m_env.m_dbg_mon.push_count_at_first_pop > 160)) begin
        `uvm_error("P021", $sformatf(
          "Long-latency selection escaped the expected pre-service window: observed=%0d expected=[96,160]",
          m_env.m_dbg_mon.push_count_at_first_pop))
      end
    end else if (case_id == "P022") begin
      run_profile_integrity_case(
        "P022 min-latency profile",
        16'd1, 4_096, 2, 4, 1,
        15, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 500_000,
        0, 0, 4, 1'b1);
      if (m_env.m_dbg_mon.first_pop_cycle == 0 ||
          m_env.m_dbg_mon.push_count_at_first_pop > 24) begin
        `uvm_error("P022", $sformatf(
          "Min-latency profile delayed the first pop too much: first_pop=%0d pushes_before_first_pop=%0d expected_pushes_at_most=24",
          m_env.m_dbg_mon.first_pop_cycle, m_env.m_dbg_mon.push_count_at_first_pop))
      end
    end else if (case_id == "P023") begin
      run_profile_integrity_case(
        "P023 long-latency profile",
        16'd2000, 4_096, 2, 4, 1,
        15, 0, 0, 1'b0, 1'b0, 8'd0,
        0, 0, 0, 1_200_000,
        0, 0, 4, 1'b1);
      if (m_env.m_dbg_mon.first_pop_cycle == 0 ||
          m_env.m_dbg_mon.push_count_at_first_pop < 96) begin
        `uvm_error("P023", $sformatf(
          "Long-latency profile did not hold enough traffic before first pop: first_pop=%0d pushes_before_first_pop=%0d expected_at_least=96",
          m_env.m_dbg_mon.first_pop_cycle, m_env.m_dbg_mon.push_count_at_first_pop))
      end
    end else if (case_id == "P024") begin
      run_profile_exact_depth_boundary_case(
        "P024 exact-depth boundary",
        16'd2000, 2);
    end else if (case_id == "P025") begin
      run_single_key_fixed_point_profile_case(
        "P025 steady-state single-key fixed-point",
        16'd128,
        50_000,
        2,
        16,
        2_500_000,
        128,
        384);
    end else if (case_id == "P026") begin
      run_weighted_multi_key_case(
        "P026 equal-weight four-key",
        16'd128, 2,
        1, 1, 1, 1,
        1024, 13, 0, 750_000);
    end else if (case_id == "P027") begin
      run_weighted_multi_key_case(
        "P027 skewed four-key",
        16'd128, 2,
        7, 2, 1, 1,
        256, 13, 0, 750_000);
    end else if (case_id == "P028") begin
      int unsigned lane_key_ords[$];
      int unsigned key_hits_per_epoch[$];
      for (int idx = 0; idx < 8; idx++) begin
        lane_key_ords.push_back(2 + idx);
      end
      key_hits_per_epoch.push_back(17145);
      key_hits_per_epoch.push_back(7463);
      key_hits_per_epoch.push_back(4588);
      key_hits_per_epoch.push_back(3248);
      key_hits_per_epoch.push_back(2485);
      key_hits_per_epoch.push_back(1997);
      key_hits_per_epoch.push_back(1660);
      key_hits_per_epoch.push_back(1414);
      run_dynamic_weighted_multi_key_case(
        "P028 Zipfian eight-key fairness profile",
        16'd128,
        lane_key_ords,
        key_hits_per_epoch,
        1,
        13,
        0,
        8_000_000);
    end else if (case_id == "P029") begin
      int unsigned lane_key_ords[$];
      int unsigned expected_hits[$];
      for (int idx = 0; idx < 4; idx++) begin
        lane_key_ords.push_back(2 + idx);
        expected_hits.push_back(5000);
      end
      run_multi_key_profile_case(
        "P029 deterministic four-key round-robin",
        16'd128,
        20_000,
        2,
        4,
        1,
        1'b0,
        13,
        4_000_000,
        32'h0290_0001);
      check_exact_key_counts(
        "P029 deterministic four-key round-robin",
        lane_key_ords,
        expected_hits,
        4);
    end else if (case_id == "P030") begin
      run_weighted_multi_key_case(
        "P030 rotating 256-hit bursts",
        16'd128, 2,
        256, 256, 256, 256,
        2, 13, 176, 1_500_000, 48);
    end else if (case_id == "P031") begin
      run_adversarial_overlap_profile_case(
        "P031 adversarial two-key alternation",
        16'd128,
        2,
        3,
        10_000,
        11,
        2_000_000);
    end else if (case_id == "P032") begin
      run_randomized_multi_key_integrity_case(
        "P032 randomized eight-key reuse-50",
        16'd128, 4096, 2, 8, 13, 1_000_000, 48);
    end else if (case_id == "P033") begin
      run_randomized_multi_key_integrity_case(
        "P033 randomized sixteen-key reuse-80",
        16'd128, 4096, 2, 16, 13, 1_000_000, 8);
    end else if (case_id == "P034") begin
      int unsigned active_lane_key_ords[$];
      int unsigned silent_lane_key_ords[$];
      active_lane_key_ords.push_back(2);
      active_lane_key_ords.push_back(3);
      active_lane_key_ords.push_back(4);
      silent_lane_key_ords.push_back(5);
      run_silent_key_profile_case(
        "P034 one silent key amid three active",
        16'd128, 3072, 13,
        active_lane_key_ords, silent_lane_key_ords, 1_000_000);
    end else if (case_id == "P035") begin
      int unsigned active_lane_key_ords[$];
      int unsigned silent_lane_key_ords[$];
      active_lane_key_ords.push_back(2);
      active_lane_key_ords.push_back(4);
      active_lane_key_ords.push_back(6);
      silent_lane_key_ords.push_back(3);
      silent_lane_key_ords.push_back(5);
      run_silent_key_profile_case(
        "P035 interleaved active-silent five-key window",
        16'd128, 3072, 13,
        active_lane_key_ords, silent_lane_key_ords, 1_000_000);
    end else if (case_id == "P036") begin
      run_multi_key_profile_case(
        "P036 seeded random four-key profile A",
        16'd128,
        30_000,
        2,
        4,
        1,
        1'b1,
        13,
        5_000_000,
        32'h0360_1a2b);
    end else if (case_id == "P037") begin
      run_multi_key_profile_case(
        "P037 seeded random four-key profile B",
        16'd128,
        30_000,
        2,
        4,
        1,
        1'b1,
        13,
        5_000_000,
        32'h0370_3c4d);
    end else if (case_id == "P038") begin
      run_multi_key_profile_case(
        "P038 seeded random four-key profile C",
        16'd128,
        30_000,
        2,
        4,
        1,
        1'b1,
        13,
        5_000_000,
        32'h0380_5e6f);
    end else if (case_id == "P039") begin
      run_weighted_multi_key_case(
        "P039 heavy-key 2x fairness profile",
        16'd128, 2,
        2, 1, 1, 1,
        1024, 13, 0, 1_000_000);
    end else if (case_id == "P040") begin
      run_multi_key_profile_case(
        "P040 seeded random four-key integrity audit",
        16'd128,
        20_000,
        2,
        4,
        1,
        1'b1,
        13,
        4_000_000,
        32'h0400_f10f);
    end else if (case_id == "P041") begin
      run_profile_integrity_case(
        "P041 four-key random backpressure profile",
        16'd128,
        30_000,
        2,
        4,
        1,
        13,
        0,
        0,
        1'b1,
        1'b0,
        8'd0,
        0,
        0,
        3,
        6_000_000,
        128,
        0,
        64,
        1'b1,
        32'h0410_70af);
      check_per_key_drain_conservation(
        "P041 four-key random backpressure profile",
        2,
        4,
        4);
    end else if (case_id == "P042") begin
      run_staggered_multi_key_profile_case(
        "P042 eight-key staged late-arrival profile",
        16'd128,
        15_000,
        15_000,
        2,
        4,
        6,
        4,
        13,
        32,
        6_000_000,
        32'h0420_1a2b,
        32'h0420_3c4d,
        448);
    end else if (case_id == "P043") begin
      run_latency_reprogrammed_multi_key_case(
        "P043 eight-key latency sweep profile",
        16'd128,
        30_000,
        2,
        8,
        13,
        8_000_000,
        32'h0430_5e6f,
        768);
    end else if (case_id == "P044") begin
      int unsigned lane_key_ords[$];
      int unsigned expected_hits[$];
      for (int idx = 0; idx < 4; idx++) begin
        lane_key_ords.push_back(2 + idx);
        expected_hits.push_back(10_000);
      end
      run_multi_key_profile_case(
        "P044 long four-key continuous reuse profile",
        16'd128,
        40_000,
        2,
        4,
        1,
        1'b0,
        13,
        6_000_000,
        32'h0440_0001);
      check_exact_key_counts(
        "P044 long four-key continuous reuse profile",
        lane_key_ords,
        expected_hits,
        4);
    end else if (case_id == "P045") begin
      run_multi_key_profile_case(
        "P045 random sixteen-key arbitration profile",
        16'd128,
        50_000,
        2,
        16,
        1,
        1'b1,
        13,
        8_000_000,
        32'h0450_7f91,
        256);
    end else if (case_id == "P046") begin
      run_partition_local_profile_case(
        "P046 default-build partition0-local profile",
        16'd2000,
        1,
        0,
        2,
        partition_size - (partition_size / 4),
        4'b0001,
        2_000_000);
    end else if (case_id == "P047") begin
      if (m_cfg.n_partitions < 2) begin
        `uvm_fatal("P047", "P047 requires at least two active partitions in the current build")
      end
      run_partition_local_profile_case(
        "P047 default-build partition handoff profile",
        16'd2000,
        1,
        partition_size,
        2,
        partition_size + (partition_size / 4),
        4'b0011,
        2_500_000,
        0);
    end else if (case_id == "P048") begin
      if (m_cfg.n_partitions != 4) begin
        `uvm_fatal("P048", "P048 requires the 4-partition p4 build variant")
      end
      run_partition_local_profile_case(
        "P048 p4 partition2-local profile",
        16'd2000,
        1,
        partition_size * 2,
        2,
        partition_size - (partition_size / 4),
        4'b0100,
        3_000_000);
    end else if (case_id == "P049") begin
      if (m_cfg.n_partitions != 4) begin
        `uvm_fatal("P049", "P049 requires the 4-partition p4 build variant")
      end
      run_partition_local_profile_case(
        "P049 p4 partition3-local profile",
        16'd2000,
        1,
        partition_size * 3,
        2,
        partition_size - (partition_size / 4),
        4'b1000,
        3_000_000);
    end else if (case_id == "P058") begin
      int unsigned expected_partition_hits[$];
      int unsigned staged_prefill_hits;
      int unsigned spill_hits;
      if (m_cfg.n_partitions < 2) begin
        `uvm_fatal("P058", "P058 requires at least two active partitions in the current build")
      end
      staged_prefill_hits = partition_size - (partition_size / 5);
      spill_hits = (partition_size - staged_prefill_hits) + 1;
      expected_partition_hits.push_back(partition_size - staged_prefill_hits);
      expected_partition_hits.push_back(1);
      for (int idx = 2; idx < m_cfg.n_partitions; idx++) begin
        expected_partition_hits.push_back(0);
      end
      run_staged_partition_share_profile_case(
        "P058 active-build partition handoff profile",
        16'd2000,
        1,
        staged_prefill_hits,
        2,
        spill_hits,
        expected_partition_hits,
        2_500_000,
        4'b0000,
        partition_size - staged_prefill_hits,
        200_000);
    end else if (case_id == "P050") begin
      int unsigned expected_partition_hits[$];
      for (int idx = 0; idx < m_cfg.n_partitions; idx++) begin
        expected_partition_hits.push_back(partition_size);
      end
      run_partition_share_profile_case(
        "P050 active-build partition balance profile",
        16'd2000,
        2,
        m_cfg.ring_buffer_n_entry,
        expected_partition_hits,
        3_000_000);
    end else if (case_id == "P051") begin
      int unsigned expected_partition_hits[$];
      int unsigned target_hits;
      if (m_cfg.n_partitions == 2) begin
        expected_partition_hits.push_back(partition_size);
        expected_partition_hits.push_back((partition_size + 7) / 8);
      end else begin
        expected_partition_hits.push_back(partition_size);
        for (int idx = 1; idx < m_cfg.n_partitions; idx++) begin
          expected_partition_hits.push_back((partition_size + 6) / 7);
        end
      end
      target_hits = 0;
      foreach (expected_partition_hits[idx]) begin
        target_hits += expected_partition_hits[idx];
      end
      run_partition_share_profile_case(
        "P051 active-build partition skew profile",
        16'd2000,
        2,
        target_hits,
        expected_partition_hits,
        2_500_000);
    end else if (case_id == "P052") begin
      int unsigned expected_partition_hits[$];
      int unsigned target_hits;
      if (m_cfg.n_partitions == 2) begin
        expected_partition_hits.push_back(partition_size);
        expected_partition_hits.push_back(partition_size - 1);
      end else begin
        expected_partition_hits.push_back(partition_size);
        expected_partition_hits.push_back(partition_size);
        for (int idx = 2; idx < m_cfg.n_partitions; idx++) begin
          expected_partition_hits.push_back(partition_size / 4);
        end
      end
      target_hits = 0;
      foreach (expected_partition_hits[idx]) begin
        target_hits += expected_partition_hits[idx];
      end
      run_partition_share_profile_case(
        "P052 active-build partition contention profile",
        16'd2000,
        2,
        target_hits,
        expected_partition_hits,
        3_000_000);
    end else if (case_id == "P053") begin
      int unsigned expected_partition_hits[$];
      int unsigned target_hits;
      expected_partition_hits.push_back(partition_size);
      expected_partition_hits.push_back(1);
      for (int idx = 2; idx < m_cfg.n_partitions; idx++) begin
        expected_partition_hits.push_back(0);
      end
      target_hits = 0;
      foreach (expected_partition_hits[idx]) begin
        target_hits += expected_partition_hits[idx];
      end
      run_partition_share_profile_case(
        "P053 active-build adversarial partition skew profile",
        16'd2000,
        2,
        target_hits,
        expected_partition_hits,
        2_500_000);
    end else if (case_id == "P054") begin
      int unsigned expected_partition_hits[$];
      run_pipe_stage_latency_smoke_case(
        "P054 PIPE_STAGES=1 balanced profile smoke",
        "P054",
        1,
        2,
        80_000);
      for (int idx = 0; idx < m_cfg.n_partitions; idx++) begin
        expected_partition_hits.push_back(partition_size);
      end
      run_partition_share_profile_case(
        "P054 pipe1 balanced partition profile",
        16'd2000,
        2,
        m_cfg.ring_buffer_n_entry,
        expected_partition_hits,
        3_000_000);
    end else if (case_id == "P055") begin
      int unsigned expected_partition_hits[$];
      run_pipe_stage_latency_smoke_case(
        "P055 PIPE_STAGES=2 balanced profile smoke",
        "P055",
        2,
        2,
        80_000);
      for (int idx = 0; idx < m_cfg.n_partitions; idx++) begin
        expected_partition_hits.push_back(partition_size);
      end
      run_partition_share_profile_case(
        "P055 pipe2 balanced partition profile",
        16'd2000,
        2,
        m_cfg.ring_buffer_n_entry,
        expected_partition_hits,
        3_000_000);
    end else if (case_id == "P056") begin
      int unsigned expected_partition_hits[$];
      run_pipe_stage_latency_smoke_case(
        "P056 PIPE_STAGES=3 balanced profile smoke",
        "P056",
        3,
        2,
        80_000);
      for (int idx = 0; idx < m_cfg.n_partitions; idx++) begin
        expected_partition_hits.push_back(partition_size);
      end
      run_partition_share_profile_case(
        "P056 pipe3 balanced partition profile",
        16'd2000,
        2,
        m_cfg.ring_buffer_n_entry,
        expected_partition_hits,
        3_000_000);
    end else if (case_id == "P057") begin
      int unsigned expected_partition_hits[$];
      run_pipe_stage_latency_smoke_case(
        "P057 PIPE_STAGES=4 balanced profile smoke",
        "P057",
        4,
        2,
        80_000);
      for (int idx = 0; idx < m_cfg.n_partitions; idx++) begin
        expected_partition_hits.push_back(partition_size);
      end
      run_partition_share_profile_case(
        "P057 pipe4 balanced partition profile",
        16'd2000,
        2,
        m_cfg.ring_buffer_n_entry,
        expected_partition_hits,
        3_000_000);
    end else if (case_id == "P059") begin
      run_profile_integrity_case(
        "P059 active-build partition saturation and recovery",
        16'd0,
        partition_size,
        2,
        1,
        1,
        0,
        0,
        0,
        1'b0,
        1'b0,
        8'd0,
        0,
        0,
        6,
        1_200_000,
        partition_size - (partition_size / 8),
        partition_size,
        1,
        1'b1,
        32'h0590_0001,
        4'b0001);
    end else if (case_id == "P060") begin
      expected_mask = (1 << m_cfg.n_partitions) - 1;
      run_profile_integrity_case(
        "P060 active-build encoder-stall 30pct-ready profile",
        16'd0,
        768,
        2,
        1,
        1,
        23,
        0,
        0,
        1'b0,
        1'b0,
        8'd0,
        0,
        0,
        4,
        2_500_000,
        partition_size / 2,
        m_cfg.ring_buffer_n_entry,
        6,
        1'b1,
        32'h0600_30aa,
        expected_mask[3:0]);
    end else if (case_id == "P061") begin
      int unsigned expected_partition_hits[$];
      expected_mask = (1 << m_cfg.n_partitions) - 1;
      for (int idx = 0; idx < m_cfg.n_partitions; idx++) begin
        expected_partition_hits.push_back(partition_size);
      end
      run_staged_partition_share_profile_case(
        "P061 active-build cross-partition round-robin push profile",
        16'd2000,
        0,
        0,
        2,
        m_cfg.ring_buffer_n_entry,
        expected_partition_hits,
        3_000_000,
        expected_mask[3:0]);
    end else if (case_id == "P062") begin
      int unsigned expected_partition_hits[$];
      int unsigned target_hits;
      if (m_cfg.n_partitions < 2) begin
        `uvm_fatal("P062", "P062 requires at least two active partitions in the current build")
      end
      expected_partition_hits.push_back(partition_size);
      expected_partition_hits.push_back(partition_size / 2);
      for (int idx = 2; idx < m_cfg.n_partitions; idx++) begin
        expected_partition_hits.push_back(0);
      end
      target_hits = 0;
      foreach (expected_partition_hits[idx]) begin
        target_hits += expected_partition_hits[idx];
      end
      run_staged_partition_share_profile_case(
        "P062 active-build cross-partition burst profile",
        16'd2000,
        0,
        0,
        2,
        target_hits,
        expected_partition_hits,
        3_000_000,
        4'b0000,
        partition_size,
        200_000);
    end else if (case_id == "P063") begin
      int unsigned expected_partition_hits[$];
      int unsigned staged_prefill_hits;
      if (m_cfg.n_partitions != 2) begin
        `uvm_fatal("P063", "P063 is calibrated for the default two-partition build")
      end
      expected_partition_hits.push_back(1);
      expected_partition_hits.push_back(1);
      staged_prefill_hits = partition_size - 1;
      run_staged_partition_share_profile_case(
        "P063 sparse active-partition endpoint alternation profile",
        16'd2000,
        1,
        staged_prefill_hits,
        2,
        2,
        expected_partition_hits,
        2_500_000,
        4'b0000,
        1,
        200_000);
    end else if (case_id == "P064") begin
      expected_mask = (1 << m_cfg.n_partitions) - 1;
      run_profile_integrity_case(
        "P064 active-build time-varying backpressure partition profile",
        16'd0,
        1024,
        2,
        1,
        1,
        19,
        0,
        0,
        1'b0,
        1'b0,
        8'd0,
        0,
        0,
        5,
        3_000_000,
        partition_size / 2,
        m_cfg.ring_buffer_n_entry,
        6,
        1'b1,
        32'h0640_5eed,
        expected_mask[3:0]);
    end else if (case_id == "P066") begin
      profile_traffic_seq term_seq;
      int unsigned term_inject_cycles;
      int unsigned accepted_at_endofrun;
      int unsigned offered_at_endofrun;
      int unsigned push_count;
      int unsigned pop_count;
      int unsigned overwrite_count;
      int unsigned cache_miss_count;
      int unsigned fill_level;
      int unsigned post_eor_cycles;
      bit          p066_traffic_done;
      bit          saw_post_endofrun_offers;

      configure_and_start(16'd128);
      p066_traffic_done = 1'b0;
      term_inject_cycles = 1_000 + ((get_dv_seed() * 97) % 8_001);

      fork
        begin : p066_traffic_thread
          term_seq = profile_traffic_seq::type_id::create("p066_uniform_term");
          term_seq.num_hits = 1_024;
          term_seq.lane_key_start_ord = 2;
          term_seq.pool_keys = 4;
          term_seq.hits_per_key_switch = 1;
          term_seq.randomize_key_order = 1'b0;
          term_seq.inter_hit_gap_cycles = 13;
          term_seq.progress_stride = 256;
          term_seq.progress_tag = "P066 mid-run terminate profile";
          term_seq.start(m_env.m_hit_seqr);
          p066_traffic_done = 1'b1;
        end
        begin : p066_term_thread
          wait_clocks(term_inject_cycles);
          ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
          wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 20_000, "P066 TERMINATING entry");
          send_endofrun_marker();
          wait_clocks(2);
          if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
            `uvm_error("P066", "endofrun_seen did not latch after the mid-run TERMINATING marker")
          end

          accepted_at_endofrun = m_env.m_hit_drv.accepted_payload_total;
          offered_at_endofrun = m_env.m_hit_drv.offered_payload_total;
          saw_post_endofrun_offers = 1'b0;
          post_eor_cycles = 0;
          while (post_eor_cycles < 512 &&
                 (!p066_traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
            @(posedge m_env.m_csr_drv.vif.clk);
            post_eor_cycles++;
            if (m_env.m_hit_drv.offered_payload_total > offered_at_endofrun) begin
              saw_post_endofrun_offers = 1'b1;
            end
            if (m_env.m_hit_drv.accepted_payload_total > accepted_at_endofrun) begin
              `uvm_error("P066", $sformatf(
                "Accepted payload advanced after endofrun_seen latched: accepted_before=%0d accepted_now=%0d offered_now=%0d pending_source=%0d",
                accepted_at_endofrun,
                m_env.m_hit_drv.accepted_payload_total,
                m_env.m_hit_drv.offered_payload_total,
                m_env.m_hit_drv.pending_source_items()))
              accepted_at_endofrun = m_env.m_hit_drv.accepted_payload_total;
            end
          end
          if (!saw_post_endofrun_offers) begin
            `uvm_error("P066", $sformatf(
              "Mid-run TERMINATING never observed additional offered traffic after endofrun_seen: offered_at_eor=%0d final_offered=%0d",
              offered_at_endofrun,
              m_env.m_hit_drv.offered_payload_total))
          end

          ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
        end
      join

      discard_pending_source_backlog("P066 dropped post-endofrun backlog");
      wait_clocks(2);
      wait_for_scoreboard_idle(200_000, "P066 mid-run terminate cleanup");
      expect_service_model_accounting("P066 mid-run terminate cleanup", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != m_env.m_hit_drv.accepted_payload_total) begin
        `uvm_error("P066", $sformatf(
          "PUSH_COUNT disagreed with accepted payload total after mid-run TERMINATING: push=%0d accepted=%0d pop=%0d overwrite=%0d",
          push_count,
          m_env.m_hit_drv.accepted_payload_total,
          pop_count,
          overwrite_count))
      end
      if (cache_miss_count != 0 || fill_level != 0) begin
        `uvm_error("P066", $sformatf(
          "Mid-run TERMINATING left unexpected residual accounting: fill=%0d cache_miss=%0d push=%0d pop=%0d overwrite=%0d",
          fill_level, cache_miss_count, push_count, pop_count, overwrite_count))
      end
    end else if (case_id == "P067") begin
      profile_traffic_seq flush_seq;
      int unsigned accepted_before_flush;
      int unsigned accepted_after_flush;
      int unsigned push_count;
      int unsigned pop_count;
      int unsigned overwrite_count;
      int unsigned cache_miss_count;
      int unsigned fill_level;
      int unsigned hits_after_flush;
      int unsigned hits_after_flush_wait;
      bit          stop_requested;
      bit          p067_traffic_done;
      int unsigned burst_idx;

      configure_and_start(16'd128);
      stop_requested = 1'b0;
      p067_traffic_done = 1'b0;

      fork
        begin : p067_traffic_thread
          burst_idx = 0;
          while (!stop_requested && burst_idx < 256) begin
            flush_seq = profile_traffic_seq::type_id::create($sformatf("p067_uniform_flush_%0d", burst_idx));
            flush_seq.num_hits = 64;
            flush_seq.lane_key_start_ord = 4;
            flush_seq.pool_keys = 4;
            flush_seq.hits_per_key_switch = 1;
            flush_seq.randomize_key_order = 1'b0;
            flush_seq.inter_hit_gap_cycles = 13;
            flush_seq.fingerprint_start_index = burst_idx * 64;
            flush_seq.progress_stride = 0;
            flush_seq.start(m_env.m_hit_seqr);
            burst_idx++;
          end
          p067_traffic_done = 1'b1;
        end
        begin : p067_flush_thread
          wait_for_accepted_payload_count(1_024, 2_000_000, "P067 pre-FLUSH accepted threshold");
          stop_requested = 1'b1;
          accepted_before_flush = m_env.m_hit_drv.accepted_payload_total;
          enter_run_prepare();
          accepted_after_flush = m_env.m_hit_drv.accepted_payload_total;
          hits_after_flush = m_env.m_out_mon.total_hits_seen;
          wait_clocks(64);
          hits_after_flush_wait = m_env.m_out_mon.total_hits_seen;
        end
      join

      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 0 || pop_count != 0 || overwrite_count != 0 ||
          cache_miss_count != 0 || fill_level != 0) begin
        `uvm_error("P067", $sformatf(
          "Mid-run FLUSH did not clear the DUT accounting state: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d accepted_before=%0d accepted_after=%0d",
          push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
          accepted_before_flush, accepted_after_flush))
      end
      if (m_env.m_scb.remaining_entries() != 0 ||
          m_env.m_scb.pending_drain_entries() != 0) begin
        `uvm_error("P067", $sformatf(
          "Mid-run FLUSH left scoreboard residual state: remaining=%0d pending_drain=%0d accepted_before=%0d accepted_after=%0d",
          m_env.m_scb.remaining_entries(), m_env.m_scb.pending_drain_entries(),
          accepted_before_flush, accepted_after_flush))
      end
      if (m_env.m_dbg_mon.vif.pop_cmd_fifo_empty !== 1'b1 ||
          m_env.m_dbg_mon.vif.deassembly_fifo_empty !== 1'b1) begin
        `uvm_error("P067", $sformatf(
          "Mid-run FLUSH left internal FIFOs non-empty: pop_cmd_empty=%0d deassm_empty=%0d pop_cmd_usedw=%0d deassm_usedw=%0d",
          m_env.m_dbg_mon.vif.pop_cmd_fifo_empty,
          m_env.m_dbg_mon.vif.deassembly_fifo_empty,
          m_env.m_dbg_mon.vif.pop_cmd_fifo_usedw,
          m_env.m_dbg_mon.vif.deassembly_fifo_usedw))
      end
      if (hits_after_flush_wait != hits_after_flush) begin
        `uvm_error("P067", $sformatf(
          "Mid-run FLUSH leaked output activity after the flush-complete handshake: hits_after_flush=%0d hits_after_wait=%0d",
          hits_after_flush, hits_after_flush_wait))
      end
      if (!p067_traffic_done) begin
        `uvm_error("P067", "Traffic thread did not quiesce after the stop request")
      end

      m_env.m_hit_drv.pending_q.delete();
      wait_clocks(2);
      return_to_idle();
    end else if (case_id == "P068") begin
      profile_traffic_seq term_seq;
      profile_traffic_seq final_seq;
      int unsigned term_inject_cycles;
      int unsigned accepted_at_endofrun;
      int unsigned offered_at_endofrun;
      int unsigned push_count;
      int unsigned pop_count;
      int unsigned overwrite_count;
      int unsigned cache_miss_count;
      int unsigned fill_level;
      int unsigned post_eor_cycles;
      int unsigned accepted_after_restart_checkpoint;
      bit          p068_traffic_done;
      bit          saw_post_endofrun_offers;

      configure_and_start(16'd128);
      p068_traffic_done = 1'b0;
      term_inject_cycles = 1_000 + ((get_dv_seed() * 193) % 8_001);

      fork
        begin : p068_traffic_thread
          term_seq = profile_traffic_seq::type_id::create("p068_uniform_term");
          term_seq.num_hits = 1_024;
          term_seq.lane_key_start_ord = 4;
          term_seq.pool_keys = 4;
          term_seq.hits_per_key_switch = 1;
          term_seq.randomize_key_order = 1'b0;
          term_seq.inter_hit_gap_cycles = 13;
          term_seq.progress_stride = 256;
          term_seq.progress_tag = "P068 mid-run terminate profile";
          term_seq.start(m_env.m_hit_seqr);
          p068_traffic_done = 1'b1;
        end
        begin : p068_term_thread
          wait_clocks(term_inject_cycles);
          ctrl_pulse_raw(ring_buffer_cam_pkg::CTRL_TERMINATING);
          wait_for_run_state(ring_buffer_cam_pkg::RUN_STATE_TERMINATING, 20_000, "P068 TERMINATING entry");
          send_endofrun_marker();
          wait_clocks(2);
          if (m_env.m_dbg_mon.endofrun_seen !== 1'b1) begin
            `uvm_error("P068", "endofrun_seen did not latch after the mid-run TERMINATING marker")
          end

          accepted_at_endofrun = m_env.m_hit_drv.accepted_payload_total;
          offered_at_endofrun = m_env.m_hit_drv.offered_payload_total;
          saw_post_endofrun_offers = 1'b0;
          post_eor_cycles = 0;
          while (post_eor_cycles < 512 &&
                 (!p068_traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
            @(posedge m_env.m_csr_drv.vif.clk);
            post_eor_cycles++;
            if (m_env.m_hit_drv.offered_payload_total > offered_at_endofrun) begin
              saw_post_endofrun_offers = 1'b1;
            end
            if (m_env.m_hit_drv.accepted_payload_total > accepted_at_endofrun) begin
              `uvm_error("P068", $sformatf(
                "Accepted payload advanced after endofrun_seen latched: accepted_before=%0d accepted_now=%0d offered_now=%0d pending_source=%0d",
                accepted_at_endofrun,
                m_env.m_hit_drv.accepted_payload_total,
                m_env.m_hit_drv.offered_payload_total,
                m_env.m_hit_drv.pending_source_items()))
              accepted_at_endofrun = m_env.m_hit_drv.accepted_payload_total;
            end
          end
          if (!saw_post_endofrun_offers) begin
            `uvm_error("P068", $sformatf(
              "Mid-run TERMINATING never observed additional offered traffic after endofrun_seen: offered_at_eor=%0d final_offered=%0d",
              offered_at_endofrun,
              m_env.m_hit_drv.offered_payload_total))
          end

          ctrl_send(ring_buffer_cam_pkg::CTRL_TERMINATING);
        end
      join

      discard_pending_source_backlog("P068 dropped post-endofrun backlog");
      wait_clocks(2);
      wait_for_scoreboard_idle(200_000, "P068 mid-run terminate cleanup");
      expect_service_model_accounting("P068 mid-run terminate cleanup", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != m_env.m_hit_drv.accepted_payload_total) begin
        `uvm_error("P068", $sformatf(
          "PUSH_COUNT disagreed with accepted payload total before RESTART: push=%0d accepted=%0d pop=%0d overwrite=%0d",
          push_count,
          m_env.m_hit_drv.accepted_payload_total,
          pop_count,
          overwrite_count))
      end
      if (cache_miss_count != 0 || fill_level != 0) begin
        `uvm_error("P068", $sformatf(
          "Mid-run TERMINATING left unexpected residual accounting before RESTART: fill=%0d cache_miss=%0d push=%0d pop=%0d overwrite=%0d",
          fill_level, cache_miss_count, push_count, pop_count, overwrite_count))
      end

      restart_after_flush(16'd128);
      accepted_after_restart_checkpoint = m_env.m_hit_drv.accepted_payload_total;
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 0 || pop_count != 0 || overwrite_count != 0 ||
          cache_miss_count != 0 || fill_level != 0) begin
        `uvm_error("P068", $sformatf(
          "RESTART did not begin from a clean accounting state: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d",
          push_count, pop_count, overwrite_count, cache_miss_count, fill_level))
      end

      final_seq = profile_traffic_seq::type_id::create("p068_final_postrestart_window");
      final_seq.num_hits = 1_024;
      final_seq.lane_key_start_ord = 4;
      final_seq.pool_keys = 4;
      final_seq.hits_per_key_switch = 1;
      final_seq.randomize_key_order = 1'b0;
      final_seq.inter_hit_gap_cycles = 13;
      final_seq.fingerprint_start_index = 16_384;
      final_seq.progress_stride = 256;
      final_seq.progress_tag = "P068 final post-restart window";
      final_seq.start(m_env.m_hit_seqr);

      terminate_and_drain(400_000, "P068 final post-restart terminate");
      expect_service_model_accounting("P068 final post-restart terminate", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != (m_env.m_hit_drv.accepted_payload_total - accepted_after_restart_checkpoint)) begin
        `uvm_error("P068", $sformatf(
          "Post-restart PUSH_COUNT drifted from the accepted delta of the final window: push=%0d accepted_restart_delta=%0d accepted_total=%0d accepted_restart_checkpoint=%0d pop=%0d overwrite=%0d",
          push_count,
          m_env.m_hit_drv.accepted_payload_total - accepted_after_restart_checkpoint,
          m_env.m_hit_drv.accepted_payload_total,
          accepted_after_restart_checkpoint,
          pop_count,
          overwrite_count))
      end
      if (overwrite_count != 0 || cache_miss_count != 0 || fill_level != 0) begin
        `uvm_error("P068", $sformatf(
          "Final post-restart accounting was not lossless-clean: pop=%0d overwrite=%0d cache_miss=%0d fill=%0d",
          pop_count, overwrite_count, cache_miss_count, fill_level))
      end
    end else if (case_id == "P069") begin
      profile_traffic_seq flush_seq;
      profile_traffic_seq final_seq;
      int unsigned accepted_window_checkpoint;
      int unsigned accepted_before_flush;
      int unsigned accepted_after_flush;
      int unsigned accepted_after_final_restart;
      int unsigned push_count;
      int unsigned pop_count;
      int unsigned overwrite_count;
      int unsigned cache_miss_count;
      int unsigned fill_level;
      int unsigned hits_after_flush;
      int unsigned hits_after_flush_wait;
      bit          stop_requested;
      bit          p069_traffic_done;
      int unsigned burst_idx;

      configure_and_start(16'd128);
      accepted_window_checkpoint = m_env.m_hit_drv.accepted_payload_total;

      for (int window = 0; window < 2; window++) begin
        stop_requested = 1'b0;
        p069_traffic_done = 1'b0;

        fork
          begin
            burst_idx = 0;
            while (!stop_requested && burst_idx < 256) begin
              flush_seq = profile_traffic_seq::type_id::create($sformatf("p069_uniform_flush_w%0d_b%0d", window, burst_idx));
              flush_seq.num_hits = 64;
              flush_seq.lane_key_start_ord = 4;
              flush_seq.pool_keys = 4;
              flush_seq.hits_per_key_switch = 1;
              flush_seq.randomize_key_order = 1'b0;
              flush_seq.inter_hit_gap_cycles = 13;
              flush_seq.fingerprint_start_index = (window * 16_384) + (burst_idx * 64);
              flush_seq.progress_stride = 0;
              flush_seq.start(m_env.m_hit_seqr);
              burst_idx++;
            end
            p069_traffic_done = 1'b1;
          end
          begin
            wait_for_accepted_payload_count(
              accepted_window_checkpoint + 1_024,
              2_000_000,
              $sformatf("P069 window %0d pre-FLUSH accepted threshold", window));
            stop_requested = 1'b1;
            accepted_before_flush = m_env.m_hit_drv.accepted_payload_total;
            enter_run_prepare();
            accepted_after_flush = m_env.m_hit_drv.accepted_payload_total;
            hits_after_flush = m_env.m_out_mon.total_hits_seen;
            wait_clocks(64);
            hits_after_flush_wait = m_env.m_out_mon.total_hits_seen;
          end
        join

        read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
        read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
        read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
        read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
        read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
        if (push_count != 0 || pop_count != 0 || overwrite_count != 0 ||
            cache_miss_count != 0 || fill_level != 0) begin
          `uvm_error("P069", $sformatf(
            "FLUSH window %0d did not clear the DUT accounting state: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d accepted_before=%0d accepted_after=%0d",
            window, push_count, pop_count, overwrite_count, cache_miss_count, fill_level,
            accepted_before_flush, accepted_after_flush))
        end
        if (m_env.m_scb.remaining_entries() != 0 ||
            m_env.m_scb.pending_drain_entries() != 0) begin
          `uvm_error("P069", $sformatf(
            "FLUSH window %0d left scoreboard residual state: remaining=%0d pending_drain=%0d accepted_before=%0d accepted_after=%0d",
            window, m_env.m_scb.remaining_entries(), m_env.m_scb.pending_drain_entries(),
            accepted_before_flush, accepted_after_flush))
        end
        if (hits_after_flush_wait != hits_after_flush) begin
          `uvm_error("P069", $sformatf(
            "FLUSH window %0d leaked output activity after flush-complete: hits_after_flush=%0d hits_after_wait=%0d",
            window, hits_after_flush, hits_after_flush_wait))
        end
        if (!p069_traffic_done) begin
          `uvm_error("P069", $sformatf(
            "FLUSH window %0d traffic thread did not quiesce after the stop request",
            window))
        end

        discard_pending_source_backlog($sformatf("P069 dropped post-flush backlog window %0d", window));
        wait_clocks(2);
        restart_after_flush(16'd128);
        accepted_window_checkpoint = m_env.m_hit_drv.accepted_payload_total;
      end

      accepted_after_final_restart = m_env.m_hit_drv.accepted_payload_total;
      final_seq = profile_traffic_seq::type_id::create("p069_final_postflush_window");
      final_seq.num_hits = 1_024;
      final_seq.lane_key_start_ord = 4;
      final_seq.pool_keys = 4;
      final_seq.hits_per_key_switch = 1;
      final_seq.randomize_key_order = 1'b0;
      final_seq.inter_hit_gap_cycles = 13;
      final_seq.fingerprint_start_index = 32_768;
      final_seq.progress_stride = 256;
      final_seq.progress_tag = "P069 final post-flush window";
      final_seq.start(m_env.m_hit_seqr);

      terminate_and_drain(400_000, "P069 final post-flush terminate");
      expect_service_model_accounting("P069 final post-flush terminate", 1, 0);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, cache_miss_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != (m_env.m_hit_drv.accepted_payload_total - accepted_after_final_restart)) begin
        `uvm_error("P069", $sformatf(
          "Post-restart PUSH_COUNT drifted from the accepted delta of the final window: push=%0d accepted_restart_delta=%0d accepted_total=%0d accepted_restart_checkpoint=%0d pop=%0d overwrite=%0d",
          push_count,
          m_env.m_hit_drv.accepted_payload_total - accepted_after_final_restart,
          m_env.m_hit_drv.accepted_payload_total,
          accepted_after_final_restart,
          pop_count,
          overwrite_count))
      end
      if (overwrite_count != 0 || cache_miss_count != 0 || fill_level != 0) begin
        `uvm_error("P069", $sformatf(
          "Final post-restart accounting was not lossless-clean: pop=%0d overwrite=%0d cache_miss=%0d fill=%0d",
          pop_count, overwrite_count, cache_miss_count, fill_level))
      end
    end else if (case_id == "P111") begin
      run_single_key_overwrite_window("P111 single-key overwrite 10pct-ish", 576);
    end else if (case_id == "P112") begin
      run_single_key_overwrite_window("P112 single-key overwrite 30pct-ish", 736);
    end else if (case_id == "P113") begin
      run_single_key_overwrite_window("P113 single-key overwrite 50pct-ish", 1024);
    end else if (case_id == "P114") begin
      int unsigned bp_push_count;
      int unsigned bp_pop_count;
      int unsigned bp_overwrite_count;
      int unsigned bp_fill_level;
      int unsigned sink_low_cycles;
      bit          bp_traffic_done;

      configure_and_start(2000);
      sink_low_cycles = 0;
      bp_traffic_done = 1'b0;
      fork
        begin
          search_cycles = 0;
          while (search_cycles < 800_000 &&
                 (!bp_traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
            if ((search_cycles & 1) == 0) begin
              set_sink_ready(1'b1);
            end else begin
              set_sink_ready(1'b0);
              sink_low_cycles++;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
          set_sink_ready(1'b1);
        end
        begin
          burst_seq = same_key_burst_seq::type_id::create("p114_prefill");
          burst_seq.num_hits = 544;
          burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
          burst_seq.start(m_env.m_hit_seqr);

          pressure_seq = overwrite_profile_seq::type_id::create("p114_slow_pressure");
          pressure_seq.num_hits = 4096;
          pressure_seq.lane_key_start_ord = 2;
          pressure_seq.pool_keys = 1;
          pressure_seq.hits_per_key_switch = 1;
          pressure_seq.inter_hit_gap_cycles = 39;
          pressure_seq.progress_stride = 1024;
          pressure_seq.progress_tag = "P114";
          pressure_seq.start(m_env.m_hit_seqr);
          bp_traffic_done = 1'b1;
        end
      join
      terminate_and_drain(1_000_000, "P114 backpressure overwrite terminate drain");
      expect_service_model_accounting("P114 backpressure overwrite post-terminate", 1, 1);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, bp_push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, bp_pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, bp_overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, bp_fill_level);
      if (sink_low_cycles == 0) begin
        `uvm_error("P114", "Backpressure-driven overwrite case never actually forced sink-ready low")
      end
      if (bp_push_count != 4640 || bp_fill_level != 0) begin
        `uvm_error("P114", $sformatf(
          "Backpressure overwrite accounting precondition failed: push=%0d pop=%0d overwrite=%0d fill=%0d",
          bp_push_count, bp_pop_count, bp_overwrite_count, bp_fill_level))
      end
      if (bp_overwrite_count < 64) begin
        `uvm_error("P114", $sformatf(
          "Backpressure-driven overwrite stayed too small to prove the sink-pressure regime: overwrite=%0d expected_at_least=64",
          bp_overwrite_count))
      end
      if ((bp_overwrite_count * 10_000) >= (bp_push_count * 4000)) begin
        `uvm_error("P114", $sformatf(
          "Backpressure overwrite escaped the intended moderate regime: push=%0d overwrite=%0d pct_x100=%0d",
          bp_push_count,
          bp_overwrite_count,
          (bp_push_count == 0) ? 0 : ((bp_overwrite_count * 10_000) / bp_push_count)))
      end
    end else if (case_id == "P115") begin
      int unsigned bp_push_count;
      int unsigned bp_pop_count;
      int unsigned bp_overwrite_count;
      int unsigned bp_fill_level;
      int unsigned sink_low_cycles;
      bit          bp_traffic_done;
      logic [31:0] ready_lfsr;

      configure_and_start(2000);
      sink_low_cycles = 0;
      bp_traffic_done = 1'b0;
      ready_lfsr = 32'h1bad_cafe;
      fork
        begin
          search_cycles = 0;
          while (search_cycles < 800_000 &&
                 (!bp_traffic_done || m_env.m_hit_drv.pending_source_items() != 0)) begin
            ready_lfsr = {ready_lfsr[30:0],
                          ready_lfsr[31] ^ ready_lfsr[21] ^ ready_lfsr[1] ^ ready_lfsr[0]};
            if (ready_lfsr[7:0] < 8'd77) begin
              set_sink_ready(1'b1);
            end else begin
              set_sink_ready(1'b0);
              sink_low_cycles++;
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            search_cycles++;
          end
          set_sink_ready(1'b1);
        end
        begin
          burst_seq = same_key_burst_seq::type_id::create("p115_prefill");
          burst_seq.num_hits = 544;
          burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
          burst_seq.start(m_env.m_hit_seqr);

          pressure_seq = overwrite_profile_seq::type_id::create("p115_random_backpressure");
          pressure_seq.num_hits = 4096;
          pressure_seq.lane_key_start_ord = 2;
          pressure_seq.pool_keys = 1;
          pressure_seq.hits_per_key_switch = 1;
          pressure_seq.inter_hit_gap_cycles = 19;
          pressure_seq.progress_stride = 1024;
          pressure_seq.progress_tag = "P115";
          pressure_seq.start(m_env.m_hit_seqr);
          bp_traffic_done = 1'b1;
        end
      join
      terminate_and_drain(1_000_000, "P115 random-backpressure overwrite terminate drain");
      expect_service_model_accounting("P115 random-backpressure overwrite post-terminate", 1, 1);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, bp_push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, bp_pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, bp_overwrite_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, bp_fill_level);
      if (sink_low_cycles == 0) begin
        `uvm_error("P115", "Random-backpressure overwrite case never actually forced sink-ready low")
      end
      if (bp_push_count != 4640 || bp_fill_level != 0) begin
        `uvm_error("P115", $sformatf(
          "Random-backpressure overwrite accounting precondition failed: push=%0d pop=%0d overwrite=%0d fill=%0d",
          bp_push_count, bp_pop_count, bp_overwrite_count, bp_fill_level))
      end
      if (bp_overwrite_count < 96) begin
        `uvm_error("P115", $sformatf(
          "Random-backpressure overwrite stayed too small to prove the 30%%-ready regime: overwrite=%0d expected_at_least=96",
          bp_overwrite_count))
      end
      if ((bp_overwrite_count * 10_000) >= (bp_push_count * 6500)) begin
        `uvm_error("P115", $sformatf(
          "Random-backpressure overwrite escaped the intended stochastic regime: push=%0d overwrite=%0d pct_x100=%0d",
          bp_push_count,
          bp_overwrite_count,
          (bp_push_count == 0) ? 0 : ((bp_overwrite_count * 10_000) / bp_push_count)))
      end
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
    end else if (case_id == "P120") begin
      run_single_key_slow_creep_case(
        "P120 single-key overwrite slow-creep",
        16'd2000, 544, 4096, 39, 16, 500);
    end else if (case_id == "P123") begin
      run_single_key_latency_extreme_case(
        "P123 single-key overwrite max-latency",
        16'hFFFF, 1024, 900, 1024, 480, 520, 1'b1);
    end else if (case_id == "P124") begin
      run_single_key_latency_extreme_case(
        "P124 single-key overwrite min-latency",
        16'd1, 1024, 32, 96, 320, 416, 1'b0, 128, 192);
    end else if (case_id == "P125") begin
      int unsigned key_hits_per_epoch[$];
      int unsigned lane_key_ord_list[$];
      int unsigned p125_total_hits;
      int unsigned p125_push_count;
      int unsigned p125_pop_count;
      int unsigned p125_overwrite_count;
      int unsigned p125_cache_miss_count;
      int unsigned p125_fill_level;
      int unsigned p125_sink_low_cycles;
      int unsigned p125_ow16;
      int unsigned p125_ow20;
      int unsigned p125_ow24;
      int unsigned p125_ow28;
      int unsigned p125_ow_min;
      int unsigned p125_ow_max;

      key_hits_per_epoch.push_back(25);
      key_hits_per_epoch.push_back(25);
      key_hits_per_epoch.push_back(25);
      key_hits_per_epoch.push_back(25);
      lane_key_ord_list.push_back(4);
      lane_key_ord_list.push_back(5);
      lane_key_ord_list.push_back(6);
      lane_key_ord_list.push_back(7);

      run_weighted_overwrite_checkpoint_case(
        "P125 overwrite checkpoint soak balanced",
        "P125",
        16'd2000,
        10_000,
        key_hits_per_epoch,
        lane_key_ord_list,
        0,
        0,
        0,
        25_000_000,
        256,
        p125_total_hits,
        p125_push_count,
        p125_pop_count,
        p125_overwrite_count,
        p125_cache_miss_count,
        p125_fill_level,
        p125_sink_low_cycles);

      if (m_env.m_scb.max_remaining_entries() < m_cfg.ring_buffer_n_entry) begin
        `uvm_error("P125", $sformatf(
          "Balanced overwrite soak never saturated the shared rbCAM: max_remaining=%0d ring_depth=%0d overwrite=%0d",
          m_env.m_scb.max_remaining_entries(),
          m_cfg.ring_buffer_n_entry,
          p125_overwrite_count))
      end
      if ((p125_overwrite_count * 10_000) > (p125_total_hits * 3500)) begin
        `uvm_error("P125", $sformatf(
          "Balanced overwrite soak escaped the intended bounded regime: total_hits=%0d overwrite=%0d pct_x100=%0d",
          p125_total_hits,
          p125_overwrite_count,
          (p125_total_hits == 0) ? 0 : ((p125_overwrite_count * 10_000) / p125_total_hits)))
      end

      p125_ow16 = m_env.m_scb.overwrite_events_for_new_key(8'd16);
      p125_ow20 = m_env.m_scb.overwrite_events_for_new_key(8'd20);
      p125_ow24 = m_env.m_scb.overwrite_events_for_new_key(8'd24);
      p125_ow28 = m_env.m_scb.overwrite_events_for_new_key(8'd28);
      p125_ow_min = p125_ow16;
      p125_ow_max = p125_ow16;
      if (p125_ow20 < p125_ow_min) p125_ow_min = p125_ow20;
      if (p125_ow24 < p125_ow_min) p125_ow_min = p125_ow24;
      if (p125_ow28 < p125_ow_min) p125_ow_min = p125_ow28;
      if (p125_ow20 > p125_ow_max) p125_ow_max = p125_ow20;
      if (p125_ow24 > p125_ow_max) p125_ow_max = p125_ow24;
      if (p125_ow28 > p125_ow_max) p125_ow_max = p125_ow28;
      if ((p125_ow_max - p125_ow_min) > ((p125_overwrite_count / 10) + 1)) begin
        `uvm_error("P125", $sformatf(
          "Balanced long-soak overwrite ownership drifted too far across the 4 keys: key16=%0d key20=%0d key24=%0d key28=%0d total_overwrite=%0d",
          p125_ow16, p125_ow20, p125_ow24, p125_ow28, p125_overwrite_count))
      end
    end else if (case_id == "P126") begin
      int unsigned key_hits_per_epoch[$];
      int unsigned lane_key_ord_list[$];
      int unsigned p126_total_hits;
      int unsigned p126_push_count;
      int unsigned p126_pop_count;
      int unsigned p126_overwrite_count;
      int unsigned p126_cache_miss_count;
      int unsigned p126_fill_level;
      int unsigned p126_sink_low_cycles;
      int unsigned p126_ow16;
      int unsigned p126_ow20;
      int unsigned p126_ow24;
      int unsigned p126_ow28;

      key_hits_per_epoch.push_back(53);
      key_hits_per_epoch.push_back(23);
      key_hits_per_epoch.push_back(14);
      key_hits_per_epoch.push_back(10);
      lane_key_ord_list.push_back(4);
      lane_key_ord_list.push_back(5);
      lane_key_ord_list.push_back(6);
      lane_key_ord_list.push_back(7);

      run_weighted_overwrite_checkpoint_case(
        "P126 overwrite checkpoint soak zipf-like",
        "P126",
        16'd2000,
        10_000,
        key_hits_per_epoch,
        lane_key_ord_list,
        0,
        0,
        0,
        25_000_000,
        256,
        p126_total_hits,
        p126_push_count,
        p126_pop_count,
        p126_overwrite_count,
        p126_cache_miss_count,
        p126_fill_level,
        p126_sink_low_cycles);

      if (m_env.m_scb.max_remaining_entries() < m_cfg.ring_buffer_n_entry) begin
        `uvm_error("P126", $sformatf(
          "Zipf-like overwrite soak never saturated the shared rbCAM: max_remaining=%0d ring_depth=%0d overwrite=%0d",
          m_env.m_scb.max_remaining_entries(),
          m_cfg.ring_buffer_n_entry,
          p126_overwrite_count))
      end

      p126_ow16 = m_env.m_scb.overwrite_events_for_new_key(8'd16);
      p126_ow20 = m_env.m_scb.overwrite_events_for_new_key(8'd20);
      p126_ow24 = m_env.m_scb.overwrite_events_for_new_key(8'd24);
      p126_ow28 = m_env.m_scb.overwrite_events_for_new_key(8'd28);
      if (!(p126_ow16 > p126_ow20 &&
            p126_ow20 > p126_ow24 &&
            p126_ow24 > p126_ow28)) begin
        `uvm_error("P126", $sformatf(
          "Zipf-like overwrite ordering mismatch: key16=%0d key20=%0d key24=%0d key28=%0d",
          p126_ow16, p126_ow20, p126_ow24, p126_ow28))
      end
      if (p126_total_hits >= 512_000 && p126_ow28 == 0) begin
        `uvm_error("P126", $sformatf(
          "Tail-key overwrite never appeared by the time the soak exceeded 512k accepted hits: total_hits=%0d key28=%0d",
          p126_total_hits, p126_ow28))
      end
      if (p126_ow16 <= (p126_ow28 * 2)) begin
        `uvm_error("P126", $sformatf(
          "Dominant-key overwrite did not separate enough from the tail under the zipf-like profile: key16=%0d key28=%0d",
          p126_ow16, p126_ow28))
      end
    end else if (case_id == "P127") begin
      weighted_profile_seq prefill_seq;
      weighted_profile_seq pressure_seq;
      int unsigned prefill_key_hits[$];
      int unsigned key_hits_per_epoch[$];
      int unsigned lane_key_ord_list[$];
      int unsigned checkpoint_targets[$];
      int unsigned hits_per_epoch_total;
      int unsigned num_epochs;
      int unsigned override_hits;
      int unsigned p127_total_hits;
      int unsigned p127_push_count;
      int unsigned p127_pop_count;
      int unsigned p127_overwrite_count;
      int unsigned p127_cache_miss_count;
      int unsigned p127_fill_level;
      int unsigned p127_sink_low_cycles;
      int unsigned p127_prefill_overwrite_count;
      int unsigned p127_prefill_fill_level;
      int unsigned p127_prefill_max_live_fill;
      int unsigned p127_ow16;
      int unsigned p127_ow20;
      int unsigned p127_ow24;
      int unsigned p127_ow28;
      int unsigned p127_nonzero_keys;
      int unsigned sink_cycles;
      int unsigned p127_timeout_cycles;
      logic [31:0] ready_lfsr;
      bit          ready_now;
      bit          traffic_done;
      bit          pressure_phase_active;

      prefill_key_hits.push_back(128);
      prefill_key_hits.push_back(128);
      prefill_key_hits.push_back(128);
      prefill_key_hits.push_back(128);
      key_hits_per_epoch.push_back(25);
      key_hits_per_epoch.push_back(25);
      key_hits_per_epoch.push_back(25);
      key_hits_per_epoch.push_back(25);
      lane_key_ord_list.push_back(4);
      lane_key_ord_list.push_back(5);
      lane_key_ord_list.push_back(6);
      lane_key_ord_list.push_back(7);

      hits_per_epoch_total = 0;
      foreach (key_hits_per_epoch[idx]) begin
        hits_per_epoch_total += key_hits_per_epoch[idx];
      end
      num_epochs = 10_000;
      override_hits = get_long_txn_override();
      if (override_hits > 0) begin
        int unsigned remaining_hits_target;

        if (override_hits > 512) begin
          remaining_hits_target = override_hits - 512;
        end else begin
          remaining_hits_target = hits_per_epoch_total;
        end
        num_epochs = (remaining_hits_target + hits_per_epoch_total - 1) / hits_per_epoch_total;
        if (num_epochs == 0) begin
          num_epochs = 1;
        end
        `uvm_info("PROF", $sformatf(
          "P127 overwrite checkpoint soak backpressure applying DV_LONG_TXN_OVERRIDE=%0d -> prefill=512 epochs=%0d",
          override_hits, num_epochs), UVM_LOW)
      end

      p127_total_hits = 512 + (hits_per_epoch_total * num_epochs);
      build_log_spaced_checkpoint_targets(p127_total_hits, checkpoint_targets);
      p127_timeout_cycles = 60_000_000;

      p127_sink_low_cycles = 0;
      traffic_done = 1'b0;
      pressure_phase_active = 1'b0;
      ready_lfsr = 32'h1bad_cafe;

      configure_and_start(16'd2000);

      fork
        begin : p127_sink_ready_thread
          sink_cycles = 0;
          while (sink_cycles < p127_timeout_cycles &&
                 (!traffic_done ||
                  m_env.m_hit_drv.pending_source_items() != 0 ||
                  m_env.m_scb.remaining_entries() != 0)) begin
            if (!pressure_phase_active) begin
              set_sink_ready(1'b1);
            end else begin
              ready_lfsr = {ready_lfsr[30:0],
                            ready_lfsr[31] ^ ready_lfsr[21] ^ ready_lfsr[1] ^ ready_lfsr[0]};
              ready_now = (ready_lfsr[7:0] < 8'd77); // ~30% ready
              set_sink_ready(ready_now);
              if (!ready_now) begin
                p127_sink_low_cycles++;
              end
            end
            @(posedge m_env.m_csr_drv.vif.clk);
            sink_cycles++;
          end
          set_sink_ready(1'b1);
        end
        begin : p127_checkpoint_thread
          save_txn_growth_checkpoints(
            "P127", checkpoint_targets, p127_timeout_cycles,
            "P127 overwrite checkpoint soak backpressure");
        end
        begin : p127_traffic_thread
          prefill_seq = weighted_profile_seq::type_id::create("p127_prefill");
          prefill_seq.num_epochs = 1;
          prefill_seq.inter_hit_gap_cycles = 0;
          prefill_seq.inter_epoch_gap_cycles = 0;
          prefill_seq.progress_stride = 200;
          prefill_seq.progress_tag = "P127 overwrite checkpoint soak backpressure prefill";
          foreach (prefill_key_hits[idx]) begin
            prefill_seq.key_hits_per_epoch.push_back(prefill_key_hits[idx]);
          end
          foreach (lane_key_ord_list[idx]) begin
            prefill_seq.lane_key_ord_list.push_back(lane_key_ord_list[idx]);
          end
          prefill_seq.start(m_env.m_hit_seqr);

          wait_clocks(32);
          read_counter_u32(CSR_OVERWRITE_ADDR, p127_prefill_overwrite_count);
          read_counter_u32(CSR_FILL_LEVEL_ADDR, p127_prefill_fill_level);
          p127_prefill_max_live_fill = m_env.m_dbg_mon.max_live_fill;
          pressure_phase_active = 1'b1;

          pressure_seq = weighted_profile_seq::type_id::create("p127_pressure");
          pressure_seq.num_epochs = num_epochs;
          pressure_seq.inter_hit_gap_cycles = 19;
          pressure_seq.inter_epoch_gap_cycles = 0;
          pressure_seq.fingerprint_start_index = 512;
          pressure_seq.progress_stride = ((p127_total_hits - 512) >= 4096) ? ((p127_total_hits - 512) / 4) :
                                         (((p127_total_hits - 512) >= 4) ? ((p127_total_hits - 512) / 2) : 1);
          pressure_seq.progress_tag = "P127 overwrite checkpoint soak backpressure";
          foreach (key_hits_per_epoch[idx]) begin
            pressure_seq.key_hits_per_epoch.push_back(key_hits_per_epoch[idx]);
          end
          foreach (lane_key_ord_list[idx]) begin
            pressure_seq.lane_key_ord_list.push_back(lane_key_ord_list[idx]);
          end
          pressure_seq.start(m_env.m_hit_seqr);

          pressure_phase_active = 1'b0;
          traffic_done = 1'b1;
        end
      join

      set_sink_ready(1'b1);
      terminate_and_drain(p127_timeout_cycles, "P127 overwrite checkpoint soak backpressure terminate drain");
      expect_service_model_accounting("P127 overwrite checkpoint soak backpressure post-terminate", 1, 1);

      read_counter_u32(CSR_PUSH_COUNT_ADDR, p127_push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, p127_pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, p127_overwrite_count);
      read_counter_u32(CSR_CACHE_MISS_ADDR, p127_cache_miss_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, p127_fill_level);

      if (p127_push_count != p127_total_hits ||
          p127_cache_miss_count != 0 ||
          p127_fill_level != 0) begin
        `uvm_error("P127", $sformatf(
          "Backpressure-driven overwrite accounting mismatch: push=%0d pop=%0d overwrite=%0d cache_miss=%0d fill=%0d expected_push=%0d",
          p127_push_count, p127_pop_count, p127_overwrite_count,
          p127_cache_miss_count, p127_fill_level, p127_total_hits))
      end
      if (p127_sink_low_cycles == 0) begin
        `uvm_error("P127", "Backpressure-driven overwrite case never actually forced sink-ready low")
      end
      if (p127_prefill_overwrite_count != 0) begin
        `uvm_error("P127", $sformatf(
          "Balanced prefill already overwrote before sink pressure started: overwrite=%0d fill=%0d max_live_fill=%0d",
          p127_prefill_overwrite_count, p127_prefill_fill_level, p127_prefill_max_live_fill))
      end
      if (p127_prefill_max_live_fill < m_cfg.ring_buffer_n_entry) begin
        `uvm_error("P127", $sformatf(
          "Balanced prefill never saturated the shared rbCAM before pressure started: max_live_fill=%0d ring_depth=%0d",
          p127_prefill_max_live_fill, m_cfg.ring_buffer_n_entry))
      end
      if (p127_overwrite_count <= p127_prefill_overwrite_count) begin
        `uvm_error("P127", $sformatf(
          "Backpressure phase never added overwrite beyond the non-overwriting prefill: prefill=%0d final=%0d",
          p127_prefill_overwrite_count, p127_overwrite_count))
      end

      if (p127_overwrite_count == 0) begin
        `uvm_error("P127", $sformatf(
          "Backpressure-driven multi-key soak never produced overwrite: total_hits=%0d push=%0d pop=%0d overwrite=%0d",
          p127_total_hits, p127_push_count, p127_pop_count, p127_overwrite_count))
      end
      if ((p127_overwrite_count * 10_000) > (p127_total_hits * 7000)) begin
        `uvm_error("P127", $sformatf(
          "Backpressure-driven multi-key soak escaped the intended bounded regime: total_hits=%0d overwrite=%0d pct_x100=%0d",
          p127_total_hits,
          p127_overwrite_count,
          (p127_total_hits == 0) ? 0 : ((p127_overwrite_count * 10_000) / p127_total_hits)))
      end

      p127_ow16 = m_env.m_scb.overwrite_events_for_new_key(8'd16);
      p127_ow20 = m_env.m_scb.overwrite_events_for_new_key(8'd20);
      p127_ow24 = m_env.m_scb.overwrite_events_for_new_key(8'd24);
      p127_ow28 = m_env.m_scb.overwrite_events_for_new_key(8'd28);
      p127_nonzero_keys = 0;
      if (p127_ow16 != 0) p127_nonzero_keys++;
      if (p127_ow20 != 0) p127_nonzero_keys++;
      if (p127_ow24 != 0) p127_nonzero_keys++;
      if (p127_ow28 != 0) p127_nonzero_keys++;
      if (p127_nonzero_keys < 2) begin
        `uvm_error("P127", $sformatf(
          "Backpressure-driven overwrite did not spread beyond one key: key16=%0d key20=%0d key24=%0d key28=%0d",
          p127_ow16, p127_ow20, p127_ow24, p127_ow28))
      end
    end else if (case_id == "P129") begin
      int unsigned key_hits_per_epoch[$];
      int unsigned lane_key_ord_list[$];
      int unsigned p129_total_hits;
      int unsigned p129_push_count;
      int unsigned p129_pop_count;
      int unsigned p129_overwrite_count;
      int unsigned p129_cache_miss_count;
      int unsigned p129_fill_level;
      int unsigned p129_sink_low_cycles;
      int unsigned focus_search_key;

      key_hits_per_epoch.push_back(100);
      lane_key_ord_list.push_back(2);
      run_weighted_overwrite_checkpoint_case(
        "P129 deterministic throughput replay",
        "P129",
        16'd2000,
        10_000,
        key_hits_per_epoch,
        lane_key_ord_list,
        0,
        0,
        0,
        25_000_000,
        1,
        p129_total_hits,
        p129_push_count,
        p129_pop_count,
        p129_overwrite_count,
        p129_cache_miss_count,
        p129_fill_level,
        p129_sink_low_cycles);

      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      if (p129_overwrite_count < 4096 ||
          (p129_push_count != 0 &&
           ((p129_overwrite_count * 10_000) < (p129_push_count * 50)))) begin
        `uvm_error("P129", $sformatf(
          "Directed throughput replay did not sustain a nontrivial overwrite regime after hotspot saturation: push=%0d pop=%0d overwrite=%0d pct_x100=%0d",
          p129_push_count, p129_pop_count, p129_overwrite_count,
          (p129_push_count == 0) ? 0 : ((p129_overwrite_count * 10_000) / p129_push_count)))
      end
      if (m_env.m_scb.max_remaining_entries_for_key(focus_search_key) < m_cfg.ring_buffer_n_entry) begin
        `uvm_error("P129", $sformatf(
          "Directed throughput replay never saturated the replay hotspot: key=%0d max_for_key=%0d ring_depth=%0d",
          focus_search_key,
          m_env.m_scb.max_remaining_entries_for_key(focus_search_key),
          m_cfg.ring_buffer_n_entry))
      end
    end else if (case_id == "P128") begin
      run_single_key_overwrite_window("P128 single-key overwrite upper-bound", 2048);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (overwrite_count < 640) begin
        `uvm_error("P128", $sformatf(
          "Upper-bound overwrite case did not sustain the expected heavy overwrite regime: overwrite=%0d expected_at_least=640",
          overwrite_count))
      end
    end else if (case_id == "P117") begin
      configure_and_start(2000);
      emit_weighted_overwrite_epochs(
        "P117 skewed overwrite",
        m_cfg.lane_key_ord_to_search_key(4),
        m_cfg.lane_key_ord_to_search_key(5),
        m_cfg.lane_key_ord_to_search_key(6),
        m_cfg.lane_key_ord_to_search_key(7),
        64);
      terminate_and_drain(150_000, "P117 skewed overwrite terminate drain");
      expect_service_model_accounting("P117 skewed overwrite post-terminate", 1, 1);
      if (m_env.m_scb.max_remaining_entries() < m_cfg.ring_buffer_n_entry) begin
        `uvm_error("P117", $sformatf(
          "Skewed overwrite case never saturated the rbCAM: max_remaining=%0d ring_depth=%0d",
          m_env.m_scb.max_remaining_entries(), m_cfg.ring_buffer_n_entry))
      end
      ow16 = m_env.m_scb.overwrite_events_for_new_key(8'd16);
      ow20 = m_env.m_scb.overwrite_events_for_new_key(8'd20);
      ow24 = m_env.m_scb.overwrite_events_for_new_key(8'd24);
      ow28 = m_env.m_scb.overwrite_events_for_new_key(8'd28);
      if (ow16 <= (ow20 + ow24 + ow28)) begin
        `uvm_error("P117", $sformatf(
          "Heavy-key overwrite did not dominate skewed traffic: key16=%0d key20=%0d key24=%0d key28=%0d",
          ow16, ow20, ow24, ow28))
      end
      if (ow20 > (ow16 / 2) || ow24 > (ow16 / 2) || ow28 > (ow16 / 2)) begin
        `uvm_error("P117", $sformatf(
          "Minority-key overwrite was too large under 70/10/10/10 skew: key16=%0d key20=%0d key24=%0d key28=%0d",
          ow16, ow20, ow24, ow28))
      end
    end else if (case_id == "P118") begin
      configure_and_start(2000);
      emit_weighted_overwrite_epochs(
        "P118 phase_a",
        m_cfg.lane_key_ord_to_search_key(4),
        m_cfg.lane_key_ord_to_search_key(5),
        m_cfg.lane_key_ord_to_search_key(6),
        m_cfg.lane_key_ord_to_search_key(7),
        64);
      wait_clocks(32);
      ow16 = m_env.m_scb.overwrite_events_for_new_key(8'd16);
      ow20 = m_env.m_scb.overwrite_events_for_new_key(8'd20);
      ow24 = m_env.m_scb.overwrite_events_for_new_key(8'd24);
      ow28 = m_env.m_scb.overwrite_events_for_new_key(8'd28);
      if (ow16 == 0 || ow16 <= (ow20 + ow24 + ow28)) begin
        `uvm_error("P118", $sformatf(
          "Phase-A overwrite did not concentrate on the first dominant key: key16=%0d key20=%0d key24=%0d key28=%0d",
          ow16, ow20, ow24, ow28))
      end

      enter_run_prepare();
      configure_and_start(2000);
      emit_weighted_overwrite_epochs(
        "P118 phase_b",
        m_cfg.lane_key_ord_to_search_key(5),
        m_cfg.lane_key_ord_to_search_key(4),
        m_cfg.lane_key_ord_to_search_key(6),
        m_cfg.lane_key_ord_to_search_key(7),
        64);
      wait_clocks(32);
      terminate_and_drain(150_000, "P118 rotating overwrite terminate drain");
      expect_service_model_accounting("P118 rotating overwrite post-terminate", 1, 1);
      ow16_b = m_env.m_scb.overwrite_events_for_new_key(8'd16);
      ow20_b = m_env.m_scb.overwrite_events_for_new_key(8'd20);
      ow24_b = m_env.m_scb.overwrite_events_for_new_key(8'd24);
      ow28_b = m_env.m_scb.overwrite_events_for_new_key(8'd28);
      if (ow16_b == 0 || ow20_b == 0) begin
        `uvm_error("P118", $sformatf(
          "Rotating dominance did not produce overwrite on both second-window dominant candidates: key16_delta=%0d key20_delta=%0d key24_delta=%0d key28_delta=%0d",
          ow16_b, ow20_b, ow24_b, ow28_b))
      end
      if (ow20_b <= (ow16_b + ow24_b + ow28_b)) begin
        `uvm_error("P118", $sformatf(
          "Phase-B overwrite did not rotate to the second dominant key: key16_delta=%0d key20_delta=%0d key24_delta=%0d key28_delta=%0d",
          ow16_b, ow20_b, ow24_b, ow28_b))
      end
      if (ow24_b > (ow20_b / 2) || ow28_b > (ow20_b / 2)) begin
        `uvm_error("P118", $sformatf(
          "Non-dominant keys overwrote too often in phase-B rotating dominance: key16_delta=%0d key20_delta=%0d key24_delta=%0d key28_delta=%0d",
          ow16_b, ow20_b, ow24_b, ow28_b))
      end
    end else if (case_id == "P121") begin
      configure_and_start(2000);
      pressure_seq = overwrite_profile_seq::type_id::create("p121_window_a");
      pressure_seq.num_hits = 640;
      pressure_seq.lane_key_start_ord = 4;
      pressure_seq.pool_keys = 4;
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.start(m_env.m_hit_seqr);
      wait_clocks(32);
      enter_run_prepare();
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_POP_COUNT_ADDR, pop_count);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (fill_level != 0 || push_count != 0 || pop_count != 0 || overwrite_count != 0) begin
        `uvm_error("P121", $sformatf(
          "FLUSH window did not reset the pressure epoch cleanly: fill=%0d push=%0d pop=%0d overwrite=%0d",
          fill_level, push_count, pop_count, overwrite_count))
      end

      configure_and_start(2000);
      pressure_seq = overwrite_profile_seq::type_id::create("p121_window_b");
      pressure_seq.num_hits = 640;
      pressure_seq.lane_key_start_ord = 4;
      pressure_seq.pool_keys = 4;
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.start(m_env.m_hit_seqr);
      terminate_and_drain(150_000, "P121 balanced overwrite terminate drain");
      expect_service_model_accounting("P121 balanced overwrite post-terminate", 1, 1);
      read_counter_u32(CSR_OVERWRITE_ADDR, overwrite_count);
      if (overwrite_count == 0) begin
        `uvm_error("P121", "Second overwrite-pressure window never accumulated overwrite after FLUSH restart")
      end
    end else if (case_id == "P122") begin
      int unsigned window_push_a;
      int unsigned window_pop_a;
      int unsigned window_overwrite_a;
      int unsigned window_fill_a;
      int unsigned window_push_b;
      int unsigned window_pop_b;
      int unsigned window_overwrite_b;
      int unsigned window_fill_b;

      configure_and_start(2000);
      pressure_seq = overwrite_profile_seq::type_id::create("p122_window_a");
      pressure_seq.num_hits = 640;
      pressure_seq.lane_key_start_ord = 4;
      pressure_seq.pool_keys = 4;
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.progress_stride = 128;
      pressure_seq.progress_tag = "P122-A";
      pressure_seq.start(m_env.m_hit_seqr);
      terminate_and_drain(200_000, "P122 window_a terminate drain");
      expect_service_model_accounting("P122 window_a post-terminate", 1, 1);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, window_push_a);
      read_counter_u32(CSR_POP_COUNT_ADDR, window_pop_a);
      read_counter_u32(CSR_OVERWRITE_ADDR, window_overwrite_a);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, window_fill_a);
      if (window_push_a != 640 || window_fill_a != 0 || window_overwrite_a == 0) begin
        `uvm_error("P122", $sformatf(
          "Window-A terminate/restart precondition failed: push=%0d pop=%0d overwrite=%0d fill=%0d",
          window_push_a, window_pop_a, window_overwrite_a, window_fill_a))
      end

      configure_and_start(2000);
      pressure_seq = overwrite_profile_seq::type_id::create("p122_window_b");
      pressure_seq.num_hits = 640;
      pressure_seq.lane_key_start_ord = 4;
      pressure_seq.pool_keys = 4;
      pressure_seq.hits_per_key_switch = 1;
      pressure_seq.progress_stride = 128;
      pressure_seq.progress_tag = "P122-B";
      pressure_seq.start(m_env.m_hit_seqr);
      terminate_and_drain(200_000, "P122 window_b terminate drain");
      expect_service_model_accounting("P122 window_b post-terminate", 1, 1);
      read_counter_u32(CSR_PUSH_COUNT_ADDR, window_push_b);
      read_counter_u32(CSR_POP_COUNT_ADDR, window_pop_b);
      read_counter_u32(CSR_OVERWRITE_ADDR, window_overwrite_b);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, window_fill_b);
      if (window_push_b != 640 || window_fill_b != 0 || window_overwrite_b == 0) begin
        `uvm_error("P122", $sformatf(
          "Window-B terminate/restart precondition failed: push=%0d pop=%0d overwrite=%0d fill=%0d",
          window_push_b, window_pop_b, window_overwrite_b, window_fill_b))
      end
      if (window_push_a != window_push_b ||
          window_pop_a != window_pop_b ||
          window_overwrite_a != window_overwrite_b) begin
        `uvm_error("P122", $sformatf(
          "Terminate/restart windows were not repeatable: A(push=%0d pop=%0d overwrite=%0d) B(push=%0d pop=%0d overwrite=%0d)",
          window_push_a, window_pop_a, window_overwrite_a,
          window_push_b, window_pop_b, window_overwrite_b))
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
    end else if (case_id == "X005") begin
      configure_and_start(2000);
      focus_search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq = same_key_burst_seq::type_id::create("x005_prefill");
      burst_seq.num_hits = 16;
      burst_seq.search_key = focus_search_key;
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(16, 20_000, "X005 active-traffic precondition");
      read_counter_u32(CSR_PUSH_COUNT_ADDR, push_count);
      read_counter_u32(CSR_FILL_LEVEL_ADDR, fill_level);
      if (push_count != 16 || fill_level == 0) begin
        `uvm_error("X005", $sformatf(
          "Precondition failed before soft_reset: push=%0d fill=%0d expected_push=16 expected_fill>0",
          push_count, fill_level))
      end

      csr_write(CSR_CTRL_ADDR, 32'h0000_0013);
      expect_soft_reset_abort(
        "X005 active-traffic soft_reset",
        32'h0000_0013,
        32'h0000_0011);
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
    end else if (case_id == "X031") begin
      run_x031_terminate_empty_case();
    end else if (case_id == "X032") begin
      run_x032_terminate_during_burst_case();
    end else if (case_id == "X033") begin
      run_x033_terminate_during_drain_case();
    end else if (case_id == "X034") begin
      run_x034_terminate_mid_subheader_case();
    end else if (case_id == "X035") begin
      run_x035_terminate_before_soft_reset_case();
    end else if (case_id == "X036") begin
      run_x036_terminate_after_soft_reset_case();
    end else if (case_id == "X038") begin
      run_x038_double_terminate_case();
    end else if (case_id == "X039") begin
      run_x039_terminate_then_prep_case();
    end else if (case_id == "X040") begin
      run_x040_terminate_then_running_case();
    end else if (case_id == "X069") begin
      run_x069_flush_ready_latch_case();
    end else if (case_id == "X041") begin
      run_x041_terminate_with_valid_hit_case();
    end else if (case_id == "X043") begin
      run_x043_full_ring_terminate_case();
    end else if (case_id == "X045") begin
      run_x045_terminate_ready_with_backlog_case();
    end else if (case_id == "X049") begin
      run_x049_terminate_terminate_idle_case();
    end else if (case_id == "X046") begin
      run_x046_filllevel_read_during_terminate_case();
    end else if (case_id == "X047") begin
      run_x047_terminate_push_count_write_case();
    end else if (case_id == "X048") begin
      run_x048_multihot_terminate_case();
    end else if (case_id == "X050") begin
      run_x050_filllevel_underflow_guard_case();
    end else if (case_id == "X051") begin
      run_x051_flush_empty_case();
    end else if (case_id == "X052") begin
      run_x052_flush_full_ring_case();
    end else if (case_id == "X116") begin
      run_x116_good_error_good_case("X116 GOOD-ERROR-GOOD");
    end else if (case_id == "X053") begin
      run_x053_flush_preempts_drain_case();
    end else if (case_id == "X054") begin
      run_x054_flush_drops_incoming_hits_case();
    end else if (case_id == "X055") begin
      run_x055_flush_error_reset_case();
    end else if (case_id == "X056") begin
      run_x056_flush_restart_good_push_case();
    end else if (case_id == "X057") begin
      run_x057_flush_cross_partition_case();
    end else if (case_id == "X059") begin
      run_x059_flush_clears_stale_epoch_case();
    end else if (case_id == "X061") begin
      run_x061_flush_backlog_popcmd_case();
    end else if (case_id == "X082") begin
      csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
      wait_clocks(2);
      csr_expect_mask(
        CSR_CTRL_ADDR, 32'h0000_0002, 32'h0000_0000,
        "X082 CTRL soft_reset bit self-clears after a single write");
    end else if (case_id == "X083") begin
      wait_clocks(2);
      csr_expect_mask(
        CSR_CTRL_ADDR, 32'h0000_0002, 32'h0000_0000,
        "X083 CTRL soft_reset bit stays low without writes");
      wait_clocks(2);
      csr_expect_mask(
        CSR_CTRL_ADDR, 32'h0000_0002, 32'h0000_0000,
        "X083 CTRL soft_reset bit remains low on the next idle sample");
    end else if (case_id == "X084") begin
      csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
      csr_write(CSR_CTRL_ADDR, 32'h0000_0002);
      wait_clocks(2);
      csr_expect_mask(
        CSR_CTRL_ADDR, 32'h0000_0002, 32'h0000_0000,
        "X084 back-to-back soft-reset writes still self-clear");
    end else if (case_id == "X097") begin
      data_a = 0;
      m_env.m_ctrl_drv.vif.data <= 9'h0ff;
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (2) begin
        @(posedge m_env.m_ctrl_drv.vif.clk);
        if (m_env.m_ctrl_drv.vif.ready === 1'b1)
          data_a = 1;
      end
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      if (data_a != 0 || m_env.m_dbg_mon.run_state_code != ring_buffer_cam_pkg::RUN_STATE_ERROR) begin
        `uvm_error("X097", $sformatf(
          "Multi-hot run-control command was not contained: ack=%0d state=%0d",
          data_a, m_env.m_dbg_mon.run_state_code))
      end
    end else if (case_id == "X098") begin
      data_a = 0;
      m_env.m_ctrl_drv.vif.data <= 9'h000;
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (2) begin
        @(posedge m_env.m_ctrl_drv.vif.clk);
        if (m_env.m_ctrl_drv.vif.ready === 1'b1)
          data_a = 1;
      end
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      if (data_a != 0 || m_env.m_dbg_mon.run_state_code != ring_buffer_cam_pkg::RUN_STATE_ERROR) begin
        `uvm_error("X098", $sformatf(
          "Zero run-control command was not contained: ack=%0d state=%0d",
          data_a, m_env.m_dbg_mon.run_state_code))
      end
    end else if (case_id == "X099") begin
      data_a = 0;
      m_env.m_ctrl_drv.vif.data <= ring_buffer_cam_pkg::CTRL_TERMINATING;
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (8) begin
        @(posedge m_env.m_ctrl_drv.vif.clk);
        if (m_env.m_ctrl_drv.vif.ready === 1'b1)
          data_a = 1;
      end
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      if (data_a != 1 || m_env.m_dbg_mon.run_state_code != ring_buffer_cam_pkg::RUN_STATE_TERMINATING) begin
        `uvm_error("X099", $sformatf(
          "IDLE TERMINATE did not ack cleanly: ack=%0d state=%0d endofrun_seen=%0d term_done=%0d",
          data_a, m_env.m_dbg_mon.run_state_code,
          m_env.m_dbg_mon.endofrun_seen,
          m_env.m_dbg_mon.terminating_drain_done))
      end
    end else if (case_id == "X100") begin
      data_a = 0;
      m_env.m_ctrl_drv.vif.data <= 9'h020;
      m_env.m_ctrl_drv.vif.valid <= 1'b1;
      repeat (2) begin
        @(posedge m_env.m_ctrl_drv.vif.clk);
        if (m_env.m_ctrl_drv.vif.ready === 1'b1)
          data_a = 1;
      end
      m_env.m_ctrl_drv.vif.valid <= 1'b0;
      m_env.m_ctrl_drv.vif.data <= '0;
      repeat (2) @(posedge m_env.m_ctrl_drv.vif.clk);
      if (data_a != 0 || m_env.m_dbg_mon.run_state_code != ring_buffer_cam_pkg::RUN_STATE_LINK_TEST) begin
        `uvm_error("X100", $sformatf(
          "LINK_TEST command did not enter the unsupported state cleanly: ack=%0d state=%0d",
          data_a, m_env.m_dbg_mon.run_state_code))
      end
    end else if (case_id == "X126") begin
      run_x126_recovery_empty_terminate_case();
    end else if (case_id == "X127") begin
      run_x127_endofrun_clear_case();
    end else if (case_id == "X128") begin
      run_x128_gts_end_of_run_refresh_case();
    end else if (case_id == "X042") begin
      run_x042_prelatched_endofrun_case();
    end else if (case_id == "X044") begin
      run_x044_bad_hits_survive_terminate_case();
    end else if (case_id == "X065") begin
      run_x065_flush_then_terminate_case();
    end else if (case_id == "X066") begin
      run_x066_duplicate_prep_noop_case();
    end else if (case_id == "X067") begin
      run_x067_flush_complete_sync_case();
    end else if (case_id == "X068") begin
      run_x068_prep_clears_endofrun_case();
    end else if (case_id == "X071") begin
      run_x071_soft_reset_running_case();
    end else if (case_id == "X072") begin
      run_x072_soft_reset_in_prep_case();
    end else if (case_id == "X073") begin
      run_x073_soft_reset_in_terminate_case();
    end else if (case_id == "X074") begin
      run_x074_soft_reset_during_drain_case();
    end else if (case_id == "X075") begin
      run_x075_soft_reset_with_latency_write_case();
    end else if (case_id == "X076") begin
      run_x076_soft_reset_retains_inerr_case();
    end else if (case_id == "X077") begin
      run_x077_soft_reset_retains_push_case();
    end else if (case_id == "X078") begin
      run_x078_soft_reset_retains_pop_case();
    end else if (case_id == "X079") begin
      run_x079_soft_reset_retains_overwrite_case();
    end else if (case_id == "X058") begin
      run_x058_flush_from_running_case();
    end else if (case_id == "X060") begin
      run_x060_flush_backlog_deassembly_case();
    end else if (case_id == "X062") begin
      run_x062_flush_start_latch_case();
    end else if (case_id == "X063") begin
      run_x063_flush_beats_push_erase_case();
    end else if (case_id == "X081") begin
      run_x081_soft_reset_retains_fill_case();
    end else if (case_id == "X085") begin
      run_x085_soft_reset_uid_read_case();
    end else if (case_id == "X086") begin
      run_x086_reserved_ctrl_bit2_case();
    end else if (case_id == "X087") begin
      run_x087_reserved_ctrl_bit3_case();
    end else if (case_id == "X088") begin
      run_x088_reserved_ctrl_upper_bits_case();
    end else if (case_id == "X089") begin
      expect_csr_write_no_effect(
        CSR_UID_ADDR, 32'hFFFF_FFFF, 32'hFFFF_FFFF,
        "X089 UID write must be inert");
    end else if (case_id == "X090") begin
      run_x090_meta_selector_case();
    end else if (case_id == "X101") begin
      run_x101_single_bad_hit_counter_observer_case();
    end else if (case_id == "X102") begin
      run_x102_bad_hit_burst_counter_observer_case();
    end else if (case_id == "X018") begin
      run_x018_bad_hit_idle_to_prep_boundary_case();
    end else if (case_id == "X019") begin
      run_x019_bad_hit_running_entry_boundary_case();
    end else if (case_id == "X022") begin
      run_x022_inerr_with_expected_latency_read_case();
    end else if (case_id == "X023") begin
      run_x023_inerr_with_filter_disable_write_case();
    end else if (case_id == "X024") begin
      run_x024_inerr_with_inerr_write_case();
    end else if (case_id == "X025") begin
      run_x025_bad_hit_window_case();
    end else if (case_id == "X030") begin
      run_x030_sustained_bad_hit_rate_case();
    end else if (case_id == "X104") begin
      run_x104_push_counter_before_terminate_ready_case();
    end else if (case_id == "X105") begin
      run_x105_pop_counter_before_terminate_ready_case();
    end else if (case_id == "X106") begin
      run_x106_overwrite_counter_before_terminate_ready_case();
    end else if (case_id == "X108") begin
      run_x108_inerr_near_flush_case();
    end else if (case_id == "X109") begin
      run_x109_overwrite_near_flush_case();
    end else if (case_id == "X111") begin
      run_x111_push_near_flush_case();
    end else if (case_id == "X112") begin
      run_x112_pop_near_flush_case();
    end else if (case_id == "X113") begin
      run_x113_inerr_coincident_soft_reset_case();
    end else if (case_id == "X114") begin
      run_x114_push_count_read_race_case();
    end else if (case_id == "X091") begin
      configure_and_start(2000);
      burst_seq = same_key_burst_seq::type_id::create("x091_filllevel_prefill");
      burst_seq.num_hits = 8;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(8, 10_000, "X091 prefill");
      wait_clocks(4);
      expect_csr_write_no_effect(
        CSR_FILL_LEVEL_ADDR, 32'hFFFF_FFFF, 32'hFFFF_FFFF,
        "X091 FILL_LEVEL write must be inert");
      m_env.m_scb.note_intentional_nonempty_end(8);
    end else if (case_id == "X092") begin
      configure_and_start(2000);
      burst_seq = same_key_burst_seq::type_id::create("x092_pushcount_prefill");
      burst_seq.num_hits = 8;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(8, 10_000, "X092 prefill");
      wait_clocks(4);
      expect_csr_write_no_effect(
        CSR_PUSH_COUNT_ADDR, 32'hFFFF_FFFF, 32'hFFFF_FFFF,
        "X092 PUSH_COUNT write must be inert");
      m_env.m_scb.note_intentional_nonempty_end(8);
    end else if (case_id == "X093") begin
      configure_and_start(2000);
      single_seq = single_push_pop_seq::type_id::create("x093_single");
      single_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      single_seq.start(m_env.m_hit_seqr);
      wait_for_scoreboard_idle(80_000, "X093 pre-drain");
      expect_csr_write_no_effect(
        CSR_POP_COUNT_ADDR, 32'hFFFF_FFFF, 32'hFFFF_FFFF,
        "X093 POP_COUNT write must be inert");
      expect_service_model_accounting("X093 post-drain", 1, 0);
    end else if (case_id == "X094") begin
      configure_and_start(2000);
      burst_seq = same_key_burst_seq::type_id::create("x094_overwrite_prefill");
      burst_seq.num_hits = m_cfg.ring_buffer_n_entry + 1;
      burst_seq.search_key = m_cfg.lane_key_ord_to_search_key(2);
      burst_seq.start(m_env.m_hit_seqr);
      wait_for_push_count(m_cfg.ring_buffer_n_entry + 1, 30_000, "X094 overwrite prefill");
      wait_clocks(4);
      expect_csr_write_no_effect(
        CSR_OVERWRITE_ADDR, 32'hFFFF_FFFF, 32'hFFFF_FFFF,
        "X094 OVERWRITE_COUNT write must be inert");
      m_env.m_scb.note_intentional_nonempty_end(m_cfg.ring_buffer_n_entry);
    end else if (case_id == "X095") begin
      expect_csr_write_no_effect(
        CSR_CACHE_MISS_ADDR, 32'hFFFF_FFFF, 32'hFFFF_FFFF,
        "X095 CACHE_MISS_COUNT write must be inert");
    end else if (case_id == "X096") begin
      expect_csr_write_no_effect(
        32'd10, 32'hFFFF_FFFF, 32'hFFFF_FFFF,
        "X096 unmapped CSR write must be inert");
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

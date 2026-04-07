// base_test.sv — Base UVM test with common setup and teardown

`ifndef BASE_TEST_SV
`define BASE_TEST_SV

import uvm_pkg::*;
`include "uvm_macros.svh"

class base_test extends uvm_test;
  `uvm_component_utils(base_test)

  localparam int unsigned RUN_PREPARE_FLUSH_CYCLES = 150_000;

  ring_buffer_cam_env                       m_env;
  ring_buffer_cam_pkg::ring_buffer_cam_cfg  m_cfg;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    m_cfg = ring_buffer_cam_pkg::ring_buffer_cam_cfg::type_id::create("cfg");
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

  task automatic ctrl_send(logic [8:0] cmd);
    ctrl_cmd_seq ctrl_seq;
    ctrl_seq = ctrl_cmd_seq::type_id::create($sformatf("ctrl_%0h", cmd));
    ctrl_seq.cmd = cmd;
    ctrl_seq.start(m_env.m_ctrl_seqr);
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
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUN_PREPARE);
    wait_clocks(RUN_PREPARE_FLUSH_CYCLES);
    csr_write(1, latency);
    ctrl_send(ring_buffer_cam_pkg::CTRL_SYNC);
    wait_clocks(2);
    ctrl_send(ring_buffer_cam_pkg::CTRL_RUNNING);
    wait_clocks(20);
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
    #(500 * 8ns);  // wait for pop
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
    #(2000 * 8ns);
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
    #(4000 * 8ns);
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
    #(5000 * 8ns);
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
    #(10000 * 8ns);
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
    #(5000 * 8ns);
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
    #(10000 * 8ns);
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
    #(10000 * 8ns);
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
    csr_expect_mask(0, 32'h0000_0013, 32'h0000_0011, "CTRL reset defaults");
    csr_expect_mask(1, 32'h0000_FFFF, 32'd2000, "EXPECTED_LATENCY reset default");
    csr_expect_mask(2, 32'hFFFF_FFFF, 32'd0, "FILL_LEVEL reset default");
    for (int addr = 3; addr <= 7; addr++) begin
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

    csr_write(0, 32'h0000_0000);
    wait_clocks(2);
    csr_expect_mask(0, 32'h0000_0013, 32'h0000_0000, "CTRL writable bits clear");

    csr_write(0, 32'h0000_0011);
    wait_clocks(2);
    csr_expect_mask(0, 32'h0000_0013, 32'h0000_0011, "CTRL writable bits set");

    csr_write(0, 32'h0000_0013);
    wait_clocks(2);
    csr_expect_mask(0, 32'h0000_0013, 32'h0000_0011, "CTRL soft_reset self-clears");

    csr_write(1, 32'h0000_0123);
    wait_clocks(2);
    csr_expect_mask(1, 32'h0000_FFFF, 32'h0000_0123, "EXPECTED_LATENCY writable");

    for (int addr = 2; addr <= 7; addr++) begin
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
    same_key_burst_seq burst_seq;
    single_error_hit_seq err_seq;
    logic [31:0] inerr_count;
    logic [31:0] push_count;
    logic [31:0] pop_count;
    logic [31:0] fill_level;

    phase.raise_objection(this);
    wait_for_reset_release();
    configure_and_start();

    err_seq = single_error_hit_seq::type_id::create("err_seq");
    err_seq.search_key = 8;
    err_seq.start(m_env.m_hit_seqr);
    wait_clocks(32);
    csr_read(3, inerr_count);
    if (inerr_count < 1) begin
      `uvm_error("CFG", $sformatf("INERR_COUNT did not increment after error hit: %0d", inerr_count))
    end

    burst_seq = same_key_burst_seq::type_id::create("burst_seq");
    burst_seq.num_hits = 32;
    burst_seq.search_key = 12;
    burst_seq.start(m_env.m_hit_seqr);

    wait_clocks(400);
    csr_read(4, push_count);
    if (push_count < 32) begin
      `uvm_error("CFG", $sformatf("PUSH_COUNT too small after burst: %0d", push_count))
    end

    wait_clocks(800);
    csr_read(5, pop_count);
    if (pop_count == 0) begin
      `uvm_error("CFG", "POP_COUNT did not advance after activity")
    end

    csr_read(2, fill_level);
    if (fill_level > push_count) begin
      `uvm_error("CFG", $sformatf(
        "FILL_LEVEL larger than PUSH_COUNT: fill=%0d push=%0d",
        fill_level, push_count))
    end

    phase.drop_objection(this);
  endtask
endclass

`endif

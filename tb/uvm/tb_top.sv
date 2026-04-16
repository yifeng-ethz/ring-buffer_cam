// tb_top.sv — Top-level UVM testbench module
// Instantiates clock, reset, interfaces, DUT (VHDL), and connects UVM config_db.
//
// NOTE: The DUT is VHDL; this file uses a mixed-language instantiation.
// Compile with: vcom (VHDL sources) then vlog (SV sources) then vsim tb_top.

`timescale 1ns / 1ps

module tb_top;

  import uvm_pkg::*;
  import ring_buffer_cam_pkg::*;
  `include "uvm_macros.svh"

  // ── Parameters (override via vsim -g) ─────────────────────────
  parameter int G_RING_BUFFER_N_ENTRY  = 512;
  parameter int G_INTERLEAVING_FACTOR  = 4;
  parameter int G_INTERLEAVING_INDEX   = 0;
  parameter int G_N_PARTITIONS         = 2;
  parameter int G_ENCODER_LEAF_WIDTH   = 16;
  parameter int G_ENCODER_PIPE_STAGES  = 4;
  parameter int G_DEBUG                = 1;

  // ── Clock and reset ───────────────────────────────────────────
  logic clk = 0;
  logic rst = 1;
  int unsigned tb_timeout_cycles = 5_000_000;

  always #4 clk = ~clk;  // 8 ns period = 125 MHz

  initial begin
    rst = 1;
    #80;
    rst = 0;
  end

  // ── Interface instances ───────────────────────────────────────
  avst_hit_if  hit_if  (.clk(clk), .rst(rst));
  avst_out_if  out_if  (.clk(clk), .rst(rst));
  avst_ctrl_if ctrl_if (.clk(clk), .rst(rst));
  avmm_csr_if  csr_if  (.clk(clk), .rst(rst));
  dut_debug_if dbg_if  (.clk(clk), .rst(rst));

  // ── DUT (VHDL entity via mixed-language instantiation) ────────
  ring_buffer_cam #(
    .SEARCH_KEY_WIDTH    (8),
    .RING_BUFFER_N_ENTRY (G_RING_BUFFER_N_ENTRY),
    .SIDE_DATA_BITS      (31),
    .INTERLEAVING_FACTOR (G_INTERLEAVING_FACTOR),
    .INTERLEAVING_INDEX  (G_INTERLEAVING_INDEX),
    .N_PARTITIONS        (G_N_PARTITIONS),
    .ENCODER_LEAF_WIDTH  (G_ENCODER_LEAF_WIDTH),
    .ENCODER_PIPE_STAGES (G_ENCODER_PIPE_STAGES),
    .DEBUG               (G_DEBUG)
  ) dut (
    // CSR
    .avs_csr_readdata            (csr_if.readdata),
    .avs_csr_read                (csr_if.read),
    .avs_csr_address             (csr_if.address),
    .avs_csr_waitrequest         (csr_if.waitrequest),
    .avs_csr_write               (csr_if.write),
    .avs_csr_writedata           (csr_if.writedata),
    // Run control
    .asi_ctrl_data               (ctrl_if.data),
    .asi_ctrl_valid              (ctrl_if.valid),
    .asi_ctrl_ready              (ctrl_if.ready),
    // Ingress
    .asi_hit_type1_channel       (hit_if.channel),
    .asi_hit_type1_startofpacket (hit_if.startofpacket),
    .asi_hit_type1_endofpacket   (hit_if.endofpacket),
    .asi_hit_type1_empty         (hit_if.empty),
    .asi_hit_type1_data          (hit_if.data),
    .asi_hit_type1_valid         (hit_if.valid),
    .asi_hit_type1_ready         (hit_if.ready),
    .asi_hit_type1_error         (hit_if.error),
    // Egress
    .aso_hit_type2_channel       (out_if.channel),
    .aso_hit_type2_startofpacket (out_if.startofpacket),
    .aso_hit_type2_endofpacket   (out_if.endofpacket),
    .aso_hit_type2_data          (out_if.data),
    .aso_hit_type2_valid         (out_if.valid),
    .aso_hit_type2_ready         (out_if.ready),
    .aso_hit_type2_error         (out_if.error),
    // Fill level
    .aso_filllevel_valid         (out_if.filllevel_valid),
    .aso_filllevel_data          (out_if.filllevel_data),
    // Clock / reset
    .i_rst                       (rst),
    .i_clk                       (clk)
  );

  // Egress always ready (can add backpressure driver later)
  assign out_if.ready = 1'b1;

  // UVM ingress keeps the optional framing sidebands at zero for the existing payload tests.
  assign hit_if.startofpacket = 1'b0;
  assign hit_if.endofpacket   = 1'b0;
  assign hit_if.empty         = 1'b0;

  always_comb begin
    dbg_if.decision_reg        = dut.v2_core.decision_reg;
    dbg_if.push_write_grant    = dut.v2_core.push_write_grant;
    dbg_if.push_erase_grant    = dut.v2_core.push_erase_grant;
    dbg_if.pop_erase_grant     = dut.v2_core.pop_erase_grant;
    dbg_if.cam_wr_addr         = dut.v2_core.cam_wr_addr;
    dbg_if.side_ram_waddr      = dut.v2_core.side_ram_waddr;
    dbg_if.side_ram_we         = dut.v2_core.side_ram_we;
    dbg_if.side_ram_din        = dut.v2_core.side_ram_din;
    dbg_if.in_hit_side         = dut.v2_core.in_hit_side;
    dbg_if.side_ram_dout       = dut.v2_core.side_ram_dout;
    dbg_if.pop_issue_addr      = dut.v2_core.pop_issue_addr;
    dbg_if.pop_current_sk      = dut.v2_core.pop_current_sk;
    dbg_if.pop_total_hits      = dut.v2_core.pop_total_hits;
    dbg_if.pop_pipeline_start  = dut.v2_core.pop_pipeline_start;
    dbg_if.pop_hit_valid       = dut.v2_core.pop_hit_valid;
    dbg_if.pop_cache_miss_pulse = dut.v2_core.pop_cache_miss_pulse;
    dbg_if.subheader_gen_done  = dut.v2_core.subheader_gen_done;
    dbg_if.pop_cmd_fifo_wrreq  = dut.v2_core.pop_cmd_fifo_wrreq;
    dbg_if.pop_cmd_fifo_rdack  = dut.v2_core.pop_cmd_fifo_rdack;
    dbg_if.pop_cmd_fifo_empty  = dut.v2_core.pop_cmd_fifo_empty;
    dbg_if.pop_cmd_fifo_usedw  = dut.v2_core.pop_cmd_fifo_usedw;
    dbg_if.deassembly_fifo_empty = dut.v2_core.deassembly_fifo_empty;
    dbg_if.deassembly_fifo_full  = dut.v2_core.deassembly_fifo_full;
    dbg_if.deassembly_fifo_usedw = dut.v2_core.deassembly_fifo_usedw;
    dbg_if.endofrun_seen       = dut.v2_core.endofrun_seen;
    dbg_if.terminating_drain_done = dut.v2_core.terminating_drain_done;
    dbg_if.run_mgmt_flushed    = dut.v2_core.run_mgmt_flushed;
    dbg_if.gts_counter_rst     = dut.v2_core.gts_counter_rst;
    dbg_if.gts_8n              = dut.v2_core.gts_8n;
    dbg_if.gts_end_of_run      = dut.v2_core.gts_end_of_run;
    dbg_if.dbg_inerr_cnt       = dut.v2_core.debug_msg2.inerr_cnt;
    dbg_if.dbg_push_cnt        = dut.v2_core.debug_msg2.push_cnt;
    dbg_if.dbg_pop_cnt         = dut.v2_core.debug_msg2.pop_cnt;
    dbg_if.dbg_overwrite_cnt   = dut.v2_core.debug_msg2.overwrite_cnt;
    dbg_if.dbg_cache_miss_cnt  = dut.v2_core.debug_msg2.cache_miss_cnt;
  end

  // ── UVM config_db wiring ──────────────────────────────────────
  initial begin
    uvm_config_db#(int unsigned)::set(null, "*", "ring_buffer_n_entry", G_RING_BUFFER_N_ENTRY);
    uvm_config_db#(int unsigned)::set(null, "*", "interleaving_factor", G_INTERLEAVING_FACTOR);
    uvm_config_db#(int unsigned)::set(null, "*", "interleaving_index", G_INTERLEAVING_INDEX);
    uvm_config_db#(int unsigned)::set(null, "*", "n_partitions", G_N_PARTITIONS);
    uvm_config_db#(int unsigned)::set(null, "*", "encoder_leaf_width", G_ENCODER_LEAF_WIDTH);
    uvm_config_db#(int unsigned)::set(null, "*", "encoder_pipe_stages", G_ENCODER_PIPE_STAGES);
    uvm_config_db#(virtual avst_hit_if.drv)::set(
      null, "uvm_test_top.m_env.m_hit_drv", "vif", hit_if);
    uvm_config_db#(virtual dut_debug_if.mon)::set(
      null, "uvm_test_top.m_env.m_hit_drv", "debug_vif", dbg_if);
    uvm_config_db#(virtual avst_hit_if.mon)::set(
      null, "uvm_test_top.m_env.m_hit_mon", "vif", hit_if);
    uvm_config_db#(virtual avst_ctrl_if.drv)::set(
      null, "uvm_test_top.m_env.m_ctrl_drv", "vif", ctrl_if);
    uvm_config_db#(virtual dut_debug_if.mon)::set(
      null, "uvm_test_top.m_env.m_ctrl_drv", "debug_vif", dbg_if);
    uvm_config_db#(virtual avmm_csr_if.drv)::set(
      null, "uvm_test_top.m_env.m_csr_drv", "vif", csr_if);
    uvm_config_db#(virtual avst_out_if.mon)::set(
      null, "uvm_test_top.m_env.m_out_mon", "vif", out_if);
    uvm_config_db#(virtual dut_debug_if.mon)::set(
      null, "uvm_test_top.m_env.m_out_mon", "debug_vif", dbg_if);
    uvm_config_db#(virtual dut_debug_if.mon)::set(
      null, "uvm_test_top.m_env.m_dbg_mon", "vif", dbg_if);

    run_test();
  end

  // ── Timeout watchdog ──────────────────────────────────────────
  initial begin
    void'($value$plusargs("TB_TIMEOUT_CYCLES=%d", tb_timeout_cycles));
    repeat (tb_timeout_cycles) @(posedge clk);
    `uvm_fatal("TB_TOP", "Global simulation timeout reached")
  end

endmodule

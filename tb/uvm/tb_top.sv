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

  // Ingress sop/eop not used by DUT for data, tie off
  assign hit_if.startofpacket = 1'b0;
  assign hit_if.endofpacket   = 1'b0;

  // ── UVM config_db wiring ──────────────────────────────────────
  initial begin
    uvm_config_db#(int unsigned)::set(null, "*", "n_partitions", G_N_PARTITIONS);
    uvm_config_db#(int unsigned)::set(null, "*", "encoder_leaf_width", G_ENCODER_LEAF_WIDTH);
    uvm_config_db#(int unsigned)::set(null, "*", "encoder_pipe_stages", G_ENCODER_PIPE_STAGES);
    uvm_config_db#(virtual avst_hit_if.drv)::set(
      null, "uvm_test_top.m_env.m_hit_drv", "vif", hit_if);
    uvm_config_db#(virtual avst_hit_if.mon)::set(
      null, "uvm_test_top.m_env.m_hit_mon", "vif", hit_if);
    uvm_config_db#(virtual avst_ctrl_if.drv)::set(
      null, "uvm_test_top.m_env.m_ctrl_drv", "vif", ctrl_if);
    uvm_config_db#(virtual avmm_csr_if.drv)::set(
      null, "uvm_test_top.m_env.m_csr_drv", "vif", csr_if);
    uvm_config_db#(virtual avst_out_if.mon)::set(
      null, "uvm_test_top.m_env.m_out_mon", "vif", out_if);

    run_test();
  end

  // ── Timeout watchdog ──────────────────────────────────────────
  initial begin
    #(500_000 * 8);  // 4 ms simulation time
    `uvm_fatal("TB_TOP", "Global simulation timeout reached")
  end

endmodule

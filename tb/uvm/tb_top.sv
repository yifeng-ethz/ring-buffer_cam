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
  parameter int G_N_SHD                = 128;
  parameter int G_EXPECTED_N_SHD       = G_N_SHD;
  parameter int G_INTERLEAVING_FACTOR  = 4;
  parameter int G_INTERLEAVING_INDEX   = 0;
  parameter int G_N_PARTITIONS         = 2;
  parameter int G_ENCODER_LEAF_WIDTH   = 16;
  parameter int G_ENCODER_PIPE_STAGES  = 4;
  parameter int G_DEBUG                = 1;
  localparam int TB_SRAM_ADDR_W        = $clog2(G_RING_BUFFER_N_ENTRY);

  // ── Clock and reset ───────────────────────────────────────────
  logic clk = 0;
  logic rst = 1;
  logic out_ready_drv = 1'b1;
  logic tb_side_ram_patch_we = 1'b0;
  logic [TB_SRAM_ADDR_W-1:0] tb_side_ram_patch_addr = '0;
  logic [39:0] tb_side_ram_patch_data = '0;
  logic [31:0] coe_debug_fill_level;
  logic [31:0] coe_debug_fifo_level;
  logic [31:0] coe_debug_queue_state;
  longint unsigned tb_timeout_cycles = 5_000_000;

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
`ifdef RBCAM_SV_IMPL
    .N_SHD               (G_N_SHD),
`endif
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
    .asi_hit_type1_metadata      (hit_if.metadata),
    .asi_hit_type1_metadata_valid(hit_if.metadata_valid),
    // Egress
    .aso_hit_type2_channel       (out_if.channel),
    .aso_hit_type2_startofpacket (out_if.startofpacket),
    .aso_hit_type2_endofpacket   (out_if.endofpacket),
    .aso_hit_type2_data          (out_if.data),
    .aso_hit_type2_valid         (out_if.valid),
    .aso_hit_type2_ready         (out_if.ready),
    .aso_hit_type2_error         (out_if.error),
    .aso_hit_type2_metadata      (out_if.metadata),
    .aso_hit_type2_metadata_valid(out_if.metadata_valid),
    // Fill level
    .aso_filllevel_valid         (out_if.filllevel_valid),
    .aso_filllevel_data          (out_if.filllevel_data),
    // DEBUG conduits
    .coe_debug_fill_level        (coe_debug_fill_level),
    .coe_debug_fifo_level        (coe_debug_fifo_level),
    .coe_debug_queue_state       (coe_debug_queue_state),
    // Clock / reset
    .i_rst                       (rst),
    .i_clk                       (clk)
  );

  assign out_if.ready = out_ready_drv;

  always_comb begin
    force dut.v2_core.dbg_side_ram_patch_we = tb_side_ram_patch_we;
    force dut.v2_core.dbg_side_ram_patch_addr = tb_side_ram_patch_addr;
    force dut.v2_core.dbg_side_ram_patch_data = tb_side_ram_patch_data;
  end

  task automatic tb_poison_side_ram_word(
    input logic [TB_SRAM_ADDR_W-1:0] addr,
    input logic [39:0] value
  );
    tb_side_ram_patch_addr = addr;
    tb_side_ram_patch_data = value;
    tb_side_ram_patch_we = 1'b1;
    @(posedge clk);
    tb_side_ram_patch_we = 1'b0;
    tb_side_ram_patch_addr = '0;
    tb_side_ram_patch_data = '0;
  endtask

  // UVM ingress keeps the optional framing sidebands at zero for the existing payload tests.
  assign hit_if.startofpacket = 1'b0;
  assign hit_if.endofpacket   = 1'b0;
  assign hit_if.empty         = 1'b0;

  always_comb begin
    dbg_if.ingress_valid       = hit_if.valid;
    dbg_if.ingress_ready       = hit_if.ready;
    dbg_if.ingress_error       = hit_if.error[0];
    dbg_if.ingress_channel     = hit_if.channel;
    dbg_if.ingress_data        = hit_if.data;
    dbg_if.decision_reg        = dut.v2_core.decision_reg;
    dbg_if.run_state_code      = dut.v2_core.dbg_run_state_code;
    dbg_if.pop_engine_state_code = dut.v2_core.dbg_pop_engine_state_code;
    dbg_if.push_state_code     = dut.v2_core.dbg_push_state_code;
    dbg_if.push_write_grant    = dut.v2_core.push_write_grant;
`ifdef RBCAM_SV_IMPL
    dbg_if.push_write_sector_locked = dut.v2_core.push_write_sector_locked;
    dbg_if.push_write_allowed = dut.v2_core.push_write_allowed;
    dbg_if.push_erase_sector_locked = dut.v2_core.push_erase_sector_locked;
    dbg_if.push_erase_addr = dut.v2_core.push_erase_addr_reg;
    dbg_if.pop_sector_lock_mask = dut.v2_core.pop_sector_lock_mask;
    dbg_if.write_pointer = dut.v2_core.write_pointer;
`else
    dbg_if.push_write_sector_locked = 1'b0;
    dbg_if.push_write_allowed = dut.v2_core.push_write_grant;
    dbg_if.push_erase_sector_locked = 1'b0;
    dbg_if.push_erase_addr = '0;
    dbg_if.pop_sector_lock_mask = '0;
    dbg_if.write_pointer = '0;
`endif
    dbg_if.push_erase_grant    = dut.v2_core.push_erase_grant;
    dbg_if.pop_erase_grant     = dut.v2_core.pop_erase_grant;
    dbg_if.pop_flush_grant     = dut.v2_core.pop_flush_grant;
    dbg_if.run_mgmt_flush_memory_start = dut.v2_core.run_mgmt_flush_memory_start;
    dbg_if.run_mgmt_flush_memory_done = dut.v2_core.run_mgmt_flush_memory_done;
    dbg_if.pop_flush_ram_done  = dut.v2_core.pop_flush_ram_done;
    dbg_if.pop_flush_cam_done  = dut.v2_core.pop_flush_cam_done;
    dbg_if.cam_wr_addr         = dut.v2_core.cam_wr_addr;
    dbg_if.side_ram_waddr      = dut.v2_core.side_ram_waddr;
    dbg_if.side_ram_we         = dut.v2_core.side_ram_we;
    dbg_if.side_ram_din        = dut.v2_core.side_ram_din;
    dbg_if.in_hit_side         = dut.v2_core.in_hit_side;
    dbg_if.side_ram_dout       = dut.v2_core.side_ram_dout;
    dbg_if.pop_issue_addr      = dut.v2_core.pop_issue_addr;
    dbg_if.pop_current_sk      = dut.v2_core.pop_current_sk;
    dbg_if.pop_total_hits      = dut.v2_core.pop_total_hits;
    dbg_if.pop_hits_count      = dut.v2_core.pop_hits_count;
    dbg_if.pop_rr_idx          = dut.v2_core.dbg_pop_rr_idx;
    dbg_if.pop_issue_partition_idx = dut.v2_core.dbg_pop_issue_partition_idx;
    dbg_if.pop_count_partition_idx = dut.v2_core.dbg_pop_count_partition_idx;
    dbg_if.pop_search_wait_cnt = dut.v2_core.pop_search_wait_cnt;
    dbg_if.pop_partition_pending = dut.v2_core.dbg_pop_partition_pending;
    dbg_if.pop_partition_load  = dut.v2_core.dbg_pop_partition_load;
    dbg_if.pop_partition_advance = dut.v2_core.dbg_pop_partition_advance;
    dbg_if.pop_partition_result_valid = dut.v2_core.dbg_pop_partition_result_valid;
    dbg_if.pop_partition_flag  = dut.v2_core.dbg_pop_partition_flag;
    dbg_if.pop_partition_has_more = dut.v2_core.dbg_pop_partition_has_more;
    dbg_if.pop_partition_eval_stage0_valid = dut.v2_core.dbg_pop_partition_eval_stage0_valid;
    dbg_if.pop_last_hit_pending = dut.v2_core.pop_last_hit_pending;
    dbg_if.pop_pipeline_start  = dut.v2_core.pop_pipeline_start;
    dbg_if.pop_hit_valid       = dut.v2_core.pop_hit_valid;
    dbg_if.pop_cache_miss_pulse = dut.v2_core.pop_cache_miss_pulse;
    dbg_if.subheader_gen_done  = dut.v2_core.subheader_gen_done;
    dbg_if.pop_cmd_fifo_sclr   = dut.v2_core.pop_cmd_fifo_sclr;
    dbg_if.deassembly_fifo_sclr = dut.v2_core.deassembly_fifo_sclr;
    dbg_if.pop_cmd_fifo_wrreq  = dut.v2_core.pop_cmd_fifo_wrreq;
    dbg_if.pop_cmd_fifo_rdack  = dut.v2_core.pop_cmd_fifo_rdack;
    dbg_if.pop_cmd_fifo_din    = dut.v2_core.pop_cmd_fifo_din;
    dbg_if.pop_cmd_fifo_dout   = dut.v2_core.pop_cmd_fifo_dout;
    dbg_if.pop_cmd_fifo_empty  = dut.v2_core.pop_cmd_fifo_empty;
    dbg_if.pop_cmd_fifo_usedw  = dut.v2_core.pop_cmd_fifo_usedw;
    dbg_if.deassembly_fifo_empty = dut.v2_core.deassembly_fifo_empty;
    dbg_if.deassembly_fifo_full  = dut.v2_core.deassembly_fifo_full;
    dbg_if.deassembly_fifo_usedw = dut.v2_core.deassembly_fifo_usedw;
    dbg_if.endofrun_seen       = dut.v2_core.endofrun_seen;
    dbg_if.terminating_drain_done = dut.v2_core.terminating_drain_done;
    dbg_if.run_mgmt_flushed    = dut.v2_core.run_mgmt_flushed;
    dbg_if.cam_clean           = dut.v2_core.debug_msg2.cam_clean;
    dbg_if.gts_counter_rst     = dut.v2_core.gts_counter_rst;
    dbg_if.expected_latency_48b = dut.v2_core.expected_latency_48b;
    dbg_if.read_time_ptr       = dut.v2_core.read_time_ptr;
    dbg_if.gts_8n              = dut.v2_core.gts_8n;
    dbg_if.gts_end_of_run      = dut.v2_core.gts_end_of_run;
    dbg_if.flush_ram_wraddr    = dut.v2_core.flush_ram_wraddr;
    dbg_if.flush_cam_wraddr    = dut.v2_core.flush_cam_wraddr;
    dbg_if.flush_cam_wrdata    = dut.v2_core.flush_cam_wrdata;
    dbg_if.dbg_inerr_cnt       = dut.v2_core.debug_msg2.inerr_cnt;
    dbg_if.dbg_push_cnt        = dut.v2_core.debug_msg2.push_cnt;
    dbg_if.dbg_pop_cnt         = dut.v2_core.debug_msg2.pop_cnt;
    dbg_if.dbg_overwrite_cnt   = dut.v2_core.debug_msg2.overwrite_cnt;
    dbg_if.dbg_cache_miss_cnt  = dut.v2_core.debug_msg2.cache_miss_cnt;
`ifdef RBCAM_SV_IMPL
    dbg_if.dbg_deasm_full_drop_cnt = dut.v2_core.debug_msg2.deasm_full_drop_cnt;
    dbg_if.dbg_pop_cmd_full_drop_cnt = dut.v2_core.debug_msg2.pop_cmd_full_drop_cnt;
    dbg_if.dbg_egress_not_ready_drop_cnt = dut.v2_core.debug_msg2.egress_not_ready_drop_cnt;
`else
    dbg_if.dbg_deasm_full_drop_cnt = '0;
    dbg_if.dbg_pop_cmd_full_drop_cnt = '0;
    dbg_if.dbg_egress_not_ready_drop_cnt = '0;
`endif
  end

  // ── UVM config_db wiring ──────────────────────────────────────
  initial begin
    uvm_config_db#(int unsigned)::set(null, "*", "ring_buffer_n_entry", G_RING_BUFFER_N_ENTRY);
    uvm_config_db#(int unsigned)::set(null, "*", "n_shd", G_EXPECTED_N_SHD);
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
    uvm_config_db#(virtual avst_out_if.drv)::set(
      null, "uvm_test_top", "out_vif", out_if);
    uvm_config_db#(virtual dut_debug_if.mon)::set(
      null, "uvm_test_top.m_env.m_out_mon", "debug_vif", dbg_if);
    uvm_config_db#(virtual dut_debug_if.mon)::set(
      null, "uvm_test_top.m_env.m_dbg_mon", "vif", dbg_if);

    run_test();
  end

  // ── Timeout watchdog ──────────────────────────────────────────
  initial begin
    string dv_case_id;
    string dv_exec_mode;
    string dv_run_id;
    longint unsigned watchdog_cycle;

    if ($value$plusargs("DV_CASE_ID=%s", dv_case_id)) begin
      if (dv_case_id == "P127") begin
        tb_timeout_cycles = 60_000_000;
      end else if (dv_case_id == "P125" ||
                   dv_case_id == "P126" ||
                   dv_case_id == "P129") begin
        tb_timeout_cycles = 25_000_000;
      end
    end
    if ($value$plusargs("DV_EXEC_MODE=%s", dv_exec_mode)) begin
      if (dv_exec_mode == "bucket_frame") begin
        tb_timeout_cycles = 100_000_000;
      end else if (dv_exec_mode == "cross") begin
        tb_timeout_cycles = 250_000_000;
        if ($value$plusargs("DV_RUN_ID=%s", dv_run_id) &&
            (dv_run_id == "CROSS-005" || dv_run_id == "CROSS-006")) begin
          tb_timeout_cycles = 400_000_000;
        end
      end
    end
    void'($value$plusargs("TB_TIMEOUT_CYCLES=%d", tb_timeout_cycles));
    for (watchdog_cycle = 0; watchdog_cycle < tb_timeout_cycles; watchdog_cycle++) begin
      @(posedge clk);
    end
    `uvm_fatal("TB_TOP", "Global simulation timeout reached")
  end

endmodule

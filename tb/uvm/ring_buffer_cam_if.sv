// ring_buffer_cam_if.sv — SystemVerilog interfaces for the DUT ports

`ifndef RING_BUFFER_CAM_IF_SV
`define RING_BUFFER_CAM_IF_SV

// ── Avalon-ST hit ingress ────────────────────────────────────────
interface avst_hit_if (input logic clk, input logic rst);
  logic [3:0]  channel;
  logic        startofpacket;
  logic        endofpacket;
  logic        empty;
  logic [38:0] data;
  logic        valid;
  logic        ready;
  logic [0:0]  error;

  modport drv (
    output data, valid, channel, startofpacket, endofpacket, empty, error,
    input  ready, clk, rst
  );
  modport mon (
    input data, valid, ready, channel, startofpacket, endofpacket, empty, error, clk, rst
  );
endinterface

// ── Avalon-ST hit egress ─────────────────────────────────────────
interface avst_out_if (input logic clk, input logic rst);
  logic [3:0]  channel;
  logic        startofpacket;
  logic        endofpacket;
  logic [35:0] data;
  logic        valid;
  logic        ready;
  logic [0:0]  error;

  logic        filllevel_valid;
  logic [15:0] filllevel_data;

  modport drv (
    output ready,
    input  data, valid, channel, startofpacket, endofpacket, error,
           filllevel_valid, filllevel_data, clk, rst
  );
  modport mon (
    input data, valid, ready, channel, startofpacket, endofpacket, error,
          filllevel_valid, filllevel_data, clk, rst
  );
endinterface

// ── Avalon-ST run control ────────────────────────────────────────
interface avst_ctrl_if (input logic clk, input logic rst);
  logic [8:0] data;
  logic       valid;
  logic       ready;

  modport drv (
    output data, valid,
    input  ready, clk, rst
  );
  modport mon (
    input data, valid, ready, clk, rst
  );
endinterface

// ── Avalon-MM CSR ────────────────────────────────────────────────
interface avmm_csr_if (input logic clk, input logic rst);
  logic [4:0]  address;
  logic        read;
  logic [31:0] readdata;
  logic        write;
  logic [31:0] writedata;
  logic        waitrequest;

  modport drv (
    output address, read, write, writedata,
    input  readdata, waitrequest, clk, rst
  );
  modport mon (
    input address, read, readdata, write, writedata, waitrequest, clk, rst
  );
endinterface

// ── Internal DUT debug tap ───────────────────────────────────────
interface dut_debug_if (input logic clk, input logic rst);
  logic         ingress_valid;
  logic         ingress_ready;
  logic         ingress_error;
  logic [3:0]   ingress_channel;
  logic [38:0]  ingress_data;
  logic [2:0]   decision_reg;
  logic [3:0]   run_state_code;
  logic [2:0]   pop_engine_state_code;
  logic         push_state_code;
  logic         push_write_grant;
  logic         push_erase_grant;
  logic         pop_erase_grant;
  logic         pop_flush_grant;
  logic         run_mgmt_flush_memory_start;
  logic         run_mgmt_flush_memory_done;
  logic         pop_flush_ram_done;
  logic         pop_flush_cam_done;
  logic [15:0]  cam_wr_addr;
  logic [15:0]  side_ram_waddr;
  logic         side_ram_we;
  logic [39:0]  side_ram_din;
  logic [38:0]  in_hit_side;
  logic [39:0]  side_ram_dout;
  logic [15:0]  pop_issue_addr;
  logic [8:0]   pop_current_sk;
  logic [15:0]  pop_total_hits;
  logic [15:0]  pop_hits_count;
  logic [1:0]   pop_rr_idx;
  logic [1:0]   pop_issue_partition_idx;
  logic [1:0]   pop_count_partition_idx;
  logic [2:0]   pop_search_wait_cnt;
  logic [3:0]   pop_partition_pending;
  logic [3:0]   pop_partition_load;
  logic [3:0]   pop_partition_advance;
  logic [3:0]   pop_partition_result_valid;
  logic [3:0]   pop_partition_flag;
  logic [3:0]   pop_partition_has_more;
  logic [3:0]   pop_partition_eval_stage0_valid;
  logic         pop_last_hit_pending;
  logic         pop_pipeline_start;
  logic         pop_hit_valid;
  logic         pop_cache_miss_pulse;
  logic         subheader_gen_done;
  logic         pop_cmd_fifo_sclr;
  logic         deassembly_fifo_sclr;
  logic         pop_cmd_fifo_wrreq;
  logic         pop_cmd_fifo_rdack;
  logic         pop_cmd_fifo_empty;
  logic [3:0]   pop_cmd_fifo_usedw;
  logic         deassembly_fifo_empty;
  logic         deassembly_fifo_full;
  logic [7:0]   deassembly_fifo_usedw;
  logic         endofrun_seen;
  logic         terminating_drain_done;
  logic         run_mgmt_flushed;
  logic         cam_clean;
  logic         gts_counter_rst;
  logic [47:0]  expected_latency_48b;
  logic [47:0]  read_time_ptr;
  logic [47:0]  gts_8n;
  logic [47:0]  gts_end_of_run;
  logic [15:0]  flush_ram_wraddr;
  logic [15:0]  flush_cam_wraddr;
  logic [15:0]  flush_cam_wrdata;
  logic [47:0]  dbg_inerr_cnt;
  logic [47:0]  dbg_push_cnt;
  logic [47:0]  dbg_pop_cnt;
  logic [47:0]  dbg_overwrite_cnt;
  logic [47:0]  dbg_cache_miss_cnt;

  modport mon (
    input ingress_valid, ingress_ready, ingress_error, ingress_channel, ingress_data,
          decision_reg, run_state_code, pop_engine_state_code, push_state_code,
          push_write_grant, push_erase_grant, pop_erase_grant, pop_flush_grant,
          run_mgmt_flush_memory_start, run_mgmt_flush_memory_done,
          pop_flush_ram_done, pop_flush_cam_done,
          cam_wr_addr, side_ram_waddr, side_ram_we, side_ram_din,
          in_hit_side, side_ram_dout,
          pop_issue_addr, pop_current_sk, pop_total_hits, pop_hits_count,
          pop_rr_idx, pop_issue_partition_idx, pop_count_partition_idx,
          pop_search_wait_cnt, pop_partition_pending, pop_partition_load,
          pop_partition_advance, pop_partition_result_valid,
          pop_partition_flag, pop_partition_has_more,
          pop_partition_eval_stage0_valid, pop_last_hit_pending,
          pop_pipeline_start, pop_hit_valid, pop_cache_miss_pulse, subheader_gen_done,
          pop_cmd_fifo_sclr, deassembly_fifo_sclr,
          pop_cmd_fifo_wrreq, pop_cmd_fifo_rdack, pop_cmd_fifo_empty,
          pop_cmd_fifo_usedw, deassembly_fifo_empty, deassembly_fifo_full,
          deassembly_fifo_usedw, endofrun_seen, terminating_drain_done,
          run_mgmt_flushed, cam_clean, gts_counter_rst,
          expected_latency_48b, read_time_ptr, gts_8n, gts_end_of_run,
          flush_ram_wraddr, flush_cam_wraddr, flush_cam_wrdata,
          dbg_inerr_cnt, dbg_push_cnt, dbg_pop_cnt, dbg_overwrite_cnt,
          dbg_cache_miss_cnt, clk, rst
  );
endinterface

`endif

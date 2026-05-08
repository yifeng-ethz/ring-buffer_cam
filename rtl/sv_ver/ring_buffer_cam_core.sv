// File name: ring_buffer_cam_core.sv
// Author  : Yifeng Wang (yifenwan@phys.ethz.ch), Codex migration
// Version : 26.2.12
// Date    : 20260508
// Change  : M10K-backed CAM/side storage, VHDL-parity CAM flush, and DEBUG gating

module ring_buffer_cam_core #(
  parameter int SEARCH_KEY_WIDTH    = 8,
  parameter int RING_BUFFER_N_ENTRY = 512,
  parameter int SIDE_DATA_BITS      = 31,
  parameter int INTERLEAVING_FACTOR = 4,
  parameter int INTERLEAVING_INDEX  = 0,
  parameter int N_PARTITIONS        = 4,
  parameter int ENCODER_LEAF_WIDTH  = 16,
  parameter int ENCODER_PIPE_STAGES = 4,
  parameter int IP_UID              = 32'h5242_434d,
  parameter int VERSION_MAJOR       = 26,
  parameter int VERSION_MINOR       = 2,
  parameter int VERSION_PATCH       = 12,
  parameter int BUILD               = 508,
  parameter int VERSION_DATE        = 20260508,
  parameter int VERSION_GIT         = 0,
  parameter int INSTANCE_ID         = 0,
  parameter int DEBUG               = 1
) (
  output logic [31:0] avs_csr_readdata,
  input  logic        avs_csr_read,
  input  logic [4:0]  avs_csr_address,
  output logic        avs_csr_waitrequest,
  input  logic        avs_csr_write,
  input  logic [31:0] avs_csr_writedata,

  input  logic [8:0]  asi_ctrl_data,
  input  logic        asi_ctrl_valid,
  output logic        asi_ctrl_ready,

  input  logic [3:0]  asi_hit_type1_channel,
  input  logic        asi_hit_type1_startofpacket,
  input  logic        asi_hit_type1_endofpacket,
  input  logic        asi_hit_type1_empty,
  input  logic [38:0] asi_hit_type1_data,
  input  logic        asi_hit_type1_valid,
  output logic        asi_hit_type1_ready,
  input  logic [0:0]  asi_hit_type1_error,
  input  logic [63:0] asi_hit_type1_metadata,
  input  logic        asi_hit_type1_metadata_valid,

  output logic [3:0]  aso_hit_type2_channel,
  output logic        aso_hit_type2_startofpacket,
  output logic        aso_hit_type2_endofpacket,
  output logic [35:0] aso_hit_type2_data,
  output logic        aso_hit_type2_valid,
  input  logic        aso_hit_type2_ready,
  output logic [0:0]  aso_hit_type2_error,
  output logic [63:0] aso_hit_type2_metadata,
  output logic        aso_hit_type2_metadata_valid,

  output logic        aso_filllevel_valid,
  output logic [15:0] aso_filllevel_data,
  output logic [31:0] coe_debug_fill_level,
  output logic [31:0] coe_debug_fifo_level,
  output logic [31:0] coe_debug_queue_state,

  input  logic        i_rst,
  input  logic        i_clk
);
  import ring_buffer_cam_sv_pkg::*;

  localparam int ADDR_W_CONST          = (RING_BUFFER_N_ENTRY <= 2) ? 1 : $clog2(RING_BUFFER_N_ENTRY);
  localparam int PARTITION_COUNT_CONST = (N_PARTITIONS < 1) ? 1 : N_PARTITIONS;
  localparam int PARTITION_SIZE_CONST  =
    (RING_BUFFER_N_ENTRY + PARTITION_COUNT_CONST - 1) / PARTITION_COUNT_CONST;
  localparam int PARTITION_ADDR_W_CONST =
    (PARTITION_SIZE_CONST <= 2) ? 1 : $clog2(PARTITION_SIZE_CONST);
  localparam int PARTITION_IDX_W_CONST =
    (PARTITION_COUNT_CONST <= 2) ? 1 : $clog2(PARTITION_COUNT_CONST);
  localparam int SEARCH_CHUNK_WIDTH_CONST =
    (ENCODER_LEAF_WIDTH < 1) ? 16 : ENCODER_LEAF_WIDTH;
  localparam int SEARCH_CHUNK_COUNT_CONST =
    (PARTITION_SIZE_CONST + SEARCH_CHUNK_WIDTH_CONST - 1) /
    SEARCH_CHUNK_WIDTH_CONST;
  localparam int SEARCH_CHUNK_IDX_W_CONST =
    (SEARCH_CHUNK_COUNT_CONST <= 2) ? 1 : $clog2(SEARCH_CHUNK_COUNT_CONST);
  // Keep aligned with the VHDL scfifo_w40d256 ingress buffer depth.
  localparam int DEASM_DEPTH_CONST     = 256;
  localparam int POP_CMD_DEPTH_CONST   = 16;
  localparam int LOCK_SECTOR_COUNT_CONST = 8;
  localparam int LOCK_SECTOR_BITS_CONST = 3;
  localparam int ADDR_SECTOR_LSB_CONST =
    (ADDR_W_CONST > LOCK_SECTOR_BITS_CONST) ?
    (ADDR_W_CONST - LOCK_SECTOR_BITS_CONST) : 0;
  localparam int MAIN_CAM_DATA_WIDTH_CONST =
    ((SEARCH_KEY_WIDTH + 7) / 8) * 8;
  localparam logic [ADDR_W_CONST-1:0] RAM_FLUSH_ADDR_MAX_CONST =
    ADDR_W_CONST'(RING_BUFFER_N_ENTRY - 1);
  localparam logic [ADDR_W_CONST-1:0] CAM_FLUSH_ADDR_MAX_CONST =
    {ADDR_W_CONST{1'b1}};
  localparam logic [MAIN_CAM_DATA_WIDTH_CONST-1:0] CAM_FLUSH_DATA_MAX_CONST =
    {MAIN_CAM_DATA_WIDTH_CONST{1'b1}};
  localparam logic [8:0] CTRL_IDLE_CONST        = 9'b0_0000_0001;
  localparam logic [8:0] CTRL_RUN_PREPARE_CONST = 9'b0_0000_0010;
  localparam logic [8:0] CTRL_SYNC_CONST        = 9'b0_0000_0100;
  localparam logic [8:0] CTRL_RUNNING_CONST     = 9'b0_0000_1000;
  localparam logic [8:0] CTRL_TERMINATING_CONST = 9'b0_0001_0000;
  localparam logic [8:0] CTRL_LINK_TEST_CONST   = 9'b0_0010_0000;
  localparam bit DEBUG_OBSERVABILITY_EN_CONST = (DEBUG >= 1);
  localparam bit DEBUG_METADATA_EN_CONST      = (DEBUG >= 2);

  typedef struct packed {
    logic [39:0] hit_word;
    logic [63:0] metadata;
    logic        metadata_valid;
  } deasm_word_t;
  localparam int DEASM_WORD_BITS_CONST = $bits(deasm_word_t);

  typedef enum logic [1:0] {
    PUSH_WRITING = 2'd0,
    PUSH_ERASING = 2'd1
  } push_state_e;

  run_state_e run_state_cmd;
  pop_state_e pop_engine_state;
  push_state_e push_state;
  debug_msg_t debug_msg2;
  debug_msg_t debug_msg2_snap;

  logic csr_go;
  logic csr_soft_reset_pulse;
  logic csr_filter_inerr;
  logic csr_counter_freeze;
  logic [31:0] csr_expected_latency;
  logic [1:0] csr_meta_sel;
  logic [47:0] expected_latency_48b;
  logic [47:0] read_time_ptr;
  logic [47:0] read_time_ptr_comb;
  logic [47:0] gts_8n;
  logic [47:0] gts_end_of_run;
  logic gts_counter_rst;

  deasm_word_t deassembly_fifo_din;
  deasm_word_t deassembly_fifo_dout;
  logic deassembly_fifo_wrreq;
  logic deassembly_fifo_rdack;
  logic deassembly_fifo_empty;
  logic deassembly_fifo_full;
  logic deassembly_fifo_sclr;
  logic [$clog2(DEASM_DEPTH_CONST+1)-1:0] deassembly_fifo_usedw_int;
  logic [7:0] deassembly_fifo_usedw;
  logic ingress_active_window;
  logic ingress_lane_match;
  logic ingress_payload_countable;
  logic deasm_full_drop_event;

  logic [8:0] pop_cmd_fifo_din;
  logic [8:0] pop_cmd_fifo_dout;
  logic pop_cmd_fifo_wrreq;
  logic pop_cmd_fifo_rdack;
  logic pop_cmd_fifo_empty;
  logic pop_cmd_fifo_full;
  logic pop_cmd_fifo_sclr;
  logic [$clog2(POP_CMD_DEPTH_CONST+1)-1:0] pop_cmd_fifo_usedw_int;
  logic [3:0] pop_cmd_fifo_usedw;
  logic pop_cmd_tick_due;
  logic pop_cmd_full_drop_event;

  logic [RING_BUFFER_N_ENTRY-1:0] slot_valid;
  logic [RING_BUFFER_N_ENTRY-1:0][63:0] slot_metadata;
  logic [RING_BUFFER_N_ENTRY-1:0] slot_metadata_valid;
  logic [ADDR_W_CONST-1:0] write_pointer;

  logic in_payload_valid;
  logic [38:0] in_hit_side;
  logic [12:0] in_hit_ts8n;
  logic [7:0] in_hit_sk;

  logic push_write_req;
  logic push_write_allowed;
  logic push_write_sector_locked;
  logic push_erase_sector_locked;
  logic pop_search_collecting;
  logic push_write_search_key_conflict;
  logic [LOCK_SECTOR_COUNT_CONST-1:0] pop_sector_lock_mask;
  logic push_write_grant;
  logic push_erase_req;
  logic push_erase_grant;
  logic [7:0] push_write_sk_reg;
  logic [ADDR_W_CONST-1:0] push_erase_addr_reg;
  logic [7:0] push_erase_sk;
  logic [7:0] push_erase_sk_reg;
  logic push_erase_sk_valid;

  logic [2:0] decision;
  logic [2:0] decision_reg;
  logic cam_wr_en;
  logic cam_erase_en;
  logic [15:0] cam_wr_addr;
  logic [MAIN_CAM_DATA_WIDTH_CONST-1:0] cam_wr_data;
  logic cam_wr_en_d1;
  logic cam_erase_en_d1;
  logic [ADDR_W_CONST-1:0] cam_wr_addr_d1;
  logic [MAIN_CAM_DATA_WIDTH_CONST-1:0] cam_wr_data_d1;
  logic [MAIN_CAM_DATA_WIDTH_CONST-1:0] cam_cmp_din;
  logic [RING_BUFFER_N_ENTRY-1:0] cam_match_addr_oh;
  logic [15:0] side_ram_waddr;
  logic [39:0] side_ram_din;
  logic side_ram_we;
  logic [15:0] side_ram_raddr;
  logic [39:0] side_ram_q;
  logic [39:0] side_ram_dout;

  logic [PARTITION_SIZE_CONST-1:0] pop_partition_snapshot [PARTITION_COUNT_CONST];
  logic [PARTITION_COUNT_CONST-1:0] pop_partition_active;
  logic [PARTITION_COUNT_CONST-1:0] pop_partition_head_valid;
  logic [PARTITION_ADDR_W_CONST-1:0] pop_partition_head_local [PARTITION_COUNT_CONST];
  logic [SEARCH_CHUNK_COUNT_CONST-1:0] pop_partition_chunk_active [PARTITION_COUNT_CONST];
  logic [15:0] pop_partition_hit_count [PARTITION_COUNT_CONST];
  logic [LOCK_SECTOR_COUNT_CONST-1:0] pop_partition_sector_mask [PARTITION_COUNT_CONST];
  logic [LOCK_SECTOR_COUNT_CONST-1:0] pop_snapshot_sector_mask;
  logic [LOCK_SECTOR_COUNT_CONST-1:0] pop_search_unscanned_sector_mask;
  logic [8:0] pop_current_sk;
  logic [15:0] pop_total_hits;
  logic [15:0] pop_hits_count;
  logic [15:0] pop_issue_addr;
  logic [15:0] pop_issue_addr_pending;
  logic [38:0] pop_hit_pending;
  logic [63:0] pop_metadata_pending;
  logic pop_metadata_valid_pending;
  logic pop_occupied_pending;
  logic pop_output_pending;
  logic pop_side_read_pending;
  logic pop_issue_inflight;
  logic pop_emit_pending;
  logic pop_erase_req;
  logic pop_erase_grant;
  logic pop_flush_req;
  logic pop_flush_grant;
  logic pop_pipeline_start;
  logic pop_hit_valid;
  logic [8:0] pop_hit_pending_epoch;
  logic pop_cache_miss_pulse;
  logic pop_last_hit_pending;
  logic subheader_gen_done;
  logic [2:0] pop_search_wait_cnt;
  logic [PARTITION_IDX_W_CONST-1:0] pop_search_partition_idx;
  logic [SEARCH_CHUNK_IDX_W_CONST-1:0] pop_search_chunk_idx;
  logic pop_search_capture_done;
  logic pop_search_capture_stage_valid;
  logic [PARTITION_IDX_W_CONST-1:0] pop_search_capture_stage_partition_idx;
  logic [SEARCH_CHUNK_IDX_W_CONST-1:0] pop_search_capture_stage_chunk_idx;
  logic [SEARCH_CHUNK_WIDTH_CONST-1:0] pop_search_capture_stage_snapshot;
  logic pop_search_chunk_stage_valid;
  logic [PARTITION_IDX_W_CONST-1:0] pop_search_stage_partition_idx;
  logic [SEARCH_CHUNK_IDX_W_CONST-1:0] pop_search_stage_chunk_idx;
  logic [SEARCH_CHUNK_WIDTH_CONST-1:0] pop_search_stage_snapshot;
  logic [LOCK_SECTOR_COUNT_CONST-1:0] pop_search_stage_sector_mask;
  logic [15:0] pop_search_stage_hit_count;
  logic pop_search_stage_head_valid;
  logic [PARTITION_ADDR_W_CONST-1:0] pop_search_stage_head_local;
  logic [PARTITION_IDX_W_CONST-1:0] pop_drain_partition_idx;
  logic pop_head_update_valid;
  logic [PARTITION_IDX_W_CONST-1:0] pop_head_update_partition_idx;
  logic [SEARCH_CHUNK_IDX_W_CONST-1:0] pop_head_update_chunk_idx;
  logic [PARTITION_SIZE_CONST-1:0] pop_head_update_snapshot;
  logic pop_head_update_scan_valid;
  logic [PARTITION_IDX_W_CONST-1:0] pop_head_update_scan_partition_idx;
  logic [SEARCH_CHUNK_IDX_W_CONST-1:0] pop_head_update_scan_chunk_idx;
  logic [PARTITION_SIZE_CONST-1:0] pop_head_update_scan_snapshot;
  logic [SEARCH_CHUNK_COUNT_CONST-1:0] pop_head_update_scan_chunk_active;
  logic [LOCK_SECTOR_COUNT_CONST-1:0] pop_head_update_scan_part_mask;
  logic [15:0] pop_head_update_scan_hit_count;
  logic pop_head_update_apply_valid;
  logic [PARTITION_IDX_W_CONST-1:0] pop_head_update_apply_partition_idx;
  logic [SEARCH_CHUNK_COUNT_CONST-1:0] pop_head_update_apply_chunk_active;
  logic [LOCK_SECTOR_COUNT_CONST-1:0] pop_head_update_apply_part_mask;
  logic [15:0] pop_head_update_apply_hit_count;
  logic pop_head_update_apply_next_chunk_valid;
  logic [SEARCH_CHUNK_IDX_W_CONST-1:0] pop_head_update_apply_next_chunk_idx;
  logic [SEARCH_CHUNK_WIDTH_CONST-1:0] pop_head_update_apply_chunk_snapshot;
  logic [PARTITION_ADDR_W_CONST-1:0] pop_head_update_apply_head_local;
  logic [1:0] dbg_pop_rr_idx;
  logic [1:0] dbg_pop_issue_partition_idx;
  logic [1:0] dbg_pop_count_partition_idx;
  logic [3:0] dbg_pop_partition_pending;
  logic [3:0] dbg_pop_partition_load;
  logic [3:0] dbg_pop_partition_advance;
  logic [3:0] dbg_pop_partition_result_valid;
  logic [3:0] dbg_pop_partition_flag;
  logic [3:0] dbg_pop_partition_has_more;
  logic [3:0] dbg_pop_partition_eval_stage0_valid;
  logic egress_not_ready_drop_event;
  logic [31:0] resident_fill_level;
  logic [31:0] fill_level_word;

  logic endofrun_seen;
  logic terminating_drain_done;
  logic run_mgmt_flush_memory_start;
  logic run_mgmt_flush_memory_done;
  logic run_mgmt_flushed;
  logic pop_flush_ram_done;
  logic pop_flush_cam_done;
  logic [15:0] flush_ram_wraddr;
  logic [15:0] flush_cam_wraddr;
  logic [MAIN_CAM_DATA_WIDTH_CONST-1:0] flush_cam_wrdata;
  logic [15:0] flush_cycle_count;
  logic prep_ready_latched;

  wire dbg_side_ram_patch_we;
  wire [ADDR_W_CONST-1:0] dbg_side_ram_patch_addr;
  wire [39:0] dbg_side_ram_patch_data;

  assign avs_csr_waitrequest = 1'b0;
  assign dbg_side_ram_patch_we = 1'b0;
  assign dbg_side_ram_patch_addr = '0;
  assign dbg_side_ram_patch_data = '0;
  assign expected_latency_48b = {16'd0, csr_expected_latency};
  assign deassembly_fifo_usedw = 8'(deassembly_fifo_usedw_int);
  assign pop_cmd_fifo_usedw = pop_cmd_fifo_usedw_int[3:0];
  assign deassembly_fifo_sclr = csr_soft_reset_pulse || (run_state_cmd == RUN_PREPARING && flush_cycle_count == 16'd0);
  assign pop_cmd_fifo_sclr = deassembly_fifo_sclr;
  assign gts_counter_rst = (run_state_cmd != RUN_RUNNING && run_state_cmd != RUN_TERMINATING);
  assign egress_not_ready_drop_event = aso_hit_type2_valid && !aso_hit_type2_ready;

  assign fill_level_word = avs_fill_level_word();
  assign coe_debug_fill_level = DEBUG_OBSERVABILITY_EN_CONST ? fill_level_word : 32'd0;
  assign coe_debug_fifo_level = DEBUG_OBSERVABILITY_EN_CONST ?
    {
      4'd0,
      pop_cmd_fifo_usedw,
      4'd0,
      deassembly_fifo_usedw,
      4'd0,
      pop_cmd_fifo_full,
      pop_cmd_fifo_empty,
      deassembly_fifo_full,
      deassembly_fifo_empty,
      4'd0
    } : 32'd0;
  assign coe_debug_queue_state = DEBUG_OBSERVABILITY_EN_CONST ?
    {
      12'd0,
      decision_reg,
      pop_engine_state,
      run_state_cmd,
      push_state,
      6'd0
    } : 32'd0;
  assign aso_filllevel_valid = 1'b1;
  assign aso_filllevel_data = fill_level_word[15:0];
  assign pop_hit_pending_epoch = hit_search_epoch(pop_hit_pending);
  assign pop_search_collecting =
    pop_engine_state == POP_SEARCHING &&
    pop_search_wait_cnt >= 3'd6;
  assign push_write_search_key_conflict =
    (pop_engine_state == POP_SEARCHING) &&
    (in_hit_sk == pop_current_sk[7:0]);
  assign push_erase_sk =
    push_erase_sk_valid ? push_erase_sk_reg : hit_search_key(side_ram_q[38:0]);
  assign cam_cmp_din = MAIN_CAM_DATA_WIDTH_CONST'(pop_current_sk[7:0]);

  cam_mem_a5 #(
    .CAM_SIZE      (RING_BUFFER_N_ENTRY),
    .CAM_WIDTH     (MAIN_CAM_DATA_WIDTH_CONST),
    .WR_ADDR_WIDTH (ADDR_W_CONST)
  ) main_cam (
    .i_clk        (i_clk),
    .i_rst        (i_rst),
    .i_erase_en   (cam_erase_en_d1),
    .i_wr_en      (cam_wr_en_d1),
    .i_wr_data    (cam_wr_data_d1),
    .i_wr_addr    (cam_wr_addr_d1),
    .i_cmp_din    (cam_cmp_din),
    .o_match_addr (cam_match_addr_oh)
  );

  ring_buffer_cam_side_ram #(
    .DATA_WIDTH (40),
    .ADDR_WIDTH (ADDR_W_CONST)
  ) side_ram (
    .clk            (i_clk),
    .raddr          (side_ram_raddr[ADDR_W_CONST-1:0]),
    .waddr          (side_ram_waddr[ADDR_W_CONST-1:0]),
    .dbg_patch_addr (dbg_side_ram_patch_addr),
    .data           (side_ram_din),
    .dbg_patch_data (dbg_side_ram_patch_data),
    .dbg_patch_we   (dbg_side_ram_patch_we),
    .we             (side_ram_we),
    .q              (side_ram_q)
  );

  function automatic logic [31:0] avs_fill_level_word();
    return resident_fill_level;
  endfunction

  function automatic logic [31:0] counter_word(
    input logic [63:0] live_value,
    input logic [63:0] snap_value,
    input logic        high_word
  );
    logic [63:0] selected;
    selected = csr_counter_freeze ? snap_value : live_value;
    if (high_word) begin
      return selected[63:32];
    end
    return selected[31:0];
  endfunction

  function automatic logic [ADDR_W_CONST-1:0] ptr_inc(
    input logic [ADDR_W_CONST-1:0] ptr
  );
    if (ptr == RING_BUFFER_N_ENTRY - 1) begin
      return '0;
    end
    return ptr + {{(ADDR_W_CONST-1){1'b0}}, 1'b1};
  endfunction

  function automatic logic [LOCK_SECTOR_BITS_CONST-1:0] addr_sector(
    input logic [ADDR_W_CONST-1:0] addr
  );
    return LOCK_SECTOR_BITS_CONST'(addr >> ADDR_SECTOR_LSB_CONST);
  endfunction

  function automatic int unsigned partition_base_addr(input int unsigned part_idx);
    return part_idx * PARTITION_SIZE_CONST;
  endfunction

  function automatic logic [ADDR_W_CONST-1:0] partition_global_addr(
    input int unsigned part_idx,
    input int unsigned local_idx
  );
    int unsigned raw_addr;

    raw_addr = partition_base_addr(part_idx) + local_idx;
    if (raw_addr >= RING_BUFFER_N_ENTRY) begin
      return ADDR_W_CONST'(RING_BUFFER_N_ENTRY - 1);
    end
    return ADDR_W_CONST'(raw_addr);
  endfunction

  function automatic logic [SEARCH_CHUNK_WIDTH_CONST-1:0] build_search_chunk_snapshot(
    input int unsigned part_idx,
    input int unsigned chunk_idx
  );
    logic [SEARCH_CHUNK_WIDTH_CONST-1:0] snap;
    int unsigned local_idx;
    int unsigned global_idx;

    snap = '0;
    for (int chunk_bit = 0; chunk_bit < SEARCH_CHUNK_WIDTH_CONST; chunk_bit++) begin
      local_idx = chunk_idx * SEARCH_CHUNK_WIDTH_CONST + chunk_bit;
      global_idx = partition_base_addr(part_idx) + local_idx;
      if (local_idx < PARTITION_SIZE_CONST &&
          global_idx < RING_BUFFER_N_ENTRY &&
          slot_valid[global_idx] &&
          cam_match_addr_oh[global_idx]) begin
        snap[chunk_bit] = 1'b1;
      end
    end
    return snap;
  endfunction

  function automatic logic [15:0] count_search_chunk(
    input logic [SEARCH_CHUNK_WIDTH_CONST-1:0] snap
  );
    logic [15:0] count;
    logic [2:0] group_count;

    count = '0;
    for (int group_base = 0; group_base < SEARCH_CHUNK_WIDTH_CONST; group_base += 4) begin
      group_count = '0;
      for (int group_bit = 0; group_bit < 4; group_bit++) begin
        if ((group_base + group_bit) < SEARCH_CHUNK_WIDTH_CONST &&
            snap[group_base + group_bit]) begin
          group_count = group_count + 3'd1;
        end
      end
      count = count + 16'(group_count);
    end
    return count;
  endfunction

  function automatic int find_next_search_chunk_local(
    input logic [SEARCH_CHUNK_WIDTH_CONST-1:0] snap
  );
    for (int i = 0; i < SEARCH_CHUNK_WIDTH_CONST; i++) begin
      if (snap[i]) begin
        return i;
      end
    end
    return -1;
  endfunction

  function automatic logic [SEARCH_CHUNK_WIDTH_CONST-1:0] partition_chunk_snapshot(
    input logic [PARTITION_SIZE_CONST-1:0] snap,
    input int unsigned chunk_idx
  );
    logic [SEARCH_CHUNK_WIDTH_CONST-1:0] chunk;
    int unsigned local_idx;

    chunk = '0;
    for (int chunk_bit = 0; chunk_bit < SEARCH_CHUNK_WIDTH_CONST; chunk_bit++) begin
      local_idx = chunk_idx * SEARCH_CHUNK_WIDTH_CONST + chunk_bit;
      if (local_idx < PARTITION_SIZE_CONST) begin
        chunk[chunk_bit] = snap[local_idx];
      end
    end
    return chunk;
  endfunction

  function automatic int find_next_active_chunk(
    input logic [SEARCH_CHUNK_COUNT_CONST-1:0] active,
    input int unsigned start_chunk
  );
    for (int chunk_idx = 0; chunk_idx < SEARCH_CHUNK_COUNT_CONST; chunk_idx++) begin
      if (chunk_idx >= start_chunk && active[chunk_idx]) begin
        return chunk_idx;
      end
    end
    return -1;
  endfunction

  function automatic logic [LOCK_SECTOR_COUNT_CONST-1:0] search_chunk_sector_mask(
    input int unsigned part_idx,
    input int unsigned chunk_idx,
    input logic [SEARCH_CHUNK_WIDTH_CONST-1:0] snap
  );
    logic [LOCK_SECTOR_COUNT_CONST-1:0] mask;
    int unsigned local_idx;
    logic [ADDR_W_CONST-1:0] addr;

    mask = '0;
    for (int chunk_bit = 0; chunk_bit < SEARCH_CHUNK_WIDTH_CONST; chunk_bit++) begin
      if (snap[chunk_bit]) begin
        local_idx = chunk_idx * SEARCH_CHUNK_WIDTH_CONST + chunk_bit;
        if (local_idx < PARTITION_SIZE_CONST) begin
          addr = partition_global_addr(part_idx, local_idx);
          mask[addr_sector(addr)] = 1'b1;
        end
      end
    end
    return mask;
  endfunction

  function automatic logic [LOCK_SECTOR_COUNT_CONST-1:0] search_chunk_complete_sector_mask(
    input int unsigned part_idx,
    input int unsigned chunk_idx
  );
    logic [LOCK_SECTOR_COUNT_CONST-1:0] mask;
    int unsigned local_idx;
    int unsigned global_idx;
    logic [ADDR_W_CONST-1:0] addr;
    logic [ADDR_W_CONST-1:0] next_addr;

    mask = '0;
    for (int chunk_bit = 0; chunk_bit < SEARCH_CHUNK_WIDTH_CONST; chunk_bit++) begin
      local_idx = chunk_idx * SEARCH_CHUNK_WIDTH_CONST + chunk_bit;
      global_idx = partition_base_addr(part_idx) + local_idx;
      if (local_idx < PARTITION_SIZE_CONST &&
          global_idx < RING_BUFFER_N_ENTRY) begin
        addr = ADDR_W_CONST'(global_idx);
        if (global_idx == (RING_BUFFER_N_ENTRY - 1)) begin
          mask[addr_sector(addr)] = 1'b1;
        end else begin
          next_addr = ADDR_W_CONST'(global_idx + 1);
          if (addr_sector(next_addr) != addr_sector(addr)) begin
            mask[addr_sector(addr)] = 1'b1;
          end
        end
      end
    end
    return mask;
  endfunction

  function automatic logic [LOCK_SECTOR_COUNT_CONST-1:0] partition_sector_mask(
    input int unsigned part_idx,
    input logic [PARTITION_SIZE_CONST-1:0] snap
  );
    logic [LOCK_SECTOR_COUNT_CONST-1:0] mask;
    logic [ADDR_W_CONST-1:0] addr;

    mask = '0;
    for (int local_idx = 0; local_idx < PARTITION_SIZE_CONST; local_idx++) begin
      if (snap[local_idx]) begin
        addr = partition_global_addr(part_idx, local_idx);
        mask[addr_sector(addr)] = 1'b1;
      end
    end
    return mask;
  endfunction

  function automatic logic [31:0] csr_read_data(input logic [4:0] addr);
    logic [31:0] data;
    data = '0;
    unique case (addr)
      5'd0: data = IP_UID[31:0];
      5'd1: begin
        unique case (csr_meta_sel)
          2'd0: data = {
            VERSION_MAJOR[7:0],
            VERSION_MINOR[7:0],
            VERSION_PATCH[3:0],
            BUILD[11:0]
          };
          2'd1: data = VERSION_DATE[31:0];
          2'd2: data = VERSION_GIT[31:0];
          default: data = INSTANCE_ID[31:0];
        endcase
      end
      5'd2: data = {26'd0, csr_counter_freeze, csr_filter_inerr, 2'd0, 1'b0, csr_go};
      5'd3: data = csr_expected_latency;
      5'd4: data = avs_fill_level_word();
      5'd5: data = counter_word(debug_msg2.inerr_cnt, debug_msg2_snap.inerr_cnt, 1'b0);
      5'd6: data = counter_word(debug_msg2.push_cnt, debug_msg2_snap.push_cnt, 1'b0);
      5'd7: data = counter_word(debug_msg2.pop_cnt, debug_msg2_snap.pop_cnt, 1'b0);
      5'd8: data = counter_word(debug_msg2.overwrite_cnt, debug_msg2_snap.overwrite_cnt, 1'b0);
      5'd9: data = counter_word(debug_msg2.cache_miss_cnt, debug_msg2_snap.cache_miss_cnt, 1'b0);
      5'd10: data = counter_word(debug_msg2.inerr_cnt, debug_msg2_snap.inerr_cnt, 1'b1);
      5'd11: data = counter_word(debug_msg2.push_cnt, debug_msg2_snap.push_cnt, 1'b1);
      5'd12: data = counter_word(debug_msg2.pop_cnt, debug_msg2_snap.pop_cnt, 1'b1);
      5'd13: data = counter_word(debug_msg2.overwrite_cnt, debug_msg2_snap.overwrite_cnt, 1'b1);
      5'd14: data = counter_word(debug_msg2.cache_miss_cnt, debug_msg2_snap.cache_miss_cnt, 1'b1);
      5'd15: data = counter_word(debug_msg2.deasm_full_drop_cnt, debug_msg2_snap.deasm_full_drop_cnt, 1'b0);
      5'd16: data = counter_word(debug_msg2.deasm_full_drop_cnt, debug_msg2_snap.deasm_full_drop_cnt, 1'b1);
      5'd17: data = counter_word(debug_msg2.pop_cmd_full_drop_cnt, debug_msg2_snap.pop_cmd_full_drop_cnt, 1'b0);
      5'd18: data = counter_word(debug_msg2.pop_cmd_full_drop_cnt, debug_msg2_snap.pop_cmd_full_drop_cnt, 1'b1);
      5'd19: data = counter_word(debug_msg2.egress_not_ready_drop_cnt, debug_msg2_snap.egress_not_ready_drop_cnt, 1'b0);
      5'd20: data = counter_word(debug_msg2.egress_not_ready_drop_cnt, debug_msg2_snap.egress_not_ready_drop_cnt, 1'b1);
      default: data = '0;
    endcase
    return data;
  endfunction

  ring_buffer_cam_fifo #(
    .WIDTH(DEASM_WORD_BITS_CONST),
    .DEPTH(DEASM_DEPTH_CONST)
  ) deassembly_fifo (
    .clk(i_clk),
    .rst(i_rst),
    .sclr(deassembly_fifo_sclr),
    .wrreq(deassembly_fifo_wrreq),
    .rdack(deassembly_fifo_rdack),
    .din(deassembly_fifo_din),
    .dout(deassembly_fifo_dout),
    .empty(deassembly_fifo_empty),
    .full(deassembly_fifo_full),
    .usedw(deassembly_fifo_usedw_int)
  );

  ring_buffer_cam_fifo #(
    .WIDTH(9),
    .DEPTH(POP_CMD_DEPTH_CONST)
  ) pop_cmd_fifo (
    .clk(i_clk),
    .rst(i_rst),
    .sclr(pop_cmd_fifo_sclr),
    .wrreq(pop_cmd_fifo_wrreq),
    .rdack(pop_cmd_fifo_rdack),
    .din(pop_cmd_fifo_din),
    .dout(pop_cmd_fifo_dout),
    .empty(pop_cmd_fifo_empty),
    .full(pop_cmd_fifo_full),
    .usedw(pop_cmd_fifo_usedw_int)
  );

  always_comb begin
    ingress_lane_match = lane_matches(
      asi_hit_type1_data,
      asi_hit_type1_channel,
      asi_hit_type1_empty,
      INTERLEAVING_INDEX
    );
    ingress_active_window =
      (run_state_cmd == RUN_RUNNING ||
       (run_state_cmd == RUN_TERMINATING && !endofrun_seen)) &&
      csr_go &&
      asi_hit_type1_valid &&
      !asi_hit_type1_empty &&
      ingress_lane_match;
    ingress_payload_countable =
      ingress_active_window &&
      (!csr_filter_inerr || !asi_hit_type1_error[0]);
    deasm_full_drop_event = ingress_payload_countable && deassembly_fifo_full;

    deassembly_fifo_din = '0;
    deassembly_fifo_din.hit_word = {1'b0, asi_hit_type1_data};
    deassembly_fifo_din.metadata =
      (DEBUG_METADATA_EN_CONST && asi_hit_type1_metadata_valid) ?
      asi_hit_type1_metadata : 64'd0;
    deassembly_fifo_din.metadata_valid =
      DEBUG_METADATA_EN_CONST && asi_hit_type1_metadata_valid;
    deassembly_fifo_wrreq = 1'b0;
    if (ingress_payload_countable && !deassembly_fifo_full) begin
      deassembly_fifo_wrreq = 1'b1;
    end

    asi_hit_type1_ready =
      !csr_soft_reset_pulse &&
      csr_go &&
      (run_state_cmd == RUN_RUNNING ||
       (run_state_cmd == RUN_TERMINATING && !endofrun_seen)) &&
      !deassembly_fifo_full;

    in_payload_valid = !deassembly_fifo_empty;
    in_hit_side = deassembly_fifo_dout.hit_word[38:0];
    in_hit_ts8n = hit_tcc8n(deassembly_fifo_dout.hit_word[38:0]);
    in_hit_sk = hit_search_key(deassembly_fifo_dout.hit_word[38:0]);
    push_write_req = in_payload_valid &&
      (run_state_cmd == RUN_RUNNING || run_state_cmd == RUN_TERMINATING) &&
      !csr_soft_reset_pulse;
    push_erase_req = (push_state == PUSH_ERASING);
    deassembly_fifo_rdack = push_write_grant;
  end

  always_comb begin
    if (gts_8n >= expected_latency_48b) begin
      read_time_ptr_comb = gts_8n - expected_latency_48b;
    end else begin
      read_time_ptr_comb = 48'd1;
    end
  end

  always_comb begin
    pop_cmd_fifo_wrreq = 1'b0;
    pop_cmd_fifo_din = '0;
    pop_cmd_tick_due =
      (run_state_cmd == RUN_RUNNING ||
       (run_state_cmd == RUN_TERMINATING && !terminating_drain_done)) &&
      read_time_ptr[3:0] == 4'h0 &&
      read_time_ptr[5:4] == INTERLEAVING_INDEX[1:0];
    pop_cmd_full_drop_event = pop_cmd_tick_due && pop_cmd_fifo_full;
    if (pop_cmd_tick_due && !pop_cmd_fifo_full) begin
      pop_cmd_fifo_wrreq = 1'b1;
      pop_cmd_fifo_din = read_time_ptr[12:4];
    end
  end

  always_comb begin
    push_write_grant = 1'b0;
    push_erase_grant = 1'b0;
    pop_erase_grant = 1'b0;
    pop_flush_grant = 1'b0;
    push_write_allowed = 1'b0;
    push_write_sector_locked = 1'b0;
    push_erase_sector_locked = 1'b0;
    if (pop_search_collecting) begin
      pop_sector_lock_mask =
        pop_snapshot_sector_mask | pop_search_unscanned_sector_mask;
    end else if ((pop_engine_state == POP_LOADING) ||
        (pop_engine_state == POP_COUNTING) ||
        (pop_engine_state == POP_DRAINING)) begin
      pop_sector_lock_mask = pop_snapshot_sector_mask;
    end else begin
      pop_sector_lock_mask = '0;
    end

    if (pop_issue_inflight || pop_output_pending || pop_emit_pending) begin
      pop_sector_lock_mask[
        addr_sector(pop_issue_addr_pending[ADDR_W_CONST-1:0])
      ] = 1'b1;
    end
    if (pop_erase_req) begin
      pop_sector_lock_mask[
        addr_sector(pop_issue_addr[ADDR_W_CONST-1:0])
      ] = 1'b1;
    end
    if (pop_search_collecting ||
        (pop_engine_state == POP_LOADING) ||
        (pop_engine_state == POP_COUNTING) ||
        (pop_engine_state == POP_DRAINING)) begin
      push_write_sector_locked =
        pop_sector_lock_mask[addr_sector(write_pointer)];
      push_erase_sector_locked =
        pop_sector_lock_mask[addr_sector(push_erase_addr_reg)];
    end

    push_write_allowed =
      push_write_req &&
      !push_erase_req &&
      !pop_flush_req &&
      ((pop_engine_state != POP_SEARCHING) ||
       !push_write_search_key_conflict) &&
      !push_write_sector_locked;

    if (csr_soft_reset_pulse) begin
      push_write_grant = 1'b0;
      push_erase_grant = 1'b0;
      pop_erase_grant = 1'b0;
      pop_flush_grant = 1'b0;
    end else if (pop_flush_req) begin
      pop_flush_grant = 1'b1;
    end else begin
      push_erase_grant =
        push_erase_req &&
        (pop_engine_state != POP_SEARCHING) &&
        !push_erase_sector_locked;
      if (push_erase_grant) begin
        pop_erase_grant = 1'b0;
        push_write_grant = 1'b0;
      end else if (pop_erase_req) begin
        pop_erase_grant = 1'b1;
        push_write_grant = 1'b0;
      end else begin
        pop_erase_grant = 1'b0;
        push_write_grant = push_write_allowed;
      end
    end

    decision = 3'd4;
    if (pop_flush_grant) begin
      decision = 3'd3;
    end else if (pop_erase_grant) begin
      decision = 3'd2;
    end else if (push_erase_grant) begin
      decision = 3'd1;
    end else if (push_write_grant) begin
      decision = 3'd0;
    end

    cam_wr_en = 1'b0;
    cam_erase_en = 1'b0;
    cam_wr_addr = {{(16-ADDR_W_CONST){1'b0}}, write_pointer};
    cam_wr_data = MAIN_CAM_DATA_WIDTH_CONST'(in_hit_sk);
    side_ram_we = 1'b0;
    side_ram_waddr = {{(16-ADDR_W_CONST){1'b0}}, write_pointer};
    side_ram_din = {1'b1, in_hit_side};
    side_ram_raddr = {{(16-ADDR_W_CONST){1'b0}}, write_pointer};

    if (push_write_grant) begin
      cam_wr_en = 1'b1;
      side_ram_we = 1'b1;
      side_ram_waddr = {{(16-ADDR_W_CONST){1'b0}}, write_pointer};
      side_ram_din = {1'b1, in_hit_side};
    end

    if (push_erase_grant && !pop_erase_grant && !push_write_grant) begin
      cam_erase_en = (push_erase_sk != push_write_sk_reg);
      cam_wr_addr = {{(16-ADDR_W_CONST){1'b0}}, push_erase_addr_reg};
      cam_wr_data = MAIN_CAM_DATA_WIDTH_CONST'(push_erase_sk);
      side_ram_waddr = {{(16-ADDR_W_CONST){1'b0}}, push_erase_addr_reg};
    end

    if (pop_erase_grant) begin
      cam_erase_en = 1'b1;
      cam_wr_addr = pop_issue_addr;
      cam_wr_data = MAIN_CAM_DATA_WIDTH_CONST'(pop_current_sk[7:0]);
      side_ram_raddr = pop_issue_addr;
      if (!push_write_grant) begin
        side_ram_we = 1'b1;
        side_ram_waddr = pop_issue_addr;
        side_ram_din = '0;
      end
    end

    if (pop_flush_grant) begin
      cam_erase_en = !pop_flush_cam_done;
      cam_wr_addr = flush_cam_wraddr;
      cam_wr_data = flush_cam_wrdata;
      side_ram_we = !pop_flush_ram_done;
      side_ram_waddr = flush_ram_wraddr;
      side_ram_din = '0;
    end
  end

  // synthesis translate_off
  always_ff @(posedge i_clk) begin
    if (!i_rst && !csr_soft_reset_pulse) begin
      assert (!(push_write_grant && pop_erase_grant))
        else $error("single CAM port conflict: push_write_grant and pop_erase_grant asserted together");
    end
  end
  // synthesis translate_on

  always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
      cam_wr_en_d1 <= 1'b0;
      cam_erase_en_d1 <= 1'b0;
      cam_wr_addr_d1 <= '0;
      cam_wr_data_d1 <= '0;
    end else if (csr_soft_reset_pulse) begin
      cam_wr_en_d1 <= 1'b0;
      cam_erase_en_d1 <= 1'b0;
      cam_wr_addr_d1 <= '0;
      cam_wr_data_d1 <= '0;
    end else begin
      cam_wr_en_d1 <= cam_wr_en;
      cam_erase_en_d1 <= cam_erase_en;
      cam_wr_addr_d1 <= cam_wr_addr[ADDR_W_CONST-1:0];
      cam_wr_data_d1 <= cam_wr_data;
    end
  end

  always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
      avs_csr_readdata <= '0;
      csr_go <= 1'b1;
      csr_soft_reset_pulse <= 1'b0;
      csr_filter_inerr <= 1'b1;
      csr_counter_freeze <= 1'b0;
      csr_expected_latency <= DEFAULT_EXPECTED_LATENCY_CONST;
      csr_meta_sel <= 2'd0;
      debug_msg2_snap <= '0;
      debug_msg2_snap.cam_clean <= 1'b1;
    end else begin
      csr_soft_reset_pulse <= 1'b0;
      if (avs_csr_read) begin
        avs_csr_readdata <= csr_read_data(avs_csr_address);
      end else begin
        avs_csr_readdata <= csr_read_data(avs_csr_address);
      end
      if (csr_soft_reset_pulse) begin
        csr_counter_freeze <= 1'b0;
        debug_msg2_snap <= '0;
        debug_msg2_snap.cam_clean <= 1'b1;
      end else if (avs_csr_write) begin
        unique case (avs_csr_address)
          5'd1: csr_meta_sel <= avs_csr_writedata[1:0];
          5'd2: begin
            csr_go <= avs_csr_writedata[0];
            csr_filter_inerr <= avs_csr_writedata[4];
            csr_counter_freeze <= avs_csr_writedata[5] && !avs_csr_writedata[1];
            if (avs_csr_writedata[5] && !avs_csr_writedata[1]) begin
              debug_msg2_snap <= debug_msg2;
            end
            if (avs_csr_writedata[1]) begin
              csr_soft_reset_pulse <= 1'b1;
              csr_counter_freeze <= 1'b0;
              debug_msg2_snap <= '0;
              debug_msg2_snap.cam_clean <= 1'b1;
            end
          end
          5'd3: csr_expected_latency <= avs_csr_writedata;
          default: begin
          end
        endcase
      end
    end
  end

  always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
      run_state_cmd <= RUN_IDLING;
      asi_ctrl_ready <= 1'b0;
      endofrun_seen <= 1'b0;
      gts_8n <= '0;
      gts_end_of_run <= '0;
      read_time_ptr <= '0;
      run_mgmt_flush_memory_start <= 1'b0;
      run_mgmt_flush_memory_done <= 1'b0;
      run_mgmt_flushed <= 1'b0;
      pop_flush_ram_done <= 1'b0;
      pop_flush_cam_done <= 1'b0;
      flush_ram_wraddr <= '0;
      flush_cam_wraddr <= '0;
      flush_cam_wrdata <= '0;
      flush_cycle_count <= '0;
      prep_ready_latched <= 1'b0;
    end else begin
      if (csr_soft_reset_pulse) begin
        run_state_cmd <= RUN_IDLING;
        asi_ctrl_ready <= 1'b0;
        endofrun_seen <= 1'b0;
        gts_8n <= '0;
        gts_end_of_run <= '0;
        read_time_ptr <= '0;
        run_mgmt_flush_memory_start <= 1'b0;
        run_mgmt_flush_memory_done <= 1'b0;
        run_mgmt_flushed <= 1'b0;
        pop_flush_ram_done <= 1'b0;
        pop_flush_cam_done <= 1'b0;
        flush_ram_wraddr <= '0;
        flush_cam_wraddr <= '0;
        flush_cam_wrdata <= '0;
        flush_cycle_count <= '0;
        prep_ready_latched <= 1'b0;
      end else begin
        if (run_state_cmd == RUN_RUNNING || run_state_cmd == RUN_TERMINATING) begin
          gts_8n <= gts_8n + 48'd1;
          read_time_ptr <= read_time_ptr_comb;
        end else begin
          read_time_ptr <= '0;
        end

        if (asi_hit_type1_valid && asi_hit_type1_empty &&
            lane_matches(asi_hit_type1_data, asi_hit_type1_channel, 1'b1, INTERLEAVING_INDEX) &&
            run_state_cmd == RUN_TERMINATING) begin
          endofrun_seen <= 1'b1;
          gts_end_of_run <= gts_8n;
        end

        if (run_state_cmd == RUN_PREPARING) begin
          run_mgmt_flush_memory_start <= 1'b1;

          if (flush_cycle_count == 16'd0) begin
            flush_cycle_count <= 16'd1;
          end

          if (!run_mgmt_flushed) begin
            run_mgmt_flushed <= 1'b0;
            run_mgmt_flush_memory_done <= 1'b0;
            prep_ready_latched <= 1'b0;

            if (!pop_flush_ram_done && pop_flush_grant) begin
              if (flush_ram_wraddr[ADDR_W_CONST-1:0] == RAM_FLUSH_ADDR_MAX_CONST) begin
                pop_flush_ram_done <= 1'b1;
              end else begin
                flush_ram_wraddr <= flush_ram_wraddr + 16'd1;
              end
            end

            if (!pop_flush_cam_done && pop_flush_grant) begin
              if (flush_cam_wrdata == CAM_FLUSH_DATA_MAX_CONST) begin
                flush_cam_wrdata <= '0;
                if (flush_cam_wraddr[ADDR_W_CONST-1:0] == CAM_FLUSH_ADDR_MAX_CONST) begin
                  pop_flush_cam_done <= 1'b1;
                end else begin
                  flush_cam_wraddr <= flush_cam_wraddr + 16'd1;
                end
              end else begin
                flush_cam_wrdata <= flush_cam_wrdata + MAIN_CAM_DATA_WIDTH_CONST'(1);
              end
            end

            if ((pop_flush_ram_done ||
                 (!pop_flush_ram_done && pop_flush_grant &&
                  (flush_ram_wraddr[ADDR_W_CONST-1:0] == RAM_FLUSH_ADDR_MAX_CONST))) &&
                (pop_flush_cam_done ||
                 (!pop_flush_cam_done && pop_flush_grant &&
                  (flush_cam_wraddr[ADDR_W_CONST-1:0] == CAM_FLUSH_ADDR_MAX_CONST) &&
                  (flush_cam_wrdata == CAM_FLUSH_DATA_MAX_CONST)))) begin
              run_mgmt_flush_memory_done <= 1'b1;
              run_mgmt_flushed <= 1'b1;
            end
          end else begin
            prep_ready_latched <= 1'b1;
          end
        end

        asi_ctrl_ready <= 1'b0;
        if (asi_ctrl_valid) begin
          unique case (asi_ctrl_data)
            CTRL_IDLE_CONST: begin
              run_state_cmd <= RUN_IDLING;
              asi_ctrl_ready <= 1'b1;
              endofrun_seen <= 1'b0;
            end
            CTRL_RUN_PREPARE_CONST: begin
              run_state_cmd <= RUN_PREPARING;
              asi_ctrl_ready <= prep_ready_latched;
              if (run_state_cmd != RUN_PREPARING) begin
                flush_cycle_count <= '0;
                flush_ram_wraddr <= '0;
                flush_cam_wraddr <= '0;
                flush_cam_wrdata <= '0;
                pop_flush_ram_done <= 1'b0;
                pop_flush_cam_done <= 1'b0;
                run_mgmt_flush_memory_start <= 1'b1;
                run_mgmt_flush_memory_done <= 1'b0;
                run_mgmt_flushed <= 1'b0;
                prep_ready_latched <= 1'b0;
                endofrun_seen <= 1'b0;
                gts_8n <= '0;
                read_time_ptr <= '0;
              end
            end
            CTRL_SYNC_CONST: begin
              run_state_cmd <= RUN_SYNCING;
              asi_ctrl_ready <= 1'b1;
              endofrun_seen <= 1'b0;
            end
            CTRL_RUNNING_CONST: begin
              run_state_cmd <= RUN_RUNNING;
              asi_ctrl_ready <= 1'b1;
              endofrun_seen <= 1'b0;
            end
            CTRL_TERMINATING_CONST: begin
              run_state_cmd <= RUN_TERMINATING;
              asi_ctrl_ready <= terminating_drain_done;
            end
            CTRL_LINK_TEST_CONST: begin
              run_state_cmd <= RUN_LINK_TESTING;
              asi_ctrl_ready <= 1'b0;
            end
            default: begin
              run_state_cmd <= RUN_ERRORING;
              asi_ctrl_ready <= 1'b0;
            end
          endcase
        end
      end
    end
  end

  always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
      for (int i = 0; i < RING_BUFFER_N_ENTRY; i++) begin
        slot_valid[i] <= 1'b0;
        slot_metadata[i] <= '0;
        slot_metadata_valid[i] <= 1'b0;
      end
      write_pointer <= '0;
      push_state <= PUSH_WRITING;
      push_write_sk_reg <= '0;
      push_erase_addr_reg <= '0;
      push_erase_sk_reg <= '0;
      push_erase_sk_valid <= 1'b0;
      side_ram_dout <= '0;
      pop_issue_addr_pending <= '0;
      pop_hit_pending <= '0;
      pop_metadata_pending <= '0;
      pop_metadata_valid_pending <= 1'b0;
      pop_occupied_pending <= 1'b0;
      pop_output_pending <= 1'b0;
      pop_side_read_pending <= 1'b0;
      resident_fill_level <= '0;
      debug_msg2 <= '0;
      debug_msg2.cam_clean <= 1'b1;
    end else begin
      debug_msg2.cam_clean <=
        (debug_msg2.push_cnt == (debug_msg2.pop_cnt + debug_msg2.overwrite_cnt));

      if (csr_soft_reset_pulse ||
          (run_state_cmd == RUN_PREPARING && flush_cycle_count == 16'd0)) begin
        for (int i = 0; i < RING_BUFFER_N_ENTRY; i++) begin
          slot_valid[i] <= 1'b0;
          slot_metadata[i] <= '0;
          slot_metadata_valid[i] <= 1'b0;
        end
        write_pointer <= '0;
        push_state <= PUSH_WRITING;
        push_write_sk_reg <= '0;
        push_erase_addr_reg <= '0;
        push_erase_sk_reg <= '0;
        push_erase_sk_valid <= 1'b0;
        side_ram_dout <= '0;
        pop_issue_addr_pending <= '0;
        pop_hit_pending <= '0;
        pop_metadata_pending <= '0;
        pop_metadata_valid_pending <= 1'b0;
        pop_occupied_pending <= 1'b0;
        pop_output_pending <= 1'b0;
        pop_side_read_pending <= 1'b0;
        resident_fill_level <= '0;
        debug_msg2 <= '0;
        debug_msg2.cam_clean <= 1'b1;
      end else begin
        resident_fill_level <= resident_fill_level +
          (push_write_grant ? 32'd1 : 32'd0) -
          (pop_erase_grant ? 32'd1 : 32'd0) -
          (push_erase_grant ? 32'd1 : 32'd0);

        if (asi_hit_type1_valid && asi_hit_type1_error[0] && asi_hit_type1_ready &&
            !asi_hit_type1_empty && csr_filter_inerr) begin
          debug_msg2.inerr_cnt <= debug_msg2.inerr_cnt + 64'd1;
        end

        if (deasm_full_drop_event) begin
          debug_msg2.deasm_full_drop_cnt <= debug_msg2.deasm_full_drop_cnt + 64'd1;
        end

        if (pop_cmd_full_drop_event) begin
          debug_msg2.pop_cmd_full_drop_cnt <= debug_msg2.pop_cmd_full_drop_cnt + 64'd1;
        end

        if (egress_not_ready_drop_event) begin
          debug_msg2.egress_not_ready_drop_cnt <= debug_msg2.egress_not_ready_drop_cnt + 64'd1;
        end

        if (push_write_grant) begin
          push_write_sk_reg <= in_hit_sk;
          push_erase_addr_reg <= write_pointer;
          push_erase_sk_reg <= '0;
          push_erase_sk_valid <= 1'b0;
          slot_valid[write_pointer] <= 1'b1;
          slot_metadata[write_pointer] <=
            DEBUG_METADATA_EN_CONST ? deassembly_fifo_dout.metadata : 64'd0;
          slot_metadata_valid[write_pointer] <=
            DEBUG_METADATA_EN_CONST && deassembly_fifo_dout.metadata_valid;
          write_pointer <= ptr_inc(write_pointer);
          debug_msg2.push_cnt <= debug_msg2.push_cnt + 64'd1;
          push_state <= slot_valid[write_pointer] ? PUSH_ERASING : PUSH_WRITING;
        end else if (push_erase_grant) begin
          debug_msg2.overwrite_cnt <= debug_msg2.overwrite_cnt + 64'd1;
          push_state <= PUSH_WRITING;
          push_erase_sk_reg <= '0;
          push_erase_sk_valid <= 1'b0;
        end else if (push_state == PUSH_ERASING && !push_erase_sk_valid) begin
          push_erase_sk_reg <= hit_search_key(side_ram_q[38:0]);
          push_erase_sk_valid <= 1'b1;
        end

        if (pop_erase_grant) begin
          pop_issue_addr_pending <= pop_issue_addr;
          pop_metadata_pending <= DEBUG_METADATA_EN_CONST ?
            slot_metadata[pop_issue_addr[ADDR_W_CONST-1:0]] : 64'd0;
          pop_metadata_valid_pending <= DEBUG_METADATA_EN_CONST &&
            slot_metadata_valid[pop_issue_addr[ADDR_W_CONST-1:0]];
          slot_valid[pop_issue_addr[ADDR_W_CONST-1:0]] <= 1'b0;
          slot_metadata_valid[pop_issue_addr[ADDR_W_CONST-1:0]] <= 1'b0;
          debug_msg2.pop_cnt <= debug_msg2.pop_cnt + 64'd1;
          pop_side_read_pending <= 1'b1;
        end else if (pop_side_read_pending) begin
          side_ram_dout <= side_ram_q;
          pop_hit_pending <= side_ram_q[38:0];
          pop_occupied_pending <= side_ram_q[39];
          pop_output_pending <= 1'b1;
          pop_side_read_pending <= 1'b0;
        end else if (pop_hit_valid) begin
          pop_output_pending <= 1'b0;
        end

        if (pop_cache_miss_pulse) begin
          debug_msg2.cache_miss_cnt <= debug_msg2.cache_miss_cnt + 64'd1;
        end
      end
    end
  end

  always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
      pop_engine_state <= POP_IDLING;
      for (int part_idx = 0; part_idx < PARTITION_COUNT_CONST; part_idx++) begin
        pop_partition_snapshot[part_idx] <= '0;
        pop_partition_head_local[part_idx] <= '0;
        pop_partition_chunk_active[part_idx] <= '0;
        pop_partition_hit_count[part_idx] <= '0;
        pop_partition_sector_mask[part_idx] <= '0;
      end
      pop_partition_active <= '0;
      pop_partition_head_valid <= '0;
      pop_snapshot_sector_mask <= '0;
      pop_search_unscanned_sector_mask <= '0;
      pop_current_sk <= '0;
      pop_total_hits <= '0;
      pop_hits_count <= '0;
      pop_issue_addr <= '0;
      pop_cmd_fifo_rdack <= 1'b0;
      pop_erase_req <= 1'b0;
      pop_flush_req <= 1'b0;
      pop_pipeline_start <= 1'b0;
      pop_hit_valid <= 1'b0;
      pop_cache_miss_pulse <= 1'b0;
      pop_last_hit_pending <= 1'b0;
      subheader_gen_done <= 1'b0;
      pop_search_wait_cnt <= '0;
      pop_search_partition_idx <= '0;
      pop_search_chunk_idx <= '0;
      pop_search_capture_done <= 1'b0;
      pop_search_capture_stage_valid <= 1'b0;
      pop_search_capture_stage_partition_idx <= '0;
      pop_search_capture_stage_chunk_idx <= '0;
      pop_search_capture_stage_snapshot <= '0;
      pop_search_chunk_stage_valid <= 1'b0;
      pop_search_stage_partition_idx <= '0;
      pop_search_stage_chunk_idx <= '0;
      pop_search_stage_snapshot <= '0;
      pop_search_stage_sector_mask <= '0;
      pop_search_stage_hit_count <= '0;
      pop_search_stage_head_valid <= 1'b0;
      pop_search_stage_head_local <= '0;
      pop_drain_partition_idx <= '0;
      pop_head_update_valid <= 1'b0;
      pop_head_update_partition_idx <= '0;
      pop_head_update_chunk_idx <= '0;
      pop_head_update_snapshot <= '0;
      pop_head_update_scan_valid <= 1'b0;
      pop_head_update_scan_partition_idx <= '0;
      pop_head_update_scan_chunk_idx <= '0;
      pop_head_update_scan_snapshot <= '0;
      pop_head_update_scan_chunk_active <= '0;
      pop_head_update_scan_part_mask <= '0;
      pop_head_update_scan_hit_count <= '0;
      pop_head_update_apply_valid <= 1'b0;
      pop_head_update_apply_partition_idx <= '0;
      pop_head_update_apply_chunk_active <= '0;
      pop_head_update_apply_part_mask <= '0;
      pop_head_update_apply_hit_count <= '0;
      pop_head_update_apply_next_chunk_valid <= 1'b0;
      pop_head_update_apply_next_chunk_idx <= '0;
      pop_head_update_apply_chunk_snapshot <= '0;
      pop_head_update_apply_head_local <= '0;
      dbg_pop_rr_idx <= '0;
      dbg_pop_issue_partition_idx <= '0;
      dbg_pop_count_partition_idx <= '0;
      dbg_pop_partition_pending <= '0;
      dbg_pop_partition_load <= '0;
      dbg_pop_partition_advance <= '0;
      dbg_pop_partition_result_valid <= '0;
      dbg_pop_partition_flag <= '0;
      dbg_pop_partition_has_more <= '0;
      dbg_pop_partition_eval_stage0_valid <= '0;
      aso_hit_type2_valid <= 1'b0;
      aso_hit_type2_channel <= '0;
      aso_hit_type2_startofpacket <= 1'b0;
      aso_hit_type2_endofpacket <= 1'b0;
      aso_hit_type2_data <= '0;
      aso_hit_type2_error <= '0;
      aso_hit_type2_metadata <= '0;
      aso_hit_type2_metadata_valid <= 1'b0;
      pop_issue_inflight <= 1'b0;
      pop_emit_pending <= 1'b0;
    end else begin
      pop_cmd_fifo_rdack <= 1'b0;
      pop_erase_req <= 1'b0;
      pop_flush_req <= 1'b0;
      pop_hit_valid <= 1'b0;
      pop_cache_miss_pulse <= 1'b0;
      pop_last_hit_pending <= 1'b0;
      aso_hit_type2_valid <= 1'b0;
      aso_hit_type2_startofpacket <= 1'b0;
      aso_hit_type2_endofpacket <= 1'b0;
      aso_hit_type2_data <= '0;
      aso_hit_type2_error <= '0;
      aso_hit_type2_metadata <= '0;
      aso_hit_type2_metadata_valid <= 1'b0;
      dbg_pop_partition_load <= '0;
      dbg_pop_partition_advance <= '0;
      dbg_pop_partition_eval_stage0_valid <= '0;

      if (csr_soft_reset_pulse) begin
        pop_engine_state <= POP_IDLING;
        for (int part_idx = 0; part_idx < PARTITION_COUNT_CONST; part_idx++) begin
          pop_partition_snapshot[part_idx] <= '0;
          pop_partition_head_local[part_idx] <= '0;
          pop_partition_chunk_active[part_idx] <= '0;
          pop_partition_hit_count[part_idx] <= '0;
          pop_partition_sector_mask[part_idx] <= '0;
        end
        pop_partition_active <= '0;
        pop_partition_head_valid <= '0;
        pop_snapshot_sector_mask <= '0;
        pop_search_unscanned_sector_mask <= '0;
        pop_current_sk <= '0;
        pop_total_hits <= '0;
        pop_hits_count <= '0;
        pop_issue_addr <= '0;
        pop_pipeline_start <= 1'b0;
        subheader_gen_done <= 1'b0;
        pop_search_wait_cnt <= '0;
        pop_search_partition_idx <= '0;
        pop_search_chunk_idx <= '0;
        pop_search_capture_done <= 1'b0;
        pop_search_capture_stage_valid <= 1'b0;
        pop_search_capture_stage_partition_idx <= '0;
        pop_search_capture_stage_chunk_idx <= '0;
        pop_search_capture_stage_snapshot <= '0;
        pop_search_chunk_stage_valid <= 1'b0;
        pop_search_stage_partition_idx <= '0;
        pop_search_stage_chunk_idx <= '0;
        pop_search_stage_snapshot <= '0;
        pop_search_stage_sector_mask <= '0;
        pop_search_stage_hit_count <= '0;
        pop_search_stage_head_valid <= 1'b0;
        pop_search_stage_head_local <= '0;
        pop_drain_partition_idx <= '0;
        pop_head_update_valid <= 1'b0;
        pop_head_update_partition_idx <= '0;
        pop_head_update_chunk_idx <= '0;
        pop_head_update_snapshot <= '0;
        pop_head_update_scan_valid <= 1'b0;
        pop_head_update_scan_partition_idx <= '0;
        pop_head_update_scan_chunk_idx <= '0;
        pop_head_update_scan_snapshot <= '0;
        pop_head_update_scan_chunk_active <= '0;
        pop_head_update_scan_part_mask <= '0;
        pop_head_update_scan_hit_count <= '0;
        pop_head_update_apply_valid <= 1'b0;
        pop_head_update_apply_partition_idx <= '0;
        pop_head_update_apply_chunk_active <= '0;
        pop_head_update_apply_part_mask <= '0;
        pop_head_update_apply_hit_count <= '0;
        pop_head_update_apply_next_chunk_valid <= 1'b0;
        pop_head_update_apply_next_chunk_idx <= '0;
        pop_head_update_apply_chunk_snapshot <= '0;
        pop_head_update_apply_head_local <= '0;
        pop_issue_inflight <= 1'b0;
        pop_emit_pending <= 1'b0;
      end else if (run_state_cmd == RUN_PREPARING) begin
        pop_engine_state <= POP_FLUSHING;
        pop_flush_req <= 1'b1;
        pop_pipeline_start <= 1'b0;
        subheader_gen_done <= 1'b0;
        for (int part_idx = 0; part_idx < PARTITION_COUNT_CONST; part_idx++) begin
          pop_partition_snapshot[part_idx] <= '0;
          pop_partition_head_local[part_idx] <= '0;
          pop_partition_chunk_active[part_idx] <= '0;
          pop_partition_hit_count[part_idx] <= '0;
          pop_partition_sector_mask[part_idx] <= '0;
        end
        pop_partition_active <= '0;
        pop_partition_head_valid <= '0;
        pop_snapshot_sector_mask <= '0;
        pop_search_unscanned_sector_mask <= '0;
        pop_search_partition_idx <= '0;
        pop_search_chunk_idx <= '0;
        pop_search_capture_done <= 1'b0;
        pop_search_capture_stage_valid <= 1'b0;
        pop_search_capture_stage_partition_idx <= '0;
        pop_search_capture_stage_chunk_idx <= '0;
        pop_search_capture_stage_snapshot <= '0;
        pop_search_chunk_stage_valid <= 1'b0;
        pop_search_stage_partition_idx <= '0;
        pop_search_stage_chunk_idx <= '0;
        pop_search_stage_snapshot <= '0;
        pop_search_stage_sector_mask <= '0;
        pop_search_stage_hit_count <= '0;
        pop_search_stage_head_valid <= 1'b0;
        pop_search_stage_head_local <= '0;
        pop_head_update_valid <= 1'b0;
        pop_head_update_partition_idx <= '0;
        pop_head_update_chunk_idx <= '0;
        pop_head_update_snapshot <= '0;
        pop_head_update_scan_valid <= 1'b0;
        pop_head_update_scan_partition_idx <= '0;
        pop_head_update_scan_chunk_idx <= '0;
        pop_head_update_scan_snapshot <= '0;
        pop_head_update_scan_chunk_active <= '0;
        pop_head_update_scan_part_mask <= '0;
        pop_head_update_scan_hit_count <= '0;
        pop_head_update_apply_valid <= 1'b0;
        pop_head_update_apply_partition_idx <= '0;
        pop_head_update_apply_chunk_active <= '0;
        pop_head_update_apply_part_mask <= '0;
        pop_head_update_apply_hit_count <= '0;
        pop_head_update_apply_next_chunk_valid <= 1'b0;
        pop_head_update_apply_next_chunk_idx <= '0;
        pop_head_update_apply_chunk_snapshot <= '0;
        pop_head_update_apply_head_local <= '0;
        if (prep_ready_latched) begin
          pop_engine_state <= POP_IDLING;
        end
      end else begin
        unique case (pop_engine_state)
          POP_IDLING: begin
            pop_pipeline_start <= 1'b0;
            subheader_gen_done <= 1'b0;
            if ((run_state_cmd == RUN_RUNNING || run_state_cmd == RUN_TERMINATING) &&
                !pop_cmd_fifo_empty) begin
              pop_current_sk <= pop_cmd_fifo_dout;
              pop_cmd_fifo_rdack <= 1'b1;
              pop_search_wait_cnt <= '0;
              pop_search_partition_idx <= '0;
              pop_search_chunk_idx <= '0;
              pop_search_capture_done <= 1'b0;
              pop_search_capture_stage_valid <= 1'b0;
              pop_search_capture_stage_partition_idx <= '0;
              pop_search_capture_stage_chunk_idx <= '0;
              pop_search_capture_stage_snapshot <= '0;
              pop_search_chunk_stage_valid <= 1'b0;
              pop_search_stage_partition_idx <= '0;
              pop_search_stage_chunk_idx <= '0;
              pop_search_stage_snapshot <= '0;
              pop_search_stage_sector_mask <= '0;
              pop_search_stage_hit_count <= '0;
              pop_search_stage_head_valid <= 1'b0;
              pop_search_stage_head_local <= '0;
              pop_drain_partition_idx <= '0;
              pop_head_update_valid <= 1'b0;
              pop_head_update_partition_idx <= '0;
              pop_head_update_chunk_idx <= '0;
              pop_head_update_snapshot <= '0;
              pop_head_update_scan_valid <= 1'b0;
              pop_head_update_scan_partition_idx <= '0;
              pop_head_update_scan_chunk_idx <= '0;
              pop_head_update_scan_snapshot <= '0;
              pop_head_update_scan_chunk_active <= '0;
              pop_head_update_scan_part_mask <= '0;
              pop_head_update_scan_hit_count <= '0;
              pop_head_update_apply_valid <= 1'b0;
              pop_head_update_apply_partition_idx <= '0;
              pop_head_update_apply_chunk_active <= '0;
              pop_head_update_apply_part_mask <= '0;
              pop_head_update_apply_hit_count <= '0;
              pop_head_update_apply_next_chunk_valid <= 1'b0;
              pop_head_update_apply_next_chunk_idx <= '0;
              pop_head_update_apply_chunk_snapshot <= '0;
              pop_head_update_apply_head_local <= '0;
              for (int part_idx = 0; part_idx < PARTITION_COUNT_CONST; part_idx++) begin
                pop_partition_snapshot[part_idx] <= '0;
                pop_partition_head_local[part_idx] <= '0;
                pop_partition_chunk_active[part_idx] <= '0;
                pop_partition_hit_count[part_idx] <= '0;
                pop_partition_sector_mask[part_idx] <= '0;
              end
              pop_partition_active <= '0;
              pop_partition_head_valid <= '0;
              pop_snapshot_sector_mask <= '0;
              pop_search_unscanned_sector_mask <= {LOCK_SECTOR_COUNT_CONST{1'b1}};
              pop_total_hits <= '0;
              pop_hits_count <= '0;
              pop_engine_state <= POP_SEARCHING;
            end
          end
          POP_SEARCHING: begin
            if (pop_search_wait_cnt < 3'd6) begin
              pop_search_wait_cnt <= pop_search_wait_cnt + 3'd1;
            end else begin
              logic consume_stage_tmp;
              logic promote_stage_tmp;
              logic capture_chunk_tmp;
              logic final_capture_tmp;
              int unsigned consume_part_idx_tmp;
              int unsigned consume_chunk_idx_tmp;
              int unsigned consume_chunk_base_tmp;
              int unsigned promote_part_idx_tmp;
              int unsigned promote_chunk_idx_tmp;
              int unsigned promote_chunk_base_tmp;
              int unsigned capture_part_idx_tmp;
              int unsigned capture_chunk_idx_tmp;
              int promote_head_local_tmp;
              logic [PARTITION_SIZE_CONST-1:0] part_snapshot_next_tmp;
              logic [SEARCH_CHUNK_WIDTH_CONST-1:0] promote_snap_tmp;
              logic [SEARCH_CHUNK_WIDTH_CONST-1:0] capture_snap_tmp;
              logic [LOCK_SECTOR_COUNT_CONST-1:0] part_mask_tmp;
              logic [LOCK_SECTOR_COUNT_CONST-1:0] sector_mask_next_tmp;
              logic [LOCK_SECTOR_COUNT_CONST-1:0] unscanned_mask_next_tmp;
              logic [PARTITION_COUNT_CONST-1:0] active_next_tmp;
              logic [PARTITION_COUNT_CONST-1:0] head_valid_next_tmp;
              logic [SEARCH_CHUNK_COUNT_CONST-1:0] chunk_active_next_tmp;
              logic [15:0] hit_total_next_tmp;

              consume_stage_tmp = pop_search_chunk_stage_valid;
              promote_stage_tmp = pop_search_capture_stage_valid;
              capture_chunk_tmp = !pop_search_capture_done;
              active_next_tmp = pop_partition_active;
              head_valid_next_tmp = pop_partition_head_valid;
              sector_mask_next_tmp = pop_snapshot_sector_mask;
              unscanned_mask_next_tmp = pop_search_unscanned_sector_mask;
              hit_total_next_tmp = pop_total_hits;

              if (consume_stage_tmp) begin
                consume_part_idx_tmp = pop_search_stage_partition_idx;
                consume_chunk_idx_tmp = pop_search_stage_chunk_idx;
                consume_chunk_base_tmp = consume_chunk_idx_tmp * SEARCH_CHUNK_WIDTH_CONST;
                part_mask_tmp =
                  pop_partition_sector_mask[consume_part_idx_tmp] |
                  pop_search_stage_sector_mask;
                part_snapshot_next_tmp = pop_partition_snapshot[consume_part_idx_tmp];
                chunk_active_next_tmp =
                  pop_partition_chunk_active[consume_part_idx_tmp];
                chunk_active_next_tmp[consume_chunk_idx_tmp] =
                  |pop_search_stage_snapshot;
                sector_mask_next_tmp =
                  pop_snapshot_sector_mask | pop_search_stage_sector_mask;
                unscanned_mask_next_tmp =
                  pop_search_unscanned_sector_mask &
                  ~search_chunk_complete_sector_mask(
                    consume_part_idx_tmp,
                    consume_chunk_idx_tmp
                  );
                hit_total_next_tmp = pop_total_hits + pop_search_stage_hit_count;

                for (int chunk_bit = 0; chunk_bit < SEARCH_CHUNK_WIDTH_CONST; chunk_bit++) begin
                  if ((consume_chunk_base_tmp + chunk_bit) < PARTITION_SIZE_CONST) begin
                    part_snapshot_next_tmp[consume_chunk_base_tmp + chunk_bit] =
                      pop_search_stage_snapshot[chunk_bit];
                  end
                end
                pop_partition_snapshot[consume_part_idx_tmp] <= part_snapshot_next_tmp;
                if (!pop_partition_head_valid[consume_part_idx_tmp] &&
                    pop_search_stage_head_valid) begin
                  head_valid_next_tmp[consume_part_idx_tmp] = 1'b1;
                  pop_partition_head_local[consume_part_idx_tmp] <=
                    pop_search_stage_head_local;
                end
                pop_partition_hit_count[consume_part_idx_tmp] <=
                  pop_partition_hit_count[consume_part_idx_tmp] + pop_search_stage_hit_count;
                pop_partition_sector_mask[consume_part_idx_tmp] <= part_mask_tmp;
                pop_partition_chunk_active[consume_part_idx_tmp] <=
                  chunk_active_next_tmp;
                active_next_tmp[consume_part_idx_tmp] =
                  pop_partition_active[consume_part_idx_tmp] || (|pop_search_stage_snapshot);
                pop_partition_active <= active_next_tmp;
                pop_partition_head_valid <= head_valid_next_tmp;
                pop_snapshot_sector_mask <= sector_mask_next_tmp;
                pop_search_unscanned_sector_mask <= unscanned_mask_next_tmp;
                pop_total_hits <= hit_total_next_tmp;
                pop_hits_count <= hit_total_next_tmp;
                dbg_pop_partition_eval_stage0_valid <= 4'(1 << consume_part_idx_tmp);
              end

              if (promote_stage_tmp) begin
                promote_part_idx_tmp = pop_search_capture_stage_partition_idx;
                promote_chunk_idx_tmp = pop_search_capture_stage_chunk_idx;
                promote_chunk_base_tmp =
                  promote_chunk_idx_tmp * SEARCH_CHUNK_WIDTH_CONST;
                promote_snap_tmp = pop_search_capture_stage_snapshot;
                promote_head_local_tmp =
                  find_next_search_chunk_local(promote_snap_tmp);

                pop_search_stage_partition_idx <=
                  PARTITION_IDX_W_CONST'(promote_part_idx_tmp);
                pop_search_stage_chunk_idx <=
                  SEARCH_CHUNK_IDX_W_CONST'(promote_chunk_idx_tmp);
                pop_search_stage_snapshot <= promote_snap_tmp;
                pop_search_stage_sector_mask <=
                  search_chunk_sector_mask(
                    promote_part_idx_tmp,
                    promote_chunk_idx_tmp,
                    promote_snap_tmp
                  );
                pop_search_stage_hit_count <= count_search_chunk(promote_snap_tmp);
                pop_search_stage_head_valid <= (promote_head_local_tmp >= 0);
                if (promote_head_local_tmp >= 0) begin
                  pop_search_stage_head_local <=
                    PARTITION_ADDR_W_CONST'(
                      promote_chunk_base_tmp + promote_head_local_tmp
                    );
                end else begin
                  pop_search_stage_head_local <= '0;
                end
                pop_search_chunk_stage_valid <= 1'b1;
              end else if (consume_stage_tmp) begin
                pop_search_chunk_stage_valid <= 1'b0;
              end

              if (capture_chunk_tmp) begin
                capture_part_idx_tmp = pop_search_partition_idx;
                capture_chunk_idx_tmp = pop_search_chunk_idx;
                capture_snap_tmp = build_search_chunk_snapshot(
                  capture_part_idx_tmp,
                  capture_chunk_idx_tmp
                );
                final_capture_tmp =
                  (capture_chunk_idx_tmp == SEARCH_CHUNK_COUNT_CONST - 1) &&
                  (capture_part_idx_tmp == PARTITION_COUNT_CONST - 1);

                pop_search_capture_stage_partition_idx <=
                  PARTITION_IDX_W_CONST'(capture_part_idx_tmp);
                pop_search_capture_stage_chunk_idx <=
                  SEARCH_CHUNK_IDX_W_CONST'(capture_chunk_idx_tmp);
                pop_search_capture_stage_snapshot <= capture_snap_tmp;
                pop_search_capture_stage_valid <= 1'b1;

                if (final_capture_tmp) begin
                  pop_search_capture_done <= 1'b1;
                end else if (capture_chunk_idx_tmp == SEARCH_CHUNK_COUNT_CONST - 1) begin
                  pop_search_chunk_idx <= '0;
                  pop_search_partition_idx <=
                    PARTITION_IDX_W_CONST'(capture_part_idx_tmp + 1);
                end else begin
                  pop_search_chunk_idx <=
                    SEARCH_CHUNK_IDX_W_CONST'(capture_chunk_idx_tmp + 1);
                end
              end else if (promote_stage_tmp) begin
                pop_search_capture_stage_valid <= 1'b0;
              end

              if (consume_stage_tmp && pop_search_capture_done && !promote_stage_tmp) begin
                pop_drain_partition_idx <= '0;
                dbg_pop_partition_pending <= 4'(active_next_tmp);
                pop_search_chunk_stage_valid <= 1'b0;
                pop_engine_state <= POP_LOADING;
              end
            end
          end
          POP_LOADING: begin
            logic [3:0] has_more_tmp;

            has_more_tmp = '0;
            for (int part_idx = 0; part_idx < PARTITION_COUNT_CONST; part_idx++) begin
              if (part_idx < 4 && pop_partition_hit_count[part_idx] > 16'd1) begin
                has_more_tmp[part_idx] = 1'b1;
              end
            end
            dbg_pop_partition_load <= 4'(pop_partition_active);
            dbg_pop_partition_result_valid <=
              (pop_total_hits > 16'd1) ? 4'({PARTITION_COUNT_CONST{1'b1}}) :
              4'(pop_partition_active);
            dbg_pop_partition_flag <= 4'(pop_partition_active);
            dbg_pop_partition_has_more <= has_more_tmp;
            pop_engine_state <= POP_COUNTING;
          end
          POP_COUNTING: begin
            dbg_pop_count_partition_idx <= dbg_pop_count_partition_idx + 2'd1;
            if (pop_total_hits == 0) begin
              aso_hit_type2_valid <= 1'b1;
              aso_hit_type2_startofpacket <= 1'b1;
              aso_hit_type2_endofpacket <= 1'b1;
              aso_hit_type2_channel <= INTERLEAVING_INDEX[3:0];
              aso_hit_type2_data <= make_subheader(pop_current_sk[7:0], 8'd0);
              pop_engine_state <= POP_RESETTING;
            end else begin
              pop_pipeline_start <= 1'b1;
              subheader_gen_done <= 1'b0;
              pop_engine_state <= POP_DRAINING;
            end
          end
          POP_DRAINING: begin
            pop_pipeline_start <= 1'b1;
            if (pop_erase_grant) begin
              dbg_pop_issue_partition_idx <=
                2'(pop_issue_addr[ADDR_W_CONST-1:0] / PARTITION_SIZE_CONST);
            end

            if (pop_head_update_valid) begin
              int update_part_tmp;
              logic [SEARCH_CHUNK_COUNT_CONST-1:0] chunk_active_next_tmp;
              logic [SEARCH_CHUNK_WIDTH_CONST-1:0] chunk_snapshot_next_tmp;
              logic [LOCK_SECTOR_COUNT_CONST-1:0] part_mask_next_tmp;
              int head_local_next_tmp;

              update_part_tmp = pop_head_update_partition_idx;
              chunk_active_next_tmp =
                pop_partition_chunk_active[update_part_tmp];
              chunk_snapshot_next_tmp = partition_chunk_snapshot(
                pop_head_update_snapshot,
                pop_head_update_chunk_idx
              );
              chunk_active_next_tmp[pop_head_update_chunk_idx] =
                |chunk_snapshot_next_tmp;
              part_mask_next_tmp =
                partition_sector_mask(update_part_tmp, pop_head_update_snapshot);

              if (pop_partition_hit_count[update_part_tmp] == 16'd0) begin
                chunk_active_next_tmp = '0;
                pop_head_update_apply_valid <= 1'b1;
                pop_head_update_apply_partition_idx <=
                  PARTITION_IDX_W_CONST'(update_part_tmp);
                pop_head_update_apply_chunk_active <= chunk_active_next_tmp;
                pop_head_update_apply_part_mask <= part_mask_next_tmp;
                pop_head_update_apply_hit_count <=
                  pop_partition_hit_count[update_part_tmp];
                pop_head_update_apply_next_chunk_valid <= 1'b0;
                pop_head_update_apply_next_chunk_idx <= '0;
                pop_head_update_apply_chunk_snapshot <= '0;
                pop_head_update_apply_head_local <= '0;
              end else begin
                if (|chunk_snapshot_next_tmp) begin
                  pop_head_update_apply_valid <= 1'b1;
                  pop_head_update_apply_partition_idx <=
                    PARTITION_IDX_W_CONST'(update_part_tmp);
                  pop_head_update_apply_chunk_active <= chunk_active_next_tmp;
                  pop_head_update_apply_part_mask <= part_mask_next_tmp;
                  pop_head_update_apply_hit_count <=
                    pop_partition_hit_count[update_part_tmp];
                  pop_head_update_apply_next_chunk_valid <= 1'b1;
                  pop_head_update_apply_next_chunk_idx <=
                    SEARCH_CHUNK_IDX_W_CONST'(pop_head_update_chunk_idx);
                  pop_head_update_apply_chunk_snapshot <= chunk_snapshot_next_tmp;
                  head_local_next_tmp =
                    find_next_search_chunk_local(chunk_snapshot_next_tmp);
                  if (head_local_next_tmp >= 0) begin
                    pop_head_update_apply_head_local <=
                      PARTITION_ADDR_W_CONST'(
                        pop_head_update_chunk_idx * SEARCH_CHUNK_WIDTH_CONST +
                        head_local_next_tmp
                      );
                  end else begin
                    pop_head_update_apply_head_local <= '0;
                  end
                end else begin
                  if (pop_head_update_chunk_idx == SEARCH_CHUNK_COUNT_CONST - 1) begin
                    pop_head_update_apply_valid <= 1'b1;
                    pop_head_update_apply_partition_idx <=
                      PARTITION_IDX_W_CONST'(update_part_tmp);
                    pop_head_update_apply_chunk_active <= chunk_active_next_tmp;
                    pop_head_update_apply_part_mask <= part_mask_next_tmp;
                    pop_head_update_apply_hit_count <=
                      pop_partition_hit_count[update_part_tmp];
                    pop_head_update_apply_next_chunk_valid <= 1'b0;
                    pop_head_update_apply_next_chunk_idx <= '0;
                    pop_head_update_apply_chunk_snapshot <= '0;
                    pop_head_update_apply_head_local <= '0;
                  end else begin
                    pop_head_update_scan_valid <= 1'b1;
                    pop_head_update_scan_partition_idx <=
                      PARTITION_IDX_W_CONST'(update_part_tmp);
                    pop_head_update_scan_chunk_idx <=
                      SEARCH_CHUNK_IDX_W_CONST'(pop_head_update_chunk_idx + 1);
                    pop_head_update_scan_snapshot <= pop_head_update_snapshot;
                    pop_head_update_scan_chunk_active <= chunk_active_next_tmp;
                    pop_head_update_scan_part_mask <= part_mask_next_tmp;
                    pop_head_update_scan_hit_count <=
                      pop_partition_hit_count[update_part_tmp];
                  end
                end
              end

              pop_head_update_valid <= 1'b0;
            end

            if (pop_head_update_scan_valid) begin
              logic [SEARCH_CHUNK_WIDTH_CONST-1:0] chunk_snapshot_next_tmp;
              int head_local_next_tmp;

              chunk_snapshot_next_tmp = partition_chunk_snapshot(
                pop_head_update_scan_snapshot,
                pop_head_update_scan_chunk_idx
              );

              if (|chunk_snapshot_next_tmp ||
                  pop_head_update_scan_chunk_idx == SEARCH_CHUNK_COUNT_CONST - 1) begin
                pop_head_update_apply_valid <= 1'b1;
                pop_head_update_apply_partition_idx <=
                  pop_head_update_scan_partition_idx;
                pop_head_update_apply_chunk_active <=
                  pop_head_update_scan_chunk_active;
                pop_head_update_apply_part_mask <=
                  pop_head_update_scan_part_mask;
                pop_head_update_apply_hit_count <=
                  pop_head_update_scan_hit_count;
                pop_head_update_apply_next_chunk_valid <= |chunk_snapshot_next_tmp;
                if (|chunk_snapshot_next_tmp) begin
                  pop_head_update_apply_next_chunk_idx <=
                    pop_head_update_scan_chunk_idx;
                  head_local_next_tmp =
                    find_next_search_chunk_local(chunk_snapshot_next_tmp);
                  if (head_local_next_tmp >= 0) begin
                    pop_head_update_apply_head_local <=
                      PARTITION_ADDR_W_CONST'(
                        pop_head_update_scan_chunk_idx * SEARCH_CHUNK_WIDTH_CONST +
                        head_local_next_tmp
                      );
                  end else begin
                    pop_head_update_apply_head_local <= '0;
                  end
                end else begin
                  pop_head_update_apply_next_chunk_idx <= '0;
                  pop_head_update_apply_head_local <= '0;
                end
                pop_head_update_apply_chunk_snapshot <= chunk_snapshot_next_tmp;
                pop_head_update_scan_valid <= 1'b0;
              end else begin
                pop_head_update_scan_chunk_idx <=
                  SEARCH_CHUNK_IDX_W_CONST'(pop_head_update_scan_chunk_idx + 1);
              end
            end

            if (pop_head_update_apply_valid) begin
              int update_part_tmp;
              logic [LOCK_SECTOR_COUNT_CONST-1:0] sector_mask_next_tmp;

              update_part_tmp = pop_head_update_apply_partition_idx;
              sector_mask_next_tmp = '0;
              if (pop_head_update_apply_hit_count == 16'd0 ||
                  !pop_head_update_apply_next_chunk_valid) begin
                pop_partition_active[update_part_tmp] <= 1'b0;
                pop_partition_head_valid[update_part_tmp] <= 1'b0;
                pop_partition_head_local[update_part_tmp] <= '0;
              end else begin
                pop_partition_active[update_part_tmp] <= 1'b1;
                pop_partition_head_valid[update_part_tmp] <= 1'b1;
                pop_partition_head_local[update_part_tmp] <=
                  pop_head_update_apply_head_local;
              end

              for (int part_idx = 0; part_idx < PARTITION_COUNT_CONST; part_idx++) begin
                if (part_idx == update_part_tmp) begin
                  sector_mask_next_tmp |= pop_head_update_apply_part_mask;
                end else begin
                  sector_mask_next_tmp |= pop_partition_sector_mask[part_idx];
                end
              end

              pop_partition_chunk_active[update_part_tmp] <=
                pop_head_update_apply_chunk_active;
              pop_partition_sector_mask[update_part_tmp] <=
                pop_head_update_apply_part_mask;
              pop_snapshot_sector_mask <= sector_mask_next_tmp;
              pop_head_update_apply_valid <= 1'b0;
            end

            if (!subheader_gen_done) begin
              aso_hit_type2_valid <= 1'b1;
              aso_hit_type2_startofpacket <= 1'b1;
              aso_hit_type2_channel <= INTERLEAVING_INDEX[3:0];
              aso_hit_type2_data <= make_subheader(pop_current_sk[7:0], pop_total_hits[7:0]);
              subheader_gen_done <= 1'b1;
            end else if (pop_emit_pending) begin
              aso_hit_type2_valid <= 1'b1;
              aso_hit_type2_channel <= INTERLEAVING_INDEX[3:0];
              aso_hit_type2_data <= make_hit_type2(pop_hit_pending);
              aso_hit_type2_error[0] <= (pop_hit_pending_epoch[8] != pop_current_sk[8]);
              aso_hit_type2_metadata <=
                DEBUG_METADATA_EN_CONST ? pop_metadata_pending : 64'd0;
              aso_hit_type2_metadata_valid <=
                DEBUG_METADATA_EN_CONST && pop_metadata_valid_pending;
              if (!pop_occupied_pending) begin
                pop_cache_miss_pulse <= 1'b1;
              end
              if (pop_hits_count == 16'd0) begin
                aso_hit_type2_endofpacket <= 1'b1;
                pop_last_hit_pending <= 1'b1;
                pop_engine_state <= POP_RESETTING;
              end
              pop_emit_pending <= 1'b0;
            end else if (pop_output_pending) begin
              pop_hit_valid <= 1'b1;
              pop_issue_inflight <= 1'b0;
              pop_emit_pending <= 1'b1;
            end else if (pop_issue_inflight ||
                         pop_head_update_valid ||
                         pop_head_update_scan_valid ||
                         pop_head_update_apply_valid) begin
              pop_engine_state <= POP_DRAINING;
            end else begin
              int next_part;
              int next_local;
              logic [PARTITION_SIZE_CONST-1:0] part_next_tmp;
              logic [ADDR_W_CONST-1:0] issue_addr_tmp;

              next_part = -1;
              next_local = -1;
              for (int part_idx = 0; part_idx < PARTITION_COUNT_CONST; part_idx++) begin
                if (next_part < 0 &&
                    pop_partition_active[part_idx] &&
                    pop_partition_head_valid[part_idx]) begin
                  next_part = part_idx;
                  next_local = pop_partition_head_local[part_idx];
                end
              end

              if (next_part >= 0 && next_local >= 0) begin
                issue_addr_tmp = partition_global_addr(next_part, next_local);
                part_next_tmp = pop_partition_snapshot[next_part];
                part_next_tmp[next_local] = 1'b0;

                pop_issue_addr <= 16'(issue_addr_tmp);
                pop_partition_snapshot[next_part] <= part_next_tmp;
                pop_head_update_valid <= 1'b1;
                pop_head_update_partition_idx <= PARTITION_IDX_W_CONST'(next_part);
                pop_head_update_chunk_idx <=
                  SEARCH_CHUNK_IDX_W_CONST'(
                    next_local / SEARCH_CHUNK_WIDTH_CONST
                  );
                pop_head_update_snapshot <= part_next_tmp;
                if (pop_partition_hit_count[next_part] != 0) begin
                  pop_partition_hit_count[next_part] <=
                    pop_partition_hit_count[next_part] - 16'd1;
                end
                if (pop_hits_count != 0) begin
                  pop_hits_count <= pop_hits_count - 16'd1;
                end
                pop_erase_req <= 1'b1;
                pop_issue_inflight <= 1'b1;
                dbg_pop_rr_idx <= dbg_pop_rr_idx + 2'd1;
                dbg_pop_issue_partition_idx <= 2'(next_part);
                dbg_pop_partition_advance <= 4'(1 << next_part);
                pop_drain_partition_idx <=
                  (next_part == PARTITION_COUNT_CONST - 1) ?
                  '0 : PARTITION_IDX_W_CONST'(next_part + 1);
              end else if (!pop_issue_inflight) begin
                pop_engine_state <= POP_RESETTING;
              end
            end
          end
          POP_RESETTING: begin
            pop_pipeline_start <= 1'b0;
            subheader_gen_done <= 1'b0;
            for (int part_idx = 0; part_idx < PARTITION_COUNT_CONST; part_idx++) begin
              pop_partition_snapshot[part_idx] <= '0;
              pop_partition_head_local[part_idx] <= '0;
              pop_partition_chunk_active[part_idx] <= '0;
              pop_partition_hit_count[part_idx] <= '0;
              pop_partition_sector_mask[part_idx] <= '0;
            end
            pop_partition_active <= '0;
            pop_partition_head_valid <= '0;
            pop_snapshot_sector_mask <= '0;
            pop_issue_inflight <= 1'b0;
            pop_emit_pending <= 1'b0;
            pop_hits_count <= '0;
            pop_total_hits <= '0;
            pop_search_capture_done <= 1'b0;
            pop_search_partition_idx <= '0;
            pop_search_chunk_idx <= '0;
            pop_search_capture_stage_valid <= 1'b0;
            pop_search_capture_stage_partition_idx <= '0;
            pop_search_capture_stage_chunk_idx <= '0;
            pop_search_capture_stage_snapshot <= '0;
            pop_search_chunk_stage_valid <= 1'b0;
            pop_search_stage_partition_idx <= '0;
            pop_search_stage_chunk_idx <= '0;
            pop_search_stage_snapshot <= '0;
            pop_search_stage_sector_mask <= '0;
            pop_search_stage_hit_count <= '0;
            pop_search_stage_head_valid <= 1'b0;
            pop_search_stage_head_local <= '0;
            pop_drain_partition_idx <= '0;
            pop_head_update_valid <= 1'b0;
            pop_head_update_partition_idx <= '0;
            pop_head_update_chunk_idx <= '0;
            pop_head_update_snapshot <= '0;
            pop_head_update_scan_valid <= 1'b0;
            pop_head_update_scan_partition_idx <= '0;
            pop_head_update_scan_chunk_idx <= '0;
            pop_head_update_scan_snapshot <= '0;
            pop_head_update_scan_chunk_active <= '0;
            pop_head_update_scan_part_mask <= '0;
            pop_head_update_scan_hit_count <= '0;
            pop_head_update_apply_valid <= 1'b0;
            pop_head_update_apply_partition_idx <= '0;
            pop_head_update_apply_chunk_active <= '0;
            pop_head_update_apply_part_mask <= '0;
            pop_head_update_apply_hit_count <= '0;
            pop_head_update_apply_next_chunk_valid <= 1'b0;
            pop_head_update_apply_next_chunk_idx <= '0;
            pop_head_update_apply_chunk_snapshot <= '0;
            pop_head_update_apply_head_local <= '0;
            dbg_pop_partition_pending <= '0;
            dbg_pop_partition_result_valid <= '0;
            dbg_pop_partition_flag <= '0;
            dbg_pop_partition_has_more <= '0;
            pop_engine_state <= POP_IDLING;
          end
          default: begin
            pop_engine_state <= POP_IDLING;
          end
        endcase
      end
    end
  end

  assign decision_reg = (i_rst || csr_soft_reset_pulse) ? 3'd4 : decision;

  always_comb begin
    terminating_drain_done =
      run_state_cmd == RUN_TERMINATING &&
      deassembly_fifo_empty &&
      pop_cmd_fifo_empty &&
      pop_engine_state == POP_IDLING &&
      debug_msg2.push_cnt == (debug_msg2.pop_cnt + debug_msg2.overwrite_cnt);
  end

  logic [3:0] dbg_run_state_code;
  logic [2:0] dbg_pop_engine_state_code;
  logic dbg_push_state_code;
  assign dbg_run_state_code = run_state_cmd;
  assign dbg_pop_engine_state_code =
    pop_search_collecting ? POP_LOADING : pop_engine_state;
  assign dbg_push_state_code = (push_state == PUSH_ERASING);

endmodule

module ring_buffer_cam_side_ram #(
  parameter int DATA_WIDTH = 40,
  parameter int ADDR_WIDTH = 9
) (
  input  logic                  clk,
  input  logic [ADDR_WIDTH-1:0] raddr,
  input  logic [ADDR_WIDTH-1:0] waddr,
  input  logic [ADDR_WIDTH-1:0] dbg_patch_addr,
  input  logic [DATA_WIDTH-1:0] data,
  input  logic [DATA_WIDTH-1:0] dbg_patch_data,
  input  logic                  dbg_patch_we,
  input  logic                  we,
  output logic [DATA_WIDTH-1:0] q
);
  (* ramstyle = "M10K", noprune, preserve *) logic [DATA_WIDTH-1:0]
    ram [0:(1 << ADDR_WIDTH)-1];

  always_ff @(posedge clk) begin
    if (dbg_patch_we) begin
      ram[dbg_patch_addr] <= dbg_patch_data;
    end else if (we) begin
      ram[waddr] <= data;
    end
    q <= ram[raddr];
  end
endmodule

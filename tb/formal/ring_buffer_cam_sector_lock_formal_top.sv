// File name: ring_buffer_cam_sector_lock_formal_top.sv
// Purpose  : Formal wrapper that exposes rbCAM sector-lock, accounting, and metadata assertions.

module ring_buffer_cam_sector_lock_formal_top #(
  parameter int RING_BUFFER_N_ENTRY = 512,
  parameter int INTERLEAVING_FACTOR = 4,
  parameter int INTERLEAVING_INDEX  = 0,
  parameter int N_PARTITIONS        = 2,
  parameter int ENCODER_LEAF_WIDTH  = 16,
  parameter int ENCODER_PIPE_STAGES = 4,
  parameter int DEBUG               = 1
) (
  output logic [31:0] avs_csr_readdata,
  input  logic        avs_csr_read,
  input  logic [4:0]  avs_csr_address,
  output logic        avs_csr_waitrequest,
  input  logic        avs_csr_write,
  input  logic [31:0] avs_csr_writedata,
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
  input  logic        i_clk,
  input  logic        i_rst,
  input  logic [8:0]  asi_ctrl_data,
  input  logic        asi_ctrl_valid,
  output logic        asi_ctrl_ready,
  output logic [15:0] aso_filllevel_data,
  output logic        aso_filllevel_valid,
  output logic [31:0] coe_debug_fill_level,
  output logic [31:0] coe_debug_fifo_level,
  output logic [31:0] coe_debug_queue_state
);

  localparam int ADDR_W_CONST            = (RING_BUFFER_N_ENTRY <= 2) ? 1 : $clog2(RING_BUFFER_N_ENTRY);
  localparam int LOCK_SECTOR_COUNT_CONST = 8;
  localparam int LOCK_SECTOR_BITS_CONST  = 3;
  localparam int ADDR_SECTOR_LSB_CONST =
    (ADDR_W_CONST > LOCK_SECTOR_BITS_CONST) ?
    (ADDR_W_CONST - LOCK_SECTOR_BITS_CONST) : 0;

  ring_buffer_cam #(
    .RING_BUFFER_N_ENTRY         (RING_BUFFER_N_ENTRY),
    .INTERLEAVING_FACTOR         (INTERLEAVING_FACTOR),
    .INTERLEAVING_INDEX          (INTERLEAVING_INDEX),
    .N_PARTITIONS                (N_PARTITIONS),
    .ENCODER_LEAF_WIDTH          (ENCODER_LEAF_WIDTH),
    .ENCODER_PIPE_STAGES         (ENCODER_PIPE_STAGES),
    .DEBUG                       (DEBUG)
  ) dut (
    .avs_csr_readdata            (avs_csr_readdata),
    .avs_csr_read                (avs_csr_read),
    .avs_csr_address             (avs_csr_address),
    .avs_csr_waitrequest         (avs_csr_waitrequest),
    .avs_csr_write               (avs_csr_write),
    .avs_csr_writedata           (avs_csr_writedata),
    .asi_hit_type1_channel       (asi_hit_type1_channel),
    .asi_hit_type1_startofpacket (asi_hit_type1_startofpacket),
    .asi_hit_type1_endofpacket   (asi_hit_type1_endofpacket),
    .asi_hit_type1_empty         (asi_hit_type1_empty),
    .asi_hit_type1_data          (asi_hit_type1_data),
    .asi_hit_type1_valid         (asi_hit_type1_valid),
    .asi_hit_type1_ready         (asi_hit_type1_ready),
    .asi_hit_type1_error         (asi_hit_type1_error),
    .asi_hit_type1_metadata      (asi_hit_type1_metadata),
    .asi_hit_type1_metadata_valid(asi_hit_type1_metadata_valid),
    .aso_hit_type2_channel       (aso_hit_type2_channel),
    .aso_hit_type2_startofpacket (aso_hit_type2_startofpacket),
    .aso_hit_type2_endofpacket   (aso_hit_type2_endofpacket),
    .aso_hit_type2_data          (aso_hit_type2_data),
    .aso_hit_type2_valid         (aso_hit_type2_valid),
    .aso_hit_type2_ready         (aso_hit_type2_ready),
    .aso_hit_type2_error         (aso_hit_type2_error),
    .aso_hit_type2_metadata      (aso_hit_type2_metadata),
    .aso_hit_type2_metadata_valid(aso_hit_type2_metadata_valid),
    .i_clk                       (i_clk),
    .i_rst                       (i_rst),
    .asi_ctrl_data               (asi_ctrl_data),
    .asi_ctrl_valid              (asi_ctrl_valid),
    .asi_ctrl_ready              (asi_ctrl_ready),
    .aso_filllevel_data          (aso_filllevel_data),
    .aso_filllevel_valid         (aso_filllevel_valid),
    .coe_debug_fill_level        (coe_debug_fill_level),
    .coe_debug_fifo_level        (coe_debug_fifo_level),
    .coe_debug_queue_state       (coe_debug_queue_state)
  );

  ring_buffer_cam_sector_lock_sva #(
    .RING_BUFFER_N_ENTRY     (RING_BUFFER_N_ENTRY),
    .ADDR_W_CONST            (ADDR_W_CONST),
    .LOCK_SECTOR_COUNT_CONST (LOCK_SECTOR_COUNT_CONST),
    .LOCK_SECTOR_BITS_CONST  (LOCK_SECTOR_BITS_CONST),
    .ADDR_SECTOR_LSB_CONST   (ADDR_SECTOR_LSB_CONST)
  ) sector_lock_sva (
    .i_clk                    (i_clk),
    .i_rst                    (i_rst),
    .run_state_cmd            (dut.v2_core.run_state_cmd),
    .csr_soft_reset_pulse     (dut.v2_core.csr_soft_reset_pulse),
    .pop_engine_state         (dut.v2_core.pop_engine_state),
    .push_write_req           (dut.v2_core.push_write_req),
    .push_write_grant         (dut.v2_core.push_write_grant),
    .push_erase_grant         (dut.v2_core.push_erase_grant),
    .pop_erase_grant          (dut.v2_core.pop_erase_grant),
    .pop_flush_grant          (dut.v2_core.pop_flush_grant),
    .push_write_sector_locked (dut.v2_core.push_write_sector_locked),
    .push_erase_sector_locked (dut.v2_core.push_erase_sector_locked),
    .pop_flush_req            (dut.v2_core.pop_flush_req),
    .push_erase_req           (dut.v2_core.push_erase_req),
    .pop_erase_req            (dut.v2_core.pop_erase_req),
    .pop_issue_inflight       (dut.v2_core.pop_issue_inflight),
    .pop_output_pending       (dut.v2_core.pop_output_pending),
    .pop_emit_pending         (dut.v2_core.pop_emit_pending),
    .pop_snapshot             (dut.v2_core.pop_snapshot),
    .write_pointer            (dut.v2_core.write_pointer),
    .push_erase_addr_reg      (dut.v2_core.push_erase_addr_reg),
    .pop_issue_addr           (dut.v2_core.pop_issue_addr),
    .pop_issue_addr_pending   (dut.v2_core.pop_issue_addr_pending),
    .pop_search_wait_cnt      (dut.v2_core.pop_search_wait_cnt),
    .decision                 (dut.v2_core.decision),
    .decision_reg             (dut.v2_core.decision_reg),
    .pop_sector_lock_mask     (dut.v2_core.pop_sector_lock_mask)
  );

  ring_buffer_cam_accounting_sva #(
    .RING_BUFFER_N_ENTRY     (RING_BUFFER_N_ENTRY),
    .ADDR_W_CONST            (ADDR_W_CONST),
    .LOCK_SECTOR_COUNT_CONST (LOCK_SECTOR_COUNT_CONST)
  ) accounting_sva (
    .i_clk                       (i_clk),
    .i_rst                       (i_rst),
    .avs_csr_read                (avs_csr_read),
    .avs_csr_write               (avs_csr_write),
    .avs_csr_address             (avs_csr_address),
    .avs_csr_writedata           (avs_csr_writedata),
    .avs_csr_readdata            (avs_csr_readdata),
    .csr_counter_freeze          (dut.v2_core.csr_counter_freeze),
    .csr_soft_reset_pulse        (dut.v2_core.csr_soft_reset_pulse),
    .run_state_cmd               (dut.v2_core.run_state_cmd),
    .pop_engine_state            (dut.v2_core.pop_engine_state),
    .flush_cycle_count           (dut.v2_core.flush_cycle_count),
    .push_write_grant            (dut.v2_core.push_write_grant),
    .pop_erase_grant             (dut.v2_core.pop_erase_grant),
    .push_erase_grant            (dut.v2_core.push_erase_grant),
    .deasm_full_drop_event       (dut.v2_core.deasm_full_drop_event),
    .pop_cmd_full_drop_event     (dut.v2_core.pop_cmd_full_drop_event),
    .egress_not_ready_drop_event (dut.v2_core.egress_not_ready_drop_event),
    .deassembly_fifo_sclr        (dut.v2_core.deassembly_fifo_sclr),
    .pop_cmd_fifo_sclr           (dut.v2_core.pop_cmd_fifo_sclr),
    .deassembly_fifo_empty       (dut.v2_core.deassembly_fifo_empty),
    .pop_cmd_fifo_empty          (dut.v2_core.pop_cmd_fifo_empty),
    .terminating_drain_done      (dut.v2_core.terminating_drain_done),
    .slot_valid                  (dut.v2_core.slot_valid),
    .write_pointer               (dut.v2_core.write_pointer),
    .pop_sector_lock_mask        (dut.v2_core.pop_sector_lock_mask),
    .debug_msg2                  (dut.v2_core.debug_msg2),
    .debug_msg2_snap             (dut.v2_core.debug_msg2_snap)
  );

  ring_buffer_cam_metadata_sva #(
    .RING_BUFFER_N_ENTRY (RING_BUFFER_N_ENTRY),
    .ADDR_W_CONST        (ADDR_W_CONST)
  ) metadata_sva (
    .i_clk                                (i_clk),
    .i_rst                                (i_rst),
    .run_state_cmd                        (dut.v2_core.run_state_cmd),
    .csr_soft_reset_pulse                 (dut.v2_core.csr_soft_reset_pulse),
    .asi_hit_type1_metadata               (asi_hit_type1_metadata),
    .asi_hit_type1_metadata_valid         (asi_hit_type1_metadata_valid),
    .deassembly_fifo_wrreq                (dut.v2_core.deassembly_fifo_wrreq),
    .deassembly_fifo_din_metadata         (dut.v2_core.deassembly_fifo_din.metadata),
    .deassembly_fifo_din_metadata_valid   (dut.v2_core.deassembly_fifo_din.metadata_valid),
    .deassembly_fifo_dout_metadata        (dut.v2_core.deassembly_fifo_dout.metadata),
    .deassembly_fifo_dout_metadata_valid  (dut.v2_core.deassembly_fifo_dout.metadata_valid),
    .push_write_grant                     (dut.v2_core.push_write_grant),
    .write_pointer                        (dut.v2_core.write_pointer),
    .slot_valid                           (dut.v2_core.slot_valid),
    .slot_metadata                        (dut.v2_core.slot_metadata),
    .slot_metadata_valid                  (dut.v2_core.slot_metadata_valid),
    .pop_erase_grant                      (dut.v2_core.pop_erase_grant),
    .pop_issue_addr                       (dut.v2_core.pop_issue_addr),
    .pop_metadata_pending                 (dut.v2_core.pop_metadata_pending),
    .pop_metadata_valid_pending           (dut.v2_core.pop_metadata_valid_pending),
    .aso_hit_type2_valid                  (aso_hit_type2_valid),
    .aso_hit_type2_startofpacket          (aso_hit_type2_startofpacket),
    .aso_hit_type2_metadata               (aso_hit_type2_metadata),
    .aso_hit_type2_metadata_valid         (aso_hit_type2_metadata_valid)
  );

endmodule

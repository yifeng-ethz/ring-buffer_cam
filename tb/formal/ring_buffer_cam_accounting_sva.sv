// File name: ring_buffer_cam_accounting_sva.sv
// Purpose  : Formal checks for rbCAM counters, freeze, drops, and run-control cleanup.

module ring_buffer_cam_accounting_sva
  import ring_buffer_cam_sv_pkg::*;
#(
  parameter int RING_BUFFER_N_ENTRY     = 512,
  parameter int ADDR_W_CONST            = 9,
  parameter int LOCK_SECTOR_COUNT_CONST = 8
) (
  input logic                               i_clk,
  input logic                               i_rst,
  input logic                               avs_csr_read,
  input logic                               avs_csr_write,
  input logic [4:0]                         avs_csr_address,
  input logic [31:0]                        avs_csr_writedata,
  input logic [31:0]                        avs_csr_readdata,
  input logic                               csr_counter_freeze,
  input logic                               csr_soft_reset_pulse,
  input run_state_e                         run_state_cmd,
  input pop_state_e                         pop_engine_state,
  input logic [15:0]                        flush_cycle_count,
  input logic                               push_write_grant,
  input logic                               pop_erase_grant,
  input logic                               push_erase_grant,
  input logic                               deasm_full_drop_event,
  input logic                               pop_cmd_full_drop_event,
  input logic                               egress_not_ready_drop_event,
  input logic                               deassembly_fifo_sclr,
  input logic                               pop_cmd_fifo_sclr,
  input logic                               deassembly_fifo_empty,
  input logic                               pop_cmd_fifo_empty,
  input logic                               terminating_drain_done,
  input logic [RING_BUFFER_N_ENTRY-1:0]     slot_valid,
  input logic [ADDR_W_CONST-1:0]            write_pointer,
  input logic [LOCK_SECTOR_COUNT_CONST-1:0] pop_sector_lock_mask,
  input debug_msg_t                         debug_msg2,
  input debug_msg_t                         debug_msg2_snap
);

  default clocking cb @(posedge i_clk);
  endclocking

  default disable iff (i_rst);

  logic counter_clear;
  assign counter_clear =
    csr_soft_reset_pulse ||
    (run_state_cmd == RUN_PREPARING && flush_cycle_count == 16'd0);

  function automatic logic [31:0] counter_word_for_addr(
    input debug_msg_t msg,
    input logic [4:0] addr
  );
    unique case (addr)
      5'd5:  return msg.inerr_cnt[31:0];
      5'd6:  return msg.push_cnt[31:0];
      5'd7:  return msg.pop_cnt[31:0];
      5'd8:  return msg.overwrite_cnt[31:0];
      5'd9:  return msg.cache_miss_cnt[31:0];
      5'd10: return msg.inerr_cnt[63:32];
      5'd11: return msg.push_cnt[63:32];
      5'd12: return msg.pop_cnt[63:32];
      5'd13: return msg.overwrite_cnt[63:32];
      5'd14: return msg.cache_miss_cnt[63:32];
      5'd15: return msg.deasm_full_drop_cnt[31:0];
      5'd16: return msg.deasm_full_drop_cnt[63:32];
      5'd17: return msg.pop_cmd_full_drop_cnt[31:0];
      5'd18: return msg.pop_cmd_full_drop_cnt[63:32];
      5'd19: return msg.egress_not_ready_drop_cnt[31:0];
      5'd20: return msg.egress_not_ready_drop_cnt[63:32];
      default: return '0;
    endcase
  endfunction

  function automatic logic counters_zero(input debug_msg_t msg);
    return
      msg.inerr_cnt == 64'd0 &&
      msg.push_cnt == 64'd0 &&
      msg.pop_cnt == 64'd0 &&
      msg.overwrite_cnt == 64'd0 &&
      msg.cache_miss_cnt == 64'd0 &&
      msg.deasm_full_drop_cnt == 64'd0 &&
      msg.pop_cmd_full_drop_cnt == 64'd0 &&
      msg.egress_not_ready_drop_cnt == 64'd0 &&
      msg.cam_clean == 1'b1;
  endfunction

  ap_push_count_binds_to_push_write_grant:
    assert property (
      !counter_clear |=>
      debug_msg2.push_cnt ==
      $past(debug_msg2.push_cnt) + ($past(push_write_grant) ? 64'd1 : 64'd0)
    );

  ap_pop_count_binds_to_pop_erase_grant:
    assert property (
      !counter_clear |=>
      debug_msg2.pop_cnt ==
      $past(debug_msg2.pop_cnt) + ($past(pop_erase_grant) ? 64'd1 : 64'd0)
    );

  ap_overwrite_count_binds_to_push_erase_grant:
    assert property (
      !counter_clear |=>
      debug_msg2.overwrite_cnt ==
      $past(debug_msg2.overwrite_cnt) + ($past(push_erase_grant) ? 64'd1 : 64'd0)
    );

  ap_freeze_rising_edge_snapshots_live_counters:
    assert property (
      !csr_counter_freeze &&
      !csr_soft_reset_pulse &&
      avs_csr_write &&
      avs_csr_address == 5'd2 &&
      avs_csr_writedata[5] &&
      !avs_csr_writedata[1] |=>
      csr_counter_freeze &&
      debug_msg2_snap == $past(debug_msg2)
    );

  ap_frozen_csr_counter_reads_use_snapshot:
    assert property (
      csr_counter_freeze &&
      avs_csr_read &&
      avs_csr_address inside {[5'd5:5'd20]} |=>
      avs_csr_readdata == counter_word_for_addr($past(debug_msg2_snap), $past(avs_csr_address))
    );

  ap_live_csr_counter_reads_use_live_previous_cycle:
    assert property (
      !csr_counter_freeze &&
      avs_csr_read &&
      avs_csr_address inside {[5'd5:5'd20]} |=>
      avs_csr_readdata == counter_word_for_addr($past(debug_msg2), $past(avs_csr_address))
    );

  ap_soft_reset_clears_live_and_snapshot_counters:
    assert property (
      csr_soft_reset_pulse |=>
      counters_zero(debug_msg2) &&
      counters_zero(debug_msg2_snap) &&
      !csr_counter_freeze
    );

  ap_deasm_full_drop_counter_binding:
    assert property (
      !counter_clear |=>
      debug_msg2.deasm_full_drop_cnt ==
      $past(debug_msg2.deasm_full_drop_cnt) +
      ($past(deasm_full_drop_event) ? 64'd1 : 64'd0)
    );

  ap_pop_cmd_full_drop_counter_binding:
    assert property (
      !counter_clear |=>
      debug_msg2.pop_cmd_full_drop_cnt ==
      $past(debug_msg2.pop_cmd_full_drop_cnt) +
      ($past(pop_cmd_full_drop_event) ? 64'd1 : 64'd0)
    );

  ap_egress_not_ready_drop_counter_binding:
    assert property (
      !counter_clear |=>
      debug_msg2.egress_not_ready_drop_cnt ==
      $past(debug_msg2.egress_not_ready_drop_cnt) +
      ($past(egress_not_ready_drop_event) ? 64'd1 : 64'd0)
    );

  ap_deasm_only_drop_distinguishable:
    assert property (
      !counter_clear &&
      deasm_full_drop_event && !pop_cmd_full_drop_event && !egress_not_ready_drop_event |=>
      debug_msg2.deasm_full_drop_cnt == $past(debug_msg2.deasm_full_drop_cnt) + 64'd1 &&
      debug_msg2.pop_cmd_full_drop_cnt == $past(debug_msg2.pop_cmd_full_drop_cnt) &&
      debug_msg2.egress_not_ready_drop_cnt == $past(debug_msg2.egress_not_ready_drop_cnt)
    );

  ap_pop_cmd_only_drop_distinguishable:
    assert property (
      !counter_clear &&
      pop_cmd_full_drop_event && !deasm_full_drop_event && !egress_not_ready_drop_event |=>
      debug_msg2.pop_cmd_full_drop_cnt == $past(debug_msg2.pop_cmd_full_drop_cnt) + 64'd1 &&
      debug_msg2.deasm_full_drop_cnt == $past(debug_msg2.deasm_full_drop_cnt) &&
      debug_msg2.egress_not_ready_drop_cnt == $past(debug_msg2.egress_not_ready_drop_cnt)
    );

  ap_egress_only_drop_distinguishable:
    assert property (
      !counter_clear &&
      egress_not_ready_drop_event && !deasm_full_drop_event && !pop_cmd_full_drop_event |=>
      debug_msg2.egress_not_ready_drop_cnt == $past(debug_msg2.egress_not_ready_drop_cnt) + 64'd1 &&
      debug_msg2.deasm_full_drop_cnt == $past(debug_msg2.deasm_full_drop_cnt) &&
      debug_msg2.pop_cmd_full_drop_cnt == $past(debug_msg2.pop_cmd_full_drop_cnt)
    );

  ap_soft_reset_collapses_run_and_lock_state:
    assert property (
      csr_soft_reset_pulse |=>
      pop_engine_state == POP_IDLING &&
      run_state_cmd == RUN_IDLING &&
      pop_sector_lock_mask == '0 &&
      write_pointer == '0
    );

  ap_prepare_entry_clears_fifos:
    assert property (
      run_state_cmd == RUN_PREPARING && flush_cycle_count == 16'd0 |->
      deassembly_fifo_sclr && pop_cmd_fifo_sclr
    );

  ap_prepare_entry_clears_slot_valid:
    assert property (
      run_state_cmd == RUN_PREPARING && flush_cycle_count == 16'd0 |=>
      slot_valid == '0
    );

  ap_terminating_drain_done_is_exact:
    assert property (
      terminating_drain_done ==
      (run_state_cmd == RUN_TERMINATING &&
       deassembly_fifo_empty &&
       pop_cmd_fifo_empty &&
       pop_engine_state == POP_IDLING &&
       debug_msg2.push_cnt == (debug_msg2.pop_cnt + debug_msg2.overwrite_cnt))
    );

endmodule

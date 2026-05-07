// File name: ring_buffer_cam_sector_lock_sva.sv
// Purpose  : Formal safety checks for rbCAM sector-granular pop ownership.

module ring_buffer_cam_sector_lock_sva
  import ring_buffer_cam_sv_pkg::*;
#(
  parameter int RING_BUFFER_N_ENTRY     = 512,
  parameter int ADDR_W_CONST            = 9,
  parameter int LOCK_SECTOR_COUNT_CONST = 8,
  parameter int LOCK_SECTOR_BITS_CONST  = 3,
  parameter int ADDR_SECTOR_LSB_CONST   = 6
) (
  input logic                                      i_clk,
  input logic                                      i_rst,
  input run_state_e                                run_state_cmd,
  input logic                                      csr_soft_reset_pulse,
  input pop_state_e                                pop_engine_state,
  input logic                                      push_write_req,
  input logic                                      push_write_grant,
  input logic                                      push_erase_grant,
  input logic                                      pop_erase_grant,
  input logic                                      pop_flush_grant,
  input logic                                      push_write_sector_locked,
  input logic                                      push_erase_sector_locked,
  input logic                                      pop_flush_req,
  input logic                                      push_erase_req,
  input logic                                      pop_erase_req,
  input logic                                      pop_issue_inflight,
  input logic                                      pop_output_pending,
  input logic                                      pop_emit_pending,
  input logic [RING_BUFFER_N_ENTRY-1:0]            pop_snapshot,
  input logic [ADDR_W_CONST-1:0]                   write_pointer,
  input logic [ADDR_W_CONST-1:0]                   push_erase_addr_reg,
  input logic [15:0]                               pop_issue_addr,
  input logic [15:0]                               pop_issue_addr_pending,
  input logic [2:0]                                pop_search_wait_cnt,
  input logic [2:0]                                decision,
  input logic [2:0]                                decision_reg,
  input logic [LOCK_SECTOR_COUNT_CONST-1:0]        pop_sector_lock_mask
);

  default clocking cb @(posedge i_clk);
  endclocking

  default disable iff (i_rst || csr_soft_reset_pulse);

  function automatic logic [LOCK_SECTOR_BITS_CONST-1:0] addr_sector(
    input logic [ADDR_W_CONST-1:0] addr
  );
    return LOCK_SECTOR_BITS_CONST'(addr >> ADDR_SECTOR_LSB_CONST);
  endfunction

  function automatic logic [LOCK_SECTOR_COUNT_CONST-1:0] snapshot_sector_mask(
    input logic [RING_BUFFER_N_ENTRY-1:0] snap
  );
    logic [LOCK_SECTOR_COUNT_CONST-1:0] mask;
    logic [ADDR_W_CONST-1:0] addr;

    mask = '0;
    for (int i = 0; i < RING_BUFFER_N_ENTRY; i++) begin
      if (snap[i]) begin
        addr = ADDR_W_CONST'(i);
        mask[addr_sector(addr)] = 1'b1;
      end
    end
    return mask;
  endfunction

  initial begin
    ai_lock_mask_width_matches_parameter:
      assert ($bits(pop_sector_lock_mask) == LOCK_SECTOR_COUNT_CONST);
  end

  ap_search_blocks_push_write:
    assert property (!(pop_engine_state == POP_SEARCHING && push_write_grant));

  ap_search_blocks_push_erase:
    assert property (!(pop_engine_state == POP_SEARCHING && push_erase_grant));

  ap_locked_sector_blocks_push_write:
    assert property (
      !((pop_engine_state inside {POP_LOADING, POP_COUNTING, POP_DRAINING}) &&
        push_write_sector_locked &&
        push_write_grant)
    );

  ap_locked_sector_blocks_push_erase:
    assert property (
      !((pop_engine_state inside {POP_LOADING, POP_COUNTING, POP_DRAINING}) &&
        push_erase_sector_locked &&
        push_erase_grant)
    );

  ap_flush_owns_memory_arbiter:
    assert property (
      pop_flush_grant |-> (!push_write_grant && !push_erase_grant && !pop_erase_grant)
    );

  ap_push_erase_not_granted_during_flush:
    assert property (push_erase_grant |-> !pop_flush_req);

  ap_push_write_not_granted_during_flush:
    assert property (push_write_grant |-> !pop_flush_req);

  ap_blocked_push_erase_req_holds_without_grant:
    assert property (
      (push_erase_req &&
       ((pop_engine_state == POP_SEARCHING) ||
        ((pop_engine_state inside {POP_LOADING, POP_COUNTING, POP_DRAINING}) &&
         push_erase_sector_locked))) |-> !push_erase_grant
    );

  ap_mask_equals_snapshot_without_extra_locks:
    assert property (
      (pop_engine_state inside {POP_LOADING, POP_COUNTING, POP_DRAINING}) &&
      !pop_issue_inflight &&
      !pop_output_pending &&
      !pop_emit_pending &&
      !pop_erase_req |->
      pop_sector_lock_mask == snapshot_sector_mask(pop_snapshot)
    );

  ap_inflight_emit_addr_stays_locked:
    assert property (
      (pop_issue_inflight || pop_output_pending || pop_emit_pending) |->
      pop_sector_lock_mask[addr_sector(pop_issue_addr_pending[ADDR_W_CONST-1:0])]
    );

  ap_pop_erase_addr_stays_locked:
    assert property (
      pop_erase_req |->
      pop_sector_lock_mask[addr_sector(pop_issue_addr[ADDR_W_CONST-1:0])]
    );

  ap_push_write_sector_lock_is_exact:
    assert property (
      push_write_sector_locked ==
      ((pop_engine_state inside {POP_LOADING, POP_COUNTING, POP_DRAINING}) &&
       pop_sector_lock_mask[addr_sector(write_pointer)])
    );

  ap_push_erase_sector_lock_is_exact:
    assert property (
      push_erase_sector_locked ==
      ((pop_engine_state inside {POP_LOADING, POP_COUNTING, POP_DRAINING}) &&
       pop_sector_lock_mask[addr_sector(push_erase_addr_reg)])
    );

  ap_locks_clear_outside_active_pop_window:
    assert property (
      (pop_engine_state inside {
        POP_IDLING, POP_RESETTING, POP_SEARCHING, POP_FLUSHING, POP_FLUSHING_RST
      }) |->
      (!push_write_sector_locked && !push_erase_sector_locked)
    );

  ap_addr_sector_write_pointer_in_range:
    assert property (addr_sector(write_pointer) < LOCK_SECTOR_COUNT_CONST);

  ap_addr_sector_push_erase_in_range:
    assert property (addr_sector(push_erase_addr_reg) < LOCK_SECTOR_COUNT_CONST);

  ap_decision5_means_dual_grant:
    assert property (decision_reg == 3'd5 |-> push_write_grant && pop_erase_grant);

  ap_dual_grant_requires_disjoint_sectors:
    assert property (
      push_write_grant && pop_erase_grant |->
      addr_sector(write_pointer) != addr_sector(pop_issue_addr[ADDR_W_CONST-1:0])
    );

  ap_dual_grant_only_in_active_pop_window:
    assert property (
      push_write_grant && pop_erase_grant |->
      (pop_engine_state inside {POP_LOADING, POP_COUNTING, POP_DRAINING}) &&
      !pop_flush_req
    );

  ap_flush_decision_blocks_other_grants:
    assert property (
      decision == 3'd3 |->
      pop_flush_grant && !push_write_grant && !push_erase_grant && !pop_erase_grant
    );

  ap_decision_codepoint_valid:
    assert property (decision inside {3'd0, 3'd1, 3'd2, 3'd3, 3'd4, 3'd5});

  ap_unblocked_push_write_grants_now:
    assert property (
      push_write_req &&
      !push_erase_req &&
      !push_write_sector_locked &&
      !pop_flush_req &&
      pop_engine_state != POP_SEARCHING |->
      push_write_grant
    );

  ap_unblocked_pop_erase_grants_now:
    assert property (pop_erase_req && !pop_flush_req |-> pop_erase_grant);

  ap_search_wait_advances_before_load:
    assert property (
      pop_engine_state == POP_SEARCHING &&
      run_state_cmd != RUN_PREPARING &&
      pop_search_wait_cnt < 3'd5 |=>
      pop_engine_state == POP_SEARCHING &&
      pop_search_wait_cnt == $past(pop_search_wait_cnt) + 3'd1
    );

  ap_search_tail_enters_load:
    assert property (
      pop_engine_state == POP_SEARCHING &&
      run_state_cmd != RUN_PREPARING &&
      pop_search_wait_cnt == 3'd5 |=>
      pop_engine_state == POP_LOADING
    );

  ap_flush_request_grants_now:
    assert property (pop_flush_req |-> pop_flush_grant);

endmodule

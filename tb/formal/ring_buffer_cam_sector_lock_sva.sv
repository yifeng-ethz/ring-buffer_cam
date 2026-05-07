// File name: ring_buffer_cam_sector_lock_sva.sv
// Purpose  : Formal safety checks for rbCAM sector-granular pop ownership.

module ring_buffer_cam_sector_lock_sva
  import ring_buffer_cam_sv_pkg::*;
(
  input logic       i_clk,
  input logic       i_rst,
  input pop_state_e pop_engine_state,
  input logic       push_write_grant,
  input logic       push_erase_grant,
  input logic       pop_erase_grant,
  input logic       pop_flush_grant,
  input logic       push_write_sector_locked,
  input logic       push_erase_sector_locked,
  input logic       pop_flush_req,
  input logic       push_erase_req
);

  default clocking cb @(posedge i_clk);
  endclocking

  default disable iff (i_rst);

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

endmodule

// File name: ring_buffer_cam_metadata_sva.sv
// Purpose  : Formal checks for rbCAM metadata lineage through FIFO, slots, and emit.

module ring_buffer_cam_metadata_sva
  import ring_buffer_cam_sv_pkg::*;
#(
  parameter int RING_BUFFER_N_ENTRY = 512,
  parameter int ADDR_W_CONST        = 9
) (
  input logic                           i_clk,
  input logic                           i_rst,
  input run_state_e                     run_state_cmd,
  input logic                           csr_soft_reset_pulse,
  input logic [63:0]                    asi_hit_type1_metadata,
  input logic                           asi_hit_type1_metadata_valid,
  input logic                           deassembly_fifo_wrreq,
  input logic [63:0]                    deassembly_fifo_din_metadata,
  input logic                           deassembly_fifo_din_metadata_valid,
  input logic [63:0]                    deassembly_fifo_dout_metadata,
  input logic                           deassembly_fifo_dout_metadata_valid,
  input logic                           push_write_grant,
  input logic [ADDR_W_CONST-1:0]        write_pointer,
  input logic [RING_BUFFER_N_ENTRY-1:0] slot_valid,
  input logic [63:0]                    slot_metadata [RING_BUFFER_N_ENTRY],
  input logic                           slot_metadata_valid [RING_BUFFER_N_ENTRY],
  input logic                           pop_erase_grant,
  input logic [15:0]                    pop_issue_addr,
  input logic [63:0]                    pop_metadata_pending,
  input logic                           pop_metadata_valid_pending,
  input logic                           aso_hit_type2_valid,
  input logic                           aso_hit_type2_startofpacket,
  input logic [63:0]                    aso_hit_type2_metadata,
  input logic                           aso_hit_type2_metadata_valid
);

  default clocking cb @(posedge i_clk);
  endclocking

  default disable iff (i_rst || csr_soft_reset_pulse || run_state_cmd == RUN_PREPARING);

  ap_deassembly_fifo_write_metadata_matches_ingress:
    assert property (
      deassembly_fifo_wrreq |->
      deassembly_fifo_din_metadata == asi_hit_type1_metadata &&
      deassembly_fifo_din_metadata_valid == asi_hit_type1_metadata_valid
    );

  ap_slot_write_metadata_comes_from_fifo_output:
    assert property (
      push_write_grant |=>
      !slot_valid[$past(write_pointer)] ||
      (slot_metadata_valid[$past(write_pointer)] == $past(deassembly_fifo_dout_metadata_valid) &&
       (!$past(deassembly_fifo_dout_metadata_valid) ||
        slot_metadata[$past(write_pointer)] == $past(deassembly_fifo_dout_metadata)))
    );

  ap_pop_metadata_pending_comes_from_slot:
    assert property (
      pop_erase_grant && slot_valid[pop_issue_addr[ADDR_W_CONST-1:0]] |=>
      pop_metadata_valid_pending == $past(slot_metadata_valid[pop_issue_addr[ADDR_W_CONST-1:0]]) &&
      (!$past(slot_metadata_valid[pop_issue_addr[ADDR_W_CONST-1:0]]) ||
       pop_metadata_pending == $past(slot_metadata[pop_issue_addr[ADDR_W_CONST-1:0]]))
    );

  ap_emit_hit_metadata_uses_pending_pop_metadata:
    assert property (
      aso_hit_type2_valid && !aso_hit_type2_startofpacket |->
      aso_hit_type2_metadata == pop_metadata_pending &&
      aso_hit_type2_metadata_valid == pop_metadata_valid_pending
    );

endmodule

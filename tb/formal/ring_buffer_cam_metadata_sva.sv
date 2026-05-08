// File name: ring_buffer_cam_metadata_sva.sv
// Purpose  : Formal checks for rbCAM metadata lineage through FIFO, slots, and emit.

module ring_buffer_cam_metadata_sva
  import ring_buffer_cam_sv_pkg::*;
#(
  parameter int RING_BUFFER_N_ENTRY = 512,
  parameter int ADDR_W_CONST        = 9,
  parameter int DEBUG               = 1
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
  input logic [RING_BUFFER_N_ENTRY-1:0][63:0] slot_metadata,
  input logic [RING_BUFFER_N_ENTRY-1:0] slot_metadata_valid,
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

  generate
    if (DEBUG >= 2) begin : gen_metadata_enabled
      logic                           tracker_disable;
      logic                           slot_write_check_d1;
      logic [ADDR_W_CONST-1:0]        slot_write_addr_d1;
      logic [63:0]                    slot_write_metadata_d1;
      logic                           slot_write_metadata_valid_d1;
      logic                           pop_metadata_check_d1;
      logic [ADDR_W_CONST-1:0]        pop_issue_addr_d1;
      logic [63:0]                    pop_slot_metadata_d1;
      logic                           pop_slot_metadata_valid_d1;

      assign tracker_disable = csr_soft_reset_pulse || run_state_cmd == RUN_PREPARING;

      always_ff @(posedge i_clk or posedge i_rst) begin
        if (i_rst) begin
          slot_write_check_d1 <= 1'b0;
          slot_write_addr_d1 <= '0;
          slot_write_metadata_d1 <= '0;
          slot_write_metadata_valid_d1 <= 1'b0;
          pop_metadata_check_d1 <= 1'b0;
          pop_issue_addr_d1 <= '0;
          pop_slot_metadata_d1 <= '0;
          pop_slot_metadata_valid_d1 <= 1'b0;
        end else if (tracker_disable) begin
          slot_write_check_d1 <= 1'b0;
          slot_write_addr_d1 <= '0;
          slot_write_metadata_d1 <= '0;
          slot_write_metadata_valid_d1 <= 1'b0;
          pop_metadata_check_d1 <= 1'b0;
          pop_issue_addr_d1 <= '0;
          pop_slot_metadata_d1 <= '0;
          pop_slot_metadata_valid_d1 <= 1'b0;
        end else begin
          if (slot_write_check_d1) begin
            ap_slot_write_metadata_comes_from_fifo_output:
              assert (
                slot_valid[slot_write_addr_d1] &&
                slot_metadata_valid[slot_write_addr_d1] == slot_write_metadata_valid_d1 &&
                (!slot_write_metadata_valid_d1 ||
                 slot_metadata[slot_write_addr_d1] == slot_write_metadata_d1)
              );
          end

          if (pop_metadata_check_d1) begin
            ap_pop_metadata_pending_comes_from_slot:
              assert (
                pop_metadata_valid_pending == pop_slot_metadata_valid_d1 &&
                (!pop_slot_metadata_valid_d1 ||
                 pop_metadata_pending == pop_slot_metadata_d1)
              );
          end

          slot_write_check_d1 <= push_write_grant;
          slot_write_addr_d1 <= write_pointer;
          slot_write_metadata_d1 <= deassembly_fifo_dout_metadata;
          slot_write_metadata_valid_d1 <= deassembly_fifo_dout_metadata_valid;

          pop_metadata_check_d1 <=
            pop_erase_grant && slot_valid[pop_issue_addr[ADDR_W_CONST-1:0]];
          pop_issue_addr_d1 <= pop_issue_addr[ADDR_W_CONST-1:0];
          pop_slot_metadata_d1 <= slot_metadata[pop_issue_addr[ADDR_W_CONST-1:0]];
          pop_slot_metadata_valid_d1 <=
            slot_metadata_valid[pop_issue_addr[ADDR_W_CONST-1:0]];
        end
      end

      ap_deassembly_fifo_write_metadata_matches_ingress:
        assert property (
          deassembly_fifo_wrreq |->
          deassembly_fifo_din_metadata ==
            (asi_hit_type1_metadata_valid ? asi_hit_type1_metadata : 64'd0) &&
          deassembly_fifo_din_metadata_valid == asi_hit_type1_metadata_valid
        );

      ap_emit_hit_metadata_uses_pending_pop_metadata:
        assert property (
          aso_hit_type2_valid && !aso_hit_type2_startofpacket |->
          aso_hit_type2_metadata == pop_metadata_pending &&
          aso_hit_type2_metadata_valid == pop_metadata_valid_pending
        );
    end else begin : gen_metadata_disabled
      ap_deassembly_fifo_metadata_tied_off:
        assert property (
          deassembly_fifo_din_metadata == 64'd0 &&
          !deassembly_fifo_din_metadata_valid
        );

      ap_emit_metadata_tied_off:
        assert property (
          aso_hit_type2_metadata == 64'd0 &&
          !aso_hit_type2_metadata_valid
        );
    end
  endgenerate

endmodule

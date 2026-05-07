module ring_buffer_cam_syn_sv_p4_top (
  input  logic        clk125,
  input  logic        reset_n,
  output logic [31:0] probe_out
);
  ring_buffer_cam_syn_sv_harness #(
    .G_RING_BUFFER_N_ENTRY(512),
    .G_N_PARTITIONS(4),
    .G_ENCODER_LEAF_WIDTH(16),
    .G_ENCODER_PIPE_STAGES(4)
  ) u_harness (
    .clk125(clk125),
    .reset_n(reset_n),
    .probe_out(probe_out)
  );
endmodule

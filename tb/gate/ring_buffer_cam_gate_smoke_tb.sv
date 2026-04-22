`timescale 1ps/1ps

module ring_buffer_cam_gate_smoke_tb;
  logic        clk125  = 1'b0;
  logic        reset_n = 1'b0;
  logic [31:0] probe_out;

  int unsigned sample_cycles = 500_000;

  ring_buffer_cam_syn_p4_top dut (
    .clk125    (clk125),
    .reset_n   (reset_n),
    .probe_out (probe_out)
  );

  always #4000 clk125 = ~clk125;

  initial begin
    void'($value$plusargs("SAMPLE_CYCLES=%d", sample_cycles));

    repeat (20) @(posedge clk125);
    reset_n = 1'b1;
    repeat (sample_cycles) @(posedge clk125);

    if ($isunknown(probe_out)) begin
      $display("RBCAM_SIGNATURE=UNKNOWN");
      $fatal(1, "probe_out contains X/Z after %0d cycles", sample_cycles);
    end
    if (probe_out == 32'h0000_0000) begin
      $display("RBCAM_SIGNATURE=0x%08x", probe_out);
      $fatal(1, "probe_out stayed zero after %0d cycles", sample_cycles);
    end

    $display("RBCAM_SIGNATURE=0x%08x", probe_out);
    $display("*** TEST PASSED ***");
    $finish;
  end
endmodule

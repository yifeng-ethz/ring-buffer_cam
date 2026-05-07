// Standalone synthesis harness for the SystemVerilog rbCAM implementation.
module ring_buffer_cam_syn_sv_harness #(
  parameter int G_RING_BUFFER_N_ENTRY = 512,
  parameter int G_N_PARTITIONS = 4,
  parameter int G_ENCODER_LEAF_WIDTH = 16,
  parameter int G_ENCODER_PIPE_STAGES = 4
) (
  input  logic        clk125,
  input  logic        reset_n,
  output logic [31:0] probe_out
);
  localparam logic [8:0] CTRL_IDLE_CONST        = 9'b0_0000_0001;
  localparam logic [8:0] CTRL_RUN_PREPARE_CONST = 9'b0_0000_0010;
  localparam logic [8:0] CTRL_SYNC_CONST        = 9'b0_0000_0100;
  localparam logic [8:0] CTRL_RUNNING_CONST     = 9'b0_0000_1000;
  localparam int PREPARE_WAIT_CYCLES_CONST = 300000;
  localparam int EXPECTED_LATENCY_CONST = 128;

  typedef enum logic [2:0] {
    ISSUE_PREPARE,
    WAIT_PREPARE,
    ISSUE_LATENCY,
    ISSUE_SYNC,
    ISSUE_RUNNING,
    STREAM
  } stim_state_e;

  logic rst;
  stim_state_e stim_state;
  logic [18:0] prepare_wait_counter;
  logic [15:0] hit_counter;
  logic [5:0] search_key_counter;
  logic [31:0] signature;

  logic [31:0] avs_csr_readdata;
  logic avs_csr_read;
  logic [4:0] avs_csr_address;
  logic avs_csr_waitrequest;
  logic avs_csr_write;
  logic [31:0] avs_csr_writedata;

  logic [8:0] asi_ctrl_data;
  logic asi_ctrl_valid;
  logic asi_ctrl_ready;

  logic [3:0] asi_hit_type1_channel;
  logic asi_hit_type1_startofpacket;
  logic asi_hit_type1_endofpacket;
  logic asi_hit_type1_empty;
  logic [38:0] asi_hit_type1_data;
  logic asi_hit_type1_valid;
  logic asi_hit_type1_ready;
  logic [0:0] asi_hit_type1_error;
  logic [63:0] asi_hit_type1_metadata;
  logic asi_hit_type1_metadata_valid;

  logic [3:0] aso_hit_type2_channel;
  logic aso_hit_type2_startofpacket;
  logic aso_hit_type2_endofpacket;
  logic [35:0] aso_hit_type2_data;
  logic aso_hit_type2_valid;
  logic aso_hit_type2_ready;
  logic [0:0] aso_hit_type2_error;
  logic [63:0] aso_hit_type2_metadata;
  logic aso_hit_type2_metadata_valid;
  logic [15:0] aso_filllevel_data;
  logic aso_filllevel_valid;
  logic [31:0] coe_debug_fill_level;
  logic [31:0] coe_debug_fifo_level;
  logic [31:0] coe_debug_queue_state;

  assign rst = ~reset_n;
  assign aso_hit_type2_ready = 1'b1;
  assign asi_hit_type1_metadata = 64'h0;
  assign asi_hit_type1_metadata_valid = 1'b0;
  assign probe_out = signature;

  always_ff @(posedge clk125 or posedge rst) begin
    if (rst) begin
      stim_state <= ISSUE_PREPARE;
      prepare_wait_counter <= '0;
      hit_counter <= '0;
      search_key_counter <= '0;
      avs_csr_read <= 1'b0;
      avs_csr_address <= '0;
      avs_csr_write <= 1'b0;
      avs_csr_writedata <= '0;
      asi_ctrl_data <= CTRL_IDLE_CONST;
      asi_ctrl_valid <= 1'b0;
      asi_hit_type1_channel <= '0;
      asi_hit_type1_startofpacket <= 1'b0;
      asi_hit_type1_endofpacket <= 1'b0;
      asi_hit_type1_empty <= 1'b0;
      asi_hit_type1_data <= '0;
      asi_hit_type1_valid <= 1'b0;
      asi_hit_type1_error <= '0;
    end else begin
      avs_csr_read <= 1'b0;
      avs_csr_address <= '0;
      avs_csr_write <= 1'b0;
      avs_csr_writedata <= '0;
      asi_ctrl_data <= CTRL_IDLE_CONST;
      asi_ctrl_valid <= 1'b0;
      asi_hit_type1_channel <= '0;
      asi_hit_type1_startofpacket <= 1'b0;
      asi_hit_type1_endofpacket <= 1'b0;
      asi_hit_type1_empty <= 1'b0;
      asi_hit_type1_data <= '0;
      asi_hit_type1_valid <= 1'b0;
      asi_hit_type1_error <= '0;

      unique case (stim_state)
        ISSUE_PREPARE: begin
          asi_ctrl_data <= CTRL_RUN_PREPARE_CONST;
          asi_ctrl_valid <= 1'b1;
          if (asi_ctrl_ready) begin
            prepare_wait_counter <= '0;
            stim_state <= WAIT_PREPARE;
          end
        end
        WAIT_PREPARE: begin
          if (prepare_wait_counter == PREPARE_WAIT_CYCLES_CONST - 1) begin
            stim_state <= ISSUE_LATENCY;
          end else begin
            prepare_wait_counter <= prepare_wait_counter + 1'b1;
          end
        end
        ISSUE_LATENCY: begin
          avs_csr_address <= 5'd1;
          avs_csr_writedata <= EXPECTED_LATENCY_CONST[31:0];
          avs_csr_write <= 1'b1;
          if (!avs_csr_waitrequest) begin
            stim_state <= ISSUE_SYNC;
          end
        end
        ISSUE_SYNC: begin
          asi_ctrl_data <= CTRL_SYNC_CONST;
          asi_ctrl_valid <= 1'b1;
          if (asi_ctrl_ready) begin
            stim_state <= ISSUE_RUNNING;
          end
        end
        ISSUE_RUNNING: begin
          asi_ctrl_data <= CTRL_RUNNING_CONST;
          asi_ctrl_valid <= 1'b1;
          if (asi_ctrl_ready) begin
            stim_state <= STREAM;
          end
        end
        STREAM: begin
          logic [7:0] search_key;
          logic [12:0] tcc8n;
          search_key = {search_key_counter, 2'b00};
          tcc8n = {1'b0, search_key, 4'b0000};
          asi_hit_type1_valid <= 1'b1;
          asi_hit_type1_channel <= hit_counter[3:0];
          asi_hit_type1_data[38:35] <= hit_counter[3:0];
          asi_hit_type1_data[34:30] <= hit_counter[8:4];
          asi_hit_type1_data[29:17] <= tcc8n;
          asi_hit_type1_data[16:14] <= hit_counter[11:9];
          asi_hit_type1_data[13:9] <= hit_counter[15:11] ^ 5'b10101;
          asi_hit_type1_data[8:0] <= hit_counter[8:0];
          if (asi_hit_type1_ready) begin
            hit_counter <= hit_counter + 1'b1;
            search_key_counter <= search_key_counter + 1'b1;
          end
        end
        default: stim_state <= ISSUE_PREPARE;
      endcase
    end
  end

  always_ff @(posedge clk125 or posedge rst) begin
    if (rst) begin
      signature <= '0;
    end else begin
      if (aso_hit_type2_valid) begin
        signature <= signature ^
          {aso_hit_type2_error,
           aso_hit_type2_startofpacket,
           aso_hit_type2_endofpacket,
           aso_hit_type2_channel,
           aso_hit_type2_data[24:0]};
      end
      if (aso_filllevel_valid) begin
        signature[15:0] <= signature[15:0] ^ aso_filllevel_data;
      end
      if (aso_hit_type2_metadata_valid) begin
        signature <= signature ^ aso_hit_type2_metadata[31:0] ^ aso_hit_type2_metadata[63:32];
      end
      signature[31:24] <= signature[31:24] ^ hit_counter[7:0];
      signature[23] <= signature[23] ^ asi_hit_type1_ready;
      signature[22:20] <= signature[22:20] ^ coe_debug_queue_state[2:0];
      signature[19:16] <= signature[19:16] ^ coe_debug_fifo_level[3:0];
      signature[15:12] <= signature[15:12] ^ coe_debug_fill_level[3:0];
      signature[11:0] <= signature[11:0] ^ avs_csr_readdata[11:0];
    end
  end

  ring_buffer_cam #(
    .SEARCH_KEY_WIDTH(G_ENCODER_LEAF_WIDTH / 2),
    .RING_BUFFER_N_ENTRY(G_RING_BUFFER_N_ENTRY),
    .SIDE_DATA_BITS(31),
    .INTERLEAVING_FACTOR(4),
    .INTERLEAVING_INDEX(0),
    .N_PARTITIONS(G_N_PARTITIONS),
    .ENCODER_LEAF_WIDTH(G_ENCODER_LEAF_WIDTH),
    .ENCODER_PIPE_STAGES(G_ENCODER_PIPE_STAGES),
    .DEBUG(0)
  ) u_dut (
    .avs_csr_readdata(avs_csr_readdata),
    .avs_csr_read(avs_csr_read),
    .avs_csr_address(avs_csr_address),
    .avs_csr_waitrequest(avs_csr_waitrequest),
    .avs_csr_write(avs_csr_write),
    .avs_csr_writedata(avs_csr_writedata),
    .asi_hit_type1_channel(asi_hit_type1_channel),
    .asi_hit_type1_startofpacket(asi_hit_type1_startofpacket),
    .asi_hit_type1_endofpacket(asi_hit_type1_endofpacket),
    .asi_hit_type1_empty(asi_hit_type1_empty),
    .asi_hit_type1_data(asi_hit_type1_data),
    .asi_hit_type1_valid(asi_hit_type1_valid),
    .asi_hit_type1_ready(asi_hit_type1_ready),
    .asi_hit_type1_error(asi_hit_type1_error),
    .asi_hit_type1_metadata(asi_hit_type1_metadata),
    .asi_hit_type1_metadata_valid(asi_hit_type1_metadata_valid),
    .aso_hit_type2_channel(aso_hit_type2_channel),
    .aso_hit_type2_startofpacket(aso_hit_type2_startofpacket),
    .aso_hit_type2_endofpacket(aso_hit_type2_endofpacket),
    .aso_hit_type2_data(aso_hit_type2_data),
    .aso_hit_type2_valid(aso_hit_type2_valid),
    .aso_hit_type2_ready(aso_hit_type2_ready),
    .aso_hit_type2_error(aso_hit_type2_error),
    .aso_hit_type2_metadata(aso_hit_type2_metadata),
    .aso_hit_type2_metadata_valid(aso_hit_type2_metadata_valid),
    .i_clk(clk125),
    .i_rst(rst),
    .asi_ctrl_data(asi_ctrl_data),
    .asi_ctrl_valid(asi_ctrl_valid),
    .asi_ctrl_ready(asi_ctrl_ready),
    .aso_filllevel_data(aso_filllevel_data),
    .aso_filllevel_valid(aso_filllevel_valid),
    .coe_debug_fill_level(coe_debug_fill_level),
    .coe_debug_fifo_level(coe_debug_fifo_level),
    .coe_debug_queue_state(coe_debug_queue_state)
  );
endmodule

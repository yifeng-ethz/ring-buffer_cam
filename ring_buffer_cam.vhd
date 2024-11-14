-- File name: ring_buffer_cam.vhd 
-- Author: Yifeng Wang (yifenwan@phys.ethz.ch)
-- =======================================
-- Revision: 1.0 (file created)
--		Date: Jul 4, 2024
-- Revision: 2.0 (functional verified)
--		Date: Jul 29, 2024
--
-- =========
-- Description:	[Ring-buffer Shaped Content-Addressable-Memory (CAM)] 
--		This cam is implemented in a special shape of a ring-buffer, where write pointer (wr_ptr) is ever mono-increasing 
--		and the read pointer (rd_ptr) is controlled by the look-up result address. The look-up is supported by the CAM natively.
--
--		Functional Description: 

--			Push: Write-alike. The "Push Flow" is: (1) Check the targeted address, if free, skip step 2. (2) Erase that address 
--		
--			Pop: Read-alike. The "Pop Flow" is: (1) Look up the data in the CAM, if found, go to step 2 (2) Erase that address
--				and retrieve the stored data. 
--		
--		Functional Diagram:
--			
--
--
-- ================ synthsizer configuration =================== 		
-- altera vhdl_input_version vhdl_2008
-- ============================================================= 

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use IEEE.math_real.log2;
use IEEE.math_real.ceil;
use ieee.std_logic_misc.or_reduce;

entity ring_buffer_cam is 
generic(
	SEARCH_KEY_WIDTH	: natural := 8; -- = timestamp length (8ns) [11:4]
	RING_BUFFER_N_ENTRY	: natural := 512; -- = CAM size, can be tuned
	SIDE_DATA_BITS		: natural := 31; -- type 1 has 39 bit, exclude the search key (8bit), resulting 31 bit
	--SEL_CHANNEL			: natural := 0; -- from 0 to 31 (the selected channel of avst input, other channels will be ignored) -- deprecated
	INTERLEAVING_FACTOR	: natural := 4; -- only power of 2 are allowed, this number multiple of ring buffer cam will be instantiated
	INTERLEAVING_INDEX	: natural := 0; -- assign an unique index for each ring buffer cam. 
	DEBUG				: natural := 1
);
port(
	-- control and status registers interface
	avs_csr_readdata				: out std_logic_vector(31 downto 0);
	avs_csr_read					: in  std_logic;
	avs_csr_address					: in  std_logic_vector(4 downto 0);
	avs_csr_waitrequest				: out std_logic; 
	avs_csr_write					: in  std_logic;
	avs_csr_writedata				: in  std_logic_vector(31 downto 0);
	
	-- run control interface
	asi_ctrl_data					: in  std_logic_vector(8 downto 0); 
	asi_ctrl_valid					: in  std_logic;
	asi_ctrl_ready					: out std_logic;
	
	-- ============ INGRESS ==============
	-- input stream of processed hits
	asi_hit_type1_channel			: in  std_logic_vector(3 downto 0); -- max_channel=15
	asi_hit_type1_startofpacket		: in  std_logic; -- packet is supported by upstream
	asi_hit_type1_endofpacket		: in  std_logic; -- processor use this to mark the start and end of run (sor/eor)
	asi_hit_type1_data				: in  std_logic_vector(38 downto 0);
	asi_hit_type1_valid				: in  std_logic;
	asi_hit_type1_ready				: out std_logic; -- itself has fifo, so no backpressure is required for upstream, just check for fifo full. 
	
	-- ============ EGRESS ==============
	-- output stream of framed hits (aligned to word) (ts[3:0])
	aso_hit_type2_channel			: out std_logic_vector(3 downto 0); -- max_channel=15
	aso_hit_type2_startofpacket		: out std_logic; -- sop at each subheader
	aso_hit_type2_endofpacket		: out std_logic; -- eop at last hit in this subheader. if no hit, eop at subheader.
	aso_hit_type2_data				: out std_logic_vector(35 downto 0); -- [35:32] byte_is_k: "0001"=sub-header. "0000"=hit.
	-- two cases for [31:0]
	-- 1) sub-header: [31:24]=ts[11:4], [23:16]=TBD, [15:8]=hit_cnt[7:0], [7:0]=K23.7
	-- 2) hit: [31:0]=specbook MuTRiG hit format
	aso_hit_type2_valid				: out std_logic;
	aso_hit_type2_ready				: in  std_logic;
	
	
	-- clock and reset interface
	i_rst							: in  std_logic;
	i_clk							: in  std_logic



);
end entity ring_buffer_cam;

architecture rtl of ring_buffer_cam is 
	
	-- universal 8b10b
	constant K285					: std_logic_vector(7 downto 0) := "10111100"; -- 16#BC#
	constant K284					: std_logic_vector(7 downto 0) := "10011100"; -- 16#9C#
	constant K237					: std_logic_vector(7 downto 0) := "11110111"; -- 16#F7#
	-- input avst data format
	constant ASIC_HI				: natural := 38;
	constant ASIC_LO				: natural := 35;
	constant CHANNEL_HI				: natural := 34;
	constant CHANNEL_LO				: natural := 30;
	constant TCC8N_HI				: natural := 29; -- 28
	constant TCC8N_LO				: natural := 17; -- 21
	constant TCC1n6_HI				: natural := 16; 
	constant TCC1n6_LO				: natural := 14; 
	constant TFINE_HI				: natural := 13;
	constant TFINE_LO				: natural := 9;
	constant ET1n6_HI				: natural := 8;
	constant ET1n6_LO				: natural := 0;
	-- search key range in data ts8n
	constant SK_RANGE_HI			: natural := 11; -- only this range of ts8n are treated as search key
	constant SK_RANGE_LO			: natural := 4;
	-- generic for main cam
	constant MAIN_CAM_SIZE			: natural := RING_BUFFER_N_ENTRY;
	constant MAIN_CAM_DATA_WIDTH	: natural := integer(ceil(real(SEARCH_KEY_WIDTH)/8.0)*8.0); -- TODO: adapt for true dpram
	constant MAIN_CAM_ADDR_WIDTH	: natural := integer(ceil(log2(real(RING_BUFFER_N_ENTRY)))); -- 9 bit(512 entry), 6 bit(64 entry)
	-- generic for side ram
	constant SRAM_DATA_WIDTH		: natural := 1 + SEARCH_KEY_WIDTH + SIDE_DATA_BITS; -- bit[39] = occupancy flag, bit[38:0] = hit_type1
	constant SRAM_ADDR_WIDTH		: natural := integer(ceil(log2(real(RING_BUFFER_N_ENTRY)))); -- 9 bit
	-- interleaving feature
	constant TCC8N_INTERLEAVING_BITS	: natural := integer(ceil(log2(real(INTERLEAVING_FACTOR)))); -- 2 bit for 4 copies
	constant TCC8N_INTERLEAVING_LO		: natural := TCC8N_LO + SK_RANGE_LO; -- 21 (offset to input avst data format)
	constant TCC8N_INTERLEAVING_HI		: natural := TCC8N_INTERLEAVING_LO + TCC8N_INTERLEAVING_BITS - 1; -- 22
	
	-- main cam
	signal cam_erase_en				: std_logic;
	signal cam_wr_en				: std_logic;
	signal cam_wr_data				: std_logic_vector(MAIN_CAM_DATA_WIDTH-1 downto 0);
	signal cam_wr_addr				: std_logic_vector(MAIN_CAM_ADDR_WIDTH-1 downto 0);
	signal cam_cmp_din				: std_logic_vector(MAIN_CAM_DATA_WIDTH-1 downto 0);
	signal cam_match_addr_oh		: std_logic_vector(MAIN_CAM_SIZE-1 downto 0);
	
	
	-- side ram
	signal side_ram_raddr			: std_logic_vector(SRAM_ADDR_WIDTH-1 downto 0);
	signal side_ram_dout			: std_logic_vector(SRAM_DATA_WIDTH-1 downto 0);
	signal side_ram_we				: std_logic;
	signal side_ram_waddr			: std_logic_vector(SRAM_ADDR_WIDTH-1 downto 0);
	signal side_ram_din				: std_logic_vector(SRAM_DATA_WIDTH-1 downto 0);
	
	-- pop command fifo 
	constant POP_CMD_FIFO_LPM_WIDTH	: natural := 8;
	constant POP_CMD_FIFO_LPM_WIDTHU	: natural := 2;
	signal pop_cmd_fifo_rdack		: std_logic;
	signal pop_cmd_fifo_dout		: std_logic_vector(POP_CMD_FIFO_LPM_WIDTH-1 downto 0);
	signal pop_cmd_fifo_wrreq		: std_logic;
	signal pop_cmd_fifo_din			: std_logic_vector(POP_CMD_FIFO_LPM_WIDTH-1 downto 0);
	signal pop_cmd_fifo_empty		: std_logic;
	signal pop_cmd_fifo_full		: std_logic;
	signal pop_cmd_fifo_usedw		: std_logic_vector(POP_CMD_FIFO_LPM_WIDTHU-1 downto 0);
	signal pop_cmd_fifo_sclr		: std_logic;
	-- pop command fifo (cmp)
	component scfifo_w8d32
	PORT
	(
		clock		: IN STD_LOGIC ;
		data		: IN STD_LOGIC_VECTOR (POP_CMD_FIFO_LPM_WIDTH-1 DOWNTO 0);
		rdreq		: IN STD_LOGIC ;
		sclr		: IN STD_LOGIC ;
		wrreq		: IN STD_LOGIC ;
		empty		: OUT STD_LOGIC ;
		full		: OUT STD_LOGIC ;
		q			: OUT STD_LOGIC_VECTOR (POP_CMD_FIFO_LPM_WIDTH-1 DOWNTO 0);
		usedw		: OUT STD_LOGIC_VECTOR (POP_CMD_FIFO_LPM_WIDTHU-1 DOWNTO 0)
	);
	end component;
	
	-- deassembly fifo
	constant DEASSEMBLY_FIFO_LPM_WIDTH	: natural := 40;
	constant DEASSEMBLY_FIFO_LPM_WIDTHU	: natural := 8;
	signal deassembly_fifo_rdack		: std_logic;
	signal deassembly_fifo_dout			: std_logic_vector(DEASSEMBLY_FIFO_LPM_WIDTH-1 downto 0);
	signal deassembly_fifo_wrreq		: std_logic;
	signal deassembly_fifo_din			: std_logic_vector(DEASSEMBLY_FIFO_LPM_WIDTH-1 downto 0);
	signal deassembly_fifo_empty		: std_logic;
	signal deassembly_fifo_full			: std_logic;
	signal deassembly_fifo_usedw		: std_logic_vector(DEASSEMBLY_FIFO_LPM_WIDTHU-1 downto 0);
	signal deassembly_fifo_sclr			: std_logic;
	-- deassembly fifo (cmp)
	component scfifo_w40d256
	PORT
	(
		clock		: IN STD_LOGIC ;
		data		: IN STD_LOGIC_VECTOR (39 DOWNTO 0);
		rdreq		: IN STD_LOGIC ;
		sclr		: IN STD_LOGIC ;
		wrreq		: IN STD_LOGIC ;
		empty		: OUT STD_LOGIC ;
		full		: OUT STD_LOGIC ;
		q			: OUT STD_LOGIC_VECTOR (39 DOWNTO 0);
		usedw		: OUT STD_LOGIC_VECTOR (7 DOWNTO 0)
	);
	end component;
	
	-- gts counter
	signal gts_8n					: unsigned(47 downto 0);
	signal gts_counter_rst			: std_logic;
	
	-- ------------------
	-- match encoder
	-- ------------------
	-- one-hot to binary encoding combinational logic 
	-- all comb avaliable at EVAL state
	-- all reg must be latched before EVAL, so in POPING or SEARCH state. 
	constant SMALL_LOGIC_CAM_SIZE			: natural := 64; -- this is the maximum on Arria V
	constant N_SMALL_LOGIC					: natural := RING_BUFFER_N_ENTRY / SMALL_LOGIC_CAM_SIZE;
	constant SMALL_LOGIC_ADDR_W				: natural := integer(ceil(log2(real(SMALL_LOGIC_CAM_SIZE)))); -- should be 6 bit
	-- one-hot
	type small_logic_onehot_t				is array (0 to N_SMALL_LOGIC-1) of std_logic_vector(SMALL_LOGIC_CAM_SIZE-1 downto 0);
	signal cam_match_addr_oh_2d_comb		: small_logic_onehot_t; -- input to match encoder (comb)
	signal match_encoder_addr_raw			: small_logic_onehot_t; -- input to match encoder (reg)
	signal match_encoder_addr_raw_nxt_comb	: small_logic_onehot_t; -- output onehot
	-- binary
	type small_logic_binary_t				is array (0 to N_SMALL_LOGIC-1) of std_logic_vector(SMALL_LOGIC_ADDR_W-1 downto 0);
	type small_logic_binary_cnt_t			is array (0 to N_SMALL_LOGIC-1) of std_logic_vector(SMALL_LOGIC_ADDR_W downto 0);
	signal match_encoder_count_comb			: small_logic_binary_cnt_t; -- output count 
	signal match_encoder_addr0_comb			: small_logic_binary_t; -- addr in each bank (lsb)
	signal match_encoder_addr1_comb			: small_logic_binary_t; -- addr in each bank (msb)
	-- flag
	signal match_encoder_flag_comb			: std_logic_vector(N_SMALL_LOGIC-1 downto 0); -- flags from all small logic entities
	
	

	-- ---------------------
	-- bank combiner 
	-- ---------------------
	-- combinational logic derive central values from the latch output from match encoder
	-- works at POPING reg to provide address. if granted, go to EVAL or RESET, based on total count.
	-- internal
	signal bank_combiner_grant_comb			: std_logic_vector(N_SMALL_LOGIC-1 downto 0); -- grant based on the req (flags), default (0) means no grant
	-- input
	signal bank_combiner_addr_raw_nxt		: small_logic_onehot_t; -- from match_encoder_addr_raw_nxt_comb
	signal bank_combiner_addr_raw			: small_logic_onehot_t; -- from match_encoder_addr_raw, which shoud be stale value before set. only updated after POPING or SEARCH.
	signal bank_combiner_addr0				: small_logic_binary_t; -- from match_encoder_addr0_comb
	signal bank_combiner_flag				: std_logic_vector(N_SMALL_LOGIC-1 downto 0); -- from match_encoder_flag_comb
	signal bank_combiner_count				: small_logic_binary_cnt_t; -- from match_encoder_count_comb + 1 bit
	-- output
	signal bank_combiner_addr_raw_comb		: small_logic_onehot_t; -- feed to match_encoder_addr_raw
	signal bank_combiner_total_count_comb	: std_logic_vector(MAIN_CAM_ADDR_WIDTH downto 0); -- summed of all banks + 1 (0 to 512!)
	
	-- avmm (csr)
	type csr_t is record
		-- word 0 
		go							: std_logic; -- (RW)
		soft_reset					: std_logic; -- (RW)
		-- word 1
		expected_latency			: std_logic_vector(15 downto 0); -- (RW)
		-- word 2
		fill_level					: std_logic_vector(31 downto 0); -- (RO)
		-- word 3
		overwrite_cnt				: std_logic_vector(31 downto 0); -- (RO)
		overwrite_cnt_rst			: std_logic; -- (WO)
		overwrite_cnt_rst_done		: std_logic; -- (internal)
	end record; 
	signal csr						: csr_t;

	-- streaming input deassembly
	constant HIT_TYPE1_DATA_WIDTH			: natural := asi_hit_type1_data'length;
	constant HIT_TYPE1_TS8N_WIDTH			: natural := TCC8N_HI-TCC8N_LO+1;
	constant SK_WIDTH						: natural := SK_RANGE_HI-SK_RANGE_LO+1;
	signal in_payload_valid					: std_logic;
	signal in_hit_side						: std_logic_vector(HIT_TYPE1_DATA_WIDTH-1 downto 0);
	signal in_hit_ts8n						: std_logic_vector(HIT_TYPE1_TS8N_WIDTH-1 downto 0);
	signal in_hit_sk						: std_logic_vector(SK_WIDTH-1 downto 0); -- search key
	
	-- push engine
	type push_state_t is (ERASE, WRITE_AND_CHECK);
	signal push_state					: push_state_t;
	signal write_pointer				: unsigned(SRAM_ADDR_WIDTH-1 downto 0);
	signal push_erase_req				: std_logic;
	signal push_write_req				: std_logic;
	signal push_erase_grant				: std_logic;
	signal push_write_grant				: std_logic;
	signal push_write_grant_reg			: std_logic;
	signal pop_current_sk				: std_logic_vector(MAIN_CAM_DATA_WIDTH-1 downto 0);
	
	-- pop descriptor generator
	signal expected_latency_48b			: std_logic_vector(47 downto 0);
	signal read_time_ptr				: unsigned(47 downto 0);
	
	-- pop engine
	type pop_engine_state_t is (IDLE, SEARCH, EVAL, POPING, RESET, FLUSHING, FLUSHING_RST);
	signal pop_engine_state							: pop_engine_state_t;
	signal pop_pipeline_start						: std_logic;
	signal pop_erase_req							: std_logic;
	signal pop_erase_grant							: std_logic;
	signal pop_hit_valid_comb						: std_logic;
	signal pop_hit_valid							: std_logic;
	signal pop_cam_match_addr						: std_logic_vector(MAIN_CAM_ADDR_WIDTH-1 downto 0);
	signal pop_hits_count							: unsigned(MAIN_CAM_ADDR_WIDTH downto 0); -- + 1 bit
	signal pop_engine_match_exist					: std_logic;
	signal pop_search_wait_cnt						: unsigned(2 downto 0);
	-- pop engine (flushing)
	constant POP_FLUSH_CAM_DATA_MAX					: unsigned(MAIN_CAM_DATA_WIDTH-1 downto 0) := (others => '1'); -- for flushing
	constant POP_FLUSH_CAM_ADDR_MAX					: unsigned(MAIN_CAM_ADDR_WIDTH-1 downto 0) := (others => '1'); -- for flushing
	signal pop_flush_req							: std_logic;
	signal pop_flush_grant							: std_logic;
	signal pop_flush_ram_done						: std_logic;
	signal pop_flush_cam_done						: std_logic;
	signal flush_ram_wraddr							: unsigned(SRAM_ADDR_WIDTH-1 downto 0);
	signal flush_cam_wrdata							: unsigned(MAIN_CAM_DATA_WIDTH-1 downto 0);
	signal flush_cam_wraddr							: unsigned(MAIN_CAM_ADDR_WIDTH-1 downto 0);
	
	-- streaming output assembly
	signal subheader_gen_done				: std_logic;
	
	
	-- cam and ram arbiter
	signal decision,decision_reg			: natural range 0 to 4;
	signal req								: std_logic_vector(3 downto 0);
	
	-- memory out cleanup
	constant SK_BITS						: natural := SK_RANGE_HI-SK_RANGE_LO+1;
	signal addr_occupied					: std_logic;
	signal cam_erase_ts8n					: std_logic_vector(HIT_TYPE1_TS8N_WIDTH-1 downto 0);
	signal cam_erase_data					: std_logic_vector(SK_BITS-1 downto 0);
	signal hit_pop_data_comb				: std_logic_vector(HIT_TYPE1_DATA_WIDTH-1 downto 0);
	signal side_ram_dout_valid_comb			: std_logic;
	signal side_ram_dout_valid				: std_logic;
	
	-- fill level meter
	type debug_msg_t is record
		push_cnt					: unsigned(47 downto 0);
		pop_cnt						: unsigned(47 downto 0);
		overwrite_cnt				: unsigned(47 downto 0);
		cam_clean					: std_logic;
		cache_miss_cnt				: unsigned(47 downto 0);
	end record;
	signal debug_msg				: debug_msg_t;
	signal debug_msg2				: debug_msg_t;
	signal fill_level_tmp					: unsigned (47 downto 0);
	signal ow_cnt_tmp						: unsigned(31 downto 0);
	
	-- run control management 
	type run_state_t is (IDLE, RUN_PREPARE, SYNC, RUNNING, TERMINATING, LINK_TEST, SYNC_TEST, RESET, OUT_OF_DAQ, ERROR);
	signal run_state_cmd					: run_state_t;
	signal run_state_cmd_d1					: run_state_t;
	signal endofrun_seen					: std_logic;
	signal gts_end_of_run					: unsigned(47 downto 0);
	signal run_mgmt_flush_memory_start		: std_logic;
	signal run_mgmt_flush_memory_done		: std_logic;
	signal run_mgmt_flushed					: std_logic;
	
begin

	main_cam : entity work.cam_mem_a5
	-- primitive cam construction
	generic map(
		CAM_SIZE 	=> MAIN_CAM_SIZE,
		CAM_WIDTH 	=> MAIN_CAM_DATA_WIDTH,
		WR_ADDR_WIDTH	=> MAIN_CAM_ADDR_WIDTH,
		RAM_TYPE 	=> "Simple Dual-Port RAM") -- currently only simple dp ram is supported
	port map(
		-- Modifying control port ("Erase-Mode": erase=1, write=X; "Write-Mode": erase=0, write=1; "Idle-Mode": erase=0, write=0)
		i_erase_en		=> cam_erase_en, -- erase has higher priority than write
		i_wr_en			=> cam_wr_en,
		-- Write port
		i_wr_data		=> cam_wr_data,
		i_wr_addr		=> cam_wr_addr,
		-- Loop up port 
		i_cmp_din		=> cam_cmp_din,
		o_match_addr	=> cam_match_addr_oh,
		-- clock and reset interface
		i_rst			=> i_rst,
		i_clk			=> i_clk
	);
	
	side_ram : entity work.alt_simple_dpram
	-- Side ram stores the hit type 1 and occupancy flag with the same address as CAM
	-- used to indicate occupancy when write attemp is seen 
	-- RDW: "old data"
	generic map(
		DATA_WIDTH	=> SRAM_DATA_WIDTH,
		ADDR_WIDTH	=> SRAM_ADDR_WIDTH)
	port map(
		-- read port
		raddr	=> to_integer(unsigned(side_ram_raddr)),
		q		=> side_ram_dout,
		-- write port
		we		=> side_ram_we,
		waddr	=> to_integer(unsigned(side_ram_waddr)),
		data	=> side_ram_din,
		-- clock interface
		clk		=> i_clk
	);
	
	pop_cmd_fifo : scfifo_w8d32 PORT MAP (
		-- read port
		rdreq	=> pop_cmd_fifo_rdack,
		q		=> pop_cmd_fifo_dout,
		-- write port
		wrreq	=> pop_cmd_fifo_wrreq,
		data	=> pop_cmd_fifo_din,
		-- status 
		empty	=> pop_cmd_fifo_empty,
		full	=> pop_cmd_fifo_full,
		usedw	=> pop_cmd_fifo_usedw,
		-- clock and reset interface
		sclr	=> pop_cmd_fifo_sclr,
		clock	=> i_clk
	);
	
	deassembly_fifo : scfifo_w40d256 PORT MAP (
		-- read port 
		rdreq	=> deassembly_fifo_rdack,
		q	 	=> deassembly_fifo_dout,
		-- write port
		wrreq	=> deassembly_fifo_wrreq,
		data	=> deassembly_fifo_din,
		-- status
		empty	=> deassembly_fifo_empty,
		full	=> deassembly_fifo_full,
		usedw	=> deassembly_fifo_usedw,
		-- clock and reset interface
		sclr	=> deassembly_fifo_sclr,
		clock	=> i_clk
	);

	proc_gts_counter : process (i_clk)
	-- counter of the global timestamp on the FPGA
		-- needs to be 48 bit at 125 MHz
	begin
		if rising_edge(i_clk) then
			if (gts_counter_rst = '1') then 
				 -- reset counter
				gts_8n		<= (others => '0');
			else
				-- begin counter
				gts_8n		<= gts_8n + to_unsigned(1,gts_8n'length);
			end if;
		end if;
	end process;
	
	
	gen_addr_enc_logic : for i in 0 to N_SMALL_LOGIC-1 generate 
	-- there are two read heads, one in lsb(addr1), one in msb (addr0).
	-- for best timing: reg in, comb out. user must latch the comb before processing it. 
		e_enc_logic	: entity work.addr_enc_logic_small
		generic map(
			CAM_SIZE 		=> SMALL_LOGIC_CAM_SIZE, -- one-hot code length
			CAM_ADDR_BITS 	=> SMALL_LOGIC_ADDR_W -- binary code length
		)
		port map(
			i_cam_address_onehot		=> match_encoder_addr_raw(i), -- onehot in 
			o_cam_address_onehot_next	=> match_encoder_addr_raw_nxt_comb(i), -- onehot out
			o_cam_address_binary		=> open, -- binary out
			o_cam_address_binary_lsb	=> match_encoder_addr0_comb(i), -- binary out
			o_cam_match_flag			=> match_encoder_flag_comb(i), -- flags array out
			o_cam_match_count			=> match_encoder_count_comb(i) -- binary count out
		);
	end generate gen_addr_enc_logic;
	
	proc_avmm_csr : process (i_rst, i_clk)
	-- avalon memory-mapped interface for accessing the control and status registers
	-- address map:
	-- 		0: control and status
	-- 		1: set read pointer latency 
	--		2: fill level
	--		3: overwrite count
	begin
		if (i_rst = '1') then 
			csr.go					<= '1'; -- default is "allowed to go"
			csr.soft_reset			<= '0';
			csr.fill_level			<= (others => '0');
			csr.expected_latency	<= std_logic_vector(to_unsigned(2000,csr.expected_latency'length)); -- this is the total latency of read pointer time respect to current gts
			csr.overwrite_cnt		<= (others => '0');
			csr.overwrite_cnt_rst	<= '1';
			avs_csr_waitrequest		<= '1';
		elsif (rising_edge(i_clk)) then 
			-- default
			avs_csr_readdata		<= (others => '0');
			-- host read local
			if (avs_csr_read = '1') then 
				avs_csr_waitrequest		<= '0';
				case to_integer(unsigned(avs_csr_address)) is 
					when 0 =>
						avs_csr_readdata(0)					<= csr.go;
						avs_csr_readdata(1)					<= csr.soft_reset;
					when 1 =>
						avs_csr_readdata(15 downto 0)		<= csr.expected_latency;
					when 2 =>
						avs_csr_readdata(31 downto 0)		<= csr.fill_level;
					when 3 =>
						avs_csr_readdata(31 downto 0)		<= csr.overwrite_cnt;
					-- below is for debug only
					when 4 =>
						avs_csr_readdata(31 downto 0)		<= std_logic_vector(debug_msg2.push_cnt)(31 downto 0);
					when 5 =>
						avs_csr_readdata(31 downto 0)		<= std_logic_vector(debug_msg2.pop_cnt)(31 downto 0);
					when 6 =>
						avs_csr_readdata(31 downto 0)		<= std_logic_vector(debug_msg2.overwrite_cnt)(31 downto 0);
					when 7 =>
						avs_csr_readdata(31 downto 0)		<= std_logic_vector(debug_msg2.cache_miss_cnt)(31 downto 0);
					when others =>
				end case;
			-- host write local
			elsif (avs_csr_write = '1') then 
				avs_csr_waitrequest		<= '0';
				case to_integer(unsigned(avs_csr_address)) is 
					when 0 =>
						csr.go						<= avs_csr_writedata(0);
						csr.soft_reset				<= avs_csr_writedata(1);
					when 1 =>
						csr.expected_latency		<= avs_csr_writedata(15 downto 0);
					when 2 => 
						-- do nothing
					when 3 => -- write side effect: reset overwrite counter 
						csr.overwrite_cnt_rst		<= avs_csr_writedata(0);
					when others =>
				end case;
			else -- idle, update the csr registers
				avs_csr_waitrequest		<= '1';
				-- soft reset
				csr.soft_reset			<= '0'; -- TODO: implement this
				-- fill level and ow cnt
				fill_level_tmp				<= to_unsigned((to_integer(debug_msg2.push_cnt) - to_integer(debug_msg2.pop_cnt) - to_integer(debug_msg2.overwrite_cnt) + to_integer(debug_msg2.cache_miss_cnt)),fill_level_tmp'length);	
				csr.fill_level				<= std_logic_vector(fill_level_tmp(31 downto 0)); -- direct mapped
				csr.overwrite_cnt			<= std_logic_vector(debug_msg2.overwrite_cnt(31 downto 0) - ow_cnt_tmp); -- this only shows the incremental amount after csr reset
				if (csr.overwrite_cnt_rst_done = '1' and csr.overwrite_cnt_rst = '1') then -- ack the agent
					csr.overwrite_cnt_rst			<= '0';
				end if;
			end if;
		end if;
	end process;
	
	
	
	proc_avst_input_deassembly_comb : process (all)
	-- The deassembly digest the streaming input in combinational logic
	-- Support backpressure internally, so no backpressure fifo in the upstream needed in normal operation. (TODO: add a monitor to detect missing hits at its input)
	-- Functional Description:
	--		Ignore other channels and only take in (write to deassembly_fifo) selected channel.
	--		(interlacer enabled) In this mode, further restriction on the input hit timestamp ts8n(11:4) is made. ts modulo <# interleaving ways> has remainder equal to this IP's index will be accepted.
	begin		
		-- fifo write port
		-- triming input stream 
		deassembly_fifo_din(asi_hit_type1_data'high downto 0)		<= asi_hit_type1_data;
		deassembly_fifo_din(39)										<= '0';
		-- write with validation 
		-- default
		deassembly_fifo_wrreq				<= '0';
		if ( (run_state_cmd = RUNNING or (run_state_cmd = TERMINATING and endofrun_seen = '0')) and csr.go = '1') then -- only in running, input are fully allowed. in TERMINATING, take in new hits, which remains in the processor fifo 
			if (asi_hit_type1_valid = '1' and to_integer(unsigned(asi_hit_type1_data(TCC8N_INTERLEAVING_HI downto TCC8N_INTERLEAVING_LO))) = INTERLEAVING_INDEX) then 
			-- only takes the ts modulo interleaving_factor = interleaving_index
			-- (deprecated) since the data streams are merged in mts_processor, ignore data from other non-selected channel
				deassembly_fifo_wrreq			<= '1';
			end if;
		end if;
		-- fifo read port
		if (deassembly_fifo_empty /= '1') then -- always latch data when fifo not empty
			in_payload_valid	<= '1';
			in_hit_side		<= deassembly_fifo_dout(in_hit_side'high downto 0);
			in_hit_ts8n		<= deassembly_fifo_dout(TCC8N_HI downto TCC8N_LO);
			in_hit_sk		<= deassembly_fifo_dout(SK_RANGE_HI+TCC8N_LO downto SK_RANGE_LO+TCC8N_LO);
		else -- if empty, the showahead data is not valid
			in_payload_valid				<= '0'; 
			in_hit_side						<= (others => '0');
			in_hit_ts8n						<= (others => '0');
			in_hit_sk						<= (others => '0');
		end if;
		-- fifo rdack 
		if (push_write_grant = '1') then -- only ack this fifo, show next word, if write is granted. derive in the same cycle, new word immediately.
			deassembly_fifo_rdack		<= '1';
		else
			deassembly_fifo_rdack		<= '0';
		end if;
		-- avst ready
		if (deassembly_fifo_full /= '1') then -- TODO: refine it. For now, this ready signal is for sanity check only.
			asi_hit_type1_ready			<= '1';
		else
			asi_hit_type1_ready			<= '0';
		end if;
		
	end process;
	
	
	
	
	proc_push_engine : process (i_clk,i_rst)
	-- The push engine has two states: write_and_check and erase
	-- during write_and_lookup, write cam for the input hit type 1 (only ts[11:4]) and write side ram for hit type 1 (all), 
	-- and check (read) the side ram for its old data and occupancy flag. (since RDW is "old-data", it is valid)
	-- If occupancy is high, jump to erase state. Else, stay in write_and_lookup state. 
	-- Throughput: 1 cycle (no overwrite), 2 cycle (overwrite). 
	begin
		if (i_rst = '1') then 
--			debug_msg.push_cnt			<= (others => '0');
--			debug_msg.overwrite_cnt		<= (others => '0');
		elsif (rising_edge(i_clk)) then 
			-- latch the grant comb for switching to erase
			push_write_grant_reg	<= push_write_grant;
			case push_state is		
				when ERASE => -- erase
					if (push_erase_grant = '1') then -- this must be granted (highest priority after push_write)!
--						debug_msg.overwrite_cnt		<= debug_msg.overwrite_cnt + 1;
					end if;
				when WRITE_AND_CHECK => -- write 
					if (push_write_grant = '1') then -- incr ptr and cnt
						write_pointer				<= write_pointer + 1; 
--						debug_msg.push_cnt			<= debug_msg.push_cnt + 1;
					elsif (pop_flush_cam_done = '1') then -- reset when pop has flushed
						-- reset in this push state, not in ERASE, because push should in this state while flush has been executed
--						debug_msg.push_cnt			<= (others => '0');
--						debug_msg.overwrite_cnt		<= (others => '0');
						write_pointer				<= (others => '0');
					else -- maybe pop erase in action
						-- idle
					end if;
				when others =>
			end case;
		end if;
	end process;
	
	
	proc_push_engine_comb : process (all)
	begin
		-- derive the state in comb 
		if (side_ram_dout_valid = '1' and addr_occupied = '1' and push_write_grant_reg = '1') then -- only when last state has written a hit
			push_state		<= ERASE;
		else
			push_state		<= WRITE_AND_CHECK;
		end if;
		-- NOTE: although the case flag is comb, valid and occupied are fresh reg out, minimizing the glitch.
		-- default
		push_erase_req 		<= '0';
		push_write_req 		<= '0';
		case push_state is		
			when ERASE => -- erase
				push_erase_req 		<= '1'; -- the grant is derived in comb
			when WRITE_AND_CHECK => -- write (write through while no show stopper (addr_occupied) is seen) 
				if (in_payload_valid = '1') then -- always ask for grant access when new data 
					push_write_req 		<= '1';
				else
					push_write_req 		<= '0';
				end if;
			when others =>
		end case;
	end process;
	

	
	proc_pop_descriptor_generator : process (i_clk,i_rst)
	-- the pop descriptor generator will generate the 8 bit command ts[11:4] for the search key, 
	-- which is processed by the pop engine to pop out all match hits.
	begin
		if (i_rst = '1') then 
			pop_cmd_fifo_wrreq		<= '0';
			pop_cmd_fifo_din		<= (others => '0');
		elsif (rising_edge(i_clk)) then 
			-- default
			pop_cmd_fifo_wrreq		<= '0';
			pop_cmd_fifo_din		<= (others => '0');
			if (run_state_cmd = RUNNING) then -- normal pop
				if (read_time_ptr(3 downto 0) = "0000" and to_integer(unsigned(read_time_ptr(TCC8N_INTERLEAVING_BITS+3 downto 4))) = INTERLEAVING_INDEX) then -- generate read command every 16 cycle 
					-- only generate when interleaving condition is met (see input deassembly)
					pop_cmd_fifo_wrreq		<= '1';
					pop_cmd_fifo_din		<= std_logic_vector(read_time_ptr(SK_RANGE_HI downto SK_RANGE_LO)); -- NOTE: search key also controls the subheader gen cmd
				end if;
			elsif (run_state_cmd = TERMINATING) then -- end of run pop
				if (read_time_ptr < gts_end_of_run) then -- continue to generate until read the end of run ts marker, at this point all hits should be popped out
					if (read_time_ptr(3 downto 0) = "0000" and to_integer(unsigned(read_time_ptr(TCC8N_INTERLEAVING_BITS+3 downto 4))) = INTERLEAVING_INDEX ) then -- generate read command every 16 cycle 
						pop_cmd_fifo_wrreq		<= '1';
						pop_cmd_fifo_din		<= std_logic_vector(read_time_ptr(SK_RANGE_HI downto SK_RANGE_LO)); -- NOTE: search key also controls the subheader gen cmd
					end if;
				end if;
			end if;
			
		end if;
	end process;
	
	proc_pop_descriptor_generator_comb : process (all)
	begin
		expected_latency_48b(csr.expected_latency'high downto 0)								<= csr.expected_latency; 
		expected_latency_48b(expected_latency_48b'high downto csr.expected_latency'length)		<= (others => '0'); -- pads 0 in msb
        if (to_integer(gts_8n) > to_integer(unsigned(expected_latency_48b))) then 
            read_time_ptr		<= gts_8n - unsigned(expected_latency_48b);
        else 
            read_time_ptr       <= (0 => '1', others => '0'); -- avoid generate descriptor when run has just started
        end if;
	end process;
	
	
	
	
	proc_pop_engine : process (i_clk,i_rst)
	-- The pop engine reads the pop command fifo for the descriptor/command of search key for a new sub-header (ts[11:4])
	-- The work flow is search for 1 cycle, and evaluate the result for 1 empty cycle, and decode the hits (1 per cycle) until last match is consumed.
	-- During POPING, it communicates with arbiter for requesting the control of erasing cam, write and read the side ram. 
	-- 		Details: Erasing the cam for the matched hit address. Write zeros on side ram. Read side ram for the hit type 1. 
	-- 
	begin
		if (i_rst = '1') then 
			pop_engine_state			<= IDLE; --- TODO: think about it
			pop_flush_ram_done			<= '0';
			pop_flush_cam_done			<= '0';
--			debug_msg.pop_cnt			<= (others => '0');
			run_mgmt_flush_memory_done	<= '0';
			pop_pipeline_start			<= '0';
			pop_search_wait_cnt			<= (others => '0');
		elsif (rising_edge(i_clk)) then 
			-- pop command executor
			case pop_engine_state is 
				-- ============= IDLE ==============
				when IDLE =>
					if (pop_cmd_fifo_empty /= '1') then -- if not empty, read the showahead word
						pop_current_sk		<= pop_cmd_fifo_dout; -- latch pop search key for this round
						pop_engine_state	<= SEARCH;
					end if;
					-- inter-fsm communication (with run state mgmt)
					if (run_mgmt_flush_memory_start = '1' and run_mgmt_flush_memory_done = '0') then
						pop_engine_state	<= FLUSHING; -- start the sub-routine
					elsif (run_mgmt_flush_memory_start = '1' and run_mgmt_flush_memory_done = '1') then
						-- wait for host to ack
					elsif (run_mgmt_flush_memory_start = '0' and run_mgmt_flush_memory_done = '1') then
						run_mgmt_flush_memory_done		<= '0'; -- ack the host
					end if;
				-- ============= OP POP ==============
				when SEARCH =>	-- pop_current_sk was connected to the read port of cam
					if (to_integer(pop_search_wait_cnt)	< 5) then -- wait for 4 more cycle until the cam lookup result is available (guard band to prevent push to the same address)
						pop_search_wait_cnt		<= pop_search_wait_cnt + 1;
					else -- latch at result 2nd cycle and exit
						match_encoder_addr_raw		<= cam_match_addr_oh_2d_comb; -- latch the search result and post for encoding
						pop_engine_match_exist		<= or_reduce(cam_match_addr_oh); -- calculate large or'd match flag
						pop_engine_state			<= EVAL;
						pop_search_wait_cnt			<= (others => '0');
					end if;
				when EVAL => -- enc done, reg the first binary address (this is rather an intermediate idle state)
					if (pop_engine_match_exist = '1') then -- there is match, this large OR is the only comb appended after enc logic
					-- go to poping anyways 
						pop_pipeline_start		<= '1'; -- set for each pop command 
						pop_engine_state		<= POPING;
--	
					else -- no match
						pop_cmd_fifo_rdack		<= '1'; -- ack the command is completed
						pop_engine_state		<= RESET;
					end if;
				when POPING => -- post addr0
					if (pop_hits_count = 0) then -- when in this state, there must be some hits
						pop_hits_count		<= unsigned(bank_combiner_total_count_comb); -- latch it
					end if;
				
				
					if (pop_erase_grant = '1') then
--						debug_msg.pop_cnt			<= debug_msg.pop_cnt + 1;	-- incr global counter
						match_encoder_addr_raw		<= bank_combiner_addr_raw_comb; -- post the next raw 
						if (to_integer(unsigned(bank_combiner_total_count_comb)) <= 1) then -- last hit in all ways, or no hit
							-- reset ...
							pop_engine_state			<= RESET;
							pop_cmd_fifo_rdack			<= '1'; -- ack the command is completed
						else -- pop in other ways
							pop_engine_state			<= EVAL;
						end if;
					else 
						-- idle
					
					end if;
			
				when RESET =>
					-- reset for this sub-header scope
					match_encoder_addr_raw		<= (others => (others => '0')); -- this reset the match encoder and subsequently bank combiner input
					pop_engine_state			<= IDLE;
					pop_cmd_fifo_rdack			<= '0'; -- this must only be high for 1 cycle!
					pop_pipeline_start			<= '0';
					pop_hits_count				<= (others => '0');
					--pop_engine_match_exist		<= '0'; -- for speed, no need to reset
				-- ============= OP FLUSHING ==============
				when FLUSHING =>
					-- flush ram
					if (pop_flush_ram_done = '0' and pop_flush_grant = '1') then -- grant should be true after write erase 
						flush_ram_wraddr			<= flush_ram_wraddr + 1;
					end if;
					if (to_integer(flush_ram_wraddr) = RING_BUFFER_N_ENTRY-1) then 
						pop_flush_ram_done				<= '1';
					end if;
					-- flush cam (2d flush, for each addr line, all data must be looped) 
					if (pop_flush_cam_done = '0' and pop_flush_grant = '1') then 
						if (flush_cam_wrdata = POP_FLUSH_CAM_DATA_MAX) then -- incr addr
							flush_cam_wraddr			<= flush_cam_wraddr + 1;
							if (flush_cam_wraddr = POP_FLUSH_CAM_ADDR_MAX) then -- exit condition
								pop_flush_cam_done			<= '1';
							end if;
						end if;
						flush_cam_wrdata			<= flush_cam_wrdata + 1; -- incr data
					end if;
					if (pop_flush_ram_done = '1' and pop_flush_cam_done = '1') then -- flushing is done, go back to idle
						pop_engine_state			<= FLUSHING_RST;
						run_mgmt_flush_memory_done	<= '1';
					end if;
				when FLUSHING_RST =>
					-- ===============================
					-- reset after flush
					-- flags
					pop_flush_ram_done				<= '0';
					pop_flush_cam_done				<= '0';
					-- pointer 
						-- write_pointer (by push engine)
					-- counters 
						-- push_cnt and erase_cnt (by push engine)
--					debug_msg.pop_cnt			<= (others => '0');
					pop_engine_state			<= IDLE;
					
				when others =>
			end case;
		end if;
	end process;
	
	
	proc_pop_engine_comb : process (all)
	begin
		-- default
		pop_erase_req			<= '0';
		pop_flush_req			<= '0';
		pop_hit_valid_comb		<= '0';
		cam_cmp_din				<= pop_current_sk; -- bug fixed! pop_cmd_fifo_dout; -- pop search key
		-- logic
		case pop_engine_state is 
			when EVAL =>
--				if (pop_pipeline_start = '1') then -- if already started; camd back from POPING state
--					pop_erase_req		<= '1';
--				end if;
			when POPING => -- ask for access if there is any hit
--if (to_integer(unsigned(bank_combiner_total_count_comb)) /= 0) then 
					pop_erase_req		<= '1'; -- always do this, otherwise if this depend on count, timing violation report.
				--else 
					--pop_erase_req		<= '0';
				--end if;
				-- if granted, the hit in the next cycle is valid for output (assembly)
				if (pop_erase_grant = '1') then
					pop_hit_valid_comb			<= '1';
				end if;
			when FLUSHING => 
				pop_flush_req		<= '1';
			when others =>
		end case;
		-- transformation cam_match_addr_oh to 2d
		for i in 0 to N_SMALL_LOGIC-1 loop
			cam_match_addr_oh_2d_comb(i)		<= cam_match_addr_oh((i+1)*SMALL_LOGIC_CAM_SIZE-1 downto i*SMALL_LOGIC_CAM_SIZE);
		end loop;
		
	end process;
	
	proc_bank_combiner : process (i_rst, i_clk)
	begin
		if (i_rst = '1') then 
		
		elsif (rising_edge(i_clk)) then
			bank_combiner_addr_raw_nxt		<= match_encoder_addr_raw_nxt_comb;
			bank_combiner_addr_raw			<= match_encoder_addr_raw; -- it just the input to match encoder 
			bank_combiner_addr0				<= match_encoder_addr0_comb;
			bank_combiner_flag				<= match_encoder_flag_comb;
			bank_combiner_count				<= match_encoder_count_comb;
		end if;
	
	end process;
	
	proc_bank_combiner_comb : process (all)
		-- arbitration scheme: fix-priority, lsb first
		-- match_encoder_flag(i) is req
		-- bank_combiner_grant_comb(i) is grant (internal use)
		-- derive comb based on the reg'd value
		-- INPUT: (reg)
--			bank_combiner_addr_raw_nxt(i)		
--			bank_combiner_addr_raw(i)			
--			bank_combiner_addr0(i)				
--			bank_combiner_flag(i)				
--			bank_combiner_count(i)			
		-- OUTPUT: (comb) used at the rising_edge of POPING
		--		bank_combiner_addr_raw_comb(i)
		--		pop_cam_match_addr -- !!! co-validate with pop_erase_grant
		--		bank_combiner_total_count_comb
		variable tot_count 		: unsigned(MAIN_CAM_ADDR_WIDTH downto 0);  -- + 1 bit
	begin
		-- default
		
		pop_cam_match_addr				<= (others => '0');
		tot_count						:= (others => '0');
		bank_combiner_grant_comb		<= (others => '0');
		-- arbiter
		for i in N_SMALL_LOGIC-1 downto 0 loop
			if (bank_combiner_flag(i) = '1') then
				bank_combiner_grant_comb		<= (i=>'1', others => '0');
			end if;
		end loop;
		-- binary addr (in pop cycles)
		for i in 0 to N_SMALL_LOGIC-1 loop
			if (bank_combiner_grant_comb(i) = '1') then -- grant this way
				pop_cam_match_addr			<= std_logic_vector(to_unsigned( to_integer(unsigned(bank_combiner_addr0(i))) + i*SMALL_LOGIC_CAM_SIZE , pop_cam_match_addr'length ));
				bank_combiner_addr_raw_comb(i)	<= bank_combiner_addr_raw_nxt(i); -- show next
			else
				bank_combiner_addr_raw_comb(i)	<= bank_combiner_addr_raw(i);
			end if;
		end loop;
		-- count (sum of all ways)
		for i in 0 to N_SMALL_LOGIC-1 loop -- incremental sum
--			tot_count				:= tot_count + unsigned(match_encoder_count(i)); -- wrong!!!! wrong!!!!
			tot_count				:= tot_count + unsigned(bank_combiner_count(i)); -- 9 bit + 7 bit
			--bank_combiner_total_count_comb		<= std_logic_vector(to_unsigned( to_integer(unsigned(bank_combiner_total_count_comb)) + to_integer(unsigned(match_encoder_count(i))) , bank_combiner_total_count_comb'length ));
		end loop;
		bank_combiner_total_count_comb		<= std_logic_vector(tot_count); -- 8 bit to 8 bit
		
	
	end process;
	
	
	
	proc_memory_arbiter_comb : process (all)
	-- Combinational memory arbiter for access contention of cam and side ram from push and pop engine. (priority is demostrated in code)
	-- Note: push write is a later stage than push erase, so push write must be cleared first, otherwise push erase will overflow the push write stage. 
	-- pop_erase and push_write is interleaving. 
	begin
		-- arbiter for cam write
		req			<= (0 => push_write_req, 1 => push_erase_req, 2 => pop_erase_req, 3 => pop_flush_req);
		-- default decision (nothing is granted)
		decision	<= 4; -- idle
		
		if (req(3) = '1') then -- flush should be always ok
			decision		<= 3;
		elsif (req(1) = '1') then -- always grant erase even in pop phase (appear in the first cycle as last push write is just granted)
			decision		<= 1;
		elsif (pop_engine_state /= IDLE) then -- only grant pop during pop phase (ignore new push), but let the last erase pass (otherwise multiple data at the same location in cam)
			if (req(2) = '1') then
				decision		<= 2;
			end if;
		elsif (req(0) = '1') then -- grant push write
			decision		<= 0;
		end if;
		
		
		
--		case (decision_reg) is 
--			when 0 => -- last selected is push write, must grant push erase (data dependency)
--			
--				if (req(0) = '1') then
--					decision	<= 0;
--				end if;
--				if (req(2) = '1') then -- refine: pop is ahead of new push
--					decision	<= 2;
--				end if;
--				if (req(3) = '1') then
--					decision	<= 3;
--				end if;
--				if (req(1) = '1') then
--					decision	<= 1;
--				end if;
--			when 1 => -- bang-bang between push erase and pop erase, can grant pop erase
--				if (req(1) = '1') then
--					decision	<= 1;
--				end if;
--				if (req(0) = '1') then
--					decision	<= 0;
--				end if;
--				if (req(2) = '1') then
--					decision	<= 2;
--				end if;
--				if (req(3) = '1') then
--					decision	<= 3;
--				end if;
--			when 2 => -- always grant pop erase (lock), otherwise start write
--				if (req(1) = '1') then
--					decision	<= 1;
--				end if;
--				if (req(0) = '1') then
--					decision	<= 0;
--				end if;
--				if (req(2) = '1') then
--					decision	<= 2;
--				end if;
--				if (req(3) = '1') then
--					decision	<= 3;
--				end if;
--			when 3 => -- flushing has highest priority
--				if (req(1) = '1') then
--					decision	<= 1;
--				end if;
--				if (req(0) = '1') then
--					decision	<= 0;
--				end if;
--				if (req(2) = '1') then
--					decision	<= 2;
--				end if;
--				if (req(3) = '1') then
--					decision	<= 3;
--				end if;
--			when others => -- idle decision, grant flush > pop erase (lock) > start write
--				if (req(1) = '1') then
--					decision	<= 1;
--				end if;
--				if (req(0) = '1') then
--					decision	<= 0;
--				end if;
--				if (req(2) = '1') then
--					decision	<= 2;
--				end if;
--				if (req(3) = '1') then
--					decision	<= 3;
--				end if;
--		end case;
		
		-- mux: grant the access based on the decision of the arbiter 
		case (decision) is 
			when 0 => -- grant push write
				push_write_grant	<= '1';
				push_erase_grant	<= '0';
				pop_erase_grant		<= '0';
				pop_flush_grant		<= '0';
				-- put main cam into "Write-Mode"
				cam_wr_en		<= '1'; 
				cam_erase_en	<= '0';
				cam_wr_addr		<= std_logic_vector(write_pointer);
				cam_wr_data		<= in_hit_sk; -- search key
				-- write side-ram, of the current side data
				side_ram_we		<= '1';
				side_ram_waddr	<= std_logic_vector(write_pointer);
				side_ram_din	<= '1' & in_hit_side;
				-- read side-ram, for occupancy of the next addr, NOTE: RDW must be "old-data"
				side_ram_raddr	<= std_logic_vector(write_pointer);
				side_ram_dout_valid_comb		<= '1';
				
				
			when 1 => -- grant push erase
				push_write_grant	<= '0';
				push_erase_grant	<= '1';
				pop_erase_grant		<= '0';
				pop_flush_grant		<= '0';
				-- put main cam into "Erase-Mode"
				cam_wr_en		<= '0';
				cam_erase_en	<= '1';
				cam_wr_addr		<= std_logic_vector(write_pointer-1); -- wr_ptr has been incr'd in write state, so we go back 1 step
				cam_wr_data		<= cam_erase_data; -- erase the search key that is occupying this location 
				-- do not write side-ram, since it has been written for new data in push_write
				side_ram_we		<= '0';
				side_ram_waddr	<= (others => '0');
				side_ram_din	<= (others => '0'); -- clear the occupancy flag
				-- read side ram, do nothing 
				side_ram_raddr	<= (others => '0');
				side_ram_dout_valid_comb		<= '0';
				
				
			when 2 =>  -- grant pop erase
				push_write_grant	<= '0';
				push_erase_grant	<= '0';
				pop_erase_grant		<= '1';
				pop_flush_grant		<= '0';
				-- put the main cam into "Erase-Mode"
				cam_wr_en		<= '0';
				cam_erase_en	<= '1';
				cam_wr_addr		<= pop_cam_match_addr; -- erase this consumed hit in cam
				cam_wr_data		<= pop_current_sk; -- search key for this sub-header -- bug fixed
				-- write side-ram 
				side_ram_we		<= '1';
				side_ram_waddr	<= pop_cam_match_addr; -- erase this consumed hit in side ram
				side_ram_din	<= (others => '0'); -- note: clear the occupancy flag is enough
				-- read side ram (NOTE: RDW must be old-data)
				side_ram_raddr	<= pop_cam_match_addr;
				side_ram_dout_valid_comb		<= '1';
			
			when 3 => -- flushing the cam and ram 
				push_write_grant	<= '0';
				push_erase_grant	<= '0';
				pop_erase_grant		<= '0';
				pop_flush_grant		<= '1';
				-- put main cam into "Erase-Mode"
				cam_wr_en			<= '0';
				cam_erase_en		<= '1';
				cam_wr_addr			<= std_logic_vector(flush_cam_wraddr);
				cam_wr_data			<= std_logic_vector(flush_cam_wrdata);
				-- write side ram with empty
				side_ram_we			<= '1';
				side_ram_waddr		<= std_logic_vector(flush_ram_wraddr);
				side_ram_din		<= (others => '0');
				-- read side ram, do nothing 
				side_ram_raddr	<= (others => '0');
				side_ram_dout_valid_comb		<= '0';
				
			when others => -- idle
				push_write_grant	<= '0';
				push_erase_grant	<= '0';
				pop_erase_grant		<= '0';
				pop_flush_grant		<= '0';
				-- put the main cam into idle
				cam_wr_en		<= '0';
				cam_erase_en	<= '0';
				cam_wr_addr		<= (others => '0'); -- erase this consumed hit in cam 
				cam_wr_data		<= (others => '0'); -- search key for this sub-header
				-- do not write side-ram
				side_ram_we		<= '0';
				side_ram_waddr	<= (others => '0'); -- erase this consumed hit in side ram
				side_ram_din	<= (others => '0'); -- note: clear the occupancy flag is enough
				-- do not read side ram 
				side_ram_raddr	<= (others => '0');
				side_ram_dout_valid_comb		<= '0';
		end case;
	end process;
	
	proc_debug_counter : process (i_clk)
	begin
		if (rising_edge(i_clk)) then 
			case decision_reg is 
				when 0 => -- push write
					debug_msg2.push_cnt			<= debug_msg2.push_cnt + 1;
				when 1 => -- push erase
					debug_msg2.overwrite_cnt	<= debug_msg2.overwrite_cnt + 1;
				when 2 => -- pop erase
					debug_msg2.pop_cnt			<= debug_msg2.pop_cnt + 1;
				when 3 => -- flushing
					debug_msg2.push_cnt			<= (others => '0');
					debug_msg2.overwrite_cnt	<= (others => '0');
					debug_msg2.pop_cnt			<= (others => '0');
					debug_msg2.cache_miss_cnt	<= (others => '0');
				when others =>
			end case;
			
			if (pop_hit_valid = '1' and side_ram_dout(side_ram_dout'high) = '0') then -- cache miss
				debug_msg2.cache_miss_cnt		<= debug_msg2.cache_miss_cnt + 1;
			end if;
		end if;
	
	end process;
	
	proc_memory_arbiter : process (i_clk, i_rst)
	-- reg part of the memory arbiter, simply latch last decision for switching priority
	begin
		if (i_rst = '1') then 
			decision_reg		<= 4; -- reset the arbiter to idle
		elsif (rising_edge(i_clk)) then 
			decision_reg		<= decision; -- latch the decision to do bang-bang
		end if;
	end process;
	
	
	proc_memory_out_cleanup_comb : process (all)
	-- wire connected to the output port of ram, with derived valid signal. 
	begin
		-- flag signal for push_erase
		addr_occupied		<= side_ram_dout(side_ram_dout'high); 
		-- reg only valid after push_write, for the use in push_erase
		cam_erase_ts8n		<= side_ram_dout(TCC8N_HI downto TCC8N_LO); -- side_ram_dout is valid after push_write, in the immediate next cycle, we can directly use this reg
		cam_erase_data		<= cam_erase_ts8n(SK_RANGE_HI downto SK_RANGE_LO); -- feed the cam for erase in the next cycle
		-- assemble output hit
		hit_pop_data_comb	<= side_ram_dout(side_ram_dout'high-1 downto 0); -- simply strip the msb to get the hit type 1
	end process;
	
	proc_memory_out_cleanup : process (i_clk, i_rst)
	-- latch the data valid from side ram
	begin
		if (i_rst = '1') then 
			
		elsif (rising_edge(i_clk)) then -- 
			side_ram_dout_valid		<= side_ram_dout_valid_comb;
		end if;
	end process;
	
	
	proc_avst_output_assembly : process (i_clk,i_rst)
	-- The streaming output assembly generates hit type 2 data replying on the pop engine state and other scattered data
	-- This assembly also support packetized transmission with sop/eop.
	begin
		if (i_rst = '1') then 
			-- it is handled by pop engine -> RESET
			subheader_gen_done			<= '0';
			aso_hit_type2_valid			<= '0';
			aso_hit_type2_data			<= (others => '0');
			aso_hit_type2_startofpacket	<= '0';
			aso_hit_type2_endofpacket	<= '0';
		elsif (rising_edge(i_clk)) then 
			pop_hit_valid		<= pop_hit_valid_comb; -- latched so, high in the cycle after POPING, come together with the cam/ram's q
			-- 1) generate sub header (w/ sop or sop+eop)
			if ((pop_pipeline_start = '1' or pop_cmd_fifo_rdack = '1') and subheader_gen_done = '0') then 
                -- 1) generate only one sub-header at POPING or 2) generate sub-header for empty subframe
				-- Streaming
				aso_hit_type2_valid		<= '1';
				-- assemble sub-header
				aso_hit_type2_data(35 downto 32)	<= "0001"; -- byte is k
				aso_hit_type2_data(31 downto 24)	<= pop_current_sk; -- this is ts[11:4] in the scope of this subheader
				aso_hit_type2_data(23 downto 16)	<= (others => '0'); -- free space, TBD
				aso_hit_type2_data(15 downto 8)		<= std_logic_vector(to_unsigned(to_integer(unsigned(bank_combiner_total_count_comb)),8));
				-- fix it '0' & bank_combiner_total_count_comb(6 downto 0); -- truncate (512 entry) 9 bit -> 8 bit. or expand (64 entry) 6 bit -> 8 bit. 
				aso_hit_type2_data(7 downto 0)		<= K237; -- identifier for sub-header (ref: specbook)
				-- misc.
				subheader_gen_done 					<= '1'; -- marks sop to avoid repetitive generation 
				-- channel 
				aso_hit_type2_channel				<= std_logic_vector(to_unsigned(INTERLEAVING_INDEX,aso_hit_type2_channel'length));
				-- packet
				aso_hit_type2_startofpacket			<= '1';
				if (to_integer(unsigned(bank_combiner_total_count_comb)) = 0) then -- gen eop for no hit scenario, or last hit
					aso_hit_type2_endofpacket			<= '1';
				else 
					aso_hit_type2_endofpacket			<= '0';
				end if;
            -- 2) generate hits (w/ eop)
			elsif (pop_pipeline_start = '1' and subheader_gen_done = '1') then -- at EVAL, 1 cycle after POPING, hits should be available
				if (pop_hit_valid = '1' and decision_reg = 2) then -- this co-validate this hit is not from push_write (0) . push can win arb in bubbles of pop
					-- Streaming
					aso_hit_type2_valid		<= '1';
					-- assemble hit type 2
					aso_hit_type2_data(35 downto 32)	<= (others => '0'); -- byte is k
					aso_hit_type2_data(31 downto 28)	<= hit_pop_data_comb(TCC8N_LO+3 downto TCC8N_LO);
					aso_hit_type2_data(27 downto 22)	<= "00" & hit_pop_data_comb(ASIC_HI downto ASIC_LO);
					aso_hit_type2_data(21 downto 17)	<= hit_pop_data_comb(CHANNEL_HI downto CHANNEL_LO);
					aso_hit_type2_data(16 downto 9)		<= hit_pop_data_comb(TCC1n6_HI downto TFINE_LO); -- tcc1.6(1.6ns) & tfine(50ps) = ts50p
					aso_hit_type2_data(8 downto 0)		<= hit_pop_data_comb(ET1n6_HI downto ET1n6_LO);
					-- channel
					aso_hit_type2_channel				<= std_logic_vector(to_unsigned(INTERLEAVING_INDEX,aso_hit_type2_channel'length)); -- re-assemble the channel
						-- equivalent alternative: hit_pop_data_comb(ASIC_HI downto ASIC_LO); 
					-- packet 
					aso_hit_type2_startofpacket			<= '0';
					if (to_integer(unsigned(bank_combiner_total_count_comb)) <= 1) then -- gen eop for last hit 
						aso_hit_type2_endofpacket			<= '1';
					else
						aso_hit_type2_endofpacket			<= '0';
					end if;
				else -- idle
					aso_hit_type2_valid			<= '0';
					aso_hit_type2_data			<= (others => '0');
					aso_hit_type2_startofpacket	<= '0';
					aso_hit_type2_endofpacket	<= '0';
				end if;
			elsif (pop_pipeline_start = '0') then 
				subheader_gen_done			<= '0';
				-- Streaming
				aso_hit_type2_valid			<= '0';
				aso_hit_type2_data			<= (others => '0');
				aso_hit_type2_startofpacket	<= '0';
				aso_hit_type2_endofpacket	<= '0';
			else -- idle
				aso_hit_type2_valid			<= '0';
				aso_hit_type2_data			<= (others => '0');
				aso_hit_type2_startofpacket	<= '0';
				aso_hit_type2_endofpacket	<= '0';
			end if;
	
		end if;
	end process;
	
	
	
	
	
	proc_fill_level_meter : process (i_clk,i_rst)
	-- fill level meter tracks the number of hits on the stack.
	-- the meter derive this by precisely counting the push and pop hits and over write.
	-- the dbg registers are always true, while the csr registers are inferred from it and can be sclr'd.
	-- 
	begin
		if (i_rst = '1') then 
			ow_cnt_tmp					<= (others => '0');
		elsif (rising_edge(i_clk)) then 
			-- sclr the csr overwrite_cnt
			if (csr.overwrite_cnt_rst = '1' and csr.overwrite_cnt_rst_done = '0') then -- latch the current value
				ow_cnt_tmp					<= debug_msg2.overwrite_cnt(31 downto 0);
				csr.overwrite_cnt_rst_done	<= '1';
			elsif (csr.overwrite_cnt_rst = '1' and csr.overwrite_cnt_rst_done = '1') then
				-- idle
				ow_cnt_tmp					<= ow_cnt_tmp;
			elsif (csr.overwrite_cnt_rst = '0' and csr.overwrite_cnt_rst_done = '1') then -- ack the host
				csr.overwrite_cnt_rst_done	<= '0';
			else
				ow_cnt_tmp					<= ow_cnt_tmp;
			end if;
			-- cam empty flag
			if (to_integer(debug_msg2.push_cnt) = 0 and to_integer(debug_msg2.pop_cnt) = 0 and to_integer(debug_msg2.overwrite_cnt) = 0) then -- very clean, no underflow
				debug_msg2.cam_clean				<= '1';
			else 
				debug_msg2.cam_clean				<= '0';
			end if;
		end if;
	end process;
	

	
	proc_run_control_mgmt : process (i_clk,i_rst)
	-- In mu3e run control system, each feb has a run control management host which runs in reset clock domain, while other IPs must feature
	-- run control management agent which listens the run state command to capture the transition.
	-- The state transition are only ack by the agent for as little as 1 cycle, but the host must assert the valid until all ack by the agents are received,
	-- during transitioning period. 
	-- The host should record the timestamps (clock cycle and phase) difference between the run command signal is received by its lvds_rx and 
	-- agents' ready signal. This should ensure all agents are running at the same time, despite there is phase uncertainty between the clocks, which 
	-- might results in 1 clock cycle difference and should be compensated offline. 
	begin 
		if (i_rst = '1') then 
			run_mgmt_flush_memory_start			<= '0';
			run_mgmt_flushed					<= '0';
			run_state_cmd						<= IDLE;
		elsif (rising_edge(i_clk)) then 
			-- valid
			if (asi_ctrl_valid = '1') then 
				-- payload of run control to run cmd
				case asi_ctrl_data is 
					when "000000001" =>
						run_state_cmd		<= IDLE;
					when "000000010" => 
						run_state_cmd		<= RUN_PREPARE;
					when "000000100" =>
						run_state_cmd		<= SYNC;
					when "000001000" =>
						run_state_cmd		<= RUNNING;
					when "000010000" =>
						run_state_cmd		<= TERMINATING;
					when "000100000" => 
						run_state_cmd		<= LINK_TEST;
					when "001000000" =>
						run_state_cmd		<= SYNC_TEST;
					when "010000000" =>
						run_state_cmd		<= RESET;
					when "100000000" =>
						run_state_cmd		<= OUT_OF_DAQ;
					when others =>
						run_state_cmd		<= ERROR;
				end case;
			else 
				run_state_cmd		<= run_state_cmd;
			end if;
			
			-- register the global timestamp when transition to TERMINATING
			run_state_cmd_d1	<= run_state_cmd;
			if (run_state_cmd_d1 /= TERMINATING and run_state_cmd = TERMINATING) then 
				gts_end_of_run	<= gts_8n;
			else
				gts_end_of_run	<= gts_end_of_run;
			end if;
			
			-- packet support (hit type 1: mu3e run)
			if (asi_hit_type1_endofpacket = '1') then 
				endofrun_seen 		<= '1';
			elsif (run_state_cmd = IDLE) then -- reset it here
				endofrun_seen		<= '0';
			end if;
			
			
			-- mgmt main state machine
			case run_state_cmd is 
				when IDLE => 
					if (asi_ctrl_valid = '1') then 
						asi_ctrl_ready			<= '1';
					else
						asi_ctrl_ready			<= '0';
					end if;
				when RUN_PREPARE =>
					-- flush the fifo
					pop_cmd_fifo_sclr		<= '1';
					deassembly_fifo_sclr	<= '1';
					-- flush the cam and ram (inter-fsm communication)
					if (run_mgmt_flush_memory_start = '0' and run_mgmt_flushed = '0') then -- only flush once
						run_mgmt_flush_memory_start	<= '1'; -- start cam and ram flushing
					elsif (run_mgmt_flush_memory_start = '1' and run_mgmt_flush_memory_done = '1') then
						run_mgmt_flush_memory_start	<= '0'; -- ack the agent is done
						run_mgmt_flushed			<= '1';
					end if;
					-- ack the run state 
					if (pop_cmd_fifo_empty = '1' and deassembly_fifo_empty = '1' and debug_msg.cam_clean = '1' and (run_mgmt_flush_memory_start = '1' and run_mgmt_flush_memory_done = '1')) then 
						asi_ctrl_ready			<= '1'; -- clean (is initially, check other cnts), additionally must be flushed (only 1 cycle, but guarantee the flush was performed)
					else
						asi_ctrl_ready			<= '0';
					end if;
					-- counters were reset by pop engine
				when SYNC => 
					gts_counter_rst			<= '1';
					if (asi_ctrl_valid = '1') then -- ack the host immediately 
						asi_ctrl_ready			<= '1';
					else
						asi_ctrl_ready			<= '0';
					end if;
				when RUNNING =>
					-- release the reset and sclr
					gts_counter_rst			<= '0';
					pop_cmd_fifo_sclr		<= '0';
					deassembly_fifo_sclr	<= '0';
					if (asi_ctrl_valid = '1') then -- ack the host immediately 
						asi_ctrl_ready			<= '1';
					else
						asi_ctrl_ready			<= '0';
					end if;
					run_mgmt_flushed		<= '0'; -- unset this flag so flush must be once
				when TERMINATING => 
					if (read_time_ptr >= gts_end_of_run) then -- TODO: change it to output fifo empty
						asi_ctrl_ready			<= '1';
					else
						asi_ctrl_ready			<= '0';
					end if;
					run_mgmt_flushed		<= '0'; -- unset this flag so flush must be once
				when others =>
					pop_cmd_fifo_sclr		<= '0';
					deassembly_fifo_sclr	<= '0';
					asi_ctrl_ready			<= '0'; -- not supported yet
					run_mgmt_flushed		<= '0'; -- unset this flag so flush must be once
			end case;
		end if;
		
	end process;
	
	







end architecture rtl;










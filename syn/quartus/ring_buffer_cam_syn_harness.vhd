library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ring_buffer_cam_syn_harness is
    generic (
        G_RING_BUFFER_N_ENTRY : natural := 1024;
        G_N_PARTITIONS        : natural := 4;
        G_ENCODER_LEAF_WIDTH  : natural := 16;
        G_ENCODER_PIPE_STAGES : natural := 4
    );
    port (
        clk125      : in  std_logic;
        reset_n     : in  std_logic;
        probe_out   : out std_logic_vector(31 downto 0)
    );
end entity ring_buffer_cam_syn_harness;

architecture rtl of ring_buffer_cam_syn_harness is
    constant CTRL_IDLE_CONST         : std_logic_vector(8 downto 0) := "000000001";
    constant CTRL_RUN_PREPARE_CONST  : std_logic_vector(8 downto 0) := "000000010";
    constant CTRL_SYNC_CONST         : std_logic_vector(8 downto 0) := "000000100";
    constant CTRL_RUNNING_CONST      : std_logic_vector(8 downto 0) := "000001000";
    constant PREPARE_WAIT_CYCLES     : natural := 300000;
    constant EXPECTED_LATENCY_CONST  : natural := 128;

    type stim_state_t is (
        ISSUE_PREPARE,
        WAIT_PREPARE,
        ISSUE_LATENCY,
        ISSUE_SYNC,
        ISSUE_RUNNING,
        STREAM
    );

    signal rst                       : std_logic;
    signal stim_state                : stim_state_t;
    signal prepare_wait_counter      : unsigned(18 downto 0);
    signal hit_counter               : unsigned(15 downto 0);
    signal search_key_counter        : unsigned(5 downto 0);
    signal signature                 : std_logic_vector(31 downto 0);

    signal avs_csr_readdata          : std_logic_vector(31 downto 0);
    signal avs_csr_read              : std_logic;
    signal avs_csr_address           : std_logic_vector(4 downto 0);
    signal avs_csr_waitrequest       : std_logic;
    signal avs_csr_write             : std_logic;
    signal avs_csr_writedata         : std_logic_vector(31 downto 0);

    signal asi_ctrl_data             : std_logic_vector(8 downto 0);
    signal asi_ctrl_valid            : std_logic;
    signal asi_ctrl_ready            : std_logic;

    signal asi_hit_type1_channel     : std_logic_vector(3 downto 0);
    signal asi_hit_type1_sop         : std_logic;
    signal asi_hit_type1_eop         : std_logic;
    signal asi_hit_type1_data        : std_logic_vector(38 downto 0);
    signal asi_hit_type1_valid       : std_logic;
    signal asi_hit_type1_ready       : std_logic;
    signal asi_hit_type1_error       : std_logic_vector(0 downto 0);

    signal aso_hit_type2_channel     : std_logic_vector(3 downto 0);
    signal aso_hit_type2_sop         : std_logic;
    signal aso_hit_type2_eop         : std_logic;
    signal aso_hit_type2_data        : std_logic_vector(35 downto 0);
    signal aso_hit_type2_valid       : std_logic;
    signal aso_hit_type2_ready       : std_logic;
    signal aso_hit_type2_error       : std_logic_vector(0 downto 0);

    signal aso_filllevel_valid       : std_logic;
    signal aso_filllevel_data        : std_logic_vector(15 downto 0);

begin
    rst                     <= not reset_n;
    aso_hit_type2_ready     <= '1';
    probe_out               <= signature;

    proc_stimulus : process (clk125, rst)
        variable search_key_v    : std_logic_vector(7 downto 0);
        variable tcc8n_v         : std_logic_vector(12 downto 0);
    begin
        if rst = '1' then
            stim_state               <= ISSUE_PREPARE;
            prepare_wait_counter     <= (others => '0');
            hit_counter              <= (others => '0');
            search_key_counter       <= (others => '0');
            avs_csr_read             <= '0';
            avs_csr_address          <= (others => '0');
            avs_csr_write            <= '0';
            avs_csr_writedata        <= (others => '0');
            asi_ctrl_data            <= CTRL_IDLE_CONST;
            asi_ctrl_valid           <= '0';
            asi_hit_type1_channel    <= (others => '0');
            asi_hit_type1_sop        <= '0';
            asi_hit_type1_eop        <= '0';
            asi_hit_type1_data       <= (others => '0');
            asi_hit_type1_valid      <= '0';
            asi_hit_type1_error      <= (others => '0');
        elsif rising_edge(clk125) then
            avs_csr_read             <= '0';
            avs_csr_address          <= (others => '0');
            avs_csr_write            <= '0';
            avs_csr_writedata        <= (others => '0');
            asi_ctrl_data            <= CTRL_IDLE_CONST;
            asi_ctrl_valid           <= '0';
            asi_hit_type1_channel    <= (others => '0');
            asi_hit_type1_sop        <= '0';
            asi_hit_type1_eop        <= '0';
            asi_hit_type1_data       <= (others => '0');
            asi_hit_type1_valid      <= '0';
            asi_hit_type1_error      <= (others => '0');

            case stim_state is
                when ISSUE_PREPARE =>
                    asi_ctrl_data    <= CTRL_RUN_PREPARE_CONST;
                    asi_ctrl_valid   <= '1';
                    if asi_ctrl_ready = '1' then
                        prepare_wait_counter <= (others => '0');
                        stim_state           <= WAIT_PREPARE;
                    end if;

                when WAIT_PREPARE =>
                    if prepare_wait_counter = to_unsigned(PREPARE_WAIT_CYCLES - 1, prepare_wait_counter'length) then
                        stim_state           <= ISSUE_LATENCY;
                    else
                        prepare_wait_counter <= prepare_wait_counter + 1;
                    end if;

                when ISSUE_LATENCY =>
                    avs_csr_address          <= std_logic_vector(to_unsigned(1, avs_csr_address'length));
                    avs_csr_writedata        <= std_logic_vector(to_unsigned(EXPECTED_LATENCY_CONST, 32));
                    avs_csr_write            <= '1';
                    if avs_csr_waitrequest = '0' then
                        stim_state           <= ISSUE_SYNC;
                    end if;

                when ISSUE_SYNC =>
                    asi_ctrl_data            <= CTRL_SYNC_CONST;
                    asi_ctrl_valid           <= '1';
                    if asi_ctrl_ready = '1' then
                        stim_state           <= ISSUE_RUNNING;
                    end if;

                when ISSUE_RUNNING =>
                    asi_ctrl_data            <= CTRL_RUNNING_CONST;
                    asi_ctrl_valid           <= '1';
                    if asi_ctrl_ready = '1' then
                        stim_state           <= STREAM;
                    end if;

                when STREAM =>
                    search_key_v             := std_logic_vector(search_key_counter) & "00";
                    tcc8n_v                  := '0' & search_key_v & "0000";
                    asi_hit_type1_valid      <= '1';
                    asi_hit_type1_channel    <= std_logic_vector(hit_counter(3 downto 0));
                    asi_hit_type1_data(38 downto 35) <= std_logic_vector(hit_counter(3 downto 0));
                    asi_hit_type1_data(34 downto 30) <= std_logic_vector(hit_counter(8 downto 4));
                    asi_hit_type1_data(29 downto 17) <= tcc8n_v;
                    asi_hit_type1_data(16 downto 14) <= std_logic_vector(hit_counter(11 downto 9));
                    asi_hit_type1_data(13 downto 9)  <= std_logic_vector(hit_counter(15 downto 11) xor "10101");
                    asi_hit_type1_data(8 downto 0)   <= std_logic_vector(hit_counter(8 downto 0));
                    if asi_hit_type1_ready = '1' then
                        hit_counter          <= hit_counter + 1;
                        search_key_counter   <= search_key_counter + 1;
                    end if;
            end case;
        end if;
    end process;

    proc_signature : process (clk125, rst)
        variable signature_v : std_logic_vector(31 downto 0);
    begin
        if rst = '1' then
            signature <= (others => '0');
        elsif rising_edge(clk125) then
            signature_v := signature;

            if aso_hit_type2_valid = '1' then
                signature_v := signature_v xor aso_hit_type2_data(31 downto 0);
                signature_v(3 downto 0) := signature_v(3 downto 0) xor aso_hit_type2_data(35 downto 32);
            end if;

            if aso_filllevel_valid = '1' then
                signature_v(15 downto 0) := signature_v(15 downto 0) xor aso_filllevel_data;
            end if;

            signature_v(19 downto 16) := signature_v(19 downto 16) xor aso_hit_type2_channel;
            signature_v(20)           := signature_v(20) xor aso_hit_type2_sop;
            signature_v(21)           := signature_v(21) xor aso_hit_type2_eop;
            signature_v(22)           := signature_v(22) xor aso_hit_type2_error(0);
            signature_v(30 downto 23) := signature_v(30 downto 23) xor std_logic_vector(hit_counter(7 downto 0));
            signature_v(31)           := signature_v(31) xor asi_hit_type1_ready;

            signature <= signature_v;
        end if;
    end process;

    u_dut : entity work.ring_buffer_cam
        generic map (
            SEARCH_KEY_WIDTH    => 8,
            RING_BUFFER_N_ENTRY => G_RING_BUFFER_N_ENTRY,
            SIDE_DATA_BITS      => 31,
            INTERLEAVING_FACTOR => 4,
            INTERLEAVING_INDEX  => 0,
            N_PARTITIONS        => G_N_PARTITIONS,
            ENCODER_LEAF_WIDTH  => G_ENCODER_LEAF_WIDTH,
            ENCODER_PIPE_STAGES => G_ENCODER_PIPE_STAGES,
            DEBUG               => 0
        )
        port map (
            avs_csr_readdata            => avs_csr_readdata,
            avs_csr_read                => avs_csr_read,
            avs_csr_address             => avs_csr_address,
            avs_csr_waitrequest         => avs_csr_waitrequest,
            avs_csr_write               => avs_csr_write,
            avs_csr_writedata           => avs_csr_writedata,
            asi_ctrl_data               => asi_ctrl_data,
            asi_ctrl_valid              => asi_ctrl_valid,
            asi_ctrl_ready              => asi_ctrl_ready,
            asi_hit_type1_channel       => asi_hit_type1_channel,
            asi_hit_type1_startofpacket => asi_hit_type1_sop,
            asi_hit_type1_endofpacket   => asi_hit_type1_eop,
            asi_hit_type1_data          => asi_hit_type1_data,
            asi_hit_type1_valid         => asi_hit_type1_valid,
            asi_hit_type1_ready         => asi_hit_type1_ready,
            asi_hit_type1_error         => asi_hit_type1_error,
            aso_hit_type2_channel       => aso_hit_type2_channel,
            aso_hit_type2_startofpacket => aso_hit_type2_sop,
            aso_hit_type2_endofpacket   => aso_hit_type2_eop,
            aso_hit_type2_data          => aso_hit_type2_data,
            aso_hit_type2_valid         => aso_hit_type2_valid,
            aso_hit_type2_ready         => aso_hit_type2_ready,
            aso_hit_type2_error         => aso_hit_type2_error,
            aso_filllevel_valid         => aso_filllevel_valid,
            aso_filllevel_data          => aso_filllevel_data,
            i_rst                       => rst,
            i_clk                       => clk125
        );

end architecture rtl;

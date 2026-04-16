library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.ring_buffer_cam_tb_pkg.all;

entity ring_buffer_cam_terminate_marker_tb is
end entity ring_buffer_cam_terminate_marker_tb;

architecture tb of ring_buffer_cam_terminate_marker_tb is
  constant CLK_PERIOD_CONST        : time    := 8 ns;
  constant TARGET_SEARCH_KEY_CONST : natural := 16#04#;
  constant TARGET_TS8N_CONST       : natural := TARGET_SEARCH_KEY_CONST * 16;
  constant EXPECTED_LATENCY_CONST  : natural := 16;
  constant FLUSH_WAIT_CONST        : time    := 1200 us;

  signal clk : std_logic := '0';
  signal rst : std_logic := '1';

  signal avs_csr_readdata    : std_logic_vector(31 downto 0);
  signal avs_csr_read        : std_logic := '0';
  signal avs_csr_address     : std_logic_vector(4 downto 0) := (others => '0');
  signal avs_csr_waitrequest : std_logic;
  signal avs_csr_write       : std_logic := '0';
  signal avs_csr_writedata   : std_logic_vector(31 downto 0) := (others => '0');

  signal asi_ctrl_data  : std_logic_vector(8 downto 0) := (others => '0');
  signal asi_ctrl_valid : std_logic := '0';
  signal asi_ctrl_ready : std_logic;

  signal asi_hit_type1_channel       : std_logic_vector(3 downto 0) := (others => '0');
  signal asi_hit_type1_startofpacket : std_logic := '0';
  signal asi_hit_type1_endofpacket   : std_logic := '0';
  signal asi_hit_type1_empty         : std_logic := '0';
  signal asi_hit_type1_data          : std_logic_vector(38 downto 0) := (others => '0');
  signal asi_hit_type1_valid         : std_logic := '0';
  signal asi_hit_type1_ready         : std_logic;
  signal asi_hit_type1_error         : std_logic_vector(0 downto 0) := (others => '0');

  signal aso_hit_type2_channel       : std_logic_vector(3 downto 0);
  signal aso_hit_type2_startofpacket : std_logic;
  signal aso_hit_type2_endofpacket   : std_logic;
  signal aso_hit_type2_data          : std_logic_vector(35 downto 0);
  signal aso_hit_type2_valid         : std_logic;
  signal aso_hit_type2_ready         : std_logic := '1';
  signal aso_hit_type2_error         : std_logic_vector(0 downto 0);

  signal aso_filllevel_valid : std_logic;
  signal aso_filllevel_data  : std_logic_vector(15 downto 0);

  signal observed_beat_count      : natural := 0;
  signal observed_subheader_count : natural := 0;
  signal observed_hit_count       : natural := 0;
  signal observed_data_eop_count  : natural := 0;
begin
  clk <= not clk after CLK_PERIOD_CONST / 2;

  dut : entity work.ring_buffer_cam
    generic map (
      SEARCH_KEY_WIDTH             => 8,
      RING_BUFFER_N_ENTRY          => 512,
      SIDE_DATA_BITS               => 31,
      INTERLEAVING_FACTOR          => 4,
      INTERLEAVING_INDEX           => 0,
      N_PARTITIONS                 => 1,
      ENCODER_LEAF_WIDTH           => 16,
      DEBUG                        => 0
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
      asi_hit_type1_startofpacket => asi_hit_type1_startofpacket,
      asi_hit_type1_endofpacket   => asi_hit_type1_endofpacket,
      asi_hit_type1_empty         => asi_hit_type1_empty,
      asi_hit_type1_data          => asi_hit_type1_data,
      asi_hit_type1_valid         => asi_hit_type1_valid,
      asi_hit_type1_ready         => asi_hit_type1_ready,
      asi_hit_type1_error         => asi_hit_type1_error(0),
      aso_hit_type2_channel       => aso_hit_type2_channel,
      aso_hit_type2_startofpacket => aso_hit_type2_startofpacket,
      aso_hit_type2_endofpacket   => aso_hit_type2_endofpacket,
      aso_hit_type2_data          => aso_hit_type2_data,
      aso_hit_type2_valid         => aso_hit_type2_valid,
      aso_hit_type2_ready         => aso_hit_type2_ready,
      aso_hit_type2_error         => aso_hit_type2_error(0),
      aso_filllevel_valid         => aso_filllevel_valid,
      aso_filllevel_data          => aso_filllevel_data,
      i_rst                       => rst,
      i_clk                       => clk
    );

  stim : process
    variable base_beat_count_v      : natural;
    variable base_subheader_count_v : natural;
    variable base_hit_count_v       : natural;
    variable base_data_eop_count_v  : natural;

    procedure tick(cycles_v : positive := 1) is
    begin
      for i in 1 to cycles_v loop
        wait until rising_edge(clk);
      end loop;
    end procedure tick;

    procedure csr_write(addr_v : natural; data_v : std_logic_vector(31 downto 0)) is
    begin
      avs_csr_address   <= std_logic_vector(to_unsigned(addr_v, avs_csr_address'length));
      avs_csr_writedata <= data_v;
      avs_csr_write     <= '1';
      wait until rising_edge(clk);
      avs_csr_write     <= '0';
      avs_csr_address   <= (others => '0');
      avs_csr_writedata <= (others => '0');
      wait until rising_edge(clk);
    end procedure csr_write;

    procedure send_ctrl(cmd_v : std_logic_vector(8 downto 0)) is
    begin
      asi_ctrl_data  <= cmd_v;
      asi_ctrl_valid <= '1';
      loop
        wait until rising_edge(clk);
        exit when asi_ctrl_ready = '1';
      end loop;
      asi_ctrl_valid <= '0';
      asi_ctrl_data  <= (others => '0');
      wait until rising_edge(clk);
    end procedure send_ctrl;

    procedure send_hit(et_v : natural) is
      variable hit_data_v : std_logic_vector(38 downto 0);
    begin
      hit_data_v := make_hit_type1(
        asic_v    => 2,
        channel_v => 3,
        tcc8n_v   => TARGET_TS8N_CONST,
        tcc1n6_v  => 0,
        tfine_v   => 0,
        et1n6_v   => et_v
      );

      asi_hit_type1_channel       <= (others => '0');
      asi_hit_type1_startofpacket <= '1';
      asi_hit_type1_endofpacket   <= '0';
      asi_hit_type1_empty         <= '0';
      asi_hit_type1_data          <= hit_data_v;
      asi_hit_type1_valid         <= '1';
      loop
        wait until rising_edge(clk);
        exit when asi_hit_type1_ready = '1';
      end loop;
      asi_hit_type1_channel       <= (others => '0');
      asi_hit_type1_startofpacket <= '0';
      asi_hit_type1_endofpacket   <= '0';
      asi_hit_type1_empty         <= '0';
      asi_hit_type1_data          <= (others => '0');
      asi_hit_type1_valid         <= '0';
    end procedure send_hit;

    procedure send_close_marker(sop_v : std_logic) is
    begin
      asi_hit_type1_channel       <= (others => '0');
      asi_hit_type1_startofpacket <= sop_v;
      asi_hit_type1_endofpacket   <= '1';
      asi_hit_type1_empty         <= '1';
      asi_hit_type1_data          <= (others => '0');
      asi_hit_type1_valid         <= '1';
      loop
        wait until rising_edge(clk);
        exit when asi_hit_type1_ready = '1';
      end loop;
      asi_hit_type1_channel       <= (others => '0');
      asi_hit_type1_startofpacket <= '0';
      asi_hit_type1_endofpacket   <= '0';
      asi_hit_type1_empty         <= '0';
      asi_hit_type1_data          <= (others => '0');
      asi_hit_type1_valid         <= '0';
    end procedure send_close_marker;
  begin
    rst <= '1';
    tick(8);
    rst <= '0';
    tick(4);

    send_ctrl(CTRL_RUN_PREPARE_CONST);
    wait for FLUSH_WAIT_CONST;
    csr_write(1, std_logic_vector(to_unsigned(EXPECTED_LATENCY_CONST, 32)));
    send_ctrl(CTRL_SYNC_CONST);
    tick(2);
    send_ctrl(CTRL_RUNNING_CONST);

    send_hit(16#012#);
    base_beat_count_v      := observed_beat_count;
    base_subheader_count_v := observed_subheader_count;
    base_hit_count_v       := observed_hit_count;
    base_data_eop_count_v  := observed_data_eop_count;

    asi_ctrl_data  <= CTRL_TERMINATING_CONST;
    asi_ctrl_valid <= '1';
    wait until rising_edge(clk);
    send_close_marker('0');
    wait until asi_ctrl_ready = '1' for 100 us;
    assert asi_ctrl_ready = '1'
      report "Timed out waiting for TERMINATING ready after the payload-lane close marker"
      severity failure;
    asi_ctrl_valid <= '0';
    asi_ctrl_data  <= (others => '0');

    wait until observed_hit_count = base_hit_count_v + 1 and observed_data_eop_count = base_data_eop_count_v + 1 for 200 us;
    assert observed_hit_count = base_hit_count_v + 1
      report "Exactly one payload hit must drain after the close marker"
      severity failure;
    assert observed_data_eop_count = base_data_eop_count_v + 1
      report "The drained payload must still close with one output EOP"
      severity failure;
    send_ctrl(CTRL_IDLE_CONST);
    send_ctrl(CTRL_RUN_PREPARE_CONST);
    wait for FLUSH_WAIT_CONST;
    csr_write(1, std_logic_vector(to_unsigned(EXPECTED_LATENCY_CONST, 32)));
    send_ctrl(CTRL_SYNC_CONST);
    tick(2);
    send_ctrl(CTRL_RUNNING_CONST);

    base_beat_count_v      := observed_beat_count;
    base_subheader_count_v := observed_subheader_count;
    base_hit_count_v       := observed_hit_count;
    base_data_eop_count_v  := observed_data_eop_count;

    asi_ctrl_data  <= CTRL_TERMINATING_CONST;
    asi_ctrl_valid <= '1';
    wait until rising_edge(clk);
    send_close_marker('1');
    wait until asi_ctrl_ready = '1' for 100 us;
    assert asi_ctrl_ready = '1'
      report "Timed out waiting for TERMINATING ready after the idle-lane close marker"
      severity failure;
    asi_ctrl_valid <= '0';
    asi_ctrl_data  <= (others => '0');

    tick(256);
    assert observed_beat_count = base_beat_count_v
      report "Idle close marker must not create any output beat"
      severity failure;
    assert observed_subheader_count = base_subheader_count_v
      report "Idle close marker must not create a ghost subheader"
      severity failure;
    assert observed_hit_count = base_hit_count_v
      report "Idle close marker must not create a ghost hit"
      severity failure;
    assert observed_data_eop_count = base_data_eop_count_v
      report "Idle close marker must not create a ghost EOP"
      severity failure;

    send_ctrl(CTRL_IDLE_CONST);
    report "RING_BUFFER_CAM_TERMINATE_MARKER_PASS" severity note;
    finish;
  end process;

  monitor : process
  begin
    wait until rising_edge(clk);
    loop
      wait until rising_edge(clk);
      if rst = '1' then
        observed_beat_count      <= 0;
        observed_subheader_count <= 0;
        observed_hit_count       <= 0;
        observed_data_eop_count  <= 0;
      elsif aso_hit_type2_valid = '1' then
        observed_beat_count <= observed_beat_count + 1;
        assert aso_hit_type2_error = "0"
          report "Terminate-marker case saw unexpected output error"
          severity failure;
        if aso_hit_type2_data(35 downto 32) = "0001" then
          observed_subheader_count <= observed_subheader_count + 1;
        else
          observed_hit_count <= observed_hit_count + 1;
          if aso_hit_type2_endofpacket = '1' then
            observed_data_eop_count <= observed_data_eop_count + 1;
          end if;
        end if;
      end if;
    end loop;
  end process;
end architecture tb;

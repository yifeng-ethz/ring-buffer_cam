library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.ring_buffer_cam_tb_pkg.all;

entity ring_buffer_cam_pipeline_smoke_tb is
end entity ring_buffer_cam_pipeline_smoke_tb;

architecture tb of ring_buffer_cam_pipeline_smoke_tb is
  constant CLK_PERIOD_CONST        : time    := 8 ns;
  constant TARGET_SEARCH_KEY_CONST : natural := 16#04#;
  constant TARGET_TS8N_CONST       : natural := TARGET_SEARCH_KEY_CONST * 16;
  constant TARGET_HIT_COUNT_CONST  : natural := 65;
  constant EXPECTED_LATENCY_CONST  : natural := 96;
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

  signal target_done : std_logic := '0';
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
      DEBUG                        => 1
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

    procedure send_hit(hit_index_v : natural) is
      variable hit_data_v : std_logic_vector(38 downto 0);
    begin
      hit_data_v := make_hit_type1(
        asic_v    => hit_index_v mod 16,
        channel_v => 3,
        tcc8n_v   => TARGET_TS8N_CONST,
        tcc1n6_v  => hit_index_v mod 8,
        tfine_v   => hit_index_v mod 32,
        et1n6_v   => hit_index_v
      );

      asi_hit_type1_channel <= std_logic_vector(to_unsigned(hit_index_v mod 16, asi_hit_type1_channel'length));
      asi_hit_type1_data    <= hit_data_v;
      asi_hit_type1_valid   <= '1';
      loop
        wait until rising_edge(clk);
        exit when asi_hit_type1_ready = '1';
      end loop;
      asi_hit_type1_valid   <= '0';
      asi_hit_type1_data    <= (others => '0');
      asi_hit_type1_channel <= (others => '0');
    end procedure send_hit;
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

    for i in 0 to TARGET_HIT_COUNT_CONST - 1 loop
      send_hit(i);
    end loop;

    wait until target_done = '1' for 100 us;
    assert target_done = '1'
      report "timeout waiting for the target subheader to drain"
      severity failure;

    tick(16);
    assert aso_filllevel_valid = '1'
      report "fill-level stream did not stay valid after reset release"
      severity failure;
    assert aso_filllevel_data = x"0000"
      report "fill level did not return to zero after the target drain"
      severity failure;

    report "RING_BUFFER_CAM_PIPELINE_SMOKE_PASS" severity note;
    finish;
  end process;

  monitor : process
    variable in_target_frame_v  : boolean := false;
    variable target_hit_count_v : natural := 0;
  begin
    wait until rising_edge(clk);
    loop
      wait until rising_edge(clk);

      if rst = '1' then
        in_target_frame_v  := false;
        target_hit_count_v := 0;
        target_done        <= '0';
      elsif aso_hit_type2_valid = '1' then
        assert aso_hit_type2_error = "0"
          report "unexpected output error flag during smoke test"
          severity failure;

        if aso_hit_type2_data(35 downto 32) = "0001" then
          if to_integer(unsigned(aso_hit_type2_data(31 downto 24))) = TARGET_SEARCH_KEY_CONST then
            assert to_integer(unsigned(aso_hit_type2_data(15 downto 8))) = TARGET_HIT_COUNT_CONST
              report "target subheader did not carry the expected hit count"
              severity failure;
            assert aso_hit_type2_startofpacket = '1'
              report "target subheader missed SOP"
              severity failure;
            assert aso_hit_type2_endofpacket = '0'
              report "target subheader unexpectedly ended the packet"
              severity failure;
            in_target_frame_v  := true;
            target_hit_count_v := 0;
          end if;
        elsif in_target_frame_v then
          target_hit_count_v := target_hit_count_v + 1;
          if target_hit_count_v < TARGET_HIT_COUNT_CONST then
            assert aso_hit_type2_endofpacket = '0'
              report "packet ended before all target hits drained"
              severity failure;
          else
            assert target_hit_count_v = TARGET_HIT_COUNT_CONST
              report "target frame drained too many hits"
              severity failure;
            assert aso_hit_type2_endofpacket = '1'
              report "last target hit missed EOP"
              severity failure;
            in_target_frame_v := false;
            target_done       <= '1';
          end if;
        end if;
      end if;
    end loop;
  end process;
end architecture tb;

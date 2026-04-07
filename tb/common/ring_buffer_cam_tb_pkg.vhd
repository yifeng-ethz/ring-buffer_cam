library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package ring_buffer_cam_tb_pkg is
  constant CTRL_IDLE_CONST        : std_logic_vector(8 downto 0) := "000000001";
  constant CTRL_RUN_PREPARE_CONST : std_logic_vector(8 downto 0) := "000000010";
  constant CTRL_SYNC_CONST        : std_logic_vector(8 downto 0) := "000000100";
  constant CTRL_RUNNING_CONST     : std_logic_vector(8 downto 0) := "000001000";
  constant CTRL_TERMINATING_CONST : std_logic_vector(8 downto 0) := "000010000";

  function make_hit_type1(
    asic_v    : natural;
    channel_v : natural;
    tcc8n_v   : natural;
    tcc1n6_v  : natural;
    tfine_v   : natural;
    et1n6_v   : natural
  ) return std_logic_vector;
end package ring_buffer_cam_tb_pkg;

package body ring_buffer_cam_tb_pkg is
  function make_hit_type1(
    asic_v    : natural;
    channel_v : natural;
    tcc8n_v   : natural;
    tcc1n6_v  : natural;
    tfine_v   : natural;
    et1n6_v   : natural
  ) return std_logic_vector is
    variable data_v : std_logic_vector(38 downto 0) := (others => '0');
  begin
    data_v(38 downto 35) := std_logic_vector(to_unsigned(asic_v, 4));
    data_v(34 downto 30) := std_logic_vector(to_unsigned(channel_v, 5));
    data_v(29 downto 17) := std_logic_vector(to_unsigned(tcc8n_v, 13));
    data_v(16 downto 14) := std_logic_vector(to_unsigned(tcc1n6_v, 3));
    data_v(13 downto 9)  := std_logic_vector(to_unsigned(tfine_v, 5));
    data_v(8 downto 0)   := std_logic_vector(to_unsigned(et1n6_v, 9));
    return data_v;
  end function make_hit_type1;
end package body ring_buffer_cam_tb_pkg;

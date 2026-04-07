library ieee;
use ieee.std_logic_1164.all;

entity ring_buffer_cam_syn_v23_top is
    port (
        clk125      : in  std_logic;
        reset_n     : in  std_logic;
        probe_out   : out std_logic_vector(31 downto 0)
    );
end entity ring_buffer_cam_syn_v23_top;

architecture rtl of ring_buffer_cam_syn_v23_top is
begin
    u_harness : entity work.ring_buffer_cam_v23_syn_harness
        generic map (
            G_RING_BUFFER_N_ENTRY => 1024
        )
        port map (
            clk125    => clk125,
            reset_n   => reset_n,
            probe_out => probe_out
        );
end architecture rtl;

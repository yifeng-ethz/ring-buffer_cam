library ieee;
use ieee.std_logic_1164.all;

entity ring_buffer_cam_syn_p4_top is
    port (
        clk125      : in  std_logic;
        reset_n     : in  std_logic;
        probe_out   : out std_logic_vector(31 downto 0)
    );
end entity ring_buffer_cam_syn_p4_top;

architecture rtl of ring_buffer_cam_syn_p4_top is
begin
    u_harness : entity work.ring_buffer_cam_syn_harness
        generic map (
            G_RING_BUFFER_N_ENTRY => 1024,
            G_N_PARTITIONS        => 4,
            G_ENCODER_LEAF_WIDTH  => 16,
            G_ENCODER_PIPE_STAGES => 4
        )
        port map (
            clk125    => clk125,
            reset_n   => reset_n,
            probe_out => probe_out
        );
end architecture rtl;

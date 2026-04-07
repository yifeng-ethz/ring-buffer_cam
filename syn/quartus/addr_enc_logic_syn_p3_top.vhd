library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity addr_enc_logic_syn_p3_top is
    port (
        clk125      : in  std_logic;
        reset_n     : in  std_logic;
        cam_onehot  : in  std_logic_vector(511 downto 0);
        probe_out   : out std_logic_vector(31 downto 0)
    );
end entity addr_enc_logic_syn_p3_top;

architecture rtl of addr_enc_logic_syn_p3_top is
    signal activity_counter : unsigned(5 downto 0);
    signal load_req         : std_logic;
    signal advance_req      : std_logic;
    signal result_valid     : std_logic;
    signal match_flag       : std_logic;
    signal has_more         : std_logic;
    signal lsb_addr         : std_logic_vector(8 downto 0);
    signal match_count      : std_logic_vector(9 downto 0);
    signal onehot_next      : std_logic_vector(511 downto 0);
    signal signature        : std_logic_vector(31 downto 0);
begin
    proc_activity : process (clk125, reset_n)
    begin
        if (reset_n = '0') then
            activity_counter <= (others => '0');
        elsif rising_edge(clk125) then
            activity_counter <= activity_counter + 1;
        end if;
    end process;

    load_req    <= '1' when activity_counter = 0 else '0';
    advance_req <= '0';

    u_enc : entity work.addr_enc_logic_partitioned
        generic map (
            PARTITION_SIZE      => 512,
            PARTITION_ADDR_BITS => 9,
            LEAF_WIDTH          => 16,
            PIPE_STAGES         => 3
        )
        port map (
            i_clk                     => clk125,
            i_rst                     => not reset_n,
            i_load                    => load_req,
            i_advance                 => advance_req,
            i_cam_address_onehot      => cam_onehot,
            o_result_valid            => result_valid,
            o_cam_address_binary_lsb  => lsb_addr,
            o_cam_match_flag          => match_flag,
            o_cam_has_more_matches    => has_more,
            o_cam_match_count         => match_count,
            o_cam_address_onehot_next => onehot_next
        );

    signature(0)            <= result_valid;
    signature(1)            <= match_flag;
    signature(2)            <= has_more;
    signature(11 downto 3)  <= lsb_addr;
    signature(21 downto 12) <= match_count;
    signature(22)           <= load_req;
    signature(23)           <= has_more;
    signature(31 downto 24) <= onehot_next(7 downto 0);
    probe_out               <= signature;
end architecture rtl;

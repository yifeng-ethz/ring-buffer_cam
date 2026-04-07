library ieee;
use ieee.std_logic_1164.all;

entity cam_mem_blk_a5 is
    generic (
        WORDS : natural := 256;
        RW    : natural := 32;
        WW    : natural := 1
    );
    port (
        we    : in std_logic;
        clk   : in std_logic;
        waddr : in natural range 0 to (WORDS * RW) - 1;
        wdata : in std_logic_vector(WW - 1 downto 0);
        raddr : in natural range 0 to WORDS - 1;
        q     : out std_logic_vector(RW - 1 downto 0)
    );
end entity cam_mem_blk_a5;

architecture rtl_simple_dpram of cam_mem_blk_a5 is
    constant RATIO_CONST : natural := RW / WW;

    type word_t is array (RATIO_CONST - 1 downto 0) of std_logic_vector(WW - 1 downto 0);
    type ram_t is array (0 to WORDS - 1) of word_t;

    signal ram : ram_t := (others => (others => (others => '0')));
    signal q_local : word_t := (others => (others => '0'));
begin
    unpack : for i in 0 to RATIO_CONST - 1 generate
        q(WW * (i + 1) - 1 downto WW * i) <= q_local(i);
    end generate unpack;

    process(clk)
    begin
        if rising_edge(clk) then
            if we = '1' then
                ram(waddr / RATIO_CONST)(waddr mod RATIO_CONST) <= wdata;
            end if;
            q_local <= ram(raddr);
        end if;
    end process;
end architecture rtl_simple_dpram;

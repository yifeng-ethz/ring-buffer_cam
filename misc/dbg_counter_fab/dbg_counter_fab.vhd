-- File name: dbg_counter_fab.vhd
-- Author: Yifeng Wang (yifenwan@phys.ethz.ch)
-- =======================================
-- Revision: 1.0 (file created)
--     Date: Feb 26, 2024
-- =========
-- Description: [(Debug) Counter Interconnect]
--      It connects the conduit port of the counter into the ISSP counduit.
--      mapping of signal roles
--
-- ================ synthsizer configuration ===================
-- altera vhdl_input_version vhdl_2008
-- =============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use IEEE.math_real.log2;
use IEEE.math_real.ceil;
use ieee.std_logic_arith.conv_std_logic_vector;


entity dbg_counter_fab is

    port(

        source              : in  std_logic_vector(1 downto 0);
        enable              : out std_logic;
        sclr                : out std_logic;
        sclr1               : out std_logic;
        i_clk               : in  std_logic;
        i_clk1              : in  std_logic
    );
end entity dbg_counter_fab;

architecture rtl of dbg_counter_fab is

begin

    enable  <= '0';
    sclr1   <= source(1);
    sclr    <= source(0);

end architecture rtl;

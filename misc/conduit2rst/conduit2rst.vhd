-- File name: conduit2rst.vhd
-- Author: Yifeng Wang (yifenwan@phys.ethz.ch)
-- =======================================
-- Revision: 1.0 (file created)
--              Date: Feb 26, 2024
-- =========
-- Description: [Conduit to Reset Interface (Debug Module)]
--              Conduit to reset interface
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

entity conduit2rst is
    generic(
        DEBUG : natural := 1
    );
    port(
        i_rst : in  std_logic;
        i_clk : in  std_logic;
        o_rst : out std_logic
    );
end entity conduit2rst;

architecture rtl of conduit2rst is
begin
    o_rst <= i_rst;
end architecture rtl;

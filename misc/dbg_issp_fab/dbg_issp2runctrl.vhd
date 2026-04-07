-- File name: dbg_issp2runctrl.vhd
-- Author: Yifeng Wang (yifenwan@phys.ethz.ch)
-- =======================================
-- Revision: 1.0 (file created)
--              Date: Jul 2, 2024
-- =========
-- Description: [(Debug) ISSP to RUN Control AVST]
--
-- ================ synthsizer configuration ===================
-- altera vhdl_input_version vhdl_2008
-- =============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use IEEE.math_real.log2;
use IEEE.math_real.ceil;

entity dbg_issp2runctrl is
port (
    aso_ctrl0_data  : out std_logic_vector(8 downto 0);
    aso_ctrl0_valid : out std_logic;
    aso_ctrl0_ready : in  std_logic;
    aso_ctrl1_data  : out std_logic_vector(8 downto 0);
    aso_ctrl1_valid : out std_logic;
    aso_ctrl1_ready : in  std_logic;
    i_source        : in  std_logic_vector(9 downto 0);
    i_rst           : in  std_logic;
    i_clk           : in  std_logic
);
end entity dbg_issp2runctrl;

architecture rtl of dbg_issp2runctrl is
begin
    aso_ctrl0_data  <= i_source(8 downto 0);
    aso_ctrl0_valid <= i_source(9);

    aso_ctrl1_data  <= i_source(8 downto 0);
    aso_ctrl1_valid <= i_source(9);
end architecture rtl;

begin_group VHDL
begin_group User
begin_template Note
-- ==================================================================================================== 
-- @commentName     Mu3e "Run Control" host-agent (h2a) handshake 
--
-- @comment             In mu3e run control system, each feb has a run control management host which runs 
--                  in reset clock domain, while other IPs must feature run control management agent 
--                  which listens the run state command to capture the transition. The state transition
--                  are only ack by the agent for as little as 1 cycle, but the host must assert the 
--                  valid until all ack by the agents are received, during transitioning period. 
--                      The host should record the timestamps (clock cycle and phase) difference between
--                  the run command signal is received by its lvds_rx and agents' ready signal. 
--                      This should ensure all agents are running at the same time, despite there is 
--                  phase uncertainty between the clocks, which might results in 1 clock cycle 
--                  difference and should be compensated offline. 
-- ====================================================================================================




end_template
begin_template Block
-- \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
-- @blockName       frame_rcv 
--
-- @berief          parse the MuTRiG frame
-- @input           
-- @output          <o_hits.*> -- output hits structure, also called "event"
--                  <aso_hit_type0_*> -- hit_type0 streaming interface signals
-- \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\



end_template
begin_template Module
-- ==================================================================================================== 
-- @moduleName      crc16_calc 
-- 
-- @berief          calculate the crc for the whole frame, result available at eop
-- @input           <n_crc_din_valid> -- data is valid 
--                  <i_data> -- crc data to roll
-- @output          <s_crc_result> -- the crc output register pack
-- ====================================================================================================


end_template
begin_template Table
-- ====================================================================================================
-- @tableName       csr address map
-- 
-- @offset_step     32 bit word
-- @size            4
-- ----------------------------------------------------------------------------------------------------
-- <offset>         <name>              <access>    <description>
-- ====================================================================================================
-- 0                csr      
-- ├── [7:0]            control         rw          [0]: '1/0'=enable/disable  
-- └── [31:24]          status          ro          [5:0]: frame flag (gen_idle fast_mode prbs_debug single_prbs fifo_full pll_lol_n)
--
-- 1                error counter
-- └── [31:0]           crc errors      ro          the number of frames with crc validation error, possibly corrupted
--
-- 2                frame counter
-- └── [31:0]           total frames    ro          the number of total frames (only header and trailer are both seen is count as frame)
-- ====================================================================================================

end_template
begin_template File Header
-- ------------------------------------------------------------------------------------------------------------
-- IP Name:             lvds_rx_controller_pro
-- Author:              Yifeng Wang (yifenwan@phys.ethz.ch)
-- =========================================================================
-- Revision:            1.0
-- Date:                Oct 14, 2024 (file created)
-- +-----------------------------------------------------------------------+
-- Revision:            1.1
-- Date:                Oct 21, 2024 (control plane verified)
-- =========================================================================
-- Description:         Controls and decodes data from 'altera_lvds_rx_28nm` IP. 
-- Usage:                
-- 		                [Control flow]: (training)
--							Operates in free-running clock. Monitors DPA phase locking, DPA fifo reset.
--
--                      [Data flow]: (boundary-search)
--                      	Operates in outclock clock. Decodes the 10b into 8b data + 1b control (8b1k). 
--							Word alignment modes: 
--								1) "bit slip': takes a tens of thousands cycles to realign after bitslip incidence.
--									slow and error-prone but requires less resources (ALM). 
--								2) adaptive selection: tracking boundary by parallel monitoring 10 combinations.
--									ultra-fast and robust but requires more resources (ALM).
--
-- ------------------------------------------------------------------------------------------------------------
--	
-- ================ synthsizer configuration =================== 		
-- altera vhdl_input_version vhdl_2008
-- =============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
end_template
end_group 
end_group 

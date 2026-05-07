-- File name: addr_encoding_logic.vhd 
-- Author: Yifeng Wang
-- =======================================
-- Revision: 1.0 (wrap about addr_encoding_logic_small)
--		Date: Jul 19, 2024
-- Description: Wrapping layer
--		encoding the output of CAM. turning the found array (one-hot) into match-address (binary) 
--		Each small logic can decode 64 bit one-hot and output 64 bit one-hot and 6 bit binary.
--		This entity packs these small logics to form a wider port. (for one-hot, simple expand; for binary, arithmetic addition)

-- ================ synthsizer configuration =================== 		
-- altera vhdl_input_version vhdl_2008
-- ============================================================= 

-- !!!!NOT USED!!!!

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity addr_enc_logic is
generic(
	CAM_SIZE			: natural := 128;
	N_WAYS				: natural := 2; -- := CAM_SIZE / 64
	CAM_ADDR_BITS		: natural := 7
);
port(
	i_cam_address_onehot		: in  std_logic_vector(CAM_SIZE-1 downto 0);
	o_cam_address_binary		: out std_logic_vector(CAM_ADDR_BITS-1 downto 0);
	o_cam_match_flag			: out std_logic_vector(N_WAYS-1 downto 0);
	o_cam_match_count			: buffer std_logic_vector(CAM_ADDR_BITS*N_WAYS-1 downto 0); -- OUT type here does not work, at least in platform designer
	o_cam_address_onehot_next	: out std_logic_vector(CAM_SIZE-1 downto 0)
);
end entity addr_enc_logic;

architecture rtl of addr_enc_logic is 
	-- ------------------------
	-- fixed constant
	-- ------------------------
	constant SMALL_LOGIC_CAM_SIZE				: natural := 64; -- this is the maximum
	constant SMALL_LOGIC_ADDR_BITS				: natural := 6; -- derived from above size
	-- ------------------------
	-- derived constant
	-- ------------------------
	constant N_SMALL_LOGIC						: natural := CAM_SIZE / SMALL_LOGIC_CAM_SIZE; -- 2

	
	type onehot_t				is array (0 to N_SMALL_LOGIC-1) of std_logic_vector(SMALL_LOGIC_CAM_SIZE-1 downto 0); 
	signal onehot				: onehot_t;
	signal onehot_nxt			: onehot_t;
	type binary_t				is array (0 to N_SMALL_LOGIC-1) of std_logic_vector(SMALL_LOGIC_ADDR_BITS-1 downto 0);
	signal binary,count			: binary_t;
	signal flag					: std_logic_vector(N_SMALL_LOGIC-1 downto 0);
	
	signal count_sum			: std_logic_vector(o_cam_match_count'length-1 downto 0);

begin

	-- -------------------------------------------
	-- entity instantiation 
	-- -------------------------------------------
	gen_small_logic : for i in 0 to N_SMALL_LOGIC-1 generate 
		e_enc_logic_small	: entity work.addr_enc_logic_small 
		generic map(
			CAM_SIZE 		=> SMALL_LOGIC_CAM_SIZE, -- one-hot code length
			CAM_ADDR_BITS 	=> SMALL_LOGIC_ADDR_BITS -- binary code length
		)
		port map(
			i_cam_address_onehot		=> onehot(i),
			o_cam_address_onehot_next	=> onehot_nxt(i),
			o_cam_address_binary		=> binary(i),
			o_cam_match_flag			=> flag(i),
			o_cam_match_count			=> count(i)
		);
	end generate gen_small_logic;
	
	o_cam_match_count	<= count_sum;
	
	
	proc_pack_and_unpack_payload : process (all)
		--variable count_sum		: unsigned(o_cam_match_count'length-1 downto 0);
	begin
		-- default
		count_sum				<= (others => '0');
		o_cam_address_binary	<= (others => '0');
		-- assign the output as the top small logic's binary plus its address offset
		gen_top_binary : for i in 0 to N_SMALL_LOGIC-1 loop
			if (flag(i) = '1') then
				o_cam_address_binary		<= std_logic_vector(to_unsigned(to_integer(unsigned(binary(i))) + 64*i, o_cam_address_binary'length));
			end if;
		end loop gen_top_binary;
		
		gen_pack_onehot : for i in 0 to N_SMALL_LOGIC-1 loop
			o_cam_address_onehot_next((i+1)*64-1 downto i*64)		<= onehot_nxt(i);
		end loop gen_pack_onehot;
		
		gen_unpack_onehot : for i in 0 to N_SMALL_LOGIC-1 loop
			onehot(i)				<= i_cam_address_onehot((i+1)*64-1 downto i*64);
		end loop gen_unpack_onehot;
	
		-- TODO: fix this with generation or template 
		--count_sum			:= to_unsigned( to_integer(unsigned(count(0))) + to_integer(unsigned(count(1))) , count_sum'length );
		--o_cam_match_count	<= std_logic_vector(count_sum);
		
		gen_sum : for i in 0 to N_SMALL_LOGIC-1 loop
			count_sum		<= std_logic_vector( unsigned(count_sum) + unsigned(count(i)) );
		end loop gen_sum;
		
		if (to_integer(unsigned(o_cam_match_count)) /= 0) then
			o_cam_match_flag		<= '1';
		else 
			o_cam_match_flag		<= '0';
		end if;
	
	end process;


	


end architecture rtl;

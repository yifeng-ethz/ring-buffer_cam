-- File name: addr_enc_logic_pipe.vhd
-- Author: Codex
-- =======================================
-- Revision: 2.0 (major staged partition encoder)
--      Date: Mar 18, 2026
-- ================ synthsizer configuration ===================
-- altera vhdl_input_version vhdl_2008
-- ============================================================
-- Description:
--      Staged one-hot encoder for a configurable CAM partition.
--      The input is first reduced into small leaf summaries, then
--      optionally reduced again through a mid-level summary tree.
--      This keeps the hot combinational depth bounded while preserving
--      the same contract as the legacy wrapper.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity addr_enc_logic_pipe is
generic(
    CAM_SIZE        : natural := 64;
    CAM_ADDR_BITS   : natural := 6;
    PIPE_STAGES     : natural := 2
);
port(
    i_clk                       : in  std_logic;
    i_rst                       : in  std_logic;
    i_load                      : in  std_logic;
    i_cam_address_onehot        : in  std_logic_vector(CAM_SIZE-1 downto 0);
    o_result_valid              : out std_logic;
    o_cam_address_binary        : out std_logic_vector(CAM_ADDR_BITS-1 downto 0);
    o_cam_address_binary_lsb    : out std_logic_vector(CAM_ADDR_BITS-1 downto 0);
    o_cam_match_flag            : out std_logic;
    o_cam_match_count           : out std_logic_vector(CAM_ADDR_BITS downto 0);
    o_cam_address_onehot_next   : out std_logic_vector(CAM_SIZE-1 downto 0)
);
end entity addr_enc_logic_pipe;

architecture rtl of addr_enc_logic_pipe is

    function min_nat (
        lhs : natural;
        rhs : natural
    ) return natural is
    begin
        if (lhs < rhs) then
            return lhs;
        end if;
        return rhs;
    end function;

    function max_nat (
        lhs : natural;
        rhs : natural
    ) return natural is
    begin
        if (lhs > rhs) then
            return lhs;
        end if;
        return rhs;
    end function;

    function ceil_div (
        numer : natural;
        denom : natural
    ) return natural is
    begin
        return (numer + denom - 1) / denom;
    end function;

    function choose_leaf_size (
        cam_size_v      : natural;
        pipe_stages_v   : natural
    ) return natural is
    begin
        if (cam_size_v <= 32) then
            return cam_size_v;
        elsif (pipe_stages_v <= 1) then
            return 32;
        else
            return 16;
        end if;
    end function;

    constant ACTIVE_PIPE_STAGES_CONST   : natural := min_nat(PIPE_STAGES, 3);
    constant EXTRA_VALID_STAGES_CONST   : natural := PIPE_STAGES - ACTIVE_PIPE_STAGES_CONST;
    constant EXTRA_VALID_VEC_LEN_CONST  : natural := max_nat(EXTRA_VALID_STAGES_CONST, 1);
    constant LEAF_SIZE_CONST            : natural := choose_leaf_size(CAM_SIZE, ACTIVE_PIPE_STAGES_CONST);
    constant N_LEAVES_CONST             : natural := ceil_div(CAM_SIZE, LEAF_SIZE_CONST);
    constant MID_GROUP_SIZE_CONST       : natural := 4;
    constant N_MIDS_CONST               : natural := ceil_div(N_LEAVES_CONST, MID_GROUP_SIZE_CONST);

    subtype cam_addr_t      is unsigned(CAM_ADDR_BITS-1 downto 0);
    subtype cam_count_t     is unsigned(CAM_ADDR_BITS downto 0);
    subtype pipe_valid_t    is std_logic_vector(PIPE_STAGES-1 downto 0);
    subtype extra_valid_t   is std_logic_vector(EXTRA_VALID_VEC_LEN_CONST-1 downto 0);

    type leaf_summary_t is record
        flag        : std_logic;
        count       : cam_count_t;
        lsb_addr    : cam_addr_t;
        msb_addr    : cam_addr_t;
        onehot_next : std_logic_vector(LEAF_SIZE_CONST-1 downto 0);
    end record;

    type leaf_summary_arr_t is array (natural range <>) of leaf_summary_t;

    type mid_summary_t is record
        flag        : std_logic;
        count       : cam_count_t;
        lsb_addr    : cam_addr_t;
        msb_addr    : cam_addr_t;
    end record;

    type mid_summary_arr_t is array (natural range <>) of mid_summary_t;

    type encoder_result_t is record
        flag            : std_logic;
        count           : cam_count_t;
        lsb_addr        : cam_addr_t;
        msb_addr        : cam_addr_t;
        onehot_next     : std_logic_vector(CAM_SIZE-1 downto 0);
    end record;

    function null_leaf_summary return leaf_summary_t is
        variable leaf_v : leaf_summary_t;
    begin
        leaf_v.flag        := '0';
        leaf_v.count       := (others => '0');
        leaf_v.lsb_addr    := (others => '0');
        leaf_v.msb_addr    := (others => '0');
        leaf_v.onehot_next := (others => '0');
        return leaf_v;
    end function;

    function null_mid_summary return mid_summary_t is
        variable mid_v : mid_summary_t;
    begin
        mid_v.flag     := '0';
        mid_v.count    := (others => '0');
        mid_v.lsb_addr := (others => '0');
        mid_v.msb_addr := (others => '0');
        return mid_v;
    end function;

    function null_result return encoder_result_t is
        variable result_v : encoder_result_t;
    begin
        result_v.flag        := '0';
        result_v.count       := (others => '0');
        result_v.lsb_addr    := (others => '0');
        result_v.msb_addr    := (others => '0');
        result_v.onehot_next := (others => '0');
        return result_v;
    end function;

    function leaf_slice (
        onehot_v    : std_logic_vector(CAM_SIZE-1 downto 0);
        leaf_idx    : natural
    ) return std_logic_vector is
        variable slice_v : std_logic_vector(LEAF_SIZE_CONST-1 downto 0);
        variable src_idx : natural;
    begin
        slice_v := (others => '0');
        for bit_idx in 0 to LEAF_SIZE_CONST-1 loop
            src_idx := leaf_idx * LEAF_SIZE_CONST + bit_idx;
            if (src_idx < CAM_SIZE) then
                slice_v(bit_idx) := onehot_v(src_idx);
            end if;
        end loop;
        return slice_v;
    end function;

    function leaf_encode (
        onehot_v    : std_logic_vector(LEAF_SIZE_CONST-1 downto 0);
        base_idx    : natural
    ) return leaf_summary_t is
        variable leaf_v      : leaf_summary_t;
        variable first_seen  : boolean;
        variable global_idx  : natural;
    begin
        leaf_v      := null_leaf_summary;
        first_seen  := false;

        for bit_idx in 0 to LEAF_SIZE_CONST-1 loop
            global_idx := base_idx + bit_idx;
            if (global_idx < CAM_SIZE and onehot_v(bit_idx) = '1') then
                leaf_v.flag     := '1';
                leaf_v.count    := leaf_v.count + 1;
                leaf_v.msb_addr := to_unsigned(global_idx, leaf_v.msb_addr'length);
                if (first_seen = false) then
                    leaf_v.lsb_addr := to_unsigned(global_idx, leaf_v.lsb_addr'length);
                    first_seen      := true;
                else
                    leaf_v.onehot_next(bit_idx) := '1';
                end if;
            end if;
        end loop;

        return leaf_v;
    end function;

    signal cam_address_onehot_reg   : std_logic_vector(CAM_SIZE-1 downto 0);
    signal pipe_valid               : pipe_valid_t;
    signal leaf_summary_comb        : leaf_summary_arr_t(0 to N_LEAVES_CONST-1);
    signal leaf_summary_reg         : leaf_summary_arr_t(0 to N_LEAVES_CONST-1);
    signal mid_summary_comb         : mid_summary_arr_t(0 to N_MIDS_CONST-1);
    signal mid_summary_reg          : mid_summary_arr_t(0 to N_MIDS_CONST-1);
    signal result_stage1_comb       : encoder_result_t;
    signal result_stage2_comb       : encoder_result_t;
    signal result_stage3_comb       : encoder_result_t;
    signal result_reg               : encoder_result_t;
    signal result_valid_reg         : std_logic;
    signal result_valid_extra       : extra_valid_t;

begin

    assert PIPE_STAGES >= 1
        report "addr_enc_logic_pipe requires PIPE_STAGES >= 1"
        severity failure;

    proc_leaf_comb : process (all)
        variable leaf_v : leaf_summary_arr_t(0 to N_LEAVES_CONST-1);
    begin
        leaf_v := (others => null_leaf_summary);
        for leaf_idx in 0 to N_LEAVES_CONST-1 loop
            leaf_v(leaf_idx) := leaf_encode(
                leaf_slice(cam_address_onehot_reg, leaf_idx),
                leaf_idx * LEAF_SIZE_CONST
            );
        end loop;
        leaf_summary_comb <= leaf_v;
    end process;

    proc_mid_comb : process (all)
        variable mid_v : mid_summary_arr_t(0 to N_MIDS_CONST-1);
        variable leaf_idx : natural;
    begin
        mid_v := (others => null_mid_summary);
        for mid_idx in 0 to N_MIDS_CONST-1 loop
            for grp_idx in 0 to MID_GROUP_SIZE_CONST-1 loop
                leaf_idx := mid_idx * MID_GROUP_SIZE_CONST + grp_idx;
                if (leaf_idx < N_LEAVES_CONST) then
                    if (leaf_summary_reg(leaf_idx).flag = '1') then
                        if (mid_v(mid_idx).flag = '0') then
                            mid_v(mid_idx).lsb_addr := leaf_summary_reg(leaf_idx).lsb_addr;
                        end if;
                        mid_v(mid_idx).flag     := '1';
                        mid_v(mid_idx).count    := mid_v(mid_idx).count + leaf_summary_reg(leaf_idx).count;
                        mid_v(mid_idx).msb_addr := leaf_summary_reg(leaf_idx).msb_addr;
                    end if;
                end if;
            end loop;
        end loop;
        mid_summary_comb <= mid_v;
    end process;

    proc_result_stage1 : process (all)
        variable result_v        : encoder_result_t;
        variable selected_leaf   : natural range 0 to N_LEAVES_CONST-1;
        variable base_idx        : natural;
    begin
        result_v      := null_result;
        selected_leaf := 0;

        for leaf_idx in 0 to N_LEAVES_CONST-1 loop
            if (leaf_summary_comb(leaf_idx).flag = '1') then
                result_v.count := result_v.count + leaf_summary_comb(leaf_idx).count;
                if (result_v.flag = '0') then
                    result_v.flag     := '1';
                    result_v.lsb_addr := leaf_summary_comb(leaf_idx).lsb_addr;
                end if;
                result_v.msb_addr := leaf_summary_comb(leaf_idx).msb_addr;
            end if;
        end loop;

        result_v.onehot_next := cam_address_onehot_reg;
        if (result_v.flag = '1') then
            selected_leaf := to_integer(result_v.lsb_addr) / LEAF_SIZE_CONST;
            base_idx      := selected_leaf * LEAF_SIZE_CONST;
            for bit_idx in 0 to LEAF_SIZE_CONST-1 loop
                if (base_idx + bit_idx < CAM_SIZE) then
                    result_v.onehot_next(base_idx + bit_idx) := leaf_summary_comb(selected_leaf).onehot_next(bit_idx);
                end if;
            end loop;
        end if;

        result_stage1_comb <= result_v;
    end process;

    proc_result_stage2 : process (all)
        variable result_v        : encoder_result_t;
        variable selected_leaf   : natural range 0 to N_LEAVES_CONST-1;
        variable base_idx        : natural;
    begin
        result_v      := null_result;
        selected_leaf := 0;

        for leaf_idx in 0 to N_LEAVES_CONST-1 loop
            if (leaf_summary_reg(leaf_idx).flag = '1') then
                result_v.count := result_v.count + leaf_summary_reg(leaf_idx).count;
                if (result_v.flag = '0') then
                    result_v.flag     := '1';
                    result_v.lsb_addr := leaf_summary_reg(leaf_idx).lsb_addr;
                end if;
                result_v.msb_addr := leaf_summary_reg(leaf_idx).msb_addr;
            end if;
        end loop;

        result_v.onehot_next := cam_address_onehot_reg;
        if (result_v.flag = '1') then
            selected_leaf := to_integer(result_v.lsb_addr) / LEAF_SIZE_CONST;
            base_idx      := selected_leaf * LEAF_SIZE_CONST;
            for bit_idx in 0 to LEAF_SIZE_CONST-1 loop
                if (base_idx + bit_idx < CAM_SIZE) then
                    result_v.onehot_next(base_idx + bit_idx) := leaf_summary_reg(selected_leaf).onehot_next(bit_idx);
                end if;
            end loop;
        end if;

        result_stage2_comb <= result_v;
    end process;

    proc_result_stage3 : process (all)
        variable result_v        : encoder_result_t;
        variable selected_leaf   : natural range 0 to N_LEAVES_CONST-1;
        variable base_idx        : natural;
    begin
        result_v      := null_result;
        selected_leaf := 0;

        for mid_idx in 0 to N_MIDS_CONST-1 loop
            if (mid_summary_reg(mid_idx).flag = '1') then
                result_v.count := result_v.count + mid_summary_reg(mid_idx).count;
                if (result_v.flag = '0') then
                    result_v.flag     := '1';
                    result_v.lsb_addr := mid_summary_reg(mid_idx).lsb_addr;
                end if;
                result_v.msb_addr := mid_summary_reg(mid_idx).msb_addr;
            end if;
        end loop;

        result_v.onehot_next := cam_address_onehot_reg;
        if (result_v.flag = '1') then
            selected_leaf := to_integer(result_v.lsb_addr) / LEAF_SIZE_CONST;
            base_idx      := selected_leaf * LEAF_SIZE_CONST;
            for bit_idx in 0 to LEAF_SIZE_CONST-1 loop
                if (base_idx + bit_idx < CAM_SIZE) then
                    result_v.onehot_next(base_idx + bit_idx) := leaf_summary_reg(selected_leaf).onehot_next(bit_idx);
                end if;
            end loop;
        end if;

        result_stage3_comb <= result_v;
    end process;

    proc_pipe : process (i_clk, i_rst)
        variable pipe_valid_v : pipe_valid_t;
    begin
        if (i_rst = '1') then
            cam_address_onehot_reg <= (others => '0');
            pipe_valid             <= (others => '0');
            leaf_summary_reg       <= (others => null_leaf_summary);
            mid_summary_reg        <= (others => null_mid_summary);
            result_reg             <= null_result;
            result_valid_reg       <= '0';
            result_valid_extra     <= (others => '0');
        elsif (rising_edge(i_clk)) then
            pipe_valid_v := (others => '0');
            if (PIPE_STAGES > 1) then
                pipe_valid_v(PIPE_STAGES-1 downto 1) := pipe_valid(PIPE_STAGES-2 downto 0);
            end if;
            pipe_valid_v(0) := i_load;

            if (i_load = '1') then
                cam_address_onehot_reg <= i_cam_address_onehot;
            end if;

            if (ACTIVE_PIPE_STAGES_CONST >= 2 and pipe_valid(0) = '1') then
                leaf_summary_reg <= leaf_summary_comb;
            end if;

            if (ACTIVE_PIPE_STAGES_CONST >= 3 and pipe_valid(1) = '1') then
                mid_summary_reg <= mid_summary_comb;
            end if;

            if (pipe_valid(ACTIVE_PIPE_STAGES_CONST-1) = '1') then
                case ACTIVE_PIPE_STAGES_CONST is
                    when 1 =>
                        result_reg <= result_stage1_comb;
                    when 2 =>
                        result_reg <= result_stage2_comb;
                    when others =>
                        result_reg <= result_stage3_comb;
                end case;
            end if;

            if (i_load = '1') then
                result_valid_reg <= '0';
            elsif (EXTRA_VALID_STAGES_CONST = 0) then
                if (pipe_valid(ACTIVE_PIPE_STAGES_CONST-1) = '1') then
                    result_valid_reg <= '1';
                end if;
            elsif (result_valid_extra(EXTRA_VALID_STAGES_CONST-1) = '1') then
                result_valid_reg <= '1';
            end if;

            if (EXTRA_VALID_STAGES_CONST > 0) then
                if (EXTRA_VALID_STAGES_CONST > 1) then
                    result_valid_extra(EXTRA_VALID_STAGES_CONST-1 downto 1)
                        <= result_valid_extra(EXTRA_VALID_STAGES_CONST-2 downto 0);
                end if;
                if (pipe_valid(ACTIVE_PIPE_STAGES_CONST-1) = '1') then
                    result_valid_extra(0) <= '1';
                else
                    result_valid_extra(0) <= '0';
                end if;
            else
                result_valid_extra <= (others => '0');
            end if;

            pipe_valid <= pipe_valid_v;
        end if;
    end process;

    o_result_valid            <= result_valid_reg;
    o_cam_address_binary      <= std_logic_vector(result_reg.msb_addr);
    o_cam_address_binary_lsb  <= std_logic_vector(result_reg.lsb_addr);
    o_cam_match_flag          <= result_reg.flag;
    o_cam_match_count         <= std_logic_vector(result_reg.count);
    o_cam_address_onehot_next <= result_reg.onehot_next;

end architecture rtl;

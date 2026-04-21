library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity scfifo is
    generic (
        add_ram_output_register : string  := "OFF";
        intended_device_family  : string  := "Arria V";
        lpm_hint                : string  := "";
        lpm_numwords            : natural := 16;
        lpm_showahead           : string  := "ON";
        lpm_type                : string  := "scfifo";
        lpm_width               : natural := 8;
        lpm_widthu              : natural := 4;
        overflow_checking       : string  := "ON";
        underflow_checking      : string  := "ON";
        use_eab                 : string  := "ON"
    );
    port (
        clock : in  std_logic;
        data  : in  std_logic_vector(lpm_width-1 downto 0);
        rdreq : in  std_logic;
        sclr  : in  std_logic;
        wrreq : in  std_logic;
        empty : out std_logic;
        full  : out std_logic;
        q     : out std_logic_vector(lpm_width-1 downto 0);
        usedw : out std_logic_vector(lpm_widthu-1 downto 0)
    );
end entity scfifo;

architecture sim of scfifo is
    type fifo_mem_t is array (0 to lpm_numwords-1) of std_logic_vector(lpm_width-1 downto 0);

    signal fifo_mem    : fifo_mem_t := (others => (others => '0'));
    signal wr_ptr      : natural range 0 to lpm_numwords-1 := 0;
    signal rd_ptr      : natural range 0 to lpm_numwords-1 := 0;
    signal count       : natural range 0 to lpm_numwords   := 0;

    function incr_ptr(ptr_v : natural) return natural is
    begin
        if ptr_v = lpm_numwords - 1 then
            return 0;
        end if;
        return ptr_v + 1;
    end function incr_ptr;

    function encode_usedw(count_v : natural; width_v : natural) return std_logic_vector is
        variable max_count_v : natural;
        variable sat_count_v : natural;
    begin
        max_count_v := (2 ** width_v) - 1;
        if count_v > max_count_v then
            sat_count_v := max_count_v;
        else
            sat_count_v := count_v;
        end if;
        return std_logic_vector(to_unsigned(sat_count_v, width_v));
    end function encode_usedw;
begin
    fifo_reg : process (clock)
        variable wr_ptr_v : natural range 0 to lpm_numwords-1;
        variable rd_ptr_v : natural range 0 to lpm_numwords-1;
        variable count_v  : natural range 0 to lpm_numwords;
    begin
        if rising_edge(clock) then
            if sclr = '1' then
                wr_ptr <= 0;
                rd_ptr <= 0;
                count  <= 0;
            else
                wr_ptr_v := wr_ptr;
                rd_ptr_v := rd_ptr;
                count_v  := count;

                if (wrreq = '1') and (count_v < lpm_numwords) then
                    fifo_mem(wr_ptr_v) <= data;
                    wr_ptr_v := incr_ptr(wr_ptr_v);
                    count_v  := count_v + 1;
                end if;

                if (rdreq = '1') and (count_v > 0) then
                    rd_ptr_v := incr_ptr(rd_ptr_v);
                    count_v  := count_v - 1;
                end if;

                wr_ptr <= wr_ptr_v;
                rd_ptr <= rd_ptr_v;
                count  <= count_v;
            end if;
        end if;
    end process fifo_reg;

    q     <= fifo_mem(rd_ptr) when count > 0 else (others => '0');
    empty <= '1' when count = 0 else '0';
    full  <= '1' when count = lpm_numwords else '0';
    -- Intel's generated wrappers use widthu=log2(depth), so the exact-full
    -- count is not representable. Saturate the compat-model debug port
    -- instead of emitting simulator truncation warnings at full occupancy.
    usedw <= encode_usedw(count, usedw'length);
end architecture sim;

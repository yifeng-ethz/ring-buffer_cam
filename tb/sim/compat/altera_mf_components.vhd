library ieee;
use ieee.std_logic_1164.all;

package altera_mf_components is

    component scfifo is
        generic (
            add_ram_output_register    : string  := "ON";
            intended_device_family     : string  := "MAX 10";
            lpm_numwords               : natural := 512;
            lpm_showahead              : string  := "ON";
            lpm_type                   : string  := "scfifo";
            lpm_width                  : natural := 8;
            lpm_widthu                 : natural := 9;
            overflow_checking          : string  := "ON";
            underflow_checking         : string  := "ON";
            use_eab                    : string  := "ON"
        );
        port (
            aclr                       : in  std_logic;
            clock                      : in  std_logic;
            data                       : in  std_logic_vector(lpm_width-1 downto 0);
            rdreq                      : in  std_logic;
            sclr                       : in  std_logic;
            wrreq                      : in  std_logic;
            empty                      : out std_logic;
            full                       : out std_logic;
            q                          : out std_logic_vector(lpm_width-1 downto 0)
        );
    end component scfifo;

    component dcfifo is
        generic (
            intended_device_family     : string  := "Arria V";
            lpm_hint                   : string  := "";
            lpm_numwords               : natural := 128;
            lpm_showahead              : string  := "ON";
            lpm_type                   : string  := "dcfifo";
            lpm_width                  : natural := 8;
            lpm_widthu                 : natural := 7;
            overflow_checking          : string  := "ON";
            rdsync_delaypipe           : natural := 4;
            read_aclr_synch            : string  := "ON";
            underflow_checking         : string  := "ON";
            use_eab                    : string  := "ON";
            write_aclr_synch           : string  := "ON";
            wrsync_delaypipe           : natural := 4
        );
        port (
            aclr                       : in  std_logic;
            data                       : in  std_logic_vector(lpm_width-1 downto 0);
            rdclk                      : in  std_logic;
            rdreq                      : in  std_logic;
            wrclk                      : in  std_logic;
            wrreq                      : in  std_logic;
            q                          : out std_logic_vector(lpm_width-1 downto 0);
            rdempty                    : out std_logic;
            rdfull                     : out std_logic;
            wrempty                    : out std_logic;
            wrfull                     : out std_logic
        );
    end component dcfifo;

end package altera_mf_components;

package body altera_mf_components is
end package body altera_mf_components;

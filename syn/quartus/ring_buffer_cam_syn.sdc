create_clock -name clk125 -period 8.000 [get_ports {clk125}]

set_false_path -from [get_ports {reset_n}]

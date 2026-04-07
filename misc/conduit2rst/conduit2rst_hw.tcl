package require -exact qsys 16.1

set_module_property DESCRIPTION ""
set_module_property NAME conduit2rst
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "Mu3e Data Plane/Debug"
set_module_property AUTHOR "Yifeng Wang"
set_module_property DISPLAY_NAME "Conduit to Reset Interface (Debug Module)"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false

add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL conduit2rst
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file conduit2rst.vhd VHDL PATH conduit2rst.vhd TOP_LEVEL_FILE

add_parameter DEBUG NATURAL 1
set_parameter_property DEBUG DEFAULT_VALUE 1
set_parameter_property DEBUG DISPLAY_NAME DEBUG
set_parameter_property DEBUG TYPE NATURAL
set_parameter_property DEBUG UNITS None
set_parameter_property DEBUG ALLOWED_RANGES 0:2147483647
set_parameter_property DEBUG HDL_PARAMETER true

add_interface reset_in conduit end
set_interface_property reset_in associatedClock dummy_clock
set_interface_property reset_in associatedReset ""
set_interface_property reset_in ENABLED true
add_interface_port reset_in i_rst source Input 1

add_interface dummy_clock clock end
set_interface_property dummy_clock clockRate 0
set_interface_property dummy_clock ENABLED true
add_interface_port dummy_clock i_clk clk Input 1

add_interface reset_source reset start
set_interface_property reset_source associatedClock dummy_clock
set_interface_property reset_source associatedDirectReset ""
set_interface_property reset_source associatedResetSinks ""
set_interface_property reset_source synchronousEdges DEASSERT
set_interface_property reset_source ENABLED true
add_interface_port reset_source o_rst reset Output 1

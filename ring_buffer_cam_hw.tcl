
# 
# ring_buffer_cam "Ring-buffer Shaped Content-Addressable-Memory (CAM)" v1.3.14
# Yifeng Wang 2024.08.10.14:51:40
# This cam is implemented in a special shape of a ring-buffer, where write pointer (wr_ptr) is ever mono-increasing and the read pointer (rd_ptr) is controlled by the look-up result address. The look-up is supported by the CAM natively.
# 

# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1


# 
# module ring_buffer_cam
# 
set_module_property DESCRIPTION "This cam is implemented in a special shape of a ring-buffer, where write pointer (wr_ptr) is ever mono-increasing and the read pointer (rd_ptr) is controlled by the look-up result address. The look-up is supported by the CAM natively."
set_module_property NAME ring_buffer_cam
set_module_property VERSION 24.0.1210
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "Mu3e Data Plane/Modules"
set_module_property AUTHOR "Yifeng Wang"
set_module_property DISPLAY_NAME "Ring-buffer Shaped Content-Addressable-Memory (CAM)"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false


################################################
# file sets
################################################
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL ring_buffer_cam
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
# +-----+
# | top |
# +-----+
add_fileset_file ring_buffer_cam.vhd VHDL PATH ring_buffer_cam.vhd TOP_LEVEL_FILE
# +-------------+
# | cam complex |
# +-------------+
# cam complex - side ram, address linked to cam, for storing data segment 
add_fileset_file alt_simple_dpram.vhd VHDL PATH alt_simple_dpram.vhd
# cam complex - cam top, including mod_ctl, mod_data and lut ports
add_fileset_file cam_mem_a5.vhd VHDL PATH cam_mem_a5.vhd
# cam complex - cam top - cam implementation, in M10K (Arria V)
add_fileset_file cam_mem_blk_a5.vhd VHDL PATH cam_mem_blk_a5.vhd
# +-------+
# | fifos |
# +-------+
# deassembly fifo, storing incoming hit from processor
add_fileset_file scfifo_w40d256.vhd VHDL PATH alt_fifo/scfifo_w40d256.vhd
# pop command fifo, storing the ts of subheader to be search and poped, if any
add_fileset_file cmd_fifo.vhd VHDL PATH alt_fifo/cmd_fifo/cmd_fifo.vhd
# +------+
# | misc |
# +------+
# decoder (binary -> onehot), for pop/erase address 
add_fileset_file b2o_encoder.v VERILOG PATH b2o_encoder.v
# encoder (onehot -> binary), for search result address
add_fileset_file addr_enc_logic_small.vhd VHDL PATH addr_enc_logic_small.vhd


# 
# parameters
# 
add_parameter SEARCH_KEY_WIDTH NATURAL 8
set_parameter_property SEARCH_KEY_WIDTH DEFAULT_VALUE 8
set_parameter_property SEARCH_KEY_WIDTH DISPLAY_NAME SEARCH_KEY_WIDTH
set_parameter_property SEARCH_KEY_WIDTH TYPE NATURAL
set_parameter_property SEARCH_KEY_WIDTH UNITS None
set_parameter_property SEARCH_KEY_WIDTH ALLOWED_RANGES 0:2147483647
set_parameter_property SEARCH_KEY_WIDTH HDL_PARAMETER true
add_parameter RING_BUFFER_N_ENTRY NATURAL 512
set_parameter_property RING_BUFFER_N_ENTRY DEFAULT_VALUE 512
set_parameter_property RING_BUFFER_N_ENTRY DISPLAY_NAME RING_BUFFER_N_ENTRY
set_parameter_property RING_BUFFER_N_ENTRY TYPE NATURAL
set_parameter_property RING_BUFFER_N_ENTRY UNITS None
set_parameter_property RING_BUFFER_N_ENTRY ALLOWED_RANGES 0:2147483647
set_parameter_property RING_BUFFER_N_ENTRY HDL_PARAMETER true
add_parameter SIDE_DATA_BITS NATURAL 31
set_parameter_property SIDE_DATA_BITS DEFAULT_VALUE 31
set_parameter_property SIDE_DATA_BITS DISPLAY_NAME SIDE_DATA_BITS
set_parameter_property SIDE_DATA_BITS TYPE NATURAL
set_parameter_property SIDE_DATA_BITS UNITS None
set_parameter_property SIDE_DATA_BITS ALLOWED_RANGES 0:2147483647
set_parameter_property SIDE_DATA_BITS HDL_PARAMETER true
add_parameter INTERLEAVING_FACTOR NATURAL 4
set_parameter_property INTERLEAVING_FACTOR DEFAULT_VALUE 4
set_parameter_property INTERLEAVING_FACTOR DISPLAY_NAME INTERLEAVING_FACTOR
set_parameter_property INTERLEAVING_FACTOR TYPE NATURAL
set_parameter_property INTERLEAVING_FACTOR UNITS None
set_parameter_property INTERLEAVING_FACTOR ALLOWED_RANGES 0:2147483647
set_parameter_property INTERLEAVING_FACTOR HDL_PARAMETER true
add_parameter INTERLEAVING_INDEX NATURAL 0
set_parameter_property INTERLEAVING_INDEX DEFAULT_VALUE 0
set_parameter_property INTERLEAVING_INDEX DISPLAY_NAME INTERLEAVING_INDEX
set_parameter_property INTERLEAVING_INDEX TYPE NATURAL
set_parameter_property INTERLEAVING_INDEX UNITS None
set_parameter_property INTERLEAVING_INDEX ALLOWED_RANGES 0:2147483647
set_parameter_property INTERLEAVING_INDEX HDL_PARAMETER true
add_parameter DEBUG NATURAL 1
set_parameter_property DEBUG DEFAULT_VALUE 1
set_parameter_property DEBUG DISPLAY_NAME DEBUG
set_parameter_property DEBUG TYPE NATURAL
set_parameter_property DEBUG UNITS None
set_parameter_property DEBUG ALLOWED_RANGES 0:2147483647
set_parameter_property DEBUG HDL_PARAMETER true


# 
# display items
# 


# 
# connection point csr
# 
add_interface csr avalon end
set_interface_property csr addressUnits WORDS
set_interface_property csr associatedClock clock_interface
set_interface_property csr associatedReset reset_interface
set_interface_property csr bitsPerSymbol 8
set_interface_property csr burstOnBurstBoundariesOnly false
set_interface_property csr burstcountUnits WORDS
set_interface_property csr explicitAddressSpan 0
set_interface_property csr holdTime 0
set_interface_property csr linewrapBursts false
set_interface_property csr maximumPendingReadTransactions 0
set_interface_property csr maximumPendingWriteTransactions 0
set_interface_property csr readLatency 0
set_interface_property csr readWaitTime 1
set_interface_property csr setupTime 0
set_interface_property csr timingUnits Cycles
set_interface_property csr writeWaitTime 0
set_interface_property csr ENABLED true
set_interface_property csr EXPORT_OF ""
set_interface_property csr PORT_NAME_MAP ""
set_interface_property csr CMSIS_SVD_VARIABLES ""
set_interface_property csr SVD_ADDRESS_GROUP ""

add_interface_port csr avs_csr_readdata readdata Output 32
add_interface_port csr avs_csr_read read Input 1
add_interface_port csr avs_csr_address address Input 5
add_interface_port csr avs_csr_waitrequest waitrequest Output 1
add_interface_port csr avs_csr_write write Input 1
add_interface_port csr avs_csr_writedata writedata Input 32
set_interface_assignment csr embeddedsw.configuration.isFlash 0
set_interface_assignment csr embeddedsw.configuration.isMemoryDevice 0
set_interface_assignment csr embeddedsw.configuration.isNonVolatileStorage 0
set_interface_assignment csr embeddedsw.configuration.isPrintableDevice 0


# 
# connection point hit_type1
# 
add_interface hit_type1 avalon_streaming end
set_interface_property hit_type1 associatedClock clock_interface
set_interface_property hit_type1 associatedReset reset_interface
set_interface_property hit_type1 dataBitsPerSymbol 39
set_interface_property hit_type1 errorDescriptor {"tserr"}
set_interface_property hit_type1 firstSymbolInHighOrderBits true
set_interface_property hit_type1 maxChannel 15
set_interface_property hit_type1 readyLatency 0
set_interface_property hit_type1 ENABLED true
set_interface_property hit_type1 EXPORT_OF ""
set_interface_property hit_type1 PORT_NAME_MAP ""
set_interface_property hit_type1 CMSIS_SVD_VARIABLES ""
set_interface_property hit_type1 SVD_ADDRESS_GROUP ""

add_interface_port hit_type1 asi_hit_type1_channel channel Input 4
add_interface_port hit_type1 asi_hit_type1_startofpacket startofpacket Input 1
add_interface_port hit_type1 asi_hit_type1_endofpacket endofpacket Input 1
add_interface_port hit_type1 asi_hit_type1_data data Input 39
add_interface_port hit_type1 asi_hit_type1_valid valid Input 1
add_interface_port hit_type1 asi_hit_type1_ready ready Output 1
add_interface_port hit_type1 asi_hit_type1_error error Input 1


# 
# connection point hit_type2
# 
add_interface hit_type2 avalon_streaming start
set_interface_property hit_type2 associatedClock clock_interface
set_interface_property hit_type2 associatedReset reset_interface
set_interface_property hit_type2 dataBitsPerSymbol 36
set_interface_property hit_type2 errorDescriptor {"tsglitcherr"}
set_interface_property hit_type2 firstSymbolInHighOrderBits true
set_interface_property hit_type2 maxChannel 15
set_interface_property hit_type2 readyLatency 0
set_interface_property hit_type2 ENABLED true
set_interface_property hit_type2 EXPORT_OF ""
set_interface_property hit_type2 PORT_NAME_MAP ""
set_interface_property hit_type2 CMSIS_SVD_VARIABLES ""
set_interface_property hit_type2 SVD_ADDRESS_GROUP ""

add_interface_port hit_type2 aso_hit_type2_channel channel Output 4
add_interface_port hit_type2 aso_hit_type2_startofpacket startofpacket Output 1
add_interface_port hit_type2 aso_hit_type2_endofpacket endofpacket Output 1
add_interface_port hit_type2 aso_hit_type2_data data Output 36
add_interface_port hit_type2 aso_hit_type2_valid valid Output 1
add_interface_port hit_type2 aso_hit_type2_ready ready Input 1
add_interface_port hit_type2 aso_hit_type2_error error Output 1


# 
# connection point clock_interface
# 
add_interface clock_interface clock end
set_interface_property clock_interface clockRate 0
set_interface_property clock_interface ENABLED true
set_interface_property clock_interface EXPORT_OF ""
set_interface_property clock_interface PORT_NAME_MAP ""
set_interface_property clock_interface CMSIS_SVD_VARIABLES ""
set_interface_property clock_interface SVD_ADDRESS_GROUP ""

add_interface_port clock_interface i_clk clk Input 1


# 
# connection point reset_interface
# 
add_interface reset_interface reset end
set_interface_property reset_interface associatedClock clock_interface
set_interface_property reset_interface synchronousEdges DEASSERT
set_interface_property reset_interface ENABLED true
set_interface_property reset_interface EXPORT_OF ""
set_interface_property reset_interface PORT_NAME_MAP ""
set_interface_property reset_interface CMSIS_SVD_VARIABLES ""
set_interface_property reset_interface SVD_ADDRESS_GROUP ""

add_interface_port reset_interface i_rst reset Input 1


# 
# connection point run_control
# 
add_interface run_control avalon_streaming end
set_interface_property run_control associatedClock clock_interface
set_interface_property run_control associatedReset reset_interface
set_interface_property run_control dataBitsPerSymbol 9
set_interface_property run_control errorDescriptor ""
set_interface_property run_control firstSymbolInHighOrderBits true
set_interface_property run_control maxChannel 0
set_interface_property run_control readyLatency 0
set_interface_property run_control ENABLED true
set_interface_property run_control EXPORT_OF ""
set_interface_property run_control PORT_NAME_MAP ""
set_interface_property run_control CMSIS_SVD_VARIABLES ""
set_interface_property run_control SVD_ADDRESS_GROUP ""

add_interface_port run_control asi_ctrl_data data Input 9
add_interface_port run_control asi_ctrl_valid valid Input 1
add_interface_port run_control asi_ctrl_ready ready Output 1


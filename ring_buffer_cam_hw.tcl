
################################################
# File description
################################################
# ring_buffer_cam "Ring-buffer Shaped Content-Addressable-Memory (CAM)" v2.1
# Yifeng Wang 2024.08.10.14:51:40
# Please refer to Patent documentation of this IP for its detail technical operation explaination.
# 

################################################
# History 
################################################
# 25.0.0228 - add <filllevel> interface 
# 25.0.0321 - add IP wrapping documentations
# 25.0.0324 - clear up csr
# 25.0.0813 - fixed bug of gts overflow due to casted to int32 type

################################################
# request TCL package from ACDS 16.1
################################################
package require qsys


################################################
# module ring_buffer_cam
################################################
set_module_property DESCRIPTION "This cam is implemented in a special shape of a ring-buffer, where write pointer (wr_ptr) is ever mono-increasing in address and the read pointer (rd_ptr) is in timestamp/rank. The look-up is supported by the CAM natively."
set_module_property NAME ring_buffer_cam
set_module_property VERSION 25.0.0813
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "Mu3e Data Plane/Modules"
set_module_property AUTHOR "Yifeng Wang"
set_module_property DISPLAY_NAME "Ring-buffer Shaped Content-Addressable-Memory (Ring-CAM)"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false
set_module_property ELABORATION_CALLBACK myelaborate


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


#######################################################################
# parameters
#######################################################################

# +------------------+
# | SEARCH_KEY_WIDTH |
# +------------------+
add_parameter SEARCH_KEY_WIDTH NATURAL 8
set_parameter_property SEARCH_KEY_WIDTH DISPLAY_NAME "Search key width"
set_parameter_property SEARCH_KEY_WIDTH TYPE NATURAL
set_parameter_property SEARCH_KEY_WIDTH UNITS Bits
set_parameter_property SEARCH_KEY_WIDTH ALLOWED_RANGES 1:512
set_parameter_property SEARCH_KEY_WIDTH HDL_PARAMETER true
set dscpt \
"<html>
Set the width of the search key for the CAM. <br>
This parameter also affects the granularity of the sorting. In the default case (see below), the resulting sorting granularity is 2^4*8ns = 128 ns. <br>
If finer granularity is desired, it is advisable to 1) enlarge the search key width (resource of CAM = key width / 8) and/or 2) use lower bits, ex: \[11:4\] -> \[7:0\]. In either case, you have to also tune the subheader generation epoch. 
<br><br> Default : timestamp length (8ns) \[11:4\]
</html>"
set_parameter_property SEARCH_KEY_WIDTH LONG_DESCRIPTION $dscpt
set_parameter_property SEARCH_KEY_WIDTH DESCRIPTION $dscpt

# +---------------------+
# | RING_BUFFER_N_ENTRY |
# +---------------------+
add_parameter RING_BUFFER_N_ENTRY NATURAL 32
set_parameter_property RING_BUFFER_N_ENTRY DISPLAY_NAME "Capacity of Ring-CAM (entries)"
set_parameter_property RING_BUFFER_N_ENTRY UNITS None
set_parameter_property RING_BUFFER_N_ENTRY HDL_PARAMETER true
set dscpt \
"<html>
<ul>
</ul>
Set the capacity (<b>C</b>) of the Ring-CAM IP. <br> <br>
<b>Resource comsumption on Arria V (quantity of core parts, excluding elastic FIFOs): </b> 
<ul>
    <li> <b>CAM</b> : <b>C</b> mod 32 </li>
    <li> <b>RAM</b> : <b>C</b> mod 256 </li>
    <li> <b>ALM</b> (for address encoding) : <b>C</b> mod 64 x 100. we use one onehot-to-binary encoder (~= 100 ALM) for every 64 bits of output from CAM. </li> 
    
</ul>
<br>
<b>How to derive the maximum required capacity:</b> <br>
C must be set to larger than Reording Byte Offset (<b>RBO</b>), which is calculated as Reordering Time Offset (<b>RTO</b>) * arrival curve (<b>T</b>), where <b>T</b> is the Timeout value = 2000 cycles by default. <br>
In short, the C must be able to accomodate the maximum flow rate * maximum detain time of the packet in the ring-CAM. <br>
Refer to Wengen 2025 talk on \"new sorter - Yifeng Wang\" or \" On Packet Reordering in Time-Sensitive Networks - J. Le Boudec et al.\" or RFC 4737 for explaination and proof. 
<br><br> Default : <b>C</b> = 32 (test) <b>C</b> = 1024 (4-way; MuTRiG)
</html>"
set_parameter_property RING_BUFFER_N_ENTRY LONG_DESCRIPTION $dscpt
set_parameter_property RING_BUFFER_N_ENTRY DESCRIPTION $dscpt

# +----------------+
# | SIDE_DATA_BITS |
# +----------------+
add_parameter SIDE_DATA_BITS NATURAL 31
set_parameter_property SIDE_DATA_BITS DISPLAY_NAME "Side band data width"
set_parameter_property SIDE_DATA_BITS TYPE NATURAL
set_parameter_property SIDE_DATA_BITS UNITS Bits
set_parameter_property SIDE_DATA_BITS ALLOWED_RANGES 0:512
set_parameter_property SIDE_DATA_BITS HDL_PARAMETER true
set dscpt \
"<html>
Set the width of side band data in bits. <br>
For simplicity, the side band data width = the ingress data width - search key width. <br>
In other words, the data that is not stored in CAM, should go to RAM. It will be read out and assembled at the output to restore the original event information. 
<br><br> Default : 31 (<b>Type-1 Event</b>)
</html>"
set_parameter_property SIDE_DATA_BITS LONG_DESCRIPTION $dscpt
set_parameter_property SIDE_DATA_BITS DESCRIPTION $dscpt

# +---------------------+
# | INTERLEAVING_FACTOR |
# +---------------------+
add_parameter INTERLEAVING_FACTOR NATURAL 4
set_parameter_property INTERLEAVING_FACTOR DISPLAY_NAME "Time interleaving factor"
set_parameter_property INTERLEAVING_FACTOR TYPE NATURAL
set_parameter_property INTERLEAVING_FACTOR UNITS None
set_parameter_property INTERLEAVING_FACTOR HDL_PARAMETER true
set dscpt \
"<html>
Set the time interleaving factor of the ring-CAM IP (must be same for all ring-CAM IP(s) in the same complex). <br>
The time interleaving factor also refers to the number of ways of a ring-CAM complex. <br>
Enabling time interleaving ability indicates the ring-CAM only intake events with lower bits equal to its <i>time interleaving index</i>. <br>

<br>
For example, when interleaving factor is 4, the lower 2 bits of the timestamp are used derive acceptance criteria. <br>

<br>
This construction allows for reducing the backlog by the burst of time adjacent events, by allocating to different ring-CAM entities. <br>
By setting it to > 1, you enable this option, thus the ring-CAM complex can operate at above line-rate. <br> <br>

<b>Note</b> : alternative hash function (other than mod INTERLEAVING_FACTOR) can be used in case of uneven timestamp distribution. <br>

<b> Throughput </b> : 1/2 (push-pop) or 1/3 (overwrite) for each ring-CAM entity, boosted proportionally as the interleaving factor. <br>
This method of parallelism helps to preserve packet ordering, as the downstream could use order-preserving arbiter to grant the packets in mono-increasing timestamp. <br>
</html>"
set_parameter_property INTERLEAVING_FACTOR LONG_DESCRIPTION $dscpt
set_parameter_property INTERLEAVING_FACTOR DESCRIPTION $dscpt

# +--------------------+
# | INTERLEAVING_INDEX |
# +--------------------+
add_parameter INTERLEAVING_INDEX NATURAL 0
set_parameter_property INTERLEAVING_INDEX DEFAULT_VALUE 0
set_parameter_property INTERLEAVING_INDEX DISPLAY_NAME "Time interleaving index"
set_parameter_property INTERLEAVING_INDEX TYPE NATURAL
set_parameter_property INTERLEAVING_INDEX UNITS None
set_parameter_property INTERLEAVING_INDEX HDL_PARAMETER true
set_parameter_property INTERLEAVING_INDEX ALLOWED_RANGES 0:64
set dscpt \
"<html>
Set the interleaving index of this ring-CAM IP (should be different for all ring-CAM IP(s) in the same complex). <br>
Only events with lower bits equal to this index will be accepted by this entity. <br>
</html>"
set_parameter_property INTERLEAVING_INDEX LONG_DESCRIPTION $dscpt
set_parameter_property INTERLEAVING_INDEX DESCRIPTION $dscpt

# +-------+
# | DEBUG |
# +-------+
add_parameter DEBUG NATURAL 1
set_parameter_property DEBUG DEFAULT_VALUE 1
set_parameter_property DEBUG DISPLAY_NAME "Debug Level"
set_parameter_property DEBUG TYPE NATURAL
set_parameter_property DEBUG UNITS None
set_parameter_property DEBUG ALLOWED_RANGES 0:2
set_parameter_property DEBUG HDL_PARAMETER true
set dscpt \
"<html>
Select the debug level of the IP (affects generation).<br>
<ul>
	<li><b>0</b> : off <br> </li>
	<li><b>1</b> : on, synthesizble <br> </li>
	<li><b>2</b> : on, non-synthesizble, simulation-only <br> </li>
</ul>
</html>"
set_parameter_property DEBUG LONG_DESCRIPTION $dscpt
set_parameter_property DEBUG DESCRIPTION $dscpt


################################################################
# display items
################################################################
# +------------+
# | IP Setting |
# +------------+
add_display_item "" "IP Setting" GROUP ""
# IP Setting - Basic
add_display_item "IP Setting" "Basic" GROUP ""
add_display_item "Basic" SEARCH_KEY_WIDTH PARAMETER 
add_display_item "Basic" RING_BUFFER_N_ENTRY PARAMETER
add_display_item "Basic" SIDE_DATA_BITS PARAMETER 
# IP Setting - Interleaving
add_display_item "IP Setting" "Interleaving" GROUP ""
add_display_item "Interleaving" INTERLEAVING_FACTOR PARAMETER 
add_display_item "Interleaving" INTERLEAVING_INDEX PARAMETER 
# IP Setting - Generation
add_display_item "IP Setting" "Generation" GROUP ""
add_display_item "Generation" DEBUG PARAMETER 
# +-------------+
# | Description |
# +-------------+
add_display_item "" "Description" GROUP ""
# Description - Interfaces
add_display_item "Description" "Interfaces" Group ""
# Description - Interfaces - Ingress
add_display_item "Interfaces" "Ingress" Group ""
set dscpt \
"<html>
Data format: <br>
<b>Type-1 hit event (<b>short</b>/long)</b> : <br> 
<ul>
<li>\[38:35\] : asic id \[3:0\] </li>
<li>\[34:30\] : channel id \[4:0\] </li>
<li>\[29:17\] : tcc8n \[12:0\] </li>
<li>\[16:14\] : tcc1n6 \[2:0\] </li>
<li>\[13: 9\] : tfine \[4:0\] (50 ps resolution)</li>
<li>\[8 : 0\] : et1n6 \[8:0\] (invalid (=0) for short-subtype, valid for long-subtype)</li>
</ul>
</html>"
add_display_item "Ingress" "ingress_text" text $dscpt
# Description - Interfaces - Egress
add_display_item "Interfaces" "Egress" Group ""
set dscpt \
"<html>
Data format: <br>
<b>Type-2 hit event</b> : <br> 
<ul>
<li>\[35:32\] : byte_is_k \[3:0\] </li>
<li>\[31:28\] : timestamp \[3:0\] (offset in the subheader)</li>
<li>\[27:22\] : asic id \[5:0\] (=ingress <i>asic id</i> \[3:0\])</li>
<li>\[21:17\] : channel id \[4:0\] </li>
<li>\[16: 9\] : ts50p \[7:0\] (50 ps resolution = concat of <i>tcc1n6</i> and <i>tfine</i>)</li>
<li>\[8 : 0\] : et1n6 \[8:0\] (invalid (=0) for short-subtype, valid for long-subtype)</li>
</ul>
</html>"
add_display_item "Egress" "egress_text" text $dscpt


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

# 
# connection point filllevel
# 
add_interface filllevel avalon_streaming start
set_interface_property filllevel associatedClock clock_interface
set_interface_property filllevel associatedReset reset_interface
set_interface_property filllevel dataBitsPerSymbol 16
set_interface_property filllevel errorDescriptor ""
set_interface_property filllevel firstSymbolInHighOrderBits true
set_interface_property filllevel maxChannel 0
set_interface_property filllevel readyLatency 0
set_interface_property filllevel ENABLED true
set_interface_property filllevel EXPORT_OF ""
set_interface_property filllevel PORT_NAME_MAP ""
set_interface_property filllevel CMSIS_SVD_VARIABLES ""
set_interface_property filllevel SVD_ADDRESS_GROUP ""

add_interface_port filllevel aso_filllevel_data data Output 16
add_interface_port filllevel aso_filllevel_valid valid Output 1

proc myelaborate {} {
	# set parameter sophisticated ranges 
    # 1)
    for {set i 5} {$i < 12} {incr i} {
        lappend ring_buffer_n_entry_ranges [expr 2**$i]
    }
	set_parameter_property RING_BUFFER_N_ENTRY ALLOWED_RANGES $ring_buffer_n_entry_ranges
    # 2) 
    for {set i 0} {$i < 6} {incr i} {
        lappend interleaving_factor_ranges [expr 2**$i]
    }
    set_parameter_property INTERLEAVING_FACTOR ALLOWED_RANGES $interleaving_factor_ranges
    # 2) 
    set interleaving_index_max [expr [get_parameter_value INTERLEAVING_FACTOR] - 1]
    set_parameter_property INTERLEAVING_INDEX ALLOWED_RANGES 0:$interleaving_index_max
    
	return -code ok
}

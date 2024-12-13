################################################
# lvds_rx_controller_pro "LVDS RX Controller Pro" v24.0.1110
# Yifeng Wang 2024.10.11
################################################

################################################
# request TCL package from ACDS 16.1
################################################ 
package require qsys 16.1
# custom macro, for building .hdl.terp -> .hdl
# loc: $::env(QUARTUS_ROOTDIR)/../ip/altera/common/hw_tcl_packages/altera_terp.tcl
package require -exact altera_terp 1.0


################################################
# module lvds_rx_controller_pro
################################################ 
set_module_property NAME lvds_rx_controller_pro
set_module_property VERSION 24.0.1110
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "Mu3e Data Plane/Modules"
set_module_property AUTHOR "Yifeng Wang"
set_module_property ICON_PATH ../figures/mu3e_logo.png
set_module_property DISPLAY_NAME "LVDS RX Controller Pro"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE false
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false
set_module_property ELABORATION_CALLBACK my_elaborate




################################################
# parameters
################################################ 
# derived
add_parameter DECODED_CHANNEL_WIDTH natural 
set_parameter_property DECODED_CHANNEL_WIDTH HDL_PARAMETER true
set_parameter_property DECODED_CHANNEL_WIDTH DERIVED true

# user-input
add_parameter N_LANE natural 9
set_parameter_property N_LANE HDL_PARAMETER true
set_parameter_property N_LANE DISPLAY_NAME "Number of data lanes to control"

add_parameter DECODED_USE_CHANNEL natural 1
set_parameter_property DECODED_USE_CHANNEL DISPLAY_NAME "Grouped output"
set_parameter_property DECODED_USE_CHANNEL ALLOWED_RANGES {"0: Enable" "1:Disable"} 
set_parameter_property DECODED_USE_CHANNEL DISPLAY_HINT "RADIO"
set_parameter_property DECODED_USE_CHANNEL HDL_PARAMETER true
set_parameter_property DECODED_USE_CHANNEL AFFECTS_ELABORATION true
set dscpt \
"<html>
Group t <br>
<ul>
    <li> <b>Enable</b>:  
            combine a single-port with all data concat. into large array, lower index in LSB. <br> </li>
    <li> <b>Disable</b>: 
            sepeate the data output into multiple interfaces with channel indicating their source.</li>
</ul>
</html>"
set_parameter_property DECODED_USE_CHANNEL LONG_DESCRIPTION $dscpt
set_parameter_property DECODED_USE_CHANNEL DESCRIPTION $dscpt


add_parameter AVMM_ADDR_W natural 4
set_parameter_property AVMM_ADDR_W HDL_PARAMETER true
set_parameter_property AVMM_ADDR_W DISPLAY_NAME "Address width of CSR interface"
set_parameter_property AVMM_ADDR_W UNITS Bits

add_parameter SYNC_PATTERN_SEL string 
set_parameter_property SYNC_PATTERN_SEL DISPLAY_NAME "Sync pattern of the byte boundary alignment"
set_parameter_property SYNC_PATTERN_SEL ALLOWED_RANGES {"0xFA: K28.5" "0xF4: K28.0" "0x3A8: K23.7"}

add_parameter SYNC_PATTERN std_logic_vector  
set_parameter_property SYNC_PATTERN WIDTH 10
set_parameter_property SYNC_PATTERN HDL_PARAMETER true
set_parameter_property SYNC_PATTERN DERIVED true
set_parameter_property SYNC_PATTERN VISIBLE 0



################################################
# ports
################################################ 
############
# redriver #
############
add_interface redriver conduit end
set_interface_property redriver associatedClock ""
set_interface_property redriver associatedReset ""
# ---------------------------------------------------------------------
# 	<interface name>	<signal name>			<role>		[<direction><width>]
# ---------------------------------------------------------------------
add_interface_port redriver coe_redriver_losn       losn        Input  "N_LANE"           





########
# ctrl #
########
add_interface ctrl conduit end
set_interface_property ctrl associatedClock ""
set_interface_property ctrl associatedReset ""
# ---------------------------------------------------------------------
# 	<interface name>	<signal name>			<role>		[<direction><width>]
# ---------------------------------------------------------------------
# pll-related
add_interface_port ctrl coe_ctrl_pllrst 		pllrst 		Output 1
add_interface_port ctrl coe_ctrl_plllock 		plllock 	Input 1
# dpa circuitry
add_interface_port ctrl coe_ctrl_dparst 		dparst 		Output "N_LANE"
add_interface_port ctrl coe_ctrl_lockrst		lockrst 	Output "N_LANE"
add_interface_port ctrl coe_ctrl_dpahold 		dpahold 	Output "N_LANE"
add_interface_port ctrl coe_ctrl_dpalock 		dpalock 	Input "N_LANE"
# dpa fifo
add_interface_port ctrl coe_ctrl_fiforst 		fiforst 	Output "N_LANE"
# bit-slip logic
add_interface_port ctrl coe_ctrl_bitslip 		bitslip 	Output "N_LANE"
add_interface_port ctrl coe_ctrl_rollover 		rollover	Input "N_LANE"

#######
# csr #
#######
add_interface csr avalon end
set_interface_property csr associatedClock "control_clock"
set_interface_property csr associatedReset "control_reset"
add_interface_port csr avs_csr_read 			read 		Input 1
add_interface_port csr avs_csr_readdata 		readdata 	Output 32
add_interface_port csr avs_csr_writedata 		writedata 	Input 32
add_interface_port csr avs_csr_address 			address 	Input "AVMM_ADDR_W"
add_interface_port csr avs_csr_waitrequest 		waitrequest Output 1
add_interface_port csr avs_csr_write 			write 		Input 1

#############
# `decoded` #
#############
# see my_elaborate

###############
# parallel... #
###############
add_interface parallel conduit end
set_interface_property parallel associatedClock "data_clock"
set_interface_property parallel associatedReset ""



#############################
# Clock and reset interface #
#############################
########
# data #
########
add_interface data_clock clock end 
set_interface_property data_clock clockRate 0
add_interface_port data_clock csi_data_clk 		clk			Input 1

add_interface data_reset reset end
set_interface_property data_reset associatedClock data_clock
set_interface_property data_reset synchronousEdges BOTH
add_interface_port data_reset rsi_data_reset	reset		Input 1

###########
# control #
###########
add_interface control_clock clock end
set_interface_property control_clock clockRate 0
add_interface_port control_clock csi_control_clk clk		Input 1

add_interface control_reset reset end
set_interface_property control_reset associatedClock control_clock
set_interface_property control_reset synchronousEdges BOTH
add_interface_port control_reset rsi_control_reset	reset	Input 1



################################################
# file sets
################################################ 
add_fileset synth   QUARTUS_SYNTH my_generate 

proc my_generate { output_name } {
    # checkout this //acds/rel/18.1std/ip/merlin/altera_merlin_router/altera_merlin_router_hw.tcl

    set template_file "lvds_rx_controller_pro.terp.vhd"


    set template    [ read [ open $template_file r ] ]

    set params(n_avst_out_ports)    			[ get_parameter_value N_LANE ]
    set params(use_channel_for_avst_out)        [ get_parameter_value DECODED_USE_CHANNEL ]

    set params(output_name) $output_name

    set result          [ altera_terp $template params ]

    add_fileset_file ${output_name}.vhd VHDL TEXT $result TOP_LEVEL_FILE
    #add_fileset_file ${output_name}.vhd VHDL PATH ${output_name}.vhd TOP_LEVEL_FILE
    add_fileset_file line_code_decoder_8b10b.v Verilog PATH line_code_decoder_8b10b.v
}




proc my_elaborate {} {
	# set hdl-parameters
	set_parameter_value "DECODED_CHANNEL_WIDTH" [expr ceil(log([get_parameter_value "N_LANE"])/(log(2)))]
	# add interface ports
    ###########
    # decoded #
    ###########
    if {![get_parameter_value DECODED_USE_CHANNEL]} {
        add_interface decoded avalon_streaming start
        set_interface_property decoded associatedClock "data_clock"
        set_interface_property decoded associatedReset "data_reset"
        set_interface_property decoded dataBitsPerSymbol [expr [get_parameter_value N_LANE]*9 ]
        add_interface_port decoded aso_decoded_data data Output [expr [get_parameter_value N_LANE]*9 ]
        ##add_interface_port decoded aso_decoded_valid valid Output 1
        ##add_interface_port decoded aso_decoded_ready ready Input 1
        add_interface_port decoded aso_decoded_error error Output [expr [get_parameter_value N_LANE]*3 ]
        set error_des [list]
        for {set i 0} {$i < [get_parameter_value N_LANE]} {incr i} {
            lappend error_des "lane${i}_loss_sync_pattern" "lane${i}_parity_error" "lane${i}_decode_error"
        }
        set_interface_property decoded errorDescriptor $error_des
    } else {
        for {set i 0} {$i < [get_parameter_value N_LANE]} {incr i} {
            add_interface decoded${i} avalon_streaming start
            set_interface_property decoded${i} associatedClock "data_clock"
            set_interface_property decoded${i} associatedReset "data_reset"
            set_interface_property decoded${i} dataBitsPerSymbol 9
            set_interface_property decoded${i} maxChannel [get_parameter_value N_LANE]
            add_interface_port decoded${i} aso_decoded${i}_data data Output 9
            ##add_interface_port decoded${i} aso_decoded${i}_valid valid Output 1
            ##add_interface_port decoded${i} aso_decoded${i}_ready ready Input 1
            add_interface_port decoded${i} aso_decoded${i}_channel channel Output "DECODED_CHANNEL_WIDTH"
            add_interface_port decoded${i} aso_decoded${i}_error error Output 3
            set_interface_property decoded${i} errorDescriptor "loss_sync_pattern parity_error decode_error"
        }
    }
    
    ###############
    # ...parallel #
    ###############
    add_interface_port parallel coe_parallel_data   data        Input [expr [get_parameter_value N_LANE]*10 ]
    
    
    
    ################
    # sync pattern #
    ################
    set_parameter_value "SYNC_PATTERN" [get_parameter_value "SYNC_PATTERN_SEL"]
    #set_parameter_value "SYNC_PATTERN" 0xfa
    #send_message INFO [get_parameter_value "SYNC_PATTERN_SEL"]
    #send_message INFO [get_parameter_value "SYNC_PATTERN"]
    
    ## test
    send_message INFO "lvds_rx_controller_pro: file generation successful!"
    
    
    return -code ok 
	
}





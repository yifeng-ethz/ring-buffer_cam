
################################################
# altera_lvds_rx_28nm "Altera LVDS RX (28nm)" v24.0.1110
# Yifeng Wang 2024.10.11
################################################

################################################
# request TCL package from ACDS 16.1
################################################ 
package require qsys 16.1


################################################
# module altera_lvds_rx_28nm
################################################ 
set_module_property DESCRIPTION \
"<html>
Wrapper of the <b><i>ALTLVDS_RX</i></b> megafunction. <br>
This IP is preset for Mu3e Experiment with following constrains: 
<ul>
	<li> operate only in <b>DPA-FIFO</b> mode; <i> use SOFT-CDR mode requires a dedicated pin adjustment</i> </li>
	<li> nicely grouped interfaces </li>
	<li> data rate and clock rate fixed </li>
	<li> use <b>internal PLL</b> </li>
</ul>
Usage:
<ul>
	<li> connect <b><i>serial</i></b> interface => fast data input </li>
	<li> connect <b><i>parallel</i></b> interface => lvds controller, so it can decode 10b data into 8b </li>
	<li> connect <b><i>ctrl</i></b> interface => lvds controller </li>
</ul>

<br>
Remarks: <br>
<ul>
	<li> System-use: connects to <b>lvds_rx_controller_pro</b> in normal operations. <br> </li>
	<li> Standalone-use: requires your own implementation of controller to perform decoding and bitslip. <br> </li>
	<li> <b>IMPORTANT</b>: To prevent deadlock, the external lvds controller IP must NOT reply on <i>outclock</i>, because it will not be running after you assert <i>pllrst</i>.  </li>
</ul>

<br>
Data flow(clock domain): <br>
DPA-circuitry(IOPLL) --1b--> DPA FIFO(IOPLL) --1b--> Bit Slip(IOPLL) --1b--> Deserilizer(IOPLL|outclock) --10b--> <br>
<br>
</html>"
set_module_property NAME altera_lvds_rx_28nm
set_module_property VERSION 24.0.1110
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "Mu3e Data Plane/Modules"
set_module_property AUTHOR "Yifeng Wang"
set_module_property ICON_PATH ../figures/mu3e_logo.png
set_module_property DISPLAY_NAME "Altera LVDS RX (28nm)"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE false
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false
set_module_property ELABORATION_CALLBACK my_elaborate


################################################
# file sets
################################################ 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL altera_lvds_rx_28nm
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file altera_lvds_rx_28nm.vhd VHDL PATH altera_lvds_rx_28nm.vhd TOP_LEVEL_FILE


################################################ 
# parameters
################################################ 
# Reference for html codes used in this section
 # ----------------------------------------------
 # &lt = less than (<)
 # &gt = greater than (>)
 # <b></b> = bold text
 # <ul></ul> = defines an unordered list
 # <li></li> = bullet list
 # <br> = line break
add_parameter "N_LANE" natural 9
set_parameter_property "N_LANE" DISPLAY_NAME "Number of RX lanes"
set range_list [list]
for {set i 1} {$i <= 24} {incr i} {
	lappend range_list $i 
}
set_parameter_property "N_LANE" ALLOWED_RANGES $range_list
set_parameter_property "N_LANE" HDL_PARAMETER true
set dscpt \
"Set the number of RX lanes. You are free to choose up to 24, however the input pins must be on the same semi-chip, otherwise fitter will report PLL placement error."
set_parameter_property "N_LANE" LONG_DESCRIPTION $dscpt
set_parameter_property "N_LANE" DESCRIPTION $dscpt

add_parameter "DATA_WIDTH" natural 
set_parameter_property "DATA_WIDTH" HDL_PARAMETER false
set_parameter_property "DATA_WIDTH" DERIVED true
set_parameter_property "DATA_WIDTH" VISIBLE false 



################################################ 
# display items
################################################ 
# --------------------------------------------------------------- 
add_display_item "" "IP Setting" GROUP 
add_display_item "IP Setting" "N_LANE" parameter 







################################################ 
# connection point serial
################################################ 
add_interface serial conduit end
set_interface_property serial associatedClock inclock
			
add_interface_port serial rx_in data Input "N_LANE"


################################################ 
# connection point parallel
################################################ 
add_interface parallel conduit end 
set_interface_property parallel associatedClock outclock

add_interface_port parallel rx_out data Output "DATA_WIDTH"

################################################ 
# connection point ctrl
################################################ 
add_interface ctrl conduit end 
set_interface_property ctrl associatedClock ""
# ---------------------------------------------------------------------
# 	<interface name>	<signal name>			<role>		[<direction><width>]
# ---------------------------------------------------------------------
# pll-related
add_interface_port ctrl pll_areset 				pllrst 		Input 1
add_interface_port ctrl rx_locked 				plllock 	Output 1
# dpa circuitry
add_interface_port ctrl rx_reset 				dparst 		Input "N_LANE"
add_interface_port ctrl rx_dpa_lock_reset		lockrst 	Input "N_LANE"
add_interface_port ctrl rx_dpll_hold 			dpahold 	Input "N_LANE"
add_interface_port ctrl rx_dpa_locked 			dpalock 	Output "N_LANE"
# dpa fifo
add_interface_port ctrl rx_fifo_reset 			fiforst 	Input "N_LANE"
# bit-slip logic
add_interface_port ctrl rx_channel_data_align 	bitslip 	Input "N_LANE"
add_interface_port ctrl rx_cda_max 				rollover	Output "N_LANE"


################################################ 
# connection point inclock
################################################ 
add_interface inclock clock sink
set_interface_property inclock clockRate 125000000

add_interface_port inclock 		rx_inclock 		clk			Input 1

################################################ 
# connection point outclock
################################################ 
add_interface outclock clock source
set_interface_property outclock clockRate 125000000

add_interface_port outclock 	rx_outclock 	clk			Output 1



################################################
# callbacks
################################################
proc my_elaborate {} {
	
	#send_message INFO "<b>Hello World!</b> Byte~"
	set_parameter_value "DATA_WIDTH" [expr [get_parameter_value "N_LANE"] * 10]
	return -code ok
}





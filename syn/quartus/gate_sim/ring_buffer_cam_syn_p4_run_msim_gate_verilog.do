transcript on
if ![file isdirectory verilog_libs] {
	file mkdir verilog_libs
}

vlib verilog_libs/altera_lnsim_ver
vmap altera_lnsim_ver ./verilog_libs/altera_lnsim_ver
vlog -sv -work altera_lnsim_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/altera_lnsim.sv}

vlib verilog_libs/altera_ver
vmap altera_ver ./verilog_libs/altera_ver
vlog -vlog01compat -work altera_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/altera_primitives.v}

vlib verilog_libs/lpm_ver
vmap lpm_ver ./verilog_libs/lpm_ver
vlog -vlog01compat -work lpm_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/220model.v}

vlib verilog_libs/sgate_ver
vmap sgate_ver ./verilog_libs/sgate_ver
vlog -vlog01compat -work sgate_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/sgate.v}

vlib verilog_libs/altera_mf_ver
vmap altera_mf_ver ./verilog_libs/altera_mf_ver
vlog -vlog01compat -work altera_mf_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/altera_mf.v}

vlib verilog_libs/arriav_ver
vmap arriav_ver ./verilog_libs/arriav_ver
vlog -vlog01compat -work arriav_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/mentor/arriav_atoms_ncrypt.v}
vlog -vlog01compat -work arriav_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/mentor/arriav_hmi_atoms_ncrypt.v}
vlog -vlog01compat -work arriav_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/arriav_atoms.v}

vlib verilog_libs/arriav_hssi_ver
vmap arriav_hssi_ver ./verilog_libs/arriav_hssi_ver
vlog -vlog01compat -work arriav_hssi_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/mentor/arriav_hssi_atoms_ncrypt.v}
vlog -vlog01compat -work arriav_hssi_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/arriav_hssi_atoms.v}

vlib verilog_libs/arriav_pcie_hip_ver
vmap arriav_pcie_hip_ver ./verilog_libs/arriav_pcie_hip_ver
vlog -vlog01compat -work arriav_pcie_hip_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/mentor/arriav_pcie_hip_atoms_ncrypt.v}
vlog -vlog01compat -work arriav_pcie_hip_ver {/data1/intelFPGA/18.1/quartus/eda/sim_lib/arriav_pcie_hip_atoms.v}

if {[file exists gate_work]} {
	vdel -lib gate_work -all
}
vlib gate_work
vmap work gate_work

vlog -vlog01compat -work work +incdir+. {ring_buffer_cam_syn_p4.vo}


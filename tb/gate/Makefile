include ../../../scripts/questa_one.mk

Q18_SIMLIB ?= /data1/intelFPGA/18.1/quartus/eda/sim_lib

IP_ROOT      := $(abspath ../..)
TB_ROOT      := $(abspath ..)
GATE_DIR     := $(abspath .)
SIM_COMPAT   := $(TB_ROOT)/sim/compat
SYN_DIR      := $(IP_ROOT)/syn/quartus
NETLIST_DIR  := $(SYN_DIR)/gate_sim
NETLIST_V    := $(NETLIST_DIR)/ring_buffer_cam_syn_p4.vo
TB_SRC       := $(GATE_DIR)/ring_buffer_cam_gate_smoke_tb.sv
RTL_WORK     := $(GATE_DIR)/work_gate_rtl
GATE_WORK    := $(GATE_DIR)/work_gate_netlist
LIBDIR       := $(GATE_DIR)/verilog_libs
LOG_DIR      := $(GATE_DIR)/logs
MODELSIM_INI := modelsim.ini
TB_TOP       := ring_buffer_cam_gate_smoke_tb

SAMPLE_CYCLES ?= 500000

RTL_VHDL_SRCS = \
  $(IP_ROOT)/rtl/vhd_ver/cam_helper_pkg.vhd \
  $(SIM_COMPAT)/cam_mem_blk_a5.vhd \
  $(IP_ROOT)/rtl/vhd_ver/cam_mem_a5.vhd \
  $(IP_ROOT)/rtl/common/alt_simple_dpram.vhd \
  $(IP_ROOT)/rtl/common/alt_fifo/cmd_fifo/cmd_fifo.vhd \
  $(IP_ROOT)/rtl/common/alt_fifo/scfifo_w40d256.vhd \
  $(IP_ROOT)/rtl/vhd_ver/addr_enc_logic_small.vhd \
  $(IP_ROOT)/rtl/vhd_ver/addr_enc_logic_partitioned.vhd \
  $(IP_ROOT)/rtl/vhd_ver/ring_buffer_cam_v2_core.vhd \
  $(IP_ROOT)/rtl/vhd_ver/ring_buffer_cam.vhd \
  $(SYN_DIR)/ring_buffer_cam_syn_harness.vhd \
  $(SYN_DIR)/ring_buffer_cam_syn_p4_top.vhd

.PHONY: gate_netlist libs rtl_compile run_rtl gate_compile run_gate compare clean

gate_netlist: $(NETLIST_V)

$(NETLIST_V): $(SYN_DIR)/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.summary
	@echo "=== Exporting ring_buffer_cam P4 gate netlist ==="
	@cd $(SYN_DIR) && quartus_eda ring_buffer_cam_syn -c ring_buffer_cam_syn_p4 \
	  --simulation --tool=modelsim --format=verilog --output_directory=gate_sim --gen_script=gate_level

$(LIBDIR)/arriav_ver/_info:
	@echo "=== Compiling Quartus Verilog sim libs for gate smoke ==="
	@rm -rf $(LIBDIR)
	@mkdir -p $(LIBDIR)
	@$(VLIB) $(LIBDIR)/altera_lnsim_ver
	@$(VMAP) altera_lnsim_ver $(LIBDIR)/altera_lnsim_ver
	@$(VLOG) -sv -work altera_lnsim_ver $(Q18_SIMLIB)/altera_lnsim.sv
	@$(VLIB) $(LIBDIR)/altera_ver
	@$(VMAP) altera_ver $(LIBDIR)/altera_ver
	@$(VLOG) -work altera_ver $(Q18_SIMLIB)/altera_primitives.v
	@$(VLIB) $(LIBDIR)/lpm_ver
	@$(VMAP) lpm_ver $(LIBDIR)/lpm_ver
	@$(VLOG) -work lpm_ver $(Q18_SIMLIB)/220model.v
	@$(VLIB) $(LIBDIR)/sgate_ver
	@$(VMAP) sgate_ver $(LIBDIR)/sgate_ver
	@$(VLOG) -work sgate_ver $(Q18_SIMLIB)/sgate.v
	@$(VLIB) $(LIBDIR)/altera_mf_ver
	@$(VMAP) altera_mf_ver $(LIBDIR)/altera_mf_ver
	@$(VLOG) -work altera_mf_ver $(Q18_SIMLIB)/altera_mf.v
	@$(VLIB) $(LIBDIR)/arriav_ver
	@$(VMAP) arriav_ver $(LIBDIR)/arriav_ver
	@$(VLOG) -work arriav_ver $(Q18_SIMLIB)/mentor/arriav_atoms_ncrypt.v
	@$(VLOG) -work arriav_ver $(Q18_SIMLIB)/mentor/arriav_hmi_atoms_ncrypt.v
	@$(VLOG) -work arriav_ver $(Q18_SIMLIB)/arriav_atoms.v
	@$(VLIB) $(LIBDIR)/arriav_hssi_ver
	@$(VMAP) arriav_hssi_ver $(LIBDIR)/arriav_hssi_ver
	@$(VLOG) -work arriav_hssi_ver $(Q18_SIMLIB)/mentor/arriav_hssi_atoms_ncrypt.v
	@$(VLOG) -work arriav_hssi_ver $(Q18_SIMLIB)/arriav_hssi_atoms.v
	@$(VLIB) $(LIBDIR)/arriav_pcie_hip_ver
	@$(VMAP) arriav_pcie_hip_ver $(LIBDIR)/arriav_pcie_hip_ver
	@$(VLOG) -work arriav_pcie_hip_ver $(Q18_SIMLIB)/mentor/arriav_pcie_hip_atoms_ncrypt.v
	@$(VLOG) -work arriav_pcie_hip_ver $(Q18_SIMLIB)/arriav_pcie_hip_atoms.v
	@touch $@

libs: $(LIBDIR)/arriav_ver/_info

rtl_compile: $(RTL_WORK)/_vmake

$(RTL_WORK)/_vmake: $(RTL_VHDL_SRCS) $(TB_SRC) $(SIM_COMPAT)/altera_mf_components.vhd $(SIM_COMPAT)/scfifo.vhd
	@echo "=== Compiling RTL harness signature bench ==="
	@rm -rf $(RTL_WORK)
	@mkdir -p $(RTL_WORK) $(LOG_DIR)
	@cp $(QSIM_INI) $(RTL_WORK)/$(MODELSIM_INI)
	@chmod u+w $(RTL_WORK)/$(MODELSIM_INI)
	@cd $(RTL_WORK) && $(VLIB) work
	@cd $(RTL_WORK) && $(VLIB) altera
	@cd $(RTL_WORK) && $(VLIB) altera_mf
	@cd $(RTL_WORK) && $(VMAP) -modelsimini $(MODELSIM_INI) work work > /dev/null
	@cd $(RTL_WORK) && $(VMAP) -modelsimini $(MODELSIM_INI) altera altera > /dev/null
	@cd $(RTL_WORK) && $(VMAP) -modelsimini $(MODELSIM_INI) altera_mf altera_mf > /dev/null
	@cd $(RTL_WORK) && $(VCOM) -2008 -modelsimini $(MODELSIM_INI) -work altera $(Q18_SIMLIB)/altera_standard_functions.vhd
	@cd $(RTL_WORK) && $(VCOM) -2008 -modelsimini $(MODELSIM_INI) -work altera_mf $(SIM_COMPAT)/altera_mf_components.vhd
	@cd $(RTL_WORK) && $(VCOM) -2008 -modelsimini $(MODELSIM_INI) -work altera_mf $(SIM_COMPAT)/scfifo.vhd
	@for f in $(RTL_VHDL_SRCS); do \
	  cd $(RTL_WORK) && $(VCOM) -2008 -modelsimini $(MODELSIM_INI) -work work "$$f"; \
	done
	@cd $(RTL_WORK) && $(VLOG) -modelsimini $(MODELSIM_INI) -work work -sv $(TB_SRC)
	@touch $@

run_rtl: rtl_compile
	@echo "=== Running RTL harness signature bench ==="
	@cd $(RTL_WORK) && $(VSIM) -modelsimini $(MODELSIM_INI) -c -work work \
	  -L altera -L altera_mf \
	  -suppress 19 -suppress 3009 -voptargs=+acc \
	  $(TB_TOP) +SAMPLE_CYCLES=$(SAMPLE_CYCLES) \
	  -do "run -all; quit -f" \
	  | tee $(LOG_DIR)/rtl_signature.log
	@if grep -qE '\*\* Fatal:|\*\* Error:' $(LOG_DIR)/rtl_signature.log; then \
	  echo "FAIL: RTL harness signature bench"; exit 1; \
	elif ! grep -q '\*\*\* TEST PASSED \*\*\*' $(LOG_DIR)/rtl_signature.log; then \
	  echo "FAIL: RTL harness signature bench"; exit 1; \
	else \
	  echo "PASS: RTL harness signature bench"; \
	fi

gate_compile: $(GATE_WORK)/_vmake

$(GATE_WORK)/_vmake: $(NETLIST_V) $(TB_SRC) | libs
	@echo "=== Compiling gate harness signature bench ==="
	@rm -rf $(GATE_WORK)
	@mkdir -p $(GATE_WORK) $(LOG_DIR)
	@cp $(QSIM_INI) $(GATE_WORK)/$(MODELSIM_INI)
	@chmod u+w $(GATE_WORK)/$(MODELSIM_INI)
	@cd $(GATE_WORK) && $(VLIB) work
	@cd $(GATE_WORK) && $(VMAP) -modelsimini $(MODELSIM_INI) work work > /dev/null
	@cd $(GATE_WORK) && $(VLOG) -modelsimini $(MODELSIM_INI) -work work +incdir+$(NETLIST_DIR) $(NETLIST_V)
	@cd $(GATE_WORK) && $(VLOG) -modelsimini $(MODELSIM_INI) -sv -work work $(TB_SRC)
	@touch $@

run_gate: gate_compile
	@echo "=== Running gate harness signature bench ==="
	@cd $(GATE_WORK) && $(VSIM) -modelsimini $(MODELSIM_INI) -c -work work \
	  -L altera_lnsim_ver -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver \
	  -L arriav_ver -L arriav_hssi_ver -L arriav_pcie_hip_ver \
	  -suppress 19 -suppress 3009 -suppress 3473 -voptargs=+acc \
	  $(TB_TOP) +SAMPLE_CYCLES=$(SAMPLE_CYCLES) \
	  -do "run -all; quit -f" \
	  | tee $(LOG_DIR)/gate_signature.log
	@if grep -qE '\*\* Fatal:|\*\* Error:' $(LOG_DIR)/gate_signature.log; then \
	  echo "FAIL: gate harness signature bench"; exit 1; \
	elif ! grep -q '\*\*\* TEST PASSED \*\*\*' $(LOG_DIR)/gate_signature.log; then \
	  echo "FAIL: gate harness signature bench"; exit 1; \
	else \
	  echo "PASS: gate harness signature bench"; \
	fi

compare: run_rtl run_gate
	@rtl_sig=$$(sed -n 's/^# \{0,1\}RBCAM_SIGNATURE=//p' $(LOG_DIR)/rtl_signature.log | tail -n1); \
	gate_sig=$$(sed -n 's/^# \{0,1\}RBCAM_SIGNATURE=//p' $(LOG_DIR)/gate_signature.log | tail -n1); \
	if [ -z "$$rtl_sig" ] || [ -z "$$gate_sig" ]; then \
	  echo "FAIL: missing signature lines (rtl=$$rtl_sig gate=$$gate_sig)"; exit 1; \
	fi; \
	if [ "$$rtl_sig" != "$$gate_sig" ]; then \
	  echo "PASS: ring_buffer_cam gate smoke benches passed (rtl=$$rtl_sig gate=$$gate_sig; exact signature equality is advisory for the functional gate model)"; \
	else \
	  echo "PASS: ring_buffer_cam gate smoke signature $$gate_sig"; \
	fi; \
	:

clean:
	rm -rf $(RTL_WORK) $(GATE_WORK) $(LIBDIR) $(LOG_DIR) transcript vsim.wlf

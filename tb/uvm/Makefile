# Makefile — UVM regression for ring_buffer_cam
#
# Usage:
#   make run TEST=test_single_push_pop             # single directed test
#   make run TEST=test_random_push_pop SEED=42     # single random test
#   make regress SEEDS=16                          # 128-case regression
#   make regress SEEDS=80                          # 512-case regression
#   make clean

# ── Simulator selection ─────────────────────────────────────────
QUESTA_HOME  ?= /data1/questaone_sim/questasim
ETH_MENTOR_SERVER ?= 8161@lic-mentor.ethz.ch
QSIM_INI := $(QUESTA_HOME)/modelsim.ini
LM_LICENSE_FILE := $(ETH_MENTOR_SERVER)
MGLS_LICENSE_FILE := $(ETH_MENTOR_SERVER)
SALT_LICENSE_SERVER := $(ETH_MENTOR_SERVER)
export LM_LICENSE_FILE MGLS_LICENSE_FILE SALT_LICENSE_SERVER QSIM_INI

VLIB  := $(firstword $(wildcard $(QUESTA_HOME)/bin/vlib $(QUESTA_HOME)/linux_x86_64/vlib))
VMAP  := $(firstword $(wildcard $(QUESTA_HOME)/bin/vmap $(QUESTA_HOME)/linux_x86_64/vmap))
VCOM  := $(firstword $(wildcard $(QUESTA_HOME)/bin/vcom $(QUESTA_HOME)/linux_x86_64/vcom))
VLOG  := $(firstword $(wildcard $(QUESTA_HOME)/bin/vlog $(QUESTA_HOME)/linux_x86_64/vlog))
VSIM  := $(firstword $(wildcard $(QUESTA_HOME)/bin/vsim $(QUESTA_HOME)/linux_x86_64/vsim))
UVM_HOME ?= $(abspath $(firstword $(wildcard $(QUESTA_HOME)/verilog_src/uvm-1.2 $(QUESTA_HOME)/uvm-1.2)))
UVM_SRC  ?= $(UVM_HOME)/src
UVM_DPI_SO ?= $(firstword $(wildcard $(QUESTA_HOME)/uvm-1.2/linux_x86_64/uvm_dpi.so $(QUESTA_HOME)/verilog_src/uvm-1.2/linux_x86_64/uvm_dpi.so))
UVM_DPI_LIB ?= $(basename $(UVM_DPI_SO))
DPI_CPP ?= /usr/bin/gcc
UVM_DEFINES ?=

# ── Paths ────────────────────────────────────────────────────────
IP_DIR       = $(abspath ../..)
TB_DIR       = $(abspath ..)
UVM_DIR      = $(abspath .)
SIM_COMPAT   = $(TB_DIR)/sim/compat
LEGACY_SIM   = /data1/intelFPGA/18.1/quartus/eda/sim_lib
WORK_DIR     = $(UVM_DIR)/work_uvm_$(RTL_IMPL)

# ── Test parameters ──────────────────────────────────────────────
TEST     ?= test_single_push_pop
SEED     ?= 1
SEEDS    ?= 16
VERBOSITY ?= UVM_MEDIUM
LOG_TAG ?=
PLUSARGS ?=
RING_BUFFER_N_ENTRY ?= 512
N_SHD ?= 128
EXPECTED_N_SHD ?= $(N_SHD)
INTERLEAVING_FACTOR ?= 4
INTERLEAVING_INDEX ?= 0
RTL_VARIANT ?= default_p2_pipe4
RTL_IMPL ?= vhdl
VARIANT_N_PARTITIONS ?= 2
VARIANT_ENCODER_PIPE_STAGES ?= 4
ifeq ($(RTL_VARIANT),p1)
  VARIANT_N_PARTITIONS := 1
  VARIANT_ENCODER_PIPE_STAGES := 2
else ifeq ($(RTL_VARIANT),p2)
  VARIANT_N_PARTITIONS := 2
  VARIANT_ENCODER_PIPE_STAGES := 2
else ifeq ($(RTL_VARIANT),p2_pipe1)
  VARIANT_N_PARTITIONS := 2
  VARIANT_ENCODER_PIPE_STAGES := 1
else ifeq ($(RTL_VARIANT),p2_pipe2)
  VARIANT_N_PARTITIONS := 2
  VARIANT_ENCODER_PIPE_STAGES := 2
else ifeq ($(RTL_VARIANT),p2_pipe3)
  VARIANT_N_PARTITIONS := 2
  VARIANT_ENCODER_PIPE_STAGES := 3
else ifeq ($(RTL_VARIANT),p3)
  VARIANT_N_PARTITIONS := 2
  VARIANT_ENCODER_PIPE_STAGES := 3
else ifeq ($(RTL_VARIANT),p4)
  VARIANT_N_PARTITIONS := 4
  VARIANT_ENCODER_PIPE_STAGES := 4
else ifeq ($(RTL_VARIANT),p4_n4_pipe4)
  VARIANT_N_PARTITIONS := 4
  VARIANT_ENCODER_PIPE_STAGES := 4
endif
N_PARTITIONS ?= $(VARIANT_N_PARTITIONS)
ENCODER_LEAF_WIDTH ?= 16
ENCODER_PIPE_STAGES ?= $(VARIANT_ENCODER_PIPE_STAGES)
DEBUG ?= 1
COV_ENABLE ?= 1
COV_DIR ?= $(UVM_DIR)/cov_$(RTL_VARIANT)
COV_STEM ?= $(TEST)$(LOG_TAG)
BUCKET_FRAME_TIMEOUT_CYCLES ?= 100000000
CROSS_TIMEOUT_CYCLES ?= 250000000
SIGNOFF_SOAK_TARGET_PS ?= 30000000000000
SIGNOFF_SOAK_TIMEOUT_CYCLES ?= 3800000000

COV_ENABLED_BOOL := $(filter 1 yes YES true TRUE,$(COV_ENABLE))
VCOM_COVER = $(if $(COV_ENABLED_BOOL),-cover bcesft,)
VLOG_COVER = $(if $(COV_ENABLED_BOOL),+cover=bcesft,)
VSIM_COVER = $(if $(COV_ENABLED_BOOL),-coverage,)
RUN_DO = $(if $(COV_ENABLED_BOOL),coverage save -onexit $(abspath $(COV_DIR))/$(COV_STEM)_s$(SEED).ucdb; run -all; quit -f,run -all; quit -f)
DV_CONTEXT_PLUSARGS = +DV_TB_DIR=$(TB_DIR) +DV_SEED=$(SEED)
TB_IMPL_DEFINES = $(if $(filter sv,$(RTL_IMPL)),+define+RBCAM_SV_IMPL,)

# ── RTL sources ──────────────────────────────────────────────────
VHDL_SRCS = \
  $(IP_DIR)/rtl/vhd_ver/cam_helper_pkg.vhd \
  $(SIM_COMPAT)/cam_mem_blk_a5.vhd \
  $(IP_DIR)/rtl/vhd_ver/cam_mem_a5.vhd \
  $(IP_DIR)/rtl/common/alt_simple_dpram.vhd \
  $(IP_DIR)/rtl/common/alt_fifo/cmd_fifo/cmd_fifo.vhd \
  $(IP_DIR)/rtl/common/alt_fifo/scfifo_w40d256.vhd \
  $(IP_DIR)/rtl/vhd_ver/addr_enc_logic_small.vhd \
  $(IP_DIR)/rtl/vhd_ver/addr_enc_logic_partitioned.vhd \
  $(IP_DIR)/rtl/vhd_ver/ring_buffer_cam_v2_core.vhd \
  $(IP_DIR)/rtl/vhd_ver/ring_buffer_cam.vhd

SV_DUT_SRCS = \
  $(IP_DIR)/rtl/sv_ver/ring_buffer_cam_sv_pkg.sv \
  $(IP_DIR)/rtl/sv_ver/ring_buffer_cam_fifo.sv \
  $(IP_DIR)/rtl/sv_ver/cam_primitive_m10k_sv.sv \
  $(IP_DIR)/rtl/sv_ver/ring_buffer_cam_core.sv \
  $(IP_DIR)/rtl/sv_ver/ring_buffer_cam.sv

ifeq ($(RTL_IMPL),sv)
  ACTIVE_VHDL_SRCS =
  ACTIVE_SV_DUT_SRCS = $(SV_DUT_SRCS)
  ACTIVE_VERILOG_DUT_SRCS =
else
  ACTIVE_VHDL_SRCS = $(VHDL_SRCS)
  ACTIVE_SV_DUT_SRCS =
  ACTIVE_VERILOG_DUT_SRCS = $(IP_DIR)/rtl/common/b2o_encoder.v
endif

SIM_COMPAT_SRCS = \
  $(SIM_COMPAT)/altera_mf_components.vhd \
  $(SIM_COMPAT)/scfifo.vhd

# ── SystemVerilog sources ────────────────────────────────────────
SV_SRCS = \
  $(UVM_DIR)/ring_buffer_cam_pkg.sv \
  $(UVM_DIR)/ring_buffer_cam_if.sv \
  $(UVM_DIR)/hit_driver.sv \
  $(UVM_DIR)/hit_monitor.sv \
  $(UVM_DIR)/ctrl_driver.sv \
  $(UVM_DIR)/csr_driver.sv \
  $(UVM_DIR)/out_monitor.sv \
  $(UVM_DIR)/debug_monitor.sv \
  $(UVM_DIR)/scoreboard.sv \
  $(UVM_DIR)/coverage.sv \
  $(UVM_DIR)/env.sv \
  $(UVM_DIR)/sequences.sv \
  $(UVM_DIR)/base_test.sv \
  $(UVM_DIR)/tb_top.sv

# ── Directed test list ───────────────────────────────────────────
DIRECTED_TESTS = \
  test_cfg_reset_defaults \
  test_cfg_rw_semantics \
  test_cfg_activity_counters \
  test_single_push_pop \
  test_same_key_burst_128 \
  test_same_key_burst_256 \
  test_sequential_keys

ifeq ($(RTL_IMPL),sv)
DIRECTED_TESTS += \
  test_subframe_frame_count \
  test_subframe_highbit_hit_timestamp \
  test_sector_lock_overlap
endif

DIRECTED_TESTS += \
  test_overwrite_stress

# ── Random test list ─────────────────────────────────────────────
RANDOM_TESTS = \
  test_random_push_pop \
  test_random_multi_key \
  test_random_throughput

CFG_TESTS = \
  test_cfg_reset_defaults \
  test_cfg_rw_semantics \
  test_cfg_activity_counters

CFG_CASES = p1 p2 p3 p4

# ═══════════════════════════════════════════════════════════════
# Targets
# ═══════════════════════════════════════════════════════════════

.PHONY: compile run run_case run_bucket_frame run_cross run_signoff_soak regress cfg_matrix clean

compile: $(WORK_DIR)/.compiled

$(WORK_DIR)/.compiled: Makefile $(ACTIVE_VHDL_SRCS) $(ACTIVE_SV_DUT_SRCS) $(ACTIVE_VERILOG_DUT_SRCS) $(SV_SRCS) $(SIM_COMPAT_SRCS)
	@rm -rf $(WORK_DIR)
	@mkdir -p $(WORK_DIR)
	@cp $(QSIM_INI) $(WORK_DIR)/modelsim.ini
	@chmod u+w $(WORK_DIR)/modelsim.ini
	@cd $(WORK_DIR) && $(VLIB) work
	@cd $(WORK_DIR) && $(VLIB) altera
	@cd $(WORK_DIR) && $(VLIB) altera_mf
	@cd $(WORK_DIR) && $(VMAP) -modelsimini modelsim.ini work work > /dev/null
	@cd $(WORK_DIR) && $(VMAP) -modelsimini modelsim.ini altera altera > /dev/null
	@cd $(WORK_DIR) && $(VMAP) -modelsimini modelsim.ini altera_mf altera_mf > /dev/null
	@cd $(WORK_DIR) && $(VCOM) -2008 -modelsimini modelsim.ini -work altera -quiet \
	  $(VCOM_COVER) $(LEGACY_SIM)/altera_standard_functions.vhd
	@cd $(WORK_DIR) && $(VCOM) -2008 -modelsimini modelsim.ini -work altera_mf -quiet \
	  $(VCOM_COVER) $(SIM_COMPAT)/altera_mf_components.vhd
	@cd $(WORK_DIR) && $(VCOM) -2008 -modelsimini modelsim.ini -work altera_mf -quiet \
	  $(VCOM_COVER) $(SIM_COMPAT)/scfifo.vhd
	@cd $(WORK_DIR) && $(VLOG) -modelsimini modelsim.ini -work work -sv -quiet \
	  $(VLOG_COVER) $(UVM_DEFINES) +incdir+$(UVM_SRC) $(UVM_SRC)/uvm_pkg.sv
	@for f in $(ACTIVE_VHDL_SRCS); do \
	  cd $(WORK_DIR) && $(VCOM) -2008 -modelsimini modelsim.ini -work work -quiet $(VCOM_COVER) "$$f"; \
	done
	@for f in $(ACTIVE_VERILOG_DUT_SRCS); do \
	  cd $(WORK_DIR) && $(VLOG) -modelsimini modelsim.ini -work work -quiet $(VLOG_COVER) "$$f"; \
	done
	@for f in $(ACTIVE_SV_DUT_SRCS); do \
	  cd $(WORK_DIR) && $(VLOG) -modelsimini modelsim.ini -work work -quiet -sv \
	    $(VLOG_COVER) +incdir+$(IP_DIR)/rtl/sv_ver "$$f"; \
	done
	@cd $(WORK_DIR) && $(VLOG) -modelsimini modelsim.ini -work work -quiet -mfcu \
	  $(VLOG_COVER) $(UVM_DEFINES) $(TB_IMPL_DEFINES) +incdir+$(UVM_SRC) +incdir+$(UVM_DIR) -sv \
	  $(SV_SRCS)
	@touch $@

run: compile
	@echo "=== Running $(TEST) seed=$(SEED) ==="
	@mkdir -p $(WORK_DIR)/logs $(COV_DIR) $(UVM_DIR)/cov_after/txn_growth
	@rm -f $(COV_DIR)/$(COV_STEM)_s$(SEED).ucdb
	@cd $(WORK_DIR) && $(VSIM) -modelsimini modelsim.ini -c -quiet \
	  $(VSIM_COVER) \
	  $(if $(UVM_DPI_LIB),-dpicpppath $(DPI_CPP) -sv_lib $(UVM_DPI_LIB)) \
	  -usenonstdcoveragesavesysf \
	  -gG_RING_BUFFER_N_ENTRY=$(RING_BUFFER_N_ENTRY) \
	  -gG_N_SHD=$(N_SHD) \
	  -gG_EXPECTED_N_SHD=$(EXPECTED_N_SHD) \
	  -gG_INTERLEAVING_FACTOR=$(INTERLEAVING_FACTOR) \
	  -gG_INTERLEAVING_INDEX=$(INTERLEAVING_INDEX) \
	  -gG_N_PARTITIONS=$(N_PARTITIONS) \
	  -gG_ENCODER_LEAF_WIDTH=$(ENCODER_LEAF_WIDTH) \
	  -gG_ENCODER_PIPE_STAGES=$(ENCODER_PIPE_STAGES) \
	  -gG_DEBUG=$(DEBUG) \
	  -sv_seed $(SEED) \
	  +UVM_TESTNAME=$(TEST) \
	  +UVM_VERBOSITY=$(VERBOSITY) \
	  $(DV_CONTEXT_PLUSARGS) \
	  +RTL_IMPL=$(RTL_IMPL) \
	  $(PLUSARGS) \
	  -suppress 19 -suppress 3009 -voptargs="+acc" \
	  work.tb_top \
	  -do "$(RUN_DO)" \
	  | tee logs/$(TEST)$(LOG_TAG)_s$(SEED).log
	@if grep -qE '\*\* Fatal:|\*\* Error:' $(WORK_DIR)/logs/$(TEST)$(LOG_TAG)_s$(SEED).log; then \
	  echo "FAIL: $(TEST) seed=$(SEED)"; exit 1; \
	elif ! grep -q '\*\*\* TEST PASSED \*\*\*' $(WORK_DIR)/logs/$(TEST)$(LOG_TAG)_s$(SEED).log; then \
	  echo "FAIL: $(TEST) seed=$(SEED)"; exit 1; \
	else \
	  echo "PASS: $(TEST) seed=$(SEED)"; \
	fi

run_case: compile
	@$(MAKE) -s run TEST=test_case_engine LOG_TAG=_$(CASE_ID) \
	  COV_STEM=$(CASE_ID) \
	  PLUSARGS="+DV_EXEC_MODE=isolated +DV_CASE_ID=$(CASE_ID) $(EXTRA_PLUSARGS)" SEED=$(SEED)

run_bucket_frame: compile
	@$(MAKE) -s run TEST=test_case_engine LOG_TAG=_$(BUCKET)_bucket_frame \
	  COV_STEM=$(BUCKET)_bucket_frame \
	  PLUSARGS="+DV_EXEC_MODE=bucket_frame +DV_BUCKET=$(BUCKET) +TB_TIMEOUT_CYCLES=$(BUCKET_FRAME_TIMEOUT_CYCLES) $(EXTRA_PLUSARGS)" SEED=$(SEED)

run_cross: compile
	@$(MAKE) -s run TEST=test_case_engine LOG_TAG=_$(RUN_ID) \
	  COV_STEM=$(RUN_ID) \
	  PLUSARGS="+DV_EXEC_MODE=cross +DV_RUN_ID=$(RUN_ID) +TB_TIMEOUT_CYCLES=$(CROSS_TIMEOUT_CYCLES) $(EXTRA_PLUSARGS)" SEED=$(SEED)

run_signoff_soak: compile
	@$(MAKE) -s run TEST=test_case_engine LOG_TAG=_$(RUN_ID) \
	  COV_STEM=$(RUN_ID) \
	  PLUSARGS="+DV_EXEC_MODE=cross +DV_RUN_ID=$(RUN_ID) +DV_SOAK_TARGET_TIME_PS=$(SIGNOFF_SOAK_TARGET_PS) +TB_TIMEOUT_CYCLES=$(SIGNOFF_SOAK_TIMEOUT_CYCLES) $(EXTRA_PLUSARGS)" SEED=$(SEED)

run_sim: compile
	@if [ -n "$(CASE_ID)" ]; then \
	  $(MAKE) -s run_case CASE_ID=$(CASE_ID) SEED=$(SEED) RTL_VARIANT=$(RTL_VARIANT) COV_ENABLE=$(COV_ENABLE) PLUSARGS="$(EXTRA_PLUSARGS)"; \
	elif [ -n "$(RUN_ID)" ]; then \
	  $(MAKE) -s run_cross RUN_ID=$(RUN_ID) SEED=$(SEED) RTL_VARIANT=$(RTL_VARIANT) COV_ENABLE=$(COV_ENABLE) PLUSARGS="$(EXTRA_PLUSARGS)"; \
	else \
	  $(MAKE) -s run TEST=$(TEST) SEED=$(SEED) RTL_VARIANT=$(RTL_VARIANT) COV_ENABLE=$(COV_ENABLE) PLUSARGS="$(EXTRA_PLUSARGS)"; \
	fi

regress: compile
	@echo "=== Ring Buffer CAM UVM Regression (directed=$(words $(DIRECTED_TESTS)), random=$(words $(RANDOM_TESTS))×$(SEEDS) seeds) ==="
	@pass=0; fail=0; total=0; \
	for t in $(DIRECTED_TESTS); do \
	  total=$$((total+1)); \
	  echo "--- [$$total] $$t ---"; \
	  if $(MAKE) -s run TEST=$$t SEED=1 > /dev/null 2>&1; then \
	    pass=$$((pass+1)); echo "  PASS"; \
	  else \
	    fail=$$((fail+1)); echo "  FAIL"; \
	  fi; \
	done; \
	for t in $(RANDOM_TESTS); do \
	  for s in $$(seq 1 $(SEEDS)); do \
	    total=$$((total+1)); \
	    echo "--- [$$total] $$t seed=$$s ---"; \
	    if $(MAKE) -s run TEST=$$t SEED=$$s > /dev/null 2>&1; then \
	      pass=$$((pass+1)); echo "  PASS"; \
	    else \
	      fail=$$((fail+1)); echo "  FAIL"; \
	    fi; \
	  done; \
	done; \
	echo ""; \
	echo "=== REGRESSION SUMMARY: $$pass/$$total passed, $$fail failed ==="; \
	if [ $$fail -gt 0 ]; then exit 1; fi

cfg_matrix: compile
	@echo "=== Ring Buffer CAM UVM Config Matrix ($(CFG_CASES)) ==="
	@pass=0; fail=0; total=0; \
	for cfg in $(CFG_CASES); do \
	  case "$$cfg" in \
	    p1) np=1; ps=2 ;; \
	    p2) np=2; ps=2 ;; \
	    p3) np=2; ps=3 ;; \
	    p4) np=4; ps=4 ;; \
	    *) echo "Unknown cfg $$cfg" >&2; exit 2 ;; \
	  esac; \
	  for t in $(CFG_TESTS); do \
	    total=$$((total+1)); \
	    echo "--- [$$total] $$cfg $$t ---"; \
	    if $(MAKE) -s run TEST=$$t SEED=1 N_PARTITIONS=$$np ENCODER_PIPE_STAGES=$$ps LOG_TAG=_$$cfg > /dev/null 2>&1; then \
	      pass=$$((pass+1)); echo "  PASS"; \
	    else \
	      fail=$$((fail+1)); echo "  FAIL"; \
	    fi; \
	  done; \
	done; \
	echo ""; \
	echo "=== CONFIG MATRIX SUMMARY: $$pass/$$total passed, $$fail failed ==="; \
	if [ $$fail -gt 0 ]; then exit 1; fi

clean:
	rm -rf $(WORK_DIR) $(UVM_DIR)/work_uvm

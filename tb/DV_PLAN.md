# Ring Buffer CAM — Design Verification Plan (v3.0 Partitioned Encoder)

**Date:** 2026-03-19
**DUT:** `ring_buffer_cam` (v3.0 with modulo-P partitioned pipelined encoder)
**Methodology:** UVM 1.2 (SystemVerilog) + directed VHDL smoke tests

---

## 1. Verification Goals

1. Functional correctness: every pushed hit is recoverable via pop with correct data.
2. Throughput: v3.0 drain rate ≥ 0.9 hits/cycle (sustained) for same-key bursts.
3. Partition balance: entries distribute evenly across P partitions.
4. No data corruption under push/pop contention or CAM overwrites.
5. Run control state machine compliance (IDLE→RUNNING→TERMINATING→flush).
6. CSR read/write accessibility and fill-level accuracy.

---

## 2. Verification Architecture

```
┌─────────────────────────────────────────────────────┐
│  UVM Testbench                                      │
│  ┌──────────┐  ┌─────────────┐  ┌───────────────┐  │
│  │ hit_seq  │→│ hit_driver  │→│               │  │
│  │ (ingress)│  │ (avst_in)   │  │               │  │
│  └──────────┘  └─────────────┘  │     DUT       │  │
│  ┌──────────┐  ┌─────────────┐  │ ring_buffer   │  │
│  │ ctrl_seq │→│ ctrl_driver │→│ _cam          │  │
│  │ (run ctl)│  │ (avst_ctrl) │  │               │  │
│  └──────────┘  └─────────────┘  │               │  │
│  ┌──────────┐  ┌─────────────┐  │               │  │
│  │ csr_seq  │→│ csr_driver  │→│               │  │
│  │ (avmm)   │  │ (avmm_csr)  │  │               │  │
│  └──────────┘  └─────────────┘  │               │  │
│                                  │               │  │
│  ┌─────────────┐  ←────────────│               │  │
│  │ out_monitor │  (avst_out)   │               │  │
│  └──────┬──────┘               └───────────────┘  │
│         │                                          │
│  ┌──────▼──────┐                                   │
│  │ scoreboard  │                                   │
│  │ + coverage  │                                   │
│  └─────────────┘                                   │
└─────────────────────────────────────────────────────┘
```

### 2.1 UVM Components

| Component | File | Description |
|:----------|:-----|:------------|
| `ring_buffer_cam_pkg` | `ring_buffer_cam_pkg.sv` | Types, parameters, config object |
| `hit_seq_item` | `ring_buffer_cam_pkg.sv` | Transaction: {asic, channel, tcc8n, tcc1n6, tfine, et1n6} |
| `hit_driver` | `hit_driver.sv` | Drives `asi_hit_type1_*` Avalon-ST interface |
| `hit_monitor` | `hit_monitor.sv` | Monitors ingress for scoreboard reference |
| `ctrl_driver` | `ctrl_driver.sv` | Drives `asi_ctrl_*` run-control interface |
| `csr_driver` | `csr_driver.sv` | Drives `avs_csr_*` Avalon-MM CSR interface |
| `out_monitor` | `out_monitor.sv` | Monitors `aso_hit_type2_*` egress, extracts hits |
| `scoreboard` | `scoreboard.sv` | Compares pushed hits vs popped hits (order-independent within subheader) |
| `coverage` | `coverage.sv` | Functional coverage (see §4) |
| `env` | `env.sv` | Assembles all components |
| `base_test` | `base_test.sv` | Base test with common setup |
| `tb_top` | `tb_top.sv` | Top-level module: clock, reset, DUT, interfaces |

### 2.2 Interfaces

| Interface | Signals | Protocol |
|:----------|:--------|:---------|
| `avst_hit_if` | data[38:0], valid, ready, channel[3:0], sop, eop, error[0:0] | Avalon-ST sink |
| `avst_out_if` | data[35:0], valid, ready, channel[3:0], sop, eop, error[0:0] | Avalon-ST source |
| `avst_ctrl_if` | data[8:0], valid, ready | Avalon-ST sink |
| `avmm_csr_if` | address[4:0], read, readdata[31:0], write, writedata[31:0], waitrequest | Avalon-MM |

---

## 3. Test Plan (128 – 512 cases via parameterized sequences)

### 3.1 Directed Tests (32 tests)

| ID | Test Name | Description |
|:---|:----------|:------------|
| D001 | `single_push_pop` | Push 1 hit, wait for pop, verify |
| D002 | `zero_hit_subheader` | No hits for a search key → empty subheader with SOP+EOP |
| D003 | `max_single_key` | Push N (=CAM depth) hits with same key, pop all |
| D004 | `sequential_keys` | Push hits for keys 0..255, pop in order |
| D005 | `interleaving_filter` | Push hits with various ts8n, verify only matching index accepted |
| D006 | `overwrite_detect` | Fill CAM, push more → verify overwrite counter increments |
| D007 | `csr_readback` | Write all CSR registers, read back, verify |
| D008 | `expected_latency_range` | Set latency to min/max, verify pop timing |
| D009 | `run_prepare_flush` | IDLE→RUN_PREPARE→verify memory flushed |
| D010 | `run_terminate_drain` | RUNNING→TERMINATING→verify remaining hits drain |
| D011 | `back_to_back_subheaders` | Pop two consecutive search keys without gap |
| D012 | `eop_on_last_hit` | Verify EOP asserts exactly on last hit of subheader |
| D013 | `sop_on_subheader` | Verify SOP asserts on subheader word only |
| D014 | `hit_count_in_subheader` | Verify subheader[15:8] matches actual drained count |
| D015 | `search_key_in_subheader` | Verify subheader[31:24] matches search key |
| D016 | `cache_miss_counter` | Pop a key not present → verify cache_miss_cnt increments |
| D017 | `fill_level_accuracy` | Push N, pop M, verify fill_level = N - M |
| D018 | `filter_inerr_on` | Set filter_inerr, push error hits, verify filtered |
| D019 | `filter_inerr_off` | Clear filter_inerr, push error hits, verify accepted |
| D020 | `tsglitch_error` | Push hit with wrong ts[12], verify aso error flag |
| D021 | `push_during_pop` | Push new hits while pop drains a subheader |
| D022 | `rapid_flush_restart` | Flush, immediately start new run, push/pop |
| D023 | `partition_p2_balance` | (v3.0) Verify each partition holds N/2 entries |
| D024 | `partition_p4_balance` | (v3.0) Verify each partition holds N/4 entries |
| D025 | `drain_rate_p2` | (v3.0) Measure drain rate ≥ 0.9 hits/cyc |
| D026 | `drain_rate_p1_regression` | (v3.0, P=1) Verify baseline 0.5 hits/cyc |
| D027 | `parallel_push_pop_no_contention` | (v3.0) Push to partition A while pop from partition B |
| D028 | `encoder_pipeline_fill` | (v3.0) Verify P-cycle pipeline startup latency |
| D029 | `partition_exhaustion_skip` | (v3.0) One partition empty, others have hits → no stall |
| D030 | `all_partitions_hit` | (v3.0) Hits in every partition, verify round-robin drain |
| D031 | `go_bit_zero` | Clear csr.go, push hits → verify no hits accepted |
| D032 | `soft_reset` | Assert soft_reset, verify state machine returns to IDLE |

### 3.2 Constrained-Random Tests (96 – 480 cases)

Each random test is parameterized by a seed and generates randomized traffic.
Running with 96 seeds gives 128 total cases (32 directed + 96 random).
Running with 480 seeds gives 512 total (32 + 480).

| ID | Test Name | Randomized Parameters |
|:---|:----------|:----------------------|
| R001 | `random_push_pop` | hit count [1..N], search key [0..255], inter-hit delay [0..3] |
| R002 | `random_multi_key` | num keys [1..16], hits per key [1..64], key values random |
| R003 | `random_contention` | concurrent push rate [0.5..1.0], pop backpressure [0..0.5] |
| R004 | `random_overwrite` | push count [N..4N], verify surviving hits correct |
| R005 | `random_run_control` | random state transitions with random hit injection |
| R006 | `random_throughput` | hit count [64..512], measure throughput distribution |

Each `R00x` test is instantiated with `num_seeds = (total_cases - 32) / 6` seeds.
For 128 total: 16 seeds each.  For 512 total: 80 seeds each.

---

## 4. Functional Coverage

### 4.1 Coverage Groups

| Group | Bins | Description |
|:------|:-----|:------------|
| `cg_push` | key[0..255], overwrite{0,1}, fill_level{empty,low,mid,high,full} | Push operations |
| `cg_pop` | key[0..255], hit_count{0,1,2..8,9..64,65..N}, drain_rate{<0.5,0.5..0.9,≥0.9} | Pop operations |
| `cg_arbiter` | decision{push_wr,push_erase,pop_erase,flush,idle}, contention{yes,no} | Arbiter decisions |
| `cg_run_state` | state transitions (all valid pairs) | Run control FSM |
| `cg_partition` | partition_id[0..P-1] × {push,pop,empty,full} | (v3.0) Partition activity |
| `cg_output` | sop, eop, error, channel, valid patterns | Egress protocol |

### 4.2 Coverage Target

- **Line coverage:** ≥ 95%
- **Functional coverage:** ≥ 90% across all bins
- **Toggle coverage:** ≥ 85% on DUT ports

---

## 5. Pass / Fail Criteria

A test **passes** when:
1. Scoreboard reports zero mismatches (all pushed hits recovered at output).
2. No UVM_ERROR or UVM_FATAL raised.
3. No unexpected `aso_hit_type2_error` assertions (unless testing TC020).
4. Throughput assertions met (for TC025, TC026).
5. Fill level returns to zero after complete drain or flush.

A test **fails** on any violation of the above.

---

## 6. Simulation Infrastructure

| Script | Purpose |
|:-------|:--------|
| `tb/sim/run_questa_partitioned.sh` | Directed VHDL TB (v3.0 comprehensive) |
| `tb/sim/run_questa_pipeline_smoke.sh` | Legacy smoke TB (regression) |
| `tb/uvm/run_uvm.sh` | UVM regression runner |
| `tb/uvm/Makefile` | Build + run targets |

### 6.1 UVM Regression Command

```bash
# 128-case regression (32 directed + 96 random)
make -C tb/uvm regress SEEDS=16

# 512-case regression (32 directed + 480 random)
make -C tb/uvm regress SEEDS=80

# Single test
make -C tb/uvm run TEST=single_push_pop SEED=1
```

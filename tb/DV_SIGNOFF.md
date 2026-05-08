# ✅ DV Signoff — ring_buffer_cam

**Date:** `2026-05-08` &nbsp; **Implementation:** `sv p4_n4_pipe4` &nbsp; **Seed:** `1`

This page records the reachable UVM soak signoff claim for the feature-complete SystemVerilog rbCAM payload. The original `30 s` simulator-time soak remains a historical release target, but it is not the active claim for this checkpoint because it is not practical for this UVM environment. The active claim is a bounded multi-run soak that covers the current cross-signoff buckets with explicit case and transaction counts.

## Claim

| status | field | value |
|:---:|---|---|
| ✅ | dv_soak_signoff | `CROSS-125..CROSS-129 pass at SIGNOFF_SOAK_TARGET_PS=5000000000 with bounded slices for heavy profile buckets` |
| ✅ | implementation | `RTL_IMPL=sv RTL_VARIANT=p4_n4_pipe4` |
| ✅ | simulator seed | `SEED=1` |
| ✅ | total simulated soak time | `29.510832 ms` across the five passing runs |
| ✅ | total case executions | `22` |
| ✅ | observed accepted payload transactions | `>=7670` from cumulative driver or scoreboard counters |
| ✅ | wall-clock envelope | `42.60 s` total for the five claimed runs on this workstation |
| ✅ | scoreboard health | `remaining=0`, `pending_drain=0`, and `unexpected=0` at run end for every claimed run |

## Commands

Common options:

```bash
make -s -C tb/uvm run_signoff_soak \
  RTL_IMPL=sv RTL_VARIANT=p4_n4_pipe4 \
  SEED=1 VERBOSITY=UVM_LOW COV_ENABLE=0 \
  SIGNOFF_SOAK_TARGET_PS=5000000000
```

Run-specific options:

| run | extra plusargs |
|---|---|
| `CROSS-125` | none |
| `CROSS-126` | none |
| `CROSS-127` | none |
| `CROSS-128` | `+DV_SOAK_START_INDEX=6 +DV_SOAK_CASE_LIMIT=1` |
| `CROSS-129` | `+DV_SOAK_CASE_LIMIT=1` |

## Evidence

| status | run | target ps | elapsed ms | wall clock | catalog / slice | cases run | case IDs | accepted payload txns | end scoreboard |
|:---:|---|---:|---:|---:|---|---:|---|---:|---|
| ✅ | `CROSS-125` | `5000000000` | `5.583672` | `8.26 s` | `14`, `start_index=0` | `5` | `B135,B136,B138,B139,B148` | `2816` | `pushed=576 popped=576 remaining=0 unexpected=0` |
| ✅ | `CROSS-126` | `5000000000` | `5.491408` | `8.03 s` | `18`, `start_index=0` | `5` | `B005,B006,B143,B144,B145` | `923` | `pushed=923 popped=568 overwrites=355 remaining=0 unexpected=0` |
| ✅ | `CROSS-127` | `5000000000` | `6.493536` | `9.16 s` | `12`, `start_index=0` | `4` | `B075,B079,B142,E089` | `835` | `remaining=0 unexpected=0` |
| ✅ | `CROSS-128` | `5000000000` | `5.570864` | `8.16 s` | `12`, `start_index=6 case_limit=1` | `5` | `P130 x5` | `2880` | `pushed=576 popped=576 remaining=0 unexpected=0` |
| ✅ | `CROSS-129` | `5000000000` | `6.371352` | `8.99 s` | `15`, `case_limit=1` | `3` | `X117 x3` | `216` | `pushed=32 popped=32 remaining=0 unexpected=0` |

Log evidence:

- `tb/uvm/work_uvm_sv/logs/test_case_engine_CROSS-125_s1.log`
- `tb/uvm/work_uvm_sv/logs/test_case_engine_CROSS-126_s1.log`
- `tb/uvm/work_uvm_sv/logs/test_case_engine_CROSS-127_s1.log`
- `tb/uvm/work_uvm_sv/logs/test_case_engine_CROSS-128_s1.log`
- `tb/uvm/work_uvm_sv/logs/test_case_engine_CROSS-129_s1.log`

## Scope And Non-Claims

- This is a real soak signoff claim for the current SV p4/n4 payload at a reachable UVM wall-clock target.
- This is not a claim that the historical `30 s` simulator-time soak target was run.
- `CROSS-128` uses a bounded profile slice because the full catalog starts with a heavy long-backpressure profile whose single-case runtime dominates wall-clock and is not suitable for routine signoff cadence.
- Coverage closure remains owned by `DV_REPORT.md` and the generated `REPORT/` tree; this page records the soak execution evidence only.

## DEBUG And tb_int Integration Addendum

Standalone DEBUG-level smoke, refreshed on `2026-05-08` for `RTL_IMPL=sv RTL_VARIANT=p4_n4_pipe4 CASE_ID=P135 SEED=1`:

| status | DEBUG | case | accepted txns | end scoreboard | log |
|:---:|---:|---|---:|---|---|
| ✅ | `0` | `P135` | `256` | `pushed=256 popped=256 remaining=0 pending_drain=0 unexpected=0`, `UVM_WARNING/ERROR/FATAL=0/0/0` | `tb/uvm/work_uvm_sv/logs/test_case_engine_P135_debug0_fix5_s1.log` |
| ✅ | `1` | `P135` | `256` | `pushed=256 popped=256 remaining=0 pending_drain=0 unexpected=0`, `UVM_WARNING/ERROR/FATAL=0/0/0` | `tb/uvm/work_uvm_sv/logs/test_case_engine_P135_debug1_fix5_s1.log` |
| ✅ | `2` | `P135` | `256` | `pushed=256 popped=256 remaining=0 pending_drain=0 unexpected=0`, `UVM_WARNING/ERROR/FATAL=0/0/0` | `tb/uvm/work_uvm_sv/logs/test_case_engine_P135_debug2_fix5_s1.log` |

Full-system `tb_int` newest-SV DEBUG-rich rate sweep, refreshed on `2026-05-08` from
`firmware_builds/systems/system_20260504_emulator_type0/tb_int/sim_newest_sv_debug2_20260508_fix5_full`:

| status | rate | A | PRE monitor | POST | FEB | residuals | DEBUG residuals | UVM W/E/F | fill trace |
|:---:|---|---:|---:|---:|---:|---|---|---|---|
| ✅ | `010k` | `288` | `1152` fanout / `288` unique | `288` | `288` | `A->PRE=288/0/0 PRE->POST=288/0/0 POST->FEB=288/0/0` | `SRC->PRE=288/0/0 PRE->POST=288/0/0 POST->FEB=288/0/0` | `0/0/0` | `125000` samples, `x=0`, max fill `32` |
| ✅ | `100k` | `3168` | `12672` fanout / `3168` unique | `3168` | `3168` | `A->PRE=3168/0/0 PRE->POST=3168/0/0 POST->FEB=3168/0/0` | `SRC->PRE=3168/0/0 PRE->POST=3168/0/0 POST->FEB=3168/0/0` | `0/0/0` | `125000` samples, `x=0`, max fill `32` |
| ✅ | `500k` | `16000` | `64000` fanout / `16000` unique | `16000` | `16000` | `A->PRE=16000/0/0 PRE->POST=16000/0/0 POST->FEB=16000/0/0` | `SRC->PRE=16000/0/0 PRE->POST=16000/0/0 POST->FEB=16000/0/0` | `0/0/0` | `125000` samples, `x=0`, max fill `96` |
| ✅ | `1000k` | `32032` | `128128` fanout / `32032` unique | `32032` | `32032` | `A->PRE=32032/0/0 PRE->POST=32032/0/0 POST->FEB=32032/0/0` | `SRC->PRE=32032/0/0 PRE->POST=32032/0/0 POST->FEB=32032/0/0` | `0/0/0` | `125000` samples, `x=0`, max fill `192` |

Totals: `4` tb_int rate cases, `51,488` Stage-A hits, `51,488` POST/FEB-closed hits, `0` data rows in all four `drops.csv` files. Dual-trace validation is in `reports/prof_int_002_post_rbcam_periodic_asic0_full32_rate_sweep_1ms_20260508_newest_sv_debug2_fix5/post_rbcam_dual_trace_validation.csv`.

The 1 MHz deassembly FIFO peak is `66/256` words with `0` full samples after matching the SV ingress FIFO depth to the VHDL `scfifo_w40d256` reference. Fill-level plots are under `reports/prof_int_002_post_rbcam_periodic_asic0_full32_rate_sweep_1ms_20260508_newest_sv_debug2_fix5/`.

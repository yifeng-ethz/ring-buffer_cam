# DV Plan: ring_buffer_cam

**DUT:** `ring_buffer_cam`  
**Primary RTL:** `ring_buffer_cam.vhd`, `ring_buffer_cam_v2_core.vhd`, `addr_enc_logic_partitioned.vhd`  
**Workflow:** local `dv-workflow` report-tree flow  
**Date:** 2026-04-16  
**Status:** migrated from legacy monolithic signoff into split-plan current tree

## 1. Scope

This plan covers standalone verification of the live `ring_buffer_cam` IP, including:

- ingress hit acceptance and egress drain correctness
- run-control state progression and flush / terminate behavior
- CSR identity, read/write semantics, and activity counters
- partitioned encoder behavior across the delivered timing-oriented configurations
- overwrite, cache-miss, and fill-level accounting
- sustained drain behavior, multi-key ordering, and stress conditions

The historical signoff record in [../SIGNOFF.md](../SIGNOFF.md) is retained as a dated baseline. It is not the active signoff source for this workflow.

## 2. Current Truth After 2026-04-16 Reruns

- The UVM bench is runnable on this host with Questa FSE.
- The CSR cases were checking the wrong register offsets against the live RTL. The bench now uses the real CSR map from `ring_buffer_cam_v2_core.vhd`.
- The scoreboard now hard-fails if accepted hits remain undrained at end of test. This prevents false-green functional smoke results.
- Live reruns currently show:
  - PASS: `test_cfg_reset_defaults`, `test_cfg_rw_semantics`, `test_cfg_activity_counters`, `test_single_push_pop`, `test_same_key_burst_128`, `test_same_key_burst_256`, `test_random_push_pop`
  - FAIL: `test_sequential_keys`, `test_overwrite_stress`, `test_random_multi_key`, `test_random_throughput`
- Continuous-frame baselines, checkpoint UCDB growth curves, and standalone code-coverage UCDBs are still missing.

## 3. Plan Files

| File | Role | Scope |
|---|---|---|
| [DV_PLAN.md](DV_PLAN.md) | entry point | scope, buckets, signoff rules |
| [DV_HARNESS.md](DV_HARNESS.md) | harness contract | agents, scoreboard, monitors, coverage, current gaps |
| [DV_BASIC.md](DV_BASIC.md) | `B001-B008` | deterministic bring-up and core feature cases |
| [DV_EDGE.md](DV_EDGE.md) | `E001-E008` | boundary, ordering, and partition corner cases |
| [DV_PROF.md](DV_PROF.md) | `P001-P006` | random, stress, soak, and throughput-oriented cases |
| [DV_ERROR.md](DV_ERROR.md) | `X001-X006` | injected faults, recovery, terminate/flush, illegal control |
| [DV_CROSS.md](DV_CROSS.md) | continuous-frame signoff | `bucket_frame`, `all_buckets_frame`, and curated mixed-pattern runs |
| `DV_REPORT.json` | machine-readable source | generated from `scripts/generate_dv_report.py` |
| `DV_REPORT.md`, `DV_COV.md`, `REPORT/` | generated review tree | dashboard plus per-case and per-run evidence |

## 4. Verification Targets

| Target | Description | Main evidence |
|---|---|---|
| T01 | Every accepted hit eventually drains with correct side data | hit monitor, out monitor, scoreboard |
| T02 | Subheader framing is correct for zero-hit and non-zero-hit drains | out monitor, protocol checks |
| T03 | CSR map matches the live RTL contract | CSR sequences, reset/readback checks |
| T04 | Activity counters track error, push, pop, overwrite, and cache-miss events | CSR reads plus directed stimulus |
| T05 | Multi-key and same-key traffic drain without corruption or silent loss | scoreboard, coverage, throughput monitor |
| T06 | Partitioned encoder and drain engine do not stall under supported contention | throughput monitor, per-partition cases |
| T07 | Terminate / flush / restart behavior is explicit and reproducible | run-control sequences, scoreboard, counter checks |
| T08 | Long-run mixed patterns expose overlap, arbitration, and counter-latency bugs | continuous-frame runs from [DV_CROSS.md](DV_CROSS.md) |

## 5. Execution Modes

The required execution modes are:

1. `isolated`: one case per fresh DUT start.
2. `bucket_frame`: all cases in one bucket executed in case-id order without a DUT restart.
3. `all_buckets_frame`: BASIC, EDGE, PROF, ERROR buckets executed in order without a DUT restart.

`bucket_frame` and `all_buckets_frame` are mandatory signoff baselines. The current harness does not close them yet; the gap is tracked in [DV_HARNESS.md](DV_HARNESS.md) and [DV_CROSS.md](DV_CROSS.md).

## 6. Coverage And Closure Rules

- Code coverage must ultimately report statement, branch, condition, expression, FSM state/transition, and toggle totals from UCDB artifacts.
- Random long runs must emit checkpoint UCDBs at log-spaced transaction counts.
- Until those artifacts exist, the generated dashboard uses passing-case closure as a temporary functional-closure proxy. That proxy is explicitly temporary and does not replace true covergroup / cross closure.
- Any case that does not add meaningful evidence must be either repaired or kept as a documented known gap. The report tree is not allowed to hide it.

## 7. Signoff Gates

The IP is not closed under the new workflow until all of the following are true:

- all promoted BASIC, EDGE, PROF, and ERROR cases have valid isolated evidence
- the remaining failing multi-key / overwrite / long-run cases are either fixed or demoted with written rationale
- `bucket_frame` and `all_buckets_frame` runs exist and are reviewable under `REPORT/cross/`
- checkpoint UCDB pages under `REPORT/txn_growth/` are populated for promoted random cases
- code-coverage UCDB capture is wired into the runner and reflected in `DV_COV.md`

## 8. Legacy Mapping

The old `Dxxx` / `Rxxx` prose in the historical plan has been promoted into the canonical `B/E/P/X` case IDs under the split files:

- deterministic contract cases live in [DV_BASIC.md](DV_BASIC.md)
- boundary and ordering cases live in [DV_EDGE.md](DV_EDGE.md)
- random and throughput cases live in [DV_PROF.md](DV_PROF.md)
- injected fault and recovery cases live in [DV_ERROR.md](DV_ERROR.md)

The split files are the authoritative case catalog for all future report generation.

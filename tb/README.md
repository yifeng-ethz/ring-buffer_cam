# `tb/` — `ring_buffer_cam` DV workspace

Active DV workflow root for the live `ring_buffer_cam` IP. Plan, harness, case buckets, and generated report tree all live under this directory. For the IP-level overview, Platform Designer contract, and preset summary, see the top-level [`../README.md`](../README.md).

## Structure

- [`DV_PLAN.md`](DV_PLAN.md) — workflow entry point, scope, buckets, and signoff rules.
- [`DV_HARNESS.md`](DV_HARNESS.md) — harness contract: agents, scoreboard, monitors, coverage, current gaps.
- [`DV_BASIC.md`](DV_BASIC.md) — `B001-B129` deterministic bring-up and core-feature cases.
- [`DV_EDGE.md`](DV_EDGE.md) — `E001-E129` boundary, ordering, and partition corner cases.
- [`DV_PROF.md`](DV_PROF.md) — `P001-P129` random, stress, soak, and throughput-oriented cases.
- [`DV_ERROR.md`](DV_ERROR.md) — `X001-X129` injected-fault, recovery, terminate/flush, and illegal-control cases.
- [`DV_CROSS.md`](DV_CROSS.md) — `CROSS-001-CROSS-129` continuous-frame signoff catalog (`bucket_frame`, `all_buckets_frame`, anchored hybrids, seed-swept and checkpoint soaks).
- [`DV_REPORT.md`](DV_REPORT.md) — generated chief-architect dashboard (source of truth is `DV_REPORT.json`).
- [`DV_COV.md`](DV_COV.md) — coverage summary (targets vs merged totals; per-bucket breakdown).
- [`DV_REPORT.json`](DV_REPORT.json) — machine-readable source of truth consumed by the report generator.
- [`BUG_HISTORY.md`](BUG_HISTORY.md) — live bug ledger across both the DUT RTL and the harness.
- [`REPORT/`](REPORT/) — generated per-bucket / per-case / per-cross evidence tree.
- [`uvm/`](uvm/) — active mixed-language UVM harness (drivers, monitors, scoreboard, coverage, tests).
- [`sim/`](sim/) — retained VHDL directed benches and legacy smoke runners.
- [`scripts/`](scripts/) — repo-local report generator and isolated-case runner.

## Reading order

1. [`DV_PLAN.md`](DV_PLAN.md) — what this workspace is verifying and which signoff gates are active.
2. [`DV_HARNESS.md`](DV_HARNESS.md) — how the bench is wired, what agents exist, and which observability pieces are still missing.
3. [`DV_REPORT.md`](DV_REPORT.md) — what is passing today and which cases are still unimplemented or failing.
4. [`DV_COV.md`](DV_COV.md) — coverage posture against the closure targets.
5. [`BUG_HISTORY.md`](BUG_HISTORY.md) — why cases have failed historically and which fixes are now landed.
6. Bucket catalogs ([`DV_BASIC.md`](DV_BASIC.md), [`DV_EDGE.md`](DV_EDGE.md), [`DV_PROF.md`](DV_PROF.md), [`DV_ERROR.md`](DV_ERROR.md), [`DV_CROSS.md`](DV_CROSS.md)) — authoritative case catalog per bucket; referenced by the report tree.

## Quick start

Ordered so every step reuses artifacts from earlier steps:

1. `bash uvm/run_uvm.sh compile` — compile the current mixed-language UVM bench against the packaged `default_p2_pipe4` RTL variant.
2. `bash uvm/run_uvm.sh test_single_push_pop 1` — smoke one promoted case through the full harness.
3. `bash uvm/run_uvm.sh cfg-matrix` — run the CSR configuration-space matrix (`test_cfg_reset_defaults`, `test_cfg_rw_semantics`, `test_cfg_activity_counters` across `p1/p2/p3/p4`).
4. `python3 scripts/run_isolated_cases.py` — replay the promoted isolated-case set and save UCDBs under `uvm/cov_after/`.
5. `python3 scripts/generate_dv_report.py` — regenerate [`DV_REPORT.md`](DV_REPORT.md), [`DV_REPORT.json`](DV_REPORT.json), [`DV_COV.md`](DV_COV.md), and the [`REPORT/`](REPORT/) tree from the isolated evidence set.
6. `bash sim/run_questa_pipeline_smoke.sh` — legacy VHDL directed smoke runner (kept for deterministic bring-up coverage).
7. `bash sim/run_questa_partitioned.sh` — legacy VHDL partitioned TB (TC1-TC9 catalog).

## Harness entry points

| Target | Purpose |
|---|---|
| `bash uvm/run_uvm.sh compile` | compile the mixed-language UVM bench against the active RTL variant |
| `bash uvm/run_uvm.sh <test> <seed>` | run one isolated UVM case with the given seed |
| `bash uvm/run_uvm.sh cfg-matrix` | run the CSR configuration-space matrix across `p1/p2/p3/p4` |
| `python3 scripts/run_isolated_cases.py` | replay the promoted isolated-case set under the workflow harness |
| `python3 scripts/generate_dv_report.py` | regenerate the report tree from isolated UCDBs |
| `bash sim/run_questa_pipeline_smoke.sh` | legacy VHDL pipeline smoke bench |
| `bash sim/run_questa_partitioned.sh` | legacy VHDL partitioned TB |
| `bash sim/run_questa_terminate_marker.sh` | legacy VHDL terminate-marker directed TB |

## Current posture

- The split `DV_*` tree under `tb/` is the active workflow. The older monolithic narrative in [`../doc/SIGNOFF.md`](../doc/SIGNOFF.md) is the master dashboard built on top of this evidence, not the primary DV owner.
- Live reruns on `2026-04-21` show the overwrite-pressure tail bug closed in RTL:
  - `P111` passes with `push=576 pop=512 overwrite=64 remaining=0`.
  - `P119` passes with `push=768 pop=512 overwrite=256 remaining=0`.
  - `B005` baseline still passes with `push=128 pop=128 remaining=0`.
- Validation note on `2026-04-22`: after rerunning `E115` and `E119` on the
  supported QuestaOne 2026 flow and regenerating
  `scripts/generate_dv_report.py`, the generated dashboard now records `365`
  evidenced isolated cases, `17` failing cases, `149` unimplemented cases, and
  `67.44% (348/516)` passing functional coverage on `default_p2_pipe4`. The
  broad nightly rerun is therefore still not a clean signoff pass; see
  [`DV_REPORT.md`](DV_REPORT.md) for the canonical active failing-case list.
- The next closure gap remains both breadth and the active failing set: `367`
  case-engine rows are implemented, `365` are currently evidenced, `149`
  planned cases are still unimplemented, and every continuous-frame `CROSS-*`
  run remains open. The generated dashboard surfaces that explicitly.
- See [`DV_REPORT.md`](DV_REPORT.md) for the current per-bucket evidence snapshot and [`BUG_HISTORY.md`](BUG_HISTORY.md) for the open ledger.

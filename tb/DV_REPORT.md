# ❌ DV Report — `ring_buffer_cam`

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-22` &nbsp;
**RTL variant:** `default_p2_pipe4` &nbsp; **Seed:** `1`

This page is the chief-architect dashboard. All per-case evidence lives under [`REPORT/`](REPORT/README.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / below target &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ❌ | failed_cases | `17` |
| ✅ | signoff_runs_with_failures | `0` |
| ⚠️ | unimplemented_cases | `157` |
| ✅ | stale_artifacts | `0` |

## Bugs

<!-- latest bug-ledger entries from tb/BUG_HISTORY.md; class = Harness or RTL; commit = fixing commit hash once recorded. -->

| status | bug_id | class | date | title | commit |
|:---:|---|---|---|---|---|
| ✅ | `BUG-063-H` | Harness | `2026-04-21` | The shared low-stage partition-latency helper anchored on a stage-4-only pulse instead of the real active-partition load event | `4c62cd0` |
| ✅ | `BUG-062-H` | Harness | `2026-04-21` | The generic idle helper treated a lone pending end-of-run marker as live traffic even after the DUT had already closed the lane | `b4e0daa` |
| ✅ | `BUG-061-R` | RTL | `2026-04-21` | `TERMINATING` control ready could acknowledge before lane-local end-of-run had actually closed the drain contract | `b4e0daa` |
| ✅ | `BUG-060-R` | RTL | `2026-04-21` | `push_erase` recomputed the just-written slot inside the remaining standalone timing-critical CAM erase cone | `1736898` |
| ✅ | `BUG-059-R` | RTL | `2026-04-21` | Wrap-overwrite `push_erase` could erase outside the configured ring span on non-power-of-two builds | `dab30da` |
| ✅ | `BUG-058-R` | RTL | `2026-04-21` | Non-power-of-two ring depths let the live write pointer escape the configured ring span | `acb9230` |
| ✅ | `BUG-057-R` | RTL | `2026-04-21` | Low-stage partitioned-encoder variants indexed `pipe_valid` beyond the active datapath width | `acb9230` |
| ✅ | `BUG-056-R` | RTL | `2026-04-21` | Settled SEARCH-tail overlap could still clobber a slot already captured in the frozen snapshot at the live write pointer | `07c0dae` |

## Formal / contract cases

<!-- scope = bucket or TOTAL aggregation of contract-style testcase execution -->
<!-- executed = cases with real isolated log evidence; executed_ratio = executed/planned cases -->
<!-- observed_txn = sum of scoreboard-observed transactions across executed cases -->
<!-- asserted_failures = summed UVM_ERROR/assert failures from those case logs -->
<!-- unexpected_outputs = summed scoreboard unexpected-output count from those case logs -->

| status | scope | planned | executed | executed_ratio | observed_txn | failing_cases | asserted_failures | unexpected_outputs |
|:---:|---|---:|---:|---:|---:|---:|---:|---:|
| ❌ | `BASIC` | 129 | 129 | 100.00% | 13771 | 2 | 3 | 0 |
| ❌ | `EDGE` | 129 | 37 | 28.68% | 39108 | 1 | 1 | 0 |
| ❌ | `PROF` | 129 | 87 | 67.44% | 2560245 | 14 | 59 | 0 |
| ⚠️ | `ERROR` | 129 | 106 | 82.17% | 2067 | 0 | 0 | 0 |
| ❌ | `TOTAL` | 516 | 359 | 69.57% | 2615191 | 17 | 63 | 0 |

## Bucket summary

<!-- status: overall per-bucket health; merged columns: bucket-local ordered-merge percentages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | functional |
|:---:|---|---:|---:|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 129 | 129 | stmt=99.09, branch=93.76, cond=81.76, expr=40.00, fsm_state=100.00, fsm_trans=73.33, toggle=70.74 | 98.45% (127/129) |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 129 | 37 | stmt=91.43, branch=81.12, cond=64.32, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=82.96 | 27.91% (36/129) |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 129 | 87 | stmt=93.13, branch=85.14, cond=71.35, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=81.60 | 56.59% (73/129) |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 129 | 106 | stmt=96.77, branch=88.83, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=80.00, toggle=67.00 | 82.17% (106/129) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ✅ | stmt | 97.17 | 95.0 |
| ✅ | branch | 90.63 | 90.0 |
| ℹ️ | cond | 75.68 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 80.00 | 90.0 |
| ✅ | toggle | 86.67 | 80.0 |

- functional coverage: `66.28% (342/516)`

## Cross / continuous-frame signoff

<!-- one row per run; follow the run_id link for the full transaction-growth curve. -->

| status | run_id | kind | build | seq | txns | cross_pct |
|:---:|---|---|---|---|---:|---:|

## Index

- [`REPORT/README.md`](REPORT/README.md) — reviewer entry point
- [`REPORT/buckets/`](REPORT/buckets/) — ordered-merge trace per bucket
- [`REPORT/cases/`](REPORT/cases/) — one page per case
- [`REPORT/cross/`](REPORT/cross/) — one page per continuous-frame run
- [`REPORT/txn_growth/`](REPORT/txn_growth/) — checkpoint UCDB curves for random cases
- [`DV_COV.md`](DV_COV.md) — coverage targets vs. merged totals (summary)
- [`DV_REPORT.json`](DV_REPORT.json) — machine-readable source of truth

_This dashboard is generated by `~/.codex/skills/dv-workflow/scripts/dv_report_gen.py`. Edits are overwritten; fix the JSON or the generator instead._

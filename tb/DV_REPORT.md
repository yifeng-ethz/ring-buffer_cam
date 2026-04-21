# ❌ DV Report — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-21` &nbsp;
**RTL variant:** `default_p2_pipe4` &nbsp; **Seed:** `1`

This page is the chief-architect dashboard. All per-case evidence lives under [`REPORT/`](REPORT/README.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / below target &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ❌ | failed_cases | `28` |
| ✅ | signoff_runs_with_failures | `0` |
| ⚠️ | unimplemented_cases | `174` |
| ✅ | stale_artifacts | `0` |

## Bugs

<!-- latest bug-ledger entries from tb/BUG_HISTORY.md; class = Harness or RTL; commit = fixing commit hash once recorded. -->

| status | bug_id | class | date | title | commit |
|:---:|---|---|---|---|---|
| ✅ | `BUG-062-H` | Harness | `2026-04-21` | The generic idle helper treated a lone pending end-of-run marker as live traffic even after the DUT had already closed the lane | `b4e0daa` |
| ✅ | `BUG-061-R` | RTL | `2026-04-21` | `TERMINATING` control ready could acknowledge before lane-local end-of-run had actually closed the drain contract | `b4e0daa` |
| ✅ | `BUG-060-R` | RTL | `2026-04-21` | `push_erase` recomputed the just-written slot inside the remaining standalone timing-critical CAM erase cone | `1736898` |
| ✅ | `BUG-059-R` | RTL | `2026-04-21` | Wrap-overwrite `push_erase` could erase outside the configured ring span on non-power-of-two builds | `dab30da` |
| ✅ | `BUG-058-R` | RTL | `2026-04-21` | Non-power-of-two ring depths let the live write pointer escape the configured ring span | `acb9230` |
| ✅ | `BUG-057-R` | RTL | `2026-04-21` | Low-stage partitioned-encoder variants indexed `pipe_valid` beyond the active datapath width | `acb9230` |
| ✅ | `BUG-056-R` | RTL | `2026-04-21` | Settled SEARCH-tail overlap could still clobber a slot already captured in the frozen snapshot at the live write pointer | `07c0dae` |
| ✅ | `BUG-055-R` | RTL | `2026-04-21` | Cross-key `push_write` could still perturb the SEARCH match fabric before the pop snapshot was frozen | `07c0dae` |

## Formal / contract cases

<!-- scope = bucket or TOTAL aggregation of contract-style testcase execution -->
<!-- executed = cases with real isolated log evidence; executed_ratio = executed/planned cases -->
<!-- observed_txn = sum of scoreboard-observed transactions across executed cases -->
<!-- asserted_failures = summed UVM_ERROR/assert failures from those case logs -->
<!-- unexpected_outputs = summed scoreboard unexpected-output count from those case logs -->

| status | scope | planned | executed | executed_ratio | observed_txn | failing_cases | asserted_failures | unexpected_outputs |
|:---:|---|---:|---:|---:|---:|---:|---:|---:|
| ❌ | `BASIC` | 129 | 125 | 96.90% | 13765 | 2 | 3 | 0 |
| ❌ | `EDGE` | 129 | 30 | 23.26% | 25560 | 1 | 1 | 0 |
| ❌ | `PROF` | 129 | 81 | 62.79% | 557365 | 16 | 59 | 0 |
| ❌ | `ERROR` | 129 | 106 | 82.17% | 2067 | 9 | 13 | 0 |
| ❌ | `TOTAL` | 516 | 342 | 66.28% | 598757 | 28 | 76 | 0 |

## Bucket summary

<!-- status: overall per-bucket health; merged columns: bucket-local ordered-merge percentages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | functional |
|:---:|---|---:|---:|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 129 | 125 | stmt=97.30, branch=89.66, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=73.33, toggle=70.74 | 95.35% (123/129) |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 129 | 30 | stmt=92.49, branch=83.42, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=65.77 | 22.48% (29/129) |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 129 | 81 | stmt=92.68, branch=84.24, cond=71.70, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.29 | 50.39% (65/129) |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 129 | 106 | stmt=96.77, branch=88.83, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=80.00, toggle=66.48 | 75.19% (97/129) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ✅ | stmt | 97.35 | 95.0 |
| ✅ | branch | 90.97 | 90.0 |
| ℹ️ | cond | 76.73 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 80.00 | 90.0 |
| ⚠️ | toggle | 72.53 | 80.0 |

- functional coverage: `60.85% (314/516)`

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

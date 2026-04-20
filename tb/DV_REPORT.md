# ⚠️ DV Report — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-21` &nbsp;
**RTL variant:** `default_p2_pipe4` &nbsp; **Seed:** `1`

This page is the chief-architect dashboard. All per-case evidence lives under [`REPORT/`](REPORT/README.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / below target &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ✅ | failed_cases | `0` |
| ✅ | signoff_runs_with_failures | `0` |
| ⚠️ | unimplemented_cases | `182` |
| ✅ | stale_artifacts | `0` |

## Bugs

<!-- latest bug-ledger entries from tb/BUG_HISTORY.md; class = Harness or RTL; commit = fixing commit hash once recorded. -->

| status | bug_id | class | date | title | commit |
|:---:|---|---|---|---|---|
| ✅ | `BUG-049-H` | Harness | `2026-04-21` | The first stale-slot injector forced the live side-RAM read bus and contaminated later reads in the same testcase | `2fd3115` |
| ✅ | `BUG-048-H` | Harness | `2026-04-21` | Cache-miss output beats left the scoreboard pending-drain queue uncleared | `2fd3115` |
| ✅ | `BUG-047-H` | Harness | `2026-04-21` | Cache-miss pop observations with `occupied=0` did not retire the live resident in the scoreboard | `2fd3115` |
| ✅ | `BUG-046-H` | Harness | `2026-04-20` | Clean terminate/drain only waited for the source queue, not for the internal deassembly FIFO, and could wedge heavy overwrite soaks | `cb365cf` |
| ✅ | `BUG-045-R` | RTL | `2026-04-20` | The equal-load partition drain scheduler held service on the just-issued partition even when another partition was already pending | `7c2beb7` |
| ✅ | `BUG-044-H` | Harness | `2026-04-20` | Backpressure-enabled PROF cases released `sink_ready` as soon as ingress stopped, not when the drain window finished | `8b9ad49` |
| ✅ | `BUG-043-H` | Harness | `2026-04-20` | The first `P042` staged late-arrival helper reused overlapping fingerprint space and doubled the aggregate ingress rate after the late-key launch | `97f956c` |
| ✅ | `BUG-042-R` | RTL | `2026-04-20` | The push path could retire one stale buffered push or overwrite after soft-reset because request generation was not gated by active run state | `08ad68b` |

## Formal / contract cases

<!-- scope = bucket or TOTAL aggregation of contract-style testcase execution -->
<!-- executed = cases with real isolated log evidence; executed_ratio = executed/planned cases -->
<!-- observed_txn = sum of scoreboard-observed transactions across executed cases -->
<!-- asserted_failures = summed UVM_ERROR/assert failures from those case logs -->
<!-- unexpected_outputs = summed scoreboard unexpected-output count from those case logs -->

| status | scope | planned | executed | executed_ratio | observed_txn | failing_cases | asserted_failures | unexpected_outputs |
|:---:|---|---:|---:|---:|---:|---:|---:|---:|
| ⚠️ | `BASIC` | 129 | 125 | 96.90% | 13825 | 0 | 0 | 0 |
| ⚠️ | `EDGE` | 129 | 30 | 23.26% | 25561 | 0 | 0 | 0 |
| ⚠️ | `PROF` | 129 | 73 | 56.59% | 554574 | 0 | 0 | 0 |
| ⚠️ | `ERROR` | 129 | 92 | 71.32% | 4353 | 0 | 0 | 0 |
| ⚠️ | `TOTAL` | 516 | 320 | 62.02% | 598313 | 0 | 0 | 0 |

## Bucket summary

<!-- status: overall per-bucket health; merged columns: bucket-local ordered-merge percentages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | functional |
|:---:|---|---:|---:|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 129 | 125 | stmt=95.48, branch=85.64, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.60 | 96.9% (125/129) |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 129 | 30 | stmt=94.76, branch=83.42, cond=69.35, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.47 | 23.26% (30/129) |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 129 | 73 | stmt=91.91, branch=82.33, cond=70.75, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.12 | 56.59% (73/129) |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 129 | 92 | stmt=95.38, branch=85.44, cond=70.97, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.10 | 71.32% (92/129) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ✅ | stmt | 95.97 | 95.0 |
| ⚠️ | branch | 84.23 | 90.0 |
| ℹ️ | cond | 72.52 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 72.33 | 80.0 |

- functional coverage: `62.02% (320/516)`

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

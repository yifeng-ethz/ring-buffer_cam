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
| ⚠️ | unimplemented_cases | `174` |
| ✅ | stale_artifacts | `0` |

## Bugs

<!-- latest bug-ledger entries from tb/BUG_HISTORY.md; class = Harness or RTL; commit = fixing commit hash once recorded. -->

| status | bug_id | class | date | title | commit |
|:---:|---|---|---|---|---|
| ⚠️ | `BUG-056-R` | RTL | `2026-04-21` | Settled SEARCH-tail overlap could still clobber a slot already captured in the frozen snapshot at the live write pointer | `pending` |
| ⚠️ | `BUG-055-R` | RTL | `2026-04-21` | Cross-key `push_write` could still perturb the SEARCH match fabric before the pop snapshot was frozen | `pending` |
| ✅ | `BUG-054-R` | RTL | `2026-04-21` | The push engine stopped draining already-buffered deassembly entries after lane-local end-of-run in `TERMINATING` | `00fc1b8` |
| ✅ | `BUG-053-R` | RTL | `2026-04-21` | Ingress `ready` stayed high after lane-local end-of-run in `TERMINATING`, so accepted beats were silently dropped | `00fc1b8` |
| ✅ | `BUG-052-R` | RTL | `2026-04-21` | The memory arbiter could still grant `push_write` after the pop snapshot had frozen in `LOAD/COUNT` | `00fc1b8` |
| ✅ | `BUG-051-R` | RTL | `2026-04-21` | The memory arbiter could still grant `push_write` while the pop engine was already in `DRAIN` | `00fc1b8` |
| ✅ | `BUG-050-H` | Harness | `2026-04-21` | The compat `scfifo` model truncated exact-full `usedw` and flooded long regressions with false warnings | `00fc1b8` |
| ✅ | `BUG-049-H` | Harness | `2026-04-21` | The first stale-slot injector forced the live side-RAM read bus and contaminated later reads in the same testcase | `2fd3115` |

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
| ⚠️ | `PROF` | 129 | 76 | 58.91% | 559659 | 0 | 0 | 0 |
| ⚠️ | `ERROR` | 129 | 92 | 71.32% | 4353 | 0 | 0 | 0 |
| ⚠️ | `TOTAL` | 516 | 323 | 62.60% | 603398 | 0 | 0 | 0 |

## Bucket summary

<!-- status: overall per-bucket health; merged columns: bucket-local ordered-merge percentages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | functional |
|:---:|---|---:|---:|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 129 | 125 | stmt=95.61, branch=85.60, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.60 | 96.9% (125/129) |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 129 | 30 | stmt=94.76, branch=83.42, cond=69.35, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.47 | 23.26% (30/129) |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 129 | 76 | stmt=91.81, branch=81.99, cond=70.75, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.17 | 58.91% (76/129) |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 129 | 92 | stmt=95.38, branch=85.44, cond=70.97, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.10 | 71.32% (92/129) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ✅ | stmt | 96.10 | 95.0 |
| ⚠️ | branch | 84.22 | 90.0 |
| ℹ️ | cond | 72.52 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 72.38 | 80.0 |

- functional coverage: `62.6% (323/516)`

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

# ⚠️ DV Report — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-20` &nbsp;
**RTL variant:** `default_p2_pipe4` &nbsp; **Seed:** `1`

This page is the chief-architect dashboard. All per-case evidence lives under [`REPORT/`](REPORT/README.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / below target &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ✅ | failed_cases | `0` |
| ✅ | signoff_runs_with_failures | `0` |
| ⚠️ | unimplemented_cases | `207` |
| ✅ | stale_artifacts | `0` |

## Bugs

<!-- latest bug-ledger entries from tb/BUG_HISTORY.md; class = Harness or RTL; commit = fixing commit hash once recorded. -->

| status | bug_id | class | date | title | commit |
|:---:|---|---|---|---|---|
| ✅ | `BUG-038-H` | Harness | `2026-04-20` | The first partition-profile promotion assumed exact-partition prefill isolated the next epoch to partition 1, but the live contract exposes a `p0->p1` handoff window instead | `ca1b044` |
| ✅ | `BUG-037-H` | Harness | `2026-04-20` | The default-build DV plan and report text drifted to `N_PARTITIONS=4` semantics even though the active dashboard build is `default_p2_pipe4` | `ca1b044` |
| ✅ | `BUG-036-H` | Harness | `2026-04-20` | Data-bearing 256-hit packet subheaders were misclassified as zero-hit headers when the 8-bit hit-count field wrapped | `ca1b044` |
| ✅ | `BUG-035-H` | Harness | `2026-04-20` | The promoted PROF ordering audit encoded a stronger FIFO contract than the product actually guarantees | `1c8118b` |
| ✅ | `BUG-034-H` | Harness | `2026-04-20` | The seeded random PROF promotion helpers did not actually propagate distinct per-case seeds | `1c8118b` |
| ✅ | `BUG-033-H` | Harness | `2026-04-20` | Long-run PROF generators reused overlapping low-order bits, so deep integrity runs could alias their own evidence | `1c8118b` |
| ✅ | `BUG-032-H` | Harness | `2026-04-20` | `profile_traffic_seq` could only model contiguous key windows, so interleaved active/silent-key profiles were not expressible faithfully | `d7ce37a` |
| ✅ | `BUG-031-H` | Harness | `2026-04-20` | The PROF silent-key plan text encoded the wrong contract for zero-hit searches | `d7ce37a` |

## Formal / contract cases

<!-- scope = bucket or TOTAL aggregation of contract-style testcase execution -->
<!-- executed = cases with real isolated log evidence; executed_ratio = executed/planned cases -->
<!-- observed_txn = sum of scoreboard-observed transactions across executed cases -->
<!-- asserted_failures = summed UVM_ERROR/assert failures from those case logs -->
<!-- unexpected_outputs = summed scoreboard unexpected-output count from those case logs -->

| status | scope | planned | executed | executed_ratio | observed_txn | failing_cases | asserted_failures | unexpected_outputs |
|:---:|---|---:|---:|---:|---:|---:|---:|---:|
| ⚠️ | `BASIC` | 129 | 120 | 93.02% | 13577 | 0 | 0 | 0 |
| ⚠️ | `EDGE` | 129 | 29 | 22.48% | 10561 | 0 | 0 | 0 |
| ⚠️ | `PROF` | 129 | 55 | 42.64% | 367951 | 0 | 0 | 0 |
| ⚠️ | `ERROR` | 129 | 84 | 65.12% | 4945 | 0 | 0 | 0 |
| ⚠️ | `TOTAL` | 516 | 288 | 55.81% | 397034 | 0 | 0 | 0 |

## Bucket summary

<!-- status: overall per-bucket health; merged columns: bucket-local ordered-merge percentages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | functional |
|:---:|---|---:|---:|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 129 | 120 | stmt=95.11, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.86 | 93.02% (120/129) |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 129 | 29 | stmt=95.31, branch=84.01, cond=69.35, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.09 | 22.48% (29/129) |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 129 | 55 | stmt=94.95, branch=83.47, cond=68.55, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.07 | 42.64% (55/129) |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 129 | 84 | stmt=95.94, branch=86.45, cond=70.97, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.58 | 65.12% (84/129) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ✅ | stmt | 96.37 | 95.0 |
| ⚠️ | branch | 86.05 | 90.0 |
| ℹ️ | cond | 72.52 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 73.60 | 80.0 |

- functional coverage: `55.81% (288/516)`

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

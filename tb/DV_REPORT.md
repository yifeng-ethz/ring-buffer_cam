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
| ⚠️ | unimplemented_cases | `200` |
| ✅ | stale_artifacts | `0` |

## Bugs

<!-- latest bug-ledger entries from tb/BUG_HISTORY.md; class = Harness or RTL; commit = fixing commit hash once recorded. -->

| status | bug_id | class | date | title | commit |
|:---:|---|---|---|---|---|
| ✅ | `BUG-043-H` | Harness | `2026-04-20` | The first `P042` staged late-arrival helper reused overlapping fingerprint space and doubled the aggregate ingress rate after the late-key launch | `97f956c` |
| ✅ | `BUG-042-R` | RTL | `2026-04-20` | The push path could retire one stale buffered push or overwrite after soft-reset because request generation was not gated by active run state | `08ad68b` |
| ✅ | `BUG-041-R` | RTL | `2026-04-20` | The pop engine could still consume stale descriptors after soft-reset had already returned the DUT to `IDLE` | `08ad68b` |
| ✅ | `BUG-040-R` | RTL | `2026-04-20` | CSR `soft_reset` only self-cleared bit1 and never reset the live DUT state, counters, or FIFOs | `08ad68b` |
| ✅ | `BUG-039-R` | RTL | `2026-04-20` | The pop descriptor generator ignored `pop_cmd_fifo_full` and could overrun the 16-entry descriptor FIFO under sustained backlog | `08ad68b` |
| ✅ | `BUG-038-H` | Harness | `2026-04-20` | The first partition-profile promotion assumed exact-partition prefill isolated the next epoch to partition 1, but the live contract exposes a `p0->p1` handoff window instead | `ca1b044` |
| ✅ | `BUG-037-H` | Harness | `2026-04-20` | The default-build DV plan and report text drifted to `N_PARTITIONS=4` semantics even though the active dashboard build is `default_p2_pipe4` | `ca1b044` |
| ✅ | `BUG-036-H` | Harness | `2026-04-20` | Data-bearing 256-hit packet subheaders were misclassified as zero-hit headers when the 8-bit hit-count field wrapped | `ca1b044` |

## Formal / contract cases

<!-- scope = bucket or TOTAL aggregation of contract-style testcase execution -->
<!-- executed = cases with real isolated log evidence; executed_ratio = executed/planned cases -->
<!-- observed_txn = sum of scoreboard-observed transactions across executed cases -->
<!-- asserted_failures = summed UVM_ERROR/assert failures from those case logs -->
<!-- unexpected_outputs = summed scoreboard unexpected-output count from those case logs -->

| status | scope | planned | executed | executed_ratio | observed_txn | failing_cases | asserted_failures | unexpected_outputs |
|:---:|---|---:|---:|---:|---:|---:|---:|---:|
| ⚠️ | `BASIC` | 129 | 120 | 93.02% | 13561 | 0 | 0 | 0 |
| ⚠️ | `EDGE` | 129 | 30 | 23.26% | 25561 | 0 | 0 | 0 |
| ⚠️ | `PROF` | 129 | 60 | 46.51% | 547951 | 0 | 0 | 0 |
| ⚠️ | `ERROR` | 129 | 92 | 71.32% | 4353 | 0 | 0 | 0 |
| ⚠️ | `TOTAL` | 516 | 302 | 58.53% | 591426 | 0 | 0 | 0 |

## Bucket summary

<!-- status: overall per-bucket health; merged columns: bucket-local ordered-merge percentages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | functional |
|:---:|---|---:|---:|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 129 | 120 | stmt=95.68, branch=85.84, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.86 | 93.02% (120/129) |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 129 | 30 | stmt=94.76, branch=83.42, cond=69.35, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.47 | 23.26% (30/129) |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 129 | 60 | stmt=94.66, branch=83.19, cond=68.55, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.29 | 46.51% (60/129) |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 129 | 92 | stmt=95.38, branch=85.44, cond=70.97, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.10 | 71.32% (92/129) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ✅ | stmt | 96.17 | 95.0 |
| ⚠️ | branch | 85.27 | 90.0 |
| ℹ️ | cond | 72.52 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 73.63 | 80.0 |

- functional coverage: `58.53% (302/516)`

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

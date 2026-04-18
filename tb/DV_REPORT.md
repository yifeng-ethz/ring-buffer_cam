# ⚠️ DV Report — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-18` &nbsp;
**RTL variant:** `default_p2_pipe4` &nbsp; **Seed:** `1`

This page is the chief-architect dashboard. All per-case evidence lives under [`REPORT/`](REPORT/README.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / below target &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ✅ | failed_cases | `0` |
| ✅ | signoff_runs_with_failures | `0` |
| ⚠️ | unimplemented_cases | `355` |
| ✅ | stale_artifacts | `0` |

## Bugs

<!-- latest bug-ledger entries from tb/BUG_HISTORY.md; class = Harness or RTL; commit = fixing commit hash once recorded. -->

| status | bug_id | class | date | title | commit |
|:---:|---|---|---|---|---|
| ⚠️ | `BUG-020-H` | Harness | `2026-04-18` | B116 sampled FILL_LEVEL before the fourth/eighth/etc push grant had actually landed | `pending` |
| ✅ | `BUG-019-H` | Harness | `2026-04-18` | E026 partition-walk harness observed the wrong partition index during skip logic | `63e3652` |
| ✅ | `BUG-018-H` | Harness | `2026-04-18` | DV report publication linked evidence into volatile `work_uvm` paths and collapsed after targeted reruns | `cec8e58` |
| ✅ | `BUG-017-H` | Harness | `2026-04-18` | Partition-stress BASIC cases hardcoded a 4-way geometry and sampled packetized output too late | `cec8e58` |
| ✅ | `BUG-016-H` | Harness | `2026-04-17` | Deep overwrite-profile stimulus reused the full scoreboard fingerprint tuple every 2048 hits | `e8c18fe` |
| ✅ | `BUG-015-H` | Harness | `2026-04-17` | Long-run output matching reused the pending-drain queue index as a live-slot index and could clear the wrong resident | `e8c18fe` |
| ✅ | `BUG-014-H` | Harness | `2026-04-17` | Long-run scoreboard lost drain traceability when a recycled slot was popped after the model had already cleared it | `e8c18fe` |
| ✅ | `BUG-013-H` | Harness | `2026-04-17` | Long-span EDGE cases assumed one subheader represented the whole same-key drain epoch | `d412d7a` |

## Formal / contract cases

<!-- scope = bucket or TOTAL aggregation of contract-style testcase execution -->
<!-- executed = cases with real isolated log evidence; executed_ratio = executed/planned cases -->
<!-- observed_txn = sum of scoreboard-observed transactions across executed cases -->
<!-- asserted_failures = summed UVM_ERROR/assert failures from those case logs -->
<!-- unexpected_outputs = summed scoreboard unexpected-output count from those case logs -->

| status | scope | planned | executed | executed_ratio | observed_txn | failing_cases | asserted_failures | unexpected_outputs |
|:---:|---|---:|---:|---:|---:|---:|---:|---:|
| ⚠️ | `BASIC` | 129 | 106 | 82.17% | 8673 | 0 | 0 | 0 |
| ⚠️ | `EDGE` | 129 | 24 | 18.60% | 9537 | 0 | 0 | 0 |
| ⚠️ | `PROF` | 129 | 9 | 6.98% | 8174 | 0 | 0 | 0 |
| ⚠️ | `ERROR` | 129 | 22 | 17.05% | 587 | 0 | 0 | 0 |
| ⚠️ | `TOTAL` | 516 | 161 | 31.20% | 26971 | 0 | 0 | 0 |

## Bucket summary

<!-- status: overall per-bucket health; merged columns: bucket-local ordered-merge percentages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | functional |
|:---:|---|---:|---:|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 129 | 106 | stmt=97.28, branch=89.33, cond=73.39, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.69 | 82.17% (106/129) |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 129 | 24 | stmt=95.56, branch=84.81, cond=69.35, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.10 | 18.6% (24/129) |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 129 | 9 | stmt=95.56, branch=84.99, cond=68.55, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=65.84 | 6.98% (9/129) |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 129 | 22 | stmt=94.20, branch=80.65, cond=67.74, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=41.16 | 17.05% (22/129) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ✅ | stmt | 97.33 | 95.0 |
| ⚠️ | branch | 89.51 | 90.0 |
| ℹ️ | cond | 73.39 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 72.93 | 80.0 |

- functional coverage: `31.2% (161/516)`

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

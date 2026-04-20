# DV Coverage Summary — ring_buffer_cam

This page is the coverage summary only. Per-case incremental coverage lives under
[`REPORT/cases/`](REPORT/cases/); per-bucket ordered-merge traces live under
[`REPORT/buckets/`](REPORT/buckets/).

## Targets vs merged totals

<!-- merged_pct = merge across all evidenced isolated-mode UCDBs across all buckets. -->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 96.84 | 95.0 |
| ⚠️ | branch | 87.57 | 90.0 |
| ℹ️ | cond | 72.52 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 73.24 | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 96.19 | 86.68 | 70.23 | 40.00 | 100.00 | 66.67 | 71.83 |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 95.44 | 84.31 | 69.35 | 40.00 | 100.00 | 66.67 | 67.09 |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 95.44 | 84.67 | 68.55 | 40.00 | 100.00 | 66.67 | 67.55 |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 95.94 | 86.45 | 70.97 | 40.00 | 100.00 | 66.67 | 67.58 |

## Continuous-frame baselines by build

<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->

| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns |
|:---:|---|---|---|---|---:|---|---|---|---:|---:|

_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._

# DV Coverage Summary — ring_buffer_cam

This page is the coverage summary only. Per-case incremental coverage lives under
[`REPORT/cases/`](REPORT/cases/); per-bucket ordered-merge traces live under
[`REPORT/buckets/`](REPORT/buckets/).

## Targets vs merged totals

<!-- merged_pct = merge across all evidenced isolated-mode UCDBs across all buckets. -->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 96.10 | 95.0 |
| ⚠️ | branch | 84.22 | 90.0 |
| ℹ️ | cond | 72.52 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 72.38 | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 95.61 | 85.60 | 70.23 | 40.00 | 100.00 | 66.67 | 70.60 |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 94.76 | 83.42 | 69.35 | 40.00 | 100.00 | 66.67 | 67.47 |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 91.81 | 81.99 | 70.75 | 40.00 | 100.00 | 66.67 | 68.17 |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 95.38 | 85.44 | 70.97 | 40.00 | 100.00 | 66.67 | 68.10 |

## Continuous-frame baselines by build

<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->

| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns |
|:---:|---|---|---|---|---:|---|---|---|---:|---:|

_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._

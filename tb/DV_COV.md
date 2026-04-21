# DV Coverage Summary — ring_buffer_cam

This page is the coverage summary only. Per-case incremental coverage lives under
[`REPORT/cases/`](REPORT/cases/); per-bucket ordered-merge traces live under
[`REPORT/buckets/`](REPORT/buckets/).

## Targets vs merged totals

<!-- merged_pct = merge across all evidenced isolated-mode UCDBs across all buckets. -->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 97.35 | 95.0 |
| ✅ | branch | 90.97 | 90.0 |
| ℹ️ | cond | 76.73 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 80.00 | 90.0 |
| ⚠️ | toggle | 72.53 | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 97.30 | 89.66 | 75.47 | 40.00 | 100.00 | 73.33 | 70.74 |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 92.49 | 83.42 | 69.81 | 40.00 | 100.00 | 66.67 | 65.77 |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 92.68 | 84.24 | 71.70 | 40.00 | 100.00 | 66.67 | 68.29 |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 96.77 | 88.83 | 74.84 | 40.00 | 100.00 | 80.00 | 66.48 |

## Continuous-frame baselines by build

<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->

| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns |
|:---:|---|---|---|---|---:|---|---|---|---:|---:|

_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._

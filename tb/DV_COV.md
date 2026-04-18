# DV Coverage Summary — ring_buffer_cam

This page is the coverage summary only. Per-case incremental coverage lives under
[`REPORT/cases/`](REPORT/cases/); per-bucket ordered-merge traces live under
[`REPORT/buckets/`](REPORT/buckets/).

## Targets vs merged totals

<!-- merged_pct = merge across all evidenced isolated-mode UCDBs across all buckets. -->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 97.33 | 95.0 |
| ⚠️ | branch | 89.33 | 90.0 |
| ℹ️ | cond | 73.39 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 72.89 | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 97.28 | 89.15 | 73.39 | 40.00 | 100.00 | 66.67 | 71.61 |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 95.56 | 84.81 | 69.35 | 40.00 | 100.00 | 66.67 | 67.10 |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 95.56 | 84.99 | 68.55 | 40.00 | 100.00 | 66.67 | 65.84 |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 94.20 | 80.65 | 67.74 | 30.00 | 100.00 | 66.67 | 41.16 |

## Continuous-frame baselines by build

<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->

| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns |
|:---:|---|---|---|---|---:|---|---|---|---:|---:|

_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._

# DV Coverage Summary — `ring_buffer_cam`

This page is the coverage summary only. Per-case incremental coverage lives under
[`REPORT/cases/`](REPORT/cases/); per-bucket ordered-merge traces live under
[`REPORT/buckets/`](REPORT/buckets/).

## Legend

✅ pass / closed &middot; ⚠️ partial / below target &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Targets vs merged totals

<!-- merged_pct = merge across all evidenced isolated-mode UCDBs across all buckets. -->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 97.17 | 95.0 |
| ✅ | branch | 90.63 | 90.0 |
| ℹ️ | cond | 75.68 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 80.00 | 90.0 |
| ✅ | toggle | 86.72 | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 99.09 | 93.76 | 81.76 | 40.00 | 100.00 | 73.33 | 70.74 |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 91.43 | 81.12 | 64.32 | 40.00 | 100.00 | 66.67 | 82.96 |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 93.13 | 85.14 | 71.35 | 40.00 | 100.00 | 66.67 | 81.64 |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 96.77 | 88.83 | 74.84 | 40.00 | 100.00 | 80.00 | 67.00 |

## Continuous-frame baselines by build

<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->

| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns |
|:---:|---|---|---|---|---:|---|---|---|---:|---:|

_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._

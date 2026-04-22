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
| ✅ | stmt | 96.79 | 95.0 |
| ✅ | branch | 90.95 | 90.0 |
| ℹ️ | cond | 74.58 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ✅ | fsm_trans | 100.00 | 90.0 |
| ✅ | toggle | 86.72 | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 99.13 | 93.87 | 81.46 | 40.00 | 100.00 | 73.33 | 70.72 |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 94.66 | 84.23 | 68.36 | 40.00 | 100.00 | 66.67 | 84.41 |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 92.85 | 84.76 | 71.35 | 40.00 | 100.00 | 66.67 | 81.88 |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 96.87 | 89.33 | 74.84 | 40.00 | 100.00 | 66.67 | 70.63 |

## Continuous-frame baselines by build

<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->

| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns |
|:---:|---|---|---|---|---:|---|---|---|---:|---:|
| ✅ | [`CROSS-008`](REPORT/cross/CROSS-008.md) | anchored_hybrid | default_p2_pipe4 | - | 1 | 92.59 | 83.33 | 59.99 | 88.8 | 2048 |
| ✅ | [`CROSS-009`](REPORT/cross/CROSS-009.md) | anchored_hybrid | default_p2_pipe4 | - | 1 | 92.64 | 83.50 | 60.19 | 88.8 | 2048 |
| ✅ | [`CROSS-010`](REPORT/cross/CROSS-010.md) | anchored_hybrid | default_p2_pipe4 | - | 1 | 92.44 | 83.00 | 57.86 | 80.9 | 784 |
| ✅ | [`CROSS-015`](REPORT/cross/CROSS-015.md) | anchored_hybrid | default_p2_pipe4 | - | 1 | 92.64 | 83.50 | 62.78 | 88.8 | 2048 |
| ✅ | [`CROSS-076`](REPORT/cross/CROSS-076.md) | seed_sweep | default_p2_pipe4 | - | 1 | 92.54 | 83.17 | 57.58 | 88.8 | 131072 |
| ✅ | [`CROSS-091`](REPORT/cross/CROSS-091.md) | seed_sweep | default_p2_pipe4 | - | 1 | 89.67 | 76.49 | 39.81 | 86.2 | 128 |

_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._

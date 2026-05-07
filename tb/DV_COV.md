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
| ✅ | stmt | 96.09 | 95.0 |
| ⚠️ | branch | 89.87 | 90.0 |
| ℹ️ | cond | 75.00 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ✅ | fsm_trans | 100.00 | 90.0 |
| ⚠️ | toggle | 73.79 | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 98.23 | 92.35 | 81.82 | 40.00 | 100.00 | 73.33 | 52.32 |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 94.37 | 83.80 | 68.89 | 40.00 | 100.00 | 66.67 | 70.50 |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 92.60 | 84.31 | 71.81 | 40.00 | 100.00 | 66.67 | 69.27 |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 96.48 | 88.48 | 75.31 | 40.00 | 100.00 | 66.67 | 51.54 |

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
| ❌ | [`CROSS-125`](REPORT/cross/CROSS-125.md) | checkpoint_soak | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ❌ | [`CROSS-126`](REPORT/cross/CROSS-126.md) | checkpoint_soak | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ❌ | [`CROSS-127`](REPORT/cross/CROSS-127.md) | checkpoint_soak | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ❌ | [`CROSS-128`](REPORT/cross/CROSS-128.md) | checkpoint_soak | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ❌ | [`CROSS-129`](REPORT/cross/CROSS-129.md) | checkpoint_soak | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |

_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._

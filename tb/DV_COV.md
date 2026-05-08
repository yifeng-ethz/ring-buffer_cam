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
| ⚠️ | stmt | 86.06 | 95.0 |
| ⚠️ | branch | 81.28 | 90.0 |
| ℹ️ | cond | 65.22 | - |
| ℹ️ | expr | 65.08 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 50.00 | 90.0 |
| ⚠️ | toggle | 41.77 | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 86.06 | 81.28 | 65.22 | 65.08 | 100.00 | 50.00 | 32.85 |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 85.57 | 82.27 | 66.00 | 63.08 | 100.00 | 50.00 | 33.01 |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 88.78 | 87.68 | 80.00 | 76.92 | 100.00 | 50.00 | 36.21 |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 94.02 | 87.75 | 80.00 | 75.38 | 100.00 | 61.11 | 30.80 |

## Continuous-frame baselines by build

<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->

| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns |
|:---:|---|---|---|---|---:|---|---|---|---:|---:|
| ❌ | [`CROSS-125`](REPORT/cross/CROSS-125.md) | checkpoint_soak | p4_n4_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ❌ | [`CROSS-126`](REPORT/cross/CROSS-126.md) | checkpoint_soak | p4_n4_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ❌ | [`CROSS-127`](REPORT/cross/CROSS-127.md) | checkpoint_soak | p4_n4_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ❌ | [`CROSS-128`](REPORT/cross/CROSS-128.md) | checkpoint_soak | p4_n4_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ❌ | [`CROSS-129`](REPORT/cross/CROSS-129.md) | checkpoint_soak | p4_n4_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |

_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._

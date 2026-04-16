# DV Coverage Summary — ring_buffer_cam

This page is the coverage summary only. Per-case incremental coverage lives under
[`REPORT/cases/`](REPORT/cases/); per-bucket ordered-merge traces live under
[`REPORT/buckets/`](REPORT/buckets/).

## Targets vs merged totals

<!-- merged_pct = merge across all evidenced isolated-mode UCDBs across all buckets. -->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ❓ | stmt | n/a | 95.0 |
| ❓ | branch | n/a | 90.0 |
| ❓ | cond | n/a | - |
| ❓ | expr | n/a | - |
| ❓ | fsm_state | n/a | 95.0 |
| ❓ | fsm_trans | n/a | 90.0 |
| ❓ | toggle | n/a | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | n/a | n/a | n/a | n/a | n/a | n/a | n/a |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | n/a | n/a | n/a | n/a | n/a | n/a | n/a |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | n/a | n/a | n/a | n/a | n/a | n/a | n/a |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | n/a | n/a | n/a | n/a | n/a | n/a | n/a |

## Continuous-frame baselines by build

<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->

| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns |
|:---:|---|---|---|---|---:|---|---|---|---:|---:|
| ⚠️ | [`CROSS-001`](REPORT/cross/CROSS-001.md) | bucket_frame | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-002`](REPORT/cross/CROSS-002.md) | bucket_frame | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-003`](REPORT/cross/CROSS-003.md) | bucket_frame | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-004`](REPORT/cross/CROSS-004.md) | bucket_frame | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-005`](REPORT/cross/CROSS-005.md) | all_buckets_frame | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-006`](REPORT/cross/CROSS-006.md) | all_buckets_frame | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-007`](REPORT/cross/CROSS-007.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-008`](REPORT/cross/CROSS-008.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-009`](REPORT/cross/CROSS-009.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-010`](REPORT/cross/CROSS-010.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-011`](REPORT/cross/CROSS-011.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-012`](REPORT/cross/CROSS-012.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-013`](REPORT/cross/CROSS-013.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-014`](REPORT/cross/CROSS-014.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-015`](REPORT/cross/CROSS-015.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-016`](REPORT/cross/CROSS-016.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-017`](REPORT/cross/CROSS-017.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-018`](REPORT/cross/CROSS-018.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-019`](REPORT/cross/CROSS-019.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-020`](REPORT/cross/CROSS-020.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-021`](REPORT/cross/CROSS-021.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-022`](REPORT/cross/CROSS-022.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-023`](REPORT/cross/CROSS-023.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-024`](REPORT/cross/CROSS-024.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-025`](REPORT/cross/CROSS-025.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-026`](REPORT/cross/CROSS-026.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-027`](REPORT/cross/CROSS-027.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-028`](REPORT/cross/CROSS-028.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-029`](REPORT/cross/CROSS-029.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-030`](REPORT/cross/CROSS-030.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-031`](REPORT/cross/CROSS-031.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-032`](REPORT/cross/CROSS-032.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-033`](REPORT/cross/CROSS-033.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-034`](REPORT/cross/CROSS-034.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-035`](REPORT/cross/CROSS-035.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-036`](REPORT/cross/CROSS-036.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-037`](REPORT/cross/CROSS-037.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-038`](REPORT/cross/CROSS-038.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-039`](REPORT/cross/CROSS-039.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-040`](REPORT/cross/CROSS-040.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-041`](REPORT/cross/CROSS-041.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-042`](REPORT/cross/CROSS-042.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-043`](REPORT/cross/CROSS-043.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-044`](REPORT/cross/CROSS-044.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-045`](REPORT/cross/CROSS-045.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-046`](REPORT/cross/CROSS-046.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-047`](REPORT/cross/CROSS-047.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-048`](REPORT/cross/CROSS-048.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-049`](REPORT/cross/CROSS-049.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-050`](REPORT/cross/CROSS-050.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-051`](REPORT/cross/CROSS-051.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-052`](REPORT/cross/CROSS-052.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-053`](REPORT/cross/CROSS-053.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-054`](REPORT/cross/CROSS-054.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-055`](REPORT/cross/CROSS-055.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-056`](REPORT/cross/CROSS-056.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-057`](REPORT/cross/CROSS-057.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-058`](REPORT/cross/CROSS-058.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-059`](REPORT/cross/CROSS-059.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-060`](REPORT/cross/CROSS-060.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-061`](REPORT/cross/CROSS-061.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-062`](REPORT/cross/CROSS-062.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-063`](REPORT/cross/CROSS-063.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-064`](REPORT/cross/CROSS-064.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-065`](REPORT/cross/CROSS-065.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-066`](REPORT/cross/CROSS-066.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-067`](REPORT/cross/CROSS-067.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-068`](REPORT/cross/CROSS-068.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-069`](REPORT/cross/CROSS-069.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-070`](REPORT/cross/CROSS-070.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-071`](REPORT/cross/CROSS-071.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-072`](REPORT/cross/CROSS-072.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-073`](REPORT/cross/CROSS-073.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-074`](REPORT/cross/CROSS-074.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-075`](REPORT/cross/CROSS-075.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-076`](REPORT/cross/CROSS-076.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-077`](REPORT/cross/CROSS-077.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-078`](REPORT/cross/CROSS-078.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-079`](REPORT/cross/CROSS-079.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-080`](REPORT/cross/CROSS-080.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-081`](REPORT/cross/CROSS-081.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-082`](REPORT/cross/CROSS-082.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-083`](REPORT/cross/CROSS-083.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-084`](REPORT/cross/CROSS-084.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-085`](REPORT/cross/CROSS-085.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-086`](REPORT/cross/CROSS-086.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-087`](REPORT/cross/CROSS-087.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-088`](REPORT/cross/CROSS-088.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-089`](REPORT/cross/CROSS-089.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-090`](REPORT/cross/CROSS-090.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-091`](REPORT/cross/CROSS-091.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-092`](REPORT/cross/CROSS-092.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-093`](REPORT/cross/CROSS-093.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-094`](REPORT/cross/CROSS-094.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-095`](REPORT/cross/CROSS-095.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-096`](REPORT/cross/CROSS-096.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-097`](REPORT/cross/CROSS-097.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-098`](REPORT/cross/CROSS-098.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-099`](REPORT/cross/CROSS-099.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-100`](REPORT/cross/CROSS-100.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-101`](REPORT/cross/CROSS-101.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-102`](REPORT/cross/CROSS-102.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-103`](REPORT/cross/CROSS-103.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-104`](REPORT/cross/CROSS-104.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-105`](REPORT/cross/CROSS-105.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-106`](REPORT/cross/CROSS-106.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-107`](REPORT/cross/CROSS-107.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-108`](REPORT/cross/CROSS-108.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-109`](REPORT/cross/CROSS-109.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-110`](REPORT/cross/CROSS-110.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-111`](REPORT/cross/CROSS-111.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-112`](REPORT/cross/CROSS-112.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-113`](REPORT/cross/CROSS-113.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-114`](REPORT/cross/CROSS-114.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-115`](REPORT/cross/CROSS-115.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-116`](REPORT/cross/CROSS-116.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-117`](REPORT/cross/CROSS-117.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-118`](REPORT/cross/CROSS-118.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-119`](REPORT/cross/CROSS-119.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-120`](REPORT/cross/CROSS-120.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-121`](REPORT/cross/CROSS-121.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-122`](REPORT/cross/CROSS-122.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-123`](REPORT/cross/CROSS-123.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-124`](REPORT/cross/CROSS-124.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-125`](REPORT/cross/CROSS-125.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-126`](REPORT/cross/CROSS-126.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-127`](REPORT/cross/CROSS-127.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-128`](REPORT/cross/CROSS-128.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |
| ⚠️ | [`CROSS-129`](REPORT/cross/CROSS-129.md) | C | default_p2_pipe4 | - | 0 | n/a | n/a | n/a | 0.0 | 0 |

_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._

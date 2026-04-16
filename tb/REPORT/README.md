# ring_buffer_cam — REPORT index

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-16` &nbsp;
**RTL variant:** `default_p2_pipe4` &nbsp; **Seed:** `1`

## Legend

✅ pass / closed / target met &middot; ⚠️ partial / below target / known limitation &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Buckets

<!-- click a bucket row to open its ordered-merge trace and linked per-case pages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) |
|:---:|---|---:|---:|---|
| ⚠️ | [`BASIC`](buckets/BASIC.md) | 129 | 0 | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a |
| ⚠️ | [`EDGE`](buckets/EDGE.md) | 129 | 0 | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a |
| ⚠️ | [`PROF`](buckets/PROF.md) | 129 | 0 | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a |
| ⚠️ | [`ERROR`](buckets/ERROR.md) | 129 | 3 | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a |

## Cross / continuous-frame runs

| status | run_id | kind | build | bucket | seq | txns | cross_pct |
|:---:|---|---|---|---|---|---:|---:|
| ⚠️ | [`CROSS-001`](cross/CROSS-001.md) | bucket_frame | default_p2_pipe4 | - | `B001-B129` in order | 0 | 0.0 |
| ⚠️ | [`CROSS-002`](cross/CROSS-002.md) | bucket_frame | default_p2_pipe4 | - | `E001-E129` in order | 0 | 0.0 |
| ⚠️ | [`CROSS-003`](cross/CROSS-003.md) | bucket_frame | default_p2_pipe4 | - | `P001-P129` in order | 0 | 0.0 |
| ⚠️ | [`CROSS-004`](cross/CROSS-004.md) | bucket_frame | default_p2_pipe4 | - | `X001-X129` in order | 0 | 0.0 |
| ⚠️ | [`CROSS-005`](cross/CROSS-005.md) | all_buckets_frame | default_p2_pipe4 | - | `BASIC -> EDGE -> PROF -> ERROR` | 0 | 0.0 |
| ⚠️ | [`CROSS-006`](cross/CROSS-006.md) | all_buckets_frame | default_p2_pipe4 | - | default `bucket_frame` + repeated B005/B006 | 0 | 0.0 |
| ⚠️ | [`CROSS-007`](cross/CROSS-007.md) | C | default_p2_pipe4 | - | `GOOD-ERROR-GOOD` sequence with explicit restart gap | 0 | 0.0 |
| ⚠️ | [`CROSS-008`](cross/CROSS-008.md) | C | default_p2_pipe4 | - | `GOOD-ERROR-GOOD` with push-heavy first window | 0 | 0.0 |
| ⚠️ | [`CROSS-009`](cross/CROSS-009.md) | C | default_p2_pipe4 | - | `GOOD-ERROR-GOOD` with same-key then multi-key | 0 | 0.0 |
| ⚠️ | [`CROSS-010`](cross/CROSS-010.md) | C | default_p2_pipe4 | - | `GOOD-ERROR-GOOD` with overwrite pressure window | 0 | 0.0 |
| ⚠️ | [`CROSS-011`](cross/CROSS-011.md) | C | default_p2_pipe4 | - | `overlap-priority` short pattern with two high-rate starters | 0 | 0.0 |
| ⚠️ | [`CROSS-012`](cross/CROSS-012.md) | C | default_p2_pipe4 | - | `overlap-priority` long pattern with delayed launch gaps | 0 | 0.0 |
| ⚠️ | [`CROSS-013`](cross/CROSS-013.md) | C | default_p2_pipe4 | - | `counter-delay` pattern: frontdoor command vs backdoor counter | 0 | 0.0 |
| ⚠️ | [`CROSS-014`](cross/CROSS-014.md) | C | default_p2_pipe4 | - | `counter-delay` repeated in all buckets | 0 | 0.0 |
| ⚠️ | [`CROSS-015`](cross/CROSS-015.md) | C | default_p2_pipe4 | - | `repeated-case interleaving` with B/E/P repeats | 0 | 0.0 |
| ⚠️ | [`CROSS-016`](cross/CROSS-016.md) | C | default_p2_pipe4 | - | `randomized-time composition` with variable phase gaps | 0 | 0.0 |
| ⚠️ | [`CROSS-017`](cross/CROSS-017.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #1 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-018`](cross/CROSS-018.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #2 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-019`](cross/CROSS-019.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #3 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-020`](cross/CROSS-020.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #4 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-021`](cross/CROSS-021.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #5 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-022`](cross/CROSS-022.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #6 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-023`](cross/CROSS-023.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #7 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-024`](cross/CROSS-024.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #8 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-025`](cross/CROSS-025.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #9 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-026`](cross/CROSS-026.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #10 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-027`](cross/CROSS-027.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #11 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-028`](cross/CROSS-028.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #12 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-029`](cross/CROSS-029.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #13 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-030`](cross/CROSS-030.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #14 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-031`](cross/CROSS-031.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #15 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-032`](cross/CROSS-032.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #16 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-033`](cross/CROSS-033.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #17 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-034`](cross/CROSS-034.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #18 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-035`](cross/CROSS-035.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #19 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-036`](cross/CROSS-036.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #20 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-037`](cross/CROSS-037.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #21 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-038`](cross/CROSS-038.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #22 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-039`](cross/CROSS-039.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #23 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-040`](cross/CROSS-040.md) | C | default_p2_pipe4 | - | GOOD-ERROR-GOOD variant #24 with randomized phase ordering | 0 | 0.0 |
| ⚠️ | [`CROSS-041`](cross/CROSS-041.md) | C | default_p2_pipe4 | - | overlap-priority variant #1 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-042`](cross/CROSS-042.md) | C | default_p2_pipe4 | - | overlap-priority variant #2 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-043`](cross/CROSS-043.md) | C | default_p2_pipe4 | - | overlap-priority variant #3 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-044`](cross/CROSS-044.md) | C | default_p2_pipe4 | - | overlap-priority variant #4 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-045`](cross/CROSS-045.md) | C | default_p2_pipe4 | - | overlap-priority variant #5 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-046`](cross/CROSS-046.md) | C | default_p2_pipe4 | - | overlap-priority variant #6 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-047`](cross/CROSS-047.md) | C | default_p2_pipe4 | - | overlap-priority variant #7 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-048`](cross/CROSS-048.md) | C | default_p2_pipe4 | - | overlap-priority variant #8 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-049`](cross/CROSS-049.md) | C | default_p2_pipe4 | - | overlap-priority variant #9 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-050`](cross/CROSS-050.md) | C | default_p2_pipe4 | - | overlap-priority variant #10 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-051`](cross/CROSS-051.md) | C | default_p2_pipe4 | - | overlap-priority variant #11 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-052`](cross/CROSS-052.md) | C | default_p2_pipe4 | - | overlap-priority variant #12 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-053`](cross/CROSS-053.md) | C | default_p2_pipe4 | - | overlap-priority variant #13 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-054`](cross/CROSS-054.md) | C | default_p2_pipe4 | - | overlap-priority variant #14 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-055`](cross/CROSS-055.md) | C | default_p2_pipe4 | - | overlap-priority variant #15 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-056`](cross/CROSS-056.md) | C | default_p2_pipe4 | - | overlap-priority variant #16 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-057`](cross/CROSS-057.md) | C | default_p2_pipe4 | - | overlap-priority variant #17 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-058`](cross/CROSS-058.md) | C | default_p2_pipe4 | - | overlap-priority variant #18 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-059`](cross/CROSS-059.md) | C | default_p2_pipe4 | - | overlap-priority variant #19 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-060`](cross/CROSS-060.md) | C | default_p2_pipe4 | - | overlap-priority variant #20 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-061`](cross/CROSS-061.md) | C | default_p2_pipe4 | - | overlap-priority variant #21 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-062`](cross/CROSS-062.md) | C | default_p2_pipe4 | - | overlap-priority variant #22 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-063`](cross/CROSS-063.md) | C | default_p2_pipe4 | - | overlap-priority variant #23 with repeated short windows | 0 | 0.0 |
| ⚠️ | [`CROSS-064`](cross/CROSS-064.md) | C | default_p2_pipe4 | - | counter-delay variant #1 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-065`](cross/CROSS-065.md) | C | default_p2_pipe4 | - | counter-delay variant #2 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-066`](cross/CROSS-066.md) | C | default_p2_pipe4 | - | counter-delay variant #3 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-067`](cross/CROSS-067.md) | C | default_p2_pipe4 | - | counter-delay variant #4 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-068`](cross/CROSS-068.md) | C | default_p2_pipe4 | - | counter-delay variant #5 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-069`](cross/CROSS-069.md) | C | default_p2_pipe4 | - | counter-delay variant #6 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-070`](cross/CROSS-070.md) | C | default_p2_pipe4 | - | counter-delay variant #7 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-071`](cross/CROSS-071.md) | C | default_p2_pipe4 | - | counter-delay variant #8 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-072`](cross/CROSS-072.md) | C | default_p2_pipe4 | - | counter-delay variant #9 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-073`](cross/CROSS-073.md) | C | default_p2_pipe4 | - | counter-delay variant #10 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-074`](cross/CROSS-074.md) | C | default_p2_pipe4 | - | counter-delay variant #11 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-075`](cross/CROSS-075.md) | C | default_p2_pipe4 | - | counter-delay variant #12 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-076`](cross/CROSS-076.md) | C | default_p2_pipe4 | - | counter-delay variant #13 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-077`](cross/CROSS-077.md) | C | default_p2_pipe4 | - | counter-delay variant #14 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-078`](cross/CROSS-078.md) | C | default_p2_pipe4 | - | counter-delay variant #15 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-079`](cross/CROSS-079.md) | C | default_p2_pipe4 | - | counter-delay variant #16 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-080`](cross/CROSS-080.md) | C | default_p2_pipe4 | - | counter-delay variant #17 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-081`](cross/CROSS-081.md) | C | default_p2_pipe4 | - | counter-delay variant #18 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-082`](cross/CROSS-082.md) | C | default_p2_pipe4 | - | counter-delay variant #19 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-083`](cross/CROSS-083.md) | C | default_p2_pipe4 | - | counter-delay variant #20 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-084`](cross/CROSS-084.md) | C | default_p2_pipe4 | - | counter-delay variant #21 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-085`](cross/CROSS-085.md) | C | default_p2_pipe4 | - | counter-delay variant #22 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-086`](cross/CROSS-086.md) | C | default_p2_pipe4 | - | counter-delay variant #23 with randomized counter visibility | 0 | 0.0 |
| ⚠️ | [`CROSS-087`](cross/CROSS-087.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #1 | 0 | 0.0 |
| ⚠️ | [`CROSS-088`](cross/CROSS-088.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #2 | 0 | 0.0 |
| ⚠️ | [`CROSS-089`](cross/CROSS-089.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #3 | 0 | 0.0 |
| ⚠️ | [`CROSS-090`](cross/CROSS-090.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #4 | 0 | 0.0 |
| ⚠️ | [`CROSS-091`](cross/CROSS-091.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #5 | 0 | 0.0 |
| ⚠️ | [`CROSS-092`](cross/CROSS-092.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #6 | 0 | 0.0 |
| ⚠️ | [`CROSS-093`](cross/CROSS-093.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #7 | 0 | 0.0 |
| ⚠️ | [`CROSS-094`](cross/CROSS-094.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #8 | 0 | 0.0 |
| ⚠️ | [`CROSS-095`](cross/CROSS-095.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #9 | 0 | 0.0 |
| ⚠️ | [`CROSS-096`](cross/CROSS-096.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #10 | 0 | 0.0 |
| ⚠️ | [`CROSS-097`](cross/CROSS-097.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #11 | 0 | 0.0 |
| ⚠️ | [`CROSS-098`](cross/CROSS-098.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #12 | 0 | 0.0 |
| ⚠️ | [`CROSS-099`](cross/CROSS-099.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #13 | 0 | 0.0 |
| ⚠️ | [`CROSS-100`](cross/CROSS-100.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #14 | 0 | 0.0 |
| ⚠️ | [`CROSS-101`](cross/CROSS-101.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #15 | 0 | 0.0 |
| ⚠️ | [`CROSS-102`](cross/CROSS-102.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #16 | 0 | 0.0 |
| ⚠️ | [`CROSS-103`](cross/CROSS-103.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #17 | 0 | 0.0 |
| ⚠️ | [`CROSS-104`](cross/CROSS-104.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #18 | 0 | 0.0 |
| ⚠️ | [`CROSS-105`](cross/CROSS-105.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #19 | 0 | 0.0 |
| ⚠️ | [`CROSS-106`](cross/CROSS-106.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #20 | 0 | 0.0 |
| ⚠️ | [`CROSS-107`](cross/CROSS-107.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #21 | 0 | 0.0 |
| ⚠️ | [`CROSS-108`](cross/CROSS-108.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #22 | 0 | 0.0 |
| ⚠️ | [`CROSS-109`](cross/CROSS-109.md) | C | default_p2_pipe4 | - | repeated-case interleaving variant #23 | 0 | 0.0 |
| ⚠️ | [`CROSS-110`](cross/CROSS-110.md) | C | default_p2_pipe4 | - | randomized-time composition variant #1 | 0 | 0.0 |
| ⚠️ | [`CROSS-111`](cross/CROSS-111.md) | C | default_p2_pipe4 | - | randomized-time composition variant #2 | 0 | 0.0 |
| ⚠️ | [`CROSS-112`](cross/CROSS-112.md) | C | default_p2_pipe4 | - | randomized-time composition variant #3 | 0 | 0.0 |
| ⚠️ | [`CROSS-113`](cross/CROSS-113.md) | C | default_p2_pipe4 | - | randomized-time composition variant #4 | 0 | 0.0 |
| ⚠️ | [`CROSS-114`](cross/CROSS-114.md) | C | default_p2_pipe4 | - | randomized-time composition variant #5 | 0 | 0.0 |
| ⚠️ | [`CROSS-115`](cross/CROSS-115.md) | C | default_p2_pipe4 | - | randomized-time composition variant #6 | 0 | 0.0 |
| ⚠️ | [`CROSS-116`](cross/CROSS-116.md) | C | default_p2_pipe4 | - | randomized-time composition variant #7 | 0 | 0.0 |
| ⚠️ | [`CROSS-117`](cross/CROSS-117.md) | C | default_p2_pipe4 | - | randomized-time composition variant #8 | 0 | 0.0 |
| ⚠️ | [`CROSS-118`](cross/CROSS-118.md) | C | default_p2_pipe4 | - | randomized-time composition variant #9 | 0 | 0.0 |
| ⚠️ | [`CROSS-119`](cross/CROSS-119.md) | C | default_p2_pipe4 | - | randomized-time composition variant #10 | 0 | 0.0 |
| ⚠️ | [`CROSS-120`](cross/CROSS-120.md) | C | default_p2_pipe4 | - | randomized-time composition variant #11 | 0 | 0.0 |
| ⚠️ | [`CROSS-121`](cross/CROSS-121.md) | C | default_p2_pipe4 | - | randomized-time composition variant #12 | 0 | 0.0 |
| ⚠️ | [`CROSS-122`](cross/CROSS-122.md) | C | default_p2_pipe4 | - | randomized-time composition variant #13 | 0 | 0.0 |
| ⚠️ | [`CROSS-123`](cross/CROSS-123.md) | C | default_p2_pipe4 | - | randomized-time composition variant #14 | 0 | 0.0 |
| ⚠️ | [`CROSS-124`](cross/CROSS-124.md) | C | default_p2_pipe4 | - | randomized-time composition variant #15 | 0 | 0.0 |
| ⚠️ | [`CROSS-125`](cross/CROSS-125.md) | C | default_p2_pipe4 | - | randomized-time composition variant #16 | 0 | 0.0 |
| ⚠️ | [`CROSS-126`](cross/CROSS-126.md) | C | default_p2_pipe4 | - | randomized-time composition variant #17 | 0 | 0.0 |
| ⚠️ | [`CROSS-127`](cross/CROSS-127.md) | C | default_p2_pipe4 | - | randomized-time composition variant #18 | 0 | 0.0 |
| ⚠️ | [`CROSS-128`](cross/CROSS-128.md) | C | default_p2_pipe4 | - | randomized-time composition variant #19 | 0 | 0.0 |
| ⚠️ | [`CROSS-129`](cross/CROSS-129.md) | C | default_p2_pipe4 | - | randomized-time composition variant #20 | 0 | 0.0 |

## Random long-run cases

<!-- each random case has a txn_growth page; pages are pending until checkpoint UCDBs exist. -->

| status | case_id | bucket | observed_txn | growth_page |
|:---:|---|---|---:|---|
| ❓ | [`P001`](cases/P001.md) | PROF | 0 | [growth](txn_growth/P001.md) |
| ❓ | [`P002`](cases/P002.md) | PROF | 0 | [growth](txn_growth/P002.md) |
| ❓ | [`P003`](cases/P003.md) | PROF | 0 | [growth](txn_growth/P003.md) |
| ❓ | [`P005`](cases/P005.md) | PROF | 0 | [growth](txn_growth/P005.md) |
| ❓ | [`P006`](cases/P006.md) | PROF | 0 | [growth](txn_growth/P006.md) |
| ❓ | [`P007`](cases/P007.md) | PROF | 0 | [growth](txn_growth/P007.md) |
| ❓ | [`P008`](cases/P008.md) | PROF | 0 | [growth](txn_growth/P008.md) |
| ❓ | [`P009`](cases/P009.md) | PROF | 0 | [growth](txn_growth/P009.md) |
| ❓ | [`P010`](cases/P010.md) | PROF | 0 | [growth](txn_growth/P010.md) |
| ❓ | [`P011`](cases/P011.md) | PROF | 0 | [growth](txn_growth/P011.md) |
| ❓ | [`P012`](cases/P012.md) | PROF | 0 | [growth](txn_growth/P012.md) |
| ❓ | [`P013`](cases/P013.md) | PROF | 0 | [growth](txn_growth/P013.md) |
| ❓ | [`P014`](cases/P014.md) | PROF | 0 | [growth](txn_growth/P014.md) |
| ❓ | [`P015`](cases/P015.md) | PROF | 0 | [growth](txn_growth/P015.md) |
| ❓ | [`P016`](cases/P016.md) | PROF | 0 | [growth](txn_growth/P016.md) |
| ❓ | [`P017`](cases/P017.md) | PROF | 0 | [growth](txn_growth/P017.md) |
| ❓ | [`P018`](cases/P018.md) | PROF | 0 | [growth](txn_growth/P018.md) |
| ❓ | [`P019`](cases/P019.md) | PROF | 0 | [growth](txn_growth/P019.md) |
| ❓ | [`P020`](cases/P020.md) | PROF | 0 | [growth](txn_growth/P020.md) |
| ❓ | [`P021`](cases/P021.md) | PROF | 0 | [growth](txn_growth/P021.md) |
| ❓ | [`P022`](cases/P022.md) | PROF | 0 | [growth](txn_growth/P022.md) |
| ❓ | [`P023`](cases/P023.md) | PROF | 0 | [growth](txn_growth/P023.md) |
| ❓ | [`P024`](cases/P024.md) | PROF | 0 | [growth](txn_growth/P024.md) |
| ❓ | [`P025`](cases/P025.md) | PROF | 0 | [growth](txn_growth/P025.md) |
| ❓ | [`P026`](cases/P026.md) | PROF | 0 | [growth](txn_growth/P026.md) |
| ❓ | [`P027`](cases/P027.md) | PROF | 0 | [growth](txn_growth/P027.md) |
| ❓ | [`P028`](cases/P028.md) | PROF | 0 | [growth](txn_growth/P028.md) |
| ❓ | [`P029`](cases/P029.md) | PROF | 0 | [growth](txn_growth/P029.md) |
| ❓ | [`P030`](cases/P030.md) | PROF | 0 | [growth](txn_growth/P030.md) |
| ❓ | [`P031`](cases/P031.md) | PROF | 0 | [growth](txn_growth/P031.md) |
| ❓ | [`P032`](cases/P032.md) | PROF | 0 | [growth](txn_growth/P032.md) |
| ❓ | [`P033`](cases/P033.md) | PROF | 0 | [growth](txn_growth/P033.md) |
| ❓ | [`P034`](cases/P034.md) | PROF | 0 | [growth](txn_growth/P034.md) |
| ❓ | [`P035`](cases/P035.md) | PROF | 0 | [growth](txn_growth/P035.md) |
| ❓ | [`P036`](cases/P036.md) | PROF | 0 | [growth](txn_growth/P036.md) |
| ❓ | [`P037`](cases/P037.md) | PROF | 0 | [growth](txn_growth/P037.md) |
| ❓ | [`P038`](cases/P038.md) | PROF | 0 | [growth](txn_growth/P038.md) |
| ❓ | [`P039`](cases/P039.md) | PROF | 0 | [growth](txn_growth/P039.md) |
| ❓ | [`P040`](cases/P040.md) | PROF | 0 | [growth](txn_growth/P040.md) |
| ❓ | [`P041`](cases/P041.md) | PROF | 0 | [growth](txn_growth/P041.md) |
| ❓ | [`P042`](cases/P042.md) | PROF | 0 | [growth](txn_growth/P042.md) |
| ❓ | [`P043`](cases/P043.md) | PROF | 0 | [growth](txn_growth/P043.md) |
| ❓ | [`P044`](cases/P044.md) | PROF | 0 | [growth](txn_growth/P044.md) |
| ❓ | [`P045`](cases/P045.md) | PROF | 0 | [growth](txn_growth/P045.md) |
| ❓ | [`P046`](cases/P046.md) | PROF | 0 | [growth](txn_growth/P046.md) |
| ❓ | [`P047`](cases/P047.md) | PROF | 0 | [growth](txn_growth/P047.md) |
| ❓ | [`P048`](cases/P048.md) | PROF | 0 | [growth](txn_growth/P048.md) |
| ❓ | [`P049`](cases/P049.md) | PROF | 0 | [growth](txn_growth/P049.md) |
| ❓ | [`P050`](cases/P050.md) | PROF | 0 | [growth](txn_growth/P050.md) |
| ❓ | [`P051`](cases/P051.md) | PROF | 0 | [growth](txn_growth/P051.md) |
| ❓ | [`P052`](cases/P052.md) | PROF | 0 | [growth](txn_growth/P052.md) |
| ❓ | [`P053`](cases/P053.md) | PROF | 0 | [growth](txn_growth/P053.md) |
| ❓ | [`P054`](cases/P054.md) | PROF | 0 | [growth](txn_growth/P054.md) |
| ❓ | [`P055`](cases/P055.md) | PROF | 0 | [growth](txn_growth/P055.md) |
| ❓ | [`P056`](cases/P056.md) | PROF | 0 | [growth](txn_growth/P056.md) |
| ❓ | [`P057`](cases/P057.md) | PROF | 0 | [growth](txn_growth/P057.md) |
| ❓ | [`P058`](cases/P058.md) | PROF | 0 | [growth](txn_growth/P058.md) |
| ❓ | [`P059`](cases/P059.md) | PROF | 0 | [growth](txn_growth/P059.md) |
| ❓ | [`P060`](cases/P060.md) | PROF | 0 | [growth](txn_growth/P060.md) |
| ❓ | [`P061`](cases/P061.md) | PROF | 0 | [growth](txn_growth/P061.md) |
| ❓ | [`P062`](cases/P062.md) | PROF | 0 | [growth](txn_growth/P062.md) |
| ❓ | [`P063`](cases/P063.md) | PROF | 0 | [growth](txn_growth/P063.md) |
| ❓ | [`P064`](cases/P064.md) | PROF | 0 | [growth](txn_growth/P064.md) |
| ❓ | [`P065`](cases/P065.md) | PROF | 0 | [growth](txn_growth/P065.md) |
| ❓ | [`P066`](cases/P066.md) | PROF | 0 | [growth](txn_growth/P066.md) |
| ❓ | [`P067`](cases/P067.md) | PROF | 0 | [growth](txn_growth/P067.md) |
| ❓ | [`P068`](cases/P068.md) | PROF | 0 | [growth](txn_growth/P068.md) |
| ❓ | [`P069`](cases/P069.md) | PROF | 0 | [growth](txn_growth/P069.md) |
| ❓ | [`P070`](cases/P070.md) | PROF | 0 | [growth](txn_growth/P070.md) |
| ❓ | [`P071`](cases/P071.md) | PROF | 0 | [growth](txn_growth/P071.md) |
| ❓ | [`P072`](cases/P072.md) | PROF | 0 | [growth](txn_growth/P072.md) |
| ❓ | [`P073`](cases/P073.md) | PROF | 0 | [growth](txn_growth/P073.md) |
| ❓ | [`P074`](cases/P074.md) | PROF | 0 | [growth](txn_growth/P074.md) |
| ❓ | [`P075`](cases/P075.md) | PROF | 0 | [growth](txn_growth/P075.md) |
| ❓ | [`P076`](cases/P076.md) | PROF | 0 | [growth](txn_growth/P076.md) |
| ❓ | [`P077`](cases/P077.md) | PROF | 0 | [growth](txn_growth/P077.md) |
| ❓ | [`P078`](cases/P078.md) | PROF | 0 | [growth](txn_growth/P078.md) |
| ❓ | [`P079`](cases/P079.md) | PROF | 0 | [growth](txn_growth/P079.md) |
| ❓ | [`P080`](cases/P080.md) | PROF | 0 | [growth](txn_growth/P080.md) |
| ❓ | [`P081`](cases/P081.md) | PROF | 0 | [growth](txn_growth/P081.md) |
| ❓ | [`P082`](cases/P082.md) | PROF | 0 | [growth](txn_growth/P082.md) |
| ❓ | [`P083`](cases/P083.md) | PROF | 0 | [growth](txn_growth/P083.md) |
| ❓ | [`P084`](cases/P084.md) | PROF | 0 | [growth](txn_growth/P084.md) |
| ❓ | [`P085`](cases/P085.md) | PROF | 0 | [growth](txn_growth/P085.md) |
| ❓ | [`P086`](cases/P086.md) | PROF | 0 | [growth](txn_growth/P086.md) |
| ❓ | [`P087`](cases/P087.md) | PROF | 0 | [growth](txn_growth/P087.md) |
| ❓ | [`P088`](cases/P088.md) | PROF | 0 | [growth](txn_growth/P088.md) |
| ❓ | [`P089`](cases/P089.md) | PROF | 0 | [growth](txn_growth/P089.md) |
| ❓ | [`P090`](cases/P090.md) | PROF | 0 | [growth](txn_growth/P090.md) |
| ❓ | [`P091`](cases/P091.md) | PROF | 0 | [growth](txn_growth/P091.md) |
| ❓ | [`P092`](cases/P092.md) | PROF | 0 | [growth](txn_growth/P092.md) |
| ❓ | [`P093`](cases/P093.md) | PROF | 0 | [growth](txn_growth/P093.md) |
| ❓ | [`P094`](cases/P094.md) | PROF | 0 | [growth](txn_growth/P094.md) |
| ❓ | [`P095`](cases/P095.md) | PROF | 0 | [growth](txn_growth/P095.md) |
| ❓ | [`P096`](cases/P096.md) | PROF | 0 | [growth](txn_growth/P096.md) |
| ❓ | [`P097`](cases/P097.md) | PROF | 0 | [growth](txn_growth/P097.md) |
| ❓ | [`P098`](cases/P098.md) | PROF | 0 | [growth](txn_growth/P098.md) |
| ❓ | [`P099`](cases/P099.md) | PROF | 0 | [growth](txn_growth/P099.md) |
| ❓ | [`P100`](cases/P100.md) | PROF | 0 | [growth](txn_growth/P100.md) |
| ❓ | [`P101`](cases/P101.md) | PROF | 0 | [growth](txn_growth/P101.md) |
| ❓ | [`P102`](cases/P102.md) | PROF | 0 | [growth](txn_growth/P102.md) |
| ❓ | [`P103`](cases/P103.md) | PROF | 0 | [growth](txn_growth/P103.md) |
| ❓ | [`P104`](cases/P104.md) | PROF | 0 | [growth](txn_growth/P104.md) |
| ❓ | [`P105`](cases/P105.md) | PROF | 0 | [growth](txn_growth/P105.md) |
| ❓ | [`P106`](cases/P106.md) | PROF | 0 | [growth](txn_growth/P106.md) |
| ❓ | [`P107`](cases/P107.md) | PROF | 0 | [growth](txn_growth/P107.md) |
| ❓ | [`P108`](cases/P108.md) | PROF | 0 | [growth](txn_growth/P108.md) |
| ❓ | [`P109`](cases/P109.md) | PROF | 0 | [growth](txn_growth/P109.md) |
| ❓ | [`P110`](cases/P110.md) | PROF | 0 | [growth](txn_growth/P110.md) |
| ❓ | [`P111`](cases/P111.md) | PROF | 0 | [growth](txn_growth/P111.md) |
| ❓ | [`P112`](cases/P112.md) | PROF | 0 | [growth](txn_growth/P112.md) |
| ❓ | [`P113`](cases/P113.md) | PROF | 0 | [growth](txn_growth/P113.md) |
| ❓ | [`P114`](cases/P114.md) | PROF | 0 | [growth](txn_growth/P114.md) |
| ❓ | [`P115`](cases/P115.md) | PROF | 0 | [growth](txn_growth/P115.md) |
| ❓ | [`P116`](cases/P116.md) | PROF | 0 | [growth](txn_growth/P116.md) |
| ❓ | [`P117`](cases/P117.md) | PROF | 0 | [growth](txn_growth/P117.md) |
| ❓ | [`P118`](cases/P118.md) | PROF | 0 | [growth](txn_growth/P118.md) |
| ❓ | [`P119`](cases/P119.md) | PROF | 0 | [growth](txn_growth/P119.md) |
| ❓ | [`P120`](cases/P120.md) | PROF | 0 | [growth](txn_growth/P120.md) |
| ❓ | [`P121`](cases/P121.md) | PROF | 0 | [growth](txn_growth/P121.md) |
| ❓ | [`P122`](cases/P122.md) | PROF | 0 | [growth](txn_growth/P122.md) |
| ❓ | [`P123`](cases/P123.md) | PROF | 0 | [growth](txn_growth/P123.md) |
| ❓ | [`P124`](cases/P124.md) | PROF | 0 | [growth](txn_growth/P124.md) |
| ❓ | [`P125`](cases/P125.md) | PROF | 0 | [growth](txn_growth/P125.md) |
| ❓ | [`P126`](cases/P126.md) | PROF | 0 | [growth](txn_growth/P126.md) |
| ❓ | [`P127`](cases/P127.md) | PROF | 0 | [growth](txn_growth/P127.md) |
| ❓ | [`P128`](cases/P128.md) | PROF | 0 | [growth](txn_growth/P128.md) |

## Totals

<!-- merged_total_code_coverage is the merge across all evidenced cases in all buckets. -->

- planned_cases = `516`
- evidenced_cases = `3`
- excluded_cases = `0`
- merged total code coverage: `stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a`
- functional coverage: `0.58% (3/516)`

---
_[Dashboard](../DV_REPORT.md) &middot; [Coverage](../DV_COV.md)_

# DV_CROSS.md — ring_buffer_cam

**Companion to:** [DV_PLAN.md](DV_PLAN.md), [DV_HARNESS.md](DV_HARNESS.md)
**Canonical Run Range:** `CROSS-001-CROSS-129`

## 1) Purpose

Cross coverage validates composition behavior that isolated cases cannot represent: no-restart bucket runs, long mixed GOOD/ERROR patterns, and randomized timing compositions. These are required before final signoff closure.

## 2) Mandatory Baselines

| case_id | kind | implementation | scenario | primary checks |
|---|---|---|---|---|
| CROSS-001 | bucket_frame | planned | `B001-B129` in order | basic cases compose without DUT restart |
| CROSS-002 | bucket_frame | planned | `E001-E129` in order | edge cases compose without DUT restart |
| CROSS-003 | bucket_frame | planned | `P001-P129` in order | profile cases compose without DUT restart |
| CROSS-004 | bucket_frame | planned | `X001-X129` in order | error/recovery cases compose without DUT restart |
| CROSS-005 | all_buckets_frame | planned | `BASIC -> EDGE -> PROF -> ERROR` | full-frame end-to-end composition |
| CROSS-006 | all_buckets_frame | planned | default `bucket_frame` + repeated B005/B006 | verifies frame restart boundaries and scoreboards across repeated deterministic IDs |

## 3) Curated Long Mixed Patterns

| case_id | method | implementation | scenario | primary checks |
|---|---|---|---|---|
| CROSS-007 | C | planned | `GOOD-ERROR-GOOD` sequence with explicit restart gap | counters rise only in ERROR window; recovery traffic drains cleanly |
| CROSS-008 | C | planned | `GOOD-ERROR-GOOD` with push-heavy first window | pre-error occupancy does not leak into post-error window |
| CROSS-009 | C | planned | `GOOD-ERROR-GOOD` with same-key then multi-key | same-key order preserved despite recovery boundary |
| CROSS-010 | C | planned | `GOOD-ERROR-GOOD` with overwrite pressure window | overwrite and inerr counters are orthogonal |
| CROSS-011 | C | planned | `overlap-priority` short pattern with two high-rate starters | overlap monitor captures deterministic acceptance priority |
| CROSS-012 | C | planned | `overlap-priority` long pattern with delayed launch gaps | near-simultaneous launches keep explainable arbitration |
| CROSS-013 | C | planned | `counter-delay` pattern: frontdoor command vs backdoor counter | backdoor delayed counters appear in bounded time window |
| CROSS-014 | C | planned | `counter-delay` repeated in all buckets | consistency of delayed counter visibility across buckets |
| CROSS-015 | C | planned | `repeated-case interleaving` with B/E/P repeats | case-boundary ownership survives repeated invocation |
| CROSS-016 | C | planned | `randomized-time composition` with variable phase gaps | composition is insensitive to idle gaps within bounded timing limits |

# Canonical Planned Cross Cases (CROSS-017 to CROSS-129)

| case_id | method | implementation | scenario | primary checks |
|---|---|---|---|---|
| CROSS-017 | C | planned | GOOD-ERROR-GOOD variant #1 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-018 | C | planned | GOOD-ERROR-GOOD variant #2 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-019 | C | planned | GOOD-ERROR-GOOD variant #3 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-020 | C | planned | GOOD-ERROR-GOOD variant #4 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-021 | C | planned | GOOD-ERROR-GOOD variant #5 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-022 | C | planned | GOOD-ERROR-GOOD variant #6 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-023 | C | planned | GOOD-ERROR-GOOD variant #7 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-024 | C | planned | GOOD-ERROR-GOOD variant #8 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-025 | C | planned | GOOD-ERROR-GOOD variant #9 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-026 | C | planned | GOOD-ERROR-GOOD variant #10 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-027 | C | planned | GOOD-ERROR-GOOD variant #11 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-028 | C | planned | GOOD-ERROR-GOOD variant #12 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-029 | C | planned | GOOD-ERROR-GOOD variant #13 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-030 | C | planned | GOOD-ERROR-GOOD variant #14 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-031 | C | planned | GOOD-ERROR-GOOD variant #15 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-032 | C | planned | GOOD-ERROR-GOOD variant #16 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-033 | C | planned | GOOD-ERROR-GOOD variant #17 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-034 | C | planned | GOOD-ERROR-GOOD variant #18 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-035 | C | planned | GOOD-ERROR-GOOD variant #19 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-036 | C | planned | GOOD-ERROR-GOOD variant #20 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-037 | C | planned | GOOD-ERROR-GOOD variant #21 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-038 | C | planned | GOOD-ERROR-GOOD variant #22 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-039 | C | planned | GOOD-ERROR-GOOD variant #23 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-040 | C | planned | GOOD-ERROR-GOOD variant #24 with randomized phase ordering | inter-window continuity and full cleanup between windows |
| CROSS-041 | C | planned | overlap-priority variant #1 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-042 | C | planned | overlap-priority variant #2 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-043 | C | planned | overlap-priority variant #3 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-044 | C | planned | overlap-priority variant #4 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-045 | C | planned | overlap-priority variant #5 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-046 | C | planned | overlap-priority variant #6 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-047 | C | planned | overlap-priority variant #7 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-048 | C | planned | overlap-priority variant #8 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-049 | C | planned | overlap-priority variant #9 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-050 | C | planned | overlap-priority variant #10 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-051 | C | planned | overlap-priority variant #11 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-052 | C | planned | overlap-priority variant #12 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-053 | C | planned | overlap-priority variant #13 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-054 | C | planned | overlap-priority variant #14 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-055 | C | planned | overlap-priority variant #15 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-056 | C | planned | overlap-priority variant #16 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-057 | C | planned | overlap-priority variant #17 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-058 | C | planned | overlap-priority variant #18 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-059 | C | planned | overlap-priority variant #19 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-060 | C | planned | overlap-priority variant #20 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-061 | C | planned | overlap-priority variant #21 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-062 | C | planned | overlap-priority variant #22 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-063 | C | planned | overlap-priority variant #23 with repeated short windows | overlap monitor must identify deterministic accepted winner |
| CROSS-064 | C | planned | counter-delay variant #1 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-065 | C | planned | counter-delay variant #2 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-066 | C | planned | counter-delay variant #3 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-067 | C | planned | counter-delay variant #4 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-068 | C | planned | counter-delay variant #5 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-069 | C | planned | counter-delay variant #6 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-070 | C | planned | counter-delay variant #7 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-071 | C | planned | counter-delay variant #8 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-072 | C | planned | counter-delay variant #9 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-073 | C | planned | counter-delay variant #10 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-074 | C | planned | counter-delay variant #11 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-075 | C | planned | counter-delay variant #12 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-076 | C | planned | counter-delay variant #13 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-077 | C | planned | counter-delay variant #14 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-078 | C | planned | counter-delay variant #15 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-079 | C | planned | counter-delay variant #16 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-080 | C | planned | counter-delay variant #17 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-081 | C | planned | counter-delay variant #18 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-082 | C | planned | counter-delay variant #19 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-083 | C | planned | counter-delay variant #20 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-084 | C | planned | counter-delay variant #21 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-085 | C | planned | counter-delay variant #22 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-086 | C | planned | counter-delay variant #23 with randomized counter visibility | bounded-delay checker validates expected timing of INERR/PUSH/POP/OVERWRITE/CACHE_MISS updates |
| CROSS-087 | C | planned | repeated-case interleaving variant #1 | composer repeats selected IDs without clearing required pending state |
| CROSS-088 | C | planned | repeated-case interleaving variant #2 | composer repeats selected IDs without clearing required pending state |
| CROSS-089 | C | planned | repeated-case interleaving variant #3 | composer repeats selected IDs without clearing required pending state |
| CROSS-090 | C | planned | repeated-case interleaving variant #4 | composer repeats selected IDs without clearing required pending state |
| CROSS-091 | C | planned | repeated-case interleaving variant #5 | composer repeats selected IDs without clearing required pending state |
| CROSS-092 | C | planned | repeated-case interleaving variant #6 | composer repeats selected IDs without clearing required pending state |
| CROSS-093 | C | planned | repeated-case interleaving variant #7 | composer repeats selected IDs without clearing required pending state |
| CROSS-094 | C | planned | repeated-case interleaving variant #8 | composer repeats selected IDs without clearing required pending state |
| CROSS-095 | C | planned | repeated-case interleaving variant #9 | composer repeats selected IDs without clearing required pending state |
| CROSS-096 | C | planned | repeated-case interleaving variant #10 | composer repeats selected IDs without clearing required pending state |
| CROSS-097 | C | planned | repeated-case interleaving variant #11 | composer repeats selected IDs without clearing required pending state |
| CROSS-098 | C | planned | repeated-case interleaving variant #12 | composer repeats selected IDs without clearing required pending state |
| CROSS-099 | C | planned | repeated-case interleaving variant #13 | composer repeats selected IDs without clearing required pending state |
| CROSS-100 | C | planned | repeated-case interleaving variant #14 | composer repeats selected IDs without clearing required pending state |
| CROSS-101 | C | planned | repeated-case interleaving variant #15 | composer repeats selected IDs without clearing required pending state |
| CROSS-102 | C | planned | repeated-case interleaving variant #16 | composer repeats selected IDs without clearing required pending state |
| CROSS-103 | C | planned | repeated-case interleaving variant #17 | composer repeats selected IDs without clearing required pending state |
| CROSS-104 | C | planned | repeated-case interleaving variant #18 | composer repeats selected IDs without clearing required pending state |
| CROSS-105 | C | planned | repeated-case interleaving variant #19 | composer repeats selected IDs without clearing required pending state |
| CROSS-106 | C | planned | repeated-case interleaving variant #20 | composer repeats selected IDs without clearing required pending state |
| CROSS-107 | C | planned | repeated-case interleaving variant #21 | composer repeats selected IDs without clearing required pending state |
| CROSS-108 | C | planned | repeated-case interleaving variant #22 | composer repeats selected IDs without clearing required pending state |
| CROSS-109 | C | planned | repeated-case interleaving variant #23 | composer repeats selected IDs without clearing required pending state |
| CROSS-110 | C | planned | randomized-time composition variant #1 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-111 | C | planned | randomized-time composition variant #2 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-112 | C | planned | randomized-time composition variant #3 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-113 | C | planned | randomized-time composition variant #4 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-114 | C | planned | randomized-time composition variant #5 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-115 | C | planned | randomized-time composition variant #6 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-116 | C | planned | randomized-time composition variant #7 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-117 | C | planned | randomized-time composition variant #8 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-118 | C | planned | randomized-time composition variant #9 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-119 | C | planned | randomized-time composition variant #10 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-120 | C | planned | randomized-time composition variant #11 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-121 | C | planned | randomized-time composition variant #12 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-122 | C | planned | randomized-time composition variant #13 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-123 | C | planned | randomized-time composition variant #14 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-124 | C | planned | randomized-time composition variant #15 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-125 | C | planned | randomized-time composition variant #16 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-126 | C | planned | randomized-time composition variant #17 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-127 | C | planned | randomized-time composition variant #18 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-128 | C | planned | randomized-time composition variant #19 | randomized gaps and seeds preserve global correctness and reproducibility |
| CROSS-129 | C | planned | randomized-time composition variant #20 | randomized gaps and seeds preserve global correctness and reproducibility |

## 4) Closure Notes

- These cases are not counted as closed unless `bucket_frame` and `all_buckets_frame` runs are executable without reset-per-case and are evidence-backed in `REPORT/cross/`.
- `GOOD-ERROR-GOOD`, `overlap-priority`, `counter-delay`, `repeated-case interleaving`, and `randomized-time composition` variants above are mandatory signoff families.
- Cross closure is gated by harness updates: run composer, state carry-over scoreboard, overlap monitor, and delayed counter observer.

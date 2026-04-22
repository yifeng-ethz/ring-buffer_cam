# DV_CROSS.md — ring_buffer_cam

**Companion to:** [DV_PLAN.md](DV_PLAN.md), [DV_HARNESS.md](DV_HARNESS.md), [BUG_HISTORY.md](BUG_HISTORY.md)
**Canonical Run Range:** `CROSS-001-CROSS-129`
**Intent:** long-run cross-signoff regression that composes the direct catalog (B/E/P/X) into continuous-frame runs, promotes anchored direct patterns to seed-swept randomized soaks, and drives merged functional + code coverage to closure.

## 1) Purpose And Long-Run Philosophy

The direct catalog (`B001-B129`, `E001-E129`, `P001-P129`, `X001-X132`) proves each RTL contract on a clean DUT. Cross runs prove the contracts compose and survive time and noise. They exist to:

- exercise every direct case in a continuous frame without reset-per-case, so any case-boundary cleanup gap is visible
- promote curated direct patterns (GOOD-ERROR-GOOD, overlap, overwrite pressure, counter delay) to randomized seed-swept soaks so the same invariants are re-hit millions of cycles apart with different arrival phases
- hit the FSM / counter / arbitration corners that the direct cases only graze, until merged code coverage clears the targets in [DV_COV.md](DV_COV.md)
- seed explicit traps for every bug in [BUG_HISTORY.md](BUG_HISTORY.md) so a regression cannot reintroduce the same class of defect silently

## 2) Direct → Random Promotion Ladder

Every cross run is one of five ladders. The ladder name is the first token in the scenario column of the catalog table in §6.

| ladder | how the stimulus is produced | what it proves beyond direct cases | typical txns per run |
|---|---|---|---|
| `bucket_frame` | direct cases in their declared order, no DUT restart | case boundaries leave no carry-over in scoreboard, CSR, or DUT state | 10-50k |
| `all_buckets_frame` | BASIC → EDGE → PROF → ERROR in order, no restart | full-stack composition; FLUSH+TERMINATE lifecycle handled once per bucket transition only | 50-200k |
| `anchored_hybrid` | one or more direct cases are pinned (the "anchor") with randomized inter-case gap, randomized side-data, randomized EXPECTED_LATENCY | the anchor's invariants hold when surrounding traffic is random | 20-100k |
| `seed_sweep` | one random case (typically P0xx) replayed with N orthogonal LCG seeds | per-case coverage bins union across seeds; seed axis is part of closure | 100k-1M per seed |
| `checkpoint_soak` | pure randomized traffic over 1M-10M txn with log-spaced UCDB checkpoints | coverage growth curve; catches bins that only saturate deep into a run; exposes BUG-007-style long-horizon counter bugs | 1M-10M |

Every `anchored_hybrid` and above saves one isolated UCDB plus a checkpoint UCDB per log-spaced milestone under [REPORT/txn_growth/](REPORT/txn_growth/). `bucket_frame` and `all_buckets_frame` save one merged UCDB under [REPORT/cross/](REPORT/cross/).

## 3) Randomization Axes

The composer draws from these axes. The axis table is the contract with the seed file; a cross run's metadata line must name every axis it varies.

| axis | values | notes |
|---|---|---|
| `seed` | 32-bit LCG seed | drives all other axes for the run; recorded in every case evidence page |
| `key_pool` | 1, 2, 4, 8, 16, 256 | uniform, Zipf s∈{1.0, 1.2, 1.5}, skewed {70/20/5/5, 99/1/0/0}, round-robin, adversarial-alternating |
| `push_shape` | continuous, Poisson λ∈{0.3, 0.5, 0.7, 0.9, 1.0}, burst (len∈[16,256], gap∈[0,256]) | `λ≥0.9` probes legal overwrite |
| `sink_ready` | 100%, Bernoulli p∈{0.3, 0.5, 0.7, 0.9} | `<100%` is blocked until `tb_top.sv` ready wire lands (see [DV_HARNESS.md §8](DV_HARNESS.md)) |
| `expected_latency` | `min`, `default=2000`, `max=65535`, mid-run sweep every 50-200k cyc | `EXPECTED_LATENCY` CSR readback checked after each change |
| `n_partitions` / `encoder_pipe_stages` | P={2, 4}, E={1, 2, 3, 4} | build-axis; one run per non-default build |
| `run_ctrl_injection` | none, single TERM, single FLUSH, periodic TERM (1/{2,5,20,100}k), periodic FLUSH, TERM+FLUSH interleaved, illegal word | composes with any `push_shape` |
| `error_rate` | 0%, 1%, 10%, 25% of beats with `asi_hit_type1_error(0)=1` | crossed with `filter_inerr ∈ {0, 1}` |
| `side_data` | fixed, sweep monotonic 31-bit range, random | drives the side-RAM toggle coverage |

## 4) Closure Targets

Merged across all cross runs (union of per-run UCDBs), the closure bar is:

| metric | target | merge scope |
|---|---|---|
| stmt | `≥95%` | all CROSS merged with all isolated B/E/P/X runs |
| branch | `≥90%` | same |
| fsm_state | `≥95%` | per-FSM: `run_state_cmd`, `pop_engine_state`, `push_state`, arbiter decision_reg |
| fsm_trans | `≥90%` | same |
| toggle | `≥80%` | all RTL nets; key toggles are CAM address, search key, side_data, counter high bits |
| functional | `≥100%` bins saturated for: `FILL_LEVEL` low/mid/high/depth/wrap, `decision_reg` {0,1,2,3}, `run_state_cmd` × `event`, `pop_engine_state` × `partition_idx`, per-key retire-latency histogram, overwrite × burst-excess | all CROSS merged |

Bins known to saturate slow (from DV_PROF §P086-P110 estimates): per-key retire-latency > 64k txn; partition-fairness > 256k txn; backpressure × multi-key > 1M txn (blocked until ready wire); counter high-bit toggles > 10M txn.

## 5) Bug-Spotting Plays (Why Each Family Is In The Plan)

Each cross family has at least one run whose direct purpose is to re-trip a previously-found bug if it regresses. The anchored direct case for each bug is listed so the composer can re-seed the trap even if the bug moves location.

| bug | pattern the play reproduces | cross runs that anchor it |
|---|---|---|
| BUG-001 lane-local search-key decode | multi-key traffic interleaved across `INTERLEAVING_INDEX` lanes; scoreboard must refuse any drained hit whose decoded `ts[11:4]` is illegal for the active lane | CROSS-031, CROSS-032, CROSS-076 |
| BUG-002 residency on push-write grant (not accept) | overwrite-pressure where accept rate and push-write grant rate diverge due to deassembly FIFO buffering | CROSS-033, CROSS-076, CROSS-078 |
| BUG-003 cache-miss stale-snapshot output branch | high-pressure overwrite followed by search of a key whose side-RAM occupancy just cleared | CROSS-034, CROSS-080, CROSS-081 |
| BUG-004 resequencing-buffer service model | same-ts hotspot burst that exceeds 512 inside `EXPECTED_LATENCY` window; legal overwrite lower bound from `pushes_before_first_pop - 512` | CROSS-035, CROSS-076, CROSS-082 |
| BUG-005 run-control startup/terminate handshake | repeated RUN_PREPARE and TERMINATE within a single long frame; asi_ctrl_ready must follow real flush/drain complete, not a fixed wait | CROSS-036, CROSS-061, CROSS-063 |
| BUG-006 INERR on lane-mismatched bad hits | bad-hit burst with `asi_hit_type1_channel ≠ INTERLEAVING_INDEX` interleaved with lane-matched bad hits | CROSS-037, CROSS-091 |
| BUG-007 48-bit counter truncation through `to_integer()` | sustained run until `push_cnt`, `pop_cnt`, `overwrite_cnt` each cross 2^31; `cam_clean` and `FILL_LEVEL` must stay correct across the crossing | CROSS-038, CROSS-095, CROSS-121, CROSS-125 |
| BUG-008 same-key overwrite erase compared against live input | same-key burst that ends exactly on an overwrite and is immediately followed by an idle cycle before any new same-key beat | CROSS-039, CROSS-078, CROSS-083 |

## 6) Canonical Cross Catalog

### 6.1 Bucket and all-bucket baselines (CROSS-001-006)

These are the reset-per-case-free direct regressions. Any case-boundary cleanup gap is caught here first.

| case_id | ladder | scenario | bug / coverage target |
|---|---|---|---|
| CROSS-001 | `bucket_frame` | `B001-B129` in order, one DUT start | case-boundary cleanup for BASIC CSR and control cases |
| CROSS-002 | `bucket_frame` | `E001-E129` in order | E009-E012 ring wrap composition; E030-E045 subheader framing chain |
| CROSS-003 | `bucket_frame` | `P001-P129` in order | all promoted random cases composed without restart |
| CROSS-004 | `bucket_frame` | `X001-X132` in order | INERR, TERM, FLUSH, RECOVERY lifecycle composed |
| CROSS-005 | `all_buckets_frame` | `BASIC → EDGE → PROF → ERROR` in order | full stack; bucket transitions handled by exactly one FLUSH each |
| CROSS-006 | `all_buckets_frame` | `BASIC → EDGE → PROF → ERROR` with B005, B006, E002, E003 repeated mid-frame | repeated same-key burst IDs stress the deassembly FIFO across bucket boundaries |

### 6.2 Anchored GOOD-ERROR-GOOD hybrids (CROSS-007-010)

Each run pins a direct X-case (the "error window") between two randomized GOOD windows. The anchor proves an ERROR epoch does not leak state forward or backward.

| case_id | ladder | scenario | bug / coverage target |
|---|---|---|---|
| CROSS-007 | `anchored_hybrid` | GOOD(random, 10k) → X116 anchor → GOOD(random, 10k); explicit FLUSH between windows | X116, X129 regression; post-recovery counter and FILL_LEVEL invariants |
| CROSS-008 | `anchored_hybrid` | nightly promoted X117 anchor: GOOD(2048) → ERROR(64) → FLUSH → GOOD(2048) | no pre-error occupancy leaks past FLUSH; `INERR_COUNT/OVERWRITE_COUNT/CACHE_MISS_COUNT` reset to 0 by FLUSH |
| CROSS-009 | `anchored_hybrid` | nightly promoted X118 anchor: GOOD(2048) → TERM → IDLE → RUN_PREPARE → RUN → GOOD(2048) | X118 regression; same-key ordering survives TERM; restart sees a clean CAM post-PREP |
| CROSS-010 | `anchored_hybrid` | overwrite-pressure GOOD(pool=1, λ=1.0, 10k) → X119 anchor → recovery GOOD(random, 10k) | BUG-004 and BUG-008 regression; `OVERWRITE_COUNT` and `INERR_COUNT` are orthogonal across the boundary |

### 6.3 Arbitration and counter-delay anchors (CROSS-011-014)

| case_id | ladder | scenario | bug / coverage target |
|---|---|---|---|
| CROSS-011 | `anchored_hybrid` | two pool=1 high-rate starters with 1-cycle staggered launch (adversarial interleave per P031), 20k | overlap monitor captures arbiter-winner contract; cross with `decision_reg` {0,1,2} |
| CROSS-012 | `anchored_hybrid` | same adversarial interleave as CROSS-011 but with randomized launch gaps in [0, 4] cycles, 50k | arbiter corner explored across every near-coincident phase; `pop_engine_state × push_state` cross |
| CROSS-013 | `anchored_hybrid` | frontdoor CSR reads of PUSH/POP/OVERWRITE/CACHE_MISS/INERR while backdoor counter observer samples internal 48-bit values; stimulus = P107 burst soak, 50k | BUG-007 anchor at small scale; bounded-delay check between backdoor and frontdoor counters |
| CROSS-014 | `anchored_hybrid` | CROSS-013 stimulus re-run with a TERM injected every 10k cyc | counter retention vs clear semantics per X101-X115 checked across TERM boundaries |

### 6.4 Interleaving and random-time composition anchors (CROSS-015-016)

| case_id | ladder | scenario | bug / coverage target |
|---|---|---|---|
| CROSS-015 | `anchored_hybrid` | nightly curated all-bucket mix: B005/B006, E002, P001-style random push-pop, X117-style error+flush recovery, plus overwrite-pressure windows, all separated by random idle gaps | case-boundary ownership under repeated invocation; direct and random patterns from all four buckets coexist in one continuous frame |
| CROSS-016 | `anchored_hybrid` | same as CROSS-015 but inter-case gap drawn uniformly from [0, 200] cycles | idle-gap insensitivity; DUT is idempotent across any idle-length in that range |

### 6.5 Seed-swept random promotions (CROSS-017-030)

Each row replays one promoted random case with an orthogonal LCG seed. Seeds `0x1…0xE` are reserved by DV_PROF §P086-P110. These runs share that seed line so merged UCDBs align.

| case_id | ladder | anchor case | seed | scenario | bug / coverage target |
|---|---|---|---|---|---|
| CROSS-017 | `seed_sweep` | P001 | `0x1` | random same-key traffic, 200k txn | per-key queue drains cleanly; stmt / branch ≥90% on single-key path |
| CROSS-018 | `seed_sweep` | P001 | `0x2` | same, orthogonal seed | seed-axis contribution to toggle coverage |
| CROSS-019 | `seed_sweep` | P002 | `0x3` | random multi-key traffic, 200k txn | BUG-001 regression; multi-lane decode coverage |
| CROSS-020 | `seed_sweep` | P002 | `0x4` | same, orthogonal seed | multi-key cross-seed UCDB merge |
| CROSS-021 | `seed_sweep` | P003 | `0x5` | random deep same-key throughput, 200k txn | P003 currently fails; this is the reproducible-failure seed gate |
| CROSS-022 | `seed_sweep` | P003 | `0x6` | same, orthogonal seed | confirms P003 fix stable across seeds |
| CROSS-023 | `seed_sweep` | P031 | `0x14` | adversarial arbiter interleave, 200k txn | overlap monitor histogram populated; covers BUG-001 + arbiter |
| CROSS-024 | `seed_sweep` | P034 | `0x15` | silent-key search 10%, 200k txn | zero-hit silent-key subheader saturation without `CACHE_MISS_COUNT` drift; BUG-031 anchor |
| CROSS-025 | `seed_sweep` | P091 | `0x6` | λ=0.9 with EXPECTED_LATENCY swept every 50k, 500k txn | `EXPECTED_LATENCY × FILL_LEVEL` cross bin saturated |
| CROSS-026 | `seed_sweep` | P092 | `0x7` | TERM every 100k cyc, 500k txn | BUG-005 anchor across a long frame |
| CROSS-027 | `seed_sweep` | P093 | `0x8` | FLUSH every 100k cyc, 500k txn | flush-state coverage × counter-clear semantics |
| CROSS-028 | `seed_sweep` | P094 | `0x9` | balanced 4 partitions, 500k txn | partition-fairness bin saturation |
| CROSS-029 | `seed_sweep` | P095 | `0xA` | skewed partitions 70/10/10/10, 500k txn | minority-partition retire-latency bin |
| CROSS-030 | `seed_sweep` | P096 | `0xB` | 1% error-rate beat injection, 500k txn | `INERR_COUNT` monotone; scoreboard never counts errored hits as references |

### 6.6 Bug-seeded long soaks (CROSS-031-045)

Each row traps one BUG-00x. Surrounding traffic is randomized so the trap is hit many times per run, not just once.

| case_id | ladder | bug anchor | scenario | bug / coverage target |
|---|---|---|---|---|
| CROSS-031 | `anchored_hybrid` | BUG-001 | pool=4 multi-lane traffic, random lane / key pairing, 500k txn | every drained hit's decoded `ts[11:4]` is legal for the active lane; one illegal decode fails the run |
| CROSS-032 | `anchored_hybrid` | BUG-001 | pool=16 multi-lane with reuse-rate 80%, 500k txn | same invariant at high key diversity |
| CROSS-033 | `anchored_hybrid` | BUG-002 | P111 shape (hotspot burst) repeated every 10k, 500k txn | residency accounted on push-write grants; scoreboard rejects any "unexpected drop" attributed to accept-order |
| CROSS-034 | `anchored_hybrid` | BUG-003 | P116-style overwrite pressure → search of just-cleared key, repeated 50 times | cache-miss hit beats tagged; never classified as "impossible output" |
| CROSS-035 | `anchored_hybrid` | BUG-004 | P112-P113 hotspot bursts with burst excess ∈ [32, 2048] over depth, 500k txn | legal overwrite lower bound matches observed burst shape; no undrained residual beyond the bound |
| CROSS-036 | `anchored_hybrid` | BUG-005 | pool=4 random with RUN_PREPARE + TERMINATE re-issued every 5k cyc, 500k txn | handshake follows real flush/drain complete; no fixed-wait regression |
| CROSS-037 | `anchored_hybrid` | BUG-006 | X013 + X007-X010 patterns composed with random lane-mismatch ratio ∈ [0, 100]%, 200k txn | `INERR_COUNT` correctly reflects raw bad-hit pulse regardless of lane filter |
| CROSS-038 | `checkpoint_soak` | BUG-007 | pool=4 λ=0.9, 10M txn to cross 2^31 in `push_cnt` | 48-bit counters remain monotone in the backdoor view; `cam_clean` and `FILL_LEVEL` still correct at the crossing |
| CROSS-039 | `anchored_hybrid` | BUG-008 | pool=1 same-key burst of length N∈[1, 2048], N random per trial, 5000 trials | final overwrite of each burst is followed by idle; latched-key erase compare holds (live `in_hit_sk` path must not re-erase the just-written resident) |
| CROSS-040 | `anchored_hybrid` | BUG-008 variant | pool=2 alternating same-key bursts with 1-cycle gap between ownership switch | latched-key vs live-input erase compare tested at burst-ownership handover |
| CROSS-041 | `anchored_hybrid` | BUG-001 + BUG-006 | multi-lane traffic with per-lane random error rate, 500k txn | combined lane-decode + error-filter coverage |
| CROSS-042 | `anchored_hybrid` | BUG-002 + BUG-004 | deep overwrite pressure with backdoor push-write grant observer running, 500k txn | overwrite counts reconcile to grant pulses exactly |
| CROSS-043 | `anchored_hybrid` | BUG-003 + BUG-008 | post-burst search after a same-key overwrite run, repeated 100 times | cache-miss path and same-key-erase path do not couple |
| CROSS-044 | `anchored_hybrid` | BUG-005 + BUG-007 | repeated TERM/FLUSH under sustained 2^30-cycle soak | handshake and 48-bit counter interact cleanly across transition cycles |
| CROSS-045 | `anchored_hybrid` | all bugs | randomized composer picking any BUG-00x anchor every 20k cyc, 500k txn | integration of every bug family into one run; any regression in any bug family fails this run |

### 6.7 Partition and encoder variant soaks (CROSS-046-060)

One run per supported `N_PARTITIONS × ENCODER_PIPE_STAGES` point, plus promotion soaks of the partition cases.

| case_id | ladder | scenario | bug / coverage target |
|---|---|---|---|
| CROSS-046 | `seed_sweep` | P046 → partition 0 saturated, 200k txn | single-partition saturation; other partitions idle |
| CROSS-047 | `seed_sweep` | P047 → exact-partition prefill followed by a target epoch that traverses the `p0->p1` handoff window, 200k txn | partition-handoff contract after a fully consumed first partition; both live partitions observed in the target issue mask |
| CROSS-048 | `seed_sweep` | partition 2 saturated, 200k txn | same for partition 2 |
| CROSS-049 | `seed_sweep` | partition 3 saturated, 200k txn | same for partition 3 |
| CROSS-050 | `seed_sweep` | P050 balanced 4 partitions, 500k txn | per-partition drain throughput within ±5% |
| CROSS-051 | `seed_sweep` | P051 70/10/10/10 skew, 500k txn | heavy-partition fairness |
| CROSS-052 | `seed_sweep` | P052 40/40/10/10 two-way contention, 500k txn | arbiter fairness metric |
| CROSS-053 | `seed_sweep` | P053 99/1/0/0 adversarial skew, 500k txn | minority-partition eventually drained |
| CROSS-054 | `seed_sweep` | P054 `ENCODER_PIPE_STAGES=1`, 200k txn | encoder pipe=1 contract |
| CROSS-055 | `seed_sweep` | P055 `ENCODER_PIPE_STAGES=2`, 200k txn | encoder pipe=2 contract |
| CROSS-056 | `seed_sweep` | P056 `ENCODER_PIPE_STAGES=3`, 200k txn | encoder pipe=3 contract |
| CROSS-057 | `seed_sweep` | P057 `ENCODER_PIPE_STAGES=4` (default build), 200k txn | encoder pipe=4 contract |
| CROSS-058 | `seed_sweep` | P058 partition-empty drain other-full, 200k txn | partition-independence coverage |
| CROSS-059 | `seed_sweep` | P061 cross-partition round-robin, 200k txn | cross-partition near-coincident bin in overlap monitor |
| CROSS-060 | `seed_sweep` | P065 `partitioned` vs `pipe` encoder equivalence, 200k txn | both builds produce the same scoreboard outcome |

### 6.8 Run-control injection soaks (CROSS-061-075)

Every run here includes random TERM and/or FLUSH. The state observer must record every transition and the scoreboard must reconcile residuals per transition.

| case_id | ladder | scenario | bug / coverage target |
|---|---|---|---|
| CROSS-061 | `seed_sweep` | P066 TERM at random time, 200k txn | BUG-005 handshake regression guard |
| CROSS-062 | `seed_sweep` | P067 FLUSH at random time, 200k txn | post-FLUSH `FILL_LEVEL==0` |
| CROSS-063 | `seed_sweep` | P068 TERM+RESTART within a run, 500k txn | counter reset semantics after restart |
| CROSS-064 | `seed_sweep` | P069 FLUSH every 5k cyc, 500k txn | leak-across-windows guard |
| CROSS-065 | `seed_sweep` | P070 TERM at 70% fill, pool=8, 500k txn | high-fill terminate |
| CROSS-066 | `seed_sweep` | P071 FLUSH at 90% fill, pool=8, 500k txn | near-full flush |
| CROSS-067 | `seed_sweep` | P072 TERM every 2k cyc, 500k txn | high-rate TERM does not deadlock |
| CROSS-068 | `seed_sweep` | P073 FLUSH every 2k cyc, 500k txn | high-rate FLUSH |
| CROSS-069 | `seed_sweep` | P074 TERM+FLUSH random interleave, 1M txn | per-event residual classification |
| CROSS-070 | `seed_sweep` | P075 TERM mid-subheader, 500k txn | subheader framing never split |
| CROSS-071 | `seed_sweep` | P076 FLUSH mid-subheader, 500k txn | no stale EOP without SOP |
| CROSS-072 | `seed_sweep` | P077 TERM with 32 in-flight, 500k txn | in-flight accounting small scale |
| CROSS-073 | `seed_sweep` | P078 TERM with 256 in-flight, 500k txn | in-flight accounting at depth |
| CROSS-074 | `seed_sweep` | P079+P080 FLUSH with 32/256 in-flight, 500k txn | flush aborts with counted residuals |
| CROSS-075 | `seed_sweep` | P085 TERM+FLUSH while `EXPECTED_LATENCY` reprogrammed, 500k txn | CSR write vs run-control no race |

### 6.9 Overwrite-pressure long soaks (CROSS-076-090)

These runs drive BUG-004 and BUG-008 the hardest. Each has an explicit overwrite lower-bound check and a push-write-grant observer anchor.

| case_id | ladder | scenario | bug / coverage target |
|---|---|---|---|
| CROSS-076 | `seed_sweep` | nightly hotspot overwrite soak derived from P111, 131072 txn same-ts pressure | `pushes_before_first_pop - 512` bound check plus deeper overwrite/toggle accumulation than the smoke pressure runs |
| CROSS-077 | `seed_sweep` | P112 hotspot medium excess, 500k txn | same, larger excess |
| CROSS-078 | `seed_sweep` | P113 hotspot sustained over-injection, 500k txn | BUG-008 regression trap at aggressive burst |
| CROSS-079 | `seed_sweep` | P116 pool=4 120% push rate, 500k txn | per-key overwrite fairness |
| CROSS-080 | `seed_sweep` | P117 pool=4 skewed 120%, 500k txn | overwrite concentrates on heaviest key |
| CROSS-081 | `seed_sweep` | P118 adversarial alternating key, 500k txn | overwrite workload rotates across keys |
| CROSS-082 | `seed_sweep` | P119 bursty hotspot 2x/0.5x, 500k txn | overwrite only in burst window; recovery in idle window |
| CROSS-083 | `seed_sweep` | P120 slow creep 101%, 2M txn | `OVERWRITE_COUNT ≈ 1%` of push count; long-horizon invariant |
| CROSS-084 | `seed_sweep` | P121 pool=4 130% + FLUSH every 20k, 1M txn | flush clears fill between overwrite windows |
| CROSS-085 | `seed_sweep` | P122 pool=4 130% + TERM every 20k, 1M txn | terminate aborts counted separately from legal overwrite |
| CROSS-086 | `seed_sweep` | P123 hotspot + `EXPECTED_LATENCY=max`, 500k txn | larger pre-service window → more overwrite |
| CROSS-087 | `seed_sweep` | P124 hotspot + `EXPECTED_LATENCY=min`, 500k txn | smaller pre-service window → less overwrite |
| CROSS-088 | `seed_sweep` | P125 pool=4 120% with checkpoint UCDB, 1M txn | overwrite-axis growth curve |
| CROSS-089 | `seed_sweep` | P128 pool=1 saturation 200%, 1M txn | worst-case overwrite (~50% of push count) |
| CROSS-090 | `seed_sweep` | P129 directed replay of `test_random_throughput` failure shape, 1M txn | regression lock-in for P003 failure fix |

### 6.10 Backdoor counter-observer soaks (CROSS-091-105)

One run per X101-X115 direct case, promoted to a long-soak with the backdoor observer continuously sampling. These are the BUG-007 guardrails.

| case_id | ladder | anchor | scenario | bug / coverage target |
|---|---|---|---|---|
| CROSS-091 | `seed_sweep` | X101-X102 promoted to a 131072-txn sustained bad-hit burst | `INERR` backdoor vs frontdoor within 4 clk plus deeper error-counter toggle accumulation |
| CROSS-092 | `seed_sweep` | X103 | bad hit concurrent with good push at 10% rate, 500k txn | both counters advance on same cycle |
| CROSS-093 | `seed_sweep` | X104 | push near TERM, TERM rate 1/5k, 500k txn | final push captured before TERMINATING freezes |
| CROSS-094 | `seed_sweep` | X105 | pop near TERM, TERM rate 1/5k, 500k txn | final pop 1 clk before asi_ctrl_ready |
| CROSS-095 | `seed_sweep` | X106 | overwrite near TERM under pressure, 500k txn | overwrite counter captures last-cycle collision |
| CROSS-096 | `seed_sweep` | X107 | cache miss near TERM, 500k txn | `CACHE_MISS_COUNT` captures last-cycle pulse |
| CROSS-097 | `seed_sweep` | X108 | INERR near FLUSH, 500k txn | backdoor shows `inerr_cnt` +=1 then =0 in FLUSHING window |
| CROSS-098 | `seed_sweep` | X109 | overwrite near FLUSH, 500k txn | same pattern for `overwrite_cnt` |
| CROSS-099 | `seed_sweep` | X110 | cache miss near FLUSH, 500k txn | same pattern for `cache_miss_cnt` |
| CROSS-100 | `seed_sweep` | X111 | push near FLUSH, 500k txn | `push_cnt` retained through FLUSHING (not cleared by decision_reg=3) |
| CROSS-101 | `seed_sweep` | X112 | pop near FLUSH, 500k txn | `pop_cnt` retained |
| CROSS-102 | `seed_sweep` | X114 | CSR read of PUSH_COUNT at push cycle, 500k txn | read ∈ {N, N+1}; backdoor shows exact increment cycle |
| CROSS-103 | `checkpoint_soak` | X115 | soak past 2^32 events in every counter, 10M txn | frontdoor truncation to 32 bits; backdoor still monotone at 48 bits |
| CROSS-104 | `seed_sweep` | X113 | INERR coincident with soft-reset write, random, 500k txn | current retention vs future clear semantics documented |
| CROSS-105 | `seed_sweep` | X102 | 16 bad hits back-to-back in every 1k cyc, 500k txn | no counter updates lost at max burst rate |

### 6.11 GOOD-ERROR-RECOVERY regression (CROSS-106-120)

Promotes X116-X132 invariants to long-soak regressions. Each pins one recovery invariant as the anchor and surrounds it with randomized GOOD traffic.

| case_id | ladder | anchor | scenario | bug / coverage target |
|---|---|---|---|---|
| CROSS-106 | `anchored_hybrid` | X116 | 32 GOOD → 8 BAD → 32 GOOD, repeated 1000 trials | all 64 GOOD drained per trial; 8 errors isolated to error window |
| CROSS-107 | `anchored_hybrid` | X117 | GOOD → ERROR → FLUSH → GOOD, 500 trials | post-FLUSH counters cleared; second burst sees fresh state |
| CROSS-108 | `anchored_hybrid` | X118 | GOOD → TERM mid-traffic → PREP → RUN → GOOD, 500 trials | endofrun_seen cleared; second burst fully drained |
| CROSS-109 | `anchored_hybrid` | X119 | GOOD → overwrite-heavy ERROR → TERM → recovery, 500 trials | overwrite-loss accounted; recovery unaffected |
| CROSS-110 | `anchored_hybrid` | X120 | GOOD → cache-miss heavy window → recovery, 500 trials | cache-miss beats carry correct error bit; recovery clean |
| CROSS-111 | `anchored_hybrid` | X121 | recovery latency invariant, 500 trials | first N pushes after recovery match cold-start latency ±2 clk |
| CROSS-112 | `anchored_hybrid` | X122 | recovery CSR constants, 500 trials | `UID` / `META` unchanged across recovery |
| CROSS-113 | `anchored_hybrid` | X123 | recovery run-state sequence, 500 trials | observer sees legal IDLE → PREP → SYNC → RUNNING → TERM → IDLE each trial |
| CROSS-114 | `anchored_hybrid` | X124 | underflow guard, 500 trials | `FILL_LEVEL` never reads all-ones |
| CROSS-115 | `anchored_hybrid` | X125 | `aso_filllevel_data` streaming invariant, 500 trials | sideband equals CSR fill-level(15:0) every valid beat |
| CROSS-116 | `anchored_hybrid` | X126 | post-recovery empty TERM, 500 trials | trivial ack within 2 clk |
| CROSS-117 | `anchored_hybrid` | X127 | endofrun_seen cleared across PREP, 500 trials | flag never inherited into second run |
| CROSS-118 | `anchored_hybrid` | X128 | `gts_end_of_run` snapshot per TERM, 500 trials | snapshot changes every TERM |
| CROSS-119 | `anchored_hybrid` | X129 | full mixed ERROR (BAD + overwrite + cache-miss) → FLUSH → GOOD, 200 trials | signoff-grade recovery invariant |
| CROSS-120 | `anchored_hybrid` | X129 variant | same as CROSS-119 with random phase offset per trial | phase-insensitivity |

### 6.12 Checkpoint UCDB long soaks (CROSS-121-125)

These are the growth-curve runs. Each saves checkpoint UCDBs at log-spaced txn counts under `REPORT/txn_growth/`. Pick set that unions the cross-axis bins fastest.

| case_id | ladder | scenario | bug / coverage target |
|---|---|---|---|
| CROSS-121 | `checkpoint_soak` | pool=4 uniform λ=0.7, seed=`0x1`, 10M txn, checkpoints at 1, 4, 16, 64, 256, 1k, 4k, 16k, 64k, 256k, 1M, 4M, 10M | primary growth curve; FSM + toggle closure driven here |
| CROSS-122 | `checkpoint_soak` | pool=8 Zipf s=1.2, seed=`0x3`, 10M txn, same checkpoints | heavy-key bins saturate early; tail-key bins saturate late |
| CROSS-123 | `checkpoint_soak` | pool=4 λ=0.9, EXPECTED_LATENCY swept every 500k, seed=`0x6`, 10M txn | latency × fill-level cross saturated |
| CROSS-124 | `checkpoint_soak` | pool=4 Poisson λ=0.9 + random TERM/FLUSH every 500k, seed=`0x7`, 10M txn | run-control × counter coverage saturated |
| CROSS-125 | `checkpoint_soak` | pool=4 120% push rate, seed=`0x20`, 10M txn, overwrite-focused | BUG-004 / BUG-008 long-horizon guard; `overwrite_cnt` crosses 2^31 |

### 6.13 Closure signoff (CROSS-126-129)

Final merged-regression runs. Every axis is randomized; these are the "one-shot" runs that must all pass for the IP to sign off.

| case_id | ladder | scenario | bug / coverage target |
|---|---|---|---|
| CROSS-126 | `checkpoint_soak` | `all_buckets_frame` + randomized inter-case gaps + random TERM/FLUSH between buckets, 5M txn, seed=`0x1000` | full catalog composed under randomized timing |
| CROSS-127 | `checkpoint_soak` | same stimulus as CROSS-126 with seed=`0x2000` (orthogonal) | seed-independence of closure |
| CROSS-128 | `checkpoint_soak` | merge-reference of CROSS-017..030 + CROSS-061..090 + CROSS-121..125, 10M txn replay | merged UCDB must clear every target in §4 |
| CROSS-129 | `checkpoint_soak` | BUG-seeded composer using the full anchor list of §5, 10M txn, random trap selection per 50k cyc | one run that must trap every historical bug if any regresses |

## 7) Coverage Closure Rules

- Every cross run emits an isolated UCDB. `bucket_frame` / `all_buckets_frame` / `checkpoint_soak` runs emit merged UCDBs as well.
- `checkpoint_soak` runs emit one UCDB per checkpoint under `REPORT/txn_growth/<case_id>/ckpt_<n>.ucdb`. The checkpoint schedule is log-spaced and fixed per run (see catalog).
- The §4 closure targets are evaluated on the **merged UCDB** of the full isolated B/E/P/X catalog + all CROSS runs. Any single-run coverage is not closure.
- A CROSS run does not count toward closure until:
  - its scoreboard reports zero unexpected drop / duplicate / leak
  - every run-control observer transition is accounted
  - every counter invariant in the anchor target column is verified
  - its isolated UCDB is present and merged into the release UCDB

## 8) Bug-Regression Gate

The following runs must be green for a release-quality RTL revision:

- every bug-seeded soak in §6.6 (CROSS-031-045)
- the full GOOD-ERROR-RECOVERY regression in §6.11 (CROSS-106-120)
- the three BUG-007 long-horizon runs (CROSS-038, CROSS-103, CROSS-125)
- the BUG-008 ownership-handover run (CROSS-039, CROSS-040)
- the closure regression CROSS-128 and CROSS-129

If any of those fail, the RTL is not releasable regardless of other coverage totals.

## 9) Signoff Gates

The IP is cross-signoff-complete when all of the following are true:

- every CROSS-001..129 has an evidence page under `REPORT/cross/` with a non-zero txn count
- the merged UCDB clears every §4 target
- the §8 bug-regression gate is green on the current RTL revision
- the DV_REPORT.md dashboard (regenerated by `~/.codex/skills/dv-workflow/scripts/dv_report_gen.py`) shows zero `unimplemented_cases` across the cross table
- `DV_HARNESS.md §8` gaps are either closed or explicitly retired with written rationale

## 10) Regenerate

After editing this file, run the generator so `DV_REPORT.json` and `DV_REPORT.md` pick up the new scenarios:

```
python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb .
```

The generator parses `DV_CROSS.md` for the catalog table. Keep the `CROSS-###` IDs contiguous from 001 to 129; do not reuse numbers across rewrites.

# DV Harness: ring_buffer_cam

**Companion to:** [DV_PLAN.md](DV_PLAN.md)

## 1. Harness Goals

The harness must make these properties observable:

- every accepted ingress hit is either drained or explicitly justified as overwritten
- subheader search-key / hit-count framing is correct
- CSR identity, control, latency, fill-level, and activity counters match RTL
- run-control transitions drive the DUT through the intended flush / sync / running / terminate states
- partitioned drain behavior remains correct under same-key, multi-key, and pressure cases
- long mixed runs can detect GOOD-ERROR-GOOD patterns, near-same-time arbitration, and delayed counter increments

## 2. Current Bench Inventory

The live harness under `tb/uvm/` currently contains:

- `tb_top.sv`: clock/reset, mixed-language DUT instantiation, default-ready egress sink with testcase-controlled ready override through `set_sink_ready()`
- `hit_driver.sv`, `ctrl_driver.sv`, `csr_driver.sv`: active drivers
- `hit_monitor.sv`, `out_monitor.sv`: passive ingress / egress monitors
- `debug_monitor.sv`: DUT-side observer for push/pop/overwrite/cache-miss counters and push-write grants
- `scoreboard.sv`: reference queue keyed by search key, now with end-of-test undrained-hit failure and push-write based residency
- `coverage.sv`: output-oriented covergroups only
- `base_test.sv`, `sequences.sv`: current promoted cases and sequence classes

## 3. Current Live Findings

The current 2026-04-21 evidence set establishes:

- the real CSR map is:
  - `0 = UID`
  - `1 = META`
  - `2 = CTRL`
  - `3 = EXPECTED_LATENCY`
  - `4 = FILL_LEVEL`
  - `5 = INERR_COUNT`
  - `6 = PUSH_COUNT`
  - `7 = POP_COUNT`
  - `8 = OVERWRITE_COUNT`
  - `9 = CACHE_MISS_COUNT`
- the previous config tests were offset by two words and therefore invalid against live RTL
- after correcting the map, the basic CSR cases pass
- the startup/terminate spine now uses the DUT's real ready handshake instead of fixed sleeps:
  - `RUN_PREPARE` is re-issued so the second command waits on flush completion
  - `TERMINATING` is re-issued after the end-of-run marker so the driver waits on the real drain-complete path
- several directed and error smoke cases now drain correctly, including `X007`, `X008`, and `X013`
- the new overlap guards now explicitly prove the frozen pop snapshot cannot be mutated by `push_write` once the engine has entered `LOAD`, `COUNT`, or `DRAIN`:
  - `B130` guards `DRAIN`
  - `B131` guards `LOAD/COUNT`
- the SEARCH-window closure now has an explicit early-window guard as well:
  - `B133` proves a cross-key `push_write_grant` cannot slip into the unstable SEARCH window before the snapshot is frozen
  - `B056` still anchors the SEARCH wait-count contract at `0..5` after the settled-tail retiming
- the final SEARCH-tail policy now preserves both throughput and snapshot integrity:
  - `P031` still passes the adversarial two-key alternation with `push=20000`, `pop=20000`, `overwrite=0`
  - `P125` and `P126` still close their reduced checkpoint overwrite soaks while the frozen-snapshot write-pointer guard is active
- the terminate path is now closed around the real DUT contract:
  - `B132` proves ingress `ready` clamps after lane-local end-of-run
  - `P066` proves already-buffered deassembly payload still drains to completion in `TERMINATING`
- the profile tranche `P007-P024` now runs through the isolated case engine with real log and UCDB evidence:
  - same-key and multi-key integrity windows had to be calibrated against the DUT's real service rate rather than the older placeholder fill/txn prose
  - the live no-overwrite profiles are therefore defined by measured ingress gaps and sink-ready patterns, not by the superseded nominal `10k/20k/30k` traffic counts in the early PROF draft
- sink backpressure is now a real harness capability rather than a paper plan item:
  - `P018-P020` drive 75%, 50%, and PRNG-shaped ready gating through `set_sink_ready()`
  - those cases now provide real no-overwrite backpressure evidence instead of a blocked placeholder
- overwrite-pressure closure now follows the resequencing-buffer service model:
  - `rbCAM` is a bounded per-sector service element, not an unbounded queue
  - with `RING_BUFFER_N_ENTRY=512` and default `EXPECTED_LATENCY=2000`, a burst that injects more than 512 hits of the same timestamp before the first pop window opens must overwrite legally
  - the harness therefore has to distinguish legal overwrite from unexpected loss by measuring the burst shape, not just by checking for nonzero residuals
- the live RTL contract for bad-hit counting is now explicit:
  - `INERR_COUNT` can increment from the raw bad-hit pulse even when the lane filter drops the beat before enqueue
  - testcase expectations must therefore separate `INERR_COUNT` from accepted/pushed traffic

## 4. Required Scoreboard Contract

The scoreboard is the core signoff checker. It must not infer correctness from "no explicit mismatch" alone.

Required behavior:

- record every accepted ingress hit with its search key, side data, and acceptance time
- consume subheaders and bind each drained hit to the active search key rather than searching globally without context
- fail the case if accepted hits remain undrained at end of test unless the case explicitly models legal overwrite loss
- distinguish these outcomes:
  - expected overwrite
  - unexpected drop
  - unexpected duplicate
  - cache miss on empty search
  - terminate / flush aborted drain
- for pressure cases, the scoreboard plus debug monitor must also explain *why* overwrite was legal:
  - maximum live occupancy reached the 512-entry sector depth
  - overwrite started before or at the moment service first caught up
  - `pushes_before_first_pop - 512` provides a lower bound on legal overwrite count for a same-ts hotspot run

Current state:

- the scoreboard now fails undrained references
- actual residency is inserted on DUT push-write grants, not on front-end accept
- cache-miss hit beats are tagged separately instead of being misclassified as impossible outputs
- overwrite-aware expectations are still incomplete for the long overwrite and random throughput cases
- same-ts hotspot pressure cases now use observed service-envelope checks instead of fixed overwrite counts
- the calibrated profile tranche closes the directed integrity windows for `P007-P024`, but the broader random multi-key / long-soak PROF space still needs a stronger per-key retire model before it can anchor signoff

## 5. Monitor And Observer Requirements

The current monitors are not enough for full signoff. The next promoted upgrades are:

- a run-state observer that timestamps every control transition at the DUT boundary
- a subheader-aware egress observer that tracks search-key epochs explicitly
- a counter observer that can check CSR activity counters with bounded delay
- a backdoor counter observer for the long mixed ERROR/CROSS runs, so delayed increments can be checked without perturbing frontdoor timing
- an overlap monitor that detects near-same-time transaction launches where one stream is accepted before another because of DUT arbitration
- a pressure observer that records:
  - max live occupancy
  - first push / first overwrite / first pop cycles
  - pushes observed before first pop service
  - hotspot occupancy per search key

The overlap monitor is required for the `cross_overlap_priority` run in [DV_CROSS.md](DV_CROSS.md). The issue is not literal packet overlap on the same sink beat; it is near-coincident stimulus that becomes back-to-back internally and exercises the arbiter corner.

## 6. Continuous-Frame Requirements

The harness must support:

- isolated reset-per-case runs
- bucket-local no-restart runs
- all-buckets no-restart runs
- curated interleaved cross runs where selected cases repeat and are randomized in time relative to each other

The curated cross runs are allowed to:

- repeat a selected case multiple times
- mix short and long cases in one run
- randomize the time between case injections
- interleave GOOD and ERROR patterns in one run

The harness does not close this today. It needs a dedicated run composer plus scoreboard state that survives case boundaries without losing ownership of pending hits.

## 7. Coverage And Artifact Requirements

The harness now saves one isolated UCDB per rerun case under `tb/uvm/cov_after/`, and the report generator publishes matching isolated case logs under `tb/uvm/logs/`. The remaining closure gaps are:

- checkpoint UCDBs for promoted random cases under `tb/uvm/cov_after/txn_growth/`
- merged UCDBs for `bucket_frame` and `all_buckets_frame`
- explicit continuous-frame coverage evidence for the future bucket / cross signoff runs
- broader code-coverage growth beyond the current isolated case set

Until those continuous-frame artifacts exist, the report generator keeps the bucket/cross growth pages partially populated and marks the missing checkpoint data explicitly.

## 8. Known Gaps To Close Next

- `test_sequential_keys`: only part of the multi-key workload drains before end-of-test
- `test_overwrite_stress` / `E003`: now reframed as a same-ts hotspot service-envelope test, but still needs a clean evidence rerun after the harness pressure observer changes
- `test_random_multi_key` and `test_random_throughput`: same issue under randomized traffic
- only `BASIC` `bucket_frame` and curated crosses `CROSS-007` / `CROSS-010` exist today; the rest of `bucket_frame`, `all_buckets_frame`, and the overlap/counter-delay run families still need implementation
- the next PROF closure tranche is beyond `P024`: fairness/skew/random-interleave cases need stronger per-key retire observability before they can be promoted without weakening the contract
- no backdoor counter checker exists for delayed counter increments after injected errors

These gaps are now first-class signoff blockers, not footnotes.

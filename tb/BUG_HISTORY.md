# BUG_HISTORY.md — ring_buffer_cam DV bug ledger

## 2026-04-16

### BUG-001: Search-key decode and interleaving model in UVM harness were wrong
- First seen in: `test_sequential_keys`, `test_random_multi_key`, `test_random_throughput`
- Symptom:
  - multi-key and random cases reported undrained residents and inconsistent lane acceptance
  - case coverage and grouped-flow evidence were not trustworthy because the harness was driving illegal lane/key combinations
- Root cause:
  - `hit_seq_item.search_key()` and the interleaving constraint were not aligned to the DUT contract `ts[11:4]`
  - sequence generation used global keys instead of lane-local legal keys for a single `INTERLEAVING_INDEX`
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - restored passing behavior for the promoted multi-key/random smoke cases and enabled the new case engine to reuse those sequences safely
- Commit: pending

### BUG-002: Scoreboard modeled accepted ingress order, not actual push-write grant order
- First seen in: `test_overwrite_stress`, `E003`
- Symptom:
  - overwrite-pressure cases reported impossible “unexpected drained hit” errors or false overwrite attribution
  - resident accounting drifted whenever push and pop arbitration interleaved
- Root cause:
  - the DUT increments `write_pointer` only on actual `decision_reg=0` push-write grants
  - the old scoreboard inserted residents on input acceptance, which is earlier than the real memory write when the deassembly FIFO buffers traffic
- Fix status: fixed in working tree via the new internal `dut_debug_if` tap and `debug_monitor`, not yet committed
- Runtime / coverage context:
  - reduced false overwrite drift under pressure and made scoreboard accounting traceable to actual CAM/RAM writes instead of front-end acceptance
- Commit: pending

### BUG-003: Stale-snapshot / cache-miss output branch was unmodeled in the harness
- First seen in: `test_overwrite_stress`, `E003`
- Symptom:
  - high-pressure runs emitted late hit beats that the scoreboard classified as impossible
  - the failures clustered in a narrow key band after the ring crossed sustained overwrite pressure
- Root cause:
  - the DUT can legally emit a hit-body beat from a stale search snapshot while `side_ram_dout(high)=0`, pulsing `pop_cache_miss_pulse`
  - the old harness had only two output dispositions: “matched resident” and “unexpected”
- Fix status: fixed in working tree by tagging cache-miss hit beats in the monitor and counting them separately in the scoreboard, not yet committed
- Runtime / coverage context:
  - this is a harness-modeling issue unless the product contract is changed to suppress stale hit-body forwarding in RTL
- Commit: pending

### BUG-004: Pressure-case expectation ignored the resequencing-buffer service model
- First seen in: `E003` planning review and user clarification on 2026-04-16
- Symptom:
  - pressure cases implicitly treated any surviving residents after a fixed wait as a bug
  - this confuses legal overflow under bursty arrival curves with illegal loss under admissible traffic
- Root cause:
  - the bench lacked a network-calculus / resequencing-buffer interpretation of `rbCAM` as a bounded service element
  - specifically, a 512-entry per-sector CAM can legally overflow if more than 512 hits with the same timestamp are injected inside the `EXPECTED_LATENCY` service window
- Fix status: in progress
- Runtime / coverage context:
  - `E003` and the profile-pressure family are being reframed around burst pattern, admissible service, legal overwrite loss, and terminate-and-drain accounting rather than a fixed generic drain timeout
- Commit: pending

### BUG-005: Run-control startup/terminate sequencing used fixed waits instead of the DUT handshake
- First seen in: `E003`, `P111`, general `test_case_engine` runtime review on 2026-04-16
- Symptom:
  - every isolated case burned 150k cycles in `RUN_PREPARE` even when the DUT had already acknowledged the flush
  - long cases hit the global watchdog or produced stale counter reads because the bench sampled before the DUT-side control/state machinery had converged
- Root cause:
  - `configure_and_start()` sent `RUN_PREPARE` once and then slept for a hard-coded delay
  - `terminate_and_drain()` relied on scoreboard waits alone instead of the DUT's own TERMINATING ready/`terminating_drain_done` handshake
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - startup now re-issues `RUN_PREPARE` so the second command waits on the actual flush-complete handshake
  - terminate now re-issues `TERMINATING` after the end-of-run marker so the driver waits on the DUT's real drain-complete path
  - this removed the artificial 1.2 ms startup tax from every case and made the added `X007`/`X008`/`X013` error cases reproducible
- Commit: pending

### BUG-006: Error-counter assumptions were too narrow for lane-mismatched bad hits
- First seen in: `X013` on 2026-04-16
- Symptom:
  - the new lane-mismatch bad-hit case initially expected `INERR_COUNT` to stay at zero because the hit never entered the selected lane/CAM path
- Root cause:
  - the plan assumption was wrong for this RTL revision
  - `proc_csr_rw` increments `INERR_COUNT` from the raw bad-hit pulse when `filter_inerr=1`, independent of whether the lane filter later drops the beat before enqueue
- Fix status: fixed in working tree by aligning the testcase to the live RTL contract, not yet committed
- Runtime / coverage context:
  - `X013` now checks the correct split:
    - `INERR_COUNT` increments
    - `PUSH_COUNT` stays at zero
    - scoreboard accepts zero hits
- Commit: pending

### BUG-007: 48-bit counter cleanup logic truncated through `to_integer()`
- First seen in: long soak / signoff review for super-long counter and toggle runs on 2026-04-16
- Symptom:
  - `cam_clean` and related drain qualification could become wrong once the 48-bit push/pop/overwrite counters exceeded the signed 32-bit range
  - this silently undermines extensive-effort signoff runs, especially the counter-toggle and long-drain families
- Root cause:
  - `proc_fill_level_meter` compared 48-bit counters by converting them through `to_integer()`
  - VHDL integer is 32-bit signed in this toolchain, so the comparison is not width-safe for long runs
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - `cam_clean` now uses width-safe zero compares against `to_unsigned(0, counter'length)`
  - this is a control/counter correctness fix, not a datapath-beat fix
- Commit: pending

### BUG-008: Same-key overwrite suppression compared against the next input beat instead of the just-written key
- First seen in: `P111` and `P119` on 2026-04-17
- Symptom:
  - same-key overwrite pressure drained almost completely, but one or more residents stayed invisible in CAM
  - pre-fix signatures:
    - `P111`: `push=576 pop=511 overwrite=64 remaining=1`
    - earlier before the first overwrite fix: `push=576 pop=448 overwrite=64 remaining=64`
    - `P119`: earlier `push=768 pop=256 overwrite=256 remaining=256`
- Root cause:
  - the overwrite self-erase suppression in the push-erase phase compared `cam_erase_data` with live `in_hit_sk`
  - during a same-key burst this works only while another same-key beat is still present on the input
  - on the final overwrite of the burst, the next input beat is idle or unrelated, so the DUT erroneously erases the newly written same-key resident
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - the RTL now latches the search key that was actually written on `push_write_grant` and uses that latched key in the following erase phase
  - verified by reruns:
    - `P111`: `push=576 pop=512 overwrite=64 remaining=0`
    - `P119`: `push=768 pop=512 overwrite=256 remaining=0`
    - `B005`: still passes with `push=128 pop=128 remaining=0`
- Commit: pending

### BUG-009: `wait_for_scoreboard_idle()` could declare quiescent before accepted ingress was written
- First seen in: `B061` and `B062` on 2026-04-17 during the standalone-UCDB refresh
- Symptom:
  - single-beat framing cases ended with `accepted=1` but `pushed=0`, no subheader/hit history, and therefore no trustworthy isolated evidence for the case body
  - the old harness could save a UCDB for a run that had not actually exercised the intended datapath/output checks yet
- Root cause:
  - the quiescence predicate only checked modeled residents, active epoch state, source backlog, and FIFO empty flags
  - it did **not** require the scoreboard's accepted-ingress count to converge with the observed push-write count, so a short test could finish while an accepted beat was still in the acceptance-to-write gap
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - `wait_for_scoreboard_idle()` now also requires `total_written == total_ingress_accepted`
  - after the fix, the full implemented isolated sweep completed with `55/55` passes and real standalone UCDBs for every implemented case
- Commit: pending

### BUG-010: Framing checks sampled “latest subheader” instead of the case-target epoch
- First seen in: `B062` and `B063` on 2026-04-17 during the standalone-UCDB refresh
- Symptom:
  - subheader search-key and hit-count checks failed even though the DUT was legally draining the directed burst
  - the failing runs showed many zero-hit subheaders from normal time advancement, which polluted `recent_subheaders[$]`
- Root cause:
  - the testcase checks assumed the most recent observed subheader belonged to the directed burst under test
  - in reality the DUT can emit zero-hit subheaders for later empty epochs, so positional sampling from `recent_subheaders[$]` was not stable evidence for `B061-B065`
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - the harness now waits for a subheader matching the target `search_key`, then performs the framing-field checks against that matched epoch
  - verified by rerunning `B061-B065` standalone and then the full `55`-case implemented isolated matrix with zero failures
- Commit: pending

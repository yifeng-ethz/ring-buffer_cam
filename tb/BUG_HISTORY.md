# BUG_HISTORY.md — ring_buffer_cam DV bug ledger

## 2026-04-16

### BUG-001-H: Search-key decode and interleaving model in UVM harness were wrong
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
- Commit: d412d7a

### BUG-002-H: Scoreboard modeled accepted ingress order, not actual push-write grant order
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
- Commit: d412d7a

### BUG-003-H: Stale-snapshot / cache-miss output branch was unmodeled in the harness
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
- Commit: d412d7a

### BUG-004-H: Pressure-case expectation ignored the resequencing-buffer service model
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
- Commit: d412d7a

### BUG-005-H: Run-control startup/terminate sequencing used fixed waits instead of the DUT handshake
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
- Commit: d412d7a

### BUG-006-H: Error-counter assumptions were too narrow for lane-mismatched bad hits
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
- Commit: d412d7a

### BUG-007-R: 48-bit counter cleanup logic truncated through `to_integer()`
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
- Commit: d412d7a

## 2026-04-17

### BUG-008-R: Same-key overwrite suppression compared against the next input beat instead of the just-written key
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
- Commit: d412d7a

### BUG-009-H: `wait_for_scoreboard_idle()` could declare quiescent before accepted ingress was written
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
  - the later full implemented isolated nightly sweep completed with `123/123` passes and real standalone UCDBs for every implemented case currently wired in the case engine
- Commit: d412d7a

### BUG-010-H: Framing checks sampled “latest subheader” instead of the case-target epoch
- First seen in: `B062`, `B063`, and later `E005` on 2026-04-17 during isolated standalone-UCDB refresh
- Symptom:
  - subheader search-key and hit-count checks failed even though the DUT was legally draining the directed burst
  - the failing runs showed many zero-hit subheaders from normal time advancement, which polluted `recent_subheaders[$]`
- Root cause:
  - the testcase checks assumed the most recent observed subheader belonged to the directed burst under test
  - in reality the DUT can emit zero-hit subheaders for later empty epochs, so positional sampling from `recent_subheaders[$]` was not stable evidence
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - the harness now waits for a subheader matching the target `search_key`, and the monitor tracks data-bearing subheaders separately from zero-hit background traffic
  - verified by rerunning the framing/basic slice and then the full `123/123` implemented isolated nightly matrix with zero failures
- Commit: d412d7a

### BUG-011-H: META VERSION/DATE checks drifted behind the packaged build metadata
- First seen in: full isolated nightly rerun on 2026-04-17, `B010`, `B011`
- Symptom:
  - `B010` failed with `META VERSION readback mismatch` (`0x1a0151a1` observed vs old `0x1a014192`)
  - `B011` failed with `META DATE readback mismatch` (`20260417` observed vs old `20260402`)
- Root cause:
  - the UVM harness and DV plan still expected the older packaged build/date constants
  - the live wrapper/core metadata had already moved forward with the packaged IP release
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - aligned `expected_meta_version()`, the DATE checks, and the BASIC-plan text with the live package metadata
  - this also triggered the build-number bump to `26.1.5.0418` for the current nightly DV checkpoint
- Commit: d412d7a

### BUG-012-H: Mid-drain flush testcase reset the scoreboard before FLUSHING actually owned the datapath
- First seen in: `B093` on 2026-04-17 during the new BASIC nightly slice
- Symptom:
  - `B093` initially failed with many `Observed hit with no active subheader context` scoreboard errors
  - the DUT-side preemption condition itself was reached, but the testcase destroyed the scoreboard epoch context too early
- Root cause:
  - the testcase called `note_flush_reset()` before the RUN_PREPARE / FLUSHING path had actually preempted the active DRAIN
  - residual hit beats from the in-flight epoch were therefore judged against an already-reset scoreboard context
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - the testcase now waits for `decision_reg=3` / FLUSHING preemption, verifies `pop_erase_grant` drops, and only then resets the scoreboard model
  - after the fix, the full targeted slice and the full `123/123` implemented nightly rerun passed with `B093` green
- Commit: d412d7a

### BUG-013-H: Long-span EDGE cases assumed one subheader represented the whole same-key drain epoch
- First seen in: `E015` and `E016` on 2026-04-17 during the refreshed EDGE nightly slice
- Symptom:
  - `E015` expected a wrapped hit-count of `1` but observed `240`
  - `E016` expected a wrapped hit-count of `0` but observed `16`
  - both runs still drained all hits losslessly according to the scoreboard and counters
- Root cause:
  - the DUT packetizes long same-key drains into multiple data-bearing subheaders
  - the testcase incorrectly assumed one subheader encoded the total epoch cardinality instead of a packetized chunk count
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - `out_monitor` now keeps `recent_data_subheaders` / `total_data_subheaders_seen`
  - `E015` and `E016` now check the real contract: full lossless drain plus multi-subheader packetization with the correct search key
  - verified in the full `123/123` implemented isolated nightly matrix with zero failures
- Commit: d412d7a

### BUG-014-H: Long-run scoreboard lost drain traceability when a recycled slot was popped after the model had already cleared it
- First seen in: `E023` on 2026-04-17 during the first steady-state EDGE long-run bring-up
- Symptom:
  - the long-run cadence testcase produced `Unexpected drained hit` scoreboard errors even though CSR accounting closed at `push=2000`, `pop=2000`, `overwrite=0`, and the DUT drained the traffic losslessly
  - the failure clustered on recycled slot `0`, where a later `pop_erase` arrived after the scoreboard had already cleared that slot from its resident model
- Root cause:
  - `write_pop()` assumed every occupied `pop_erase` must still correspond to a live slot-model entry
  - in the long-run slot-reuse corner case, the model could already have cleared that slot while the DUT still exposed the raw side-RAM payload for the consumed resident, so the scoreboard dropped the pending-drain trace and later flagged the emitted hit as unexpected
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - the scoreboard now recovers the pending drain from the raw pop-side payload when the slot model is already clear, instead of dropping the event
  - verified by rerunning `E023` cleanly after the fix; the testcase now passes with bounded backlog (`max_remaining=71`) and zero UVM errors
- Commit: e8c18fe

### BUG-015-H: Long-run output matching reused the pending-drain queue index as a live-slot index and could clear the wrong resident
- First seen in: `P004` on 2026-04-17 during the first post-`E023` full nightly rerun
- Symptom:
  - the long overwrite-profile testcase still produced residual residents and late `Unexpected drained hit` failures after the earlier slot-reuse recovery fix
  - failures appeared only under deeper pressure runs where pending-drain matches and live-slot matches diverged
- Root cause:
  - `scoreboard.write_out()` reused one match-index variable across the pending-drain queue, the live slot model, and the overlap-evicted queue
  - when a pending-drain match was found first, the later cleanup path could clear an unrelated live resident at the same numeric index
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - the scoreboard now uses separate `pending_match_idx`, `live_match_idx`, and `overlap_match_idx` variables
  - verified by rerunning `E023`, `P004`, `P111`, `P112`, `P113`, `P116`, `P119`, then the full `130/130` implemented isolated nightly matrix with zero failures
- Commit: e8c18fe

### BUG-016-H: Deep overwrite-profile stimulus reused the full scoreboard fingerprint tuple every 2048 hits
- First seen in: `P004` debug on 2026-04-17 while tracing the remaining long-pressure mismatch after `BUG-015-H`
- Symptom:
  - deep overwrite-profile runs became hard to attribute because distinct late hits could alias to the same scoreboard fingerprint tuple
  - the ambiguity did not always fail the case directly, but it obscured root-cause isolation and weakened deep-pressure traceability
- Root cause:
  - `overwrite_profile_seq` only varied a subset of the hit fields, so the complete `(search_key, ts_low, asic, channel, ts50p, et1n6)` identity repeated every 2048 hits
  - the pressure testcase therefore lost deterministic uniqueness exactly where the nightly overwrite runs needed it most
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - `overwrite_profile_seq` now varies the full stored fingerprint tuple across long runs instead of only a partial subset
  - verified together with `BUG-015-H` by the targeted pressure slice and the full `130/130` implemented isolated nightly matrix
- Commit: e8c18fe

## 2026-04-18

### BUG-017-H: Partition-stress BASIC cases hardcoded a 4-way geometry and sampled packetized output too late
- First seen in: `B075`, `B079`, and `B080` on 2026-04-18 during the next nightly BASIC expansion
- Symptom:
  - the new partition cases failed even though the DUT drained all hits losslessly
  - `B075/B079/B080` assumed `MATCH_PARTITION_SIZE=128`, but the active `default_p2_pipe4` build runs with `G_N_PARTITIONS=2`, so the real partition size is `256`
  - `B075` also assumed a single `hit_count=3` subheader even though the output path packetized the three-hit drain across multiple data-bearing subheaders
- Root cause:
  - the testcase math was written against a stale 4-partition mental model instead of `RING_BUFFER_N_ENTRY/N_PARTITIONS`
  - the monitor logic started sampling the partition walk after subheader emission, which missed the earlier two-partition issue window
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - `B075/B078/B079/B080` now derive partition size from the live config, force occupancy before drain, and sample pending/issue behavior at DRAIN entry
  - verified first by targeted reruns, then by a clean full `137/137` implemented isolated nightly matrix
- Commit: cec8e58

### BUG-018-H: DV report publication linked evidence into volatile `work_uvm` paths and collapsed after targeted reruns
- First seen in: `DV_REPORT.md` refresh on 2026-04-18 after the isolated `B071/B072` batch
- Symptom:
  - a targeted two-case rerun incorrectly collapsed the dashboard from `137` executed cases to `2`
  - the old published logs and UCDBs had been emitted as symlinks into `tb/uvm/work_uvm/`, so the next isolated run cleaned the backing files and left the report with placeholders
- Root cause:
  - `generate_dv_report.py` treated `tb/uvm/logs/` and `tb/uvm/cov_after/` as if they could safely point into the volatile work tree
  - once the work tree was cleaned, a subsequent regeneration lost real evidence for every case that had not just been rerun
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - the publisher now materializes durable copies into `tb/uvm/logs/` and `tb/uvm/cov_after/` instead of leaving volatile symlinks behind
  - verified by rerunning the full `137/137` implemented isolated matrix, regenerating the report, and confirming there are now `0` symlinks left in both evidence directories
- Commit: cec8e58

### BUG-019-H: E026 partition-walk harness observed the wrong partition index during skip logic
- First seen in: `E026` on 2026-04-18 during the first `E017/E025/E026/E032` tranche run
- Symptom:
  - `E026` failed with `Last partition P1 was never visited after drain start` and `Partition visits did not match expected mask 0x3`
  - counters still showed `pushed==popped`, so the DUT output itself appeared lossless while case bookkeeping missed partition `0x1`
- Root cause:
  - the testcase sample path read `m_env.m_dbg_mon.pop_issue_partition_idx` instead of deriving the active partition from the surfaced `pop_issue_addr`
  - in the `default_p2_pipe4` geometry this produced inconsistent partition bookkeeping when round-robin skipped empty partitions
- Fix status: fixed and verified on `dev`
- Runtime / coverage context:
  - `E026` now derives `visit_idx = pop_issue_addr / partition_size` on each pop issue and masks checks against that index
  - this aligns with the DUT interface contract used by other partition checks and removes the false-negative on empty-partition skips
- Commit: 63e3652

### BUG-020-H: B116 sampled FILL_LEVEL before the fourth/eighth/etc push grant had actually landed
- First seen in: `B116` on 2026-04-18 during the first `B116-B123` tranche run
- Symptom:
  - the new end-to-end fill trajectory case failed with `observed=3 expected=4`, `observed=7 expected=8`, and the same off-by-one pattern at the later checkpoints
  - the testcase still terminated cleanly with `pushed=16 popped=16 remaining=0`, so the DUT accounting was correct while the CSR sampling point was early
- Root cause:
  - the testcase assumed that four/eight/twelve/sixteen `single_push_pop_seq.start()` calls implied that the corresponding push grants had already committed into the debug counters and `FILL_LEVEL`
  - in reality the CSR read raced the final accepted beat in each checkpoint group and sampled one cycle before the last increment became visible
- Fix status: fixed and verified on `dev`
- Runtime / coverage context:
  - `B116` now waits for `dbg_push_cnt` to reach the checkpoint target before polling `FILL_LEVEL`
  - verified by a clean rerun of `B116-B123` and then the full `161/161` implemented isolated nightly matrix
- Commit: 740adc0

### BUG-021-H: FLUSHING cursor checks expected the terminal address to persist instead of validating the DUT's completion rollover
- First seen in: `B105` and `B106` on 2026-04-18 during the first flush-walk BASIC tranche run
- Symptom:
  - `B105` failed with `pop_flush_cam_done=1 addr=0`
  - `B106` failed with `pop_flush_ram_done=1 addr=0`
  - both cases had otherwise clean control-state progress, indicating the DUT completed the flush walk while the testcase rejected the final cursor value
- Root cause:
  - the harness assumed the terminal cursor would remain at `0x1ff` when the done pulse asserted
  - in the RTL FLUSHING state, both `flush_cam_wraddr` and `flush_ram_wraddr` increment on the final granted cycle and therefore wrap to `0` at the same edge that raises the corresponding done bit
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - `B105` now checks the documented walk property plus the terminal rollover semantics: previous address reaches `0x1ff`, current address wraps to `0`, and `pop_flush_cam_done` asserts
  - `B106` now checks the same rollover contract for the RAM flush cursor
  - verified by a clean rerun of `B098/B100/B101/B102/B103/B105/B106` after the fix
- Commit: 84b13e9

### BUG-022-H: META version smoke test hardcoded the previous build stamp and failed immediately after a legal VERSION bump
- First seen in: `B010` on 2026-04-18 during the first full `0424` nightly rerun
- Symptom:
  - `B010` failed with `META VERSION readback mismatch` and reported `got 0x1a0151a8 expected 0x1a0151a7`
  - all other early BASIC cases stayed green, indicating the DUT metadata path had moved while the testcase expectation lagged one build stamp behind
- Root cause:
  - `expected_meta_version()` in `base_test.sv` still packed build `423` after the verified harness bug-fix batch bumped the IP to `26.1.5.0424`
  - the testcase therefore encoded stale harness metadata instead of matching the current wrapper/core version word
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - `expected_meta_version()` now packs the live `26.1.5.0424` META word used by the current IP build
  - verified by a clean isolated rerun of `B010` after the fix
- Commit: 84b13e9

### BUG-023-R: TERMINATE from IDLE deadlocks `asi_ctrl_ready` because `terminating_drain_done` can never rise without `endofrun_seen`
- First seen in: `X099` on 2026-04-18 during the post-`d52b587` control-path ERROR tranche
- Symptom:
  - driving `CTRL_TERMINATING` while the DUT is in `IDLE` transitions `run_state_cmd` to `TERMINATING` but never asserts `asi_ctrl_ready`
  - the targeted run failed with `ack=0 state=4 endofrun_seen=0 term_done=0`
- Root cause:
  - `proc_run_control_mgmt` only acks `TERMINATING` when `terminating_drain_done='1'`
  - `terminating_drain_done` is hard-gated by `endofrun_seen='1'`, even when the DUT is already empty and idle
  - as a result, an out-of-order or redundant TERMINATE command from IDLE wedges the control interface in `TERMINATING` with no legal path to ready
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - reproduced by isolated case `X099`
  - fixed by adding a quiescent `IDLE -> TERMINATING` ready path while keeping the normal end-of-run `terminating_drain_done` gate unchanged
  - verified by a clean focused rerun of `B010`, `B033-B041`, `B071`, `X031`, and `X099`
  - related RTL logic:
    - `terminating_drain_done` gate at `rtl/ring_buffer_cam_v2_core.vhd:1633-1639`
    - `TERMINATING` ready gate at `rtl/ring_buffer_cam_v2_core.vhd:1765-1770`
- Commit: b203a04

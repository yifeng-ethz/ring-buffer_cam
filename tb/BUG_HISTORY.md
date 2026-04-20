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
- Fix status: fixed and verified on `dev`
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
- Fix status: fixed and verified on `dev`
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
- Fix status: fixed and committed
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

### BUG-024-H: X039 assumed one `RUN_PREPARE` send both entered PREP and waited for flush completion
- First seen in: `X039` on 2026-04-18 during the next control-path ERROR tranche after `BUG-023-R`
- Symptom:
  - `X039` timed out waiting for `CTRL_RUN_PREPARE` acknowledgement after a `TERMINATING -> RUN_PREPARE` transition
  - the failing run then reported `RUN_PREPARE did not complete the flush handshake after TERMINATING` even though the DUT had already moved into `RUN_STATE_RUN_PREPARE`
- Root cause:
  - the testcase used a single `ctrl_send(CTRL_RUN_PREPARE)` and assumed it both pulsed the control command into PREP and waited for the later flush-complete acknowledgement
  - `ctrl_driver.sv` intentionally splits those semantics: the first `RUN_PREPARE` pulse only enters PREP when the DUT is in another run state, and a second `RUN_PREPARE` send is required to wait for the flush-complete ready path once PREP is active
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - `X039` now follows the real driver contract: raw PREP pulse, wait for `RUN_STATE_RUN_PREPARE`, reset the scoreboard flush epoch, then send `RUN_PREPARE` again to wait for the flush-complete acknowledgement
  - verified by rerunning `B010`, `X039`, `X042`, and `X044` cleanly on the `0426` build before regenerating the dashboard
  - related harness logic:
    - testcase sequencing in `tb/uvm/base_test.sv`
    - PREP command contract in `tb/uvm/ctrl_driver.sv`
- Commit: c2bcc79

### BUG-025-R: Post-flush quiescent `TERMINATING` from `RUN_PREPARE` never acknowledged
- First seen in: `X065` on 2026-04-18 during the next control-path ERROR tranche after `BUG-024-H`
- Symptom:
  - after an empty `RUN_PREPARE` flush, issuing `CTRL_TERMINATING` moved `run_state_cmd` into `TERMINATING` but never raised `asi_ctrl_ready`
  - the failing run stayed quiescent with `endofrun_seen=0`, `deassembly_fifo_empty=1`, `pop_cmd_fifo_empty=1`, `push=0`, `pop=0`, and still timed out waiting for the terminate acknowledgement
- Root cause:
  - the earlier quiescent terminate fix only covered the entry case indirectly and still relied on the `TERMINATING` state logic to rediscover that the transition began from an already-empty state
  - once the DUT was in `RUN_PREPARE -> TERMINATING`, the ready path still fell through the normal `terminating_drain_done` gate, which remained `0` because `endofrun_seen` was never supposed to assert in this quiescent post-flush path
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - the RTL now latches `terminating_entry_quiescent` when a `TERMINATING` episode starts from an already-empty state and uses that latched fact in the `TERMINATING` ready gate
  - verified by a clean focused rerun of `B010`, `X031`, `X038`, `X039`, `X042`, `X043`, `X044`, `X046`, `X048`, `X051`, `X065`, and `X099` on build `26.1.5.0427`
  - related RTL logic:
    - `terminating_drain_done` / quiescent detection in `rtl/ring_buffer_cam_v2_core.vhd`
    - `TERMINATING` ready gate in `rtl/ring_buffer_cam_v2_core.vhd`
- Commit: f575af8

## 2026-04-19

### BUG-026-H: The first FLUSH counter-observer tranche encoded the wrong contract and reset scoreboard epoch under an active drain
- First seen in: `X111` / `X112` on 2026-04-19 during the next ERROR counter-observer tranche after `BUG-025-R`
- Symptom:
  - `X111` initially failed by expecting `PUSH_COUNT` to be retained across FLUSH even though the broader DV plan already required counters `6/7/8/9` to clear on `decision_reg=3`
  - `X112` then failed with `31` scoreboard "Observed hit with no active subheader context" errors because the testcase called `note_flush_reset()` while the DUT was still legally finishing the already-issued drain payload
- Root cause:
  - the testcase intent drifted away from the canonical FLUSH contract documented elsewhere in the plan (`B030` and the existing FLUSH cleanup cases)
  - `X112` also reused the flush-epoch reset helper in a situation where the scoreboard still needed the pre-FLUSH subheader context to retire in-flight hits cleanly
- Fix status: fixed in working tree, not yet committed
- Runtime / coverage context:
  - `X111` now verifies that `PUSH_COUNT` clears to zero during FLUSHING
  - `X112` now verifies `POP_COUNT` clear semantics without dropping the active subheader epoch
  - verified by clean isolated reruns of `X101`, `X102`, `X104`, `X106`, `X109`, `X111`, and `X112`
  - related harness logic:
    - `tb/uvm/base_test.sv`
    - `tb/DV_ERROR.md`
- Commit: e182765

### BUG-027-R: MM CSR traffic masked same-cycle `INERR_COUNT` updates
- First seen in: `X113` on 2026-04-19 during the backdoor counter-observer ERROR tranche
- Symptom:
  - a bad hit coincident with a `CSR_CTRL` soft-reset write left both the backdoor `debug_msg2.inerr_cnt` and the frontdoor `INERR_COUNT` at `0`
  - the bad beat was filtered as expected (`PUSH_COUNT` stayed `0`), but the raw error observer event disappeared entirely
- Root cause:
  - `proc_avmm_csr` updated `debug_msg2.inerr_cnt` only in the MM-idle branch
  - any same-cycle `avs_csr_read='1'` or `avs_csr_write='1'` therefore skipped the `INERR_COUNT` increment path, even though the raw ingress error pulse was present on `asi_hit_type1_error`
- Fix status: fixed and committed
- Runtime / coverage context:
  - `debug_msg2.inerr_cnt` accounting now runs after the MM read/write/idle decode so CSR bus traffic no longer masks the raw error observer
  - verified by clean isolated reruns of `X076`, `X113`, and `X114`, then by clean isolated reruns of `X022`, `X023`, `X024`, `X025`, and `X030`
  - related RTL logic:
    - `rtl/ring_buffer_cam_v2_core.vhd` `proc_avmm_csr`
- Commit: e182765

### BUG-028-H: The original `X019` raw boundary injector did not actually drive an ingress beat
- First seen in: `X019` on 2026-04-19 while extending the same counter-observer/boundary ERROR tranche
- Symptom:
  - a supposed one-cycle raw bad hit driven exactly with the `CTRL_RUNNING` entry pulse left both `INERR_COUNT` and `PUSH_COUNT` at `0`
  - added ingress visibility later showed `boundary_valid=0` and `retry_valid=0`, meaning the testcase never placed a real beat on the DUT interface
- Root cause:
  - the old boundary injector relied on ad-hoc raw interface pokes that did not survive the UVM driver scheduling path
  - `X019` was therefore proving a stimulus hole, not a DUT loss of a legally presented boundary beat
- Fix status: fixed and verified
- Runtime / coverage context:
  - fixed by adding ingress observability to the debug path and rewriting `X019` to use a sequencer-aligned bad-hit injection one cycle ahead of `CTRL_RUNNING`
  - verified by clean isolated reruns of `X019` together with the surrounding flush-sensitive cases `X017` and `X055`
- Commit: e85f536

### BUG-029-H: `note_flush_reset()` cleared the resident model but not the scoreboard epoch counters
- First seen in: `P118` on 2026-04-19 while bringing up rotating overwrite-pressure windows with a mid-case flush/restart
- Symptom:
  - multi-window cases that called `enter_run_prepare()` mid-test could never satisfy the final idle condition
  - after the flush, the scoreboard still carried prior `total_ingress_accepted`, `total_written`, overwrite maps, and peak-residency data, so later windows were compared against a mixed epoch
- Root cause:
  - `scoreboard.note_flush_reset()` only invalidated slot state and pending queues
  - it did not reset the per-epoch totals, overwrite ownership maps, or max-residency tracking that `wait_for_scoreboard_idle()` and the PERF cases treat as epoch-local
- Fix status: fixed and verified
- Runtime / coverage context:
  - exposed by isolated case `P118`, which timed out after a legal flush/restart boundary with `accepted=1280` and `written=1232` even though the resident model was empty
  - fixed by resetting the epoch counters/maps in `scoreboard.note_flush_reset()` and verified by clean reruns of `P118`, `P121`, `P124`, `X017`, `X019`, and `X055`
- Commit: e85f536

## 2026-04-20

### BUG-030-H: `P030`'s original zero-gap 256-hit burst profile ran above the calibrated no-overwrite envelope
- First seen in: `P030` on 2026-04-20 during the next PROF multi-key bring-up tranche
- Symptom:
  - the new rotating-burst profile produced hundreds of scoreboard `Unexpected drained hit` errors as soon as the third burst arrived
  - local diagnostics showed the case was already recycling occupied residents while key `8` was still draining, which violated the testcase's own `OVERWRITE_COUNT==0` contract
- Root cause:
  - the original case definition used `inter_hit_gap_cycles=0` for 256-hit same-key bursts
  - that traffic shape exceeded the sustainable no-overwrite service envelope for the delivered `EXPECTED_LATENCY=128` / `512`-entry configuration, so the testcase was proving a self-induced overload instead of a legal lossless burst regime
- Fix status: fixed and verified
- Runtime / coverage context:
  - `P030` was recalibrated to keep the 256-hit burst packetization focus while pacing the bursts with a 13-cycle ingress gap and a 176-cycle epoch gap
  - verified by clean isolated reruns of `P030`, then by a clean PROF slice rerun of `P026`, `P027`, `P030`, `P032`, and `P033`
- Commit: pending

### BUG-031-H: The PROF silent-key plan text encoded the wrong contract for zero-hit searches
- First seen in: `P034` planning review on 2026-04-20 while wiring the silent-key PROF cases
- Symptom:
  - the plan text for `P034` / `P035` required `CACHE_MISS_COUNT` to increment on silent-key searches
  - that contradicted the already-verified directed evidence (`B059`, `B060`, `B118`, `E001`, `E031`), which showed that empty searches emit zero-hit subheaders while `CACHE_MISS_COUNT` remains `0`
- Root cause:
  - the PROF spec drifted from the live zero-hit fast-path contract and conflated "silent-key search" with the stale-snapshot cache-miss datapath
  - the stale wording would have forced new PROF checks to claim failures against a legal DUT behavior
- Fix status: fixed and verified
- Runtime / coverage context:
  - `P034` / `P035` were implemented against the true contract: zero-hit SOP+EOP subheaders for silent keys with `CACHE_MISS_COUNT==0`
  - verified by clean isolated reruns of `P034` and `P035`, both of which observed hundreds of zero-hit subheaders while preserving lossless active-key drain accounting
- Commit: pending

### BUG-032-H: `profile_traffic_seq` could only model contiguous key windows, so interleaved active/silent-key profiles were not expressible faithfully
- First seen in: `P035` implementation on 2026-04-20 while trying to model three active keys interleaved with two silent searched keys
- Symptom:
  - the existing profile generator only derived keys as `lane_key_start_ord + key_idx`
  - that made it impossible to generate the intended active set `{2,4,6}` while leaving `{3,5}` silent, so the testcase would have had to either violate the scenario or silently test a different key pattern
- Root cause:
  - `profile_traffic_seq` lacked an explicit key-list mode and assumed every profile traffic family was a contiguous key range
  - the sequence infrastructure therefore could not represent the interleaved active/silent-key window required by the PROF plan
- Fix status: fixed and verified
- Runtime / coverage context:
  - `profile_traffic_seq` now accepts an explicit `lane_key_ord_list[$]`, and `P035` uses that mode to drive only the intended active keys while the silent keys are still searched by the DUT
  - verified by clean isolated reruns of `P034`, `P035`, and `P039`
- Commit: pending

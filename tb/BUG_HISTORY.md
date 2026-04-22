# BUG_HISTORY.md - ring_buffer_cam DV bug ledger

Class legend:
- `R` = RTL / DUT bug
- `H` = harness / testcase / reporting bug

Severity legend:
- `soft error` = a contract mismatch that corrupts one case window but does not poison later case behavior after the testcase naturally drains/restarts
- `hard stuck error` = the bug can wedge later traffic or control handshakes until the DUT/testcase forces a fresh restart or reset
- `non-datapath-refactor` = observability, reporting, testcase, or accounting consistency work with no direct payload-contract corruption

Encounter sim-time legend:
- `min / p50 / max` = first encounter in simulation time under a still-traceable randomized screen with multiple seeded reruns
- `n/a (...)` = directed-only, reporting-only, promotion-audit, or otherwise not honestly measurable in the current randomized harness

Fix status detail contract for active entries and future updates:
- `state` = fixed / open / partial plus the current verification gate
- `mechanism` = how the implemented repair changes the RTL or harness behavior
- `before_fix_outcome` and `after_fix_outcome` = concise evidence showing what changed
- `potential_hazard` = whether the fix looks permanent or is still provisional / profile-limited
- `Claude Opus 4.7 xhigh review decision` = explicit review state; use `pending / not run` until that review has actually happened

Historical formal note:
- `BUG-033-H` onward follows the packet-scheduler severity / encounter-sim-time style explicitly.
- Earlier entries remain valid historical content and are being backfilled incrementally instead of rewritten in one opaque bulk edit.
- The current supported simulator runtime is `QuestaOne 2026` at `/data1/questaone_sim/questasim`.
- No standalone formal-bug subsection is maintained in this `ring_buffer_cam` ledger yet.

## Index

| bug_id | class | severity | encounter sim-time | status | first seen | commit | summary |
|---|---|---|---|---|---|---|---|
| [BUG-001-H](#bug-001-h-search-key-decode-and-interleaving-model-in-uvm-harness-were-wrong) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `test_sequential_keys`, `test_random_multi_key`, `test_random_throughput` | `d412d7a` | Search-key decode and interleaving model in UVM harness were wrong |
| [BUG-002-H](#bug-002-h-scoreboard-modeled-accepted-ingress-order-not-actual-push-write-grant-order) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `test_overwrite_stress`, `E003` | `d412d7a` | Scoreboard modeled accepted ingress order, not actual push-write grant order |
| [BUG-003-H](#bug-003-h-stale-snapshot-cache-miss-output-branch-was-unmodeled-in-the-harness) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `test_overwrite_stress`, `E003` | `d412d7a` | Stale-snapshot / cache-miss output branch was unmodeled in the harness |
| [BUG-004-H](#bug-004-h-pressure-case-expectation-ignored-the-resequencing-buffer-service-model) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | in progress | `E003` planning review and user clarification on 2026-04-16 | `d412d7a` | Pressure-case expectation ignored the resequencing-buffer service model |
| [BUG-005-H](#bug-005-h-run-control-startup-terminate-sequencing-used-fixed-waits-instead-of-the-dut-handshake) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `E003`, `P111`, general `test_case_engine` runtime review on 2026-04-16 | `d412d7a` | Run-control startup/terminate sequencing used fixed waits instead of the DUT handshake |
| [BUG-006-H](#bug-006-h-error-counter-assumptions-were-too-narrow-for-lane-mismatched-bad-hits) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `X013` on 2026-04-16 | `d412d7a` | Error-counter assumptions were too narrow for lane-mismatched bad hits |
| [BUG-007-R](#bug-007-r-48-bit-counter-cleanup-logic-truncated-through-tointeger) | R | soft error | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | long soak / signoff review for super-long counter and toggle runs on 2026-04-16 | `d412d7a` | 48-bit counter cleanup logic truncated through `to_integer()` |
| [BUG-008-R](#bug-008-r-same-key-overwrite-suppression-compared-against-the-next-input-beat-instead-of-the-just-written-key) | R | soft error | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `P111` and `P119` on 2026-04-17 | `d412d7a` | Same-key overwrite suppression compared against the next input beat instead of the just-written key |
| [BUG-009-H](#bug-009-h-waitforscoreboardidle-could-declare-quiescent-before-accepted-ingress-was-written) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `B061` and `B062` on 2026-04-17 during the standalone-UCDB refresh | `d412d7a` | `wait_for_scoreboard_idle()` could declare quiescent before accepted ingress was written |
| [BUG-010-H](#bug-010-h-framing-checks-sampled-latest-subheader-instead-of-the-case-target-epoch) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `B062`, `B063`, and later `E005` on 2026-04-17 during isolated standalone-UCDB refresh | `d412d7a` | Framing checks sampled “latest subheader” instead of the case-target epoch |
| [BUG-011-H](#bug-011-h-meta-version-date-checks-drifted-behind-the-packaged-build-metadata) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | full isolated nightly rerun on 2026-04-17, `B010`, `B011` | `d412d7a` | META VERSION/DATE checks drifted behind the packaged build metadata |
| [BUG-012-H](#bug-012-h-mid-drain-flush-testcase-reset-the-scoreboard-before-flushing-actually-owned-the-datapath) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `B093` on 2026-04-17 during the new BASIC nightly slice | `d412d7a` | Mid-drain flush testcase reset the scoreboard before FLUSHING actually owned the datapath |
| [BUG-013-H](#bug-013-h-long-span-edge-cases-assumed-one-subheader-represented-the-whole-same-key-drain-epoch) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `E015` and `E016` on 2026-04-17 during the refreshed EDGE nightly slice | `d412d7a` | Long-span EDGE cases assumed one subheader represented the whole same-key drain epoch |
| [BUG-014-H](#bug-014-h-long-run-scoreboard-lost-drain-traceability-when-a-recycled-slot-was-popped-after-the-model-had-already-cleared-it) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `E023` on 2026-04-17 during the first steady-state EDGE long-run bring-up | `e8c18fe` | Long-run scoreboard lost drain traceability when a recycled slot was popped after the model had already cleared it |
| [BUG-015-H](#bug-015-h-long-run-output-matching-reused-the-pending-drain-queue-index-as-a-live-slot-index-and-could-clear-the-wrong-resident) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `P004` on 2026-04-17 during the first post-`E023` full nightly rerun | `e8c18fe` | Long-run output matching reused the pending-drain queue index as a live-slot index and could clear the wrong resident |
| [BUG-016-H](#bug-016-h-deep-overwrite-profile-stimulus-reused-the-full-scoreboard-fingerprint-tuple-every-2048-hits) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `P004` debug on 2026-04-17 while tracing the remaining long-pressure mismatch after `BUG-015-H` | `e8c18fe` | Deep overwrite-profile stimulus reused the full scoreboard fingerprint tuple every 2048 hits |
| [BUG-017-H](#bug-017-h-partition-stress-basic-cases-hardcoded-a-4-way-geometry-and-sampled-packetized-output-too-late) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `B075`, `B079`, and `B080` on 2026-04-18 during the next nightly BASIC expansion | `cec8e58` | Partition-stress BASIC cases hardcoded a 4-way geometry and sampled packetized output too late |
| [BUG-018-H](#bug-018-h-dv-report-publication-linked-evidence-into-volatile-workuvm-paths-and-collapsed-after-targeted-reruns) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `DV_REPORT.md` refresh on 2026-04-18 after the isolated `B071/B072` batch | `cec8e58` | DV report publication linked evidence into volatile `work_uvm` paths and collapsed after targeted reruns |
| [BUG-019-H](#bug-019-h-e026-partition-walk-harness-observed-the-wrong-partition-index-during-skip-logic) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `E026` on 2026-04-18 during the first `E017/E025/E026/E032` tranche run | `63e3652` | E026 partition-walk harness observed the wrong partition index during skip logic |
| [BUG-020-H](#bug-020-h-b116-sampled-filllevel-before-the-fourth-eighth-etc-push-grant-had-actually-landed) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `B116` on 2026-04-18 during the first `B116-B123` tranche run | `740adc0` | B116 sampled FILL_LEVEL before the fourth/eighth/etc push grant had actually landed |
| [BUG-021-H](#bug-021-h-flushing-cursor-checks-expected-the-terminal-address-to-persist-instead-of-validating-the-duts-completion-rollover) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `B105` and `B106` on 2026-04-18 during the first flush-walk BASIC tranche run | `84b13e9` | FLUSHING cursor checks expected the terminal address to persist instead of validating the DUT's completion rollover |
| [BUG-022-H](#bug-022-h-meta-version-smoke-test-hardcoded-the-previous-build-stamp-and-failed-immediately-after-a-legal-version-bump) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `B010` on 2026-04-18 during the first full `0424` nightly rerun | `84b13e9` | META version smoke test hardcoded the previous build stamp and failed immediately after a legal VERSION bump |
| [BUG-023-R](#bug-023-r-terminate-from-idle-deadlocks-asictrlready-because-terminatingdraindone-can-never-rise-without-endofrunseen) | R | hard stuck error | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `X099` on 2026-04-18 during the post-`d52b587` control-path ERROR tranche | `b203a04` | TERMINATE from IDLE deadlocks `asi_ctrl_ready` because `terminating_drain_done` can never rise without `endofrun_seen` |
| [BUG-024-H](#bug-024-h-x039-assumed-one-runprepare-send-both-entered-prep-and-waited-for-flush-completion) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `X039` on 2026-04-18 during the next control-path ERROR tranche after `BUG-023-R` | `c2bcc79` | X039 assumed one `RUN_PREPARE` send both entered PREP and waited for flush completion |
| [BUG-025-R](#bug-025-r-post-flush-quiescent-terminating-from-runprepare-never-acknowledged) | R | hard stuck error | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `X065` on 2026-04-18 during the next control-path ERROR tranche after `BUG-024-H` | `f575af8` | Post-flush quiescent `TERMINATING` from `RUN_PREPARE` never acknowledged |
| [BUG-026-H](#bug-026-h-the-first-flush-counter-observer-tranche-encoded-the-wrong-contract-and-reset-scoreboard-epoch-under-an-active-drain) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `X111` / `X112` on 2026-04-19 during the next ERROR counter-observer tranche after `BUG-025-R` | `e182765` | The first FLUSH counter-observer tranche encoded the wrong contract and reset scoreboard epoch under an active drain |
| [BUG-027-R](#bug-027-r-mm-csr-traffic-masked-same-cycle-inerrcount-updates) | R | soft error | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `X113` on 2026-04-19 during the backdoor counter-observer ERROR tranche | `e182765` | MM CSR traffic masked same-cycle `INERR_COUNT` updates |
| [BUG-028-H](#bug-028-h-the-original-x019-raw-boundary-injector-did-not-actually-drive-an-ingress-beat) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `X019` on 2026-04-19 while extending the same counter-observer/boundary ERROR tranche | `e85f536` | The original `X019` raw boundary injector did not actually drive an ingress beat |
| [BUG-029-H](#bug-029-h-noteflushreset-cleared-the-resident-model-but-not-the-scoreboard-epoch-counters) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `P118` on 2026-04-19 while bringing up rotating overwrite-pressure windows with a mid-case flush/restart | `e85f536` | `note_flush_reset()` cleared the resident model but not the scoreboard epoch counters |
| [BUG-030-H](#bug-030-h-p030s-original-zero-gap-256-hit-burst-profile-ran-above-the-calibrated-no-overwrite-envelope) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `P030` on 2026-04-20 during the next PROF multi-key bring-up tranche | `d7ce37a` | `P030`'s original zero-gap 256-hit burst profile ran above the calibrated no-overwrite envelope |
| [BUG-031-H](#bug-031-h-the-prof-silent-key-plan-text-encoded-the-wrong-contract-for-zero-hit-searches) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `P034` planning review on 2026-04-20 while wiring the silent-key PROF cases | `d7ce37a` | The PROF silent-key plan text encoded the wrong contract for zero-hit searches |
| [BUG-032-H](#bug-032-h-profiletrafficseq-could-only-model-contiguous-key-windows-so-interleaved-active-silent-key-profiles-were-not-expressible-faithfully) | H | non-datapath-refactor | `n/a (historical entry; explicit encounter sim-time not recorded)` | fixed | `P035` implementation on 2026-04-20 while trying to model three active keys interleaved with two silent searched keys | `d7ce37a` | `profile_traffic_seq` could only model contiguous key windows, so interleaved active/silent-key profiles were not expressible faithfully |
| [BUG-033-H](#bug-033-h-long-run-prof-generators-reused-overlapping-low-order-bits-so-deep-integrity-runs-could-alias-their-own-evidence) | H | non-datapath-refactor | `n/a (promotion audit before the refreshed isolated reruns)` | fixed | `P028` / `P029` / `P036-P040` implementation review on 2026-04-20 while preparing the next PROF closure tranche | `1c8118b` | Long-run PROF generators reused overlapping low-order bits, so deep integrity runs could alias their own evidence |
| [BUG-034-H](#bug-034-h-the-seeded-random-prof-promotion-helpers-did-not-actually-propagate-distinct-per-case-seeds) | H | non-datapath-refactor | `n/a (promotion audit before the refreshed seeded reruns)` | fixed | `P036-P038` implementation review on 2026-04-20 while wiring the seeded four-key random tranche | `1c8118b` | The seeded random PROF promotion helpers did not actually propagate distinct per-case seeds |
| [BUG-035-H](#bug-035-h-the-promoted-prof-ordering-audit-encoded-a-stronger-fifo-contract-than-the-product-actually-guarantees) | H | non-datapath-refactor | `n/a (isolated-only repro; first mismatch at 1.117284 ms in pre-fix P029)` | fixed | `P029` on 2026-04-20 during the first promoted deterministic four-key interleave rerun | `1c8118b` | The promoted PROF ordering audit encoded a stronger FIFO contract than the product actually guarantees |
| [BUG-036-H](#bug-036-h-data-bearing-256-hit-packet-subheaders-were-misclassified-as-zero-hit-headers-when-the-8-bit-hit-count-field-wrapped) | H | non-datapath-refactor | `n/a (isolated-only repro; first mismatch at 7.483404 ms in pre-fix P025)` | fixed | `P025` on 2026-04-20 during the first calibrated steady-state fixed-point rerun | `ca1b044` | Data-bearing 256-hit packet subheaders were misclassified as zero-hit headers when the 8-bit hit-count field wrapped |
| [BUG-037-H](#bug-037-h-the-default-build-dv-plan-and-report-text-drifted-to-npartitions4-semantics-even-though-the-active-dashboard-build-is-defaultp2pipe4) | H | non-datapath-refactor | `n/a (promotion audit while wiring the partition-profile tranche on 2026-04-20)` | fixed | `B057`, `P046-P065`, and `CROSS-047` documentation review on 2026-04-20 before promoting the next PROF tranche | `ca1b044` | The default-build DV plan and report text drifted to `N_PARTITIONS=4` semantics even though the active dashboard build is `default_p2_pipe4` |
| [BUG-038-H](#bug-038-h-the-first-partition-profile-promotion-assumed-exact-partition-prefill-isolated-the-next-epoch-to-partition-1-but-the-live-contract-exposes-a-p0-p1-handoff-window-instead) | H | non-datapath-refactor | `n/a (isolated-only repro; first mismatch at 1.140772 ms in pre-fix P047)` | fixed | `P047` on 2026-04-20 during the first default-build partition-profile rerun | `ca1b044` | The first partition-profile promotion assumed exact-partition prefill isolated the next epoch to partition 1, but the live contract exposes a `p0->p1` handoff window instead |
| [BUG-039-R](#bug-039-r-the-pop-descriptor-generator-ignored-popcmdfifofull-and-could-overrun-the-16-entry-descriptor-fifo-under-sustained-backlog) | R | soft error | `n/a (directed-only low-latency descriptor-backpressure repro)` | fixed | `E124` on 2026-04-20 during the new fixed-point descriptor-backpressure implementation | `08ad68b` | The pop descriptor generator ignored `pop_cmd_fifo_full` and could overrun the 16-entry descriptor FIFO under sustained backlog |
| [BUG-040-R](#bug-040-r-csr-softreset-only-self-cleared-bit1-and-never-reset-the-live-dut-state-counters-or-fifos) | R | hard stuck error | `n/a (directed-only control-path repro)` | fixed | `X005` on 2026-04-20 during the active-traffic soft-reset closure run | `08ad68b` | CSR `soft_reset` only self-cleared bit1 and never reset the live DUT state, counters, or FIFOs |
| [BUG-041-R](#bug-041-r-the-pop-engine-could-still-consume-stale-descriptors-after-soft-reset-had-already-returned-the-dut-to-idle) | R | hard stuck error | `n/a (directed-only soft-reset mid-drain repro)` | fixed | `B124` on 2026-04-20 while closing the mid-drain soft-reset end-to-end case | `08ad68b` | The pop engine could still consume stale descriptors after soft-reset had already returned the DUT to `IDLE` |
| [BUG-042-R](#bug-042-r-the-push-path-could-retire-one-stale-buffered-push-or-overwrite-after-soft-reset-because-request-generation-was-not-gated-by-active-run-state) | R | soft error | `n/a (directed-only overwrite-bearing soft-reset repro)` | fixed | `X079` on 2026-04-20 after the initial soft-reset cleanup had already landed | `08ad68b` | The push path could retire one stale buffered push or overwrite after soft-reset because request generation was not gated by active run state |
| [BUG-043-H](#bug-043-h-the-first-p042-staged-late-arrival-helper-reused-overlapping-fingerprint-space-and-doubled-the-aggregate-ingress-rate-after-the-late-key-launch) | H | soft error | `1180084000 ns (first staged-helper repro in P042)` | fixed | `P042` on 2026-04-20 during the first live staged late-arrival PROF promotion | `97f956c` | The first `P042` staged late-arrival helper reused overlapping fingerprint space and doubled the aggregate ingress rate after the late-key launch |
| [BUG-044-H](#bug-044-h-backpressure-enabled-prof-cases-released-sinkready-as-soon-as-ingress-stopped-not-when-the-drain-window-finished) | H | non-datapath-refactor | `1104460000 ns (pre-fix P059 stall-window repro)` | fixed | `P059` on 2026-04-20 during the active-build backpressure / partition-saturation PROF tranche, then cross-checked against `P018`, `P019`, `P020`, `P060`, and `P064` | `8b9ad49` | Backpressure-enabled PROF cases released `sink_ready` as soon as ingress stopped, not when the drain window finished |
| [BUG-045-R](#bug-045-r-the-equal-load-partition-drain-scheduler-held-service-on-the-just-issued-partition-even-when-another-partition-was-already-pending) | R | soft error | `1115780000 ns (first B099 equal-load round-robin repro)` | fixed | `B099` on 2026-04-20 during the first exactly-two-partitions equal-load promotion, then cross-checked against `B080`, `B100-B103`, `P050-P053`, and `P058/P061-P063` | `7c2beb7` | The equal-load partition drain scheduler held service on the just-issued partition even when another partition was already pending |
| [BUG-046-H](#bug-046-h-clean-terminate-drain-only-waited-for-the-source-queue-not-for-the-internal-deassembly-fifo-and-could-wedge-heavy-overwrite-soaks) | H | signoff block | `3906956000 ns (first P004 terminate-timeout repro in the post-P005/P006 P001-P006 regression rerun)` | fixed | `P004` on 2026-04-20 during the first post-P005/P006 `P001-P006` regression slice, after `P005` and `P006` had already passed in isolation | `cb365cf` | Clean terminate/drain only waited for the source queue, not for the internal deassembly FIFO, and could wedge heavy overwrite soaks |
| [BUG-047-H](#bug-047-h-cache-miss-pop-observations-with-occupied0-did-not-retire-the-live-resident-in-the-scoreboard) | H | signoff block | `1690228000 ns (first B029 forced-cache-miss idle-timeout repro)` | fixed and verified in the stale-slot closure tranche on `dev` | `B029` on 2026-04-21 during the first stale-slot BASIC closure tranche, then cross-checked against `B067` | `2fd3115` | Cache-miss pop observations with `occupied=0` did not retire the live resident in the scoreboard |
| [BUG-048-H](#bug-048-h-cache-miss-output-beats-left-the-scoreboard-pending-drain-queue-uncleared) | H | signoff block | `1690228000 ns (second B029 stale-slot repro after resident-retire fix)` | fixed and verified in the stale-slot closure tranche on `dev` | `B029` on 2026-04-21 immediately after `BUG-047-H` had retired the resident correctly, then cross-checked against `B067` | `2fd3115` | Cache-miss output beats left the scoreboard pending-drain queue uncleared |
| [BUG-049-H](#bug-049-h-the-first-stale-slot-injector-forced-the-live-side-ram-read-bus-and-contaminated-later-reads-in-the-same-testcase) | H | signoff block | `1087260000 ns (first B030 mixed-counter precondition repro after the initial stale-slot helper landed)` | fixed and verified in the stale-slot helper closure tranche on `dev` | `B030` on 2026-04-21 during the first attempt to combine stale-slot cache-miss traffic with later overwrite pressure | `2fd3115` | The first stale-slot injector forced the live side-RAM read bus and contaminated later reads in the same testcase |
| [BUG-050-H](#bug-050-h-the-compat-scfifo-model-truncated-exact-full-usedw-and-flooded-long-regressions-with-false-warnings) | H | non-datapath-refactor | `n/a (simulator-compatibility warning audit during B072 on 2026-04-21)` | fixed and verified in isolated BASIC warning-floor refresh on `dev` | `B072` on 2026-04-21 while refreshing the BASIC warning floor for the current nightly checkpoint | `00fc1b8` | The compat `scfifo` model truncated exact-full `usedw` and flooded long regressions with false warnings |
| [BUG-051-R](#bug-051-r-the-memory-arbiter-could-still-grant-pushwrite-while-the-pop-engine-was-already-in-drain) | R | soft error | `n/a (directed-only DRAIN-overlap guard repro)` | fixed and verified in directed overlap guards on `dev` | `B130` on 2026-04-21 during the overwrite-closure directed guard bring-up | `00fc1b8` | The memory arbiter could still grant `push_write` while the pop engine was already in `DRAIN` |
| [BUG-052-R](#bug-052-r-the-memory-arbiter-could-still-grant-pushwrite-after-the-pop-snapshot-had-frozen-in-load-count) | R | soft error | `n/a (directed-only frozen-snapshot overlap repro)` | fixed and verified in directed frozen-snapshot guards on `dev` | `B131` on 2026-04-21 while closing the post-search overlap path after `BUG-051-R` | `00fc1b8` | The memory arbiter could still grant `push_write` after the pop snapshot had frozen in `LOAD/COUNT` |
| [BUG-053-R](#bug-053-r-ingress-ready-stayed-high-after-lane-local-end-of-run-in-terminating-so-accepted-beats-were-silently-dropped) | R | soft error | `n/a (randomized mid-run TERMINATING repro in pre-fix P066)` | fixed and verified in the report-seed terminate-path reruns on `dev` | `P066` on 2026-04-21 during the new mid-run terminate profile implementation, then locked down with `B132` | `00fc1b8` | Ingress `ready` stayed high after lane-local end-of-run in `TERMINATING`, so accepted beats were silently dropped |
| [BUG-054-R](#bug-054-r-the-push-engine-stopped-draining-already-buffered-deassembly-entries-after-lane-local-end-of-run-in-terminating) | R | soft error | `n/a (seed-1 mid-run TERMINATING repro in post-BUG-053 P066)` | fixed and verified in the report-seed terminate-path reruns on `dev` | `P066` on 2026-04-21 during the report-seed rerun after `BUG-053-R` had already clamped post-EOR ingress acceptance | `00fc1b8` | The push engine stopped draining already-buffered deassembly entries after lane-local end-of-run in `TERMINATING` |
| [BUG-055-R](#bug-055-r-cross-key-pushwrite-could-still-perturb-the-search-match-fabric-before-the-pop-snapshot-was-frozen) | R | soft error | `n/a (directed-only early-SEARCH cross-key guard repro)` | fixed and verified in the directed SEARCH-window guard slice on `dev` | `B133` on 2026-04-21 while extending the post-`BUG-051/052` overlap closure back into the unstable SEARCH window | `07c0dae` | Cross-key `push_write` could still perturb the SEARCH match fabric before the pop snapshot was frozen |
| [BUG-056-R](#bug-056-r-settled-search-tail-overlap-could-still-clobber-a-slot-already-captured-in-the-frozen-snapshot-at-the-live-write-pointer) | R | soft error | `n/a (reduced checkpoint overwrite-soak repro in pre-fix P125; failing intermediate log not retained as a published artifact)` | fixed and verified in the checkpoint overwrite-soak reruns on `dev` | `P125` on 2026-04-21 after the early SEARCH guard had already landed, while re-opening safe cross-key overlap in the settled SEARCH tail | `07c0dae` | Settled SEARCH-tail overlap could still clobber a slot already captured in the frozen snapshot at the live write pointer |
| [BUG-057-R](#bug-057-r-low-stage-partitioned-encoder-variants-indexed-pipevalid-beyond-the-active-datapath-width) | R | signoff block | `n/a (Questa elaboration warning screen on PIPE_STAGES=1/2 build-variant runs)` | fixed and verified in the low-stage build-axis reruns on `dev` | ad-hoc `PIPE_STAGES=1/2` variant screens on 2026-04-21 while closing the low-stage encoder build axis after `BUG-056-R` | `acb9230` | Low-stage partitioned-encoder variants indexed `pipe_valid` beyond the active datapath width |
| [BUG-058-R](#bug-058-r-non-power-of-two-ring-depths-let-the-live-write-pointer-escape-the-configured-ring-span) | R | hard stuck error | `2103700 ns (first N=768 active-build profile repro before the ring-depth wrap fix)` | fixed and verified in the non-power-of-two depth reruns on `dev` | `P050` with `RING_BUFFER_N_ENTRY=768` on 2026-04-21 during the non-power-of-two depth screen | `acb9230` | Non-power-of-two ring depths let the live write pointer escape the configured ring span |
| [BUG-059-R](#bug-059-r-wrap-overwrite-pusherase-could-erase-outside-the-configured-ring-span-on-non-power-of-two-builds) | R | signoff block | `2116020 ns (first B134 directed N=768 wrap-overwrite guard repro before the erase-address wrap fix)` | fixed and verified in the directed guard and overwrite-soak reruns on `dev` | `B134` with `RING_BUFFER_N_ENTRY=768` on 2026-04-21 during the post-review audit of the non-power-of-two overwrite path | `dab30da` | Wrap-overwrite `push_erase` could erase outside the configured ring span on non-power-of-two builds |
| [BUG-060-R](#bug-060-r-pusherase-recomputed-the-just-written-slot-inside-the-remaining-standalone-timing-critical-cam-erase-cone) | R | signoff block | `n/a (standalone Quartus signoff rerun on the tightened 137.5 MHz P4 build; no functional sim-time failure)` | fixed and verified in the standalone signoff rerun plus directed overwrite-path regressions on `dev` | `ring_buffer_cam_syn_p4` on 2026-04-21 while closing the remaining standalone timing blocker after `BUG-059-R` | `1736898` | `push_erase` recomputed the just-written slot inside the remaining standalone timing-critical CAM erase cone |
| [BUG-061-R](#bug-061-r-terminating-control-ready-could-acknowledge-before-lane-local-end-of-run-had-actually-closed-the-drain-contract) | R | hard stuck error | `n/a (directed-only terminate-ack guard repro; the failing B040 log was superseded by the verification rerun after the patch landed)` | fixed and verified in the directed terminate-control reruns on `dev` | `B040` on 2026-04-21 while reopening the live failing BASIC terminate-control slice after the full implemented isolated rerun | `b4e0daa` | `TERMINATING` control ready could acknowledge before lane-local end-of-run had actually closed the drain contract |
| [BUG-062-H](#bug-062-h-the-generic-idle-helper-treated-a-lone-pending-end-of-run-marker-as-live-traffic-even-after-the-dut-had-already-closed-the-lane) | H | non-datapath-refactor | `3493980 ns (seed-1 B071 terminate-condition audit before the lone-marker cleanup exception was added)` | fixed and verified in the terminate cleanup reruns on `dev` | `B071` on 2026-04-21 immediately after `BUG-061-R` was fixed and the terminate-condition audit was rerun against the updated RTL | `b4e0daa` | The generic idle helper treated a lone pending end-of-run marker as live traffic even after the DUT had already closed the lane |
| [BUG-063-H](#bug-063-h-the-shared-low-stage-partition-latency-helper-anchored-on-a-stage-4-only-pulse-instead-of-the-real-active-partition-load-event) | H | non-datapath-refactor | `n/a (directed-only low-stage build-axis latency reruns before the helper anchor was corrected)` | fixed and verified in the low-stage directed and profile reruns on `dev` | `B094`, `B095`, `B096`, `P054`, `P055`, and `P056` on 2026-04-21 while closing the PIPE_STAGES build axis with standalone per-variant evidence | `4c62cd0` | The shared low-stage partition-latency helper anchored on a stage-4-only pulse instead of the real active-partition load event |

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
- Commit: d7ce37a

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
- Commit: d7ce37a

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
- Commit: d7ce37a

### BUG-033-H: Long-run PROF generators reused overlapping low-order bits, so deep integrity runs could alias their own evidence
- Severity: `non-datapath-refactor`
- Encounter sim-time: `n/a (promotion audit before the refreshed isolated reruns)`
- First seen in: `P028` / `P029` / `P036-P040` implementation review on 2026-04-20 while preparing the next PROF closure tranche
- Symptom:
  - the long-run profile sequences derived `asic`, `channel`, `ts50p`, and `et1n6` from overlapping low-order slices of the same loop index
  - deep runs would therefore recycle the same scoreboard fingerprint long before the case completed, weakening the ability of the promoted integrity cases to prove long-span no-loss behavior honestly
- Root cause:
  - the sequence helpers hand-packed multiple fingerprint fields from the same low bits instead of assigning non-overlapping bit ranges across the long-run index
  - this was a testcase/evidence-quality bug, not a DUT datapath corruption
- Fix status: fixed and verified
- Runtime / coverage context:
  - added `fill_long_run_fingerprint()` in `ring_buffer_cam_pkg.sv` and switched the long-run profile generators to that helper so the fingerprint fields use non-overlapping slices of the long-run index
  - verified by a clean isolated rerun of `P028` plus the later clean closure batch `B010`, `B011`, `B123`, `P029`, `P036`, `P037`, `P038`, and `P040`
- Commit: 1c8118b

### BUG-034-H: The seeded random PROF promotion helpers did not actually propagate distinct per-case seeds
- Severity: `non-datapath-refactor`
- Encounter sim-time: `n/a (promotion audit before the refreshed seeded reruns)`
- First seen in: `P036-P038` implementation review on 2026-04-20 while wiring the seeded four-key random tranche
- Symptom:
  - the new seeded cases were documented as orthogonal evidence runs, but the helper path had no reliable end-to-end seed handoff from the case declaration into `profile_traffic_seq`
  - without a seed-carrying helper interface, the published seeded tranche risked collapsing into duplicated evidence under cosmetic case IDs
- Root cause:
  - the multi-key profile helper contract was still shaped around the earlier non-seeded callers and had not been promoted into an explicit seed-bearing API for the PROF closure tranche
  - that made the intended `P036/P037/P038` seed axis under-specified in the harness implementation
- Fix status: fixed and verified
- Runtime / coverage context:
  - `run_multi_key_profile_case()` now carries the case-local `lfsr_seed` explicitly into `profile_traffic_seq`, and the seeded profile summaries publish the exact seed used for each run
  - verified by clean isolated reruns of `P036`, `P037`, `P038`, and `P040`
- Commit: 1c8118b

### BUG-035-H: The promoted PROF ordering audit encoded a stronger FIFO contract than the product actually guarantees
- Severity: `non-datapath-refactor`
- Encounter sim-time: `n/a (isolated-only repro; first mismatch at 1.117284 ms in pre-fix P029)`
- First seen in: `P029` on 2026-04-20 during the first promoted deterministic four-key interleave rerun
- Symptom:
  - the first `P029` promotion attempt produced `19,436` strict FIFO mismatch errors even though the DUT still closed the core accounting contract cleanly with `push=20000`, `pop=20000`, `overwrite=0`, `cache_miss=0`, and `remaining=0`
  - the same failure mode would have poisoned `P040`, turning a testcase/reporting assumption into a false DUT regression
- Root cause:
  - the new scoreboard path compared drained hits against accepted-ingress order and treated any same-key reordering as illegal
  - the live product contract for `ring_buffer_cam` guarantees timestamp-bucketed drain integrity and explicit loss accounting, not stable same-key FIFO retirement order at the egress beat level
- Fix status: fixed and verified
- Runtime / coverage context:
  - removed the unsupported strict-FIFO checker path, rewrote `P029` around deterministic exact per-key integrity, and rewrote `P040` as a seeded four-key integrity audit instead of a false FIFO claim
  - verified by a clean isolated rerun batch of `B010`, `B011`, `B123`, `P029`, `P036`, `P037`, `P038`, and `P040`
- Commit: 1c8118b

### BUG-036-H: Data-bearing 256-hit packet subheaders were misclassified as zero-hit headers when the 8-bit hit-count field wrapped
- Severity: `non-datapath-refactor`
- Encounter sim-time: `n/a (isolated-only repro; first mismatch at 7.483404 ms in pre-fix P025)`
- First seen in: `P025` on 2026-04-20 during the first calibrated steady-state fixed-point rerun
- Symptom:
  - `P025` failed with `observed=2 expected_at_least=32` data-bearing subheaders even though the same run's scoreboard summary still closed at `push=50000`, `pop=50000`, `overwrite=0`, `cache_miss=0`, and reported `197` non-empty subheaders overall
  - the failure would have undercounted packetized long drains anywhere a data-bearing subheader legitimately encoded `hit_count=0x00` because the true chunk size was `256`
- Root cause:
  - `out_monitor` and `scoreboard` both treated "data-bearing subheader" as `hit_count != 0`
  - for a packetized `256`-hit chunk the DUT still emits a data-bearing subheader, but the 8-bit count field wraps to `0` while `eop=0` proves the epoch continues with payload beats
- Fix status: fixed and verified
- Runtime / coverage context:
  - added a shared wrapped-count-safe classification rule in `out_monitor.sv` and `scoreboard.sv`: a subheader is data-bearing when `hit_count != 0` or `eop == 0`
  - verified by clean isolated reruns of `P025`, `P031`, `P046`, and `P047`; `P025` now publishes `data_subheaders=197` with zero UVM errors
- Commit: ca1b044

### BUG-037-H: The default-build DV plan and report text drifted to `N_PARTITIONS=4` semantics even though the active dashboard build is `default_p2_pipe4`
- Severity: `non-datapath-refactor`
- Encounter sim-time: `n/a (promotion audit while wiring the partition-profile tranche on 2026-04-20)`
- First seen in: `B057`, `P046-P065`, and `CROSS-047` documentation review on 2026-04-20 before promoting the next PROF tranche
- Symptom:
  - the live dashboard still described several default-build partition cases as if they were 4-partition runs
  - stale examples included `B057` claiming four `pop_partition_load()` pulses by default and `P047` / `CROSS-047` being written as a partition-1-only saturation case even though the active evidence build only has two live partitions
- Root cause:
  - the docs/report text had not been normalized after the dashboard standard settled on `default_p2_pipe4`
  - that left the published plan and cross-run descriptions stronger and more specific than the active build could honestly support
- Fix status: fixed and verified
- Runtime / coverage context:
  - updated `DV_BASIC.md`, `DV_PROF.md`, `DV_CROSS.md`, and the generated dashboard sources so the default build is explicitly treated as a 2-partition active variant and the `p4`-only rows remain marked as variant-only work
  - verified by the refreshed isolated reruns of `B010`, `B011`, `P025`, `P031`, `P046`, and `P047`, then by regenerating the dashboard from the corrected plan text
- Commit: ca1b044

### BUG-038-H: The first partition-profile promotion assumed exact-partition prefill isolated the next epoch to partition 1, but the live contract exposes a `p0->p1` handoff window instead
- Severity: `non-datapath-refactor`
- Encounter sim-time: `n/a (isolated-only repro; first mismatch at 1.140772 ms in pre-fix P047)`
- First seen in: `P047` on 2026-04-20 during the first default-build partition-profile rerun
- Symptom:
  - the first `P047` attempt failed with `observed=0x3 expected=0x2` for the partition issue mask even though accounting still closed exactly with `push=448`, `pop=448`, `overwrite=0`, `cache_miss=0`, and `remaining=0`
  - the failure would have published a false DUT regression by asserting that the target epoch stayed on partition 1 only after an exact-partition prefill
- Root cause:
  - the testcase/spec assumption was wrong, not the DUT
  - after a fully consumed first-partition prefill, the next target epoch can legitimately traverse the boundary and issue across both live partitions in the `default_p2_pipe4` build
- Fix status: fixed and verified
- Runtime / coverage context:
  - rewrote `P047` and its downstream report text as a partition-handoff profile with expected mask `0x3` instead of a partition-1-only saturation claim
  - verified by a clean isolated rerun of `P047`, then by the refreshed `P046/P047` publication batch with zero UVM errors
- Commit: ca1b044

### BUG-039-R: The pop descriptor generator ignored `pop_cmd_fifo_full` and could overrun the 16-entry descriptor FIFO under sustained backlog
- Severity: `soft error`
- Encounter sim-time: `n/a (directed-only low-latency descriptor-backpressure repro)`
- First seen in: `E124` on 2026-04-20 during the new fixed-point descriptor-backpressure implementation
- Symptom:
  - the low-latency descriptor stress could keep asserting `pop_cmd_fifo_wrreq` after the FIFO had already entered the saturation band
  - under that condition the DUT risked silently outrunning the 16-entry descriptor queue and losing timestamp-search work even though the steady-state ingress accounting still looked legal at the front door
- Root cause:
  - `proc_pop_descriptor_generator` gated descriptor issuance on run state, interleaving, and terminate timing, but never masked the write request with `pop_cmd_fifo_full`
  - once drain latency lagged the 16-cycle descriptor cadence, the generator could continue enqueue attempts past the physical FIFO capacity
- Fix status: fixed and verified
- Runtime / coverage context:
  - added explicit `pop_cmd_fifo_full /= '1'` gating in both `RUNNING` and `TERMINATING`, then verified with clean isolated reruns of `E019`, `E020`, `E124`, and `P025`
  - the `E124` monitor now observes real FIFO saturation while keeping outstanding descriptors bounded to the legal 16-entry depth
- Commit: 08ad68b

### BUG-040-R: CSR `soft_reset` only self-cleared bit1 and never reset the live DUT state, counters, or FIFOs
- Severity: `hard stuck error`
- Encounter sim-time: `n/a (directed-only control-path repro)`
- First seen in: `X005` on 2026-04-20 during the active-traffic soft-reset closure run
- Symptom:
  - a host `CTRL.soft_reset` write acknowledged and read back as self-clearing, but the DUT stayed live with non-zero counters, resident fill, and active push/pop state
  - that violated the product reset contract and left the testcase unable to rely on soft-reset as a clean abort-to-IDLE boundary
- Root cause:
  - `proc_avmm_csr` only cleared `csr.soft_reset` itself; the rest of the sequential state owners still ignored that pulse
  - run control, counters, fill-level accounting, push/pop engines, FIFOs, and the memory cleanup paths therefore kept executing their pre-reset state as if no reset had been requested
- Fix status: fixed and verified
- Runtime / coverage context:
  - added explicit `csr.soft_reset` handling across the live state owners so the DUT returns to `IDLE`, clears counters/fill/FIFOs, and resets the internal push/pop bookkeeping
  - verified by clean isolated reruns of `X005`, `X035`, `X036`, `X071`, `X072`, `X073`, `X074`, `X075`, `X076`, `X077`, `X078`, `X079`, `X081`, `X085`, `X113`, and `B124`
- Commit: 08ad68b

### BUG-041-R: The pop engine could still consume stale descriptors after soft-reset had already returned the DUT to `IDLE`
- Severity: `hard stuck error`
- Encounter sim-time: `n/a (directed-only soft-reset mid-drain repro)`
- First seen in: `B124` on 2026-04-20 while closing the mid-drain soft-reset end-to-end case
- Symptom:
  - after soft-reset had already cleared the visible run state and counters, the pop engine could still wake back up from `IDLE` and continue working a stale descriptor
  - that left post-reset drain activity observable after the testcase had already asked for a clean abort boundary
- Root cause:
  - the `proc_pop_engine` `IDLE` path still consumed `pop_cmd_fifo_rdack` whenever the descriptor FIFO was non-empty, regardless of whether the DUT was still in an active `RUNNING` / `TERMINATING` state
  - clearing run control alone was therefore insufficient if a stale descriptor remained queued at the soft-reset boundary
- Fix status: fixed and verified
- Runtime / coverage context:
  - restricted `IDLE` descriptor consumption to active `RUNNING` / `TERMINATING` states, then verified with clean isolated reruns of `B124` and `X005`
  - the later full soft-reset rerun slice stayed clean with no residual pop-engine activity after reset
- Commit: 08ad68b

### BUG-042-R: The push path could retire one stale buffered push or overwrite after soft-reset because request generation was not gated by active run state
- Severity: `soft error`
- Encounter sim-time: `n/a (directed-only overwrite-bearing soft-reset repro)`
- First seen in: `X079` on 2026-04-20 after the initial soft-reset cleanup had already landed
- Symptom:
  - a same-key overwrite-bearing testcase could still observe non-zero post-reset push/overwrite state even after the main soft-reset handlers had cleared the visible counters and run state
  - that meant one buffered push-side transaction could slip through the reset boundary and corrupt the supposedly clean restart state
- Root cause:
  - `proc_push_engine_comb` still generated `push_write_req` / `push_erase_req` from buffered ingress state without checking that the DUT remained in an active run state after soft-reset
  - the buffered request therefore survived long enough to retire once arbitration reopened, even though the host had already requested a local reset
- Fix status: fixed and verified
- Runtime / coverage context:
  - gated push request generation on both `csr.soft_reset /= '1'` and an active run-state condition, then verified with clean isolated reruns of `X077`, `X079`, and `X081`
  - the later full soft-reset rerun slice stayed clean with zero post-reset push/overwrite accounting
- Commit: 08ad68b

### BUG-043-H: The first `P042` staged late-arrival helper reused overlapping fingerprint space and doubled the aggregate ingress rate after the late-key launch
- Severity: `soft error`
- Encounter sim-time: `1180084000 ns (first staged-helper repro in P042)`
- First seen in: `P042` on 2026-04-20 during the first live staged late-arrival PROF promotion
- Symptom:
  - the first `P042` implementation produced scoreboard `Unexpected drained hit` failures even though the DUT counters remained in a nominal no-overwrite regime
  - after the initial identity-space fix, the case still over-drove the DUT relative to the planned `gap=13` contract because the late phase injected a second full-rate source on top of the early source
- Root cause:
  - the helper modeled the early and late populations with two independent `profile_traffic_seq` instances launched against the same hit sequencer, each starting its `fill_long_run_fingerprint()` stream at index `0`
  - that created two harness defects: the scoreboard identity tuple aliased across the phases, and the combined sequencer traffic silently halved the effective aggregate inter-hit gap after the late phase started
- Fix status: fixed and verified
- Runtime / coverage context:
  - added an explicit `fingerprint_start_index` offset to the long-run profile generators, then rewrote `P042` as a calibrated two-phase profile: early keys only, then a combined eight-key phase at the original aggregate rate
  - verified by a clean isolated rerun of `P042` and a clean `P041-P045` batch rerun on `default_p2_pipe4`
- Commit: 97f956c

### BUG-044-H: Backpressure-enabled PROF cases released `sink_ready` as soon as ingress stopped, not when the drain window finished
- Severity: `non-datapath-refactor`
- Encounter sim-time: `1104460000 ns (pre-fix P059 stall-window repro)`
- First seen in: `P059` on 2026-04-20 during the active-build backpressure / partition-saturation PROF tranche, then cross-checked against `P018`, `P019`, `P020`, `P060`, and `P064`
- Symptom:
  - the first `P059` implementation claimed a 500-cycle sink stall and clean release, but the summary only observed `sink_low_cycles=256` even though the case still had residual drain work
  - more generally, any `sink_ready_mode != 0` profile could stop applying backpressure the instant source injection ended, so the remaining drain window ran at forced-ready and under-stressed the advertised testcase contract
- Root cause:
  - the `sink_ready_thread` loop in `run_profile_integrity_case()` only tracked `!traffic_done || pending_source_items != 0`
  - it omitted the scoreboard residual (`m_env.m_scb.remaining_entries()`), so the helper exited early and restored `sink_ready=1` before the DUT had actually drained the staged traffic
- Fix status: fixed and verified
- Runtime / coverage context:
  - the helper now keeps sink-ready modulation active until source injection and modeled drain both quiesce
  - verified by clean isolated reruns of `P018`, `P019`, `P020`, `P059`, `P060`, and `P064`
  - post-fix evidence:
    - `P059` now reports `sink_low_cycles=500`, `max_live_fill=234`, `observed_partition_mask=0x1`, `push=256`, `pop=256`
    - `P060` now reports `sink_low_cycles=14428`, `data_subheaders=6`, `observed_partition_mask=0x3`, `push=768`, `pop=768`
    - `P064` now reports `sink_low_cycles=9408`, `data_subheaders=6`, `observed_partition_mask=0x3`, `push=1024`, `pop=1024`
- Commit: 8b9ad49

### BUG-045-R: The equal-load partition drain scheduler held service on the just-issued partition even when another partition was already pending
- Severity: `soft error`
- Encounter sim-time: `1115780000 ns (first B099 equal-load round-robin repro)`
- First seen in: `B099` on 2026-04-20 during the first exactly-two-partitions equal-load promotion, then cross-checked against `B080`, `B100-B103`, `P050-P053`, and `P058/P061-P063`
- Symptom:
  - the new equal-load partition audit observed repeated grants from the same partition instead of the planned `0,1,0,1,0,1,0,1` alternation, even though the run still closed cleanly at `push=260`, `pop=260`, `overwrite=0`, `cache_miss=0`, and `remaining=0`
  - that meant the full-DUT drain path could starve an equally loaded pending peer until the current partition had already consumed multiple hits, violating the intended round-robin fairness contract
- Root cause:
  - in `proc_pop_engine`, the `DRAIN` state kept `pop_rr_idx` parked on `pop_issue_partition_idx` whenever `pop_partition_has_more(...)='1'`
  - the scheduler therefore re-entered the same partition after every successful erase, even when another partition was already marked pending and ready to issue
- Fix status: fixed and verified
- Runtime / coverage context:
  - the pop scheduler now advances to the next pending partition when the just-issued partition still has more matches and a peer partition is already pending, while preserving the old stay-local behavior when the current partition is the only pending source
  - verified by clean isolated reruns of `B080`, `B099`, `B100`, `B101`, `B102`, `B103`, `P050`, `P051`, `P052`, `P053`, `P058`, `P061`, `P062`, and `P063`
- Commit: 7c2beb7

### BUG-046-H: Clean terminate/drain only waited for the source queue, not for the internal deassembly FIFO, and could wedge heavy overwrite soaks
- Severity: `signoff block`
- Encounter sim-time: `3906956000 ns (first P004 terminate-timeout repro in the post-P005/P006 P001-P006 regression rerun)`
- First seen in: `P004` on 2026-04-20 during the first post-P005/P006 `P001-P006` regression slice, after `P005` and `P006` had already passed in isolation
- Symptom:
  - the overwrite soak reached a clean scoreboard end state (`remaining=0`, `pending_drain=0`, `epoch_idle=1`) but `terminating_drain_done` never asserted, so the helper timed out waiting for a terminate acknowledgement
  - debug at timeout showed `accepted=4096` while only `push=3842`, with `deassembly_fifo_empty=0`, `deassembly_fifo_usedw=254`, and `endofrun_seen=1`, meaning accepted hits were stranded in the DUT's local deassembly buffer behind the end-of-run gate
- Root cause:
  - `terminate_and_drain()` only waited for `pending_source_items()==0` before issuing `CTRL_TERMINATING` and the end-of-run marker
  - under deep overwrite pressure, accepted hits can still be buffered in the DUT's internal `deassembly_fifo` after the source queue is empty; once `endofrun_seen=1`, the push path stops draining that fifo, so the helper could wedge `terminating_drain_done` behind self-inflicted residual buffered traffic
- Fix status: fixed and verified
- Runtime / coverage context:
  - the clean terminate helper now waits for both the source queue and the DUT-visible `deassembly_fifo_empty` condition before pulsing `CTRL_TERMINATING`, then gives the push path a short settle window before sending the end-of-run marker
  - verified by clean isolated reruns of `P004`, `P005`, and `P006`, followed by a clean `P001-P006` regression slice on `default_p2_pipe4`
- Commit: cb365cf

## 2026-04-21

### BUG-047-H: Cache-miss pop observations with `occupied=0` did not retire the live resident in the scoreboard
- Severity: `signoff block`
- Encounter sim-time: `1690228000 ns (first B029 forced-cache-miss idle-timeout repro)`
- First seen in: `B029` on 2026-04-21 during the first stale-slot BASIC closure tranche, then cross-checked against `B067`
- Symptom:
  - the DUT drained the stale-slot beat and raised `CACHE_MISS_COUNT`, but `wait_for_scoreboard_idle()` still timed out with `remaining=1` / `pending_drain=0`
  - end-of-test extraction then reported an undrained resident even though the output monitor had already seen the cache-miss-tagged hit beat
- Root cause:
  - `write_pop()` returned immediately whenever the debug pop item reported `occupied=0`
  - in the stale-slot contract that exact `occupied=0` observation is still a real resident retirement, because the pop-side raw payload can still identify the written slot that is being drained as a cache miss
- Fix status:
  - `state`: fixed and verified in the stale-slot closure tranche on `dev`
  - `mechanism`: `write_pop()` now treats `occupied=0` cache-miss pops as real resident retirements by recovering the live fingerprint from the raw pop payload instead of returning early
  - `before_fix_outcome`: `B029` / `B067` timed out in `wait_for_scoreboard_idle()` with `remaining=1` even though the DUT had already emitted the cache-miss-tagged hit beat
  - `after_fix_outcome`: `B029`, `B067`, and the mixed FLUSH counter-clear case `B030` now retire the stale-slot resident cleanly with no residual resident drift
  - `potential_hazard`: low for the current stale-slot/cache-miss slice; this path is now anchored by multiple directed cases but has not yet been stress-screened in a broader random stale-slot campaign
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - the scoreboard now recovers the matching live fingerprint from the raw pop payload even when `occupied=0`, retires the resident immediately, and leaves the later output beat to classify the drain as `cache_miss`
  - verified by clean isolated reruns of `B029`, `B067`, and the mixed FLUSH counter-clear case `B030`
- Commit: 2fd3115

### BUG-048-H: Cache-miss output beats left the scoreboard pending-drain queue uncleared
- Severity: `signoff block`
- Encounter sim-time: `1690228000 ns (second B029 stale-slot repro after resident-retire fix)`
- First seen in: `B029` on 2026-04-21 immediately after `BUG-047-H` had retired the resident correctly, then cross-checked against `B067`
- Symptom:
  - the resident model reached `remaining=0`, but `wait_for_scoreboard_idle()` still timed out with `pending_drain=1`
  - end-of-test extraction then reported a non-empty pending-drain queue even though the matching cache-miss output beat had already been observed
- Root cause:
  - `write_out()` treated `item.cache_miss` as a special classification path, but it never deleted the corresponding fingerprint from `pending_drain_q`
  - the stale-slot epoch therefore remained artificially non-quiescent forever, even after the visible output beat had closed the packet
- Fix status:
  - `state`: fixed and verified in the stale-slot closure tranche on `dev`
  - `mechanism`: `write_out()` now consumes the matching `pending_drain_q` fingerprint on cache-miss output beats before final packet-accounting checks run
  - `before_fix_outcome`: after `BUG-047-H` retired the resident, `B029` still timed out with `remaining=0` but `pending_drain=1`, leaving the stale-slot epoch permanently non-quiescent
  - `after_fix_outcome`: `B029`, `B067`, and `B030` now close both resident and pending-drain accounting cleanly when the cache-miss output beat arrives
  - `potential_hazard`: low for the current stale-slot/accounting slice; the fix is local to scoreboard pending-drain retirement and did not reopen the overwrite-path control case `B027`
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - the cache-miss output path now consumes the matching pending-drain fingerprint before final packet-accounting checks run
  - verified by clean isolated reruns of `B029`, `B067`, and the mixed FLUSH counter-clear case `B030`
- Commit: 2fd3115

### BUG-049-H: The first stale-slot injector forced the live side-RAM read bus and contaminated later reads in the same testcase
- Severity: `signoff block`
- Encounter sim-time: `1087260000 ns (first B030 mixed-counter precondition repro after the initial stale-slot helper landed)`
- First seen in: `B030` on 2026-04-21 during the first attempt to combine stale-slot cache-miss traffic with later overwrite pressure
- Symptom:
  - the mixed counter-precondition case unexpectedly observed `push=514 pop=513 overwrite=0 cache_miss=513`
  - scoreboard debug showed every later pop across the full ring being classified as `occupied=0`, proving the original one-shot stale-slot force had leaked far beyond the targeted resident
- Root cause:
  - the first helper overrode `dut.v2_core.side_ram_dout[39]` directly with `force/release`
  - that approach perturbed the live read bus instead of editing one stored side-RAM word, so later pop-side reads in the same testcase could inherit the stale forced occupancy value and collapse both overwrite detection and cache-miss attribution
- Fix status:
  - `state`: fixed and verified in the stale-slot helper closure tranche on `dev`
  - `mechanism`: replaced the live-bus `force/release` hack with a dedicated debug patch lane in `alt_simple_dpram` / `ring_buffer_cam_v2_core`, then rewrote the BASIC helper to patch exactly one stored side-RAM word for one clock
  - `before_fix_outcome`: `B030` polluted later reads across the ring and collapsed into `push=514 pop=513 overwrite=0 cache_miss=513`, proving the helper was perturbing the live side-RAM read bus rather than one target resident
  - `after_fix_outcome`: `B029`, `B030`, and `B067` now exercise one stale-slot resident without contaminating later overwrite or cache-miss attribution, while `B027` stays clean as the overwrite control
  - `potential_hazard`: low for the current directed helper flow; the patch lane is debug-only, but any future stale-slot helper changes still need to avoid touching the live read bus
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - replaced the live-bus force with a dedicated debug patch lane in `alt_simple_dpram` / `ring_buffer_cam_v2_core`, then rewrote the BASIC stale-slot helpers to patch exactly one written side-RAM entry for one clock
  - verified by clean isolated reruns of `B029`, `B030`, and `B067`, with `B027` remaining clean as the overwrite-path control
- Commit: 2fd3115

### BUG-050-H: The compat `scfifo` model truncated exact-full `usedw` and flooded long regressions with false warnings
- Severity: `non-datapath-refactor`
- Encounter sim-time: `n/a (simulator-compatibility warning audit during B072 on 2026-04-21)`
- First seen in: `B072` on 2026-04-21 while refreshing the BASIC warning floor for the current nightly checkpoint
- Symptom:
  - exact-full FIFO occupancy produced repeated `NUMERIC_STD.TO_UNSIGNED: vector truncated` style warnings from the compat `usedw` path
  - the warning burst obscured real UVM failures and made the nightly log count look unhealthy even when the DUT behavior was correct
- Root cause:
  - the compat model encoded `count` directly into a `usedw` vector whose width is only `log2(depth)`
  - Intel's generated wrappers use that narrow debug width, so the exact-full count is not representable and must saturate rather than convert lossily
- Fix status:
  - `state`: fixed and verified in isolated BASIC warning-floor refresh on `dev`
  - `mechanism`: added a saturating `encode_usedw()` helper to `tb/sim/compat/scfifo.vhd` and made the compat source a real Makefile dependency so exact-full occupancy rebuilds deterministically
  - `before_fix_outcome`: exact-full FIFO occupancy flooded `B072` and other long regressions with repeated `NUMERIC_STD.TO_UNSIGNED: vector truncated` style warnings, obscuring real failures
  - `after_fix_outcome`: `B072` reruns now keep only the remaining benign startup/tool warnings instead of the previous exact-full `usedw` flood
  - `potential_hazard`: low; this is a simulator-compatibility repair, but the compat helper must continue matching Intel's exact-full saturation semantics rather than reintroducing lossy narrow-width conversion
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - added a saturating `encode_usedw()` helper to `tb/sim/compat/scfifo.vhd` and made the compat source a real Makefile dependency so reruns rebuild it deterministically
  - verified by a clean isolated rerun of `B072`; warning count dropped from the previous full-occupancy flood to the remaining benign startup/tool warnings only
- Commit: 00fc1b8

### BUG-051-R: The memory arbiter could still grant `push_write` while the pop engine was already in `DRAIN`
- Severity: `soft error`
- Encounter sim-time: `n/a (directed-only DRAIN-overlap guard repro)`
- First seen in: `B130` on 2026-04-21 during the overwrite-closure directed guard bring-up
- Symptom:
  - a late push could still win arbitration in a `DRAIN` bubble even though the pop engine already owned a frozen resident snapshot
  - that risked mutating live RAM/CAM state underneath an active drain epoch and invalidating the scoreboard's packet-level retirement model
- Root cause:
  - `proc_mem_arbitor_comb` still treated every non-IDLE pop state as eligible for push/pop overlap
  - once the engine had entered `DRAIN`, that overlap policy was no longer legal because the frozen pop snapshot had to retire without further push-side mutation
- Fix status:
  - `state`: fixed and verified in directed overlap guards on `dev`
  - `mechanism`: `proc_mem_arbitor_comb` no longer allows push/pop overlap once the pop engine has reached `DRAIN`; overlap is now restricted to the pre-freeze search window
  - `before_fix_outcome`: the new `B130` guard showed that a late push could still win arbitration in a `DRAIN` bubble and mutate RAM/CAM state underneath an active frozen drain epoch
  - `after_fix_outcome`: `B130` now stays clean, and the follow-on guards `B131` plus the skewed overwrite soak `P126` still pass with the overlap restriction in place
  - `potential_hazard`: low to moderate; the repair is guarded by dedicated directed cases, but broad continuous-frame overlap closure is still open because the all-bucket signoff matrix has not been completed yet
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - restricted overlap to the pre-freeze search window and added the dedicated `B130` guard so any future `push_write` in `DRAIN` fails immediately
  - verified by clean isolated reruns of `B130`, `B131`, and the skewed overwrite soak `P126`
- Commit: 00fc1b8

### BUG-052-R: The memory arbiter could still grant `push_write` after the pop snapshot had frozen in `LOAD/COUNT`
- Severity: `soft error`
- Encounter sim-time: `n/a (directed-only frozen-snapshot overlap repro)`
- First seen in: `B131` on 2026-04-21 while closing the post-search overlap path after `BUG-051-R`
- Symptom:
  - the DRAIN guard removed the latest overlap window, but the arbiter could still admit a `push_write` during `LOAD` or `COUNT`
  - that meant the issue mask and exact-hit count could be computed from one resident set and then drained against a different one
- Root cause:
  - overlap had been left enabled for all non-IDLE pop states instead of only the final pre-freeze `SEARCH` phase
  - the pop-side snapshot therefore was not protected once the engine had already started loading/counting the matched partitions
- Fix status:
  - `state`: fixed and verified in directed frozen-snapshot guards on `dev`
  - `mechanism`: the arbiter now allows push/pop overlap only during `SEARCH`; `LOAD`, `COUNT`, and `DRAIN` preserve the frozen pop snapshot until retirement completes
  - `before_fix_outcome`: after the first `DRAIN` fix, `B131` still showed `push_write` could slip into `LOAD` or `COUNT`, meaning the issue mask and exact-hit count could be computed from one resident set and drained against another
  - `after_fix_outcome`: `B131` is now clean and `P126` still closes with explicit overwrite accounting (`push=2500`, `pop=1693`, `overwrite=807`, `remaining=0`)
  - `potential_hazard`: low to moderate; the frozen-snapshot contract now has a direct guard, but wider mixed random space still depends on future DV-plan closure rather than this one directed slice alone
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - the arbiter now allows overlap only during `SEARCH`, while `LOAD`, `COUNT`, and `DRAIN` preserve the frozen snapshot until retirement completes
  - verified by clean isolated reruns of `B131` and the reduced checkpoint overwrite soak `P126`, which still closes at `push=2500`, `pop=1693`, `overwrite=807`, `remaining=0`
- Commit: 00fc1b8

### BUG-053-R: Ingress `ready` stayed high after lane-local end-of-run in `TERMINATING`, so accepted beats were silently dropped
- Severity: `soft error`
- Encounter sim-time: `n/a (randomized mid-run TERMINATING repro in pre-fix P066)`
- First seen in: `P066` on 2026-04-21 during the new mid-run terminate profile implementation, then locked down with `B132`
- Symptom:
  - after `endofrun_seen` latched, the source still observed `ready=1` and continued handshaking payload beats
  - `accepted_payload_total` kept advancing, but the DUT had already stopped writing those beats into the deassembly path, leaving `PUSH_COUNT` permanently below the source-side accepted total
- Root cause:
  - `deassembly_fifo_wrreq` correctly stopped once the DUT entered `TERMINATING` with `endofrun_seen=1`, but `asi_hit_type1_ready` still depended only on FIFO fullness
  - the DUT therefore advertised acceptance after end-of-run even though the local write path had already closed
- Fix status:
  - `state`: fixed and verified in the report-seed terminate-path reruns on `dev`
  - `mechanism`: `asi_hit_type1_ready` now requires `csr.go=1`, no soft-reset, and an active `RUNNING` or pre-EOR `TERMINATING` state instead of depending only on FIFO fullness
  - `before_fix_outcome`: pre-fix `P066` kept advancing `accepted_payload_total` after lane-local end-of-run while `PUSH_COUNT` stopped, proving the DUT still advertised acceptance after the local write path had already closed
  - `after_fix_outcome`: `P066`, `B132`, `B131`, and `P126` now keep `ready` and `dbg_push_cnt` flat after lane-local end-of-run, eliminating accepted-but-dropped post-EOR beats
  - `potential_hazard`: low to moderate; the ingress contract is now guarded directly by `B132`, but terminate behavior still relies on the broader drain path staying aligned with the buffered-local-work contract fixed by `BUG-054-R`
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - `asi_hit_type1_ready` now also requires `csr.go=1`, no soft-reset, and an active `RUNNING` / pre-EOR `TERMINATING` state; the new `B132` guard checks that `ready` and `dbg_push_cnt` stay flat after lane-local end-of-run
  - verified by clean isolated reruns of `P066`, `B132`, `B131`, and `P126`
- Commit: 00fc1b8

### BUG-054-R: The push engine stopped draining already-buffered deassembly entries after lane-local end-of-run in `TERMINATING`
- Severity: `soft error`
- Encounter sim-time: `n/a (seed-1 mid-run TERMINATING repro in post-BUG-053 P066)`
- First seen in: `P066` on 2026-04-21 during the report-seed rerun after `BUG-053-R` had already clamped post-EOR ingress acceptance
- Symptom:
  - seed-1 `P066` no longer accepted new post-EOR beats, but `terminating_drain_done` still never asserted because two pre-EOR entries stayed stranded forever in the local deassembly FIFO
  - the DUT sat in `TERMINATING` with `deassm_usedw=2`, `push=83`, `pop=83`, `accepted=85`, and a permanently backpressured source, proving the remaining buffered work was not being retired
- Root cause:
  - `proc_push_engine_comb` still blocked both `push_write_req` and `push_erase_req` once `run_state_cmd=TERMINATING` and `endofrun_seen=1`
  - that was too aggressive: new ingress had to stop, but already-buffered deassembly entries and their follow-on erase work still had to drain to make `terminating_drain_done` reachable
- Fix status:
  - `state`: fixed and verified in the report-seed terminate-path reruns on `dev`
  - `mechanism`: `proc_push_engine_comb` now keeps servicing already-buffered deassembly payload and follow-on erase work during `TERMINATING`, while leaving ingress `wrreq` / `ready` clamped after lane-local end-of-run
  - `before_fix_outcome`: after `BUG-053-R`, seed-1 `P066` still wedged in `TERMINATING` with `deassm_usedw=2`, `push=83`, `pop=83`, and `accepted=85`, proving two pre-EOR entries were stranded in the local FIFO forever
  - `after_fix_outcome`: seed-1 `P066` now closes cleanly at `accepted=85`, `push=85`, `pop=85`, `remaining=0`, and the directed terminate guard `B132` plus the checkpoint overwrite soak `P126` remain green
  - `potential_hazard`: low to moderate; the buffered-local-work drain rule is now covered by the report-seed terminate repro, but long mixed terminate/no-restart space still needs broader DV-plan closure
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - allowed the push engine to continue servicing buffered local payload while the DUT is in `TERMINATING`, while keeping ingress `wrreq` / `ready` clamped after lane-local end-of-run
  - verified by the clean seed-1 reruns of `P066`, `B132`, and `P126`; `P066` now closes at `accepted=85`, `push=85`, `pop=85`, `remaining=0`
- Commit: 00fc1b8

### BUG-055-R: Cross-key `push_write` could still perturb the SEARCH match fabric before the pop snapshot was frozen
- Severity: `soft error`
- Encounter sim-time: `n/a (directed-only early-SEARCH cross-key guard repro)`
- First seen in: `B133` on 2026-04-21 while extending the post-`BUG-051/052` overlap closure back into the unstable SEARCH window
- Symptom:
  - a competing key could still win `push_write_grant` while the pop engine was in the early SEARCH wait window
  - that meant the resident set feeding `cam_match_addr_oh_partitioned_comb` could change before the pop engine froze its snapshot, risking wrong pending masks or later drain corruption under mixed-key overlap
- Root cause:
  - the arbiter still allowed general push/pop overlap throughout SEARCH instead of distinguishing between the unstable pre-freeze SEARCH cycles and the later settled tail
  - same-key overlap is harmless in that window because it preserves the active match population, but cross-key writes can perturb the in-flight match fabric before snapshot freeze
- Fix status:
  - `state`: fixed and verified in the directed SEARCH-window guard slice on `dev`
  - `mechanism`: `proc_mem_arbitor_comb` now allows only same-key `push_write` overlap while SEARCH is still in the pre-freeze window; cross-key overlap is deferred until the settled SEARCH tail
  - `before_fix_outcome`: the new `B133` guard could observe a cross-key `push_write_grant` while the active SEARCH key was still in its unstable wait window, proving the resident set was still mutable before snapshot freeze
  - `after_fix_outcome`: `B133` now stays clean and the adversarial two-key alternation case `P031` still closes at `push=20000`, `pop=20000`, `overwrite=0`, `remaining=0`
  - `potential_hazard`: low to moderate; the unstable SEARCH window is now directly guarded, but any future SEARCH timing retime must keep the same-key-only rule aligned with the actual snapshot-freeze point
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - the early SEARCH overlap rule is now explicit: same-key writes may still overlap, cross-key writes may not
  - verified by clean isolated reruns of `B133` and `P031`
- Commit: 07c0dae

### BUG-056-R: Settled SEARCH-tail overlap could still clobber a slot already captured in the frozen snapshot at the live write pointer
- Severity: `soft error`
- Encounter sim-time: `n/a (reduced checkpoint overwrite-soak repro in pre-fix P125; failing intermediate log not retained as a published artifact)`
- First seen in: `P125` on 2026-04-21 after the early SEARCH guard had already landed, while re-opening safe cross-key overlap in the settled SEARCH tail
- Symptom:
  - the early SEARCH guard fixed the pre-freeze perturbation, but the balanced checkpoint overwrite soak could still corrupt later drain ownership with pop-side key mismatches and unexpected drained hits
  - the corruption only appeared once SEARCH had already frozen its snapshot, proving the remaining hazard was no longer the match-fabric settle window itself
- Root cause:
  - the settled SEARCH tail reopened cross-key overlap without checking whether the current `write_pointer` targeted a slot already present in `pop_partition_snapshot`
  - a late cross-key write could therefore overwrite a resident that the pop engine had already captured for retirement, so the later drain observed the wrong key/data at that frozen address
- Fix status:
  - `state`: fixed and verified in the checkpoint overwrite-soak reruns on `dev`
  - `mechanism`: the arbiter now allows settled-tail cross-key overlap only when the live `write_pointer` is outside the frozen `pop_partition_snapshot`
  - `before_fix_outcome`: the intermediate post-`BUG-055-R` `P125` rerun still produced pop-side key-mismatch / unexpected-drain corruption, proving a frozen snapshot slot could still be clobbered in the settled SEARCH tail
  - `after_fix_outcome`: `P125` now closes at `push=2500`, `pop=1711`, `overwrite=789`, `remaining=0`, while `P126` still closes at `push=2500`, `pop=1713`, `overwrite=787`, `remaining=0`
  - `potential_hazard`: low to moderate; the frozen-slot guard is local and directly tied to the snapshot image, but any future write-pointer or partition-snapshot refactor must preserve that address-exclusion rule
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - settled-tail overlap is now reopened only when the live write pointer sits outside the frozen snapshot, so SEARCH can keep limited throughput overlap without mutating an already-captured retirement set
  - verified by clean isolated reruns of `B056`, `P125`, and `P126`
- Commit: 07c0dae

### BUG-057-R: Low-stage partitioned-encoder variants indexed `pipe_valid` beyond the active datapath width
- Severity: `signoff block`
- Encounter sim-time: `n/a (Questa elaboration warning screen on PIPE_STAGES=1/2 build-variant runs)`
- First seen in: ad-hoc `PIPE_STAGES=1/2` variant screens on 2026-04-21 while closing the low-stage encoder build axis after `BUG-056-R`
- Symptom:
  - low-stage partitioned-encoder builds emitted out-of-range index warnings from `addr_enc_logic_partitioned.vhd`, so the build-axis evidence was not trustworthy
  - pre-fix warning signature:
    - `PIPE_STAGES=1`: index `1` / `2` out of range `0 downto 0`
    - `PIPE_STAGES=2`: index `2` out of range `1 downto 0`
- Root cause:
  - `pipe_valid_t` was sized directly from `PIPE_STAGES` even though the partitioned encoder's active datapath is fixed-width up to four stages
  - the valid-shift logic also indexed by raw `PIPE_STAGES` instead of `ACTIVE_PIPE_STAGES_CONST`, so low-stage builds could address non-existent valid bits
- Fix status:
  - `state`: fixed and verified in the low-stage build-axis reruns on `dev`
  - `mechanism`: cap `pipe_valid_t` at four bits and shift only the active `ACTIVE_PIPE_STAGES_CONST-1 downto 1` slice instead of the raw generic width
  - `before_fix_outcome`: `PIPE_STAGES=1/2` variant elaboration emitted out-of-range index warnings from `addr_enc_logic_partitioned.vhd`, blocking clean build-axis closure
  - `after_fix_outcome`: `P050` now passes cleanly on both `PIPE_STAGES=1` and `PIPE_STAGES=2`, and the pre-fix out-of-range warnings no longer appear
  - `potential_hazard`: low; the repair is local to pipeline-valid bookkeeping, but any future pipeline-depth refactor still needs to keep the active-stage width decoupled from the extra-valid tail
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - verified by clean `P050` reruns on `PIPE_STAGES=1` and `PIPE_STAGES=2`; the warning floor returned to the remaining benign startup/tool messages only
- Commit: acb9230

### BUG-058-R: Non-power-of-two ring depths let the live write pointer escape the configured ring span
- Severity: `hard stuck error`
- Encounter sim-time: `2103700 ns (first N=768 active-build profile repro before the ring-depth wrap fix)`
- First seen in: `P050` with `RING_BUFFER_N_ENTRY=768` on 2026-04-21 during the non-power-of-two depth screen
- Symptom:
  - startup `RUN_PREPARE` did not return ready in the usual window, and the simulation later fatals inside `proc_pop_write_pointer_snapshot_guard`
  - pre-fix fatal signature: `Value 768 is out of range 0 to 767`
- Root cause:
  - `write_pointer` incremented modulo the raw `SRAM_ADDR_WIDTH` instead of modulo `RING_BUFFER_N_ENTRY`, so non-power-of-two depths could generate illegal live slot indices
  - `proc_pop_write_pointer_snapshot_guard` assumed the raw pointer already lived inside `0..MAIN_CAM_SIZE-1`, so the first escaped value caused an immediate fatal
- Fix status:
  - `state`: fixed and verified in the non-power-of-two depth reruns on `dev`
  - `mechanism`: add a ring-depth-aware `ring_ptr_inc()` helper for the push pointer and guard snapshot indexing whenever `write_pointer` sits outside the live ring span
  - `before_fix_outcome`: `P050` with `RING_BUFFER_N_ENTRY=768` hung in `RUN_PREPARE` long enough to expose `write_pointer=768`, then fatals with `Value 768 is out of range 0 to 767`
  - `after_fix_outcome`: the same `P050` `N=768` rerun now passes cleanly at `push=768`, `pop=768`, `remaining=0`, and no fatal
  - `potential_hazard`: low to moderate; the live pointer is now ring-depth safe, but non-power-of-two builds still pay the longer padded CAM flush latency during `RUN_PREPARE`
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - verified by the clean `P050` rerun at `RING_BUFFER_N_ENTRY=768`, alongside the release-metadata smoke `B010` after the patch/build bump
- Commit: acb9230

### BUG-059-R: Wrap-overwrite `push_erase` could erase outside the configured ring span on non-power-of-two builds
- Severity: `signoff block`
- Encounter sim-time: `2116020 ns (first B134 directed N=768 wrap-overwrite guard repro before the erase-address wrap fix)`
- First seen in: `B134` with `RING_BUFFER_N_ENTRY=768` on 2026-04-21 during the post-review audit of the non-power-of-two overwrite path
- Symptom:
  - the second-lap wrap-overwrite path drove `cam_wr_addr=1023` instead of erasing slot `767`, so the old resident could stay live in CAM after the slot was rewritten
  - pre-fix directed guard signature: `Wrap-overwrite erased the wrong CAM slot: expected=767 observed=1023 ring_depth=768`
- Root cause:
  - `push_erase` used raw `write_pointer-1` after `write_pointer` had already wrapped to zero, instead of decrementing modulo `RING_BUFFER_N_ENTRY`
  - at `RING_BUFFER_N_ENTRY=768`, the resulting address `1023` falls outside the live CAM span that `cam_mem_a5` decodes, so the intended erase becomes a no-op
- Fix status:
  - `state`: fixed and verified in the directed guard and overwrite-soak reruns on `dev`
  - `mechanism`: add a ring-depth-aware `ring_ptr_dec()` helper and use it for the `push_erase` CAM address, so the post-wrap overwrite erase targets the previous live slot inside the configured ring span
  - `before_fix_outcome`: `B134` failed deterministically at `N=768` with `expected=767 observed=1023`, proving the wrap-overwrite erase path could miss the live slot entirely
  - `after_fix_outcome`: `B134` now passes cleanly at `N=768`, `P126` with `RING_BUFFER_N_ENTRY=768` and `DV_LONG_TXN_OVERRIDE=4000` closes at `push=4000`, `pop=3377`, `overwrite=623`, `remaining=0`, and the post-bump metadata smoke `B010` still passes
  - `potential_hazard`: low to moderate; the direct wrap-overwrite hole is now closed, but future non-power-of-two geometry changes still need to preserve modulo-safe pointer decrement behavior on every overwrite-side address path
  - `Claude Opus 4.7 xhigh review decision`: `not run in this turn; a standard Claude CLI review prompted the follow-up audit, and the bug was then confirmed locally with the directed B134 repro`
- Runtime / coverage context:
  - verified by the pre-fix `B134` fail at `N=768`, the clean post-fix `B134` rerun at the same depth, the `P126` `N=768` overwrite soak with `DV_LONG_TXN_OVERRIDE=4000`, and the release-metadata smoke `B010`
- Commit: dab30da

### BUG-060-R: `push_erase` recomputed the just-written slot inside the remaining standalone timing-critical CAM erase cone
- Severity: `signoff block`
- Encounter sim-time: `n/a (standalone Quartus signoff rerun on the tightened 137.5 MHz P4 build; no functional sim-time failure)`
- First seen in: `ring_buffer_cam_syn_p4` on 2026-04-21 while closing the remaining standalone timing blocker after `BUG-059-R`
- Symptom:
  - the delivered `P4` build still missed the tightened standalone signoff clock even after the non-power-of-two overwrite fixes had landed
  - pre-fix signoff signature:
    - slow `85C` setup slack `-0.540 ns`
    - slow `0C` setup slack `-0.214 ns`
    - the worst path still started at `write_pointer` and ended at `main_cam` `porta_we_reg`, proving the remaining blocker lived on the push/erase-side CAM control cone rather than on functional drain logic
- Root cause:
  - `push_erase` still recomputed `ring_ptr_dec(write_pointer)` in the live arbiter / memory-mux combinational path after `write_pointer` had already advanced in the previous `push_write` cycle
  - that kept the decrement logic and address-selection mux inside the already critical CAM write-enable cone, so the standalone `P4` build missed the tightened `137.5 MHz` setup target even though the overwrite behavior was functionally correct
- Fix status:
  - `state`: fixed and verified in the standalone signoff rerun plus directed overwrite-path regressions on `dev`
  - `mechanism`: capture the erase slot in `push_erase_addr_reg` on `push_write_grant`, default the arbiter/memory mux to the push-write ownership shape, and let `push_erase` consume the captured slot instead of recomputing `write_pointer-1` in the critical comb cone
  - `before_fix_outcome`: `ring_buffer_cam_syn_p4` missed the tightened standalone signoff target with slow `85C` WNS `-0.540 ns` and slow `0C` WNS `-0.214 ns`, while the overwrite-focused functional reruns were already green
  - `after_fix_outcome`: `ring_buffer_cam_syn_p4` now closes at slow `85C` WNS `+0.080 ns`, slow `0C` WNS `+0.347 ns`, fast `85C` WNS `+2.771 ns`, fast `0C` WNS `+3.280 ns`, and worst reported hold slack `+0.149 ns`; `B134(n768)`, `P111`, and the patch-bumped metadata smoke `B010` all pass on `26.2.3.0421`
  - `potential_hazard`: low to moderate; the critical family still originates at `write_pointer` and ends in the `main_cam` write-enable path, so future push/erase-side decode growth on the same cone could reopen the margin
  - `Claude Opus 4.7 xhigh review decision`: `review completed; the missing-evidence and version-skew findings on the fix-only commit are resolved by the follow-up docs/evidence sync batch, and the remaining idle zero-drive note is treated as a low-risk waveform/toggle caveat rather than a promoted RTL bug`
- Runtime / coverage context:
  - this was a timing-only RTL blocker; the intended overwrite behavior stayed the same
  - verified by the clean standalone `ring_buffer_cam_syn_p4` rerun, the directed `B134` `N=768` rerun, the same-key overwrite stress `P111`, and the post-bump metadata smoke `B010`
- Commit: 1736898

### BUG-061-R: `TERMINATING` control ready could acknowledge before lane-local end-of-run had actually closed the drain contract
- Severity: `hard stuck error`
- Encounter sim-time: `n/a (directed-only terminate-ack guard repro; the failing B040 log was superseded by the verification rerun after the patch landed)`
- First seen in: `B040` on 2026-04-21 while reopening the live failing BASIC terminate-control slice after the full implemented isolated rerun
- Symptom:
  - the DUT could enter `TERMINATING` from an otherwise quiescent state and raise `asi_ctrl_ready` before any lane-local end-of-run marker had been observed
  - that violated the documented run-control contract and allowed software to see an apparent terminate ack before `endofrun_seen`, FIFO drain, and `push=pop+overwrite` had all converged
- Root cause:
  - `proc_run_control_mgmt` drove `asi_ctrl_ready` in `TERMINATING` from `terminating_drain_done OR terminating_entry_quiescent`
  - the `terminating_entry_quiescent` bypass ignored the required `endofrun_seen` conjunct, so an empty entry into `TERMINATING` could acknowledge immediately even though the true drain-done condition had not yet been met
- Fix status:
  - `state`: fixed and verified in the directed terminate-control reruns on `dev`
  - `mechanism`: remove the `terminating_entry_quiescent` shortcut and gate `TERMINATING` host acknowledgement only on `terminating_drain_done`
  - `before_fix_outcome`: `B040` observed `asi_ctrl_ready=1` a few cycles after `TERMINATING` entry while the testcase was still deliberately withholding the lane-local end-of-run marker
  - `after_fix_outcome`: `B040` now passes cleanly, and the neighboring terminate guards `B113`, `B114`, and `B132` remain green on the same `default_p2_pipe4` rerun batch
  - `potential_hazard`: low to moderate; any future refactor of the `TERMINATING` handshake still has to preserve the rule that host acknowledgement is downstream of the full `endofrun_seen & fifo-empty & pop-idle & push=pop+overwrite` drain contract
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - this is a control-path RTL bug rather than a payload-corruption datapath bug, but it directly affects the run-control sequencing contract used by the directed BASIC terminate cases
  - verified by clean reruns of `B040`, `B113`, `B114`, and `B132`
- Commit: b4e0daa

### BUG-062-H: The generic idle helper treated a lone pending end-of-run marker as live traffic even after the DUT had already closed the lane
- Severity: `non-datapath-refactor`
- Encounter sim-time: `3493980 ns (seed-1 B071 terminate-condition audit before the lone-marker cleanup exception was added)`
- First seen in: `B071` on 2026-04-21 immediately after `BUG-061-R` was fixed and the terminate-condition audit was rerun against the updated RTL
- Symptom:
  - `B071` timed out in its final cleanup wait even though every real drain condition was already satisfied: `remaining=0`, `pending_drain=0`, `accepted=written=1024`, `deassm_empty=1`, `pop_cmd_empty=1`, and `terminating_drain_done=1`
  - the only remaining blocker was `source_backlog=1`, which made the generic idle helper report a failure even though there was no live payload left in the DUT or scoreboard
- Root cause:
  - the source driver can still hold one queued empty end-of-run marker after the DUT has already latched `endofrun_seen`, because the RTL samples the marker from `valid/endofpacket` even if ingress `ready` is already low in `TERMINATING`
  - `wait_for_scoreboard_idle()` required `pending_source_items()==0` unconditionally, so it treated that lone queued marker the same as real unaccepted payload backlog
- Fix status:
  - `state`: fixed and verified in the terminate cleanup reruns on `dev`
  - `mechanism`: add a `hit_driver` helper that identifies a head pending empty marker and let `wait_for_scoreboard_idle()` ignore exactly one such marker once `endofrun_seen=1` and `terminating_drain_done=1`
  - `before_fix_outcome`: seed-1 `B071` timed out after 300000 cycles with `source_backlog=1` while all scoreboard and internal drain counters had already converged
  - `after_fix_outcome`: `B071` now passes cleanly, and the same rerun cluster keeps `B040`, `B113`, `B114`, and `B132` green, showing the cleanup relaxation did not mask real payload backlog
  - `potential_hazard`: low; the helper still blocks on any payload backlog, multiple queued markers, or lone markers before the DUT has actually latched `endofrun_seen` and reached `terminating_drain_done`
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - this is a harness/accounting closure fix with no datapath behavior change
  - verified by clean reruns of `B071`, `B040`, `B113`, `B114`, and `B132`
- Commit: b4e0daa

### BUG-063-H: The shared low-stage partition-latency helper anchored on a stage-4-only pulse instead of the real active-partition load event
- Severity: `non-datapath-refactor`
- Encounter sim-time: `n/a (directed-only low-stage build-axis latency reruns before the helper anchor was corrected)`
- First seen in: `B094`, `B095`, `B096`, `P054`, `P055`, and `P056` on 2026-04-21 while closing the PIPE_STAGES build axis with standalone per-variant evidence
- Symptom:
  - the directed PIPE_STAGES=1/2/3 latency smokes and the matching partition-balance profile smokes could report a missing or mis-timed encoder result pulse even though the DUT variant was behaving correctly
  - that blocked trustworthy case promotion on the low-stage variants because the common helper was measuring against a debug pulse shape that only lines up with the fully staged build
- Root cause:
  - `run_pipe_stage_latency_smoke_case()` anchored its timing window on `pop_partition_eval_stage0_valid`, which is not the stable contract point for the reduced-stage variants
  - the correct DUT-visible start event is the active pending-partition `pop_partition_load` pulse for the target search key, followed by the flagged `pop_partition_result_valid` on the same partition
- Fix status:
  - `state`: fixed and verified in the low-stage directed and profile reruns on `dev`
  - `mechanism`: retime the helper to capture the active pending-partition `pop_partition_load` pulse, derive the active partition from `pop_partition_pending`, and measure latency to the flagged `pop_partition_result_valid` pulse on that partition
  - `before_fix_outcome`: the low-stage build-axis cases could fail or mis-measure latency because the helper waited on a stage-4-specific internal pulse instead of the real per-variant load event
  - `after_fix_outcome`: `B094`, `B095`, `B096`, `P054`, `P055`, and `P056` now pass cleanly on their intended `PIPE_STAGES=1/2/3` variants with per-case logs and UCDBs published into the refreshed dashboard
  - `potential_hazard`: low; the helper is now keyed to the architecturally relevant partition-load/result handshake, but any future encoder debug-signal refactor still needs to preserve a variant-independent load anchor for latency checks
  - `Claude Opus 4.7 xhigh review decision`: `pending / not run`
- Runtime / coverage context:
  - this is a harness-only measurement fix with no DUT behavior change
  - verified by clean reruns of `B094`, `B095`, `B096`, `P054`, `P055`, and `P056` on the matching low-stage build variants, plus the refreshed report-generation pass that now links those variant-specific logs correctly
- Commit: 4c62cd0

# ⚠️ Signoff — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-22` &nbsp;
**Release under check:** `26.2.4.0421` &nbsp; **Evidence basis:** `DV=8cd88cd, SYN=1736898`

This page is the master signoff dashboard. Detailed synthesis evidence lives in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md); detailed DV evidence lives in [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / caveat &middot; ❌ failed / blocked &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ⚠️ | overall_signoff | `partial` |
| ⚠️ | standalone_syn_p4_512 | `the last standalone Quartus rerun on 1736898 closes the tightened 137.5 MHz / 7.273 ns signoff target (slow-85C WNS=+0.308 ns, slow-0C WNS=+0.429 ns, worst reported hold=+0.149 ns), but later RTL commits from BUG-061-R onward have not yet been rerun through that standalone flow` |
| ✅ | dv_closure | `the promoted isolated dashboard is green: failed_cases=0, signoff_runs_with_failures=0, unimplemented_cases=0, and 464/464 promoted signoff cases are evidenced; 55 catalog backlog cases remain explicitly outside the current signoff scope` |
| ⚠️ | cross_bucket_signoff | `0` continuous-frame signoff runs |
| ⚠️ | gate_level_sim | `not rerun in this refresh` |
| ⚠️ | harness_output_constraints | `32 unconstrained probe_out paths` |

## Verification

| status | area | result | source |
|:---:|---|---|---|
| ✅ | isolated DV closure | `100.0% (464/464)` promoted functional coverage, `0` failed promoted cases, and top-level totals are green on stmt/branch/fsm_state/fsm_trans/toggle | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ⚠️ | bucket / continuous-frame signoff | `0` cross runs recorded; the current DV dashboard keeps this scope as an explicit non-claim | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ✅ | implemented isolated matrix | the current supported QuestaOne 2026 isolated refresh evidences all `464` promoted signoff cases with `0` stale artifacts and `0` implemented-case failures in the published dashboard | [`../tb/REPORT/README.md`](../tb/REPORT/README.md) |
| ✅ | bug ledger | harness and RTL issues tracked in the live DV ledger | [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) |

## Synthesis

| status | item | value |
|:---:|---|---|
| ✅ | revision | `ring_buffer_cam_syn_p4` |
| ✅ | device | `5AGXBA7D4F31C5` |
| ✅ | signoff constraint | `137.5 MHz` / `7.273 ns` |
| ✅ | slow 85C WNS / TNS | `+0.308 ns` / `0.000 ns` |
| ✅ | slow 0C WNS / TNS | `+0.429 ns` / `0.000 ns` |
| ✅ | worst hold slack | `+0.149 ns` |
| ✅ | slow 85C Fmax | `143.58 MHz` |
| ✅ | nominal 125 MHz headroom | `14.9%` |
| ✅ | fitted resources | `2,519 ALMs`, `2,928 regs`, `19 RAM blocks`, `153,600` bits |
| ⚠️ | TimeQuest caveat | internal `clk125` timing now closes across the reported corners, but the standalone harness still leaves `32` unconstrained `probe_out[31:0]` observation outputs outside the DUT reg-to-reg timing domain |
| ✅ | detail report | [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) |

## Fixes In Scope

| status | class | summary |
|:---:|---|---|
| ✅ | RTL | `BUG-039-R` through `BUG-045-R` remain in scope for this release: descriptor FIFO overrun is blocked, soft-reset is a real abort-to-`IDLE`, stale post-reset work cannot retire, and equal-load partition drain still rotates fairly |
| ✅ | Harness | `BUG-043-H`, `BUG-044-H`, and `BUG-046-H` remain in scope: the staged late-arrival helper is rate-correct, PROF sink backpressure remains active through drain, and clean terminate waits for the DUT-side deassembly FIFO |
| ✅ | Harness | `BUG-050-H` removes exact-full compat `scfifo` warning floods so nightly logs only carry the remaining benign startup/tool warnings |
| ✅ | RTL | `BUG-051-R` and `BUG-052-R` remain in scope: `DRAIN`, `LOAD`, and `COUNT` still keep the frozen pop snapshot immutable until drain completes |
| ✅ | RTL | `BUG-053-R` and `BUG-054-R` remain in scope: `TERMINATING` still clamps post-EOR ingress and drains already-buffered deassembly residue to completion |
| ✅ | RTL | `BUG-055-R` blocks cross-key `push_write` while the SEARCH match fabric is still settling, so the pre-freeze snapshot cannot be perturbed |
| ✅ | RTL | `BUG-056-R` prevents settled-SEARCH tail overlap from clobbering any slot already captured in the frozen snapshot at the live write pointer |
| ✅ | RTL | `BUG-057-R`, `BUG-058-R`, and `BUG-059-R` remain in scope: low-stage encoder builds no longer index `pipe_valid` out of range, the live write pointer now wraps modulo `RING_BUFFER_N_ENTRY`, and wrap-overwrite `push_erase` now erases the previous live slot instead of falling outside the configured ring span |
| ✅ | RTL | `BUG-060-R` carries the overwrite erase slot from `push_write`, removing the last negative-slack path family on the standalone `P4` CAM erase/write-enable cone without changing overwrite semantics |
| ✅ | RTL | `BUG-061-R` removes the premature `TERMINATING` host-ack shortcut, so `asi_ctrl_ready` now waits for the full lane-local drain-done contract instead of acknowledging from an entry-quiescent bypass |
| ✅ | Harness | `BUG-062-H` lets the generic idle helper ignore exactly one queued empty end-of-run marker once `endofrun_seen` and `terminating_drain_done` are already high, so terminate cleanup no longer misclassifies that marker as live payload backlog |
| ✅ | DV | refreshed evidence keeps the promoted isolated matrix green, evidences `464/464` promoted signoff cases, and regenerates the `tb/REPORT/` dashboard for the current nightly checkpoint |
| ✅ | Metadata | wrapper defaults, Platform Designer packaging, and the CMSIS-SVD source/emit flow are aligned to `26.2.4.0421` / `20260421` with `BUILD=421` and `PATCH=4` |

## Evidence Index

- [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) — active DV dashboard
- [`../tb/DV_COV.md`](../tb/DV_COV.md) — active DV coverage summary
- [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) — bug ledger
- [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) — detailed standalone synthesis / timing report
- [`../syn/quartus/ring_buffer_cam_syn_p4.qsf`](../syn/quartus/ring_buffer_cam_syn_p4.qsf) — active standalone P4 revision
- [`../syn/quartus/ring_buffer_cam_syn.sdc`](../syn/quartus/ring_buffer_cam_syn.sdc) — tightened `137.5 MHz` signoff constraint
- [`../syn/quartus/ring_buffer_cam_syn_harness.vhd`](../syn/quartus/ring_buffer_cam_syn_harness.vhd) — standalone synthesis harness

## Notes

- This dashboard supersedes the earlier monolithic signoff note. Current closure is derived from the split DV workflow plus the standalone synthesis report.
- The current 2026-04-22 refresh keeps the promoted isolated DV matrix green, refreshes the dashboard through `8cd88cd`, and records the bug-ledger encounterability cleanup required by the live workflow linter.
- The standalone synthesis report in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) still reflects the last Quartus rerun on commit `1736898`; later RTL commits, including `BUG-061-R`, have not yet been recompiled through that standalone flow.
- The synthesis result is for the delivered `Default P4` shape: `512` entries, `4` partitions, `4` encoder stages.
- The active DV dashboard is currently built and evidenced across the promoted build matrix `default_p2_pipe4, p2_pipe1, p2_pipe2, p2_pipe3, p4_n4_pipe4`. The differing `P4` / `P2` / `P4-N4` shapes are intentional and are called out explicitly so synthesis and DV numbers are not conflated.
- Overall signoff remains `⚠️ partial` because continuous-frame signoff, gate-level simulation, and a standalone Quartus rerun after the latest RTL changes are still open even though the isolated DV dashboard and the last published standalone timing/resource gate are green.

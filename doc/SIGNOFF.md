# ⚠️ Signoff — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-21` &nbsp;
**Release under check:** `26.2.4.0421` &nbsp; **Evidence basis:** `pending current bug-fix batch`

This page is the master signoff dashboard. Detailed synthesis evidence lives in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md); detailed DV evidence lives in [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / caveat &middot; ❌ failed / blocked &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ⚠️ | overall_signoff | `partial` |
| ⚠️ | standalone_syn_p4_512 | `the last standalone Quartus rerun on 1736898 closes the tightened 137.5 MHz / 7.273 ns signoff target (slow-85C WNS=+0.080 ns, slow-0C WNS=+0.347 ns, worst reported hold=+0.149 ns), but that compile has not yet been rerun after the current 26.2.4.0421 terminate-control patch bump` |
| ❌ | dv_closure | `342/516` implemented isolated cases exercised, `28` currently failing |
| ⚠️ | cross_bucket_signoff | `0` continuous-frame signoff runs |
| ⚠️ | gate_level_sim | `not rerun in this refresh` |
| ⚠️ | harness_output_constraints | `32 unconstrained probe_out paths` |

## Verification

| status | area | result | source |
|:---:|---|---|---|
| ❌ | isolated DV closure | `60.85% (314/516)` passing functional proxy, `28` active failed implemented cases on `default_p2_pipe4` | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ⚠️ | bucket / continuous-frame signoff | `174` planned cases still unimplemented, `0` cross runs recorded | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ⚠️ | implemented isolated matrix | current supported-QuestaOne isolated refresh executes `342` implemented cases and regenerates the dashboard from that evidence, but `28` cases still fail | [`../tb/REPORT/README.md`](../tb/REPORT/README.md) |
| ✅ | bug ledger | harness and RTL issues tracked in the live DV ledger | [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) |

## Synthesis

| status | item | value |
|:---:|---|---|
| ✅ | revision | `ring_buffer_cam_syn_p4` |
| ✅ | device | `5AGXBA7D4F31C5` |
| ✅ | signoff constraint | `137.5 MHz` / `7.273 ns` |
| ✅ | slow 85C WNS / TNS | `+0.080 ns` / `0.000 ns` |
| ✅ | slow 0C WNS / TNS | `+0.347 ns` / `0.000 ns` |
| ✅ | worst hold slack | `+0.149 ns` |
| ✅ | slow 85C Fmax | `139.02 MHz` |
| ✅ | nominal 125 MHz headroom | `11.2%` |
| ✅ | fitted resources | `2,518 ALMs`, `2,938 regs`, `19 RAM blocks`, `153,600` bits |
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
| ✅ | DV | refreshed evidence keeps `B010`, `B040`, `B071`, `B113`, `B114`, and `B132` green and regenerates the `tb/REPORT/` dashboard for the current nightly checkpoint |
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
- The latest refresh fixes `BUG-061-R` and `BUG-062-H`, reruns `B010`, `B040`, `B071`, `B113`, `B114`, and `B132`, bumps the delivered metadata to `26.2.4.0421`, and regenerates the dashboard from the refreshed evidence.
- The standalone synthesis report in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) still reflects the last Quartus rerun on commit `1736898`; the current 26.2.4.0421 terminate-control patch has not yet been recompiled through that standalone flow.
- The synthesis result is for the delivered `Default P4` shape: `512` entries, `4` partitions, `4` encoder stages.
- The active DV dashboard is currently built and evidenced on the `default_p2_pipe4` simulation variant. The differing P4/P2 build shapes are intentional and are called out explicitly so synthesis and DV numbers are not conflated.
- Overall signoff remains `⚠️ partial` because DV plan closure and gate-level simulation are still incomplete even though the refreshed standalone timing/resource gate for the active `P4` build is green again.

# ⚠️ Signoff — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-20` &nbsp;
**Release under check:** `26.1.11.0419` &nbsp; **Evidence basis:** `abf5d42`

This page is the master signoff dashboard. Detailed synthesis evidence lives in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md); detailed DV evidence lives in [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / caveat &middot; ❌ failed / blocked &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ⚠️ | overall_signoff | `partial` |
| ✅ | standalone_syn_p4_512 | `pass` |
| ⚠️ | dv_closure | `306/516` planned cases evidenced |
| ⚠️ | cross_bucket_signoff | `0` continuous-frame signoff runs |
| ⚠️ | gate_level_sim | `not rerun in this refresh` |
| ⚠️ | harness_output_constraints | `32 unconstrained probe_out paths` |

## Verification

| status | area | result | source |
|:---:|---|---|---|
| ⚠️ | isolated DV closure | `59.30% (306/516)` functional proxy, `0` active failed implemented cases on `default_p2_pipe4` | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ⚠️ | bucket / continuous-frame signoff | `196` planned cases still unimplemented, `0` cross runs recorded | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ✅ | implemented isolated matrix | current implemented isolated refresh passes cleanly | [`../tb/REPORT/README.md`](../tb/REPORT/README.md) |
| ✅ | bug ledger | harness and RTL issues tracked in the live DV ledger | [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) |

## Synthesis

| status | item | value |
|:---:|---|---|
| ✅ | revision | `ring_buffer_cam_syn_p4` |
| ✅ | device | `5AGXBA7D4F31C5` |
| ✅ | signoff constraint | `137.5 MHz` / `7.273 ns` |
| ✅ | slow 85C WNS / TNS | `+0.450 ns` / `0.000 ns` |
| ✅ | worst hold slack | `+0.162 ns` |
| ✅ | slow 85C Fmax | `146.56 MHz` |
| ✅ | nominal 125 MHz headroom | `17.2%` |
| ✅ | fitted resources | `2,364 ALMs`, `2,825 regs`, `19 RAM blocks`, `161,536` bits |
| ⚠️ | TimeQuest caveat | internal `clk125` domain closes, but `probe_out[31:0]` is left unconstrained in the standalone harness |
| ✅ | detail report | [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) |

## Fixes In Scope

| status | class | summary |
|:---:|---|---|
| ✅ | RTL | `BUG-039` gates pop descriptor generation on `pop_cmd_fifo_full` so the 16-entry command FIFO cannot be overrun under sustained backlog |
| ✅ | RTL | `BUG-040` turns CSR `soft_reset` into a real abort-to-`IDLE` path that clears live counters, fill level, FIFOs, and internal state |
| ✅ | RTL | `BUG-041` blocks stale descriptor consumption after soft-reset has already returned the DUT to `IDLE` |
| ✅ | RTL | `BUG-042` blocks stale buffered push / overwrite retirement after soft-reset by requiring an active run state |
| ✅ | Harness | `BUG-043` fixes the staged `P042` late-arrival helper so the fingerprint space stays unique and the aggregate ingress rate remains calibrated to the planned `gap=13` profile |
| ✅ | DV | promoted `P041-P045` and `P050-P053` to live UVM, then refreshed the active PROF evidence slice with clean isolated reruns on `B010`, `B011`, `P041-P045`, and `P050-P053` |
| ✅ | Metadata | wrapper defaults and Platform Designer packaging are aligned to `26.1.11.0419` / `20260419` |

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
- The latest refresh was DV-only: `P050-P053` are now evidenced in the active `default_p2_pipe4` build, while the standalone synthesis result remains the last clean `ring_buffer_cam_syn_p4` compile reported in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md).
- The synthesis result is for the delivered `Default P4` shape: `512` entries, `4` partitions, `4` encoder stages.
- The active DV dashboard is currently built and evidenced on the `default_p2_pipe4` simulation variant. The differing P4/P2 build shapes are intentional and are called out explicitly so synthesis and DV numbers are not conflated.
- Overall signoff remains `⚠️ partial` because DV plan closure is still incomplete even though the standalone timing/resource gate for the active P4 build is now green.

# ⚠️ Signoff — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-21` &nbsp;
**Release under check:** `26.2.2.0421` &nbsp; **Evidence basis:** `dab30da`

This page is the master signoff dashboard. Detailed synthesis evidence lives in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md); detailed DV evidence lives in [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / caveat &middot; ❌ failed / blocked &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ⚠️ | overall_signoff | `partial` |
| ❌ | standalone_syn_p4_512 | `2026-04-21 rerun on dab30da misses the tightened 137.5 MHz / 7.273 ns signoff target: slow-85C WNS=-0.540 ns, slow-0C WNS=-0.214 ns` |
| ⚠️ | dv_closure | `323/516` planned cases evidenced |
| ⚠️ | cross_bucket_signoff | `0` continuous-frame signoff runs |
| ⚠️ | gate_level_sim | `not rerun in this refresh` |
| ⚠️ | harness_output_constraints | `32 unconstrained probe_out paths` |

## Verification

| status | area | result | source |
|:---:|---|---|---|
| ⚠️ | isolated DV closure | `62.60% (323/516)` functional proxy, `0` active failed implemented cases on `default_p2_pipe4` | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ⚠️ | bucket / continuous-frame signoff | `174` planned cases still unimplemented, `0` cross runs recorded | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ✅ | implemented isolated matrix | current implemented isolated refresh passes cleanly | [`../tb/REPORT/README.md`](../tb/REPORT/README.md) |
| ✅ | bug ledger | harness and RTL issues tracked in the live DV ledger | [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) |

## Synthesis

| status | item | value |
|:---:|---|---|
| ✅ | revision | `ring_buffer_cam_syn_p4` |
| ✅ | device | `5AGXBA7D4F31C5` |
| ✅ | signoff constraint | `137.5 MHz` / `7.273 ns` |
| ❌ | slow 85C WNS / TNS | `-0.540 ns` / `-3.973 ns` |
| ⚠️ | slow 0C WNS / TNS | `-0.214 ns` / `-0.565 ns` |
| ✅ | worst hold slack | `+0.149 ns` |
| ⚠️ | slow 85C Fmax | `127.99 MHz` |
| ⚠️ | nominal 125 MHz headroom | `2.4%` |
| ⚠️ | fitted resources | `2,547 ALMs`, `2,908 regs`, `19 RAM blocks`, `153,600` bits |
| ⚠️ | TimeQuest caveat | internal `clk125` domain still has `32` unconstrained `probe_out[31:0]` harness outputs, and the current routed P4 build misses the tightened setup target on both slow corners |
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
| ✅ | DV | refreshed evidence closes `P050(pipe1)`, `P050(pipe2)`, `P050(n768)`, `B134(n768)`, `P126(n768, DV_LONG_TXN_OVERRIDE=4000)`, and the release-metadata smoke `B010`, and regenerates the `tb/REPORT/` dashboard for the current nightly checkpoint |
| ✅ | Metadata | wrapper defaults, Platform Designer packaging, and the CMSIS-SVD source/emit flow are aligned to `26.2.2.0421` / `20260421` with `BUILD=421` and `PATCH=2` |

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
- The latest refresh fixed `BUG-057-R`, `BUG-058-R`, and `BUG-059-R`, reran the low-stage / non-power-of-two screens (`P050(pipe1/pipe2/n768)`), added the directed wrap-overwrite guard `B134(n768)`, reran the `P126(n768, DV_LONG_TXN_OVERRIDE=4000)` overwrite soak, rechecked `B010`, and regenerated the dashboard from the refreshed evidence.
- The standalone synthesis report in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) was rerun on `2026-04-21` for this release on commit `dab30da` rather than inherited from an earlier checkpoint.
- The synthesis result is for the delivered `Default P4` shape: `512` entries, `4` partitions, `4` encoder stages.
- The active DV dashboard is currently built and evidenced on the `default_p2_pipe4` simulation variant. The differing P4/P2 build shapes are intentional and are called out explicitly so synthesis and DV numbers are not conflated.
- Overall signoff remains `⚠️ partial` because DV plan closure is still incomplete and the refreshed standalone timing/resource gate for the active P4 build is no longer green.

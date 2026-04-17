# ⚠️ Signoff — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-17` &nbsp;
**Release under check:** `26.1.5.0419` &nbsp; **Git base:** `fb4c6c2 + working tree refresh`

This page is the master signoff dashboard. Detailed synthesis evidence lives in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md); detailed DV evidence lives in [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / caveat &middot; ❌ failed / blocked &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ⚠️ | overall_signoff | `partial` |
| ✅ | standalone_syn_p4_512 | `pass` |
| ⚠️ | dv_closure | `95/516` planned cases evidenced |
| ⚠️ | cross_bucket_signoff | `0` continuous-frame signoff runs |
| ⚠️ | gate_level_sim | `not rerun in this refresh` |
| ⚠️ | harness_output_constraints | `32 unconstrained probe_out paths` |

## Verification

| status | area | result | source |
|:---:|---|---|---|
| ⚠️ | isolated DV closure | `18.41% (95/516)` functional proxy, `0` active failed implemented cases | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ⚠️ | bucket / continuous-frame signoff | `421` planned cases still unimplemented, `0` cross runs recorded | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ✅ | implemented isolated matrix | current implemented isolated refresh passes cleanly | [`../tb/REPORT/README.md`](../tb/REPORT/README.md) |
| ✅ | bug ledger | harness and RTL issues tracked in the live DV ledger | [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) |

## Synthesis

| status | item | value |
|:---:|---|---|
| ✅ | revision | `ring_buffer_cam_syn_p4` |
| ✅ | device | `5AGXBA7D4F31C5` |
| ✅ | signoff constraint | `137.5 MHz` / `7.273 ns` |
| ✅ | slow 85C WNS / TNS | `+1.077 ns` / `0.000 ns` |
| ✅ | worst hold slack | `+0.152 ns` |
| ✅ | slow 85C Fmax | `161.39 MHz` |
| ✅ | nominal 125 MHz headroom | `29.1%` |
| ✅ | fitted resources | `2,383 ALMs`, `2,862 regs`, `19 RAM blocks`, `161,536` bits |
| ⚠️ | TimeQuest caveat | internal `clk125` domain closes, but `probe_out[31:0]` is left unconstrained in the standalone harness |
| ✅ | detail report | [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) |

## Fixes In Scope

| status | class | summary |
|:---:|---|---|
| ✅ | RTL | `BUG-007` width-safe 48-bit cleanup compares verified in the signoff build |
| ✅ | RTL | `BUG-008` same-key overwrite tail suppression verified in the signoff build |
| ✅ | Harness | standalone Quartus source list repaired to compile the live `rtl/` tree and active V2 core |
| ✅ | Harness | synthesis harness updated to the live wrapper contract and normalized to the delivered `512`-entry depth |
| ✅ | Metadata | wrapper defaults aligned to `26.1.5.0419` so runtime identity matches the packaged IP |

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
- The synthesis result is for the delivered `Default P4` shape: `512` entries, `4` partitions, `4` encoder stages.
- Overall signoff remains `⚠️ partial` because DV plan closure is still incomplete even though the standalone timing/resource gate for the active P4 build is now green.

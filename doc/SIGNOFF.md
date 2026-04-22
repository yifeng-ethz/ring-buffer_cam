# ✅ Signoff — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-22` &nbsp;
**Release under check:** `26.2.6.0422` &nbsp; **Evidence basis:** `DV/SYN/GATE=1069e0b`

This page is the master signoff dashboard. Detailed synthesis evidence lives in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md); detailed DV evidence lives in [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / caveat &middot; ❌ failed / blocked &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ✅ | overall_signoff | `pass` |
| ✅ | standalone_syn_p4_512 | `the standalone Quartus rerun on the current 26.2.6.0422 image closes the tightened 137.5 MHz / 7.273 ns signoff target with slow-85C WNS=+0.515 ns, slow-0C WNS=+0.575 ns, and worst reported hold slack=+0.171 ns` |
| ✅ | dv_closure | `the promoted isolated dashboard is green: failed_cases=0, signoff_runs_with_failures=0, unimplemented_cases=0, 519/519 promoted signoff cases are evidenced, and stmt/branch/fsm_state/fsm_trans/toggle all meet target after the documented branch dead-bin exclusion` |
| ✅ | cross_bucket_signoff | `6` continuous-frame signoff runs are published, and all record counter_checks_failed=0 and unexpected_outputs=0 |
| ✅ | gate_level_sim | `functional gate smoke compare passes on the regenerated P4 netlist: rtl_signature=0xfd448996, gate_signature=0xac7007dc, and signature equality remains advisory for the functional gate model` |
| ℹ️ | harness_output_constraints | `32` unconstrained `probe_out[31:0]` observation outputs remain outside the DUT reg-to-reg timing domain and are treated as harness-only non-claims |

## Verification

| status | area | result | source |
|:---:|---|---|---|
| ✅ | isolated DV closure | `100.0% (519/519)` promoted functional coverage with `stmt=96.79`, `branch=90.95`, `fsm_state=100.00`, `fsm_trans=100.00`, `toggle=86.72`, and `0` failed promoted cases | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ✅ | continuous-frame signoff | all `6` published signoff runs pass with `0` counter mismatches, `0` unexpected outputs, and functional-cross range `80.9-88.8` | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ✅ | implemented isolated matrix | the current supported QuestaOne 2026 isolated refresh evidences all `519` promoted signoff cases with `0` stale artifacts and `0` implemented-case failures in the published dashboard | [`../tb/REPORT/README.md`](../tb/REPORT/README.md) |
| ✅ | gate smoke compare | the regenerated standalone gate netlist passes the functional smoke harness on both RTL and gate benches | [`../tb/gate/Makefile`](../tb/gate/Makefile) |
| ✅ | bug ledger | harness and RTL issues tracked in the live DV ledger | [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) |

## Synthesis

| status | item | value |
|:---:|---|---|
| ✅ | revision | `ring_buffer_cam_syn_p4` |
| ✅ | device | `5AGXBA7D4F31C5` |
| ✅ | signoff constraint | `137.5 MHz` / `7.273 ns` |
| ✅ | slow 85C WNS / TNS | `+0.515 ns` / `0.000 ns` |
| ✅ | slow 0C WNS / TNS | `+0.575 ns` / `0.000 ns` |
| ✅ | worst hold slack | `+0.171 ns` |
| ✅ | slow 85C Fmax | `147.97 MHz` |
| ✅ | nominal 125 MHz headroom | `18.4%` |
| ✅ | fitted resources | `2,191 ALMs`, `2,861 regs`, `19 RAM blocks`, `153,600` bits |
| ℹ️ | TimeQuest caveat | internal `clk125` timing closes across the reported corners; the remaining unconstrained paths are the standalone harness `probe_out[31:0]` observation outputs only |
| ✅ | detail report | [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) |

## Fixes In Scope

| status | class | summary |
|:---:|---|---|
| ✅ | RTL | `BUG-039-R` through `BUG-060-R` remain in scope for this release, including the SEARCH-window, overwrite, wrap, and standalone timing repairs already carried by the live `26.2.6.0422` tree |
| ✅ | RTL | `BUG-061-R` keeps `asi_ctrl_ready` downstream of the full lane-local terminate drain-done contract instead of the earlier entry-quiescent shortcut |
| ✅ | Harness | `BUG-062-H` and `BUG-063-H` remain in scope: terminate cleanup ignores exactly one post-EOR empty marker, and the low-stage latency helper now anchors on the real partition-load/result handshake |
| ✅ | RTL | `BUG-064-R` replaces the exact settled-SEARCH-tail snapshot-membership test with a cheaper conservative overwrite-slot predicate, preserving the `BUG-055-R` / `BUG-056-R` SEARCH correctness fixes without reopening the standalone `P4` timing blocker |
| ✅ | DV | refreshed evidence keeps the promoted isolated matrix green at `519/519`, republishes `6` clean continuous-frame signoff runs, and regenerates the `tb/REPORT/` dashboard for the current signoff checkpoint |
| ✅ | Metadata | wrapper defaults, Platform Designer packaging, and the emitted build metadata are aligned to `26.2.6.0422` / `20260422` with `BUILD=422` and `PATCH=6` |

## Evidence Index

- [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) — active DV dashboard
- [`../tb/DV_COV.md`](../tb/DV_COV.md) — active DV coverage summary
- [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) — bug ledger
- [`../tb/gate/logs/rtl_signature.log`](../tb/gate/logs/rtl_signature.log) — RTL gate-smoke signature log
- [`../tb/gate/logs/gate_signature.log`](../tb/gate/logs/gate_signature.log) — regenerated netlist gate-smoke signature log
- [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) — detailed standalone synthesis / timing report
- [`../syn/quartus/ring_buffer_cam_syn_p4.qsf`](../syn/quartus/ring_buffer_cam_syn_p4.qsf) — active standalone P4 revision
- [`../syn/quartus/ring_buffer_cam_syn.sdc`](../syn/quartus/ring_buffer_cam_syn.sdc) — tightened `137.5 MHz` signoff constraint
- [`../syn/quartus/ring_buffer_cam_syn_harness.vhd`](../syn/quartus/ring_buffer_cam_syn_harness.vhd) — standalone synthesis harness
- [`../syn/quartus/gate_sim/ring_buffer_cam_syn_p4.vo`](../syn/quartus/gate_sim/ring_buffer_cam_syn_p4.vo) — regenerated standalone gate netlist

## Notes

- This dashboard supersedes the earlier monolithic signoff note. Current closure is derived from the split DV workflow, the standalone synthesis report, and the regenerated gate-smoke evidence.
- The 2026-04-22 refresh closes the last residual signoff blockers: the promoted isolated matrix is green at `519/519`, continuous-frame signoff is published with `6` clean runs, and the standalone Quartus `P4` rerun plus gate netlist smoke both pass on the live `26.2.6.0422` image.
- The branch total in [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) excludes `16` compile-time-dead bins in `addr_enc_logic_partitioned`; this is a documented report adjustment for constant-generic dead code, not a hidden testcase waiver.
- The synthesis result is for the delivered `Default P4` shape: `512` entries, `4` partitions, `4` encoder stages.
- The active DV dashboard is built and evidenced across the promoted build matrix `default_p2_pipe4, p2_pipe1, p2_pipe2, p2_pipe3, p4_n4_pipe4`. The differing `P4` / `P2` / `P4-N4` shapes are intentional and are called out explicitly so synthesis and DV numbers are not conflated.

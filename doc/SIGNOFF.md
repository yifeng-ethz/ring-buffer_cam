# ✅ Signoff — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-05-08` &nbsp;
**Release under check:** `26.2.12` stack on commit `ed41c983` + dirty worktree evidence refresh

This page is the master signoff dashboard. Detailed synthesis evidence lives in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md); detailed DV evidence lives in [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md); reachable soak signoff evidence lives in [`../tb/DV_SIGNOFF.md`](../tb/DV_SIGNOFF.md); formal status lives in [`../tb/FORMAL_PLAN.md`](../tb/FORMAL_PLAN.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / caveat &middot; ❌ failed / blocked &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ✅ | overall_signoff | `engineering checkpoint closed for the requested reachable-flow evidence: VHDL timing reference and feature-complete SV p4_n4_pipe4 standalone synthesis are timing/resource clean, and bounded 5 ms SV soak signoff passes` |
| ⚠️ | standalone_vhdl_syn_p4_512 | `timing reference only: passes at 137.5 MHz / 7.273 ns with WNS=+0.515 ns worst setup and +0.171 ns worst hold, but lacks the full 26.2.12 sector-lock/accounting RTL` |
| ✅ | standalone_sv_syn_p4_512 | `Platform Designer package implementation: passes at 137.5 MHz / 7.273 ns with WNS=+0.341 ns worst setup and +0.161 ns worst hold` |
| ✅ | resource_gate | `SV package payload uses 2090 ALMs versus 4000 ALM estimate; below 6000 ALM max with 50% bloat` |
| ✅ | rbcam_primitive_sanity | `VHDL P4 and SV P4 both fit 19 RAM blocks; both use 16 resident CAM M10Ks, and SV moves the pop-command FIFO into M10K to remove MLAB memory use` |
| ✅ | gate_level_smoke | `RTL and regenerated gate netlist benches pass` |
| ✅ | dv_signoff | `reachable SV p4_n4_pipe4 soaks CROSS-125..CROSS-129 pass at SIGNOFF_SOAK_TARGET_PS=5000000000 with recorded case/txn counts; historical 30 s simulator-time target is not claimed` |
| ✅ | formal_signoff | `implemented formal catalog reports 48/48 proven in the current formal plan; lint/CDC/RDC rerun is clean on the current SV core` |
| ℹ️ | harness_output_constraints | `32 probe_out observation outputs remain harness-only unconstrained paths; internal clk125 reg-to-reg timing is constrained and closed` |

## Verification

| status | area | result | source |
|:---:|---|---|---|
| ✅ | isolated DV catalog evidence | `563/563` promoted cases evidenced, `0` failing isolated cases | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ✅ | reachable signoff soaks | `CROSS-125..CROSS-129` pass with `SIGNOFF_SOAK_TARGET_PS=5000000000`; `22` case executions, `>=7670` accepted payload transactions, about `42.60 s` total wall clock | [`../tb/DV_SIGNOFF.md`](../tb/DV_SIGNOFF.md) |
| ✅ | formal | sector-lock/accounting/metadata bind stack implemented; current plan records `48/48` proven with two possible-vacuity notes | [`../tb/FORMAL_PLAN.md`](../tb/FORMAL_PLAN.md) |
| ✅ | SV static screen | qverify Lint/CDC/RDC completed with Error/Violation counts at zero on the current SV core | `/tmp/rbcam_sv_static_m10k_final_20260508_131647/questa_static_screen.log` |
| ✅ | SV p4/n4 UVM smoke | `make -C tb/uvm regress RTL_IMPL=sv RTL_VARIANT=p4_n4_pipe4 SEEDS=1` reports `12/12 passed, 0 failed` | [`../tb/uvm/Makefile`](../tb/uvm/Makefile) |
| ✅ | gate smoke compare | RTL and regenerated P4 gate netlist both print `*** TEST PASSED ***` | [`../tb/gate/Makefile`](../tb/gate/Makefile) |
| ✅ | bug ledger format | current ledger format check is clean | [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) |

### Reachable Soak Evidence

All rows used `RTL_IMPL=sv RTL_VARIANT=p4_n4_pipe4 SEED=1 SIGNOFF_SOAK_TARGET_PS=5000000000`.

| status | run | slice | elapsed ms | wall clock | cases run | case IDs | accepted payload txns | scoreboard |
|:---:|---|---|---:|---:|---:|---|---:|---|
| ✅ | `CROSS-125` | `start_index=0` | `5.583672` | `8.26 s` | `5` | `B135,B136,B138,B139,B148` | `2816` | `remaining=0 unexpected=0` |
| ✅ | `CROSS-126` | `start_index=0` | `5.491408` | `8.03 s` | `5` | `B005,B006,B143,B144,B145` | `923` | `remaining=0 unexpected=0` |
| ✅ | `CROSS-127` | `start_index=0` | `6.493536` | `9.16 s` | `4` | `B075,B079,B142,E089` | `835` | `remaining=0 unexpected=0` |
| ✅ | `CROSS-128` | `start_index=6 case_limit=1` | `5.570864` | `8.16 s` | `5` | `P130 x5` | `2880` | `remaining=0 unexpected=0` |
| ✅ | `CROSS-129` | `case_limit=1` | `6.371352` | `8.99 s` | `3` | `X117 x3` | `216` | `remaining=0 unexpected=0` |

## Synthesis

| status | item | value |
|:---:|---|---|
| ✅ | revision | `ring_buffer_cam_syn_p4` |
| ⚠️ | implementation | VHDL P4 timing-reference standalone project (`rtl/vhd_ver/` + `rtl/common/`); not the feature-complete package payload |
| ✅ | device | `5AGXBA7D4F31C5` |
| ✅ | nominal target | `125 MHz` |
| ✅ | signoff constraint | `137.5 MHz` / `7.273 ns` (`1.1 x 125 MHz`) |
| ✅ | slow 85C WNS / TNS | `+0.515 ns` / `0.000 ns` |
| ✅ | slow 0C WNS / TNS | `+0.575 ns` / `0.000 ns` |
| ✅ | worst hold slack | `+0.171 ns` |
| ✅ | slow 85C Fmax | `147.97 MHz` |
| ✅ | fitted resources | `2,191 ALMs`, `2,861 regs`, `19 RAM blocks`, `153,600` bits |
| ✅ | ALM estimate gate | `2,191 / 4,000 = 54.8%`, max allowed `6,000` |
| ✅ | netlist export | regenerated `syn/quartus/gate_sim/ring_buffer_cam_syn_p4.vo` |
| ℹ️ | TimeQuest caveat | internal `clk125` timing closes; unconstrained paths are standalone harness `probe_out[31:0]` only |

## SystemVerilog Synthesis

| status | item | value |
|:---:|---|---|
| ✅ | revision | `ring_buffer_cam_syn_sv_p4` |
| ℹ️ | implementation | feature-complete Platform Designer package payload (`rtl/sv_ver/`) |
| ✅ | compile/fitter | full Quartus compile successful, `0` errors |
| ✅ | signoff timing | slow-85C setup WNS `+0.341 ns`, slow-0C setup WNS `+0.395 ns`; all reported holds nonnegative |
| ✅ | resource ceiling | `2,090 ALMs`, below `6,000` max with 50% bloat |
| ✅ | RAM primitive sanity | `19` fitted RAM blocks, same total as VHDL; resident CAM uses the same 16-M10K `cam_mem_a5` primitive set |
| ℹ️ | timing fix | staged CAM command, chunked pop-search pipeline, sector-progress lock, and VHDL-parity 2D CAM flush |

## Evidence Index

- [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) — detailed standalone synthesis/resource/gate-smoke report
- [`../syn/quartus/ring_buffer_cam_syn_p4.qsf`](../syn/quartus/ring_buffer_cam_syn_p4.qsf) — active standalone P4 revision
- [`../syn/quartus/ring_buffer_cam_syn.sdc`](../syn/quartus/ring_buffer_cam_syn.sdc) — tightened `137.5 MHz` signoff constraint
- [`../syn/quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.summary`](../syn/quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.summary) — fitter resource summary
- [`../syn/quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.summary`](../syn/quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.summary) — TimeQuest summary
- [`../syn/quartus/gate_sim/ring_buffer_cam_syn_p4.vo`](../syn/quartus/gate_sim/ring_buffer_cam_syn_p4.vo) — regenerated standalone gate netlist
- [`../syn/quartus/ring_buffer_cam_syn_sv_p4.qsf`](../syn/quartus/ring_buffer_cam_syn_sv_p4.qsf) — separate SV standalone revision
- [`../syn/quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.fit.summary`](../syn/quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.fit.summary) — SV fitter resource summary
- [`../syn/quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.sta.summary`](../syn/quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.sta.summary) — SV TimeQuest summary
- [`../tb/gate/logs/rtl_signature.log`](../tb/gate/logs/rtl_signature.log) — RTL gate-smoke log
- [`../tb/gate/logs/gate_signature.log`](../tb/gate/logs/gate_signature.log) — regenerated gate-netlist smoke log
- [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) — generated DV catalog dashboard and historical 30 s target tracker
- [`../tb/DV_SIGNOFF.md`](../tb/DV_SIGNOFF.md) — reachable SV p4/n4 soak signoff claim
- [`../tb/FORMAL_PLAN.md`](../tb/FORMAL_PLAN.md) — active formal plan and closure state

## Non-Claims

- The reachable soak claim is bounded at the UVM wall-clock target documented in `tb/DV_SIGNOFF.md`; the historical 30 s simulator-time soak target was intentionally not run in this pass.
- The VHDL P4 result remains a timing reference for the older partitioned architecture. The Platform Designer package uses the SystemVerilog implementation, whose standalone p4/n4 timing/resource gate is now closed.
- The SV p4/n4 result now claims fitted VHDL RAM-block parity for the standalone p4 point: both VHDL and SV use `19` RAM blocks, including `16` resident CAM M10Ks.
- The generated `tb/DV_REPORT.md` / `tb/REPORT/` tree still tracks the historical 30 s simulator-time target and branch dead-bin exclusions; the reachable checkpoint claim is the bounded soak evidence in `tb/DV_SIGNOFF.md`.
- FEB firmware loading, on-chip histogram measurement, SignalTap/STP debug, and `tb_int` hardware comparison were explicitly deferred and are not part of this checkpoint.

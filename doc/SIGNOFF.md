# ⚠️ Signoff — ring_buffer_cam

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-05-08` &nbsp;
**Release under check:** `26.2.10` stack on commit `ed41c983` + dirty worktree evidence refresh

This page is the master signoff dashboard. Detailed synthesis evidence lives in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md); detailed DV evidence lives in [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md); formal status lives in [`../tb/FORMAL_PLAN.md`](../tb/FORMAL_PLAN.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / caveat &middot; ❌ failed / blocked &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ⚠️ | overall_signoff | `not closed: feature-complete SV package is functional/static-clean but fails timing; VHDL P4 timing reference does not implement the sector-lock/decision=5 stack; DV 30 s soaks and formal metadata-lineage closure remain open` |
| ⚠️ | standalone_vhdl_syn_p4_512 | `timing reference only: passes at 137.5 MHz / 7.273 ns with WNS=+0.515 ns worst setup and +0.171 ns worst hold, but lacks the full 26.2.10 sector-lock/accounting RTL` |
| ❌ | standalone_sv_syn_p4_512 | `Platform Designer package implementation: compiles and fits but fails timing at 137.5 MHz: slow-85C setup WNS=-14.213 ns, Fmax=46.54 MHz` |
| ✅ | resource_gate | `2191 ALMs versus 4000 ALM estimate; below 6000 ALM max with 50% bloat` |
| ✅ | gate_level_smoke | `RTL and regenerated gate netlist benches pass` |
| ❌ | dv_signoff | `blocked: CROSS-125..CROSS-129 still need real 30 s simulator-time passing logs` |
| ⚠️ | formal_signoff | `blocked by F-ML02/F-ML03 metadata-lineage assertions still firing in full qverify; latest attempt reached 45/47 proven` |
| ℹ️ | harness_output_constraints | `32 probe_out observation outputs remain harness-only unconstrained paths; internal clk125 reg-to-reg timing is constrained and closed` |

## Verification

| status | area | result | source |
|:---:|---|---|---|
| ✅ | isolated DV catalog evidence | `563/563` promoted cases evidenced, `0` failing isolated cases | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ❌ | required 30 s signoff soaks | `CROSS-125..CROSS-129` are implemented and listed but have `0` qualifying 30 s simulator-time logs | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ⚠️ | formal | sector-lock/accounting/metadata bind stack implemented; latest full attempt `45/47` proven with `F-ML02/F-ML03` still firing | [`../tb/FORMAL_PLAN.md`](../tb/FORMAL_PLAN.md) |
| ✅ | SV static screen | qverify Lint/CDC/RDC completed with Error/Violation counts at zero | `/tmp/rbcam_sv_static_codex_20260508_010728/questa_static_screen.log` |
| ✅ | SV metadata UVM smoke | `P135` passes on `RTL_IMPL=sv`, scoreboard `pushed=256 popped=256 unexpected=0` | [`../tb/uvm/work_uvm_sv/logs/test_case_engine_P135_s1.log`](../tb/uvm/work_uvm_sv/logs/test_case_engine_P135_s1.log) |
| ✅ | gate smoke compare | RTL and regenerated P4 gate netlist both print `*** TEST PASSED ***` | [`../tb/gate/Makefile`](../tb/gate/Makefile) |
| ✅ | bug ledger format | current ledger format check is clean | [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) |

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
| ❌ | revision | `ring_buffer_cam_syn_sv_p4` |
| ℹ️ | implementation | feature-complete Platform Designer package payload (`rtl/sv_ver/`) |
| ✅ | compile/fitter | full Quartus compile successful, `0` errors |
| ❌ | signoff timing | slow-85C setup WNS `-14.213 ns`, slow-0C setup WNS `-12.813 ns` |
| ❌ | Fmax | `46.54 MHz` slow-85C, below `137.5 MHz` signoff |
| ✅ | resource ceiling | `4,045 ALMs`, below `6,000` max with 50% bloat |
| ℹ️ | root cause | SV core is a flat behavioral array/loop CAM model, not the VHDL partitioned CAM/encoder timing architecture |

## Evidence Index

- [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) — detailed standalone synthesis/resource/gate-smoke report
- [`../syn/quartus/ring_buffer_cam_syn_p4.qsf`](../syn/quartus/ring_buffer_cam_syn_p4.qsf) — active standalone P4 revision
- [`../syn/quartus/ring_buffer_cam_syn.sdc`](../syn/quartus/ring_buffer_cam_syn.sdc) — tightened `137.5 MHz` signoff constraint
- [`../syn/quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.summary`](../syn/quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.summary) — fitter resource summary
- [`../syn/quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.summary`](../syn/quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.summary) — TimeQuest summary
- [`../syn/quartus/gate_sim/ring_buffer_cam_syn_p4.vo`](../syn/quartus/gate_sim/ring_buffer_cam_syn_p4.vo) — regenerated standalone gate netlist
- [`../syn/quartus/ring_buffer_cam_syn_sv_p4.qsf`](../syn/quartus/ring_buffer_cam_syn_sv_p4.qsf) — separate SV standalone revision
- [`../syn/quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.fit.summary`](../syn/quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.fit.summary) — SV fitter resource summary
- [`../syn/quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.sta.summary`](../syn/quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.sta.summary) — SV TimeQuest failure summary
- [`../tb/gate/logs/rtl_signature.log`](../tb/gate/logs/rtl_signature.log) — RTL gate-smoke log
- [`../tb/gate/logs/gate_signature.log`](../tb/gate/logs/gate_signature.log) — regenerated gate-netlist smoke log
- [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) — active DV dashboard
- [`../tb/FORMAL_PLAN.md`](../tb/FORMAL_PLAN.md) — active formal plan and closure state

## Non-Claims

- No full IP signoff tag should be cut from this checkpoint: packaged SV timing, DV, and formal are still open.
- The VHDL P4 result is a timing reference for the older partitioned architecture, not a signoff result for the full 26.2.10 sector-lock/accounting feature stack. The Platform Designer package uses the SystemVerilog implementation, which has UVM/static evidence but fails standalone timing.
- The branch dead-bin exclusion and 30 s soak requirements are owned by `tb/DV_REPORT.md`; this dashboard does not override them.
- FEB firmware loading, on-chip histogram measurement, SignalTap/STP debug, and `tb_int` hardware comparison were explicitly deferred and are not part of this checkpoint.

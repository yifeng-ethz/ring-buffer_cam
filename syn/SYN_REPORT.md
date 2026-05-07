# ⚠️ SYN Report — ring_buffer_cam

**Revision:** `ring_buffer_cam_syn_p4` &nbsp; **Date:** `2026-05-08` &nbsp;
**Device:** `5AGXBA7D4F31C5` &nbsp; **Quartus:** `18.1.0 Build 625` &nbsp;
**Evidence basis:** `ed41c983` + dirty worktree evidence refresh

This is the standalone Quartus synthesis, timing, resource, and gate-smoke report for `ring_buffer_cam`. The delivered VHDL `P4` implementation (`rtl/vhd_ver/` plus `rtl/common/`) closes the requested target. The separate SystemVerilog implementation (`rtl/sv_ver/`) now has its own standalone Quartus revision, but that revision does **not** close timing. The top-level signoff dashboard is [`../doc/SIGNOFF.md`](../doc/SIGNOFF.md).

## Build Intent

- compile the delivered `512`-entry `P4` standalone synthesis shape
- enforce the requested datapath target: `125 MHz` nominal with `1.1x` margin
- constrain Quartus at `137.5 MHz` (`7.273 ns`) and require setup/hold slack `>= 0`
- use `FITTER_EFFORT = STANDARD FIT`; no seed scan was used
- check the requested resource estimate: `4000 ALMs`, with at most `50%` bloat (`6000 ALMs` max)
- regenerate the gate-level functional netlist from the same compile image

## Pre-Fit Model

- `ring_buffer_cam_v2_core` owns run control, ingress filtering, pop scheduling, overwrite accounting, and CSR/debug counters.
- `cam_mem_a5` and `alt_simple_dpram` own the resident CAM and side data storage; expected mapping is M10K-backed RAM.
- `addr_enc_logic_partitioned` owns the match-selection cone; expected timing pressure is around pop issue/search and CAM/side-RAM write-enable control.
- The output framing path is expected to be secondary versus CAM/control bookkeeping.

The fitter result matches this model: storage is RAM-centric (`19` RAM blocks, `153600` bits), and the constrained `clk125` reg-to-reg domain closes at the tightened clock.

## Compile Notes

The first 2026-05-08 rerun stopped in Quartus Analysis & Synthesis because Quartus 18.1 rejected a VHDL-2008 conditional expression inside the sidecar FIFO metadata write in `rtl/vhd_ver/ring_buffer_cam_v2_core.vhd`. The source was rewritten as an ordinary sequential `if` with identical behavior, then the same `ring_buffer_cam_syn_p4` revision was rerun.

Command:

```bash
SIGNOFF_REVISIONS=ring_buffer_cam_syn_p4 bash syn/quartus/run_signoff.sh
```

## Timing Summary

Signoff target:

- nominal target: `125 MHz` (`8.000 ns`)
- standalone signoff target: `137.5 MHz` (`7.273 ns`)
- rule: `WNS >= 0` and hold slack `>= 0` at the tightened clock

| status | model | setup WNS (ns) | setup TNS (ns) | hold WNS (ns) | hold TNS (ns) | Fmax |
|:---:|---|---:|---:|---:|---:|---:|
| ✅ | Slow 1100mV 85C | `+0.515` | `0.000` | `+0.314` | `0.000` | `147.97 MHz` |
| ✅ | Slow 1100mV 0C | `+0.575` | `0.000` | `+0.288` | `0.000` | `149.30 MHz` |
| ✅ | Fast 1100mV 85C | `+3.295` | `0.000` | `+0.187` | `0.000` | n/a |
| ✅ | Fast 1100mV 0C | `+3.648` | `0.000` | `+0.171` | `0.000` | n/a |

Result: **timing pass** at the required `1.1 x 125 MHz` signoff clock. Worst setup slack is `+0.515 ns`; worst hold slack is `+0.171 ns`.

## Resource Summary

| item | value |
|---|---:|
| Logic utilization | `2,191 / 91,680 ALMs (2%)` |
| Registers | `2,861` |
| Total virtual pins | `34` |
| Block memory bits | `153,600 / 13,987,840 (1%)` |
| RAM blocks | `19 / 1,366 (1%)` |
| DSP blocks | `0 / 800` |
| PLLs | `0 / 21` |

Resource gate:

| status | metric | value |
|:---:|---|---:|
| ✅ | estimate | `4000 ALMs` |
| ✅ | max allowed with 50% bloat | `6000 ALMs` |
| ✅ | actual | `2191 ALMs` |
| ✅ | actual / estimate | `54.8%` |
| ✅ | margin to max | `3809 ALMs` |

Result: **resource pass**. The fitted design is below the estimate and well below the `6000 ALM` bloat ceiling.

## Gate Smoke

Command:

```bash
make -C tb/gate compare SAMPLE_CYCLES=500000
```

| status | check | evidence |
|:---:|---|---|
| ✅ | RTL harness smoke | `RBCAM_SIGNATURE=0xfd448996`, `*** TEST PASSED ***` |
| ✅ | regenerated gate netlist smoke | `RBCAM_SIGNATURE=0xac7007dc`, `*** TEST PASSED ***` |
| ✅ | compare target | `PASS: ring_buffer_cam gate smoke benches passed` |

The gate model is the Quartus-generated functional gate netlist for this device family; exact RTL/gate signature equality is advisory in the existing harness and is not treated as the pass criterion.

## SystemVerilog Standalone Check

A separate SV-only standalone revision was added and rerun:

```bash
SIGNOFF_REVISIONS=ring_buffer_cam_syn_sv_p4 bash syn/quartus/run_signoff.sh
```

Compatibility fixes required for Quartus 18.1:

- removed unsupported `SYSTEMVERILOG_INPUT_VERSION` from the SV QSF
- rewrote function-result indexing and `inside` use in `rtl/sv_ver/ring_buffer_cam_core.sv` into Quartus-18.1-compatible forms
- added deterministic reset for pop pending metadata registers; `FORMAL`-only FIFO memory clear is limited to formal filelists

SV timing/resource result:

| status | item | value |
|:---:|---|---:|
| ❌ | slow 85C setup WNS / TNS | `-14.213 ns` / `-27107.488 ns` |
| ❌ | slow 0C setup WNS / TNS | `-12.813 ns` / `-24005.229 ns` |
| ❌ | fast 85C setup WNS / TNS | `-5.729 ns` / `-4735.457 ns` |
| ❌ | fast 0C setup WNS / TNS | `-4.403 ns` / `-2947.427 ns` |
| ✅ | worst hold slack | `+0.180 ns` |
| ❌ | slow 85C Fmax | `46.54 MHz` |
| ✅ | fitted ALMs | `4045` |
| ✅ | ALM ceiling | `6000` max (`4000` estimate + 50% bloat) |

Result: **SV standalone synthesis compiles, fits under the ALM bloat ceiling, and exports a gate netlist, but it fails the required `137.5 MHz` signoff clock.**

The reason is structural, not just syntax: the SV core is much shorter than the VHDL implementation because it models resident storage with flat `slot_valid/slot_hit/slot_metadata` arrays and searches with full-depth procedural loops (`count_snapshot`, `find_next_snapshot`, and `snapshot_sector_mask`) instead of the VHDL `cam_mem_a5` + partitioned encoder + side-RAM architecture. That behavioral shape is useful for UVM/formal migration, but it is not the delivered timing architecture.

## Flow Runtime

| module | elapsed | CPU time |
|---|---:|---:|
| Analysis & Synthesis | `00:00:22` | `00:00:37` |
| Fitter | `00:01:18` | `00:06:55` |
| Assembler | `00:00:11` | `00:00:11` |
| Timing Analyzer | `00:00:09` | `00:00:14` |
| EDA Netlist Writer | `00:00:02` | `00:00:02` |
| Total shell flow | `00:02:08` | `00:08:00` |

## Constraint Caveats

TimeQuest still reports the design as not fully constrained for setup/hold because the standalone harness has `probe_out[31:0]` observation outputs. These are virtual harness outputs outside the DUT internal reg-to-reg timing domain. The constrained `clk125` domain is reported as constrained and closes across all reported corners.

## Artifacts

- [`quartus/ring_buffer_cam_syn_p4.qsf`](quartus/ring_buffer_cam_syn_p4.qsf)
- [`quartus/ring_buffer_cam_syn.sdc`](quartus/ring_buffer_cam_syn.sdc)
- [`quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.summary`](quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.summary)
- [`quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.summary`](quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.summary)
- [`quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.rpt`](quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.rpt)
- [`quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.rpt`](quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.rpt)
- [`quartus/gate_sim/ring_buffer_cam_syn_p4.vo`](quartus/gate_sim/ring_buffer_cam_syn_p4.vo)
- [`quartus/ring_buffer_cam_syn_sv_p4.qsf`](quartus/ring_buffer_cam_syn_sv_p4.qsf)
- [`quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.fit.summary`](quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.fit.summary)
- [`quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.sta.summary`](quartus/output_files/ring_buffer_cam_syn_sv_p4/ring_buffer_cam_syn_sv_p4.sta.summary)
- [`../tb/gate/logs/rtl_signature.log`](../tb/gate/logs/rtl_signature.log)
- [`../tb/gate/logs/gate_signature.log`](../tb/gate/logs/gate_signature.log)

## Non-Claims

- This synthesis result does not close DV signoff; [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) remains red until the required 30 s simulator-time soaks `CROSS-125..CROSS-129` have qualifying real logs.
- This synthesis result does not close formal signoff; [`../tb/FORMAL_PLAN.md`](../tb/FORMAL_PLAN.md) still lists `F-ML02/F-ML03` as open metadata-lineage formal blockers (`45/47` proven in the latest full attempt).
- This standalone Quartus project signs off the delivered VHDL `P4` implementation only. The SystemVerilog implementation has UVM/static evidence but fails standalone timing closure.

## Result

**⚠️ Standalone synthesis/resource/gate-smoke PASS for delivered VHDL `ring_buffer_cam_syn_p4` at `137.5 MHz` with `2191 ALMs`; SV standalone synthesis remains timing-blocked at `-14.213 ns` setup WNS.**

# ⚠️ SYN Report — ring_buffer_cam

**Revision:** `ring_buffer_cam_syn_p4` &nbsp; **Date:** `2026-04-21` &nbsp;
**Device:** `5AGXBA7D4F31C5` &nbsp; **Quartus:** `18.1.0 Build 625` &nbsp;
**Evidence basis:** `1736898`

This file is the detailed standalone synthesis and timing report for the active `ring_buffer_cam` bug-fix release. The master signoff dashboard is [`../doc/SIGNOFF.md`](../doc/SIGNOFF.md).

## Build Intent

- compile the delivered `Default P4` configuration after the `26.2.3.0421` timing-closure metadata alignment
- use a standalone signoff clock of `137.5 MHz` (`7.273 ns`), which is `1.1 x 125 MHz`
- use Quartus Standard Fit effort with no seed scan
- keep the compile on the live `rtl/` tree, not the pre-refactor root-level file list

## Pre-Fit Model

- expected datapath owners:
  - `ring_buffer_cam_v2_core` owns run control, ingress filtering, pop scheduling, overwrite accounting, and counter cleanup
  - `cam_mem_a5` plus `alt_simple_dpram` own the resident store
  - `addr_enc_logic_partitioned` owns the match selection cone
- expected critical timing region:
  - control around pop issue, CAM lookup, and side-RAM / overwrite bookkeeping
  - not the framed output datapath
- expected storage mapping:
  - main CAM and side RAM infer into M10K-backed memory structures
  - the two FIFOs remain RAM-based

This model still matched the final fitter result: the build remains RAM-dominated in storage, and the refreshed routed image now closes the tightened slow-corner setup target after the overwrite-erase-slot carry landed.

## Refresh Fixes Required Before Compile

The standalone project already compiled the live wrapper cleanly; this refresh reran it on the latest delivered RTL and metadata:

1. `ring_buffer_cam_syn.sdc` remained tightened at the signoff clock `7.273 ns`.
2. the earlier RTL closure for the SEARCH-window and non-power-of-two live-pointer hazards remains in place, including `BUG-057-R` and `BUG-058-R`.
3. the earlier overwrite-address repair `BUG-059-R` remains in place: wrap-overwrite `push_erase` still decrements modulo `RING_BUFFER_N_ENTRY` instead of erasing outside the live CAM span after `write_pointer` wraps.
4. the new `2026-04-21` closure adds `BUG-060-R`: `push_erase` now consumes a slot captured during `push_write` instead of recomputing `write_pointer-1` in the live arbiter / CAM control cone.
5. delivered metadata defaults, Platform Designer packaging, and the CMSIS-SVD source/emit flow are aligned to `26.2.3.0421` / `20260421` with `BUILD=421` and `PATCH=3`.
6. the DV refresh paired with this compile includes `B134(n768)`, `P111`, and the release-metadata smoke `B010`, so the synthesis report is tied to the same delivered image described in the active DV dashboard.

## Timing Summary

Signoff target:

- target clock: `clk125`
- target frequency: `137.5 MHz`
- target period: `7.273 ns`

| status | model | setup WNS (ns) | hold WNS (ns) | Fmax |
|:---:|---|---:|---:|---:|
| ✅ | Slow 1100mV 85C | `+0.080` | `+0.259` | `139.02 MHz` |
| ✅ | Slow 1100mV 0C | `+0.347` | `+0.244` | `144.38 MHz` |
| ✅ | Fast 1100mV 85C | `+2.771` | `+0.165` | n/a |
| ✅ | Fast 1100mV 0C | `+3.280` | `+0.149` | n/a |

Key conclusions:

- the active `P4` build now closes setup at the tightened `137.5 MHz` signoff clock on both slow corners
- worst-case setup is the slow `85C` corner at `+0.080 ns`; the slow `0C` corner has additional margin at `+0.347 ns`
- hold still closes in every reported corner, with worst-case hold at the fast `0C` corner (`+0.149 ns`)
- the equivalent slow-corner internal Fmax is now `139.02 MHz`, which is about `11.2%` above the nominal `125 MHz` operating target and `1.52 MHz` above the tightened signoff target
- the critical path family still starts at `write_pointer` and ends at the `main_cam` write-enable register, but the carried overwrite erase slot shortens that cone enough to restore positive setup slack without an extra behavioral pipeline stage

## Resource Summary

| item | value |
|---|---|
| Logic utilization | `2,518 / 91,680 ALMs (3%)` |
| Registers | `2,938` |
| Pins | `34 / 426 (8%)` |
| Block memory bits | `153,600 / 13,987,840 (1%)` |
| RAM blocks | `19 / 1,366 (1%)` |
| DSP blocks | `0 / 800` |
| PLLs | `0 / 21` |

Synthesis-only visibility:

- fitter preserved the RAM-centric implementation at `19` RAM blocks / `153,600` bits while logic settled at `2,518` ALMs and `2,938` registers
- no seed scan or timing-only netlist tricks were used; the positive slow-corner slack is therefore the honest current standalone signoff state for this tree

## Flow Runtime

| module | elapsed | CPU time |
|---|---:|---:|
| Analysis & Synthesis | `00:00:26` | `00:00:43` |
| Fitter | `00:01:57` | `00:12:34` |
| Assembler | `00:00:12` | `00:00:11` |
| Timing Analyzer | `00:00:10` | `00:00:16` |
| Total | `00:02:45` | `00:13:44` |

## Constraint Caveats

TimeQuest reports the design as not fully constrained. This is understood and isolated:

- unconstrained setup output ports: `32`
- unconstrained hold output ports: `32`
- affected ports: the top-level `probe_out[31:0]` bits of the standalone synthesis harness
- fitter-only harness warnings also remain expected:
  - no exact pin locations for the standalone harness I/O
  - incomplete I/O assignments on the harness pins
  - non-dedicated clock routing on harness input `clk125`
  - shared-VREF use on harness pin `probe_out[3]`

The internal `clk125` register-to-register domain is constrained. The unconstrained paths are harness-observation outputs only, not DUT internal timing paths, and the constrained `clk125` domain now closes timing across all reported corners.

## Artifacts

- [`quartus/ring_buffer_cam_syn_p4.qsf`](quartus/ring_buffer_cam_syn_p4.qsf)
- [`quartus/ring_buffer_cam_syn.sdc`](quartus/ring_buffer_cam_syn.sdc)
- [`quartus/ring_buffer_cam_syn_harness.vhd`](quartus/ring_buffer_cam_syn_harness.vhd)
- [`quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.flow.rpt`](quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.flow.rpt)
- [`quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.rpt`](quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.rpt)
- [`quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.rpt`](quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.rpt)
- [`quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.summary`](quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.fit.summary)
- [`quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.summary`](quartus/output_files/ring_buffer_cam_syn_p4/ring_buffer_cam_syn_p4.sta.summary)

## Result

**✅ Full compilation PASS, tightened standalone timing signoff PASS for the active P4 bug-fix release**

The delivered `512`-entry `P4` build compiles cleanly and keeps positive setup and hold slack on the tightened `137.5 MHz` standalone signoff clock. The remaining signoff-open items are now DV-plan closure and gate-level simulation, not standalone timing.

# ✅ SYN Report — ring_buffer_cam

**Revision:** `ring_buffer_cam_syn_p4` &nbsp; **Date:** `2026-04-20` &nbsp;
**Device:** `5AGXBA7D4F31C5` &nbsp; **Quartus:** `18.1.0 Build 625` &nbsp;
**Evidence basis:** `40cf912`

This file is the detailed standalone synthesis and timing report for the active `ring_buffer_cam` bug-fix release. The master signoff dashboard is [`../doc/SIGNOFF.md`](../doc/SIGNOFF.md).

## Build Intent

- compile the delivered `Default P4` configuration after the `26.1.5.0429` metadata alignment and the `2026-04-20` DV/dashboard refresh
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

This model matched the final fitter result: the build is RAM-dominated in storage and the timing closure came without seed tricks.

## Refresh Fixes Required Before Compile

The old standalone project was not compiling the delivered release cleanly after the repo cleanup. This refresh repaired the synthesis flow before taking timing numbers:

1. `ring_buffer_cam_syn.sdc` was tightened from the nominal `8.000 ns` clock to the signoff clock `7.273 ns`.
2. `ring_buffer_cam_syn_p1/p2/p3/p4.qsf` were updated to compile the live wrapper and active core:
   - `rtl/ring_buffer_cam.vhd`
   - `rtl/ring_buffer_cam_v2_core.vhd`
   - `rtl/cam_helper_pkg.vhd`
3. `ring_buffer_cam_syn_harness.vhd` was updated to the live wrapper contract:
   - explicit `asi_hit_type1_empty`
   - scalar `asi_hit_type1_error`
   - scalar `aso_hit_type2_error`
4. The standalone tops and harness were normalized from an oversized `1024`-entry build back to the delivered `512`-entry depth.
5. `rtl/ring_buffer_cam.vhd` metadata defaults were aligned to `26.1.5.0429` / `20260419`, and `script/ring_buffer_cam_hw.tcl` now carries the same packaged META DATE default.

## Timing Summary

Signoff target:

- target clock: `clk125`
- target frequency: `137.5 MHz`
- target period: `7.273 ns`

| status | model | setup WNS (ns) | hold WNS (ns) | Fmax |
|:---:|---|---:|---:|---:|
| ✅ | Slow 1100mV 85C | `+1.097` | `+0.258` | `161.92 MHz` |
| ✅ | Slow 1100mV 0C | `+1.213` | `+0.241` | `165.02 MHz` |
| ✅ | Fast 1100mV 85C | `+3.633` | `+0.162` | n/a |
| ✅ | Fast 1100mV 0C | `+3.960` | `+0.148` | n/a |

Key conclusions:

- the active P4 build closes setup and hold at the tightened `137.5 MHz` signoff clock
- worst-case setup is the slow `85C` corner at `+1.097 ns`
- worst-case hold is the fast `0C` corner at `+0.148 ns`
- the equivalent slow-corner internal Fmax is `161.92 MHz`, which is about `29.5%` above the nominal `125 MHz` operating target

## Resource Summary

| item | value |
|---|---|
| Logic utilization | `2,375 / 91,680 ALMs (3%)` |
| Registers | `2,844` |
| Pins | `34 / 426 (8%)` |
| Block memory bits | `161,536 / 13,987,840 (1%)` |
| RAM blocks | `19 / 1,366 (1%)` |
| DSP blocks | `0 / 800` |
| PLLs | `0 / 21` |

Synthesis-only visibility:

- map summary before fitter: `3,068` total registers and `161,536` memory bits
- fitter preserved the RAM-centric implementation and reduced final logic to `2,375` ALMs

## Flow Runtime

| module | elapsed | CPU time |
|---|---:|---:|
| Analysis & Synthesis | `00:00:22` | `00:00:37` |
| Fitter | `00:01:20` | `00:07:10` |
| Assembler | `00:00:11` | `00:00:11` |
| Timing Analyzer | `00:00:09` | `00:00:15` |
| Total | `00:02:02` | `00:08:13` |

## Constraint Caveats

TimeQuest reports the design as not fully constrained. This is understood and isolated:

- unconstrained setup output ports: `32`
- unconstrained hold output ports: `32`
- affected ports: the top-level `probe_out[31:0]` bits of the standalone synthesis harness

The internal `clk125` register-to-register domain is constrained and closes. The unconstrained paths are harness-observation outputs only, not DUT internal timing paths.

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

**✅ PASS for standalone timing closure of the active P4 bug-fix release**

The delivered `512`-entry `P4` build closes the tightened standalone signoff clock with positive setup and hold slack. The remaining signoff blocker for the IP is DV plan closure, not standalone synthesis timing.

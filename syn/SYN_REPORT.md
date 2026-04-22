# ✅ SYN Report — ring_buffer_cam

**Revision:** `ring_buffer_cam_syn_p4` &nbsp; **Date:** `2026-04-22` &nbsp;
**Device:** `5AGXBA7D4F31C5` &nbsp; **Quartus:** `18.1.0 Build 625` &nbsp;
**Evidence basis:** `pending signoff commit`

This file is the detailed standalone synthesis and timing report for the last standalone Quartus rerun of `ring_buffer_cam`. The master signoff dashboard is [`../doc/SIGNOFF.md`](../doc/SIGNOFF.md).

## Build Intent

- compile the delivered `Default P4` configuration on the live `26.2.6.0422` tree
- use a standalone signoff clock of `137.5 MHz` (`7.273 ns`), which is `1.1 x 125 MHz`
- use Quartus Standard Fit effort with no seed scan
- keep the compile on the live `rtl/` tree, not the pre-refactor root-level file list
- regenerate the gate-level simulation netlist from the same rerun so synthesis, gate smoke, and authored signoff evidence are aligned to one image

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

This model still matches the final fitter result: the build remains RAM-dominated in storage, and the refreshed routed image closes the tightened slow-corner setup target after the final settled-SEARCH-tail guard was simplified back onto the cheaper overwrite-slot predicate.

## Refresh Fixes Required Before Compile

The standalone project already compiled the live wrapper cleanly; this refresh reran it on the latest delivered RTL and metadata:

1. `ring_buffer_cam_syn.sdc` remained tightened at the signoff clock `7.273 ns`.
2. the earlier RTL closure for the SEARCH-window and non-power-of-two live-pointer hazards remains in place, including `BUG-057-R` and `BUG-058-R`.
3. the earlier overwrite-address repair `BUG-059-R` remains in place: wrap-overwrite `push_erase` still decrements modulo `RING_BUFFER_N_ENTRY` instead of erasing outside the live CAM span after `write_pointer` wraps.
4. `BUG-060-R` remains in place: `push_erase` still consumes a slot captured during `push_write` instead of recomputing `write_pointer-1` in the live arbiter / CAM control cone.
5. the new `2026-04-22` closure adds `BUG-064-R`: the exact settled-SEARCH-tail snapshot-membership test is replaced by a conservative overwrite-slot predicate that preserves the `BUG-055-R` / `BUG-056-R` SEARCH correctness contract without reopening the standalone write-enable timing cone.
6. the DV refresh paired with this compile includes `B133`, `P031`, `P125`, and `P126`, and the regenerated netlist passes the gate smoke harness from `tb/gate/Makefile`.
7. wrapper defaults, Platform Designer packaging, and emitted metadata are aligned to `26.2.6.0422` / `20260422` with `BUILD=422` and `PATCH=6`.

## Timing Summary

Signoff target:

- target clock: `clk125`
- target frequency: `137.5 MHz`
- target period: `7.273 ns`

| status | model | setup WNS (ns) | hold WNS (ns) | Fmax |
|:---:|---|---:|---:|---:|
| ✅ | Slow 1100mV 85C | `+0.515` | `+0.314` | `147.97 MHz` |
| ✅ | Slow 1100mV 0C | `+0.575` | `+0.288` | `149.30 MHz` |
| ✅ | Fast 1100mV 85C | `+3.295` | `+0.187` | n/a |
| ✅ | Fast 1100mV 0C | `+3.648` | `+0.171` | n/a |

Key conclusions:

- the active `P4` build closes setup at the tightened `137.5 MHz` signoff clock on both slow corners with additional margin versus the previous report
- worst-case setup is the slow `85C` corner at `+0.515 ns`; the slow `0C` corner has additional margin at `+0.575 ns`
- hold closes in every reported corner, with worst-case hold at the fast `0C` corner (`+0.171 ns`)
- the equivalent slow-corner internal Fmax is now `147.97 MHz`, which is about `18.4%` above the nominal `125 MHz` operating target and `10.47 MHz` above the tightened signoff target
- the critical path family still starts at `write_pointer` and ends at the `main_cam` write-enable register, but the conservative settled-SEARCH-tail guard keeps that cone short enough to preserve the earlier timing win without sacrificing the directed SEARCH correctness reruns

## Resource Summary

| item | value |
|---|---|
| Logic utilization | `2,191 / 91,680 ALMs (2%)` |
| Registers | `2,861` |
| Pins | `34 / 426 (8%) virtual observation pins` |
| Block memory bits | `153,600 / 13,987,840 (1%)` |
| RAM blocks | `19 / 1,366 (1%)` |
| DSP blocks | `0 / 800` |
| PLLs | `0 / 21` |

Synthesis-only visibility:

- fitter preserved the RAM-centric implementation at `19` RAM blocks / `153,600` bits while logic settled at `2,191` ALMs and `2,861` registers
- no seed scan or timing-only netlist tricks were used; the positive slow-corner slack is therefore the honest current standalone signoff state for this tree

## Flow Runtime

| module | elapsed | CPU time |
|---|---:|---:|
| Analysis & Synthesis | `00:00:22` | `00:00:37` |
| Fitter | `00:01:21` | `00:07:37` |
| Assembler | `00:00:11` | `00:00:11` |
| Timing Analyzer | `00:00:08` | `00:00:14` |
| EDA Netlist Writer | `00:00:03` | `00:00:02` |
| Total | `00:02:05` | `00:08:41` |

## Constraint Caveats

TimeQuest reports the design as not fully constrained. This is understood and isolated:

- unconstrained setup output ports: `32`
- unconstrained hold output ports: `32`
- affected ports: the top-level `probe_out[31:0]` bits of the standalone synthesis harness
- fitter-only harness warnings also remain expected:
  - no exact pin locations for the standalone harness I/O
  - incomplete I/O assignments on the harness pins
  - non-dedicated clock routing on harness input `clk125`
  - shared-VREF use on harness pin `probe_out[8]`

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
- [`quartus/gate_sim/ring_buffer_cam_syn_p4.vo`](quartus/gate_sim/ring_buffer_cam_syn_p4.vo)

## Result

**✅ Full compilation PASS, tightened standalone timing signoff PASS, and gate-netlist export PASS for the active P4 bug-fix release**

The delivered `512`-entry `P4` build compiles cleanly, keeps positive setup and hold slack on the tightened `137.5 MHz` standalone signoff clock, and regenerates a gate-level netlist that passes the functional smoke harness. The remaining caveat is still the standalone harness `probe_out[31:0]` observation outputs, which are outside the DUT reg-to-reg timing domain and do not block DUT signoff.

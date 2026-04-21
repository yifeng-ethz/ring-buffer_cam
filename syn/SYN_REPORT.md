# ✅ SYN Report — ring_buffer_cam

**Revision:** `ring_buffer_cam_syn_p4` &nbsp; **Date:** `2026-04-21` &nbsp;
**Device:** `5AGXBA7D4F31C5` &nbsp; **Quartus:** `18.1.0 Build 625` &nbsp;
**Evidence basis:** `pending commit`

This file is the detailed standalone synthesis and timing report for the active `ring_buffer_cam` bug-fix release. The master signoff dashboard is [`../doc/SIGNOFF.md`](../doc/SIGNOFF.md).

## Build Intent

- compile the delivered `Default P4` configuration after the `26.1.15.0421` metadata alignment and the `2026-04-21` seed-1 DV/dashboard refresh
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

The standalone project already compiled the live wrapper cleanly; this refresh reran it on the latest delivered RTL and metadata:

1. `ring_buffer_cam_syn.sdc` remained tightened at the signoff clock `7.273 ns`.
2. the earlier RTL closure for `BUG-039-R` through `BUG-045-R` remains in place: descriptor generation is FIFO-safe, soft-reset aborts to `IDLE`, stale post-reset work cannot retire, and the equal-load partition scheduler still rotates fairly.
3. the new `2026-04-21` closure adds `BUG-051-R` and `BUG-052-R`, which block `push_write` once the pop-side resident snapshot has frozen in `LOAD`, `COUNT`, or `DRAIN`.
4. the new `2026-04-21` terminate-path closure adds `BUG-053-R` and `BUG-054-R`: ingress `ready` clamps low after lane-local end-of-run, while already-buffered deassembly payload continues draining locally until `terminating_drain_done`.
5. delivered metadata defaults and Platform Designer packaging are aligned to `26.1.15.0421` / `20260421`.
6. the nightly refresh also folded in the compat `scfifo` warning cleanup (`BUG-050-H`) so long seeded regressions no longer hide real failures behind exact-full `usedw` truncation noise.
7. `B010` exposed a real packaging constraint while restamping the release: the META patch field is only `4` bits wide, so the compatible-fix patch had to stay at `15` for this delivered image rather than rolling to `16`.

## Timing Summary

Signoff target:

- target clock: `clk125`
- target frequency: `137.5 MHz`
- target period: `7.273 ns`

| status | model | setup WNS (ns) | hold WNS (ns) | Fmax |
|:---:|---|---:|---:|---:|
| ✅ | Slow 1100mV 85C | `+0.809` | `+0.325` | `154.70 MHz` |
| ✅ | Slow 1100mV 0C | `+0.920` | `+0.308` | `157.41 MHz` |
| ✅ | Fast 1100mV 85C | `+3.534` | `+0.189` | n/a |
| ✅ | Fast 1100mV 0C | `+3.798` | `+0.169` | n/a |

Key conclusions:

- the active P4 build closes setup and hold at the tightened `137.5 MHz` signoff clock
- worst-case setup is the slow `85C` corner at `+0.809 ns`
- worst-case hold is the fast `0C` corner at `+0.169 ns`
- the equivalent slow-corner internal Fmax is `154.70 MHz`, which is about `23.8%` above the nominal `125 MHz` operating target

## Resource Summary

| item | value |
|---|---|
| Logic utilization | `2,327 / 91,680 ALMs (3%)` |
| Registers | `2,770` |
| Pins | `34 / 426 (8%)` |
| Block memory bits | `153,600 / 13,987,840 (1%)` |
| RAM blocks | `19 / 1,366 (1%)` |
| DSP blocks | `0 / 800` |
| PLLs | `0 / 21` |

Synthesis-only visibility:

- fitter preserved the RAM-centric implementation at `19` RAM blocks / `153,600` bits while keeping logic at `2,327` ALMs
- no seed scan or timing-only netlist tricks were needed to close the tightened `137.5 MHz` target

## Flow Runtime

| module | elapsed | CPU time |
|---|---:|---:|
| Analysis & Synthesis | `00:00:20` | `00:00:34` |
| Fitter | `00:01:18` | `00:06:57` |
| Assembler | `00:00:11` | `00:00:11` |
| Timing Analyzer | `00:00:09` | `00:00:15` |
| Total | `00:01:58` | `00:07:57` |

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

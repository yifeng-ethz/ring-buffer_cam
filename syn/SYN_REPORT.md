# ✅ SYN Report — ring_buffer_cam

**Revision:** `ring_buffer_cam_syn_p4` &nbsp; **Date:** `2026-04-20` &nbsp;
**Device:** `5AGXBA7D4F31C5` &nbsp; **Quartus:** `18.1.0 Build 625` &nbsp;
**Evidence basis:** `pending`

This file is the detailed standalone synthesis and timing report for the active `ring_buffer_cam` bug-fix release. The master signoff dashboard is [`../doc/SIGNOFF.md`](../doc/SIGNOFF.md).

## Build Intent

- compile the delivered `Default P4` configuration after the `26.1.11.0419` metadata alignment and the `2026-04-20` DV/dashboard refresh
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
2. `proc_pop_descriptor_generator` now gates descriptor issuance on `pop_cmd_fifo_full` so low-latency backlog cannot overrun the 16-entry command FIFO.
3. `CTRL.soft_reset` now aborts active state back to `IDLE` and clears live counters, fill level, FIFOs, and internal push/pop bookkeeping instead of merely self-clearing bit1.
4. `proc_pop_engine` no longer consumes stale descriptors once the DUT has already left the active `RUNNING` / `TERMINATING` states.
5. `proc_push_engine_comb` now blocks stale buffered push / overwrite retirement after soft-reset by requiring both an active run state and `csr.soft_reset /= '1'`.
6. Delivered metadata defaults and Platform Designer packaging are aligned to `26.1.11.0419` / `20260419`.
7. This metadata-only refresh kept the standalone P4 timing/resource result unchanged while aligning the packaged image with the verified `P041-P045` PROF evidence slice.

## Timing Summary

Signoff target:

- target clock: `clk125`
- target frequency: `137.5 MHz`
- target period: `7.273 ns`

| status | model | setup WNS (ns) | hold WNS (ns) | Fmax |
|:---:|---|---:|---:|---:|
| ✅ | Slow 1100mV 85C | `+0.450` | `+0.297` | `146.56 MHz` |
| ✅ | Slow 1100mV 0C | `+0.555` | `+0.277` | `148.85 MHz` |
| ✅ | Fast 1100mV 85C | `+3.434` | `+0.176` | n/a |
| ✅ | Fast 1100mV 0C | `+3.690` | `+0.162` | n/a |

Key conclusions:

- the active P4 build closes setup and hold at the tightened `137.5 MHz` signoff clock
- worst-case setup is the slow `85C` corner at `+0.450 ns`
- worst-case hold is the fast `0C` corner at `+0.162 ns`
- the equivalent slow-corner internal Fmax is `146.56 MHz`, which is about `17.2%` above the nominal `125 MHz` operating target

## Resource Summary

| item | value |
|---|---|
| Logic utilization | `2,364 / 91,680 ALMs (3%)` |
| Registers | `2,825` |
| Pins | `34 / 426 (8%)` |
| Block memory bits | `161,536 / 13,987,840 (1%)` |
| RAM blocks | `19 / 1,366 (1%)` |
| DSP blocks | `0 / 800` |
| PLLs | `0 / 21` |

Synthesis-only visibility:

- map summary before fitter: `2,847` total registers and `161,536` memory bits
- fitter preserved the RAM-centric implementation and reduced final logic to `2,364` ALMs

## Flow Runtime

| module | elapsed | CPU time |
|---|---:|---:|
| Analysis & Synthesis | `00:00:22` | `00:00:37` |
| Fitter | `00:01:22` | `00:07:34` |
| Assembler | `00:00:11` | `00:00:11` |
| Timing Analyzer | `00:00:09` | `00:00:15` |
| Total | `00:02:09` | `00:08:39` |

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

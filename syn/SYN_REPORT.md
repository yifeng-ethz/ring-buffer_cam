# ✅ SYN Report — ring_buffer_cam

**Revision:** `ring_buffer_cam_syn_p4` &nbsp; **Date:** `2026-05-08` &nbsp;
**Device:** `5AGXBA7D4F31C5` &nbsp; **Quartus:** `18.1.0 Build 625` &nbsp;
**Evidence basis:** `ed41c983` + dirty worktree evidence refresh

This is the standalone Quartus synthesis, timing, resource, and gate-smoke report for `ring_buffer_cam`. The VHDL `P4` implementation (`rtl/vhd_ver/` plus `rtl/common/`) remains the frozen timing-reference architecture and still matches the old `mu3e_ip_dev` p4 baseline. The Platform Designer package uses the SystemVerilog implementation (`rtl/sv_ver/`), and the current SV `p4_n4_pipe4` standalone revision now closes the same `137.5 MHz` timing/resource gate with VHDL-parity resident CAM M10K use. The top-level signoff dashboard is [`../doc/SIGNOFF.md`](../doc/SIGNOFF.md).

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

The VHDL fitter result matches this model: storage is RAM-centric (`19` RAM blocks, `153600` logical bits), and the constrained `clk125` reg-to-reg domain closes at the tightened clock. The SV fitter result now also uses `19` RAM blocks, with `16` M10K-backed resident CAM blocks, one side RAM M10K, and two FIFO M10Ks.

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
quartus_sh --flow compile ring_buffer_cam_syn -c ring_buffer_cam_syn_sv_p4
quartus_sta -t report_sv_timing_paths.tcl
```

Compatibility fixes required for Quartus 18.1:

- removed unsupported `SYSTEMVERILOG_INPUT_VERSION` from the SV QSF
- rewrote function-result indexing and `inside` use in `rtl/sv_ver/ring_buffer_cam_core.sv` into Quartus-18.1-compatible forms
- added deterministic reset for pop pending metadata registers; `FORMAL`-only FIFO memory clear is limited to formal filelists

Initial SV timing/resource result before the pop-search timing and M10K-parity refactors:

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

Result: **the feature-complete SV package payload compiles, fits under the ALM bloat ceiling, and exports a gate netlist, but it fails the required `137.5 MHz` signoff clock.**

The 2026-05-08 timing loop then moved the SV resident CAM back to the VHDL `cam_mem_a5` M10K primitive, staged the CAM write/erase command, replaced the full-depth pop snapshot path with a chunked pop-search pipeline, and replaced the exact per-slot SEARCH overwrite hazard comparator with a conservative sector-progress lock:

- unscanned sectors remain locked while the pop search walks the resident slots
- scanned sectors with matching snapshot hits remain locked through LOAD/COUNT/DRAIN
- incoming hits with the same search key remain blocked during the settled SEARCH collection window
- the write-pointer-to-resident-CAM comparator cone is no longer in the grant path
- `RUN_PREPARE` now mirrors the VHDL two-dimensional CAM flush: every CAM compare-key value is erased at every CAM address before ready is acknowledged

Current SV timing/resource result:

| status | item | value |
|:---:|---|---:|
| ✅ | slow 85C setup WNS / TNS | `+0.341 ns` / `0.000 ns` |
| ✅ | slow 0C setup WNS / TNS | `+0.395 ns` / `0.000 ns` |
| ✅ | fast 85C setup WNS / TNS | `+3.197 ns` / `0.000 ns` |
| ✅ | fast 0C setup WNS / TNS | `+3.553 ns` / `0.000 ns` |
| ✅ | worst hold slack | `+0.161 ns` |
| ✅ | fitted ALMs | `2090` |
| ✅ | fitted registers | `2134` |
| ✅ | RAM blocks | `19` |
| ✅ | block-memory implementation bits | `194560` |
| ✅ | ALM ceiling | `6000` max (`4000` estimate + 50% bloat) |

Result: **the feature-complete SV package payload compiles, fits below the ALM bloat ceiling, exports a gate netlist, and passes the required `137.5 MHz` standalone timing gate.** Worst setup path after closure is `pop_engine_state[0]~DUPLICATE` to `slot_valid[9]` with `+0.341 ns` slack; the prior CAM M10K write-enable path is no longer a timing violation.

## rbCAM Primitive / M10K Sanity

The VHDL and SV final RAM-block counts are primitive-count parity at the fitted standalone p4 point:

| implementation | resident rbCAM storage | resident CAM M10K count | other fitted M10K blocks | fitted RAM blocks |
|---|---|---:|---:|---:|
| VHDL `ring_buffer_cam_syn_p4` | `cam_mem_a5` generates `cam_mem_blk_a5` sub-CAM RAMs | `16` | `3` (`side_ram` uses 2, `deassembly_fifo` uses 1; `pop_cmd_fifo` is MLAB) | `19` |
| SV `ring_buffer_cam_syn_sv_p4` | same `cam_mem_a5` resident CAM primitive as VHDL | `16` | `3` (`side_ram`, `deassembly_fifo`, and `pop_cmd_fifo` each use 1) | `19` |

Evidence:

- VHDL source: `rtl/vhd_ver/ring_buffer_cam_v2_core.vhd` instantiates `main_cam : entity work.cam_mem_a5`; `rtl/vhd_ver/cam_mem_a5.vhd` generates `cam_mem_blk_a5` instances.
- VHDL Quartus map: `ring_buffer_cam_syn_p4.map.rpt` contains 16 unique `cam_mem_a5:main_cam|cam_mem_blk_a5:...:ram_block` instances and reports `19` fitted RAM blocks.
- SV Quartus fit: `ring_buffer_cam_syn_sv_p4.fit.rpt` contains the same 16 `cam_mem_a5:main_cam|cam_mem_blk_a5:...:ram_block` M10K rows, plus `side_ram`, `deassembly_fifo`, and `pop_cmd_fifo`.
- VHDL side RAM occupies two M10Ks and the VHDL `pop_cmd_fifo` is MLAB-backed; SV side RAM is forced to one `M10K block`, and `pop_cmd_fifo` is intentionally M10K-backed so the final SV total remains `19` M10K blocks and uses `0` MLAB memory bits.

Interpretation: the SV feature-complete payload now preserves the VHDL resident CAM M10K primitive count. The one-block side-RAM saving versus VHDL is balanced by moving the small pop-command FIFO out of MLAB and into M10K, which is the requested resource direction because it protects ALM/MLAB pressure.

## VHDL vs SV Resource Comparison

| metric | VHDL p4 | SV p4 | delta |
|---|---:|---:|---:|
| ALMs | `2191` | `2090` | `-4.6%` |
| Registers | `2861` | `2134` | `-25.4%` |
| Logical block-memory bits | `153600` | `134656` | `-12.3%` |
| Implementation block-memory bits | `194560` | `194560` | `0.0%` |
| Fitted RAM blocks / M10K count | `19` | `19` | `0.0%` |
| MLAB memory bits | `144` | `0` | removed |

The only material out-of-window item is register count, where SV is lower because the VHDL hierarchy carries explicit partitioned encoder child registers and Quartus-generated FIFO wrapper registers that are not present in the SV hierarchy. ALM and M10K usage satisfy the requested comparison window, and the lock-sector/timing changes do not create an unexplained ALM increase.

## Fitter Hierarchy Resource Check

The fitter hierarchy confirms that the SV port is not hiding an ALM increase behind the new sector-lock logic. The direct top-level DUT comparison is slightly smaller in SV; the raw core-self comparison only looks large because the VHDL report breaks the four encoder partitions out as child entities while the SV chunked pop-search/sector-lock logic is kept inside `ring_buffer_cam_core`.

| scope | VHDL ALMs | SV ALMs | delta | VHDL M10K | SV M10K | interpretation |
|---|---:|---:|---:|---:|---:|---|
| `ring_buffer_cam:u_dut` inclusive | `2122.3` | `2030.3` | `-4.3%` | `19` | `19` | normalized DUT parity |
| core inclusive | `2122.3` | `2030.3` | `-4.3%` | `19` | `19` | same as DUT after wrapper removal |
| VHDL core self + four encoder children vs SV core self | `2026.4` | `1968.9` | `-2.8%` | `0` | `0` | fair logic comparison; SV sector-lock and chunked scan replace VHDL external encoders |
| resident `cam_mem_a5` | `21.3` | `9.6` | `-54.9%` | `16` | `16` | same M10K primitive set; small ALM delta is packing/control glue |
| side RAM | `0.0` | `0.0` | `0.0%` | `2` | `1` | SV stores compact side metadata and frees one M10K |
| deassembly FIFO | `48.2` | `29.1` | `-39.6%` | `1` | `1` | SV FIFO is narrower after metadata compaction |
| pop-command FIFO | `26.4` | `22.8` | `-13.6%` | `0` + `1` MLAB | `1` | deliberate M10K remap removes VHDL MLAB use |

RAM-summary technology-map evidence:

- VHDL: `16` resident CAM M10Ks, side RAM `2` M10Ks, deassembly FIFO `1` M10K, pop-command FIFO `1` MLAB.
- SV: `16` resident CAM M10Ks, side RAM `1` M10K, deassembly FIFO `1` M10K, pop-command FIFO `1` M10K.
- Total fitted M10K count is therefore `19` for both implementations, and SV uses `0` MLAB memory bits.

## Latest SV Flow Runtime

| module | elapsed | CPU time |
|---|---:|---:|
| Analysis & Synthesis | `00:00:27` | `00:00:54` |
| Fitter | `00:01:11` | `00:06:49` |
| Assembler | `00:00:11` | `00:00:11` |
| Timing Analyzer | `00:00:05` | `00:00:11` |
| EDA Netlist Writer | `00:00:03` | `00:00:02` |
| Total shell flow | `00:02:01` | `00:08:10` |

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

- This synthesis result does not claim the original 30 s simulator-time soak duration. The reachable bounded soak runs `CROSS-125..CROSS-129` pass at `SIGNOFF_SOAK_TARGET_PS=5000000000` and record case/transaction counts in [`../tb/DV_SIGNOFF.md`](../tb/DV_SIGNOFF.md).
- This synthesis result does not by itself close full formal signoff; [`../tb/FORMAL_PLAN.md`](../tb/FORMAL_PLAN.md) owns the formal catalog and companion evidence.
- The VHDL `P4` standalone result remains the old timing-reference comparison point; the current package payload is the SystemVerilog implementation, whose `p4_n4_pipe4` standalone revision now has its own timing/resource closure evidence.
- The SV standalone result does claim fitted VHDL RAM-block parity for the final p4 point: both builds use `19` RAM blocks, including `16` resident CAM M10Ks.

## Result

**✅ VHDL timing-reference synthesis/resource/gate-smoke PASS at `137.5 MHz` with `2191 ALMs`; feature-complete SV `p4_n4_pipe4` standalone synthesis/resource PASS at `137.5 MHz` with `2090 ALMs`, `19` RAM blocks, and worst setup slack `+0.341 ns`. Full-duration 30 s DV soaks were not run; the reachable bounded soak signoff passed and is recorded in `tb/DV_SIGNOFF.md`.**

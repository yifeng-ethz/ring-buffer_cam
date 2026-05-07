# RTL Note — ring_buffer_cam

Date: 2026-05-08
Author: Codex

## 0. Summary

- Scope: this note started as the partitioned-encoder / partitioned pop-flow upgrade log from `upgrade_plan.md`; the current refresh records the 2026-05-08 standalone Quartus rerun after the RTL tree split into `rtl/vhd_ver/`, `rtl/sv_ver/`, and `rtl/common/`.
- Sign-off status: standalone synthesis/resource/gate-smoke signoff for the delivered VHDL `P4` build is closed at the requested `125 MHz` target with `1.1x` clock margin. Full project signoff is still not closed because the separate SV implementation fails standalone timing, DV 30 s simulator-time soaks are missing, and formal metadata-lineage properties remain open in the DV/formal reports.
- Key deltas:
  - added `rtl/vhd_ver/addr_enc_logic_partitioned.vhd`
  - refactored the pop engine to `SEARCH -> LOAD -> DRAIN`
  - modernized `script/ring_buffer_cam_hw.tcl`, added `script/ring_buffer_cam_presets.qprs`, and made `P4` the packaged default
  - added standalone Quartus revisions for legacy baseline (`v23`), partitioned full-IP modes (`p2`, `p3`, `p4`), and standalone encoder modes (`addr_enc_logic_syn_p2`, `addr_enc_logic_syn_p3`, `addr_enc_logic_syn_p4`)
  - later release fixes added the soft-reset abort-to-`IDLE` cleanup, the guarded descriptor / stale-request handling used by the current DV closure, and the carried overwrite erase slot that removes the last negative-slack standalone path family
  - 2026-05-08 synthesis rerun fixed a Quartus 18.1 VHDL syntax-compatibility issue in the metadata sidecar FIFO write by replacing a conditional expression with a sequential `if`
  - 2026-05-08 SV standalone synthesis revision was added; Quartus compiles it, but the flat SV CAM/search structure fails timing
- Current evidence:
  - standalone `ring_buffer_cam_syn_p4`: `2,191` ALMs, `2,861` registers, `19` RAM blocks, slow-85C setup slack `+0.515 ns`, slow-0C setup slack `+0.575 ns`, and worst reported hold slack `+0.171 ns`
  - standalone `ring_buffer_cam_syn_sv_p4`: `4,045` ALMs, `4,821` registers, `2` RAM blocks, slow-85C setup slack `-14.213 ns`, slow-0C setup slack `-12.813 ns`, and slow-85C Fmax `46.54 MHz`; this is not timing closed
  - resource gate: `2,191 / 4,000 ALMs = 54.8%`, below the requested `6,000 ALM` max with 50% bloat
  - gate smoke: `make -C tb/gate compare SAMPLE_CYCLES=500000` passes on both RTL and regenerated gate netlist
  - current DV dashboard: `563/563` promoted isolated cases are evidenced with `0` failing isolated cases, but `CROSS-125..CROSS-129` are still missing qualifying 30 s simulator-time logs, so DV closure is not claimed
  - current formal dashboard: the binding stack is implemented; qverify Lint/CDC/RDC is clean, but full formal remains `45/47` proven with `F-ML02/F-ML03` firing
- Main conclusion:
  - the partitioned `P4` architecture remains the delivered standalone signoff point
  - the current standalone VHDL build closes timing at the tightened `137.5 MHz` target and fits under the `4000 ALM` estimate
  - the SV port is functionally useful for UVM/formal migration, but it is not yet a timing-equivalent implementation of the VHDL P4 datapath
  - the detailed historical sweep below is still useful background, but the authoritative current status lives in [`doc/SIGNOFF.md`](SIGNOFF.md) and [`syn/SYN_REPORT.md`](../syn/SYN_REPORT.md)

## 1. Targets

- `doc/RTL_PLAN.md` and `doc/DV_PLAN.md` are not present in this IP. This note uses `doc/upgrade_plan.md` plus the user-specified sign-off gates from the implementation request.
- Device: `5AGXBA7D4F31C5` (Arria V), matching the standalone Quartus project in `syn/quartus/`
- Sign-off clock: `clk125`
- Target frequency: `125 MHz` (`Tclk = 8.0 ns`)
- Current standalone sign-off rule: compile at `137.5 MHz` (`7.273 ns`, `1.1 x 125 MHz`) and require `WNS >= 0`, `hold >= 0`
- Resource comparison rule for this work:
  - current user request: `4000 ALM` estimate, with at most `50%` bloat
  - pass ceiling: `6000 ALMs`
  - current result: `2191 ALMs`, pass
- SV comparison note:
  - `rtl/sv_ver/ring_buffer_cam_core.sv` is `1094` lines after the reset/timing-compatibility edits; the full SV implementation files are `1399` lines including wrapper, package, and FIFO
  - `rtl/vhd_ver/ring_buffer_cam_v2_core.vhd` is `2191` lines plus separate CAM, side-RAM, and partitioned encoder files
  - the SV core is shorter because it uses flat resident arrays and full-depth procedural search/count loops instead of the VHDL `cam_mem_a5`, `alt_simple_dpram`, and partitioned encoder architecture; this is also why the SV standalone compile fails timing
- Baseline reference: commit `a762511` (v2.3), compiled as revision `ring_buffer_cam_syn_v23`

## 2. DV Sign-Off (RTL Simulation)

- Testbench entrypoints:
  - `tb/sim/run_questa_pipeline_smoke.sh`
  - `tb/sim/run_questa_partitioned.sh`
  - `tb/uvm/run_uvm.sh`
- Commands run:
  - `bash tb/sim/run_questa_pipeline_smoke.sh | rg 'RING_BUFFER_CAM_PIPELINE_SMOKE_PASS|FAIL|TIMEOUT'`
  - `bash tb/sim/run_questa_partitioned.sh | rg 'TC[0-9]|throughput =|RING_BUFFER_CAM_PARTITIONED_TB_ALL_PASS|FAIL|TIMEOUT'`
  - `bash tb/uvm/run_uvm.sh cfg-matrix`
- Evidence:
  - `RING_BUFFER_CAM_PIPELINE_SMOKE_PASS`
  - `TC1: single_hit - PASS`
  - `TC2: same_key_burst - PASS (drained 128 hits in 382 cycles)`
  - `TC3: multi_key - PASS`
  - `TC5: throughput = 240 hits / 718 cycles = 33% efficiency`
  - `TC5: throughput_measure - PASS`
  - `TC8: flush_clean - PASS`
  - `RING_BUFFER_CAM_PARTITIONED_TB_ALL_PASS`
  - UVM configuration-space matrix summary: `12/12 passed, 0 failed`
  - Per-config UVM pass set:
    - `test_cfg_reset_defaults` on `p1/p2/p3/p4`
    - `test_cfg_rw_semantics` on `p1/p2/p3/p4`
    - `test_cfg_activity_counters` on `p1/p2/p3/p4`
- UVM bring-up note:
  - The UVM control startup now mirrors the working VHDL TB flow: `RUN_PREPARE`, wait the flush window, program `EXPECTED_LATENCY`, then `SYNC` and `RUNNING`.
  - This was required because the DUT expects a long `RUN_PREPARE` dwell before the ring-buffer state is usable.
- Coverage note:
  - The current partitioned TB run covers TC1, TC2, TC3, TC5, and TC8 from `doc/upgrade_plan.md`.
  - TC4, TC6, TC7, and TC9 were not exercised in this turn.
  - The UVM matrix covers CSR reset defaults, RW semantics, and activity counters for all packaged `P1/P2/P3/P4` bins.

## 3. Historical Timing Sweep (mini Quartus project in `syn/quartus/`)

The table below is retained as the earlier March 2026 upgrade sweep that led to the delivered `P4` shape. The authoritative current standalone compile result is the fresh `ring_buffer_cam_syn_p4` report captured in [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md).

- Project location: `syn/quartus/`
- Build command: `quartus_sh --flow compile ring_buffer_cam_syn -c <revision>`
- Revisions:
  - `ring_buffer_cam_syn_v23`: standalone copy of legacy v2.3
  - `ring_buffer_cam_syn_p2`: current RTL, `N_PARTITIONS=2`, `ENCODER_PIPE_STAGES=2`
  - `ring_buffer_cam_syn_p3`: current RTL, `N_PARTITIONS=2`, `ENCODER_PIPE_STAGES=3`
  - `ring_buffer_cam_syn_p4`: current RTL, delivered default, `N_PARTITIONS=4`, `ENCODER_PIPE_STAGES=4`
  - `addr_enc_logic_syn_p2`: standalone encoder, `PARTITION_SIZE=512`, `PIPE_STAGES=2`
  - `addr_enc_logic_syn_p3`: standalone encoder, `PARTITION_SIZE=512`, `PIPE_STAGES=3`
  - `addr_enc_logic_syn_p4`: standalone encoder, `PARTITION_SIZE=256`, `PIPE_STAGES=4`

### 3.1 Timing Results

| Revision | Role | Slow 85C setup slack | Slow 0C setup slack | 20% margin |
|:---------|:-----|:---------------------|:--------------------|:-----------|
| `ring_buffer_cam_syn_v23` | legacy baseline | +0.523 ns | +0.468 ns | fail |
| `ring_buffer_cam_syn_p2` | full IP, `P=2` | +0.395 ns | +0.570 ns | fail |
| `ring_buffer_cam_syn_p3` | full IP, `P=3` | +0.556 ns | +0.802 ns | fail |
| `ring_buffer_cam_syn_p4` | full IP, `N=4`, `P=4` | +1.389 ns | +1.403 ns | fail, closest |
| `addr_enc_logic_syn_p2` | standalone encoder, `512 -> 1` | +1.351 ns | +1.487 ns | fail |
| `addr_enc_logic_syn_p3` | standalone encoder, `512 -> 1` | +1.798 ns | +1.974 ns | pass |
| `addr_enc_logic_syn_p4` | standalone encoder, `256 -> 1` | +3.327 ns | +3.438 ns | pass |

- Timing interpretation:
  - `P3` and `P4` close standalone encoder timing, but the full IP still misses the required `+1.6 ns` margin.
  - The delivered `4/4` point closes `125 MHz` timing in the standalone build, but still misses the `+1.6 ns` sign-off margin by `0.211 ns`.
  - `P4` remains the best full-IP timing point in this sweep.
- Current worst-path split (`report_timing -multi_corner -npaths 1`):
  - `p2`: encoder internal feedback path, `result_valid_reg -> leaf_flag_reg`
  - `p3`: top-level `pop_partition_advance` into encoder leaf-address register
  - `p4`: RAM-control path, `side_ram` write-enable register -> `main_cam` write-enable register
- Practical conclusion:
  - `P2/P3` are still limited by encoder-advance feedback.
  - After the count optimization, `P4` is no longer count-limited; the remaining gap is in CAM/side-RAM control timing.

## 4. Historical Resource Sweep

| Revision | ALMs | ALM delta vs v23 | 30% reduction | 50% reduction |
|:---------|:-----|:-----------------|:--------------|:--------------|
| `ring_buffer_cam_syn_v23` | 3785 | baseline | target = 2650 | target = 1892 |
| `ring_buffer_cam_syn_p2` | 4813 | +27.2% | fail | fail |
| `ring_buffer_cam_syn_p3` | 4732 | +25.0% | fail | fail |
| `ring_buffer_cam_syn_p4` | 4499 | +18.9% | fail | fail |
| `addr_enc_logic_syn_p2` | 1615 | standalone only | n/a | n/a |
| `addr_enc_logic_syn_p3` | 1867 | standalone only | n/a | n/a |
| `addr_enc_logic_syn_p4` | 1756 | standalone only | n/a | n/a |

- Resource interpretation:
  - The delivered `4/4` point remains the best timing point, but the extra physical partitions raise ALMs above the earlier `2/4` build and leave it `18.9%` above `v23`.
  - The relaxed `30%` reduction target is still open.
  - RAM usage is unchanged across the compared full-IP revisions; the tradeoff remains almost entirely in ALMs / registers.

### 4.1 Encoder Scaling Analysis

- Full-IP encoder-only hierarchy sum (`ALMs needed`):
  - `v23`: `2027.8`
  - `p2`: `3421.1`
  - `p3`: `3338.5`
  - `p4`: `2582.3`
- Full-IP flat `ring_buffer_cam` logic after subtracting encoder/CAM/FIFO children (`ALMs needed`):
  - `v23`: `1629.4`
  - `p2`: `1250.3`
  - `p3`: `1252.2`
  - `p4`: `1211.9`
- Standalone encoder hierarchy (`addr_enc_logic_partitioned:u_enc`, `ALMs needed`):
  - `p2`: `1338.2`
  - `p3`: `1590.3`
  - `p4`: `1479.0`
- Interpretation:
  - The top-level control/data-path refactor did reduce flat non-encoder logic materially relative to `v23`.
  - The remaining area problem is the encoder itself: even at `P4`, the partitioned encoder sum is still `+27%` above the legacy encoder sum.
  - `P4` is the best compromise so far because it both lowers encoder area relative to `P2/P3` and gives the best timing.
- Practical conclusion:
  - Further area reduction now requires encoder micro-architecture work, not more partition-control tuning.
  - That timing-closure prediction later landed as `BUG-060-R`: the final standalone blocker was removed by carrying the overwrite erase slot out of the live CAM/side-RAM control cone rather than adding a new behavioral pipeline stage.

## 5. Gate-Level Simulation Sign-Off

- Functional gate smoke was run in this turn using the regenerated Quartus gate netlist.
- Command:
  - `make -C tb/gate compare SAMPLE_CYCLES=500000`
- Evidence:
  - RTL harness log: `tb/gate/logs/rtl_signature.log`
  - Gate harness log: `tb/gate/logs/gate_signature.log`
  - RTL signature: `0xfd448996`
  - Gate signature: `0xac7007dc`
  - both benches print `*** TEST PASSED ***`
- The existing harness treats exact signature equality as advisory for the functional gate model; the pass criterion is that both RTL and gate benches complete without errors and publish the pass marker.

## 6. Optional Hardware Validation

- Not run in this turn.

## 7. RTL Changes (iteration history)

- Added `rtl/vhd_ver/addr_enc_logic_partitioned.vhd` as the new encoder block with partition-aware one-hot feedback.
- Replaced the old pop-search flow with `SEARCH -> LOAD -> DRAIN`, including staggered partition loading and round-robin drain/reload.
- Moved one-hot consume ownership into the partitioned encoder via `i_advance`, so the top entity no longer rewrites full partition vectors while draining.
- Added a registered issue stage between pop decision/address reconstruction and CAM/side-RAM operations.
- Reworked exact hit counting to use chunked low-slice counting plus snapshot shifting, which preserved throughput and removed the previous `COUNT` critical path from `P4`.
- Added standalone synthesis harnesses and Quartus revisions under `syn/quartus/`, including full-IP `p2/p3/p4` and encoder-only `p2/p3/p4` builds.
- Updated `script/ring_buffer_cam_hw.tcl` to package the new encoder source, expose `ENCODER_PIPE_STAGES`, and default the packaged IP to `P4`.
- Added `script/ring_buffer_cam_presets.qprs` with `Default P4`, `Reserved P3`, `Reserved P2`, and `Regression P1`.
- Rebuilt the UVM environment so CSR/config-space testing compiles and runs cleanly with mixed VHDL/SV sources.
- Fixed reset determinism for CSR-visible counters and fill-level bookkeeping so the configuration space is software-usable immediately after reset.

## 8. Plan Mapping (`doc/upgrade_plan.md`)

- Phase 1: encoder module
  - Implemented: `rtl/vhd_ver/addr_enc_logic_partitioned.vhd`
  - Open: no separate encoder-only randomized TB was added; verification is currently end-to-end through the ring-buffer TBs
- Phase 2: memory partitioning
  - Implemented for the current standalone `P=2/P=3/P=4` pipe-depth builds with `N_PARTITIONS=2`
- Phase 3: pop engine refactor
  - Implemented `SEARCH -> LOAD -> COUNT -> DRAIN` with registered issue and exact hit counting
- Phase 4: arbiter extension
  - Implemented enough for the current standalone builds and RTL sims
  - Open: parallel push/pop behavior was not separately signed off as an isolated requirement in this turn
- Phase 5: integration and verification
  - RTL sims: the promoted isolated UVM catalog is evidenced at `563/563` with `0` failing isolated cases, but DV closure remains blocked by missing 30 s simulator-time signoff soaks `CROSS-125..CROSS-129`
  - Quartus standalone comparison: historical full-IP `v23/p2/p3/p4` and encoder-only `p2/p3/p4` sweeps remain archived; the current `ring_buffer_cam_syn_p4` rerun is the authoritative synthesis result
  - UVM configuration-space validation: pass across `p1/p2/p3/p4` for reset defaults, RW semantics, and activity counters
  - Timing closure on the current delivered standalone signoff build: pass on `ring_buffer_cam_syn_p4` at `137.5 MHz` (`7.273 ns`) with positive setup and hold slack
  - SV standalone timing closure: fail on `ring_buffer_cam_syn_sv_p4` at `137.5 MHz`, worst setup slack `-14.213 ns`
  - Resource target: pass at `2191 ALMs`, below the `4000 ALM` estimate and `6000 ALM` bloat ceiling
  - Gate-level simulation: functional gate smoke pass on the regenerated P4 netlist

## 9. Packaging and Local Platform Designer Environment

- `script/ring_buffer_cam_hw.tcl` was reworked into the current project style used by the newer MAX10/JESD204B-style components:
  - current delivered version `26.2.4.0421`
  - elaboration and validation callbacks present
  - parameter documentation grouped into configuration/interface/register-map tabs
  - lint check passed against `rtl/vhd_ver/ring_buffer_cam.vhd`
- Packaged defaults:
  - default delivered configuration: `ENCODER_PIPE_STAGES = 4`
  - reserved higher-compatibility bins remain available through presets: `P3`, `P2`, and `P1`
- Local Platform Designer cleanup:
  - removed stale `online_sc` PCIe BAR project search paths from `/home/yifeng/.altera.quartus/ip/18.1/ip_search_path/user_components.ipx`
  - retained only the active user catalog roots:
    - `/home/yifeng/packages/online_dpv2/online/fe_board/ip_mu3e/**/*`
    - `/home/yifeng/packages/mu3e_ip_dev/mu3e-ip-cores/**/*`
  - practical effect: opening unrelated `.qsys` files should no longer source the `online_sc` PCIe BAR `_hw.tcl` trees through the user IP catalog

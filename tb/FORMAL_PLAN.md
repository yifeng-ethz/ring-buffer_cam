# FORMAL_PLAN.md — ring_buffer_cam

**DUT:** `ring_buffer_cam` (SV core under `rtl/sv_ver/ring_buffer_cam_core.sv`, packaged top in `rtl/sv_ver/ring_buffer_cam.sv`)
**Active variant:** `default_p2_pipe4` (`N_PARTITIONS=2`, `ENCODER_PIPE_STAGES=4`)
**Date:** `2026-05-08`

**Companion to:** [DV_PLAN.md](DV_PLAN.md), [DV_HARNESS.md](DV_HARNESS.md), [DV_BASIC.md](DV_BASIC.md), [DV_EDGE.md](DV_EDGE.md), [DV_PROF.md](DV_PROF.md), [DV_ERROR.md](DV_ERROR.md), [BUG_HISTORY.md](BUG_HISTORY.md)

## Legend

✅ proven (qverify FORMAL pass) &middot; ⚠️ partial / bounded / cex-pending &middot; ❌ failed / cex found &middot; ❓ pending / not yet bound &middot; ℹ️ informational

## 1. Scope

This file is the canonical formal contract for `ring_buffer_cam`. It catalogs every property currently bound to the DUT through the qverify Lint+CDC+RDC+Formal screen, the implemented upgrades targeted at the v26.2.9 sector-lock and v26.2.10 accounting/metadata stack, and the harness rules every property must satisfy.

Formal evidence is owned by the `rtl-linter-and-checker` skill at `~/.codex/skills/rtl-linter-and-checker/` and the canonical entry-point script:

```bash
python3 ~/.codex/skills/rtl-linter-and-checker/scripts/questa_static_screen.py \
    --top ring_buffer_cam_sector_lock_formal_top \
    --filelist tb/formal/ring_buffer_cam_sector_lock_formal.f \
    --modes lint,cdc,rdc,formal
```

Pass criteria: `qverify` exits clean, Lint `Error (0)`, CDC `Violations (0)`, RDC `Violation (0)`, and the Formal property summary lists every signoff assertion as `proven`. A non-zero exit is a hard fail; `--legacy-waiver` is only valid after explicit user authorization.

DV closure cannot count any sector-lock contract as covered unless the matching formal property in this file is in the `Proven` set or a UVM-only justification is recorded under §6.

## 2. Files

| File | Role |
|---|---|
| `rtl/sv_ver/ring_buffer_cam_core.sv` | DUT core, sector-lock arbiter, 64-bit accounting, drop counters, freeze, deassembly FIFO |
| `rtl/sv_ver/ring_buffer_cam.sv` | top wrapper that exposes the same boundary used by the formal harness |
| `rtl/sv_ver/ring_buffer_cam_fifo.sv` | shared FIFO primitive used for the deassembly and pop-cmd FIFOs |
| `rtl/sv_ver/ring_buffer_cam_sv_pkg.sv` | shared types, `pop_state_e`, `run_state_e`, `debug_msg_t`, helpers |
| `tb/formal/ring_buffer_cam_sector_lock_formal_top.sv` | explicit formal wrapper: instantiates the DUT, exposes metadata sideband inputs, and binds all SVA modules |
| `tb/formal/ring_buffer_cam_sector_lock_sva.sv` | bound SVA module for sector-lock mask geometry, decision-5, and bounded progress assertions |
| `tb/formal/ring_buffer_cam_accounting_sva.sv` | bound SVA module for 64-bit accounting, freeze, drop counters, and run-control cleanup |
| `tb/formal/ring_buffer_cam_metadata_sva.sv` | bound SVA module for deassembly FIFO / slot / emit metadata lineage checks |
| `tb/formal/ring_buffer_cam_sector_lock_formal.f` | qverify filelist for the full formal flow |
| `tb/formal/ring_buffer_cam_sv_static.f` | qverify filelist for the SV Lint/CDC/RDC static screen |

## 3. Currently Proven Set (8/8)

The 2026-05-07 qverify run on commit `e2c672ae` proved the following 8 assertions in `tb/formal/ring_buffer_cam_sector_lock_sva.sv`; that proven sector-lock set remains part of the carried stack through `ed41c983`. Every assertion is a single-clock safety property under `default disable iff (i_rst)`.

| ID | Property | Intent | Status |
|---|---|---|---|
| F-SL01 | `ap_search_blocks_push_write` | `pop_engine_state == POP_SEARCHING |-> push_write_grant=0` | ✅ |
| F-SL02 | `ap_search_blocks_push_erase` | `pop_engine_state == POP_SEARCHING |-> push_erase_grant=0` | ✅ |
| F-SL03 | `ap_locked_sector_blocks_push_write` | `LOAD/COUNT/DRAIN ∧ push_write_sector_locked |-> push_write_grant=0` | ✅ |
| F-SL04 | `ap_locked_sector_blocks_push_erase` | `LOAD/COUNT/DRAIN ∧ push_erase_sector_locked |-> push_erase_grant=0` | ✅ |
| F-SL05 | `ap_flush_owns_memory_arbiter` | `pop_flush_grant |-> ¬push_write_grant ∧ ¬push_erase_grant ∧ ¬pop_erase_grant` | ✅ |
| F-SL06 | `ap_push_erase_not_granted_during_flush` | `push_erase_grant |-> ¬pop_flush_req` | ✅ |
| F-SL07 | `ap_push_write_not_granted_during_flush` | `push_write_grant |-> ¬pop_flush_req` | ✅ |
| F-SL08 | `ap_blocked_push_erase_req_holds_without_grant` | request-with-block does not raise `push_erase_grant` | ✅ |

These eight properties anchor BUG-065-R sector-lock closure but only cover the *negative* contracts (what cannot happen). The §4 catalog is now authored and bound; closure status is tracked row-by-row below.

## 4. Implemented Property Catalog

Every property below maps to the SystemVerilog binding in §5 and to the matching DV bucket cases in [DV_BASIC.md](DV_BASIC.md), [DV_EDGE.md](DV_EDGE.md), [DV_PROF.md](DV_PROF.md), [DV_ERROR.md](DV_ERROR.md). Status `✅` means the assertion is authored, bound, and observed as `Proven` in the current qverify transcript. Status `⚠️` means the assertion is authored and bound, but the current transcript still reports a fired or proof-open condition.

Current evidence:

- qverify Lint/CDC/RDC pass after the reset/formal-model cleanup: `/tmp/rbcam_sv_static_codex_20260508_010728/questa_static_screen.log`
- latest full qverify attempt after the metadata assertion cleanup: `/tmp/rbcam_sv_full_formal_codex5_20260508_010530/questa_static_screen.log`
- latest full qverify property summary: `45/47` proven, `2` fired-with-warning
- open formal blockers: `metadata_sva.ap_slot_write_metadata_comes_from_fifo_output` and `metadata_sva.ap_pop_metadata_pending_comes_from_slot` still fire in qverify; do not claim formal closure until those are resolved.

### 4.1 Sector-Lock Geometry And Mask Correctness

| ID | Property (informal) | Why it matters | Bind site | Backing DV cases | Status |
|---|---|---|---|---|---|
| F-LM01 | `pop_sector_lock_mask` after SEARCH-tail equals `snapshot_sector_mask(pop_snapshot)` (set-equality, not just superset) when no in-flight emit and no `pop_erase_req` | catches a regression that would silently widen or narrow the lock | inside `ring_buffer_cam_sector_lock_sva` (read internal `pop_snapshot` via top wrapper) | B135, B140 | ✅ |
| F-LM02 | When `pop_issue_inflight ∨ pop_output_pending ∨ pop_emit_pending`, the bit `addr_sector(pop_issue_addr_pending)` is set in `pop_sector_lock_mask` | proves the in-flight extra lock fires on every cycle of the emit window | same | B138, E135 | ✅ |
| F-LM03 | When `pop_erase_req=1`, the bit `addr_sector(pop_issue_addr)` is set in `pop_sector_lock_mask` | proves the request-cycle extra lock | same | B139, E134 | ✅ |
| F-LM04 | `push_write_sector_locked == (pop_engine_state inside {LOAD,COUNT,DRAIN}) ∧ pop_sector_lock_mask[addr_sector(write_pointer)]` | exact equality, not just one-way implication | same | B137, E132, E133 | ✅ |
| F-LM05 | `push_erase_sector_locked == (pop_engine_state inside {LOAD,COUNT,DRAIN}) ∧ pop_sector_lock_mask[addr_sector(push_erase_addr_reg)]` | symmetric to F-LM04 for the delayed erase | same | B142 | ✅ |
| F-LM06 | `pop_engine_state inside {POP_IDLING, POP_RESETTING, POP_SEARCHING, POP_FLUSHING, POP_FLUSHING_RST} |-> push_write_sector_locked=0 ∧ push_erase_sector_locked=0` | proves no lock leakage outside the active service window | same | B137, E139 | ✅ |
| F-LM07 | Width invariant: `$bits(pop_sector_lock_mask) == LOCK_SECTOR_COUNT_CONST` and `addr_sector(addr) < LOCK_SECTOR_COUNT_CONST` for any `addr` of width `ADDR_W_CONST` | catches a parameterization regression | static elaboration assertion | B136, E136, E137 | ✅ |

### 4.2 Decision-5 (Concurrent Push-Write + Pop-Erase) Certification

| ID | Property (informal) | Why it matters | Bind site | Backing DV cases | Status |
|---|---|---|---|---|---|
| F-D501 | `decision_reg == 3'd5 |-> push_write_grant=1 ∧ pop_erase_grant=1` (1-cycle-prior) | the new code 5 means both grants fire | sva module | B136, E138 | ✅ |
| F-D502 | `push_write_grant ∧ pop_erase_grant |-> addr_sector(write_pointer) != addr_sector(pop_issue_addr)` | the lock guarantees the two grants act on disjoint sectors | sva module | B136, E134 | ✅ |
| F-D503 | `push_write_grant ∧ pop_erase_grant |-> pop_engine_state inside {POP_LOADING, POP_COUNTING, POP_DRAINING} ∧ ¬pop_flush_req` | rules out concurrent grants outside the active service window | sva module | B137, B148 | ✅ |
| F-D504 | `decision == 3'd3 |-> ¬push_write_grant ∧ ¬push_erase_grant ∧ ¬pop_erase_grant` (already covered for `pop_flush_grant`, this generalizes to the comb decision code) | flush priority over decision=5 | sva module | B148, X134 | ✅ |
| F-D505 | Mutual exclusivity of `decision`: at most one of `{0,1,2,3}` is the canonical encoding for solo grants and `5` for the concurrent grant; the encoding is one of `{0,1,2,3,4,5}` exclusively | catches a missing case in the comb arbiter | sva module | E138 | ✅ |

### 4.3 Liveness And Bounded Progress

| ID | Property (informal) | Why it matters | Bind site | Backing DV cases | Status |
|---|---|---|---|---|---|
| F-LV01 | `push_write_req ∧ ¬push_write_sector_locked ∧ ¬pop_flush_req ∧ pop_engine_state != POP_SEARCHING |-> ##[0:1] push_write_grant` | proves the sector-lock fix delivers progress for any unblocked sector | sva module (use `s_eventually` if available, else bounded-time bound) | B136, P130, P137 | ✅ |
| F-LV02 | `pop_erase_req |-> ##[0:1] pop_erase_grant` (no lock blocks pop_erase; pop_flush_req has higher priority but is gated separately) | catches a regression that would let push starve pop | sva module | B093, B130 | ✅ |
| F-LV03 | `pop_engine_state == POP_SEARCHING |-> ##[6:6] pop_engine_state == POP_LOADING` (the SEARCH wait counter `pop_search_wait_cnt` is exactly 0..5 so the LOAD entry is at wait-cycle 6) | catches an off-by-one in the SEARCH guard | sva module | B056, B131 | ✅ |
| F-LV04 | `pop_flush_req |-> ##[0:1] pop_flush_grant` (with no other request blocking) | catches a regression that would let the flush starve | sva module | B108, X134 | ✅ |

### 4.4 64-Bit Accounting Counters And Freeze

| ID | Property (informal) | Why it matters | Bind site | Backing DV cases | Status |
|---|---|---|---|---|---|
| F-AC01 | Monotonicity: `debug_msg2.push_cnt` only increments on `push_write_grant`, only by 1, and never decreases (except on soft-reset / RUN_PREPARING flush) | catches a multi-bump or stale-bump regression | sva module | B025, B147 | ✅ |
| F-AC02 | Monotonicity for `debug_msg2.pop_cnt` (only increments on `pop_erase_grant`) | symmetric to F-AC01 | sva module | B026 | ✅ |
| F-AC03 | Monotonicity for `debug_msg2.overwrite_cnt` (only increments on `push_erase_grant`) | symmetric | sva module | B027 | ✅ |
| F-AC04 | `csr_counter_freeze` rising-edge invariant: on the cycle `csr_counter_freeze` transitions 0→1, `debug_msg2_snap` is updated to the value of `debug_msg2` (the read-side of the always_ff is the previous register value) | proves the freeze snapshot is atomic | sva module | B143, E140 | ✅ |
| F-AC05 | While `csr_counter_freeze=1`, every CSR read of words 5..20 returns the matching field of `debug_msg2_snap` regardless of `debug_msg2` activity | proves the freeze readback path | sva module | B143 | ✅ |
| F-AC06 | `csr_soft_reset_pulse |-> ##1 (debug_msg2 == '0 ∧ debug_msg2_snap == '0)` | catches a regression that would let `debug_msg2_snap` survive a soft-reset | sva module | X136 | ✅ |
| F-AC07 | High-half / low-half 64-bit consistency: when `csr_counter_freeze=0`, the concatenation of CSR-read of word `(N+5)` (high) and word `N` (low) for `N ∈ {5,6,7,8,9}` reproduces `debug_msg2.<counter>[63:0]` to within the bounded read latency of the always_ff | catches any 32-bit truncation regression | sva module | B147 | ✅ |

### 4.5 Drop Counters

| ID | Property (informal) | Why it matters | Bind site | Backing DV cases | Status |
|---|---|---|---|---|---|
| F-DC01 | `debug_msg2.deasm_full_drop_cnt` increments iff `ingress_payload_countable ∧ deassembly_fifo_full` (event = `deasm_full_drop_event`) | exact event-to-counter binding | sva module | B144, P134 | ✅ |
| F-DC02 | `debug_msg2.pop_cmd_full_drop_cnt` increments iff `pop_cmd_tick_due ∧ pop_cmd_fifo_full` (event = `pop_cmd_full_drop_event`) | exact binding | sva module | B145, P134 | ✅ |
| F-DC03 | `debug_msg2.egress_not_ready_drop_cnt` increments iff `aso_hit_type2_valid ∧ ¬aso_hit_type2_ready` (event = `egress_not_ready_drop_event`) | exact binding | sva module | B146, P134 | ✅ |
| F-DC04 | The three drop events are pairwise distinguishable: under each single-regime stress, exactly one drop counter advances and the other two stay at zero | catches a counter-leakage regression | sva module (forall-cycle relational property) | P134 | ✅ |

### 4.6 Run-Control / Flush Priority

| ID | Property (informal) | Why it matters | Bind site | Backing DV cases | Status |
|---|---|---|---|---|---|
| F-RC01 | `pop_flush_req |-> decision == 3'd3 ∨ pop_engine_state == POP_FLUSHING_RST` (i.e. the comb arbiter never selects code 0/1/2/5 while flush is requested) | flush priority over the new decision-5 path | sva module | B148, X134 | ✅ |
| F-RC02 | `csr_soft_reset_pulse |-> ##1 (pop_engine_state == POP_IDLING ∧ run_state_cmd == RUN_IDLING ∧ pop_sector_lock_mask == '0 ∧ write_pointer == '0)` | catches a stale lock-mask or stale write_pointer after soft-reset | sva module | X133, X138 | ✅ |
| F-RC03 | `(run_state_cmd == RUN_PREPARING) ∧ (flush_cycle_count == 0) |-> deassembly_fifo_sclr=1 ∧ pop_cmd_fifo_sclr=1 ∧ slot_valid='0` | catches a missing PREP-entry clear that would leak residents into the next run | sva module | B034, X138 | ✅ |
| F-RC04 | `terminating_drain_done == (run_state_cmd == RUN_TERMINATING) ∧ deassembly_fifo_empty ∧ pop_cmd_fifo_empty ∧ pop_engine_state == POP_IDLING ∧ debug_msg2.push_cnt == debug_msg2.pop_cnt + debug_msg2.overwrite_cnt` (combinational equality) | exact contract reproduction; catches a relaxed ack | sva module (already a comb assignment in RTL — assert the equality back) | B040, B071 | ✅ |

### 4.7 Metadata Lineage Through The Deassembly FIFO

| ID | Property (informal) | Why it matters | Bind site | Backing DV cases | Status |
|---|---|---|---|---|---|
| F-ML01 | On every `deassembly_fifo_wrreq=1` cycle, the FIFO write-data carries `(asi_hit_type1_metadata, asi_hit_type1_metadata_valid)` of that cycle | catches a regression that would silently drop the sideband at write side | sva module | P135 | ✅ |
| F-ML02 | On every resident `push_write_grant=1` slot write, `slot_metadata_valid[write_pointer]` takes `deassembly_fifo_dout.metadata_valid`; when valid is set, `slot_metadata[write_pointer]` takes `deassembly_fifo_dout.metadata`, NOT the live `asi_hit_type1_metadata*` ports | this is the BUG-066-R closure invariant; invalid metadata payload bits are not architectural | sva module | P135 | ⚠️ |
| F-ML03 | On every valid resident `pop_erase_grant=1`, `pop_metadata_valid_pending` reflects the slot valid bit; when valid is set, `pop_metadata_pending` reflects the slot metadata captured for that hit; hit-beat emit uses the pending fields | end-to-end metadata lineage from ingress through the full pipeline | sva module | P135 | ⚠️ |

F-ML01 is proven. F-ML02/F-ML03 are implemented and bound, and P135 passes in UVM on the SV implementation, but the current formal transcript still reports fired metadata assertions; these two rows block formal closure.

## 5. Binding Implementation

The §4 catalog is now bound through `tb/formal/ring_buffer_cam_sector_lock_formal_top.sv` using the `dut.v2_core.*` hierarchical reference idiom.

1. The metadata sideband is no longer tied off. `asi_hit_type1_metadata` and `asi_hit_type1_metadata_valid` are free formal inputs and feed `ring_buffer_cam_metadata_sva`.
2. `ring_buffer_cam_sector_lock_sva` carries the sector-lock mask, decision-5, flush-priority, and bounded-progress assertions.
3. `ring_buffer_cam_accounting_sva` carries the 64-bit accounting, freeze, soft-reset, drop-counter, and run-control assertions.
4. `ring_buffer_cam_metadata_sva` carries the metadata FIFO/slot/emit assertions. The deassembly FIFO write and hit-emit properties are proven; the slot-write and pop-pending properties remain open.
5. `tb/formal/ring_buffer_cam_sector_lock_formal.f` compiles all three SVA modules and the lifted top.
6. The formal filelist defines `FORMAL` so the FIFO memory model clears internal RAM only for formal X-determinism; synthesis does not see that define.
7. The canonical full run uses `--formal-timeout 30m`. A passing lint-only screen is not a formal closure substitute.

## 6. UVM-Only Justifications

A property may stay outside the formal set if and only if all of the following hold:

- the property's combinational state space is too large for the qverify Formal mode at acceptable runtime (justified per property)
- the property is fully covered by an isolated DV case PLUS a continuous-frame regression in [DV_CROSS.md](DV_CROSS.md)
- the omission is recorded under this section with a one-line rationale

Currently no property is accepted as a UVM-only substitute. P135 is companion evidence for F-ML02/F-ML03 and passes on `RTL_IMPL=sv`, but those rows remain formal blockers until the qverify fired metadata assertions are resolved or an explicit waiver is approved.

## 7. Closure Gates

The IP is "formal-closed" when all of the following hold:

- §3 stays at `8/8 proven` and the §4 catalog reaches `Proven` for every row not justified under §6
- the qverify Lint+CDC+RDC screen passes at `Error (0)` / `Violations (0)` / `Violation (0)` for the SV core under `tb/formal/ring_buffer_cam_sv_static.f`
- every BUG-065-R / BUG-066-R commit (currently `da80afbb`, `e2c672ae`, `3086685e`, `ed41c983`) is referenced from the `Commit:` field of the matching property's evidence row in `DV_REPORT.json`
- the formal screen and the matching DV cases are both green on the current RTL revision (no isolated formal closure without DV companion evidence)

A signoff git tag for `ring_buffer_cam` cannot be cut while any §4 row is in `❓` or `⚠️` and not justified under §6. Current blocker: F-ML02/F-ML03 are implemented and UVM-covered but not formally closed.

## 8. Run Recipes

### Current 8/8 sector-lock proof

```bash
python3 ~/.codex/skills/rtl-linter-and-checker/scripts/questa_static_screen.py \
    --top ring_buffer_cam_sector_lock_formal_top \
    --filelist tb/formal/ring_buffer_cam_sector_lock_formal.f \
    --modes lint,cdc,rdc,formal \
    --work-dir /tmp/rbcam_sv_sector_lock_formal_$(date +%Y%m%d_%H%M%S)
```

### SV static-screen for lint / CDC / RDC of the SV core

```bash
python3 ~/.codex/skills/rtl-linter-and-checker/scripts/questa_static_screen.py \
    --top ring_buffer_cam \
    --filelist tb/formal/ring_buffer_cam_sv_static.f \
    --modes lint,cdc,rdc \
    --work-dir /tmp/rbcam_sv_static_$(date +%Y%m%d_%H%M%S)
```

### Full implemented formal catalog

```bash
python3 ~/.codex/skills/rtl-linter-and-checker/scripts/questa_static_screen.py \
    --top ring_buffer_cam_sector_lock_formal_top \
    --filelist tb/formal/ring_buffer_cam_sector_lock_formal.f \
    --modes lint,cdc,rdc,formal \
    --formal-timeout 30m \
    --work-dir /tmp/rbcam_sv_full_formal_$(date +%Y%m%d_%H%M%S)
```

## 9. Change Log

- 2026-05-07: First version. Records the 8/8 proven set from commit `e2c672ae` (sector-lock SVA) and `ed41c983` (BUG-066 ledger close), plus the queued §4 catalog driven by BUG-065-R / BUG-066-R review.
- 2026-05-08: Implemented the §4 binding stack in three SVA modules and lifted metadata inputs in the formal top. qverify lint is clean; full formal closure remains blocked by F-ML02/F-ML03 metadata tracker fires.
- 2026-05-08: Added reset cleanup for SV pop pending metadata registers, `FORMAL`-only FIFO memory clearing, and metadata assertions that treat invalid payload bits as don't-care while preserving valid-bit lineage. SV static Lint/CDC/RDC passes and P135 passes in UVM; full formal is still `45/47` with F-ML02/F-ML03 firing.

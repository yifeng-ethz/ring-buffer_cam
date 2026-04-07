# Ring Buffer CAM v3.0 — Partitioned Pipelined Encoder Upgrade Plan

**Author:** Architect
**Date:** 2026-03-19
**Baseline:** commit `a762511` (v2.3, RING_BUFFER_N_ENTRY=1024, SMALL_LOGIC=64×16)

---

## 1. Problem Statement

### 1.1 Current Architecture (v2.3)

The CAM lookup returns a **1024-bit one-hot** match vector.  This is split into
**N_SMALL_LOGIC = 16 partitions** of 64 bits each, feeding 16 instances of
`addr_enc_logic_small`.  All 16 encoders are purely combinational (no pipeline
registers); results are consumed in the same clock cycle they are loaded.

The pop engine state machine:

```
IDLE → SEARCH (5 cyc wait) → EVAL → POPING → EVAL → POPING → … → RESET → IDLE
```

**EVAL→POPING takes 2 cycles per hit** because:
1. EVAL selects a bank and computes the binary address (1 cycle).
2. POPING requests arbiter access, erases CAM/side-RAM, reads hit (1 cycle).

Peak drain rate = **0.5 hits/cycle**.

### 1.2 Failed Attempt (addr_enc_logic_pipe, discarded)

A staged/pipelined encoder was added within each 64-bit partition.  This improved
timing closure but **increased ALM usage** (extra pipeline registers × 16
instances) without improving throughput — the bottleneck is still 2 cycles per hit
in the pop engine.

### 1.3 Resource Concern

For 1024 entries with 16 × 64-bit one-hot encoders:
- 16 combinational priority-encode trees, each scanning 64 bits.
- Bank combiner: 16-way priority arbiter + address offset adder.
- Total: ~1600 ALMs on Arria V (addr encoding alone).

---

## 2. Proposed Architecture: Modulo-P Partitioned Pipeline

### 2.1 Core Idea

Partition the ring buffer into **P physical memory partitions** (CAM + side-RAM),
where entry at global address `a` belongs to **partition `a mod P`**.

Each partition has its own one-hot encoder.  The P encoders are **staggered** in a
pipeline: encoder `p` starts on cycle `p`, takes **P cycles** to produce a result.
After the pipeline is full, **one result is ready every cycle** → drain rate = **1
hit/cycle**.

### 2.2 Parameters

| Parameter | Symbol | Default | Constraint |
|:----------|:-------|:--------|:-----------|
| Total CAM depth | `N` | 1024 | power of 2 |
| Pipeline partitions | `P` | 2 | power of 2, P ≤ 8 |
| Leaf encoder width | `L` | 16 | power of 2 |
| Entries per partition | `N/P` | 512 | |
| One-hot width per partition | `N/P` | 512 | |
| Encoder tree levels | `K` | `ceil(log_L(N/P))` | = P (must match) |
| Encoder latency (cycles) | | P | registered between levels |

**Matching constraint:** `P = K = ceil(log_L(N/P))`.
For the default configuration: `P = 2`, `L = 16`, `N/P = 512`:
- Level 1: 32 leaf encoders of 16 bits each → 32 partial results (1 cycle).
- Level 2: reduce 32 partial results via 2 groups of 16 → final binary addr (1 cycle).
- Total latency = 2 cycles = P. ✓

For `P = 4`, `L = 16`, `N/P = 256`:
- Level 1: 16 leaves of 16 → 16 partials (1 cycle).
- Level 2: 1 group of 16 → 1 partial (1 cycle).
- Level 3: combine with count/flag across levels (1 cycle).
- Level 4: result snapshot (1 cycle).
- Total = 4 cycles = P. ✓

### 2.3 Memory Organization

```
Global addr a:
  partition_index = a mod P       (= a(log2(P)-1 downto 0))
  local_addr      = a / P         (= a(ADDR_W-1 downto log2(P)))

Reconstruction:
  global_addr = local_addr * P + partition_index
```

**Physical resources per partition:**
- 1 × `cam_mem_a5` instance (depth = N/P, width = SEARCH_KEY_WIDTH)
- 1 × `alt_simple_dpram` instance (depth = N/P, width = 1 + SIDE_DATA_BITS)
- 1 × pipelined one-hot encoder (P-stage, L-wide leaves)

**Total resources (P=2):** 2 CAMs + 2 side-RAMs + 2 encoders.
BRAM usage doubles but ALM usage for encoding drops by ~50% (smaller trees).

### 2.4 Push Path

The write pointer increments as before (monotonically increasing mod N).

```
write_pointer: 0, 1, 2, 3, 4, 5, ...
partition:     0, 1, 0, 1, 0, 1, ...   (for P=2)
local_addr:    0, 0, 1, 1, 2, 2, ...
```

Each partition has its own write port driven by the global arbiter only when
`write_pointer mod P == partition_index`.  The arbiter mux selects which partition's
CAM/side-RAM gets written.

**Throughput impact:** Push still takes 1 cycle (no overwrite) or 2 cycles
(overwrite erase).  Since pushes arrive one per cycle and are distributed
round-robin, no partition is starved.

### 2.5 Search/Match Path

1. **SEARCH state:** The search key (`pop_current_sk`) is broadcast to ALL P
   CAMs simultaneously.  Each CAM returns an `(N/P)`-bit one-hot match vector
   after the existing 2-cycle CAM read latency (registered output in `cam_mem_a5`).

2. **LOAD phase (replaces PRIMING):** After the CAM results are ready, the pop
   engine loads partition encoders in a **staggered** sequence:
   - Cycle T+0: load encoder[0] with partition 0's one-hot
   - Cycle T+1: load encoder[1] with partition 1's one-hot
   - ...
   - Cycle T+(P-1): load encoder[P-1]

3. **DRAIN phase (replaces EVAL→POPING loop):**
   After P cycles of pipeline fill, results arrive every cycle:
   - Cycle T+P: encoder[0] result valid → erase + output hit
   - Cycle T+P+1: encoder[1] result valid → erase + output hit
   - ...
   - Round-robin through partitions; if a partition has remaining matches,
     reload its encoder with the `onehot_next` vector.
   - If a partition is exhausted (flag=0), skip it (NOP cycle or borrow slot
     for push).

   **Drain rate = 1 hit/cycle** (sustained, after P-cycle pipeline fill).

### 2.6 Pop Engine State Machine (New)

```
IDLE → SEARCH (5 cyc) → LOAD (P cyc stagger) → DRAIN (round-robin) → RESET → IDLE
                                                   ↑_____reload________|
```

**DRAIN sub-states per partition `p`:**

| Sub-state | Duration | Action |
|:----------|:---------|:-------|
| ENCODING  | P cycles | Encoder processing; no arbiter request |
| RESULT    | 1 cycle  | Encoder output valid; request erase grant |
| RELOAD    | 1 cycle  | Feed `onehot_next` back to encoder if matches remain |
| DONE      | —        | Partition exhausted; skip in round-robin |

The round-robin index `rr_idx` cycles 0 → 1 → … → P-1 → 0.  On each cycle:
- If `partition[rr_idx]` is in RESULT state → erase + output + reload.
- If ENCODING → NOP (push can borrow this slot).
- If DONE → NOP.

When all partitions are DONE, transition to RESET.

### 2.7 Arbiter Changes

The memory arbiter (`proc_memory_arbiter_comb`) must be extended:
- **P sets of CAM write ports** and **P sets of side-RAM ports** (one per partition).
- Only ONE partition is active for pop-erase on any given cycle (selected by `rr_idx`).
- Push write/erase targets the partition selected by `write_pointer mod P`.
- If push and pop target different partitions → **no contention** (both proceed in
  parallel) → further throughput improvement.

### 2.8 Output Assembly

No change to the `proc_avst_output_assembly` process.  The subheader is emitted
once (with total hit count summed across all P partitions), and hits follow at 1
per cycle.  The `pop_last_hit_pending` signal fires when the global count reaches 1.

### 2.9 One-Hot Encoder Module (New: `addr_enc_logic_partitioned.vhd`)

Replaces both `addr_enc_logic_small` and `addr_enc_logic_pipe`.

**Interface:**
```vhdl
entity addr_enc_logic_partitioned is
generic(
    PARTITION_SIZE  : natural := 512;   -- N/P
    LEAF_WIDTH      : natural := 16;    -- L
    PIPE_STAGES     : natural := 2      -- P (must = encoding depth)
);
port(
    i_clk                       : in  std_logic;
    i_rst                       : in  std_logic;
    i_load                      : in  std_logic;
    i_cam_address_onehot        : in  std_logic_vector(PARTITION_SIZE-1 downto 0);
    o_result_valid              : out std_logic;
    o_cam_address_binary_lsb    : out std_logic_vector(log2(PARTITION_SIZE)-1 downto 0);
    o_cam_match_flag            : out std_logic;
    o_cam_match_count           : out std_logic_vector(log2(PARTITION_SIZE) downto 0);
    o_cam_address_onehot_next   : out std_logic_vector(PARTITION_SIZE-1 downto 0)
);
end entity;
```

**Internal architecture:**
```
Level 1 (comb):  Slice into PARTITION_SIZE/L groups of L bits.
                 Each group → leaf_encode() → {flag, lsb, msb, count, onehot_next}
                 Register results.

Level 2 (comb):  Reduce groups into PARTITION_SIZE/(L*L) super-groups of L.
                 Each super-group → mid_encode() → {flag, lsb, count}
                 Register results.

(If P > 2, repeat with more levels.)

Final:           Priority-select across remaining results; reconstruct global
                 local_addr; assemble onehot_next by zeroing consumed leaf.
                 Register result.
```

All operations registered between levels → bounded combinational depth =
one `L`-wide priority scan per level.

---

## 3. Resource Estimation (P=2, N=1024, L=16)

| Resource | v2.3 (baseline) | v3.0 (proposed) | Delta |
|:---------|:----------------|:----------------|:------|
| M10K BRAM (CAM) | 16 blocks | 32 blocks | +16 |
| M10K BRAM (side-RAM) | 4 blocks | 8 blocks | +4 |
| ALM (encoder) | ~1600 | ~900 | −700 |
| ALM (arbiter) | ~200 | ~350 | +150 |
| ALM (total est.) | ~2800 | ~2350 | −450 |
| Fmax (encoder path) | ~120 MHz | ~180 MHz | +50% |
| Pop throughput | 0.5 hits/cyc | 1.0 hits/cyc | **2×** |

BRAM increase is acceptable — Arria V 5AGXFB3H has 240 M10K blocks; the CAM
uses < 15% even after doubling.

---

## 4. Backward Compatibility

### 4.1 Port-level

**No changes** to the entity ports of `ring_buffer_cam`.  The same Avalon-ST
input/output interfaces, CSR, and run-control are preserved.

### 4.2 New Generics

| Generic | Default | Note |
|:--------|:--------|:-----|
| `N_PARTITIONS` | 2 | Replaces `MATCH_ENCODER_PARTITION_SIZE` |
| `ENCODER_LEAF_WIDTH` | 16 | One-hot leaf width |

`MATCH_ENCODER_PARTITION_SIZE` and `MATCH_ENCODER_PIPE_STAGES` are removed
(they were only in the uncommitted changes, not in v2.3).

### 4.3 Functional Equivalence

For `N_PARTITIONS = 1`, the architecture degenerates to the v2.3 baseline (single
CAM + single encoder, 2 cycles/hit).  This serves as a regression fallback.

---

## 5. Implementation Plan

### Phase 1: Encoder Module (`addr_enc_logic_partitioned.vhd`)
- [ ] Implement P-stage pipelined encoder with L-wide leaves.
- [ ] Standalone testbench: verify against `addr_enc_logic_small` with random vectors.
- [ ] Parameterize for P=1 (baseline), P=2, P=4.

### Phase 2: Memory Partitioning
- [ ] Instantiate P × `cam_mem_a5` with depth = N/P.
- [ ] Instantiate P × `alt_simple_dpram` with depth = N/P.
- [ ] Route write_pointer partitioning: `partition = wp mod P`, `local_addr = wp / P`.
- [ ] Route search key broadcast to all P CAMs.
- [ ] Merge one-hot outputs into P separate vectors.

### Phase 3: Pop Engine Refactor
- [ ] Replace SEARCH→EVAL→POPING with SEARCH→LOAD→DRAIN.
- [ ] Implement round-robin partition scheduler.
- [ ] Implement staggered encoder loading.
- [ ] Handle partition exhaustion (DONE state).
- [ ] Wire `pop_cam_match_addr` reconstruction: `local_addr * P + partition_index`.

### Phase 4: Arbiter Extension
- [ ] Partition-aware mux for CAM write ports (push vs pop target different partitions).
- [ ] Allow parallel push + pop when targeting different partitions.
- [ ] Maintain backward-compatible flush sequence (iterate all partitions).

### Phase 5: Integration & Verification
- [ ] Run `ring_buffer_cam_partitioned_tb` (new comprehensive TB, see below).
- [ ] Run existing `ring_buffer_cam_pipeline_smoke_tb` with N_PARTITIONS=1 for regression.
- [ ] Quartus synthesis: Arria V resource + Fmax report.
- [ ] Functional comparison: push 10K hits, pop all, verify zero loss.

---

## 6. Testbench Plan

### 6.1 New Testbench: `ring_buffer_cam_partitioned_tb.vhd`

**Purpose:** Validate the partitioned architecture end-to-end.

**Test Cases:**

| ID | Name | Description | Pass Criteria |
|:---|:-----|:------------|:--------------|
| TC1 | single_hit | Push 1 hit, pop 1 hit | Correct output, fill=0 |
| TC2 | same_key_burst | Push 128 hits with same ts[11:4], pop all | All 128 hits recovered, correct subheader count, drain rate ≥ 0.9 hits/cyc |
| TC3 | multi_key | Push hits across 4 different search keys, pop in order | Correct grouping per subheader |
| TC4 | overwrite_stress | Push > N hits (force overwrites), pop | Overwrite count matches, no corruption |
| TC5 | throughput_measure | Push 256 same-key hits, measure cycles from first to last pop | Assert ≤ 256 + P + SEARCH_LATENCY cycles |
| TC6 | push_pop_contention | Continuous push while pop drains | No deadlock, all hits accounted |
| TC7 | partition_balance | Push N hits, verify each partition holds N/P entries | Internal probe on partition fill levels |
| TC8 | flush_clean | Start run, push hits, terminate, flush | Fill level = 0, all counters reset |
| TC9 | parametric_P1 | Run TC2 with N_PARTITIONS=1 | Regression vs v2.3 behavior |

**Throughput Checker (built into TB):**
```
measure_throughput: process
  -- Count cycles between first and last aso_hit_type2_valid in a subheader frame.
  -- Assert: cycles ≤ hit_count + P + OVERHEAD_CONST
end process;
```

### 6.2 Simulation Script Update

Update `run_questa_pipeline_smoke.sh` to compile the new testbench and encoder
module alongside the existing smoke test.

---

## 7. Risk Assessment

| Risk | Impact | Mitigation |
|:-----|:-------|:-----------|
| BRAM doubling exceeds budget | Medium | P=2 uses 40 blocks (17% of 5AGXFB3H); acceptable |
| Arbiter complexity for parallel push+pop | Low | Fall back to serial arbiter if timing fails |
| Round-robin NOP cycles when partitions unbalanced | Low | Hits distribute evenly by construction (sequential write_pointer) |
| Flush time increases (P × original) | Low | Flush is rare (end-of-run only) |

---

## 8. Decision Required

- **P value:** P=2 (recommended, 2× throughput, moderate BRAM increase) vs P=4 (4× peak but 4× BRAM).
- **Parallel arbiter:** Allow simultaneous push + pop to different partitions? (Adds complexity but removes contention.)
- **Fallback:** Keep N_PARTITIONS=1 degenerate mode for regression?

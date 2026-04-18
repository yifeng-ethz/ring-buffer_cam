# Ring Buffer CAM Mu3E IP

Timestamp-ordered resequencing buffer built as a ring-buffer-shaped content-addressable
memory. It accepts MuTRiG Type-1 hits, stores them under `ts[11:4]`, and emits
timestamp-ordered Type-2 framed output with live fill-level and overwrite accounting.

**Version:** 26.1.5.0422
**Module name:** `ring_buffer_cam`
**Platform Designer group:** Mu3e Data Plane / Modules

---

## Use Case

The Mu3e FEB datapath contains local resequencing stages between front-end hit processors
and framed egress. Hits arrive continuously, can be interleaved across lanes, and can be
temporarily out of order inside a bounded timestamp window. `ring_buffer_cam` absorbs that
window locally, stores hits in a circular CAM + side-RAM structure, and drains them in
timestamp order after the configured service delay.

Typical deployment:

- **Timestamp resequencing:** reorder MuTRiG hits by `ts[11:4]` before frame assembly.
- **Bounded buffering:** hold hits for `EXPECTED_LATENCY` cycles before they become eligible
  for ordered pop.
- **Overflow accounting:** if a burst injects more than the local depth before service catches
  up, the IP overwrites old residents and reports the legal loss explicitly in `OVERWRITE_COUNT`.
- **Run-control integration:** align start/stop behavior with Mu3e run states and end-of-run
  draining.

The delivered default is a **512-entry** ring per instance with **4-way interleaving** and a
**4-partition / 4-stage** match encoder. In TLM terms this behaves as a bounded resequencing
buffer, not an unbounded queue: burst pattern matters, not just average arrival rate.

---

## Architecture

```
 asi_hit_type1 ----> ingress filter / deassembly_fifo ----> push engine ----+
       |                                                                     |
       |                                                                     v
       |                                                           main_cam + side_ram
       |                                                                     |
 asi_ctrl ------> run control / expected latency ----> pop_cmd_fifo ----> SEARCH / LOAD / DRAIN
       |                                                                     |
 avs_csr  <---------------------- CSR / counters / identity -----------------+
                                                                             |
                                                                             v
                                                             output assembly / subheaders
                                                                             |
                                                                             v
                                                                      aso_hit_type2

                                                             +----------------------+
                                                             | fill level / debug    |
                                                             | push/pop/overwrite    |
                                                             +----------------------+
```

### Pipeline Stages

| Stage | Component | Latency | Description |
|-------|-----------|---------|-------------|
| 1 | `ingress_comb` | 0 (comb) | Lane filtering, timestamp-error filtering, run-state gating |
| 2 | `deassembly_fifo` | 1+ cycles | Elastic input buffering before local CAM write ownership |
| 3 | `push_write` | 1 cycle | Write new resident into CAM + side-RAM at `write_pointer` |
| 4 | `push_erase` | 1 cycle on overwrite | Evict old resident if the slot was already occupied |
| 5 | `pop_descriptor_generator` | periodic | Generate search keys from `gts - EXPECTED_LATENCY` |
| 6 | `SEARCH -> LOAD -> COUNT -> DRAIN` | multi-cycle | Broadcast search key, load partition snapshots, count exact hits, drain round-robin |
| 7 | `output_assembly` | 1 cycle | Emit K23.7 subheader then ordered Type-2 hits |
| 8 | `fill_level_meter / counters` | 1 cycle | Derive `FILL_LEVEL`, `PUSH_COUNT`, `POP_COUNT`, `OVERWRITE_COUNT`, and cache-miss counters |

### Delivered Configuration

| Generic | Value | Description |
|---------|-------|-------------|
| `SEARCH_KEY_WIDTH` | 8 | Search key is `ts[11:4]` |
| `RING_BUFFER_N_ENTRY` | 512 | Ring / CAM depth |
| `SIDE_DATA_BITS` | 31 | Stored payload width beside the key |
| `INTERLEAVING_FACTOR` | 4 | Timestamp modulo fanout across sibling instances |
| `INTERLEAVING_INDEX` | 0 | This instance's accepted modulo lane |
| `N_PARTITIONS` | 4 | Physical match partitions |
| `ENCODER_LEAF_WIDTH` | 16 | Leaf width of the partitioned encoder |
| `ENCODER_PIPE_STAGES` | 4 | Delivered encoder pipeline depth |
| `DEBUG` | 1 | Synthesizable debug enabled |

### Legal Overwrite Semantics

The overwrite counter is part of the contract, not a fallback error path. If more than
`RING_BUFFER_N_ENTRY` hits with the same eligible search key accumulate inside the effective
service window before the first matching pop drains, the oldest residents can be overwritten.
That loss must be visible as:

- `PUSH_COUNT - POP_COUNT - OVERWRITE_COUNT == FILL_LEVEL`
- no unexplained resident loss beyond `OVERWRITE_COUNT`
- no invisible same-key tail residents after terminate-and-drain

Version `26.1.5.0422` carries the previously fixed same-key overwrite tail issue correction plus the current durable-DV-evidence publisher checkpoint and live-partition harness refresh, so the
final overwrite in a burst no longer erases the newly written resident.

---

## CSR Register Map

All registers are word-addressed through the `csr` Avalon-MM slave. Words `0` and `1`
form the common Mu3e identity header.

| Word | Name | Access | Description |
|------|------|--------|-------------|
| 0x00 | UID | RO | IP identifier, default ASCII `RBCM` |
| 0x01 | META | RW/RO | Write page selector, read VERSION / DATE / GIT / INSTANCE_ID |
| 0x02 | CTRL | RW | `[0] go`, `[1] soft_reset`, `[4] filter_inerr` |
| 0x03 | EXPECTED_LATENCY | RW | Read-pointer service delay in cycles |
| 0x04 | FILL_LEVEL | RO | Live fill estimate derived from push/pop/overwrite counters |
| 0x05 | INERR_COUNT | RO | Count of filtered timestamp-error ingress hits |
| 0x06 | PUSH_COUNT | RO | Total accepted push operations |
| 0x07 | POP_COUNT | RO | Total drained hits |
| 0x08 | OVERWRITE_COUNT | RO | Total overwrite events |
| 0x09 | CACHE_MISS_COUNT | RO | Count of cache-miss / empty-search output events |

### Runtime Configuration Workflow

1. Drive run control into `RUN_PREPARE` and wait for the DUT flush handshake.
2. Program `EXPECTED_LATENCY` and any desired `CTRL` bits.
3. Move through `SYNC` into `RUNNING`.
4. Observe `FILL_LEVEL`, `PUSH_COUNT`, `POP_COUNT`, `OVERWRITE_COUNT`, and `INERR_COUNT`.
5. On end of run, send `TERMINATING` and the lane-local end-of-run marker so the IP drains all
   remaining eligible residents before returning quiescent.

---

## Platform Designer GUI

The `_hw.tcl` presents four tabs in Platform Designer:

### Configuration Tab

**Overview** -- block-level function, clocking, and interface summary.

**Sizing** -- `SEARCH_KEY_WIDTH`, `RING_BUFFER_N_ENTRY`, `SIDE_DATA_BITS`, and the
derived storage estimates for CAM and side-RAM.

**Match Engine** -- `INTERLEAVING_FACTOR`, `INTERLEAVING_INDEX`, `N_PARTITIONS`,
`ENCODER_LEAF_WIDTH`, and `ENCODER_PIPE_STAGES`.

**Throughput** -- validation-callback estimates for nominal distributed-case pop rate
and worst-case single-partition drain rate.

**Advanced** -- contract notes about the fixed 10-word CSR aperture, identity header,
and compatibility of the packaged partitioned V2 core.

### Identity Tab

**Delivered Profile** -- packaged catalog revision and compatibility note.

**Versioning** -- `IP_UID`, `VERSION_MAJOR`, `VERSION_MINOR`, `VERSION_PATCH`, `BUILD`,
`VERSION_DATE`, `VERSION_GIT`, and `INSTANCE_ID`.

**Debug** -- single debug-level control for synthesizable / simulation-oriented debug.

### Interfaces Tab

Clock/reset, ingress `hit_type1`, ordered egress `hit_type2`, `run_control`, `csr`,
and `filllevel` interface descriptions.

### Register Map Tab

Interactive HTML table of the 10-word CSR window and the common identity header.

### Tab Overview

```
┌────────────────────────────────────────────────────────────────────────┐
│  Platform Designer Component Editor: ring_buffer_cam                 │
│                                                                       │
│  [Configuration Tab]                                                  │
│    ├─ Overview ── function and clocking summary                       │
│    ├─ Sizing ── search key, ring depth, side-data width              │
│    ├─ Match Engine ── interleaving, partitions, encoder shape        │
│    ├─ Throughput ── validation-derived pop-rate estimates            │
│    └─ Advanced ── integration notes and CSR aperture                 │
│                                                                       │
│  [Identity Tab]                                                       │
│    ├─ Delivered Profile ── packaged revision                          │
│    ├─ Versioning ── UID, META pages, VERSION_*, DATE, GIT            │
│    └─ Debug ── debug-level parameter                                  │
│                                                                       │
│  [Interfaces Tab] ── clock/reset, AVST, CSR, filllevel               │
│  [Register Map Tab] ── 10-word CSR window                            │
│                                                                       │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Presets

Four presets are shipped in `script/ring_buffer_cam_presets.qprs`:

| Preset | Partitions | Pipe | Depth | Use Case |
|--------|------------|------|-------|----------|
| **Default P4** | 4 | 4 | 512 | Delivered production configuration |
| **Reserved P3** | 2 | 3 | 512 | Timing / latency comparison against P4 |
| **Reserved P2** | 2 | 2 | 512 | Lower-latency partitioned comparison point |
| **Regression P1** | 1 | 2 | 512 | Single-partition regression reference |

The default shipped configuration is `Default P4`. The lower-partition presets are kept for
timing, area, and regression comparisons; they are not the recommended integration point.

---

## Verification Workflow

The active verification workflow lives under [`tb/`](tb/README.md), not in the older
monolithic signoff note.

- [`tb/DV_REPORT.md`](tb/DV_REPORT.md) — chief-architect dashboard
- [`tb/REPORT/README.md`](tb/REPORT/README.md) — per-case and per-cross-run evidence tree
- [`tb/DV_PLAN.md`](tb/DV_PLAN.md) — frozen bucket plan and signoff rules
- [`tb/BUG_HISTORY.md`](tb/BUG_HISTORY.md) — live bug ledger
- [`doc/SIGNOFF.md`](doc/SIGNOFF.md) — master signoff dashboard linking DV and synthesis status
- [`syn/SYN_REPORT.md`](syn/SYN_REPORT.md) — detailed standalone synthesis / timing evidence

Current closure status is summarized in [`doc/SIGNOFF.md`](doc/SIGNOFF.md), with detailed DV
evidence under `tb/` and detailed synthesis evidence in [`syn/SYN_REPORT.md`](syn/SYN_REPORT.md).

---

## Repository Layout

- [`rtl/ring_buffer_cam.vhd`](rtl/ring_buffer_cam.vhd) — packaged top-level wrapper
- [`rtl/ring_buffer_cam_v2_core.vhd`](rtl/ring_buffer_cam_v2_core.vhd) — active RTL core
- [`rtl/addr_enc_logic_partitioned.vhd`](rtl/addr_enc_logic_partitioned.vhd) — partitioned staged encoder
- [`script/ring_buffer_cam_hw.tcl`](script/ring_buffer_cam_hw.tcl) — Platform Designer packaging
- [`script/ring_buffer_cam_presets.qprs`](script/ring_buffer_cam_presets.qprs) — packaged presets
- [`tb/`](tb/README.md) — active DV workflow, UVM harness, reports, and bug history
- [`syn/`](syn) — standalone synthesis / timing work area

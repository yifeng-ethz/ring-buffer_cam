# ❌ DV Report — `ring_buffer_cam`

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-05-08` &nbsp;
**RTL variant:** `promoted_build_matrix` &nbsp; **Seed:** `1`

This page is the chief-architect dashboard. All per-case evidence lives under [`REPORT/`](REPORT/README.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / below target / known limitation &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ✅ | failed_cases | `0` |
| ❌ | signoff_runs_with_failures | `5` |
| ✅ | catalog_backlog_cases | `0` |
| ✅ | unimplemented_cases | `0` |
| ⚠️ | stale_artifacts | `5` |

## Signoff Scope

| field | claimed value |
|---|---|
| RTL_BUILD_MATRIX | `default_p2_pipe4, p2_pipe1, p2_pipe2, p2_pipe3, p4_n4_pipe4, depth64_p2_pipe4, depth384_p2_pipe4, depth4096_p2_pipe4` |
| G_N_PARTITIONS | `2, 4` |
| G_ENCODER_PIPE_STAGES | `1, 2, 3, 4` |
| G_INTERLEAVING_FACTOR | `4` |
| G_RING_BUFFER_N_ENTRY | `64, 384, 512, 4096` |
| probe_only_exclusions |  |

## Non-Claims

- DV closure is not claimed until required 30 s simulator-time signoff soaks CROSS-125..CROSS-129 have real passing logs with elapsed_ps >= 30000000000000.
- cross scope: DV_CROSS continuous-frame ladders are tracked separately from the canonical per-case isolated matrix in this refresh.
- branch total excludes 16 compile-time-dead bins in `addr_enc_logic_partitioned`: the `clear_onehot_bit()` out-of-range guard is unreachable for the generated callers, and the `result_valid_extra` branches are unreachable across the promoted `PIPE_STAGES=1..4` build matrix, including the p4-only `gen_addr_enc_logic(2/3)` instances.

## Bucket Summary

<!-- status: overall per-bucket health; merged columns: bucket-local ordered-merge percentages. -->

| status | bucket | catalog_planned | promoted | evidenced | backlog | merged | promoted functional |
|:---:|---|---:|---:|---:|---:|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 148 | 148 | 148 | 0 | stmt=98.23, branch=92.35, cond=81.82, expr=40.00, fsm_state=100.00, fsm_trans=73.33, toggle=52.32 | 100.0% (148/148) |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 140 | 140 | 140 | 0 | stmt=94.37, branch=83.80, cond=68.89, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.50 | 100.0% (140/140) |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 137 | 137 | 137 | 0 | stmt=92.60, branch=84.31, cond=71.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.27 | 100.0% (137/137) |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 138 | 138 | 138 | 0 | stmt=96.48, branch=88.48, cond=75.31, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.54 | 100.0% (138/138) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ✅ | stmt | 96.09 | 95.0 |
| ⚠️ | branch | 89.87 | 90.0 |
| ℹ️ | cond | 75.00 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ✅ | fsm_trans | 100.00 | 90.0 |
| ⚠️ | toggle | 73.79 | 80.0 |

- catalog_planned_cases: `563`
- promoted_signoff_cases: `563`
- evidenced_promoted_cases: `563`
- promoted functional coverage: `100.0% (563/563)`

## Signoff Runs

<!-- one row per run; follow the run_id link for the full transaction-growth curve. -->

| status | run_id | kind | build | seq | txns | cross_pct |
|:---:|---|---|---|---|---:|---:|
| ✅ | [`CROSS-008`](REPORT/cross/CROSS-008.md) | anchored_hybrid | default_p2_pipe4 | nightly promoted X117 anchor: GOOD(2048) → ERROR(64) → FLUSH → GOOD(2048) | 2048 | 88.8 |
| ✅ | [`CROSS-009`](REPORT/cross/CROSS-009.md) | anchored_hybrid | default_p2_pipe4 | nightly promoted X118 anchor: GOOD(2048) → TERM → IDLE → RUN_PREPARE → RUN → GOOD(2048) | 2048 | 88.8 |
| ✅ | [`CROSS-010`](REPORT/cross/CROSS-010.md) | anchored_hybrid | default_p2_pipe4 | overwrite-pressure GOOD(pool=1, λ=1.0, 10k) → X119 anchor → recovery GOOD(random, 10k) | 784 | 80.9 |
| ✅ | [`CROSS-015`](REPORT/cross/CROSS-015.md) | anchored_hybrid | default_p2_pipe4 | nightly curated all-bucket mix: B005/B006, E002, P001-style random push-pop, X117-style error+flush recovery, plus overwrite-pressure windows, all separated by random idle gaps | 2048 | 88.8 |
| ✅ | [`CROSS-076`](REPORT/cross/CROSS-076.md) | seed_sweep | default_p2_pipe4 | nightly hotspot overwrite soak derived from P111, 131072 txn same-ts pressure | 131072 | 88.8 |
| ✅ | [`CROSS-091`](REPORT/cross/CROSS-091.md) | seed_sweep | default_p2_pipe4 | X101-X102 promoted to a sustained bad-hit burst bracketed by 64-hit GOOD warmup/cooldown windows | 128 | 86.2 |
| ❌ | [`CROSS-125`](REPORT/cross/CROSS-125.md) | checkpoint_soak | default_p2_pipe4 | 30 s simulator-time sector-lock/arbiter soak: repeated B090/B091/B092/B130/B131/B133 plus E073/E102/E103, P031, X063 with random inter-case gaps | 0 | 0.0 |
| ❌ | [`CROSS-126`](REPORT/cross/CROSS-126.md) | checkpoint_soak | default_p2_pipe4 | 30 s simulator-time all-bucket soak: B005/B006, E082, P086, P096, X117, X118, and P110 replayed with random inter-case gaps | 0 | 0.0 |
| ❌ | [`CROSS-127`](REPORT/cross/CROSS-127.md) | checkpoint_soak | default_p2_pipe4 | 30 s simulator-time overwrite/recovery soak: B075/B079, E089, P125/P126, X119/X120 with random inter-case gaps | 0 | 0.0 |
| ❌ | [`CROSS-128`](REPORT/cross/CROSS-128.md) | checkpoint_soak | default_p2_pipe4 | 30 s simulator-time PROF-heavy soak: P110, P125, P126, P127, P128, P129 replayed with random inter-case gaps | 0 | 0.0 |
| ❌ | [`CROSS-129`](REPORT/cross/CROSS-129.md) | checkpoint_soak | default_p2_pipe4 | 30 s simulator-time ERROR-heavy soak: X117/X118/X119/X120/X129/X130/X131/X132 plus CROSS-091 bad-hit counter burst | 0 | 0.0 |

## Index

- [`REPORT/README.md`](REPORT/README.md) — reviewer entry point
- [`REPORT/buckets/`](REPORT/buckets/) — ordered-merge trace per bucket
- [`REPORT/cases/`](REPORT/cases/) — one page per case
- [`REPORT/cross/`](REPORT/cross/) — one page per signoff run
- [`DV_COV.md`](DV_COV.md) — coverage totals, ordering, and baseline scope
- [`DV_REPORT.json`](DV_REPORT.json) — machine-readable source of truth

_This dashboard is generated by `~/.codex/skills/dv-workflow/scripts/dv_report_gen.py`. Edits are overwritten; fix the JSON or the generator instead._

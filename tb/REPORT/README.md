# ring_buffer_cam — REPORT index

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-05-08` &nbsp;
**RTL variant:** `sv_promoted_build_matrix` &nbsp; **Seed:** `1`

## Legend

✅ pass / closed / target met &middot; ⚠️ partial / below target / known limitation &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Buckets

<!-- click a bucket row to open its ordered-merge trace and linked per-case pages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) |
|:---:|---|---:|---:|---|
| ⚠️ | [`BASIC`](buckets/BASIC.md) | 148 | 17 | stmt=86.06, branch=81.28, cond=65.22, expr=65.08, fsm_state=100.00, fsm_trans=50.00, toggle=32.85 |
| ⚠️ | [`EDGE`](buckets/EDGE.md) | 140 | 11 | stmt=85.57, branch=82.27, cond=66.00, expr=63.08, fsm_state=100.00, fsm_trans=50.00, toggle=33.01 |
| ⚠️ | [`PROF`](buckets/PROF.md) | 137 | 8 | stmt=88.78, branch=87.68, cond=80.00, expr=76.92, fsm_state=100.00, fsm_trans=50.00, toggle=36.21 |
| ⚠️ | [`ERROR`](buckets/ERROR.md) | 138 | 6 | stmt=94.02, branch=87.75, cond=80.00, expr=75.38, fsm_state=100.00, fsm_trans=61.11, toggle=30.80 |

## Cross / continuous-frame runs

| status | run_id | kind | build | bucket | seq | txns | cross_pct |
|:---:|---|---|---|---|---|---:|---:|
| ❌ | [`CROSS-125`](cross/CROSS-125.md) | checkpoint_soak | p4_n4_pipe4 | - | 30 s simulator-time SV p4/n4 sector-lock/decision-5 soak: B135/B136/B138/B139/B148, E130/E134/E135/E138/E139, P130/P132, X133/X138 with random inter-case gaps | 0 | 0.0 |
| ❌ | [`CROSS-126`](cross/CROSS-126.md) | checkpoint_soak | p4_n4_pipe4 | - | 30 s simulator-time SV p4/n4 accounting/drop all-bucket soak: B005/B006/B143-B147, E082/E140, P086/P096/P110/P133/P134, X117/X118/X135/X136 with random inter-case gaps | 0 | 0.0 |
| ❌ | [`CROSS-127`](cross/CROSS-127.md) | checkpoint_soak | p4_n4_pipe4 | - | 30 s simulator-time SV p4/n4 overwrite/recovery/metadata soak: B075/B079/B142, E089/E131, P125/P126/P135/P137, X119/X120/X134 with random inter-case gaps | 0 | 0.0 |
| ❌ | [`CROSS-128`](cross/CROSS-128.md) | checkpoint_soak | p4_n4_pipe4 | - | 30 s simulator-time SV p4/n4 PROF-heavy soak: P110/P125/P126/P127/P128/P129/P130/P131/P132/P134/P135/P137 replayed with random inter-case gaps | 0 | 0.0 |
| ❌ | [`CROSS-129`](cross/CROSS-129.md) | checkpoint_soak | p4_n4_pipe4 | - | 30 s simulator-time SV p4/n4 ERROR-heavy soak: X117/X118/X119/X120/X129-X138 plus CROSS-091 bad-hit counter burst | 0 | 0.0 |

## Random long-run cases

<!-- each random case has a txn_growth page; pages are pending until checkpoint UCDBs exist. -->

| status | case_id | bucket | observed_txn | growth_page |
|:---:|---|---|---:|---|
| ❓ | [`P001`](cases/P001.md) | PROF | 0 | [growth](txn_growth/P001.md) |
| ❓ | [`P002`](cases/P002.md) | PROF | 0 | [growth](txn_growth/P002.md) |
| ❓ | [`P003`](cases/P003.md) | PROF | 0 | [growth](txn_growth/P003.md) |
| ❓ | [`P005`](cases/P005.md) | PROF | 0 | [growth](txn_growth/P005.md) |
| ❓ | [`P006`](cases/P006.md) | PROF | 0 | [growth](txn_growth/P006.md) |
| ❓ | [`P007`](cases/P007.md) | PROF | 0 | [growth](txn_growth/P007.md) |
| ❓ | [`P008`](cases/P008.md) | PROF | 0 | [growth](txn_growth/P008.md) |
| ❓ | [`P009`](cases/P009.md) | PROF | 0 | [growth](txn_growth/P009.md) |
| ❓ | [`P010`](cases/P010.md) | PROF | 0 | [growth](txn_growth/P010.md) |
| ❓ | [`P011`](cases/P011.md) | PROF | 0 | [growth](txn_growth/P011.md) |
| ❓ | [`P012`](cases/P012.md) | PROF | 0 | [growth](txn_growth/P012.md) |
| ❓ | [`P013`](cases/P013.md) | PROF | 0 | [growth](txn_growth/P013.md) |
| ❓ | [`P014`](cases/P014.md) | PROF | 0 | [growth](txn_growth/P014.md) |
| ❓ | [`P015`](cases/P015.md) | PROF | 0 | [growth](txn_growth/P015.md) |
| ❓ | [`P016`](cases/P016.md) | PROF | 0 | [growth](txn_growth/P016.md) |
| ❓ | [`P017`](cases/P017.md) | PROF | 0 | [growth](txn_growth/P017.md) |
| ❓ | [`P018`](cases/P018.md) | PROF | 0 | [growth](txn_growth/P018.md) |
| ❓ | [`P019`](cases/P019.md) | PROF | 0 | [growth](txn_growth/P019.md) |
| ❓ | [`P020`](cases/P020.md) | PROF | 0 | [growth](txn_growth/P020.md) |
| ❓ | [`P021`](cases/P021.md) | PROF | 0 | [growth](txn_growth/P021.md) |
| ❓ | [`P022`](cases/P022.md) | PROF | 0 | [growth](txn_growth/P022.md) |
| ❓ | [`P023`](cases/P023.md) | PROF | 0 | [growth](txn_growth/P023.md) |
| ❓ | [`P024`](cases/P024.md) | PROF | 0 | [growth](txn_growth/P024.md) |
| ❓ | [`P025`](cases/P025.md) | PROF | 0 | [growth](txn_growth/P025.md) |
| ❓ | [`P026`](cases/P026.md) | PROF | 0 | [growth](txn_growth/P026.md) |
| ❓ | [`P027`](cases/P027.md) | PROF | 0 | [growth](txn_growth/P027.md) |
| ❓ | [`P028`](cases/P028.md) | PROF | 0 | [growth](txn_growth/P028.md) |
| ❓ | [`P029`](cases/P029.md) | PROF | 0 | [growth](txn_growth/P029.md) |
| ❓ | [`P030`](cases/P030.md) | PROF | 0 | [growth](txn_growth/P030.md) |
| ❓ | [`P031`](cases/P031.md) | PROF | 0 | [growth](txn_growth/P031.md) |
| ❓ | [`P032`](cases/P032.md) | PROF | 0 | [growth](txn_growth/P032.md) |
| ❓ | [`P033`](cases/P033.md) | PROF | 0 | [growth](txn_growth/P033.md) |
| ❓ | [`P034`](cases/P034.md) | PROF | 0 | [growth](txn_growth/P034.md) |
| ❓ | [`P035`](cases/P035.md) | PROF | 0 | [growth](txn_growth/P035.md) |
| ❓ | [`P036`](cases/P036.md) | PROF | 0 | [growth](txn_growth/P036.md) |
| ❓ | [`P037`](cases/P037.md) | PROF | 0 | [growth](txn_growth/P037.md) |
| ❓ | [`P038`](cases/P038.md) | PROF | 0 | [growth](txn_growth/P038.md) |
| ❓ | [`P039`](cases/P039.md) | PROF | 0 | [growth](txn_growth/P039.md) |
| ❓ | [`P040`](cases/P040.md) | PROF | 0 | [growth](txn_growth/P040.md) |
| ❓ | [`P041`](cases/P041.md) | PROF | 0 | [growth](txn_growth/P041.md) |
| ❓ | [`P042`](cases/P042.md) | PROF | 0 | [growth](txn_growth/P042.md) |
| ❓ | [`P043`](cases/P043.md) | PROF | 0 | [growth](txn_growth/P043.md) |
| ❓ | [`P044`](cases/P044.md) | PROF | 0 | [growth](txn_growth/P044.md) |
| ❓ | [`P045`](cases/P045.md) | PROF | 0 | [growth](txn_growth/P045.md) |
| ❓ | [`P050`](cases/P050.md) | PROF | 0 | [growth](txn_growth/P050.md) |
| ❓ | [`P051`](cases/P051.md) | PROF | 0 | [growth](txn_growth/P051.md) |
| ❓ | [`P052`](cases/P052.md) | PROF | 0 | [growth](txn_growth/P052.md) |
| ❓ | [`P053`](cases/P053.md) | PROF | 0 | [growth](txn_growth/P053.md) |
| ❓ | [`P054`](cases/P054.md) | PROF | 0 | [growth](txn_growth/P054.md) |
| ❓ | [`P055`](cases/P055.md) | PROF | 0 | [growth](txn_growth/P055.md) |
| ❓ | [`P056`](cases/P056.md) | PROF | 0 | [growth](txn_growth/P056.md) |
| ❓ | [`P057`](cases/P057.md) | PROF | 0 | [growth](txn_growth/P057.md) |
| ❓ | [`P058`](cases/P058.md) | PROF | 0 | [growth](txn_growth/P058.md) |
| ❓ | [`P059`](cases/P059.md) | PROF | 0 | [growth](txn_growth/P059.md) |
| ❓ | [`P060`](cases/P060.md) | PROF | 0 | [growth](txn_growth/P060.md) |
| ❓ | [`P061`](cases/P061.md) | PROF | 0 | [growth](txn_growth/P061.md) |
| ❓ | [`P062`](cases/P062.md) | PROF | 0 | [growth](txn_growth/P062.md) |
| ❓ | [`P063`](cases/P063.md) | PROF | 0 | [growth](txn_growth/P063.md) |
| ❓ | [`P064`](cases/P064.md) | PROF | 0 | [growth](txn_growth/P064.md) |
| ❓ | [`P065`](cases/P065.md) | PROF | 0 | [growth](txn_growth/P065.md) |
| ❓ | [`P066`](cases/P066.md) | PROF | 0 | [growth](txn_growth/P066.md) |
| ❓ | [`P067`](cases/P067.md) | PROF | 0 | [growth](txn_growth/P067.md) |
| ❓ | [`P068`](cases/P068.md) | PROF | 0 | [growth](txn_growth/P068.md) |
| ❓ | [`P069`](cases/P069.md) | PROF | 0 | [growth](txn_growth/P069.md) |
| ❓ | [`P070`](cases/P070.md) | PROF | 0 | [growth](txn_growth/P070.md) |
| ❓ | [`P071`](cases/P071.md) | PROF | 0 | [growth](txn_growth/P071.md) |
| ❓ | [`P072`](cases/P072.md) | PROF | 0 | [growth](txn_growth/P072.md) |
| ❓ | [`P073`](cases/P073.md) | PROF | 0 | [growth](txn_growth/P073.md) |
| ❓ | [`P074`](cases/P074.md) | PROF | 0 | [growth](txn_growth/P074.md) |
| ❓ | [`P075`](cases/P075.md) | PROF | 0 | [growth](txn_growth/P075.md) |
| ❓ | [`P076`](cases/P076.md) | PROF | 0 | [growth](txn_growth/P076.md) |
| ❓ | [`P077`](cases/P077.md) | PROF | 0 | [growth](txn_growth/P077.md) |
| ❓ | [`P078`](cases/P078.md) | PROF | 0 | [growth](txn_growth/P078.md) |
| ❓ | [`P079`](cases/P079.md) | PROF | 0 | [growth](txn_growth/P079.md) |
| ❓ | [`P080`](cases/P080.md) | PROF | 0 | [growth](txn_growth/P080.md) |
| ❓ | [`P081`](cases/P081.md) | PROF | 0 | [growth](txn_growth/P081.md) |
| ❓ | [`P082`](cases/P082.md) | PROF | 0 | [growth](txn_growth/P082.md) |
| ❓ | [`P083`](cases/P083.md) | PROF | 0 | [growth](txn_growth/P083.md) |
| ❓ | [`P084`](cases/P084.md) | PROF | 0 | [growth](txn_growth/P084.md) |
| ❓ | [`P085`](cases/P085.md) | PROF | 0 | [growth](txn_growth/P085.md) |
| ❓ | [`P086`](cases/P086.md) | PROF | 0 | [growth](txn_growth/P086.md) |
| ❓ | [`P087`](cases/P087.md) | PROF | 0 | [growth](txn_growth/P087.md) |
| ❓ | [`P088`](cases/P088.md) | PROF | 0 | [growth](txn_growth/P088.md) |
| ❓ | [`P089`](cases/P089.md) | PROF | 0 | [growth](txn_growth/P089.md) |
| ❓ | [`P090`](cases/P090.md) | PROF | 0 | [growth](txn_growth/P090.md) |
| ❓ | [`P091`](cases/P091.md) | PROF | 0 | [growth](txn_growth/P091.md) |
| ❓ | [`P092`](cases/P092.md) | PROF | 0 | [growth](txn_growth/P092.md) |
| ❓ | [`P093`](cases/P093.md) | PROF | 0 | [growth](txn_growth/P093.md) |
| ❓ | [`P094`](cases/P094.md) | PROF | 0 | [growth](txn_growth/P094.md) |
| ❓ | [`P095`](cases/P095.md) | PROF | 0 | [growth](txn_growth/P095.md) |
| ❓ | [`P096`](cases/P096.md) | PROF | 0 | [growth](txn_growth/P096.md) |
| ❓ | [`P097`](cases/P097.md) | PROF | 0 | [growth](txn_growth/P097.md) |
| ❓ | [`P098`](cases/P098.md) | PROF | 0 | [growth](txn_growth/P098.md) |
| ❓ | [`P099`](cases/P099.md) | PROF | 0 | [growth](txn_growth/P099.md) |
| ❓ | [`P100`](cases/P100.md) | PROF | 0 | [growth](txn_growth/P100.md) |
| ❓ | [`P101`](cases/P101.md) | PROF | 0 | [growth](txn_growth/P101.md) |
| ❓ | [`P102`](cases/P102.md) | PROF | 0 | [growth](txn_growth/P102.md) |
| ❓ | [`P103`](cases/P103.md) | PROF | 0 | [growth](txn_growth/P103.md) |
| ❓ | [`P104`](cases/P104.md) | PROF | 0 | [growth](txn_growth/P104.md) |
| ❓ | [`P105`](cases/P105.md) | PROF | 0 | [growth](txn_growth/P105.md) |
| ❓ | [`P106`](cases/P106.md) | PROF | 0 | [growth](txn_growth/P106.md) |
| ❓ | [`P107`](cases/P107.md) | PROF | 0 | [growth](txn_growth/P107.md) |
| ❓ | [`P108`](cases/P108.md) | PROF | 0 | [growth](txn_growth/P108.md) |
| ❓ | [`P109`](cases/P109.md) | PROF | 0 | [growth](txn_growth/P109.md) |
| ❓ | [`P110`](cases/P110.md) | PROF | 0 | [growth](txn_growth/P110.md) |
| ❓ | [`P111`](cases/P111.md) | PROF | 0 | [growth](txn_growth/P111.md) |
| ❓ | [`P112`](cases/P112.md) | PROF | 0 | [growth](txn_growth/P112.md) |
| ❓ | [`P113`](cases/P113.md) | PROF | 0 | [growth](txn_growth/P113.md) |
| ❓ | [`P114`](cases/P114.md) | PROF | 0 | [growth](txn_growth/P114.md) |
| ❓ | [`P115`](cases/P115.md) | PROF | 0 | [growth](txn_growth/P115.md) |
| ❓ | [`P116`](cases/P116.md) | PROF | 0 | [growth](txn_growth/P116.md) |
| ❓ | [`P117`](cases/P117.md) | PROF | 0 | [growth](txn_growth/P117.md) |
| ❓ | [`P118`](cases/P118.md) | PROF | 0 | [growth](txn_growth/P118.md) |
| ❓ | [`P119`](cases/P119.md) | PROF | 0 | [growth](txn_growth/P119.md) |
| ❓ | [`P120`](cases/P120.md) | PROF | 0 | [growth](txn_growth/P120.md) |
| ❓ | [`P121`](cases/P121.md) | PROF | 0 | [growth](txn_growth/P121.md) |
| ❓ | [`P122`](cases/P122.md) | PROF | 0 | [growth](txn_growth/P122.md) |
| ❓ | [`P123`](cases/P123.md) | PROF | 0 | [growth](txn_growth/P123.md) |
| ❓ | [`P124`](cases/P124.md) | PROF | 0 | [growth](txn_growth/P124.md) |
| ✅ | [`P125`](cases/P125.md) | PROF | 0 | [growth](txn_growth/P125.md) |
| ✅ | [`P126`](cases/P126.md) | PROF | 0 | [growth](txn_growth/P126.md) |
| ✅ | [`P127`](cases/P127.md) | PROF | 0 | [growth](txn_growth/P127.md) |
| ❓ | [`P128`](cases/P128.md) | PROF | 0 | [growth](txn_growth/P128.md) |
| ❓ | [`P130`](cases/P130.md) | PROF | 576 | [growth](txn_growth/P130.md) |
| ❓ | [`P131`](cases/P131.md) | PROF | 576 | [growth](txn_growth/P131.md) |
| ❓ | [`P132`](cases/P132.md) | PROF | 576 | [growth](txn_growth/P132.md) |
| ❓ | [`P133`](cases/P133.md) | PROF | 44 | [growth](txn_growth/P133.md) |
| ❓ | [`P134`](cases/P134.md) | PROF | 1973 | [growth](txn_growth/P134.md) |
| ❓ | [`P135`](cases/P135.md) | PROF | 256 | [growth](txn_growth/P135.md) |
| ❓ | [`P136`](cases/P136.md) | PROF | 768 | [growth](txn_growth/P136.md) |
| ❓ | [`P137`](cases/P137.md) | PROF | 576 | [growth](txn_growth/P137.md) |

## Totals

<!-- merged_total_code_coverage is the merge across all evidenced cases in all buckets. -->

- planned_cases = `563`
- evidenced_cases = `42`
- excluded_cases = `0`
- merged total code coverage: `stmt=86.06, branch=81.28, cond=65.22, expr=65.08, fsm_state=100.00, fsm_trans=50.00, toggle=41.77`
- functional coverage: `7.46% (42/563)`

---
_[Dashboard](../DV_REPORT.md) &middot; [Coverage](../DV_COV.md)_

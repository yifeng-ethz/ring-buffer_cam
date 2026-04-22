# ring_buffer_cam — REPORT index

**DUT:** `ring_buffer_cam` &nbsp; **Date:** `2026-04-22` &nbsp;
**RTL variant:** `promoted_build_matrix` &nbsp; **Seed:** `1`

## Legend

✅ pass / closed / target met &middot; ⚠️ partial / below target / known limitation &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Buckets

<!-- click a bucket row to open its ordered-merge trace and linked per-case pages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) |
|:---:|---|---:|---:|---|
| ⚠️ | [`BASIC`](buckets/BASIC.md) | 129 | 129 | stmt=99.13, branch=93.87, cond=81.46, expr=40.00, fsm_state=100.00, fsm_trans=73.33, toggle=70.72 |
| ⚠️ | [`EDGE`](buckets/EDGE.md) | 129 | 123 | stmt=94.66, branch=84.23, cond=68.36, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=84.41 |
| ⚠️ | [`PROF`](buckets/PROF.md) | 129 | 84 | stmt=93.05, branch=84.87, cond=70.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=81.57 |
| ⚠️ | [`ERROR`](buckets/ERROR.md) | 132 | 128 | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.60 |

## Cross / continuous-frame runs

| status | run_id | kind | build | bucket | seq | txns | cross_pct |
|:---:|---|---|---|---|---|---:|---:|

## Random long-run cases

<!-- each random case has a txn_growth page; pages are pending until checkpoint UCDBs exist. -->

| status | case_id | bucket | observed_txn | growth_page |
|:---:|---|---|---:|---|
| ❓ | [`P001`](cases/P001.md) | PROF | 61 | [growth](txn_growth/P001.md) |
| ❓ | [`P002`](cases/P002.md) | PROF | 616 | [growth](txn_growth/P002.md) |
| ❓ | [`P003`](cases/P003.md) | PROF | 380 | [growth](txn_growth/P003.md) |
| ❓ | [`P005`](cases/P005.md) | PROF | 125 | [growth](txn_growth/P005.md) |
| ❓ | [`P006`](cases/P006.md) | PROF | 1024 | [growth](txn_growth/P006.md) |
| ❓ | [`P007`](cases/P007.md) | PROF | 4096 | [growth](txn_growth/P007.md) |
| ❓ | [`P008`](cases/P008.md) | PROF | 4096 | [growth](txn_growth/P008.md) |
| ❓ | [`P009`](cases/P009.md) | PROF | 4096 | [growth](txn_growth/P009.md) |
| ❓ | [`P010`](cases/P010.md) | PROF | 4096 | [growth](txn_growth/P010.md) |
| ❓ | [`P011`](cases/P011.md) | PROF | 4096 | [growth](txn_growth/P011.md) |
| ❓ | [`P012`](cases/P012.md) | PROF | 4096 | [growth](txn_growth/P012.md) |
| ❓ | [`P013`](cases/P013.md) | PROF | 4096 | [growth](txn_growth/P013.md) |
| ❓ | [`P014`](cases/P014.md) | PROF | 4096 | [growth](txn_growth/P014.md) |
| ❓ | [`P015`](cases/P015.md) | PROF | 4096 | [growth](txn_growth/P015.md) |
| ❓ | [`P016`](cases/P016.md) | PROF | 4096 | [growth](txn_growth/P016.md) |
| ❓ | [`P017`](cases/P017.md) | PROF | 4096 | [growth](txn_growth/P017.md) |
| ❓ | [`P018`](cases/P018.md) | PROF | 4096 | [growth](txn_growth/P018.md) |
| ❓ | [`P019`](cases/P019.md) | PROF | 4096 | [growth](txn_growth/P019.md) |
| ❓ | [`P020`](cases/P020.md) | PROF | 4096 | [growth](txn_growth/P020.md) |
| ❓ | [`P021`](cases/P021.md) | PROF | 4096 | [growth](txn_growth/P021.md) |
| ❓ | [`P022`](cases/P022.md) | PROF | 4096 | [growth](txn_growth/P022.md) |
| ❓ | [`P023`](cases/P023.md) | PROF | 4096 | [growth](txn_growth/P023.md) |
| ❓ | [`P024`](cases/P024.md) | PROF | 513 | [growth](txn_growth/P024.md) |
| ❓ | [`P025`](cases/P025.md) | PROF | 50000 | [growth](txn_growth/P025.md) |
| ❓ | [`P026`](cases/P026.md) | PROF | 4096 | [growth](txn_growth/P026.md) |
| ❓ | [`P027`](cases/P027.md) | PROF | 2816 | [growth](txn_growth/P027.md) |
| ❓ | [`P028`](cases/P028.md) | PROF | 40000 | [growth](txn_growth/P028.md) |
| ❓ | [`P029`](cases/P029.md) | PROF | 20000 | [growth](txn_growth/P029.md) |
| ❓ | [`P030`](cases/P030.md) | PROF | 2048 | [growth](txn_growth/P030.md) |
| ❓ | [`P031`](cases/P031.md) | PROF | 20000 | [growth](txn_growth/P031.md) |
| ❓ | [`P032`](cases/P032.md) | PROF | 4096 | [growth](txn_growth/P032.md) |
| ❓ | [`P033`](cases/P033.md) | PROF | 4096 | [growth](txn_growth/P033.md) |
| ❓ | [`P034`](cases/P034.md) | PROF | 3072 | [growth](txn_growth/P034.md) |
| ❓ | [`P035`](cases/P035.md) | PROF | 3072 | [growth](txn_growth/P035.md) |
| ❓ | [`P036`](cases/P036.md) | PROF | 30000 | [growth](txn_growth/P036.md) |
| ❓ | [`P037`](cases/P037.md) | PROF | 30000 | [growth](txn_growth/P037.md) |
| ❓ | [`P038`](cases/P038.md) | PROF | 30000 | [growth](txn_growth/P038.md) |
| ❓ | [`P039`](cases/P039.md) | PROF | 5120 | [growth](txn_growth/P039.md) |
| ❓ | [`P040`](cases/P040.md) | PROF | 20000 | [growth](txn_growth/P040.md) |
| ❓ | [`P041`](cases/P041.md) | PROF | 30000 | [growth](txn_growth/P041.md) |
| ❓ | [`P042`](cases/P042.md) | PROF | 30000 | [growth](txn_growth/P042.md) |
| ❓ | [`P043`](cases/P043.md) | PROF | 30000 | [growth](txn_growth/P043.md) |
| ❓ | [`P044`](cases/P044.md) | PROF | 40000 | [growth](txn_growth/P044.md) |
| ❓ | [`P045`](cases/P045.md) | PROF | 50000 | [growth](txn_growth/P045.md) |
| ❓ | [`P050`](cases/P050.md) | PROF | 512 | [growth](txn_growth/P050.md) |
| ❓ | [`P051`](cases/P051.md) | PROF | 288 | [growth](txn_growth/P051.md) |
| ❓ | [`P052`](cases/P052.md) | PROF | 511 | [growth](txn_growth/P052.md) |
| ❓ | [`P053`](cases/P053.md) | PROF | 257 | [growth](txn_growth/P053.md) |
| ❓ | [`P054`](cases/P054.md) | PROF | 512 | [growth](txn_growth/P054.md) |
| ❓ | [`P055`](cases/P055.md) | PROF | 512 | [growth](txn_growth/P055.md) |
| ❓ | [`P056`](cases/P056.md) | PROF | 512 | [growth](txn_growth/P056.md) |
| ❓ | [`P057`](cases/P057.md) | PROF | 512 | [growth](txn_growth/P057.md) |
| ❓ | [`P058`](cases/P058.md) | PROF | 257 | [growth](txn_growth/P058.md) |
| ❓ | [`P059`](cases/P059.md) | PROF | 256 | [growth](txn_growth/P059.md) |
| ❓ | [`P060`](cases/P060.md) | PROF | 768 | [growth](txn_growth/P060.md) |
| ❓ | [`P061`](cases/P061.md) | PROF | 512 | [growth](txn_growth/P061.md) |
| ❓ | [`P062`](cases/P062.md) | PROF | 384 | [growth](txn_growth/P062.md) |
| ❓ | [`P063`](cases/P063.md) | PROF | 257 | [growth](txn_growth/P063.md) |
| ❓ | [`P064`](cases/P064.md) | PROF | 1024 | [growth](txn_growth/P064.md) |
| ❓ | [`P066`](cases/P066.md) | PROF | 85 | [growth](txn_growth/P066.md) |
| ❓ | [`P111`](cases/P111.md) | PROF | 576 | [growth](txn_growth/P111.md) |
| ❓ | [`P112`](cases/P112.md) | PROF | 736 | [growth](txn_growth/P112.md) |
| ❓ | [`P113`](cases/P113.md) | PROF | 1024 | [growth](txn_growth/P113.md) |
| ❓ | [`P114`](cases/P114.md) | PROF | 4640 | [growth](txn_growth/P114.md) |
| ❓ | [`P115`](cases/P115.md) | PROF | 4640 | [growth](txn_growth/P115.md) |
| ❓ | [`P116`](cases/P116.md) | PROF | 640 | [growth](txn_growth/P116.md) |
| ❓ | [`P117`](cases/P117.md) | PROF | 640 | [growth](txn_growth/P117.md) |
| ❓ | [`P118`](cases/P118.md) | PROF | 640 | [growth](txn_growth/P118.md) |
| ❓ | [`P119`](cases/P119.md) | PROF | 768 | [growth](txn_growth/P119.md) |
| ❓ | [`P120`](cases/P120.md) | PROF | 4640 | [growth](txn_growth/P120.md) |
| ❓ | [`P121`](cases/P121.md) | PROF | 640 | [growth](txn_growth/P121.md) |
| ❓ | [`P122`](cases/P122.md) | PROF | 640 | [growth](txn_growth/P122.md) |
| ❓ | [`P123`](cases/P123.md) | PROF | 1024 | [growth](txn_growth/P123.md) |
| ❓ | [`P124`](cases/P124.md) | PROF | 1024 | [growth](txn_growth/P124.md) |
| ✅ | [`P125`](cases/P125.md) | PROF | 1000000 | [growth](txn_growth/P125.md) |
| ✅ | [`P126`](cases/P126.md) | PROF | 1000000 | [growth](txn_growth/P126.md) |
| ✅ | [`P127`](cases/P127.md) | PROF | 1000512 | [growth](txn_growth/P127.md) |
| ❓ | [`P128`](cases/P128.md) | PROF | 2048 | [growth](txn_growth/P128.md) |

## Totals

<!-- merged_total_code_coverage is the merge across all evidenced cases in all buckets. -->

- planned_cases = `519`
- evidenced_cases = `464`
- excluded_cases = `0`
- merged total code coverage: `stmt=97.16, branch=90.11, cond=74.58, expr=40.00, fsm_state=100.00, fsm_trans=100.00, toggle=86.71`
- functional coverage: `100.0% (464/464)`

---
_[Dashboard](../DV_REPORT.md) &middot; [Coverage](../DV_COV.md)_

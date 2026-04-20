# ⚠️ BASIC bucket

**Planned:** `129` &nbsp; **Evidenced:** `121` &nbsp; **Status:** ⚠️

## Merged code coverage (this bucket)

<!-- column legend:
  metric          = code-coverage category (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle)
  merged_pct      = bucket-local ordered-merge percentage across all evidenced cases
  target          = workflow coverage target (blank = no hard target for that category)
  status          = target check vs merged_pct
-->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 95.69 | 95.0 |
| ⚠️ | branch | 86.18 | 90.0 |
| ℹ️ | cond | 70.23 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 71.86 | 80.0 |

## Ordered merge trace

<!-- each row is the merged coverage total after that case was added to the bucket in case-id order. -->

| status | step | case_id | merged_total (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | detail |
|:---:|---:|---|---|---|
| ✅ | 1 | `B001` | stmt=78.01, branch=41.44, cond=25.95, expr=0.00, fsm_state=12.50, fsm_trans=0.00, toggle=2.57 | [case](../cases/B001.md) |
| ✅ | 2 | `B002` | stmt=78.27, branch=41.98, cond=25.95, expr=0.00, fsm_state=12.50, fsm_trans=0.00, toggle=3.36 | [case](../cases/B002.md) |
| ✅ | 3 | `B003` | stmt=93.29, branch=76.76, cond=53.44, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=32.38 | [case](../cases/B003.md) |
| ✅ | 4 | `B004` | stmt=93.29, branch=76.76, cond=53.44, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=32.98 | [case](../cases/B004.md) |
| ✅ | 5 | `B005` | stmt=93.39, branch=77.12, cond=53.44, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=39.43 | [case](../cases/B005.md) |
| ✅ | 6 | `B006` | stmt=93.65, branch=77.84, cond=54.20, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=45.77 | [case](../cases/B006.md) |
| ✅ | 7 | `B007` | stmt=93.65, branch=77.84, cond=54.20, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=47.75 | [case](../cases/B007.md) |
| ✅ | 8 | `B008` | stmt=93.65, branch=77.84, cond=54.20, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=47.75 | [case](../cases/B008.md) |
| ✅ | 9 | `B009` | stmt=94.05, branch=80.54, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.08 | [case](../cases/B009.md) |
| ✅ | 10 | `B010` | stmt=93.24, branch=79.93, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.19 | [case](../cases/B010.md) |
| ✅ | 11 | `B011` | stmt=93.24, branch=79.93, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.37 | [case](../cases/B011.md) |
| ✅ | 12 | `B012` | stmt=93.34, branch=80.46, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.38 | [case](../cases/B012.md) |
| ✅ | 13 | `B013` | stmt=93.39, branch=80.64, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.38 | [case](../cases/B013.md) |
| ✅ | 14 | `B014` | stmt=93.39, branch=80.64, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.38 | [case](../cases/B014.md) |
| ✅ | 15 | `B015` | stmt=93.39, branch=80.64, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.38 | [case](../cases/B015.md) |
| ✅ | 16 | `B016` | stmt=93.39, branch=80.64, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.38 | [case](../cases/B016.md) |
| ✅ | 17 | `B017` | stmt=93.39, branch=80.64, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.02 | [case](../cases/B017.md) |
| ✅ | 18 | `B018` | stmt=93.39, branch=80.64, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.02 | [case](../cases/B018.md) |
| ✅ | 19 | `B019` | stmt=93.39, branch=80.64, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.47 | [case](../cases/B019.md) |
| ✅ | 20 | `B020` | stmt=93.39, branch=80.64, cond=61.07, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.02 | [case](../cases/B020.md) |
| ✅ | 21 | `B021` | stmt=93.49, branch=80.82, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.02 | [case](../cases/B021.md) |
| ✅ | 22 | `B022` | stmt=93.49, branch=80.99, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.10 | [case](../cases/B022.md) |
| ✅ | 23 | `B023` | stmt=93.54, branch=81.17, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.10 | [case](../cases/B023.md) |
| ✅ | 24 | `B024` | stmt=93.54, branch=81.17, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.37 | [case](../cases/B024.md) |
| ✅ | 25 | `B025` | stmt=93.54, branch=81.17, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.37 | [case](../cases/B025.md) |
| ✅ | 26 | `B026` | stmt=93.54, branch=81.17, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.37 | [case](../cases/B026.md) |
| ✅ | 27 | `B027` | stmt=93.54, branch=81.17, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.00 | [case](../cases/B027.md) |
| ✅ | 28 | `B028` | stmt=93.54, branch=81.17, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.00 | [case](../cases/B028.md) |
| ❓ | 29 | `B029` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B029.md) |
| ❓ | 30 | `B030` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B030.md) |
| ✅ | 31 | `B031` | stmt=93.54, branch=81.17, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.22 | [case](../cases/B031.md) |
| ✅ | 32 | `B032` | stmt=93.54, branch=81.17, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.22 | [case](../cases/B032.md) |
| ✅ | 33 | `B033` | stmt=93.64, branch=81.53, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.28 | [case](../cases/B033.md) |
| ✅ | 34 | `B034` | stmt=93.64, branch=81.53, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.28 | [case](../cases/B034.md) |
| ✅ | 35 | `B035` | stmt=93.64, branch=81.53, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.28 | [case](../cases/B035.md) |
| ✅ | 36 | `B036` | stmt=93.64, branch=81.53, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.28 | [case](../cases/B036.md) |
| ✅ | 37 | `B037` | stmt=93.64, branch=81.53, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.28 | [case](../cases/B037.md) |
| ✅ | 38 | `B038` | stmt=93.64, branch=81.53, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.28 | [case](../cases/B038.md) |
| ✅ | 39 | `B039` | stmt=93.64, branch=81.53, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.28 | [case](../cases/B039.md) |
| ✅ | 40 | `B040` | stmt=93.64, branch=81.53, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.28 | [case](../cases/B040.md) |
| ✅ | 41 | `B041` | stmt=93.64, branch=81.53, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.33 | [case](../cases/B041.md) |
| ✅ | 42 | `B042` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.41 | [case](../cases/B042.md) |
| ✅ | 43 | `B043` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.44 | [case](../cases/B043.md) |
| ✅ | 44 | `B044` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.44 | [case](../cases/B044.md) |
| ✅ | 45 | `B045` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.44 | [case](../cases/B045.md) |
| ✅ | 46 | `B046` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.44 | [case](../cases/B046.md) |
| ✅ | 47 | `B047` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.63 | [case](../cases/B047.md) |
| ✅ | 48 | `B048` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.63 | [case](../cases/B048.md) |
| ✅ | 49 | `B049` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.71 | [case](../cases/B049.md) |
| ✅ | 50 | `B050` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.79 | [case](../cases/B050.md) |
| ✅ | 51 | `B051` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.79 | [case](../cases/B051.md) |
| ✅ | 52 | `B052` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.94 | [case](../cases/B052.md) |
| ✅ | 53 | `B053` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.94 | [case](../cases/B053.md) |
| ✅ | 54 | `B054` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.94 | [case](../cases/B054.md) |
| ✅ | 55 | `B055` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.94 | [case](../cases/B055.md) |
| ✅ | 56 | `B056` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.94 | [case](../cases/B056.md) |
| ✅ | 57 | `B057` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.94 | [case](../cases/B057.md) |
| ✅ | 58 | `B058` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.98 | [case](../cases/B058.md) |
| ✅ | 59 | `B059` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.98 | [case](../cases/B059.md) |
| ✅ | 60 | `B060` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.98 | [case](../cases/B060.md) |
| ✅ | 61 | `B061` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.98 | [case](../cases/B061.md) |
| ✅ | 62 | `B062` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.98 | [case](../cases/B062.md) |
| ✅ | 63 | `B063` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=52.98 | [case](../cases/B063.md) |
| ✅ | 64 | `B064` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=53.04 | [case](../cases/B064.md) |
| ✅ | 65 | `B065` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=53.04 | [case](../cases/B065.md) |
| ✅ | 66 | `B066` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=54.71 | [case](../cases/B066.md) |
| ❓ | 67 | `B067` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B067.md) |
| ✅ | 68 | `B068` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=56.42 | [case](../cases/B068.md) |
| ✅ | 69 | `B069` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=56.42 | [case](../cases/B069.md) |
| ✅ | 70 | `B070` | stmt=92.68, branch=80.91, cond=64.12, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=56.42 | [case](../cases/B070.md) |
| ✅ | 71 | `B071` | stmt=94.45, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.22 | [case](../cases/B071.md) |
| ✅ | 72 | `B072` | stmt=94.45, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.25 | [case](../cases/B072.md) |
| ✅ | 73 | `B073` | stmt=94.45, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.25 | [case](../cases/B073.md) |
| ✅ | 74 | `B074` | stmt=94.45, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.25 | [case](../cases/B074.md) |
| ✅ | 75 | `B075` | stmt=94.45, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.28 | [case](../cases/B075.md) |
| ✅ | 76 | `B076` | stmt=94.45, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.28 | [case](../cases/B076.md) |
| ✅ | 77 | `B077` | stmt=94.45, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.28 | [case](../cases/B077.md) |
| ✅ | 78 | `B078` | stmt=94.45, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.28 | [case](../cases/B078.md) |
| ✅ | 79 | `B079` | stmt=94.45, branch=85.81, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.28 | [case](../cases/B079.md) |
| ✅ | 80 | `B080` | stmt=95.14, branch=86.16, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.30 | [case](../cases/B080.md) |
| ✅ | 81 | `B081` | stmt=95.14, branch=86.16, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.30 | [case](../cases/B081.md) |
| ✅ | 82 | `B082` | stmt=95.14, branch=86.16, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.40 | [case](../cases/B082.md) |
| ✅ | 83 | `B083` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.40 | [case](../cases/B083.md) |
| ✅ | 84 | `B084` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.40 | [case](../cases/B084.md) |
| ✅ | 85 | `B085` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B085.md) |
| ✅ | 86 | `B086` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B086.md) |
| ✅ | 87 | `B087` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B087.md) |
| ✅ | 88 | `B088` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B088.md) |
| ✅ | 89 | `B089` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B089.md) |
| ✅ | 90 | `B090` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B090.md) |
| ✅ | 91 | `B091` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B091.md) |
| ✅ | 92 | `B092` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B092.md) |
| ✅ | 93 | `B093` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B093.md) |
| ❓ | 94 | `B094` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B094.md) |
| ❓ | 95 | `B095` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B095.md) |
| ❓ | 96 | `B096` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B096.md) |
| ⚠️ | 97 | `B097` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B097.md) |
| ✅ | 98 | `B098` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B098.md) |
| ✅ | 99 | `B099` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B099.md) |
| ✅ | 100 | `B100` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B100.md) |
| ✅ | 101 | `B101` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B101.md) |
| ✅ | 102 | `B102` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B102.md) |
| ✅ | 103 | `B103` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B103.md) |
| ❓ | 104 | `B104` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B104.md) |
| ✅ | 105 | `B105` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B105.md) |
| ✅ | 106 | `B106` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.45 | [case](../cases/B106.md) |
| ✅ | 107 | `B107` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.51 | [case](../cases/B107.md) |
| ✅ | 108 | `B108` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.51 | [case](../cases/B108.md) |
| ✅ | 109 | `B109` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.51 | [case](../cases/B109.md) |
| ✅ | 110 | `B110` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.51 | [case](../cases/B110.md) |
| ✅ | 111 | `B111` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.51 | [case](../cases/B111.md) |
| ✅ | 112 | `B112` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.51 | [case](../cases/B112.md) |
| ✅ | 113 | `B113` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.51 | [case](../cases/B113.md) |
| ✅ | 114 | `B114` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.51 | [case](../cases/B114.md) |
| ✅ | 115 | `B115` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.59 | [case](../cases/B115.md) |
| ✅ | 116 | `B116` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.59 | [case](../cases/B116.md) |
| ✅ | 117 | `B117` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.59 | [case](../cases/B117.md) |
| ✅ | 118 | `B118` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.59 | [case](../cases/B118.md) |
| ✅ | 119 | `B119` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.59 | [case](../cases/B119.md) |
| ✅ | 120 | `B120` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.59 | [case](../cases/B120.md) |
| ✅ | 121 | `B121` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.59 | [case](../cases/B121.md) |
| ✅ | 122 | `B122` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.59 | [case](../cases/B122.md) |
| ✅ | 123 | `B123` | stmt=95.68, branch=86.51, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.70 | [case](../cases/B123.md) |
| ✅ | 124 | `B124` | stmt=95.69, branch=86.18, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.70 | [case](../cases/B124.md) |
| ✅ | 125 | `B125` | stmt=95.69, branch=86.18, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.70 | [case](../cases/B125.md) |
| ✅ | 126 | `B126` | stmt=95.69, branch=86.18, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.70 | [case](../cases/B126.md) |
| ✅ | 127 | `B127` | stmt=95.69, branch=86.18, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.70 | [case](../cases/B127.md) |
| ✅ | 128 | `B128` | stmt=95.69, branch=86.18, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.86 | [case](../cases/B128.md) |
| ✅ | 129 | `B129` | stmt=95.69, branch=86.18, cond=70.23, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=71.86 | [case](../cases/B129.md) |

---
_Back to [dashboard](../../DV_REPORT.md)_

# ⚠️ BASIC bucket

**Planned:** `129` &nbsp; **Evidenced:** `125` &nbsp; **Status:** ⚠️

## Merged code coverage (this bucket)

<!-- column legend:
  metric          = code-coverage category (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle)
  merged_pct      = bucket-local ordered-merge percentage across all evidenced cases
  target          = workflow coverage target (blank = no hard target for that category)
  status          = target check vs merged_pct
-->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 97.30 | 95.0 |
| ⚠️ | branch | 89.66 | 90.0 |
| ℹ️ | cond | 75.47 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 73.33 | 90.0 |
| ⚠️ | toggle | 70.74 | 80.0 |

## Ordered merge trace

<!-- each row is the merged coverage total after that case was added to the bucket in case-id order. -->

| status | step | case_id | merged_total (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | detail |
|:---:|---:|---|---|---|
| ✅ | 1 | `B001` | stmt=74.72, branch=40.23, cond=12.58, expr=0.00, fsm_state=12.50, fsm_trans=0.00, toggle=2.54 | [case](../cases/B001.md) |
| ✅ | 2 | `B002` | stmt=78.29, branch=42.86, cond=13.84, expr=0.00, fsm_state=12.50, fsm_trans=0.00, toggle=3.65 | [case](../cases/B002.md) |
| ✅ | 3 | `B003` | stmt=92.44, branch=76.03, cond=50.94, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=31.65 | [case](../cases/B003.md) |
| ✅ | 4 | `B004` | stmt=92.44, branch=76.03, cond=50.94, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=32.24 | [case](../cases/B004.md) |
| ✅ | 5 | `B005` | stmt=92.54, branch=76.35, cond=52.20, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=39.04 | [case](../cases/B005.md) |
| ✅ | 6 | `B006` | stmt=92.87, branch=77.34, cond=53.46, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=45.96 | [case](../cases/B006.md) |
| ✅ | 7 | `B007` | stmt=92.87, branch=77.34, cond=53.46, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=47.72 | [case](../cases/B007.md) |
| ✅ | 8 | `B008` | stmt=92.87, branch=77.34, cond=53.46, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=47.72 | [case](../cases/B008.md) |
| ✅ | 9 | `B009` | stmt=93.31, branch=80.13, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.09 | [case](../cases/B009.md) |
| ✅ | 10 | `B010` | stmt=93.60, branch=80.13, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.12 | [case](../cases/B010.md) |
| ✅ | 11 | `B011` | stmt=93.69, branch=80.62, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.32 | [case](../cases/B011.md) |
| ✅ | 12 | `B012` | stmt=93.74, branch=80.79, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.34 | [case](../cases/B012.md) |
| ✅ | 13 | `B013` | stmt=93.79, branch=80.95, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.34 | [case](../cases/B013.md) |
| ✅ | 14 | `B014` | stmt=93.79, branch=80.95, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.34 | [case](../cases/B014.md) |
| ✅ | 15 | `B015` | stmt=93.79, branch=80.95, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.34 | [case](../cases/B015.md) |
| ✅ | 16 | `B016` | stmt=93.79, branch=80.95, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=48.34 | [case](../cases/B016.md) |
| ✅ | 17 | `B017` | stmt=93.79, branch=80.95, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=49.93 | [case](../cases/B017.md) |
| ✅ | 18 | `B018` | stmt=93.79, branch=80.95, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=49.93 | [case](../cases/B018.md) |
| ✅ | 19 | `B019` | stmt=93.79, branch=80.95, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.42 | [case](../cases/B019.md) |
| ✅ | 20 | `B020` | stmt=93.79, branch=80.95, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.59 | [case](../cases/B020.md) |
| ✅ | 21 | `B021` | stmt=93.89, branch=81.12, cond=66.04, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.59 | [case](../cases/B021.md) |
| ✅ | 22 | `B022` | stmt=93.89, branch=81.28, cond=66.04, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.61 | [case](../cases/B022.md) |
| ✅ | 23 | `B023` | stmt=93.93, branch=81.44, cond=66.04, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.61 | [case](../cases/B023.md) |
| ✅ | 24 | `B024` | stmt=93.93, branch=81.44, cond=66.04, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.87 | [case](../cases/B024.md) |
| ✅ | 25 | `B025` | stmt=93.93, branch=81.44, cond=66.04, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.87 | [case](../cases/B025.md) |
| ✅ | 26 | `B026` | stmt=93.93, branch=81.44, cond=66.04, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=50.87 | [case](../cases/B026.md) |
| ✅ | 27 | `B027` | stmt=94.61, branch=82.92, cond=67.92, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.65 | [case](../cases/B027.md) |
| ✅ | 28 | `B028` | stmt=94.61, branch=83.09, cond=67.92, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.65 | [case](../cases/B028.md) |
| ✅ | 29 | `B029` | stmt=94.75, branch=83.58, cond=67.92, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=51.94 | [case](../cases/B029.md) |
| ✅ | 30 | `B030` | stmt=94.75, branch=83.58, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.21 | [case](../cases/B030.md) |
| ✅ | 31 | `B031` | stmt=94.75, branch=83.58, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.40 | [case](../cases/B031.md) |
| ✅ | 32 | `B032` | stmt=94.75, branch=83.58, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.40 | [case](../cases/B032.md) |
| ✅ | 33 | `B033` | stmt=94.85, branch=83.91, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.46 | [case](../cases/B033.md) |
| ✅ | 34 | `B034` | stmt=94.85, branch=83.91, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.46 | [case](../cases/B034.md) |
| ✅ | 35 | `B035` | stmt=94.85, branch=83.91, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.46 | [case](../cases/B035.md) |
| ✅ | 36 | `B036` | stmt=94.85, branch=83.91, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.46 | [case](../cases/B036.md) |
| ✅ | 37 | `B037` | stmt=94.85, branch=83.91, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.46 | [case](../cases/B037.md) |
| ✅ | 38 | `B038` | stmt=94.85, branch=83.91, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.46 | [case](../cases/B038.md) |
| ✅ | 39 | `B039` | stmt=94.85, branch=83.91, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.46 | [case](../cases/B039.md) |
| ✅ | 40 | `B040` | stmt=94.85, branch=83.91, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.46 | [case](../cases/B040.md) |
| ✅ | 41 | `B041` | stmt=94.85, branch=83.91, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.52 | [case](../cases/B041.md) |
| ✅ | 42 | `B042` | stmt=94.99, branch=84.24, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.59 | [case](../cases/B042.md) |
| ✅ | 43 | `B043` | stmt=95.28, branch=84.56, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.63 | [case](../cases/B043.md) |
| ✅ | 44 | `B044` | stmt=95.28, branch=84.56, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.63 | [case](../cases/B044.md) |
| ✅ | 45 | `B045` | stmt=95.28, branch=84.56, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.63 | [case](../cases/B045.md) |
| ✅ | 46 | `B046` | stmt=95.28, branch=84.56, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.63 | [case](../cases/B046.md) |
| ✅ | 47 | `B047` | stmt=95.28, branch=84.56, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.63 | [case](../cases/B047.md) |
| ✅ | 48 | `B048` | stmt=95.28, branch=84.56, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.63 | [case](../cases/B048.md) |
| ✅ | 49 | `B049` | stmt=95.33, branch=84.73, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.70 | [case](../cases/B049.md) |
| ✅ | 50 | `B050` | stmt=95.38, branch=84.89, cond=68.55, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.78 | [case](../cases/B050.md) |
| ✅ | 51 | `B051` | stmt=95.38, branch=84.89, cond=69.18, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.78 | [case](../cases/B051.md) |
| ✅ | 52 | `B052` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B052.md) |
| ✅ | 53 | `B053` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B053.md) |
| ✅ | 54 | `B054` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B054.md) |
| ✅ | 55 | `B055` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B055.md) |
| ✅ | 56 | `B056` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B056.md) |
| ✅ | 57 | `B057` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B057.md) |
| ✅ | 58 | `B058` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B058.md) |
| ✅ | 59 | `B059` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B059.md) |
| ✅ | 60 | `B060` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B060.md) |
| ✅ | 61 | `B061` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B061.md) |
| ✅ | 62 | `B062` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B062.md) |
| ✅ | 63 | `B063` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B063.md) |
| ✅ | 64 | `B064` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B064.md) |
| ✅ | 65 | `B065` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=60.93 | [case](../cases/B065.md) |
| ✅ | 66 | `B066` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=62.40 | [case](../cases/B066.md) |
| ✅ | 67 | `B067` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=62.71 | [case](../cases/B067.md) |
| ✅ | 68 | `B068` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.18 | [case](../cases/B068.md) |
| ✅ | 69 | `B069` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.18 | [case](../cases/B069.md) |
| ✅ | 70 | `B070` | stmt=95.38, branch=84.89, cond=69.81, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.18 | [case](../cases/B070.md) |
| ✅ | 71 | `B071` | stmt=96.82, branch=88.18, cond=71.70, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.16 | [case](../cases/B071.md) |
| ✅ | 72 | `B072` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B072.md) |
| ✅ | 73 | `B073` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B073.md) |
| ✅ | 74 | `B074` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B074.md) |
| ✅ | 75 | `B075` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B075.md) |
| ✅ | 76 | `B076` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B076.md) |
| ✅ | 77 | `B077` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B077.md) |
| ✅ | 78 | `B078` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B078.md) |
| ✅ | 79 | `B079` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B079.md) |
| ✅ | 80 | `B080` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B080.md) |
| ✅ | 81 | `B081` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=69.19 | [case](../cases/B081.md) |
| ✅ | 82 | `B082` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.26 | [case](../cases/B082.md) |
| ✅ | 83 | `B083` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.26 | [case](../cases/B083.md) |
| ✅ | 84 | `B084` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.26 | [case](../cases/B084.md) |
| ✅ | 85 | `B085` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B085.md) |
| ✅ | 86 | `B086` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B086.md) |
| ✅ | 87 | `B087` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B087.md) |
| ✅ | 88 | `B088` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B088.md) |
| ✅ | 89 | `B089` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B089.md) |
| ✅ | 90 | `B090` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B090.md) |
| ✅ | 91 | `B091` | stmt=97.21, branch=89.16, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B091.md) |
| ❌ | 92 | `B092` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B092.md) |
| ✅ | 93 | `B093` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B093.md) |
| ❓ | 94 | `B094` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B094.md) |
| ❓ | 95 | `B095` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B095.md) |
| ❓ | 96 | `B096` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B096.md) |
| ✅ | 97 | `B097` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B097.md) |
| ✅ | 98 | `B098` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B098.md) |
| ✅ | 99 | `B099` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B099.md) |
| ✅ | 100 | `B100` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B100.md) |
| ✅ | 101 | `B101` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B101.md) |
| ✅ | 102 | `B102` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B102.md) |
| ✅ | 103 | `B103` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B103.md) |
| ❓ | 104 | `B104` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B104.md) |
| ✅ | 105 | `B105` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B105.md) |
| ✅ | 106 | `B106` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.31 | [case](../cases/B106.md) |
| ✅ | 107 | `B107` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.37 | [case](../cases/B107.md) |
| ✅ | 108 | `B108` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.37 | [case](../cases/B108.md) |
| ✅ | 109 | `B109` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.37 | [case](../cases/B109.md) |
| ✅ | 110 | `B110` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.37 | [case](../cases/B110.md) |
| ✅ | 111 | `B111` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.37 | [case](../cases/B111.md) |
| ✅ | 112 | `B112` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.37 | [case](../cases/B112.md) |
| ✅ | 113 | `B113` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.37 | [case](../cases/B113.md) |
| ✅ | 114 | `B114` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.37 | [case](../cases/B114.md) |
| ✅ | 115 | `B115` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.51 | [case](../cases/B115.md) |
| ✅ | 116 | `B116` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.51 | [case](../cases/B116.md) |
| ✅ | 117 | `B117` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.51 | [case](../cases/B117.md) |
| ✅ | 118 | `B118` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.51 | [case](../cases/B118.md) |
| ✅ | 119 | `B119` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.51 | [case](../cases/B119.md) |
| ✅ | 120 | `B120` | stmt=97.21, branch=89.16, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.51 | [case](../cases/B120.md) |
| ✅ | 121 | `B121` | stmt=97.21, branch=89.33, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.51 | [case](../cases/B121.md) |
| ✅ | 122 | `B122` | stmt=97.21, branch=89.33, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.51 | [case](../cases/B122.md) |
| ✅ | 123 | `B123` | stmt=97.26, branch=89.49, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.58 | [case](../cases/B123.md) |
| ✅ | 124 | `B124` | stmt=97.26, branch=89.49, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=73.33, toggle=70.58 | [case](../cases/B124.md) |
| ✅ | 125 | `B125` | stmt=97.26, branch=89.49, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=73.33, toggle=70.58 | [case](../cases/B125.md) |
| ✅ | 126 | `B126` | stmt=97.26, branch=89.49, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=73.33, toggle=70.58 | [case](../cases/B126.md) |
| ✅ | 127 | `B127` | stmt=97.26, branch=89.49, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=73.33, toggle=70.58 | [case](../cases/B127.md) |
| ✅ | 128 | `B128` | stmt=97.30, branch=89.66, cond=75.47, expr=40.00, fsm_state=100.00, fsm_trans=73.33, toggle=70.74 | [case](../cases/B128.md) |
| ❌ | 129 | `B129` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/B129.md) |

---
_Back to [dashboard](../../DV_REPORT.md)_

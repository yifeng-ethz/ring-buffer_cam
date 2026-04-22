# ⚠️ ERROR bucket

**Planned:** `132` &nbsp; **Evidenced:** `128` &nbsp; **Status:** ⚠️

## Merged code coverage (this bucket)

<!-- column legend:
  metric          = code-coverage category (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle)
  merged_pct      = bucket-local ordered-merge percentage across all evidenced cases
  target          = workflow coverage target (blank = no hard target for that category)
  status          = target check vs merged_pct
-->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 96.87 | 95.0 |
| ⚠️ | branch | 89.33 | 90.0 |
| ℹ️ | cond | 74.84 | - |
| ℹ️ | expr | 40.00 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 70.60 | 80.0 |

## Ordered merge trace

<!-- each row is the merged coverage total after that case was added to the bucket in case-id order. -->

| status | step | case_id | merged_total (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | detail |
|:---:|---:|---|---|---|
| ✅ | 1 | `X001` | stmt=89.12, branch=73.56, cond=50.31, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=29.42 | [case](../cases/X001.md) |
| ✅ | 2 | `X002` | stmt=89.36, branch=74.38, cond=50.31, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=30.89 | [case](../cases/X002.md) |
| ✅ | 3 | `X003` | stmt=89.89, branch=77.34, cond=62.26, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=33.31 | [case](../cases/X003.md) |
| ✅ | 4 | `X004` | stmt=89.89, branch=77.34, cond=62.89, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=33.98 | [case](../cases/X004.md) |
| ✅ | 5 | `X005` | stmt=93.02, branch=79.15, cond=64.78, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.15 | [case](../cases/X005.md) |
| ✅ | 6 | `X006` | stmt=93.31, branch=79.47, cond=64.78, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.18 | [case](../cases/X006.md) |
| ✅ | 7 | `X007` | stmt=93.31, branch=79.47, cond=64.78, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.18 | [case](../cases/X007.md) |
| ✅ | 8 | `X008` | stmt=93.31, branch=79.47, cond=64.78, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.18 | [case](../cases/X008.md) |
| ✅ | 9 | `X009` | stmt=93.31, branch=79.47, cond=64.78, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.18 | [case](../cases/X009.md) |
| ✅ | 10 | `X010` | stmt=93.31, branch=79.47, cond=64.78, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.27 | [case](../cases/X010.md) |
| ✅ | 11 | `X011` | stmt=93.31, branch=79.47, cond=64.78, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.30 | [case](../cases/X011.md) |
| ✅ | 12 | `X012` | stmt=93.31, branch=79.47, cond=64.78, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.30 | [case](../cases/X012.md) |
| ✅ | 13 | `X013` | stmt=93.31, branch=79.47, cond=65.41, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.46 | [case](../cases/X013.md) |
| ✅ | 14 | `X014` | stmt=93.31, branch=79.47, cond=65.41, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.46 | [case](../cases/X014.md) |
| ✅ | 15 | `X015` | stmt=93.31, branch=79.47, cond=65.41, expr=30.00, fsm_state=100.00, fsm_trans=66.67, toggle=34.46 | [case](../cases/X015.md) |
| ✅ | 16 | `X016` | stmt=95.14, branch=83.91, cond=67.30, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=61.54 | [case](../cases/X016.md) |
| ✅ | 17 | `X017` | stmt=95.14, branch=83.91, cond=67.30, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=61.54 | [case](../cases/X017.md) |
| ✅ | 18 | `X018` | stmt=95.14, branch=83.91, cond=67.30, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=61.54 | [case](../cases/X018.md) |
| ✅ | 19 | `X019` | stmt=95.14, branch=83.91, cond=67.30, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=61.70 | [case](../cases/X019.md) |
| ✅ | 20 | `X020` | stmt=95.14, branch=83.91, cond=67.30, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=61.73 | [case](../cases/X020.md) |
| ✅ | 21 | `X021` | stmt=95.14, branch=83.91, cond=67.30, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=61.73 | [case](../cases/X021.md) |
| ✅ | 22 | `X022` | stmt=95.19, branch=84.07, cond=67.30, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=61.73 | [case](../cases/X022.md) |
| ✅ | 23 | `X023` | stmt=95.19, branch=84.07, cond=67.92, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=61.74 | [case](../cases/X023.md) |
| ✅ | 24 | `X024` | stmt=95.19, branch=84.24, cond=67.92, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=62.85 | [case](../cases/X024.md) |
| ✅ | 25 | `X025` | stmt=95.19, branch=84.24, cond=67.92, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=63.06 | [case](../cases/X025.md) |
| ✅ | 26 | `X026` | stmt=95.19, branch=84.24, cond=67.92, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.07 | [case](../cases/X026.md) |
| ✅ | 27 | `X027` | stmt=95.19, branch=84.24, cond=67.92, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.44 | [case](../cases/X027.md) |
| ✅ | 28 | `X028` | stmt=95.19, branch=84.24, cond=67.92, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.44 | [case](../cases/X028.md) |
| ✅ | 29 | `X029` | stmt=95.19, branch=84.24, cond=67.92, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.44 | [case](../cases/X029.md) |
| ✅ | 30 | `X030` | stmt=95.19, branch=84.24, cond=67.92, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.69 | [case](../cases/X030.md) |
| ✅ | 31 | `X031` | stmt=95.19, branch=84.24, cond=68.55, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.70 | [case](../cases/X031.md) |
| ✅ | 32 | `X032` | stmt=95.19, branch=84.24, cond=68.55, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.79 | [case](../cases/X032.md) |
| ✅ | 33 | `X033` | stmt=95.19, branch=84.24, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.79 | [case](../cases/X033.md) |
| ✅ | 34 | `X034` | stmt=95.19, branch=84.24, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.79 | [case](../cases/X034.md) |
| ✅ | 35 | `X035` | stmt=95.19, branch=84.24, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.85 | [case](../cases/X035.md) |
| ✅ | 36 | `X036` | stmt=95.19, branch=84.24, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=64.91 | [case](../cases/X036.md) |
| ✅ | 37 | `X037` | stmt=95.23, branch=84.40, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=65.21 | [case](../cases/X037.md) |
| ✅ | 38 | `X038` | stmt=95.23, branch=84.40, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=65.21 | [case](../cases/X038.md) |
| ✅ | 39 | `X039` | stmt=95.23, branch=84.40, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=65.24 | [case](../cases/X039.md) |
| ✅ | 40 | `X040` | stmt=95.23, branch=84.40, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=65.24 | [case](../cases/X040.md) |
| ✅ | 41 | `X041` | stmt=95.23, branch=84.40, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=65.24 | [case](../cases/X041.md) |
| ✅ | 42 | `X042` | stmt=95.23, branch=84.40, cond=69.81, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=65.24 | [case](../cases/X042.md) |
| ✅ | 43 | `X043` | stmt=95.62, branch=85.39, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.00 | [case](../cases/X043.md) |
| ✅ | 44 | `X044` | stmt=95.62, branch=85.39, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.00 | [case](../cases/X044.md) |
| ✅ | 45 | `X045` | stmt=95.62, branch=85.39, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.15 | [case](../cases/X045.md) |
| ✅ | 46 | `X046` | stmt=95.62, branch=85.39, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.15 | [case](../cases/X046.md) |
| ✅ | 47 | `X047` | stmt=95.62, branch=85.55, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.15 | [case](../cases/X047.md) |
| ✅ | 48 | `X048` | stmt=95.62, branch=85.55, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.15 | [case](../cases/X048.md) |
| ✅ | 49 | `X049` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X049.md) |
| ✅ | 50 | `X050` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X050.md) |
| ✅ | 51 | `X051` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X051.md) |
| ✅ | 52 | `X052` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X052.md) |
| ✅ | 53 | `X053` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X053.md) |
| ✅ | 54 | `X054` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X054.md) |
| ✅ | 55 | `X055` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X055.md) |
| ✅ | 56 | `X056` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X056.md) |
| ✅ | 57 | `X057` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X057.md) |
| ✅ | 58 | `X058` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X058.md) |
| ✅ | 59 | `X059` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X059.md) |
| ✅ | 60 | `X060` | stmt=95.71, branch=85.88, cond=71.07, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X060.md) |
| ✅ | 61 | `X061` | stmt=95.71, branch=85.88, cond=71.70, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X061.md) |
| ✅ | 62 | `X062` | stmt=95.71, branch=85.88, cond=71.70, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X062.md) |
| ✅ | 63 | `X063` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.21 | [case](../cases/X063.md) |
| ✅ | 64 | `X064` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X064.md) |
| ✅ | 65 | `X065` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X065.md) |
| ✅ | 66 | `X066` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X066.md) |
| ✅ | 67 | `X067` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X067.md) |
| ✅ | 68 | `X068` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X068.md) |
| ✅ | 69 | `X069` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X069.md) |
| ✅ | 70 | `X070` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X070.md) |
| ✅ | 71 | `X071` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X071.md) |
| ✅ | 72 | `X072` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X072.md) |
| ✅ | 73 | `X073` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X073.md) |
| ✅ | 74 | `X074` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X074.md) |
| ✅ | 75 | `X075` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X075.md) |
| ✅ | 76 | `X076` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X076.md) |
| ✅ | 77 | `X077` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.23 | [case](../cases/X077.md) |
| ✅ | 78 | `X078` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.24 | [case](../cases/X078.md) |
| ✅ | 79 | `X079` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.24 | [case](../cases/X079.md) |
| ✅ | 80 | `X080` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.24 | [case](../cases/X080.md) |
| ✅ | 81 | `X081` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.24 | [case](../cases/X081.md) |
| ✅ | 82 | `X082` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.24 | [case](../cases/X082.md) |
| ✅ | 83 | `X083` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.24 | [case](../cases/X083.md) |
| ✅ | 84 | `X084` | stmt=96.34, branch=87.36, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.24 | [case](../cases/X084.md) |
| ✅ | 85 | `X085` | stmt=96.39, branch=87.52, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.61 | [case](../cases/X085.md) |
| ✅ | 86 | `X086` | stmt=96.39, branch=87.52, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.61 | [case](../cases/X086.md) |
| ✅ | 87 | `X087` | stmt=96.39, branch=87.52, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.61 | [case](../cases/X087.md) |
| ✅ | 88 | `X088` | stmt=96.39, branch=87.52, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.92 | [case](../cases/X088.md) |
| ✅ | 89 | `X089` | stmt=96.39, branch=87.68, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=67.92 | [case](../cases/X089.md) |
| ✅ | 90 | `X090` | stmt=96.82, branch=88.34, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.27 | [case](../cases/X090.md) |
| ✅ | 91 | `X091` | stmt=96.82, branch=88.51, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.27 | [case](../cases/X091.md) |
| ✅ | 92 | `X092` | stmt=96.82, branch=88.51, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.27 | [case](../cases/X092.md) |
| ✅ | 93 | `X093` | stmt=96.82, branch=88.67, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.27 | [case](../cases/X093.md) |
| ✅ | 94 | `X094` | stmt=96.82, branch=88.83, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.27 | [case](../cases/X094.md) |
| ✅ | 95 | `X095` | stmt=96.82, branch=89.00, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.27 | [case](../cases/X095.md) |
| ✅ | 96 | `X096` | stmt=96.82, branch=89.16, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.27 | [case](../cases/X096.md) |
| ✅ | 97 | `X097` | stmt=96.82, branch=89.16, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.45 | [case](../cases/X097.md) |
| ✅ | 98 | `X098` | stmt=96.82, branch=89.16, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.45 | [case](../cases/X098.md) |
| ✅ | 99 | `X099` | stmt=96.82, branch=89.16, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.45 | [case](../cases/X099.md) |
| ✅ | 100 | `X100` | stmt=96.87, branch=89.33, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X100.md) |
| ✅ | 101 | `X101` | stmt=96.87, branch=89.33, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X101.md) |
| ✅ | 102 | `X102` | stmt=96.87, branch=89.33, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X102.md) |
| ❓ | 103 | `X103` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/X103.md) |
| ✅ | 104 | `X104` | stmt=96.87, branch=89.33, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X104.md) |
| ✅ | 105 | `X105` | stmt=96.87, branch=89.33, cond=74.21, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X105.md) |
| ✅ | 106 | `X106` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X106.md) |
| ❓ | 107 | `X107` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/X107.md) |
| ✅ | 108 | `X108` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X108.md) |
| ✅ | 109 | `X109` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X109.md) |
| ❓ | 110 | `X110` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/X110.md) |
| ✅ | 111 | `X111` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X111.md) |
| ✅ | 112 | `X112` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X112.md) |
| ✅ | 113 | `X113` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X113.md) |
| ✅ | 114 | `X114` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X114.md) |
| ❓ | 115 | `X115` | stmt=n/a, branch=n/a, cond=n/a, expr=n/a, fsm_state=n/a, fsm_trans=n/a, toggle=n/a | [case](../cases/X115.md) |
| ✅ | 116 | `X116` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X116.md) |
| ✅ | 117 | `X117` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.47 | [case](../cases/X117.md) |
| ✅ | 118 | `X118` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=68.52 | [case](../cases/X118.md) |
| ✅ | 119 | `X119` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.15 | [case](../cases/X119.md) |
| ✅ | 120 | `X120` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.34 | [case](../cases/X120.md) |
| ✅ | 121 | `X121` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.34 | [case](../cases/X121.md) |
| ✅ | 122 | `X122` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.54 | [case](../cases/X122.md) |
| ✅ | 123 | `X123` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.54 | [case](../cases/X123.md) |
| ✅ | 124 | `X124` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.54 | [case](../cases/X124.md) |
| ✅ | 125 | `X125` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.54 | [case](../cases/X125.md) |
| ✅ | 126 | `X126` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.54 | [case](../cases/X126.md) |
| ✅ | 127 | `X127` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.54 | [case](../cases/X127.md) |
| ✅ | 128 | `X128` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.58 | [case](../cases/X128.md) |
| ✅ | 129 | `X129` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.60 | [case](../cases/X129.md) |
| ✅ | 130 | `X130` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.60 | [case](../cases/X130.md) |
| ✅ | 131 | `X131` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.60 | [case](../cases/X131.md) |
| ✅ | 132 | `X132` | stmt=96.87, branch=89.33, cond=74.84, expr=40.00, fsm_state=100.00, fsm_trans=66.67, toggle=70.60 | [case](../cases/X132.md) |

---
_Back to [dashboard](../../DV_REPORT.md)_

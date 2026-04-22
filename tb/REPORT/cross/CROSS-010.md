# ✅ CROSS-010

**Kind:** `anchored_hybrid` &nbsp; **Build:** `default_p2_pipe4` &nbsp; **Bucket:** `-` &nbsp; **Sequence:** `overwrite-pressure GOOD(pool=1, λ=1.0, 10k) → X119 anchor → recovery GOOD(random, 10k)`

## Summary

<!-- field legend:
  case_count              = number of plan cases composed into this run
  effort                  = practical (capped per case) or extensive (full planned stress)
  iter_cap, payload_cap   = practical-mode budget caps
  txns                    = total transactions driven through the DUT in this run
  functional_cross_pct    = functional coverage against DV_CROSS.md (percent)
  queued_overlap          = transactions enqueued before the previous drained
  counter_checks_failed   = scoreboard counter mismatches observed (0 is required for pass)
  unexpected_outputs      = outputs the scoreboard did not predict
-->

| status | field | value |
|:---:|---|---|
| ℹ️ | case_count | `1` |
| ℹ️ | effort | `smoke` |
| ℹ️ | iter_cap | `n/a` |
| ℹ️ | payload_cap | `n/a` |
| ℹ️ | txns | `784` |
| ✅ | functional_cross_pct | `80.9` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 92.44 |
| branch | 83.00 |
| cond | 62.91 |
| expr | 40.00 |
| fsm_state | 100.00 |
| fsm_trans | 66.67 |
| toggle | 57.86 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

❓ no curve data available for this run.

---
_Back to [dashboard](../../DV_REPORT.md)_

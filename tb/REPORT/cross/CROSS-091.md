# ✅ CROSS-091

**Kind:** `seed_sweep` &nbsp; **Build:** `default_p2_pipe4` &nbsp; **Bucket:** `-` &nbsp; **Sequence:** `X101-X102 promoted to a sustained bad-hit burst bracketed by 64-hit GOOD warmup/cooldown windows`

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
| ℹ️ | txns | `128` |
| ✅ | functional_cross_pct | `86.2` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 89.67 |
| branch | 76.49 |
| cond | 59.48 |
| expr | 30.00 |
| fsm_state | 100.00 |
| fsm_trans | 66.67 |
| toggle | 39.81 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

❓ no curve data available for this run.

---
_Back to [dashboard](../../DV_REPORT.md)_

# ❌ CROSS-128

**Kind:** `checkpoint_soak` &nbsp; **Build:** `p4_n4_pipe4` &nbsp; **Bucket:** `-` &nbsp; **Sequence:** `30 s simulator-time SV p4/n4 PROF-heavy soak: P110/P125/P126/P127/P128/P129/P130/P131/P132/P134/P135/P137 replayed with random inter-case gaps`

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
| ℹ️ | case_count | `0` |
| ℹ️ | effort | `missing_required_30s_sim_soak` |
| ℹ️ | iter_cap | `target_ps=30000000000000` |
| ℹ️ | payload_cap | `PROF-heavy overwrite 30 s simulator-time soak` |
| ℹ️ | txns | `0` |
| ❌ | functional_cross_pct | `0.0` |
| ℹ️ | queued_overlap | `0` |
| ❌ | counter_checks_failed | `1` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | n/a |
| branch | n/a |
| cond | n/a |
| expr | n/a |
| fsm_state | n/a |
| fsm_trans | n/a |
| toggle | n/a |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

❓ no curve data available for this run.

---
_Back to [dashboard](../../DV_REPORT.md)_

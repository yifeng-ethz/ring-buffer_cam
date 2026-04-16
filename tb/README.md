# ring_buffer_cam TB

Active DV workflow root for the live `ring_buffer_cam` IP.

## Structure

- `sim/`: retained VHDL directed benches and legacy smoke runners.
- `uvm/`: active mixed-language UVM harness and local work area.
- `scripts/generate_dv_report.py`: repo-local source-of-truth generator for `DV_REPORT.json` plus the `REPORT/` tree.
- `DV_PLAN.md`: workflow entry point and signoff rules.
- `DV_HARNESS.md`: harness contract, current bench inventory, and required upgrades.
- `DV_BASIC.md`, `DV_EDGE.md`, `DV_PROF.md`, `DV_ERROR.md`, `DV_CROSS.md`: frozen case buckets and continuous-frame intent.
- `DV_REPORT.md`, `DV_COV.md`, `DV_REPORT.json`, `REPORT/`: generated dashboard and per-case review tree.

## Quick Start

1. Compile the current UVM bench:
   - `bash ring-buffer_cam/tb/uvm/run_uvm.sh compile`
2. Run a single promoted UVM case:
   - `bash ring-buffer_cam/tb/uvm/run_uvm.sh test_single_push_pop 1`
3. Regenerate the workflow report tree:
   - `python3 ring-buffer_cam/tb/scripts/generate_dv_report.py`
4. Review the chief-architect dashboard:
   - [DV_REPORT.md](/home/yifeng/packages/mu3e_ip_dev/mu3e-ip-cores/ring-buffer_cam/tb/DV_REPORT.md:1)

## Current Status

- The active workflow is the split `DV_*` tree under `tb/`; the old monolithic narrative in [SIGNOFF.md](/home/yifeng/packages/mu3e_ip_dev/mu3e-ip-cores/ring-buffer_cam/SIGNOFF.md:1) is historical evidence only.
- Live post-fix reruns on 2026-04-16 show that the CSR-map UVM cases are now aligned with RTL, several directed/random smoke cases pass, and several multi-key / overwrite / long-run cases still fail on undrained-reference scoreboard checks.
- Continuous-frame `bucket_frame`, `all_buckets_frame`, checkpoint-UCDB growth curves, and real code-coverage UCDB capture are not closed yet. The generated dashboard surfaces those gaps explicitly instead of treating the legacy signoff prose as current closure.

# ring_buffer_cam TB

Directed simulation collateral for the live `ring_buffer_cam` IP in this repo.

## Layout
- `common/`: shared helpers for local benches
- `sim/`: standalone directed smoke benches and run scripts

## Quick run
- `bash tb/sim/run_questa_pipeline_smoke.sh`

## Current coverage
- `ring_buffer_cam_pipeline_smoke_tb.vhd` targets the March 2026 pop-path timing refactor in `ring_buffer_cam.vhd`.
- The smoke case pushes a same-key burst across two CAM banks and checks that the reported subheader count and final drain behavior stay functionally correct through the pipeline change.

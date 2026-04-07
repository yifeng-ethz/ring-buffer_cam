#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_name="ring_buffer_cam_syn"
revisions=(
  "ring_buffer_cam_syn_v23"
  "ring_buffer_cam_syn_p2"
  "ring_buffer_cam_syn_p3"
  "ring_buffer_cam_syn_p4"
  "addr_enc_logic_syn_p2"
  "addr_enc_logic_syn_p3"
  "addr_enc_logic_syn_p4"
)

cd "${script_dir}"

for revision in "${revisions[@]}"; do
  echo "==> compiling ${revision}"
  quartus_sh --flow compile "${project_name}" -c "${revision}"
  echo "==> reports: ${script_dir}/output_files/${revision}"
done

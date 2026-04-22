#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_name="ring_buffer_cam_syn"
revisions=(${SIGNOFF_REVISIONS:-ring_buffer_cam_syn_p4})

cd "${script_dir}"

echo "==> signoff revisions: ${revisions[*]}"
for revision in "${revisions[@]}"; do
  echo "==> compiling ${revision}"
  quartus_sh --flow compile "${project_name}" -c "${revision}"
  echo "==> reports: ${script_dir}/output_files/${revision}"
done

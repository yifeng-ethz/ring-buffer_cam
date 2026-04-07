#!/usr/bin/env bash
# Convenience wrapper for the UVM Makefile.
# Usage:
#   bash tb/uvm/run_uvm.sh                             # single directed test
#   bash tb/uvm/run_uvm.sh test_random_push_pop 42     # specific test+seed
#   bash tb/uvm/run_uvm.sh regress 16                  # 128-case regression
#   bash tb/uvm/run_uvm.sh regress 80                  # 512-case regression
#   bash tb/uvm/run_uvm.sh cfg-matrix                  # P1/P2/P3/P4 config-space sweep
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mentor_questa_home="/data1/intelFPGA_pro/23.1/questa_fse"
default_local_lic="${mentor_questa_home}/LR-287689_License.dat"
default_chain="${default_local_lic}:8161@lic-mentor.ethz.ch"
legacy_bad_lic="/data1/intelFPGA/LR-121070_License.dat"

if [[ -z "${LM_LICENSE_FILE:-}" || "${LM_LICENSE_FILE}" == *"${legacy_bad_lic}"* ]]; then
  export LM_LICENSE_FILE="${default_chain}"
fi
export MGLS_LICENSE_FILE="${LM_LICENSE_FILE}"

cmd="${1:-run}"
arg="${2:-}"

case "${cmd}" in
  regress)
    seeds="${arg:-16}"
    make -C "${script_dir}" regress SEEDS="${seeds}"
    ;;
  cfg-matrix|cfg_matrix)
    make -C "${script_dir}" cfg_matrix
    ;;
  compile)
    make -C "${script_dir}" compile
    ;;
  clean)
    make -C "${script_dir}" clean
    ;;
  *)
    test="${cmd}"
    seed="${arg:-1}"
    make -C "${script_dir}" run TEST="${test}" SEED="${seed}"
    ;;
esac

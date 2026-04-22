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

mentor_server="8161@lic-mentor.ethz.ch"
: "${QUESTA_HOME:=/data1/questaone_sim/questasim}"

if [[ "${QUESTA_HOME}" != "/data1/questaone_sim/questasim" ]]; then
  echo "ERROR: QUESTA_HOME must use /data1/questaone_sim/questasim, not ${QUESTA_HOME}" >&2
  exit 2
fi

export QUESTA_HOME
export SALT_LICENSE_SERVER="${mentor_server}"
export LM_LICENSE_FILE="${mentor_server}"
export MGLS_LICENSE_FILE="${mentor_server}"
export QSIM_INI="${QUESTA_HOME}/modelsim.ini"

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

#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ip_dir="$(cd "${script_dir}/../.." && pwd)"
work_dir="${TB_WORK_DIR:-${script_dir}/work_terminate_marker}"

mentor_server="8161@lic-mentor.ethz.ch"
: "${QUESTA_HOME:=/data1/questaone_sim/questasim}"
export QUESTA_HOME
export SALT_LICENSE_SERVER="${mentor_server}"
export LM_LICENSE_FILE="${mentor_server}"
export MGLS_LICENSE_FILE="${mentor_server}"
export QSIM_INI="${QUESTA_HOME}/modelsim.ini"

sim_bin_dir="${QUESTA_HOME}/bin"
if [ ! -x "${sim_bin_dir}/vsim" ]; then
  sim_bin_dir="${QUESTA_HOME}/linux_x86_64"
fi

vlib_cmd="${sim_bin_dir}/vlib"
vmap_cmd="${sim_bin_dir}/vmap"
vcom_cmd="${sim_bin_dir}/vcom"
vlog_cmd="${sim_bin_dir}/vlog"
vsim_cmd="${sim_bin_dir}/vsim"

if [ ! -x "${vsim_cmd}" ]; then
  echo "ERROR: vsim not found under '${QUESTA_HOME}'" >&2
  exit 2
fi

legacy_sim_dir="/data1/intelFPGA/18.1/quartus/eda/sim_lib"
compat_dir="${script_dir}/compat"

rm -rf -- "${work_dir}"
mkdir -p -- "${work_dir}"
cd -- "${work_dir}"

"${vlib_cmd}" work
"${vlib_cmd}" altera
"${vlib_cmd}" altera_mf
"${vmap_cmd}" -c >/dev/null
modelsim_ini="${work_dir}/modelsim.ini"
cp "${QSIM_INI}" "${modelsim_ini}"
chmod u+w "${modelsim_ini}"
"${vmap_cmd}" -modelsimini "${modelsim_ini}" work work >/dev/null
"${vmap_cmd}" -modelsimini "${modelsim_ini}" altera altera >/dev/null
"${vmap_cmd}" -modelsimini "${modelsim_ini}" altera_mf altera_mf >/dev/null

"${vcom_cmd}" -2008 -modelsimini "${modelsim_ini}" -work altera -quiet "${legacy_sim_dir}/altera_standard_functions.vhd"
"${vcom_cmd}" -2008 -modelsimini "${modelsim_ini}" -work altera_mf -quiet "${compat_dir}/altera_mf_components.vhd"
"${vcom_cmd}" -2008 -modelsimini "${modelsim_ini}" -work altera_mf -quiet "${compat_dir}/scfifo.vhd"

vhdl_files=(
  "${ip_dir}/rtl/cam_helper_pkg.vhd"
  "${compat_dir}/cam_mem_blk_a5.vhd"
  "${ip_dir}/rtl/cam_mem_a5.vhd"
  "${ip_dir}/rtl/alt_simple_dpram.vhd"
  "${ip_dir}/rtl/alt_fifo/cmd_fifo/cmd_fifo.vhd"
  "${ip_dir}/rtl/alt_fifo/scfifo_w40d256.vhd"
  "${ip_dir}/tb/common/ring_buffer_cam_tb_pkg.vhd"
  "${ip_dir}/rtl/addr_enc_logic_small.vhd"
  "${ip_dir}/rtl/addr_enc_logic_partitioned.vhd"
  "${ip_dir}/rtl/ring_buffer_cam_v2_core.vhd"
  "${ip_dir}/rtl/ring_buffer_cam.vhd"
  "${ip_dir}/tb/sim/ring_buffer_cam_terminate_marker_tb.vhd"
)

for vhdl_file in "${vhdl_files[@]}"; do
  "${vcom_cmd}" -2008 -modelsimini "${modelsim_ini}" -work work -quiet "${vhdl_file}"
done

"${vlog_cmd}" -modelsimini "${modelsim_ini}" -work work -quiet "${ip_dir}/rtl/b2o_encoder.v"

log_file="${work_dir}/terminate_marker.log"
"${vsim_cmd}" -modelsimini "${modelsim_ini}" -c -quiet "work.ring_buffer_cam_terminate_marker_tb" -do "run -all; quit -f" | tee "${log_file}"

if rg -n "\*\* Fatal:|\*\* Error:|^Fatal:" "${log_file}" >/dev/null; then
  echo "FAIL: ring_buffer_cam_terminate_marker_tb" >&2
  exit 1
fi
if ! rg -q "RING_BUFFER_CAM_TERMINATE_MARKER_PASS" "${log_file}"; then
  echo "FAIL: ring_buffer_cam_terminate_marker_tb (pass token missing)" >&2
  exit 1
fi

echo "PASS: ring_buffer_cam_terminate_marker_tb"

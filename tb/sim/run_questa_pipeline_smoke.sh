#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ip_dir="$(cd "${script_dir}/../.." && pwd)"
work_dir="${TB_WORK_DIR:-${script_dir}/work_pipeline_smoke}"

mentor_sim_bin_dir="/data1/intelFPGA_pro/23.1/questa_fse/bin"
ase_sim_root="/data1/intelFPGA/18.1/modelsim_ase"
ase_sim_bin_dir="${ase_sim_root}/linuxaloem"
if [ ! -x "${ase_sim_bin_dir}/vsim" ]; then
  ase_sim_bin_dir="${ase_sim_root}/bin"
fi

sim_flavor="${TB_SIM_FLAVOR:-auto}"
sim_bin_dir=""
case "${sim_flavor}" in
  mentor) sim_bin_dir="${mentor_sim_bin_dir}" ;;
  ase)    sim_bin_dir="${ase_sim_bin_dir}" ;;
  auto)
    if [ -x "${ase_sim_bin_dir}/vsim" ]; then
      sim_bin_dir="${ase_sim_bin_dir}"
    elif [ -x "${mentor_sim_bin_dir}/vsim" ]; then
      sim_bin_dir="${mentor_sim_bin_dir}"
    else
      sim_bin_dir="$(dirname "$(command -v vsim)")"
    fi
    ;;
  *)
    echo "ERROR: unknown TB_SIM_FLAVOR='${sim_flavor}' (use: auto|mentor|ase)" >&2
    exit 2
    ;;
esac

vlib_cmd="${sim_bin_dir}/vlib"
vmap_cmd="${sim_bin_dir}/vmap"
vcom_cmd="${sim_bin_dir}/vcom"
vlog_cmd="${sim_bin_dir}/vlog"
vsim_cmd="${sim_bin_dir}/vsim"

if [ ! -x "${vsim_cmd}" ]; then
  echo "ERROR: vsim not found under '${sim_bin_dir}'" >&2
  exit 2
fi

mentor_questa_home="/data1/intelFPGA_pro/23.1/questa_fse"
default_local_lic="${mentor_questa_home}/LR-287689_License.dat"
default_chain="${default_local_lic}:8161@lic-mentor.ethz.ch"
legacy_bad_lic="/data1/intelFPGA/LR-121070_License.dat"

if [ -z "${LM_LICENSE_FILE:-}" ] || [[ "${LM_LICENSE_FILE}" == *"${legacy_bad_lic}"* ]]; then
  export LM_LICENSE_FILE="${default_chain}"
fi
export MGLS_LICENSE_FILE="${LM_LICENSE_FILE}"

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
"${vmap_cmd}" -modelsimini "${modelsim_ini}" work work >/dev/null
"${vmap_cmd}" -modelsimini "${modelsim_ini}" altera altera >/dev/null
"${vmap_cmd}" -modelsimini "${modelsim_ini}" altera_mf altera_mf >/dev/null

"${vcom_cmd}" -2008 -modelsimini "${modelsim_ini}" -work altera -quiet "${legacy_sim_dir}/altera_standard_functions.vhd"
"${vcom_cmd}" -2008 -modelsimini "${modelsim_ini}" -work altera_mf -quiet "${compat_dir}/altera_mf_components.vhd"
"${vcom_cmd}" -2008 -modelsimini "${modelsim_ini}" -work altera_mf -quiet "${compat_dir}/scfifo.vhd"

vhdl_files=(
  "${compat_dir}/cam_mem_blk_a5.vhd"
  "${ip_dir}/cam_mem_a5.vhd"
  "${ip_dir}/alt_simple_dpram.vhd"
  "${ip_dir}/alt_fifo/cmd_fifo/cmd_fifo.vhd"
  "${ip_dir}/alt_fifo/scfifo_w40d256.vhd"
  "${ip_dir}/tb/common/ring_buffer_cam_tb_pkg.vhd"
  "${ip_dir}/addr_enc_logic_small.vhd"
  "${ip_dir}/addr_enc_logic_partitioned.vhd"
  "${ip_dir}/ring_buffer_cam.vhd"
  "${ip_dir}/tb/sim/ring_buffer_cam_pipeline_smoke_tb.vhd"
)

for vhdl_file in "${vhdl_files[@]}"; do
  "${vcom_cmd}" -2008 -modelsimini "${modelsim_ini}" -work work -quiet "${vhdl_file}"
done

"${vlog_cmd}" -modelsimini "${modelsim_ini}" -work work -quiet "${ip_dir}/b2o_encoder.v"

log_file="${work_dir}/pipeline_smoke.log"
"${vsim_cmd}" -modelsimini "${modelsim_ini}" -c -quiet "work.ring_buffer_cam_pipeline_smoke_tb" -do "run -all; quit -f" | tee "${log_file}"

if rg -n "\*\* Fatal:|\*\* Error:|^Fatal:" "${log_file}" >/dev/null; then
  echo "FAIL: ring_buffer_cam_pipeline_smoke_tb" >&2
  exit 1
fi
if ! rg -q "RING_BUFFER_CAM_PIPELINE_SMOKE_PASS" "${log_file}"; then
  echo "FAIL: ring_buffer_cam_pipeline_smoke_tb (pass token missing)" >&2
  exit 1
fi

echo "PASS: ring_buffer_cam_pipeline_smoke_tb"

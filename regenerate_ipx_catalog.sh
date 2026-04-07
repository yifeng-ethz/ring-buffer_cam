#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage: regenerate_ipx_catalog.sh [--root <dir>] [--relative-var <name>] [--output <file>]...

Regenerate the root Platform Designer catalog for this repository.

Defaults:
  --root         directory containing this script
  --relative-var <unset> (emit absolute file paths)
  --output       components.ipx
  --output       mu3e_ip_cores.ipx

If --relative-var is supplied, Quartus must see that variable exported and
pointing at --root when it consumes the generated catalog.
EOF
}

script_dir=$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd
)
root_dir="${script_dir}"
relative_var=""
declare -a outputs=(
    "components.ipx"
    "mu3e_ip_cores.ipx"
)

while (($# > 0)); do
    case "$1" in
        --root)
            root_dir=$2
            shift 2
            ;;
        --relative-var)
            relative_var=$2
            shift 2
            ;;
        --output)
            if ((${#outputs[@]} == 2)) && [[ ${outputs[0]} == "components.ipx" ]] && [[ ${outputs[1]} == "mu3e_ip_cores.ipx" ]]; then
                outputs=()
            fi
            outputs+=("$2")
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

root_dir=$(
    cd -- "${root_dir}" && pwd
)

if ! command -v ip-make-ipx >/dev/null 2>&1; then
    echo "ERROR: ip-make-ipx is not in PATH" >&2
    exit 1
fi

tmp_dir=$(mktemp -d)
cleanup() {
    rm -rf -- "${tmp_dir}"
}
trap cleanup EXIT

tmp_ipx="${tmp_dir}/components.ipx"
norm_ipx="${tmp_dir}/components.norm.ipx"

declare -a ip_make_args=(
    "--source-directory=${root_dir}"
    "--output=${tmp_ipx}"
    --thorough-descent
)

if [[ -n "${relative_var}" ]]; then
    export "${relative_var}=${root_dir}"
    ip_make_args+=("--relative-vars=${relative_var}")
fi

ip-make-ipx "${ip_make_args[@]}"

if [[ ! -s "${tmp_ipx}" ]]; then
    echo "ERROR: generated catalog is empty: ${tmp_ipx}" >&2
    exit 1
fi

python3 "${script_dir}/normalize_ipx_catalog.py" \
    --input "${tmp_ipx}" \
    --output "${norm_ipx}"

component_count=$(grep -c '^[[:space:]]*<component' "${norm_ipx}")
if ((component_count == 0)); then
    echo "ERROR: generated catalog contains no components: ${norm_ipx}" >&2
    exit 1
fi

for output_name in "${outputs[@]}"; do
    install -m 0644 "${norm_ipx}" "${root_dir}/${output_name}"
    echo "Wrote ${root_dir}/${output_name}"
done

echo "Catalog root : ${root_dir}"
echo "Component cnt: ${component_count}"
if [[ -n "${relative_var}" ]]; then
    echo "Relative var : ${relative_var}=${root_dir}"
else
    echo "Path mode    : absolute"
fi

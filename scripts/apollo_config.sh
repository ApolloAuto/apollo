#! /usr/bin/env bash

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

# STAGE="${STAGE:-dev}"
: ${STAGE:=dev}

IFS='' read -r -d '' STARTUP_TXT << EOF
startup --output_user_root="${APOLLO_CACHE_DIR}/bazel"
common --distdir="${APOLLO_CACHE_DIR}/distdir"
EOF

set -e

function config_noninteractive() {
    local bzl_cfg_file="${APOLLO_ROOT_DIR}/.apollo.bazelrc"
    echo "${STARTUP_TXT}" > "${bzl_cfg_file}"
    if [ "${USE_GPU}" -eq 1 ]; then
        echo "build --config=gpu" >> "${bzl_cfg_file}"
    else
        echo "build --config=cpu" >> "${bzl_cfg_file}"
    fi
    cat "${APOLLO_ROOT_DIR}/tools/apollo.bazelrc.sample" >> "${bzl_cfg_file}"
}

function config_interactive() {
    if [ -z "$PYTHON_BIN_PATH" ]; then
        PYTHON_BIN_PATH=$(which python3 || true)
    fi
    local bzl_cfg_file="${APOLLO_ROOT_DIR}/.apollo.bazelrc"

    # Set all env variables
    "$PYTHON_BIN_PATH" "${TOP_DIR}/tools/bootstrap.py" "$@"
    echo "${STARTUP_TXT}" >> "${bzl_cfg_file}"
}

function config() {
    local stage="${STAGE}"
    if [ $# -eq 0 ]; then
        config_interactive
    else
        local mode="$1" ; shift
        if [[ "${mode}" == "--noninteractive" ]]; then
            config_noninteractive "$@"
        else
            config_interactive
        fi
    fi
}

function main() {
    config "$@"
}

main "$@"

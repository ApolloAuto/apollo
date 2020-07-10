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

BAZEL_CONF="${TOP_DIR}/.apollo.bazelrc"

function config_noninteractive() {
    echo "${STARTUP_TXT}" > "${BAZEL_CONF}"
    determine_gpu_use
    if [ "${USE_GPU}" -eq 1 ]; then
        echo "build --config=gpu" >> "${BAZEL_CONF}"
    else
        echo "build --config=cpu" >> "${BAZEL_CONF}"
    fi
    cat "${TOP_DIR}/tools/apollo.bazelrc.sample" >> "${BAZEL_CONF}"
}

function config_interactive() {
    if [ -z "$PYTHON_BIN_PATH" ]; then
        PYTHON_BIN_PATH=$(which python3 || true)
    fi

    # Set all env variables
    "$PYTHON_BIN_PATH" "${TOP_DIR}/tools/bootstrap.py" "$@"
    echo "${STARTUP_TXT}" >> "${BAZEL_CONF}"
}

function config() {
    local stage="${STAGE}"
    if [ $# -eq 0 ]; then
        config_noninteractive
    else
        local mode="$1" ; shift
        if [ "${mode}" == "--clean" ]; then
            rm -f "${BAZEL_CONF}"
        elif [[ "${mode}" == "--interactive" || "${mode}" == "-i" ]]; then
            config_interactive "$@"
        else
            config_noninteractive
        fi
    fi
}

function main() {
    config "$@"
}

main "$@"

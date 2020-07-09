#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"
source "${TOP_DIR}/scripts/apollo_base.sh"
ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}

APOLLO_BUILD_SH="${APOLLO_ROOT_DIR}/scripts/apollo_build.sh"
APOLLO_TEST_SH="${APOLLO_ROOT_DIR}/scripts/apollo_test.sh"

function run_ci_build() {
    info "Running CI Build ..."
    env USE_ESD_CAN=${USE_ESD_CAN} bash "${APOLLO_BUILD_SH}"
}

function run_ci_test() {
    info "Running CI Test ..."
    env USE_ESD_CAN=${USE_ESD_CAN} bash "${APOLLO_TEST_SH}" --config=unit_test
}

function main() {
    local cmd="$1"
    if [ -z "${cmd}" ]; then
        cmd="build"
    fi
    if [ "$1" == "test" ]; then
        info "Running CI Test ..."
        run_ci_test
    else
        run_ci_build
    fi
    success "ci${cmd} finished."
}

main "$@"


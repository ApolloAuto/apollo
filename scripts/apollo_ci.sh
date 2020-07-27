#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"
source "${TOP_DIR}/scripts/apollo_base.sh"
ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}

APOLLO_BUILD_SH="${APOLLO_ROOT_DIR}/scripts/apollo_build.sh"
APOLLO_TEST_SH="${APOLLO_ROOT_DIR}/scripts/apollo_test.sh"
APOLLO_LINT_SH="${APOLLO_ROOT_DIR}/scripts/apollo_lint.sh"

function run_ci_build() {
    env USE_ESD_CAN=${USE_ESD_CAN} bash "${APOLLO_BUILD_SH}"
}

function run_ci_test() {
    env USE_ESD_CAN=${USE_ESD_CAN} bash "${APOLLO_TEST_SH}" --config=unit_test
}

function run_ci_lint() {
    env USE_ESD_CAN=${USE_ESD_CAN} bash "${APOLLO_LINT_SH}" cpp
}

function main() {
    local cmd="$1"
    if [ -z "${cmd}" ]; then
        cmd="build"
    fi
    if [ "${cmd}" == "test" ]; then
        info "Running CI Test ..."
        run_ci_test
    elif [ "${cmd}" == "build" ]; then
        info "Running CI Build ..."
        run_ci_build
    elif [ "${cmd}" == "lint" ]; then
        info "Running CI Lint ..."
        run_ci_lint
    fi
    success "ci${cmd} finished."
}

main "$@"


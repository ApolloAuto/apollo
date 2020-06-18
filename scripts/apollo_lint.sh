#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

# STAGE="${STAGE:-dev}"
: ${STAGE:=dev}

function cpp_lint() {
    info "running cpp_lint ..."
}

function bash_lint() {
    info "running bash_lint ..."
}

function py_lint() {
    info "running py_lint ..."
}

function main() {
    local stage="${STAGE}"
    echo "$stage"
    cpp_lint "${stage}"
    bash_lint "${stage}"
    py_lint "${stage}"
}

main "$@"

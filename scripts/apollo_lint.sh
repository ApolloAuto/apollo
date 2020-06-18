#! /usr/bin/env bash
set -e

TOPDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOPDIR}/scripts/apollo.bashrc"

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
    local stage="${1:-dev}"
    echo "===${stage}"
    cpp_lint "${stage}"
    bash_lint "${stage}"
    py_lint "${stage}"
}

main "$@"

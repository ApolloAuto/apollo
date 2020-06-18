#! /usr/bin/env bash

set -e

TOPDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOPDIR}/scripts/apollo.bashrc"

function buildify() {
    local stage="${1:-dev}"

    local buildifier_cmd="$(command -v buildifier)"
    if [ -z "${buildifier_cmd}" ]; then
        local download_link="https://github.com/bazelbuild/buildtools/releases"
        error "Command buildifier not found. You can download and install" \
              "(or build) it from:"
        error "${TAB}${download_link}"
        exit 1
    fi

    buildifier_cmd="${buildifier_cmd} -mode=fix"

    local build_dirs="cyber third_party tools external"
    if [ "${stage}" == "dev" ]; then
        build_dirs="modules ${build_dirs}"
    fi
    build_dirs=$(printf "${APOLLO_ROOT_DIR}/%s " $build_dirs)

    run find ${build_dirs} -name "*BUILD" -or -name "*.bzl" -exec \
        ${buildifier_cmd} {} +

    success "buildifier run finished successfully."
    if [ -f "${APOLLO_ROOT_DIR}/BUILD" ]; then
        ${buildifier_cmd} "${APOLLO_ROOT_DIR}/BUILD"
    fi
    if [ -f "${APOLLO_ROOT_DIR}/WORKSPACE.in" ]; then
        ${buildifier_cmd} "${APOLLO_ROOT_DIR}/WORKSPACE.in"
    fi
}

function main() {
    buildify "$@"
}

main "$@"

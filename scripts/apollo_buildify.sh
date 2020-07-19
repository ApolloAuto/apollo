#! /usr/bin/env bash

set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

# STAGE="${STAGE:-dev}"
: ${STAGE:=dev}

function buildify() {
    local stage="${STAGE}"

    local buildifier_cmd="$(command -v buildifier)"
    if [ -z "${buildifier_cmd}" ]; then
        local download_link="https://github.com/bazelbuild/buildtools/releases"
        error "Command buildifier not found. You can download and install" \
              "(or build) it from:"
        error "${TAB}${download_link}"
        exit 1
    fi

    buildifier_cmd="${buildifier_cmd} -lint=fix"

    local build_dirs="cyber third_party tools"
    if [ "${stage}" == "dev" ]; then
        build_dirs="modules ${build_dirs}"
    fi
    build_dirs=$(printf "${APOLLO_ROOT_DIR}/%s " $build_dirs)

    set -x
    find ${build_dirs} -type f \
        \( -name "BUILD" -or -name "*.BUILD" -or -name "*.bzl" -or -name "*.bazel" \) \
        -exec ${buildifier_cmd} {} +
    set +x

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

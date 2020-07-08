#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

# STAGE="${STAGE:-dev}"
: ${STAGE:=dev}

function clean() {
    local stage="${STAGE}"

    if ! "${APOLLO_IN_DOCKER}" ; then
        error "The clean operation must be run from within docker container"
        exit 1
    fi
    bazel clean --async

    docs_sh="${TOP_DIR}/scripts/apollo_docs.sh"
    if [ -f "${docs_sh}" ]; then
        env STAGE="${stage}" bash "${docs_sh}" clean
    fi

    if [ "${stage}" != "dev" ]; then
        success "Apollo cleanup done."
        return
    fi

    # Remove bazel cache in associated directories
    if [ -d /apollo-simulator ]; then
        pushd /apollo-simulator >/dev/null
            bazel clean --async
        popd >/dev/null
    fi

    # Remove local bazel config.
    bash "${TOP_DIR}/scripts/apollo_config.sh" --clean

    success "Done $0 ${stage}."
}

function main() {
    clean
}

main "$@"

#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

# STAGE="${STAGE:-dev}"
: ${STAGE:=dev}
: ${USE_GPU:=0}

function bazel_build() {
    local allowed_cmodes=" fastbuild dbg opt "
    local cmode="fastbuild"
    if [ "$1" == "-c" ] || [ "$1" == "--compilation_cmode" ]; then
        cmode="$2"; shift 2
        if [[ "${allowed_cmodes}" != *" ${cmode} "* ]]; then
            error "Unknown compilation mode: ${cmode}"
            exit 1
        fi
    fi
    local modules="$@"
    #bazel build -c "${cmode}" --distdir=${APOLLO_CACHE_DIR} "$@"
}

function main() {
    bazel_build "$@"
}

main "$@"

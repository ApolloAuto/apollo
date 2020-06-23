#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}

CMDLINE_OPTIONS=
SHORTHAND_TARGETS=
DISABLED_TARGETS=

function main() {
    if [[ "$1" == "test" ]]; then
        info "CI TEST RUN"
    else
        info "CI BUILD RUN"
    fi
    success "ci${1} finished."
}

main "$@"


#! /usr/bin/env bash

set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo.bashrc"

# STAGE="${STAGE:-dev}"
: ${STAGE:=dev}

function config_noninteractive() {
    echo "config_noninteractive"
}

function config_interactive() {
    echo "config_interactive"
}

function config() {
    local stage="${STAGE}"
    local mode="$1"
    if [[ "${mode}" == "--noninteractive" ]]; then
        config_noninteractive
    else
        config_interactive
    fi
}

function main() {
    config "$@"
}

main "$@"

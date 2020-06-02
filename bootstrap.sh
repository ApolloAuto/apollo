#!/usr/bin/env bash
set -eo pipefail

TOP_DIR=$(dirname "$0")

if [[ "$1" == "--noninteractive" ]]; then
    cp -f "${TOP_DIR}/tools/sample.bazelrc" "${TOP_DIR}/.apollo.bazelrc"
    exit 0
fi

if [ -z "$PYTHON_BIN_PATH" ]; then
    PYTHON_BIN_PATH=$(which python3 || true)
fi

# Set all env variables
"$PYTHON_BIN_PATH" "${TOP_DIR}/tools/bootstrap.py" "$@"

echo "Done bootstrap.sh"

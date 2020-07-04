#!/usr/bin/env bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

bazel-bin/modules/contrib/cyber_bridge/cyber_bridge

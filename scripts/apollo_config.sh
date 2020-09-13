#! /usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/cyber/setup.bash"

set -e

BAZEL_CONF="${TOP_DIR}/.apollo.bazelrc"

ARCH="$(uname -m)"

function config_noninteractive() {
  local output_dir="${APOLLO_CACHE_DIR}/bazel"

  > "${BAZEL_CONF}"
  echo "startup --output_user_root=\"${output_dir}\"" >> "${BAZEL_CONF}"
  echo "common --distdir=\"${APOLLO_BAZEL_DISTDIR}\"" >> "${BAZEL_CONF}"
  echo >> "${BAZEL_CONF}"

  echo -e "build --action_env GCC_HOST_COMPILER_PATH=\"/usr/bin/${ARCH}-linux-gnu-gcc-7\"" >> "${BAZEL_CONF}"
  cat "${TOP_DIR}/tools/apollo.bazelrc.sample" >> "${BAZEL_CONF}"
}

function config_interactive() {
  py3_bin="$(which python3 || true)"
  # Set all env variables
  "${py3_bin}" "${TOP_DIR}/tools/bootstrap.py" "$@"
}

function config() {
  if [ $# -eq 0 ]; then
    config_noninteractive
  else
    local mode="$1"
    shift
    if [ "${mode}" == "--clean" ]; then
      rm -f "${BAZEL_CONF}"
    elif [[ "${mode}" == "--interactive" || "${mode}" == "-i" ]]; then
      config_interactive "$@"
    else
      config_noninteractive
    fi
  fi
}

function main() {
  config "$@"
}

main "$@"

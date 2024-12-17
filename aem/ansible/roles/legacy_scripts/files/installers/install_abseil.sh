#!/usr/bin/env bash

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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

# Fail on first error.
set -e

INSTALL_ATOM="${INSTALL_ATOM:-abseil-cpp-20220623.1}"

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/cmake_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/cmake_package_install_funcs.sh

# # todo(zero): if check "libabsl_base.so" is enough
if ldconfig -p | grep -q "libabsl_base.so"; then
  info "Found existing Abseil installation. Reinstallation skipped."
  exit 0
fi

# fallback
SRC_URI="${SRC_URI:-https://github.com/abseil/abseil-cpp/archive/refs/tags/${PV}.tar.gz -> ${PN}-${PV}.tar.gz}"

# overrides
src_configure() {
  mkdir -p "${WORKDIR}/${PF}_build"
  pushd "${WORKDIR}/${PF}_build"
  cmake "${WORKDIR}/${PF}" \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX}
  popd
}

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  install_package "$@"
fi

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

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/general_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/general_package_install_funcs.sh

cmake_src_configure() {
  mkdir -p "${WORKDIR}/${PF}_build"
  pushd "${WORKDIR}/${PF}_build"
  cmake ${WORKDIR}/${PF} \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release
  popd
}

cmake_src_compile() {
  pushd "${WORKDIR}/${PF}_build"
  make -j$(nproc)
  popd
}

cmake_src_install() {
  pushd "${WORKDIR}/${PF}_build"
  make install
  popd
}

# overrides

src_configure() {
  cmake_src_configure
}

src_compile() {
  cmake_src_compile
}

src_install() {
  cmake_src_install
}

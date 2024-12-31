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

INSTALL_ATOM=${INSTALL_ATOM:-gtsam-4.2}

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/cmake_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/cmake_package_install_funcs.sh

if ldconfig -p | grep libgtsam.so | grep -q /usr/lib; then
  info "gtsam was already installed"
  exit 0
fi

# fallback
# SRC_URI="${SRC_URI:-https://github.com/borglab/gtsam/archive/refs/tags/${PV}.tar.gz} -> ${PN}-${PV}.tar.gz}"
SRC_URI="${SRC_URI:-https://apollo-system.cdn.bcebos.com/archive/10.0/${PN}-${PV}.tar.gz}"
PATCHES=(
  "${FILESDIR}/gtsam-4.2_compilation_flags_$(uname -m).patch"
)

src_prepare_pre() {
  apt_get_update_and_install libboost-all-dev
  apt_get_update_and_install apollo-neo-3rd-boost
  # TODO: pack to 3rd-boost release tarball
  boost_cmake_files_url='https://apollo-system.cdn.bcebos.com/archive/10.0/3rd-boost_1.74.0_cmake_files.tar.gz'
  curl -sSL "${boost_cmake_files_url}" | tar xz -C /opt/apollo/neo/packages/3rd-boost/latest
}

cmake_src_configure() {
  mkdir -p "${WORKDIR}/${PF}_build"
  pushd "${WORKDIR}/${PF}_build"
  cmake ${WORKDIR}/${PF} \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DBoost_NO_SYSTEM_PATHS="true" \
    -DBOOST_ROOT="${BOOST_ROOT:-/opt/apollo/neo/packages/3rd-boost/latest}" \
    -DCMAKE_BUILD_TYPE=Release
  popd
}

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  install_package "$@"
fi

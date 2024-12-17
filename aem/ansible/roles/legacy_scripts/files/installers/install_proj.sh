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

INSTALL_ATOM="${INSTALL_ATOM:-proj-7.1.0}"

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/cmake_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/cmake_package_install_funcs.sh

if ldconfig -p | grep -q "libproj.so"; then
  info "Proj was already installed"
  exit 0
fi

# Note(storypku)
# apt_get_update_and_install libproj-dev
# proj installed via apt was 4.9.3, incompatible with pyproj which
# requres proj >= 6.2.0

src_prepare_pre() {
  apt_get_update_and_install \
    libsqlite3-dev \
    sqlite3 \
    libtiff-dev \
    libcurl4-openssl-dev
}

if ldconfig -p | grep -q "libproj.so"; then
  warning "Proj was already installed. Reinstallation skipped."
  exit 0
fi

SRC_URI="https://github.com/OSGeo/PROJ/releases/download/${PV}/${PN}-${PV}.tar.gz"

src_configure() {
  mkdir -p "${WORKDIR}/${PF}_build"
  pushd "${WORKDIR}/${PF}_build"
  cmake ${WORKDIR}/${PF} \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release
  popd
}

# if [[ -n "${CLEAN_DEPS}" ]]; then
#   # Remove build-deps for proj
#   apt_get_remove \
#     libsqlite3-dev \
#     sqlite3 \
#     libtiff-dev \
#     libcurl4-openssl-dev
# fi
#
# # Clean up cache to reduce layer size.
# apt-get clean &&
#   rm -rf /var/lib/apt/lists/*

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  install_package "$@"
fi

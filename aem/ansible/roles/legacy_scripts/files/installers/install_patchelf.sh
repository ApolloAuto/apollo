#!/usr/bin/env bash

###############################################################################
# Copyright 2021 The Apollo Authors. All Rights Reserved.
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

INSTALL_ATOM="${INSTALL_ATOM:-patchelf-0.12}"

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh
[[ -f "${SCRIPT_DIR}/general_package_install_funcs.sh" ]] && source ${SCRIPT_DIR}/general_package_install_funcs.sh

if command -v patchelf > /dev/null; then
  info "patchelf was already installed"
  exit 0
fi

# Note(storypku):
# Build patchelf from source to avoid the patchelf-creating-holes-in-binaries-
# thus-causing-size-bloating problem.

# VERSION="0.12"
# PKG_NAME="patchelf-${VERSION}.tar.gz"
# CHECKSUM="3dca33fb862213b3541350e1da262249959595903f559eae0fbc68966e9c3f56"
# DOWNLOAD_LINK="https://github.com/NixOS/patchelf/archive/${VERSION}.tar.gz"

SRC_URI="${SRC_URI:-https://github.com/NixOS/${PN}/archive/${PV}.tar.gz -> ${PN}-${PV}.tar.gz}"

src_configure() {
  pushd "${WORKDIR}/${PF}"
  ./bootstrap.sh
  ./configure --prefix="${INSTALL_PREFIX}"
  popd
}

# download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
#
# tar xzf "${PKG_NAME}"
# pushd "patchelf-${VERSION}" > /dev/null
# ./bootstrap.sh
# ./configure --prefix="${SYSROOT_DIR}"
# make -j "$(nproc)"
# make install
# popd
#
# rm -rf "${PKG_NAME}" "patchelf-${VERSION}"

pkg_install_post() {
  ok "Done installing ${PN}-${PV}"
}

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  # Being sourced, do nothing
  :
else
  install_package "$@"
fi

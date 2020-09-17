#!/usr/bin/env bash

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

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

# shellcheck source=./installer_base.sh
. ./installer_base.sh

apt_get_update_and_install \
    flex \
    bison \
    graphviz

# Build doxygen from source to reduce image size

VERSION="1.8.19"
PKG_NAME="doxygen-${VERSION}.src.tar.gz"
CHECKSUM="ac15d111615251ec53d3f0e54ac89c9d707538f488be57184245adef057778d3"
DOWNLOAD_LINK="http://doxygen.nl/files/${PKG_NAME}"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"
pushd "doxygen-${VERSION}" >/dev/null
    mkdir build && cd build
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=OFF

    make -j "$(nproc)"
    make install
popd

rm -rf "${PKG_NAME}" "doxygen-${VERSION}"

# VERSION="$(echo ${VERSION} | tr '_' '.')"
ok "Done installing doxygen-${VERSION}"

# Kick the ladder
apt_get_remove \
    bison flex

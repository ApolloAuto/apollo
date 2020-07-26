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
. /tmp/installers/installer_base.sh

apt-get -y update && \
    apt-get -y install \
        flex \
        bison \
        graphviz \
        doxygen

exit 0

# Source Build

VERSION="1_8_18"
PKG_NAME="doxygen-Release_${VERSION}.tar.gz"
CHECKSUM="9c88f733396dca16139483045d5afa5bbf19d67be0b8f0ea43c4e813ecfb2aa2"
DOWNLOAD_LINK="https://github.com/doxygen/doxygen/archive/Release_${VERSION}.tar.gz"
# https://github.com/doxygen/doxygen/archive/Release_1_8_18.tar.gz

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"
pushd "doxygen-Release_${VERSION}" >/dev/null
    mkdir build && cd build
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=OFF

    make -j$(nproc)
    make install
popd

rm -rf "${PKG_NAME}" "doxygen-Release_${VERSION}"

VERSION="$(echo ${VERSION} | tr '_' '.')"
info "Done installing doxygen-${VERSION}"

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

VERSION=3.16.8

TARGET_ARCH="$(uname -m)"
if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    CMAKE_SH="cmake-${VERSION}-Linux-x86_64.sh"
    SHA256SUM="0241a05bee0dcdf60e912057cc86cbedba21b9b0d67ec11bc67ad4834f182a23"
    DOWLOAD_LINK=https://github.com/Kitware/CMake/releases/download/v${VERSION}/${CMAKE_SH}
    download_if_not_cached $CMAKE_SH $SHA256SUM $DOWLOAD_LINK
    chmod a+x ${CMAKE_SH}
    mkdir -p /opt/cmake
    ./${CMAKE_SH} --skip-license --prefix="${SYSROOT_DIR}"
    rm -fr ${CMAKE_SH}
elif [[ "${ARCH}" == "aarch64" ]]; then
    apt-get -y update && \
        apt-get -y install \
        libssl-dev \
        libcurl4-openssl-dev

    PKG_NAME="CMake-${VERSION}.tar.gz"
    CHECKSUM="08b048117aa8966d477091680f1a5c67bf8ffb893a1c94ff62858cbb2358a07c"
    DOWNLOAD_LINK=https://github.com/Kitware/CMake/archive/v3.16.8.tar.gz
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
    tar xzf ${PKG_NAME}

    pushd CMake-${VERSION}
        ./bootstrap --prefix=/opt/apollo/sysroot
        make -j$(nproc)
        make install
    popd
    rm -rf "CMake-${VERSION}" "${PKG_NAME}"
fi

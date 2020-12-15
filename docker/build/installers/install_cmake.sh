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
INSTALL_MODE="$1"; shift

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

VERSION="3.16.8"

TARGET_ARCH="$(uname -m)"

function symlink() {
    ln -s ${SYSROOT_DIR}/bin/cmake /usr/local/bin/cmake
}

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    CMAKE_SH="cmake-${VERSION}-Linux-x86_64.sh"
    SHA256SUM="0241a05bee0dcdf60e912057cc86cbedba21b9b0d67ec11bc67ad4834f182a23"
    DOWLOAD_LINK=https://github.com/Kitware/CMake/releases/download/v${VERSION}/${CMAKE_SH}
    download_if_not_cached $CMAKE_SH $SHA256SUM $DOWLOAD_LINK
    chmod a+x ${CMAKE_SH}
    ./${CMAKE_SH} --skip-license --prefix="${SYSROOT_DIR}"
    symlink
    rm -fr ${CMAKE_SH}

elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    if [[ "${INSTALL_MODE}" != "build" ]]; then
        CHECKSUM="53056707081491123e705db30df6e38685f9661dc593e1790950c4ba399d3490"
        DECOMPRESSED_NAME="cmake-${VERSION}-aarch64-linux-gnu"
        PKG_NAME="${DECOMPRESSED_NAME}.tar.gz"
        DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
        download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
        tar xzf "${PKG_NAME}"
        pushd ${DECOMPRESSED_NAME}
            DEST=${SYSROOT_DIR} bash install.sh
        popd
        symlink
        rm -rf "${DECOMPRESSED_NAME}" "${PKG_NAME}"
        exit 0
    fi

    #=====================================#
    # Build CMake from source for aarch64 #
    #=====================================#
    # PreReq for source build
    apt_get_update_and_install \
        libssl-dev \
        libcurl4-openssl-dev
    PKG_NAME="CMake-${VERSION}.tar.gz"
    CHECKSUM="08b048117aa8966d477091680f1a5c67bf8ffb893a1c94ff62858cbb2358a07c"
    DOWNLOAD_LINK=https://github.com/Kitware/CMake/archive/v3.16.8.tar.gz
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
    tar xzf ${PKG_NAME}

    pushd CMake-${VERSION}
        ./bootstrap --prefix="${SYSROOT_DIR}"
        make -j$(nproc)
        make install
    popd
    symlink

    rm -rf "CMake-${VERSION}" "${PKG_NAME}"
    if [[ -n "${CLEAN_DEPS}" ]]; then
        apt_get_remove libssl-dev libcurl4-openssl-dev
    fi
fi

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*

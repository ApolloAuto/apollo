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
. ./installer_base.sh

# To reduce image size
# apt-get -y update && \
#    apt-get -y install \
#    libpoco-dev
apt_get_update_and_install \
    libssl-dev

THREAD_NUM=$(nproc)

# Install from source
VERSION=1.10.1
PKG_NAME="poco-${VERSION}-release.tar.gz"
CHECKSUM="44592a488d2830c0b4f3bfe4ae41f0c46abbfad49828d938714444e858a00818"
DOWNLOAD_LINK=https://github.com/pocoproject/poco/archive/poco-${VERSION}-release.tar.gz

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf poco-${VERSION}-release.tar.gz

pushd poco-poco-${VERSION}-release
    mkdir cmakebuild && cd cmakebuild
    # Keep only PocoFoundation
    cmake .. \
        -DENABLE_NETSSL=OFF \
        -DENABLE_CRYPTO=OFF \
        -DENABLE_JWT=OFF \
        -DENABLE_APACHECONNECTOR=OFF \
        -DENABLE_DATA_MYSQL=OFF \
        -DENABLE_DATA_POSTGRESQL=OFF \
        -DENABLE_DATA_ODBC=OFF \
        -DENABLE_MONGODB=OFF \
        -DENABLE_REDIS=OFF \
        -DENABLE_DATA_SQLITE=OFF \
        -DENABLE_DATA=OFF \
        -DENABLE_PAGECOMPILER=OFF \
        -DENABLE_PAGECOMPILER_FILE2PAGE=OFF \
        -DENABLE_ZIP=OFF \
        -DENABLE_NET=OFF \
        -DENABLE_JSON=OFF \
        -DENABLE_XML=OFF \
        -DENABLE_PDF=OFF \
        -DENABLE_POCODOC=OFF \
        -DENABLE_ENCODINGS=OFF \
        -DENABLE_UTIL=OFF \
        -DENABLE_CPPPARSER=OFF \
        -DENABLE_TESTS=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
        -DCMAKE_BUILD_TYPE=Release

    make -j${THREAD_NUM}
    make install
popd

ldconfig

# clean up
rm -rf poco-${VERSION}-release.tar.gz poco-poco-${VERSION}-release

if [[ -n "${CLEAN_DEPS}" ]]; then
    apt_get_remove libssl-dev
fi

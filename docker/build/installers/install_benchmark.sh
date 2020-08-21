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

VERSION="1.5.1"
PKG_NAME="benchmark-${VERSION}.tar.gz"
CHECKSUM="23082937d1663a53b90cb5b61df4bcc312f6dee7018da78ba00dd6bd669dfef2"
DOWNLOAD_LINK="https://github.com/google/benchmark/archive/v${VERSION}.tar.gz"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"

pushd "benchmark-${VERSION}"
    mkdir build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
        -DBENCHMARK_ENABLE_LTO=true \
        -DBENCHMARK_ENABLE_GTEST_TESTS=OFF

    make -j$(nproc)
    make install
popd

ldconfig

ok "Successfully installed benchmark ${VERSION}."

rm -fr "${PKG_NAME}" "benchmark-${VERSION}"


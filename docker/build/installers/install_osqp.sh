#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

OSQP_VER="0.5.0"
# OSQP_VER="0.6.0"
PKG_NAME_OSQP="osqp-${OSQP_VER}.tar.gz"
# FOR 0.6.0
#CHECKSUM="6e00d11d1f88c1e32a4419324b7539b89e8f9cbb1c50afe69f375347c989ba2b"

CHECKSUM="e0932d1f7bc56dbe526bee4a81331c1694d94c570f8ac6a6cb413f38904e0f64"

DOWNLOAD_LINK="https://github.com/oxfordcontrol/osqp/archive/v${OSQP_VER}.tar.gz"
download_if_not_cached "${PKG_NAME_OSQP}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME_OSQP}"

pushd "osqp-${OSQP_VER}"
    PKG_NAME="qdldl-0.1.4.tar.gz"
    CHECKSUM="4eaed3b2d66d051cea0a57b0f80a81fc04ec72c8a906f8020b2b07e31d3b549c"
    DOWNLOAD_LINK="https://github.com/oxfordcontrol/qdldl/archive/v0.1.4.tar.gz"
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
    tar xzf ${PKG_NAME} --strip-components=1 \
        -C ./lin_sys/direct/qdldl/qdldl_sources
    rm -rf ${PKG_NAME}

    mkdir build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
        -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    make install
popd

rm -rf "osqp-${OSQP_VER}" "${PKG_NAME_OSQP}"

ldconfig

ok "Successfully installed osqp-${OSQP_VER}"

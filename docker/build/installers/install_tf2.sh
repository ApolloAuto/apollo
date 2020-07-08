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

## Prereq
# apt-get -y update && apt-get -y install libboost-all-dev

VERSION="0.5.16-2"
PKG_NAME="tf2-apollo-${VERSION}.tar.gz"
DOWNLOAD_LINK="https://apollo-platform-system.bj.bcebos.com/archive/6.0/${PKG_NAME}"
CHECKSUM="486a708950319e8e6f85c4778056a30f3d9733e5cf20f37224580af1ef7937bf"

download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"

tar xzf ${PKG_NAME}

# There is a top-level README.md file, so install seperately
pushd tf2-apollo-${VERSION}
    mkdir build && cd build
    cmake .. \
        -DBUILD_TESTS=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX="${PKGS_DIR}/tf2" \
        -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    make install
popd

echo "${PKGS_DIR}/tf2/lib" >> "${APOLLO_LD_FILE}"
ldconfig

ok "Successfully installed tf2-apollo, VERSION=${VERSION}"

# clean up.
rm -fr ${PKG_NAME}  tf2-apollo-${VERSION}

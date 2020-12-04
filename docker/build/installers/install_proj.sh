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

apt_get_update_and_install \
    libsqlite3-dev \
    sqlite3 \
    libtiff-dev \
    libcurl4-openssl-dev

# Note(storypku)
# apt_get_update_and_install libproj-dev
# proj installed via apt was 4.9.3, incompatible with pyproj which
# requres proj >= 6.2.0

if ldconfig -p | grep -q "libproj.so"; then
    warning "Proj was already installed. Reinstallation skipped."
    exit 0
fi

VERSION="7.1.0"
PKG_NAME="proj-${VERSION}.tar.gz"
CHECKSUM="876151e2279346f6bdbc63bd59790b48733496a957bccd5e51b640fdd26eaa8d"
DOWNLOAD_LINK="https://github.com/OSGeo/PROJ/releases/download/${VERSION}/${PKG_NAME}"

download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"

tar xzf "${PKG_NAME}"
pushd proj-${VERSION} >/dev/null
mkdir build && cd build
cmake .. \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF \
    -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
    -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)
make install
popd >/dev/null

ldconfig

ok "Successfully built proj = ${VERSION}"

rm -fr "${PKG_NAME}" "proj-${VERSION}"

# Remove build-deps for proj
apt_get_remove \
    libsqlite3-dev \
    sqlite3 \
    libtiff-dev \
    libcurl4-openssl-dev

# Clean up cache to reduce layer size.
apt-get clean &&
    rm -rf /var/lib/apt/lists/*

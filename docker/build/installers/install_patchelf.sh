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

cd "$(dirname "${BASH_SOURCE[0]}")"

# shellcheck source=./installer_base.sh
. ./installer_base.sh

# Note(storypku):
# Build patchelf from source to avoid the patchelf-creating-holes-in-binaries-
# thus-causing-size-bloating problem.

VERSION="0.12"
PKG_NAME="patchelf-${VERSION}.tar.gz"
CHECKSUM="3dca33fb862213b3541350e1da262249959595903f559eae0fbc68966e9c3f56"
DOWNLOAD_LINK="https://github.com/NixOS/patchelf/archive/${VERSION}.tar.gz"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"
pushd "patchelf-${VERSION}" >/dev/null
    ./bootstrap.sh
    ./configure --prefix="${SYSROOT_DIR}"
    make -j "$(nproc)"
    make install
popd

rm -rf "${PKG_NAME}" "patchelf-${VERSION}"

ok "Done installing patchelf-${VERSION}"

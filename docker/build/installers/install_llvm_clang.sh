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

ARCH=$(uname -m)

CLANG_INSTALL_DIR="/opt/clang"

if [[ "${ARCH}" == "x86_64" ]]; then
    VERSION="10.0.0"

    DECOMPRESS_NAME="clang+llvm-${VERSION}-x86_64-linux-gnu-ubuntu-18.04"
    PKG_NAME="${DECOMPRESS_NAME}.tar.xz"
    CHECKSUM="b25f592a0c00686f03e3b7db68ca6dc87418f681f4ead4df4745a01d9be63843"
    DOWNLOAD_LINK="https://github.com/llvm/llvm-project/releases/download/llvmorg-${VERSION}/${PKG_NAME}"

    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    tar xJf "${PKG_NAME}"
    mv -f "${DECOMPRESS_NAME}" "${CLANG_INSTALL_DIR}"

    rm -rf "${PKG_NAME}"
fi

if [[ "${ARCH}" == "aarch64" ]]; then
    error "clang-llvm is on the way. Please stay tuned."
    exit 0
fi

__mytext="""# shellcheck shell=sh

if [ -d \"${CLANG_INSTALL_DIR}/bin\" ]; then
    export PATH=${CLANG_INSTALL_DIR}/bin:\$PATH
fi
"""

echo "${__mytext}" > /etc/profile.d/llvm-clang-path.sh


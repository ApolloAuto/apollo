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

# Install clang via apt to reduce image size
apt_get_update_and_install \
    clang-10 \
    clang-format-10

#    clang-tidy-10 \
#    clang-tools-10

sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-10 100
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-10 100
sudo update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-10 100

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*

exit 0

# Install from source

# Ref: https://releases.llvm.org/download.html
ARCH=$(uname -m)

CLANG_INSTALL_DIR="${PKGS_DIR}/llvm.clang"

VERSION="10.0.0"

if [[ "${ARCH}" == "x86_64" ]]; then
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
    DECOMPRESS_NAME="clang+llvm-${VERSION}-aarch64-linux-gnu"
    PKG_NAME="${DECOMPRESS_NAME}.tar.xz"
    CHECKSUM="c2072390dc6c8b4cc67737f487ef384148253a6a97b38030e012c4d7214b7295"
    DOWNLOAD_LINK="https://github.com/llvm/llvm-project/releases/download/llvmorg-${VERSION}/${PKG_NAME}"

    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
    tar xJf "${PKG_NAME}"
    mv -f "${DECOMPRESS_NAME}" "${CLANG_INSTALL_DIR}"

    rm -rf "${PKG_NAME}"
fi

__mytext="""
if [ -d \"${CLANG_INSTALL_DIR}/bin\" ]; then
    add_to_path ${CLANG_INSTALL_DIR}/bin
fi
"""

echo "${__mytext}" >> "${APOLLO_PROFILE}"

echo "Done installing llvm.clang ${VERSION}."

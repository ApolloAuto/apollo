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

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

TARGET_ARCH="$(uname -m)"
if [ "${TARGET_ARCH}" = "aarch64" ]; then
    warning "libtorch for aarch64 not ready"
    exit 0
fi

# Libtorch-gpu dependency
pip3_install mkl

TORCH_VERSION="1.6.0"
PKG_NAME="libtorch-cxx11-abi-shared-with-deps-${TORCH_VERSION}.zip"
DOWNLOAD_LINK="https://download.pytorch.org/libtorch/cu102/${PKG_NAME}"
CHECKSUM="fded948bd2dbee625cee33ebbd4843a69496729389e0200a90fbb667cdaeeb69"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
unzip ${PKG_NAME}

mkdir -p /usr/local/libtorch_gpu/
pushd libtorch
    mv include /usr/local/libtorch_gpu/include
    mv lib     /usr/local/libtorch_gpu/lib
    mv share   /usr/local/libtorch_gpu/share
popd

# Cleanup
rm -rf libtorch ${PKG_NAME}

PKG_NAME="libtorch-cxx11-abi-shared-with-deps-${TORCH_VERSION}+cpu.zip"
DOWNLOAD_LINK="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${TORCH_VERSION}%2Bcpu.zip"
CHECKSUM="c6c0d3a87039338f7812a1ae343b9e48198536f20d1415b0e5a9a15ba7b90b3f"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

unzip ${PKG_NAME}

mkdir -p /usr/local/libtorch_cpu/
pushd libtorch
    mv include /usr/local/libtorch_cpu/include
    mv lib     /usr/local/libtorch_cpu/lib
    mv share   /usr/local/libtorch_cpu/share
popd

# Cleanup
rm -rf libtorch ${PKG_NAME}

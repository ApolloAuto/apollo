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

# PKG_NAME="libtorch-cxx11-abi-shared-with-deps-1.5.0.zip"
# DOWNLOAD_LINK="https://download.pytorch.org/libtorch/cu102/${PKG_NAME}"
# CHECKSUM="0efdd4e709ab11088fa75f0501c19b0e294404231442bab1d1fb953924feb6b5"

PKG_NAME="libtorch-1.5.1-gpu-apollo.zip"
DOWNLOAD_LINK="https://apollo-platform-system.bj.bcebos.com/archive/6.0/${PKG_NAME}"
CHECKSUM="6e8aa94e2f7086d3ecc79484ade50cdcac69f1b51b1f04e4feda2f9384b4c380"

#https://download.pytorch.org/libtorch/cu102/libtorch-cxx11-abi-shared-with-deps-1.5.0.zip
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
unzip ${PKG_NAME}

pushd libtorch_gpu
    mkdir -p /usr/local/libtorch_gpu/
    mv include /usr/local/libtorch_gpu/include
    mv lib     /usr/local/libtorch_gpu/lib
    mv share   /usr/local/libtorch_gpu/share
popd

# Cleanup
rm -rf libtorch_gpu ${PKG_NAME}

PKG_NAME="libtorch-cxx11-abi-shared-with-deps-1.5.0+cpu.zip"
DOWNLOAD_LINK="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip"
CHECKSUM="3e438237a08099a4bf014335cd0da88708da3a1678aec12a46c67305792b5fa4"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

unzip ${PKG_NAME}
pushd libtorch
    mkdir -p /usr/local/libtorch_cpu/
    mv include /usr/local/libtorch_cpu/include
    mv lib     /usr/local/libtorch_cpu/lib
    mv share   /usr/local/libtorch_cpu/share
popd

# Cleanup
rm -rf libtorch ${PKG_NAME}

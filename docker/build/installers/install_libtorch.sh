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

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# TODO(build): Docs on how to build libtorch on Jetson boards
# References:
#   https://github.com/ApolloAuto/apollo/blob/pre6/docker/build/installers/install_libtorch.sh
#   https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.pytorch
#   https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-6-0-now-available
#   https://github.com/pytorch/pytorch/blob/master/docker/caffe2/ubuntu-16.04-cpu-all-options/Dockerfile
bash ${CURR_DIR}/install_mkl.sh

TARGET_ARCH="$(uname -m)"

##============================================================##
# libtorch_cpu

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    if [[ "${APOLLO_DIST}" == "stable" ]]; then
        # https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip
        VERSION="1.6.0-1"
        CHECKSUM="4931fe651f2098ee3f0c4147099bd5dddcc15c0e9108e49a5ffdf46d98a7092a"
    else # testing
        VERSION="1.7.0-1"
        CHECKSUM="9c99005f4335c9e8d21cf243eed557284998592e1f8b157fb659c5c887dfb9e6"
    fi
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    VERSION="1.6.0-1"
    CHECKSUM="6d1fba522e746213c209fbf6275fa6bac68e360bcd11cbd4d3bdbddb657bee82"
else
    error "libtorch for ${TARGET_ARCH} not ready. Exiting..."
    exit 1
fi

PKG_NAME="libtorch_cpu-${VERSION}-linux-${TARGET_ARCH}.tar.gz"
DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"
mv "${PKG_NAME%.tar.gz}" /usr/local/libtorch_cpu
rm -f "${PKG_NAME}"
ok "Successfully installed libtorch_cpu ${VERSION}"


##============================================================##
# libtorch_gpu
if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    if [[ "${APOLLO_DIST}" == "stable" ]]; then
        VERSION="1.6.0-1"
        CHECKSUM="8ca52235cb91aba6d39fb144ec5414bebf0e94a2ee03115ae83650fc4d421719"
        PKG_NAME="libtorch_gpu-${VERSION}-cu102-linux-x86_64.tar.gz"
    else
        VERSION="1.7.0-1"
        CHECKSUM="08932dddd0a8551e6d94397560257eefe4e209f11ac4fa6172b28def0a1d7da9"
        PKG_NAME="libtorch_gpu-${VERSION}-cu111-linux-x86_64.tar.gz"
    fi
else # AArch64
    VERSION="1.6.0-1"
    PKG_NAME="libtorch_gpu-1.6.0-1-linux-aarch64.tar.gz"
    CHECKSUM="eeb5a223d9dbe40fe96f16e6711c49a3777cea2c0a8da2445d63e117fdad0385"
fi

DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"
mv "${PKG_NAME%.tar.gz}" /usr/local/libtorch_gpu

# Cleanup
rm -f "${PKG_NAME}"
ok "Successfully installed libtorch_gpu ${VERSION}"

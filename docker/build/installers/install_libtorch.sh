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

# TODO(build): Docs on how to build libtorch on Jetson boards
# References:
#   https://github.com/ApolloAuto/apollo/blob/pre6/docker/build/installers/install_libtorch.sh
#   https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.pytorch
#   https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-6-0-now-available
#   https://github.com/pytorch/pytorch/blob/master/docker/caffe2/ubuntu-16.04-cpu-all-options/Dockerfile

TARGET_ARCH="$(uname -m)"

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    # Libtorch-gpu dependency for x86_64
    pip3_install mkl
fi

##============================================================##
# libtorch_cpu

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    if [[ "${APOLLO_DIST}" == "stable" ]]; then
        # https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip
        VERSION="1.5.0"
        CHECKSUM="0c81319793763b77b299088e86ce281e8772a6ae7b1c1a37b56d0eee93911edd"
    else # testing
        VERSION="1.7.0"
        CHECKSUM="dddde039c97fc5caf10c16c2f8fa75351fdbe79c4e90c1ec6e20a7341de9c3c8"
    fi
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    VERSION="1.6.0"
    CHECKSUM="712a33a416767de625a3f2da54ec384d50882b10e1b1fc5da8df4158ef6edd06"
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
        # TODO(build): bump libtorch to 1.6.0
        VERSION="1.5.1"
        CHECKSUM="c16dfc8393c0ea27cdbd596913ae0ac40ab5e29ba211b7ef71475ff75b8b5af0"
        PKG_NAME="libtorch_gpu-1.5.1-cu102-linux-x86_64.tar.gz"
    else
        VERSION="1.7.0"
        CHECKSUM="dfb13e750eea5a123f97d1886c7ec3e000d62e8fe20583fb2fc8a50d75d5fdce"
        PKG_NAME="libtorch_gpu-1.7.0-cu111-linux-x86_64.tar.gz"
    fi
else # AArch64
    VERSION="1.6.0"
    PKG_NAME="libtorch_gpu-1.6.0-linux-aarch64.tar.gz"
    CHECKSUM="bf9495110641b0f0dda44e3c93f06f221b54af990688a9202a377ec9b3348666"
fi

DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"
mv "${PKG_NAME%.tar.gz}" /usr/local/libtorch_gpu

# Cleanup
rm -f "${PKG_NAME}"
ok "Successfully installed libtorch_gpu ${VERSION}"

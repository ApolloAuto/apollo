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
if [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    # TODO(build): Docs on how to build libtorch on Jetson boards
    # References:
    #   https://github.com/ApolloAuto/apollo/blob/pre6/docker/build/installers/install_libtorch.sh
    #   https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.pytorch
    #   https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-6-0-now-available
    #   https://github.com/pytorch/pytorch/blob/master/docker/caffe2/ubuntu-16.04-cpu-all-options/Dockerfile
    VERSION="1.6.0"

    # libtorch_cpu
    PKG_NAME="libtorch_cpu-1.6.0-linux-aarch64.tar.gz"
    CHECKSUM="712a33a416767de625a3f2da54ec384d50882b10e1b1fc5da8df4158ef6edd06"
    DOWNLOAD_LINK="https://apollo-platform-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    tar xzf "${PKG_NAME}"
    mv "${PKG_NAME%.tar.gz}" /usr/local/libtorch_cpu
    rm -f "${PKG_NAME}"
    ok "Successfully installed libtorch_cpu ${VERSION}"

    ##============================================================##
    # libtorch_gpu
    PKG_NAME="libtorch_gpu-1.6.0-linux-aarch64.tar.gz"
    CHECKSUM="bf9495110641b0f0dda44e3c93f06f221b54af990688a9202a377ec9b3348666"
    DOWNLOAD_LINK="https://apollo-platform-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    tar xzf "${PKG_NAME}"
    mv "${PKG_NAME%.tar.gz}" /usr/local/libtorch_gpu
    rm -f "${PKG_NAME}"
    ok "Successfully installed libtorch_gpu ${VERSION}"
    exit 0
fi

##===============================================================##

# Libtorch-gpu dependency for x86_64
pip3_install mkl

# TODO(build): bump libtorch to 1.6.0
PKG_NAME="libtorch-1.5.1-gpu-apollo.zip"
DOWNLOAD_LINK="https://apollo-platform-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
CHECKSUM="6e8aa94e2f7086d3ecc79484ade50cdcac69f1b51b1f04e4feda2f9384b4c380"

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

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
    # https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip
    VERSION="1.7.0-2"
    CHECKSUM="02fd4f30e97ce8911ef933d0516660892392e95e6768b50f591f4727f6224390"
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

determine_gpu_use_host
if [[ "${USE_NVIDIA_GPU}" -eq 1 ]]; then
    # libtorch_gpu nvidia
    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        VERSION="1.7.0-2"
        CHECKSUM="b64977ca4a13ab41599bac8a846e8782c67ded8d562fdf437f0e606cd5a3b588"
        PKG_NAME="libtorch_gpu-${VERSION}-cu111-linux-x86_64.tar.gz"
    else # AArch64
        VERSION="1.6.0-1"
        PKG_NAME="libtorch_gpu-1.6.0-1-linux-aarch64.tar.gz"
        CHECKSUM="eeb5a223d9dbe40fe96f16e6711c49a3777cea2c0a8da2445d63e117fdad0385"
    fi

    DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    tar xzf "${PKG_NAME}"
    mv "${PKG_NAME%.tar.gz}" /usr/local/libtorch_gpu/
elif [[ "${USE_AMD_GPU}" -eq 1 ]]; then
    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        PKG_NAME="libtorch_amd.tar.gz"
        FILE_ID="1UMzACmxzZD8KVitEnSk-BhXa38Kkl4P-"
    else # AArch64
        error "AMD libtorch for ${TARGET_ARCH} not ready. Exiting..."
        exit 1
    fi
    DOWNLOAD_LINK="https://docs.google.com/uc?export=download&id=${FILE_ID}"
    wget --load-cookies /tmp/cookies.txt \
        "https://docs.google.com/uc?export=download&confirm=
            $(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies \
            --no-check-certificate ${DOWNLOAD_LINK} \
            -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=${FILE_ID}" \
        -O "${PKG_NAME}" && rm -rf /tmp/cookies.txt

    tar xzf "${PKG_NAME}"
    mv "${PKG_NAME%.tar.gz}/libtorch" /usr/local/libtorch_gpu/
    mv "${PKG_NAME%.tar.gz}/libtorch_deps/libamdhip64.so.4" /opt/rocm/hip/lib/
    mv "${PKG_NAME%.tar.gz}/libtorch_deps/libmagma.so" /opt/apollo/sysroot/lib/
    mv "${PKG_NAME%.tar.gz}/libtorch_deps/"* /usr/local/lib/
    rm -r "${PKG_NAME%.tar.gz}"
    ldconfig
fi

# Cleanup
rm -f "${PKG_NAME}"
ok "Successfully installed libtorch_gpu ${VERSION}"

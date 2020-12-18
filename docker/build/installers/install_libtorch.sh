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
    pip3_install mkl==2021.1.1

    # Workaround for duplicate entries in mkl installation
    MKL_LIBDIR="/usr/local/lib"
    for so in ${MKL_LIBDIR}/libmkl*.so; do
        so1="${so}.1"
        if [[ "$(basename ${so})" == "libmkl_sycl.so" ]]; then
            if [[ -f "${so1}" ]]; then
                rm -f ${so1}
            fi
            rm -f ${so}
            continue
        fi
        if [[ -f "${so}.1" ]]; then
            cs1=$(sha256sum ${so1} | awk '{print $1}')
            cs0=$(sha256sum ${so} | awk '{print $1}')
            if [[ "${cs1}" == "${cs0}" ]]; then
                so1_name="$(basename $so1)"
                warning "Duplicate so ${so} with ${so1_name} found, re-symlinking..."
                info "Now perform: rm -f ${so} && ln -s ${so1_name} ${so}"
                rm -f ${so} && ln -s ${so1_name} ${so}
            fi
        fi
    done
fi

##============================================================##
# libtorch_cpu

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    if [[ "${APOLLO_DIST}" == "stable" ]]; then
        # https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.5.0%2Bcpu.zip
        VERSION="1.6.0"
        CHECKSUM="2bc75f4b38e7c23f99a2e0367b64c8626cac1442e87b7ab9878859bbf10973fe"
    else # testing
        VERSION="1.7.0"
        CHECKSUM="1baccc141347ce33cd998513f5cfdb0b5c359d66cd2b59b055c9eadc9e954d19"
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
        VERSION="1.6.0"
        CHECKSUM="ab540c8d6c088ca5eda0e861bb39e985b046f363c3a1fbd65176090c202f8af3"
        PKG_NAME="libtorch_gpu-1.6.0-cu102-linux-x86_64.tar.gz"
    else
        VERSION="1.7.0"
        CHECKSUM="d2356b641d78e33d5decebf8c32f726b831045d5a0aff276545b40a259225885"
        PKG_NAME="libtorch_gpu-1.7.0-cu111-linux-x86_64.tar.gz"
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

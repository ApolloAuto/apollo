#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

INSTALL_MODE=$1; shift

cd "$(dirname "${BASH_SOURCE[0]}")"

. /tmp/installers/installer_base.sh

warning "Caffe 1.0 support will be abandoned soon!"

# Make caffe-1.0 compilation pass
CUDNN_HEADER_DIR="/usr/include/$(uname -m)-linux-gnu"
[[ -e "${CUDNN_HEADER_DIR}/cudnn.h" ]] || \
    ln -s "${CUDNN_HEADER_DIR}/cudnn_v7.h" "${CUDNN_HEADER_DIR}/cudnn.h"

# http://caffe.berkeleyvision.org/install_apt.html
# apt-get -y update && \
#    apt-get -y install \
#    caffe-cuda && \
#    apt-get clean && \
#    rm -rf /var/lib/apt/lists/*

# PreReqs:
#     bash /tmp/installers/install_boost.sh
# Disabled
#    libqhull-dev
#    liblmdb-dev
#    libleveldb-dev
#    libopenni-dev
#    libvtk6-dev
#    libvtk6-qt-dev

apt-get -y update && \
    apt-get -y install \
    libsnappy-dev \
    libhdf5-dev \
    libflann-dev \
    libopenblas-dev \
    libatlas-base-dev

# BLAS: install ATLAS by sudo apt-get install libatlas-base-dev or install
# OpenBLAS by sudo apt-get install libopenblas-dev or MKL for better CPU performance.

pip3_install numpy -U

# Build Caffe from source
VERSION="1.0"
PKG_NAME="caffe-1.0.tar.gz"
CHECKSUM="71d3c9eb8a183150f965a465824d01fe82826c22505f7aa314f700ace03fa77f"
DOWNLOAD_LINK="https://github.com/BVLC/caffe/archive/1.0.tar.gz"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"

# Ref: caffe-1.0/scripts/travis/configure-cmake.sh
pushd caffe-${VERSION}
    patch -p1 < "/tmp/installers/caffe-${VERSION}.apollo.patch"
    mkdir build && cd build

	cmake .. \
	    -DBUILD_SHARED_LIBS=ON \
        -DBUILD_docs=OFF \
        -DBUILD_matlab=OFF \
        -DUSE_CUDNN=ON \
        -DUSE_NCCL=OFF \
        -DUSE_LMDB=OFF \
        -DUSE_LEVELDB=OFF \
        -DUSE_OPENCV=OFF \
        -DBUILD_python=ON \
        -Dpython_version=3 \
        -DBLAS=Open \
        -DCUDA_ARCH_NAME=Manual \
        -DCUDA_ARCH_BIN="${SUPPORTED_NVIDIA_SMS}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${PKGS_DIR}/caffe"
    make -j$(nproc)
    make install
popd

echo "${PKGS_DIR}/caffe/lib" | tee -a "${APOLLO_LD_FILE}"
ldconfig

# Clean up.
rm -rf ${PKG_NAME} caffe-${VERSION}

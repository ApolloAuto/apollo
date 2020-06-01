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
warning "Caffe 1.0 support will be abandoned before Apollo 6.0 release!"

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

if [[ "${INSTALL_MODE}" != "build" ]]; then
    PKG_NAME="caffe-1.0-x86_64.tar.gz"
    CHECKSUM="aa46ad0b263ca461e18f3b424e147efd6e95ed9dd55dae200cc63f214e5e2772"
    DOWNLOAD_LINK="https://apollo-platform-system.bj.bcebos.com/archive/6.0/${PKG_NAME}"

    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    info "Extracting ${PKG_NAME} to /usr/local/caffe ..."
    tar xzf ${PKG_NAME} -C /usr/local
    rm -rf ${PKG_NAME}
    exit 0
fi

#Note(storypku): Build Caffe from source
apt-get -y update && \
    apt-get -y install \
    libleveldb-dev \
    libsnappy-dev \
    libopencv-dev \
    libhdf5-serial-dev \
    libhdf5-dev \
    libboost-all-dev \
    liblmdb-dev \
    libatlas-base-dev \
    libopenblas-dev \
    libqhull-dev

# And...
# protobuf/gflags/glog

## packages not used in building
# libflann-dev \
# libopenni-dev \
# mpi-default-dev
# libvtk6-dev
# libvtk6-qt-dev

# BLAS: install ATLAS by sudo apt-get install libatlas-base-dev or install
# OpenBLAS by sudo apt-get install libopenblas-dev or MKL for better CPU performance.

VERSION="1.0"
PKG_NAME="caffe-1.0.tar.gz"
CHECKSUM="71d3c9eb8a183150f965a465824d01fe82826c22505f7aa314f700ace03fa77f"
DOWNLOAD_LINK="https://github.com/BVLC/caffe/archive/1.0.tar.gz"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"

MY_DEST_DIR=/usr/local/caffe

pushd caffe-${VERSION}
    patch -p1 < "/tmp/installers/caffe-${VERSION}.apollo.patch"
    mkdir build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_python=OFF \
        -DBUILD_docs=OFF \
        -DUSE_NCCL=ON \
        -DCMAKE_INSTALL_PREFIX=${MY_DEST_DIR}
    make -j$(nproc)
    make install
popd

rm -rf "${MY_DEST_DIR}/{bin,python}"

# Clean up.
rm -rf ${PKG_NAME} caffe-${VERSION}

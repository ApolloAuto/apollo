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

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

ARCH=$(uname -m)
THREAD_NUM=$(nproc)

# Install gflags.
VERSION="1.30.0"
CHECKSUM="419dba362eaf8f1d36849ceee17c3e2ff8ff12ac666b42d3ff02a164ebe090e9"
PKG_NAME="grpc-${VERSION}.tar.gz"
DOWNLOAD_LINK="https://github.com/grpc/grpc/archive/refs/tags/v${VERSION}.tar.gz"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"

pushd grpc-${VERSION}
    mkdir build && cd build
    cmake ../.. -DgRPC_INSTALL=ON                \
                -DBUILD_SHARED_LIBS=ON           \
                -DCMAKE_BUILD_TYPE=Release       \
                -DgRPC_ABSL_PROVIDER=package     \
                -DgRPC_CARES_PROVIDER=package    \
                -DgRPC_PROTOBUF_PROVIDER=package \
                -DgRPC_RE2_PROVIDER=package      \
                -DgRPC_SSL_PROVIDER=package      \
                -DgRPC_ZLIB_PROVIDER=package
    make -j${THREAD_NUM}
    make install
popd

ldconfig

# cleanup
rm -rf $PKG_NAME grpc-$VERSION

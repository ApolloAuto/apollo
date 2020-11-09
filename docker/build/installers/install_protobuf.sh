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

# Notes on Protobuf Installer:
# 1) protobuf for cpp didn't need to be pre-installed into system
# 2) protobuf for python should be provided for cyber testcases

VERSION="3.12.3"

PKG_NAME="protobuf-${VERSION}.tar.gz"
CHECKSUM="71030a04aedf9f612d2991c1c552317038c3c5a2b578ac4745267a45e7037c29"
DOWNLOAD_LINK="https://github.com/protocolbuffers/protobuf/archive/v${VERSION}.tar.gz"

#PKG_NAME="protobuf-cpp-${VERSION}.tar.gz"
#CHECKSUM="4ef97ec6a8e0570d22ad8c57c99d2055a61ea2643b8e1a0998d2c844916c4968"
#DOWNLOAD_LINK="https://github.com/protocolbuffers/protobuf/releases/download/v${VERSION}/protobuf-cpp-${VERSION}.tar.gz"

download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"

tar xzf ${PKG_NAME}

# https://developers.google.com/protocol-buffers/docs/reference/python-generated#cpp_impl
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp

pushd protobuf-${VERSION}
mkdir cmake/build && cd cmake/build

cmake .. \
    -DBUILD_SHARED_LIBS=ON \
    -Dprotobuf_BUILD_TESTS=OFF \
    -DCMAKE_INSTALL_PREFIX:PATH="/usr" \
    -DCMAKE_BUILD_TYPE=Release

# ./configure --prefix=/usr
make -j$(nproc)
make install

ldconfig

cd ../../python
# Cf. https://github.com/protocolbuffers/protobuf/tree/master/python
python setup.py install --cpp_implementation

popd

echo -e "\nexport PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp" | tee -a "${APOLLO_PROFILE}"
ok "Successfully installed protobuf, VERSION=${VERSION}"

# Clean up.
rm -fr ${PKG_NAME}  protobuf-${VERSION}

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

. /tmp/installers/installer_base.sh

VERSION="3.11.2"
PKG_NAME="protobuf-cpp-${VERSION}.tar.gz"
CHECKSUM="f2f180e9343cbb2b9a8482255bfec2176a2cc7fa22de496535a0a0cf38797495"
DOWNLOAD_LINK="https://github.com/protocolbuffers/protobuf/releases/download/v${VERSION}/protobuf-cpp-${VERSION}.tar.gz"

#https://github.com/protocolbuffers/protobuf/releases/download/v3.11.2/protobuf-cpp-3.11.2.tar.gz

download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"

tar xzf ${PKG_NAME}

pushd protobuf-${VERSION}
mkdir cmake/build && cd cmake/build

# Note(storypku): We install protobuf in /usr/local to avoid
# conflicts with the system provided version.
cmake .. -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX:PATH=/usr/local

# ./configure --prefix=/usr
make -j`nproc`
make install

popd

ldconfig
ok "Successfully installed $protobuf-cpp, VERSION=${VERSION}"

# Clean up.
rm -fr ${PKG_NAME}  protobuf-${VERSION}

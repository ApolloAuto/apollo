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

VERSION="3.2.1-1"
PKG_NAME="qp-oases-${VERSION}.tar.gz"
DOWNLOAD_LINK="https://github.com/ApolloAuto/qp-oases/archive/v${VERSION}.tar.gz"
CHECKSUM="2cf87de73c9987efe9458e41c3c8782e26fac1560db565a8695f78d7ccdec38a"

download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"

tar xzf "${PKG_NAME}"

pushd qp-oases-${VERSION}
mkdir bin

cat << MYFLAGS
-Wall -pedantic -Wshadow \
-Wfloat-equal -O3 -Wconversion \
-Wsign-conversion -fPIC -DLINUX -DSOLVER_NONE \
-D__NO_COPYRIGHT__
MYFLAGS

THREAD_NUM=$(nproc)
make -j${THREAD_NUM} CXXFLAGS="${MYFLAGS}"

cp bin/libqpOASES.so /usr/local/lib
cp -r include/* /usr/local/include
popd

ok "Successfully build qp-oases-${VERSION}"

# Clean up.
rm -fr ${PKG_NAME} qp-oases-${VERSION}

# https://github.com/coin-or/qpOASES


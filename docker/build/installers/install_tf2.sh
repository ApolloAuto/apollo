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

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

. /tmp/installers/installer_base.sh

## Prereq
# apt-get -y update && apt-get -y install libboost-all-dev

VERSION="0.5.16-1"
PKG_NAME="tf2-apollo-${VERSION}.tar.gz"
DOWNLOAD_LINK="https://apollo-platform-system.bj.bcebos.com/archive/6.0/${PKG_NAME}"
CHECKSUM="ec23f028381779246a8e51fc0f0f84a6177d296337a16a4facdefdc557d3f6d1"

download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"

tar xzf ${PKG_NAME}

pushd tf2-apollo-${VERSION}
mkdir build && cd build
cmake .. -DBUILD_TESTS=OFF \
	 -DCMAKE_INSTALL_PREFIX=/usr/local/tf2
make -j$(nproc)
make install

popd

ok "Successfully installed tf2-apollo, VERSION=${VERSION}"

# clean up.
rm -fr ${PKG_NAME}  tf2-apollo-${VERSION}

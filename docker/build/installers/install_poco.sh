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

apt-get -y update && \
    apt-get -y install \
    libssl-dev \
    libpoco-dev

exit 0

. /tmp/installers/installer_base.sh

THREAD_NUM=$(nproc)

# Install from source
VERSION=1.10.1
PKG_NAME="poco-${VERSION}-release.tar.gz"
CHECKSUM="44592a488d2830c0b4f3bfe4ae41f0c46abbfad49828d938714444e858a00818"
DOWNLOAD_LINK=https://github.com/pocoproject/poco/archive/poco-${VERSION}-release.tar.gz

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf poco-${VERSION}-release.tar.gz

pushd poco-poco-${VERSION}-release
    mkdir cmakebuild && cd cmakebuild
    cmake .. -DBUILD_SHARED_LIBS=ON
    make -j${THREAD_NUM}
    make install
popd

# clean up
rm -rf poco-${VERSION}-release.tar.gz poco-poco-${VERSION}-release

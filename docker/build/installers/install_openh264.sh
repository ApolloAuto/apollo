#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
  nasm

. /tmp/installers/installer_base.sh

VERSION="2.0.0"
PKG_NAME="openh264-${VERSION}.tar.gz"
CHECKSUM="73c35f80cc487560d11ecabb6d31ad828bd2f59d412f9cd726cc26bfaf4561fd"
DOWNLOAD_LINK="https://github.com/cisco/openh264/archive/v${VERSION}.tar.gz"

# Prepare
download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"

tar xzf ${PKG_NAME}

# Build and install.
pushd openh264-${VERSION}
  make -j`nproc`
  make install
popd

# Clean
rm -fr "${PKG_NAME}" "openh264-${VERSION}"


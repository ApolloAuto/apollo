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
. ./installer_base.sh

apt_get_update_and_install nasm

VERSION="2.1.1"
PKG_NAME="openh264-${VERSION}.tar.gz"
CHECKSUM="af173e90fce65f80722fa894e1af0d6b07572292e76de7b65273df4c0a8be678"
DOWNLOAD_LINK="https://github.com/cisco/openh264/archive/v${VERSION}.tar.gz"

# Prepare
download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"

tar xzf ${PKG_NAME}

# Build and install.
pushd openh264-${VERSION}
    make \
        BUILDTYPE=Release \
        PREFIX="${SYSROOT_DIR}" \
        -j$(nproc) install
popd
ldconfig

apt_get_remove nasm

# Clean
rm -fr "${PKG_NAME}" "openh264-${VERSION}"

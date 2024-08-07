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
. ./installer_base.sh

if dpkg -l | grep -q 'hesai2-driver'; then
  info "Found existing hesai2 driver deb installation. Reinstallation skipped"
  exit 0
fi

VERSION="2.0.5"
CHECKSUM=
TARGET_ARCH="$(uname -m)"

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
  # sha256sum
    CHECKSUM="a09e9a9f08c868c7ef1172f9c7faa88e9039bf32d046d17416ddf5994de7e3d6"
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    CHECKSUM="5c4a628e2f6faf654d2618d4ed1cb4473d936468661d717e8efa8db4fc7096c0"
fi

PKG_NAME="hesai2-driver_${VERSION}_${TARGET_ARCH}.deb"
DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/9.0/${PKG_NAME}"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

dpkg -i ${PKG_NAME}

# Clean up
rm -f "${PKG_NAME}"

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

if dpkg -l | grep -q 'livox-driver'; then
  info "Found existing livox driver deb installation. Reinstallation skipped"
  exit 0
fi

VERSION="1.2.5"
CHECKSUM=
TARGET_ARCH="$(uname -m)"

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
  # sha256sum
    CHECKSUM="acfee4cfff5fc6041cf1a9dd460b8cf312a09358a95efbcd0da840072ff0b0d9"
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    CHECKSUM="e6c950d4e74f4d8a9fd951f517bdd2a9432595ebfe19ad8938b91caa8ab2217d"
fi

PKG_NAME="livox-driver_${VERSION}_${TARGET_ARCH}.deb"
DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/9.0/${PKG_NAME}"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

dpkg -i ${PKG_NAME}

# Clean up
rm -f "${PKG_NAME}"

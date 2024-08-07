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

if dpkg -l | grep -q 'vanjee-driver'; then
  info "Found existing vanjee driver deb installation. Reinstallation skipped"
  exit 0
fi


VERSION="1.10.1"
PKG_NAME="vanjee-driver_${VERSION}_all.deb"
DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/9.0/${PKG_NAME}"
CHECKSUM="bb05e18aa2044dfed898aed0e82f83754192c04e59479724eaa192b5a6c23743"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

dpkg -i ${PKG_NAME}

# Clean up
rm -f "${PKG_NAME}"

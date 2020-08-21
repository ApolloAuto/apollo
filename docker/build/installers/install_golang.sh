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

VERSION="1.14.4"
ARCH="$(uname -m)"
if [[ "${ARCH}" == "x86_64" ]]; then
    PKG_NAME="go${VERSION}.linux-amd64.tar.gz"
    CHECKSUM="b518f21f823759ee30faddb1f623810a432499f050c9338777523d9c8551c62c"
elif [[ "${ARCH}" == "aarch64" ]]; then
    PKG_NAME="go${VERSION}.linux-arm64.tar.gz"
    CHECKSUM="05dc46ada4e23a1f58e72349f7c366aae2e9c7a7f1e7653095538bc5bba5e077"
else
    error "Unsupported arch: ${ARCH}. Exiting..."
    exit 1
fi
DOWNLOAD_LINK="https://dl.google.com/go/${PKG_NAME}"

download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"
tar xzf ${PKG_NAME} -C "${PKGS_DIR}"

GOROOT="${PKGS_DIR}/go"

MY_TEXT="""
export GOROOT=${GOROOT}
if [ -x \"\${GOROOT}/bin/go\" ]; then
    add_to_path \"\${GOROOT}/bin\"
fi
"""

echo "${MY_TEXT}" | tee -a "${APOLLO_PROFILE}"
ok "Successfully installed go ${VERSION} to ${GOROOT}"

# clean up.
rm -fr "${PKG_NAME}"

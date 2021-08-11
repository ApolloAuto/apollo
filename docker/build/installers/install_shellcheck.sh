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

# shellcheck source=./installer_base.sh
. ./installer_base.sh

# Rather than
# apt_get_update_and_install \
#    shellcheck
# for shellcheck 0.4.6 on Ubuntu 18.04, we prefer to
# download latest shellcheck binaries.

TARGET_ARCH="$(uname -m)"

VERSION="0.7.1"
# As always, x86_64 and aarch64 only
PKG_NAME="shellcheck-v${VERSION}.linux.${TARGET_ARCH}.tar.xz"
DOWNLOAD_LINK="https://github.com/koalaman/shellcheck/releases/download/v${VERSION}/${PKG_NAME}"
CHECKSUM=

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    CHECKSUM="64f17152d96d7ec261ad3086ed42d18232fcb65148b44571b564d688269d36c8"
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    CHECKSUM="b50cc31509b354ab5bbfc160bc0967567ed98cd9308fd43f38551b36cccc4446"
else
    warning "${TARGET_ARCH} architecture is currently not supported."
    exit 1
fi

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xJf "${PKG_NAME}"

pushd "shellcheck-v${VERSION}" >/dev/null
    mv shellcheck "${SYSROOT_DIR}/bin"
    chmod a+x "${SYSROOT_DIR}/bin/shellcheck"
popd

rm -rf "${PKG_NAME}" "shellcheck-v${VERSION}"

ok "Done installing shellcheck ${VERSION}"

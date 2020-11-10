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

TARGET_ARCH="$(uname -m)"

if [[ "${TARGET_ARCH}" == "x86_64" && "${APOLLO_DIST}" == "testing" ]]; then
    VERSION="11.1.74-1"
else
    VERSION="10.2.89-1"
fi

MAIN_VER_DOT="${VERSION%.*}"
DEMO_SUITE_DEST_DIR="/usr/local/cuda-${MAIN_VER_DOT}/extras/demo_suite"

if [[ -x "${DEMO_SUITE_DEST_DIR}/deviceQuery" ]]; then
    info "Found existing deviceQuery under ${DEMO_SUITE_DEST_DIR}, do nothing."
    exit 0
fi

# VERSION="10.2.89-1"
function main_cuda_version() {
    local ver="${1%.*}"
    echo "${ver//./-}"
}

MAIN_VER="$(main_cuda_version ${VERSION})"
DEVICE_QUERY_BINARY=
CHECKSUM=

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    DEVICE_QUERY_BINARY="deviceQuery-${MAIN_VER}_${VERSION}_amd64.bin"
    if [[ "${APOLLO_DIST}" == "testing" ]]; then
        CHECKSUM="da573cc68ad5fb227047064d73123a8a966df35be19b68163338b6dc0d576c84"
    else
        CHECKSUM="ed6add99a9c6eb0ff922b7f6a49334665d833bdfd416f5d1920c2b4a8dc14542"
    fi
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    DEVICE_QUERY_BINARY="deviceQuery-${MAIN_VER}_${VERSION}_arm64.bin"
    CHECKSUM="fe55e0da8ec20dc13e778ddf7ba95bca45efd51d8f4e6c4fd05f2fb9856f4ac8"
else
    error "Support for ${TARGET_ARCH} not ready yet."
    exit 1
fi

DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${DEVICE_QUERY_BINARY}"
download_if_not_cached "${DEVICE_QUERY_BINARY}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

[[ -d "${DEMO_SUITE_DEST_DIR}" ]] || mkdir -p "${DEMO_SUITE_DEST_DIR}"
cp ${DEVICE_QUERY_BINARY} "${DEMO_SUITE_DEST_DIR}/deviceQuery"
chmod a+x "${DEMO_SUITE_DEST_DIR}/deviceQuery"

# clean up
rm -rf "${DEVICE_QUERY_BINARY}"

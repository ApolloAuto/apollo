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

INSTALL_MODE="$1"; shift

if [[ -z "${INSTALL_MODE}" ]]; then
    INSTALL_MODE="download"
fi

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

TARGET_ARCH="$(uname -m)"

if [[ -z "${CUDA_VERSION}" || "${INSTALL_MODE}" == "download" ]]
then
    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        CUDA_VERSION="11.1.74-1"
    else
        CUDA_VERSION="10.2.89-1"
    fi
fi

MAIN_VER_DOT="${CUDA_VERSION%.*}"
MAIN_VER="${MAIN_VER_DOT//./-}"
CUDA_SAMPLES="cuda-samples"
DEVICE_QUERY_BINARY=
CHECKSUM=
DEMO_SUITE_DEST_DIR="/usr/local/cuda-${MAIN_VER_DOT}/extras/demo_suite"

if [[ -x "${DEMO_SUITE_DEST_DIR}/deviceQuery" ]]; then
    info "Found existing deviceQuery under ${DEMO_SUITE_DEST_DIR}, do nothing."
    exit 0
fi

if [[ "${INSTALL_MODE}" == "download" ]]; then
    # steps to download
    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        DEVICE_QUERY_BINARY="deviceQuery-${MAIN_VER}_${CUDA_VERSION}_amd64.bin"
        CHECKSUM="da573cc68ad5fb227047064d73123a8a966df35be19b68163338b6dc0d576c84"
    elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
        DEVICE_QUERY_BINARY="deviceQuery-${MAIN_VER}_${CUDA_VERSION}_arm64.bin"
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
else
    # steps to build from source
    git clone -b v${MAIN_VER_DOT} --single-branch https://github.com/NVIDIA/${CUDA_SAMPLES}.git ${CUDA_SAMPLES}
    pushd ${CUDA_SAMPLES}/Samples/deviceQuery/
        if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
            make
        elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
            make TARGET_ARCH=aarch64
        else
            error "Support for ${TARGET_ARCH} not ready yet."
            exit 1
        fi

        [[ -d "${DEMO_SUITE_DEST_DIR}" ]] || mkdir -p "${DEMO_SUITE_DEST_DIR}"
        cp deviceQuery "${DEMO_SUITE_DEST_DIR}/"
        chmod a+x "${DEMO_SUITE_DEST_DIR}/deviceQuery"
    popd

    # clean up
    rm -rf "${CUDA_SAMPLES}"
fi

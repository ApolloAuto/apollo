#! /usr/bin/env bash
###############################################################################
# Copyright 2021 The Apollo Authors. All Rights Reserved.
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

set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# if ldconfig -p | grep -q magma ; then
#     warning "Magma already installed, re-installation skipped."
#     exit 0
# fi

: ${INSTALL_MODE:=build}
# : ${APOLLO_DIST:=stable} # re-enable this if differentiation is needed.

GPU_ARCHS=
function determine_gpu_targets() {
    IFS=' ' read -r -a sms <<< "${SUPPORTED_NVIDIA_SMS}"
    local archs=
    for sm in "${sms[@]}"; do
        if [[ -z "${archs}" ]]; then
            archs="sm_${sm//./}"
        else
            archs+=" sm_${sm//./}"
        fi
    done
    GPU_ARCHS="${archs}"
}
determine_gpu_targets

apt_get_update_and_install \
    libopenblas-dev \
    gfortran

TARGET_ARCH="$(uname -m)"
VERSION="2.5.4"

if [[ "${INSTALL_MODE}" == "download" ]]; then
    if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
        CHECKSUM="546f7739109ba6cf93696882d8b18c0e35e68e0c8531ce9f9ca8fa345a1f227c"
        PKG_NAME="magma-${VERSION}-cu111-x86_64.tar.gz"
    else # AArch64
        CHECKSUM="95b9cc9a42e05af3572fe22210230bdbeec023c9481eaeae1f9de051d1171893"
        PKG_NAME="magma-${VERSION}-cu102-aarch64.tar.gz"
    fi
    DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
    tar xzf ${PKG_NAME}
    pushd ${PKG_NAME%.tar.gz}
        mv include/* ${SYSROOT_DIR}/include/
        mv lib/*.so     ${SYSROOT_DIR}/lib/
        [[ -d ${SYSROOT_DIR}/lib/pkgconfig ]] || mkdir -p ${SYSROOT_DIR}/lib/pkgconfig
        install -t ${SYSROOT_DIR}/lib/pkgconfig lib/pkgconfig/magma.pc
    popd
    rm -rf ${PKG_NAME%.tar.gz} ${PKG_NAME}
    ok "Successfully installed magma-${VERSION} in download mode"
else
    PKG_NAME="magma-${VERSION}.tar.gz"
    DOWNLOAD_LINK="http://icl.utk.edu/projectsfiles/magma/downloads/${PKG_NAME}"
    CHECKSUM="7734fb417ae0c367b418dea15096aef2e278a423e527c615aab47f0683683b67"

    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    tar xzf ${PKG_NAME}

    pushd magma-${VERSION}
        mkdir build && cd build
        cmake .. \
            -DBUILD_SHARED_LIBS=ON \
            -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
            -DCMAKE_BUILD_TYPE=Release \
            -DGPU_TARGET="${GPU_ARCHS}"
        # E.g., sm_52 sm_60 sm_61 sm_70 sm_75
        make -j$(nproc)
        make install
    popd
    rm -rf magma-${VERSION} ${PKG_NAME}
fi

info "Successfully built Magma for CUDA SMs=${GPU_ARCHS}"
ldconfig

if [[ -n "${CLEAN_DEPS}" ]]; then
    apt_get_remove gfortran
fi

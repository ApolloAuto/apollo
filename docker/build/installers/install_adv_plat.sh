#!/usr/bin/env bash
###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

MY_MODE="${1:-build}"

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

ARCH="$(uname -m)"

DEST_DIR="${PKGS_DIR}/adv_plat"
[[ -d ${DEST_DIR} ]] || mkdir -p ${DEST_DIR}

#info "Git clone https://github.com/ApolloAuto/apollo-contrib.git"
#git clone https://github.com/ApolloAuto/apollo-contrib.git

PKG_NAME="apollo-contrib-baidu-1.0.tar.gz"
CHECKSUM="cd385dae6d23c6fd70c2c0dcd0ce306241f84a638f50988c6ca52952c304bbec"
DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf ${PKG_NAME}

BAIDU_DIR="apollo-contrib/baidu"
SRC_DIR="${BAIDU_DIR}/src/lib"
OUT_DIR="${BAIDU_DIR}/output"

pushd ${SRC_DIR}
    pushd adv_trigger
        make -j$(nproc)
        make install
        make clean
    popd
    pushd bcan
        make -j$(nproc)
        make install
        make clean
    popd
popd

LINUX_HEADERS="../src/kernel/include/uapi/linux"

pushd ${OUT_DIR}
    cp -r ${LINUX_HEADERS} include/
    rm -rf lib/libadv_*.a

    create_so_symlink lib

    mkdir -p "${DEST_DIR}"
    mv include ${DEST_DIR}
    mv lib ${DEST_DIR}

    echo "${DEST_DIR}/lib" >> "${APOLLO_LD_FILE}"
    ldconfig
popd

rm -rf ${PKG_NAME} apollo-contrib

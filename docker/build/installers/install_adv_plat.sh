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

MY_MODE=$1; shift

DEST_DIR="/usr/local/adv_plat"

. /tmp/installers/installer_base.sh

if [[ "${MY_MODE}" == "download" ]]; then
    PKG_NAME="adv_plat-3.0-x86_64.tar.gz"
    CHECKSUM="1c4a0e205ab2940fc547e5c61b2e181688d4396db2a699f65539add6e10b8150"
    DOWNLOAD_LINK="https://apollohost.example.com/archive/6.0/${PKG_NAME}"

    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    tar xzf adv_plat
    mv adv_plat/include ${DEST_DIR}/include
    mv adv_plat/lib     ${DEST_DIR}/lib

    rm -r ${PKG_NAME} adv_plat
    exit 0
fi

git clone https://github.com/ApolloAuto/apollo-contrib.git
BAIDU_DIR="/apollo/apollo-contrib/baidu"
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

LINUX_HEADERS="${BAIDU_DIR}/src/kernel/include/uapi/linux"

pushd ${OUT_DIR}
    cp -r ${LINUX_HEADERS} include/
    rm -rf lib/libadv_*.a

    create_so_symlink lib

    mkdir -p "${DEST_DIR}"
    mv include ${DEST_DIR}
    mv lib ${DEST_DIR}
popd

rm -rf ${OUT_DIR}


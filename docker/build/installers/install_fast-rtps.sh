#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

TARGET_ARCH="$(uname -m)"

apt_get_update_and_install \
    libasio-dev \
    libtinyxml2-dev

# Note(storypku) & FIXME(all)
# As FastRTPS installer in the master branch doesn't work well, we provide
# prebuilt version here as a workaround. To be removed when ready.
# Maybe the `cyber/transport/rtps` section needs a rewrite using more recent
# FastRTPS implentations, e.g. 2.0.0
#
# Ref: https://github.com/eProsima/Fast-DDS
# Ref: https://github.com/ros2/rmw_fastrtps
DEST_DIR="/usr/local/fast-rtps"

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    PKG_NAME="fast-rtps-1.5.0.prebuilt.x86_64.tar.gz"
    CHECKSUM="ca0534db4f757cb41a9feaebac07a13dd4b63af0a217b2cb456e20b0836bc797"
    DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"

    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    tar xzf ${PKG_NAME}
    mv fast-rtps-1.5.0 "${DEST_DIR}"
    rm -rf ${PKG_NAME}
else # aarch64
    PKG_NAME="fast-rtps-1.5.0.prebuilt.aarch64.tar.gz"
    CHECKSUM="061da391763949e39ed0ac4d0596112818e8692b938aa845d54fac1a1aa550db"
    DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"

    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
    tar xzf ${PKG_NAME}
    mv fast-rtps-1.5.0 "${DEST_DIR}"
    rm -rf ${PKG_NAME}
fi

echo "${DEST_DIR}/lib" >> "${APOLLO_LD_FILE}"
ldconfig
apt_get_remove libasio-dev libtinyxml2-dev

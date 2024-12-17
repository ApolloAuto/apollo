#!/usr/bin/env bash

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd -P
)"
[[ -f "${SCRIPT_DIR}/installer_base.sh" ]] && source ${SCRIPT_DIR}/installer_base.sh

ARCH=$(uname -m)
THREAD_NUM=$(nproc)

# Install gflags.
GFLAGS_VERSION="${GFLAGS_VERSION:-2.2.2}"
GFLAGS_PKG_NAME="${GFLAGS_PKG_NAME:-gflags-${GFLAGS_VERSION}.tar.gz}"
GFLAGS_PKG_CHECKSUM="${GFLAGS_PKG_CHECKSUM:-34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf}"
GFLAGS_PKG_DOWNLOAD_LINK="${GFLAGS_PKG_DOWNLOAD_LINK:-https://github.com/gflags/gflags/archive/v${GFLAGS_VERSION}.tar.gz}"

download_if_not_cached "${GFLAGS_PKG_NAME}" "${GFLAGS_PKG_CHECKSUM}" "${GFLAGS_PKG_DOWNLOAD_LINK}"

GFLAGS_INSTALL_PREFIX="${GFLAGS_INSTALL_PREFIX:-${APOLLO_SYS_INSTALL_PREFIX:-/usr/local}}"
GFLAGS_TMP_ROOT_DIR="${GFLAGS_TMP_ROOT_DIR:-${TMPDIR}/gflags-${GFLAGS_VERSION}}"
GFLAGS_TMP_WORK_DIR="${GFLAGS_TMP_ROOT_DIR}/work"
mkdir -p "${GFLAGS_TMP_WORK_DIR}" && tar xzf "${GFLAGS_PKG_NAME}" -C "${GFLAGS_TMP_WORK_DIR}"

pushd ${GFLAGS_TMP_WORK_DIR}/gflags-${GFLAGS_VERSION}
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
popd

ldconfig

# cleanup
rm -rf ${GFLAGS_PKG_NAME} ${GFLAGS_TMP_ROOT_DIR}

# Install glog which also depends on gflags.

GLOG_VERSION="0.4.0"
GLOG_PKG_NAME="glog-${GLOG_VERSION}.tar.gz"
GLOG_PKG_CHECKSUM="f28359aeba12f30d73d9e4711ef356dc842886968112162bc73002645139c39c"
MODULEDOWNLOAD_LINK="https://github.com/google/glog/archive/v${GLOG_VERSION}.tar.gz"
# https://github.com/google/glog/archive/v0.4.0.tar.gz

download_if_not_cached "${GLOG_PKG_NAME}" "${GLOG_PKG_CHECKSUM}" "${GLOG_PKG_DOWNLOAD_LINK}"
GLOG_INSTALL_PREFIX="${GLOG_INSTALL_PREFIX:-${APOLLO_SYS_INSTALL_PREFIX:-/usr/local}}"
GLOG_TMP_ROOT_DIR="${GLOG_TMP_ROOT_DIR:-${TMPDIR}/gflags-${GLOG_VERSION}}"
GLOG_TMP_WORK_DIR="${GLOG_TMP_ROOT_DIR}/work"
mkdir -p "${GLOG_TMP_WORK_DIR}" && tar xzf "${GLOG_PKG_NAME}" -C "${GLOG_TMP_WORK_DIR}"

tar xzf ${GLOG_PKG_NAME} -C "${GLOG_TMP_WORK_DIR}"

pushd ${GLOG_TMP_WORK_DIR}/glog-${GLOG_VERSION}
mkdir build && cd build
cmake .. \
  -DBUILD_SHARED_LIBS=ON \
  -DWITH_GFLAGS=OFF \
  -DCMAKE_BUILD_TYPE=Release

# if [ "$ARCH" == "aarch64" ]; then
#    ./configure --build=armv8-none-linux --enable-shared
# fi

make -j$(nproc)
make install
popd

ldconfig

# clean up.
rm -fr ${GLOG_PKG_NAME} "${GLOG_TMP_ROOT_DIR}"

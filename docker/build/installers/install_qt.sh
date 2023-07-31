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

# Fail on first error.
set -e

BUILD_TYPE="${1:-download}"; shift
LSB_RELEASE="${1:-18.04}"; shift

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

TARGET_ARCH="$(uname -m)"

apt_get_update_and_install \
    libx11-xcb1 \
    libfreetype6 \
    libdbus-1-3 \
    libfontconfig1 \
    libxkbcommon0   \
    libxkbcommon-x11-0

# Note(storypku)
# 1) libicu6X: required by uic
# 2) libxkbcommonX required by `ldd /usr/local/qt5/plugins/platforms/libqxcb.so`

if [ "${TARGET_ARCH}" = "aarch64" ]; then
    bash ${CURR_DIR}/install_qt5_qtbase.sh "${BUILD_TYPE}" "${LSB_RELEASE}"
    exit 0
fi

QT_VERSION_A=5.12
QT_VERSION_B=5.12.9
QT_VERSION_Z=$(echo "$QT_VERSION_B" | tr -d '.')

QT_INSTALLER=qt-opensource-linux-x64-${QT_VERSION_B}.run
CHECKSUM="63ef2b991d5fea2045b4e7058f86e36868cafae0738f36f8b0a88dc8de64ab2e"
DOWLOAD_LINK=https://download.qt.io/archive/qt/${QT_VERSION_A}/${QT_VERSION_B}/${QT_INSTALLER}

pip3_install cuteci

download_if_not_cached $QT_INSTALLER $CHECKSUM $DOWLOAD_LINK
chmod +x $QT_INSTALLER

MY_DEST_DIR="/usr/local/Qt${QT_VERSION_B}"
cuteci \
    --installer "$PWD/$QT_INSTALLER" \
    install \
    --destdir="$MY_DEST_DIR" \
    --packages "qt.qt5.${QT_VERSION_Z}.gcc_64" \
    --keep-tools

QT5_PATH="/usr/local/qt5"
# Hide qt5 version from end users
ln -s ${MY_DEST_DIR}/${QT_VERSION_B}/gcc_64 "${QT5_PATH}"

echo "${QT5_PATH}/lib" > /etc/ld.so.conf.d/qt.conf
ldconfig

__mytext="""
export QT5_PATH=\"${QT5_PATH}\"
export QT_QPA_PLATFORM_PLUGIN_PATH=\"\${QT5_PATH}/plugins\"
add_to_path \"\${QT5_PATH}/bin\"
"""

echo "${__mytext}" | tee -a "${APOLLO_PROFILE}"

# clean up
rm -f ${QT_INSTALLER}
# Keep License files
rm -rf ${MY_DEST_DIR}/{Docs,Examples,Tools,dist} || true
rm -rf ${MY_DEST_DIR}/MaintenanceTool* || true
rm -rf ${MY_DEST_DIR}/{InstallationLog.txt,installer-changelog} || true
rm -rf ${MY_DEST_DIR}/{components,network}.xml || true

pip3 uninstall -y cuteci

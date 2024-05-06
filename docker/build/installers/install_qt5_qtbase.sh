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

BUILD_TYPE="${1:-download}"; shift
LSB_RELEASE="${1:-18.04}"; shift

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

VERSION="5.12.9"
MAJOR_VERSION="${VERSION%.*}"

QT5_PREFIX="/usr/local/qt5"

if [[ "${BUILD_TYPE}" == "download" ]]; then
    if [[ "$LSB_RELEASE" == "20.04" ]]; then
        error "download method only support build bionic image"
        return -1
    fi
    PKG_NAME="Qt-${VERSION}-linux-arm64.bin.tar.gz"
    CHECKSUM="9361d04678610fe5fddebbbf9bab38d75690d691f3d88f1f2d3eb96a07364945"
    DOWNLOAD_LINK="https://apollo-system.cdn.bcebos.com/archive/6.0/${PKG_NAME}"
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
    tar xzf "${PKG_NAME}" -C /usr/local
    ln -sfnv "Qt-${VERSION}" "${QT5_PREFIX}"
else
    # References:
    # 1) http://www.linuxfromscratch.org/blfs/view/svn/x/qt5.html
    # 2) https://src.fedoraproject.org/rpms/qt5-qtbase/tree/master
    # 3) https://launchpad.net/ubuntu/+source/qtbase-opensource-src/5.12.8+dfsg-0ubuntu1
    apt_get_update_and_install \
        libicu-dev \
        libdbus-1-dev \
        libfontconfig1-dev \
        libfreetype6-dev \
        libgl1-mesa-dev  \
        libharfbuzz-dev \
        libjpeg-dev \
        libpcre3-dev \
        libpng-dev \
        libsqlite3-dev \
        libssl-dev \
        libvulkan-dev \
        libxcb1-dev \
        libexpat1-dev \
        zlib1g-dev \
        libxcb-image0-dev \
        libxcb-keysyms1-dev \
        libxcb-render-util0-dev \
        libxcb-shm0-dev \
        libxcb-util1 \
        libxcb-xinerama0-dev \
        libxcb-xkb-dev \
        libxkbcommon-dev \
        libxkbcommon-x11-dev \
        libx11-* \
        libx11* \
        libxcb-* \
        libxcb* \

    PKG_NAME="qtbase-everywhere-src-${VERSION}.tar.xz"
    CHECKSUM="331dafdd0f3e8623b51bd0da2266e7e7c53aa8e9dc28a8eb6f0b22609c5d337e"
    DOWNLOAD_LINK="http://master.qt.io/archive/qt/${MAJOR_VERSION}/${VERSION}/submodules/${PKG_NAME}"
    download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

    tar xJf ${PKG_NAME}

    mkdir -p "/usr/local/Qt-${VERSION}"

    pushd qtbase-everywhere-src-${VERSION} >/dev/null
        find . -name "*.pr[io]" | xargs sed -i 's/python/&3/'

        pushd src/3rdparty
            [ -d UNUSED ] || mkdir UNUSED
            mv freetype libjpeg libpng zlib sqlite UNUSED/ || true
        popd

        ./configure         \
                -xcb \
                -prefix $QT5_PREFIX                       \
                -sysconfdir /etc/xdg                      \
                -platform linux-g++                       \
                -release                                  \
                -optimized-qmake                          \
                -shared                                   \
                -strip                                    \
                -confirm-license                          \
                -opensource                               \
                -fontconfig                               \
                -dbus-linked                              \
                -openssl-linked                           \
                -system-harfbuzz                          \
                -system-freetype                          \
                -system-sqlite                            \
                -system-libjpeg                           \
                -system-libpng                            \
                -system-zlib                              \
                -nomake examples                          \
                -no-pch                                   \
                -no-rpath                                 \
                -skip qtwebengine

        make -j$(nproc)
        make install

        ln -sfnv "Qt-${VERSION}" "${QT5_PREFIX}"

        # PostInstall
        find $QT5_PREFIX/ -name \*.prl \
            -exec sed -i -e '/^QMAKE_PRL_BUILD_DIR/d' {} \;
        find ${QT5_PREFIX}/lib -name "*.la" \
            -exec rm -f {} \;

    popd >/dev/null
fi

echo "${QT5_PREFIX}/lib" > /etc/ld.so.conf.d/qt.conf
ldconfig

__mytext="""
export QT5_PATH=\"${QT5_PREFIX}\"
export QT_QPA_PLATFORM_PLUGIN_PATH=\"\${QT5_PATH}/plugins\"
add_to_path \"\${QT5_PATH}/bin\"
"""

echo "${__mytext}" | tee -a "${APOLLO_PROFILE}"

if [[ "${BUILD_TYPE}" == "build" ]]; then
    ok "Successfully installed Qt5 qtbase-${VERSION} from src."
    rm -rf qtbase-everywhere-src-${VERSION} ${PKG_NAME}
else
    ok "Successfully pre-built Qt5 qtbase-${VERSION}."
fi

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*

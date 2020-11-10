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

apt_get_update_and_install \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libeigen3-dev \
    liblzma-dev \
    libxml2-dev \
    liblz4-dev \
    libdouble-conversion-dev \
    libsqlite3-dev \
    libglew-dev \
    libtheora-dev \
    libogg-dev \
    libxt-dev \
    libfreetype6-dev \
    libjsoncpp-dev \
    libhdf5-dev

if ldconfig -p | grep -q libvtkCommonCore; then
    info "Found existing VTK installation. Skip re-installing."
    exit 0
fi

TARGET_ARCH="$(uname -m)"

# Note(storypku):
# Although VTK can be installed via apt, build it from source to
#   1) reduce image size
#   2) avoid a lot of dependencies
# RTFM:
# 1) https://src.fedoraproject.org/rpms/vtk/blob/master/f/vtk.spec
# 2) https://vtk.org/Wiki/VTK/Building/Linux

VERSION=8.2.0
PKG_NAME="VTK-8.2.0.tar.gz"
CHECKSUM="34c3dc775261be5e45a8049155f7228b6bd668106c72a3c435d95730d17d57bb"
DOWNLOAD_LINK=https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf ${PKG_NAME}

# Note(storypku): Qt-related features disabled
# VTK_USE_BOOST
pushd VTK-${VERSION}
mkdir build && cd build
cmake .. \
    -DVTK_USE_SYSTEM_LIBRARIES=ON \
    -DVTK_USE_SYSTEM_JPEG=ON \
    -DVTK_USE_SYSTEM_PNG=ON \
    -DVTK_USE_SYSTEM_TIFF=ON \
    -DVTK_USE_SYSTEM_EIGEN=ON \
    -DVTK_USE_SYSTEM_LZMA=ON \
    -DVTK_USE_SYSTEM_ZLIB=ON \
    -DVTK_USE_SYSTEM_LZ4=ON \
    -DVTK_USE_SYSTEM_LIBXML2=ON \
    -DVTK_USE_SYSTEM_EXPAT=ON \
    -DVTK_USE_SYSTEM_LIBPROJ=OFF \
    -DVTK_USE_SYSTEM_SQLITE=ON \
    -DVTK_USE_SYSTEM_PUGIXML=OFF \
    -DVTK_USE_SYSTEM_NETCDF=OFF \
    -DVTK_USE_SYSTEM_GL2PS=OFF \
    -DVTK_USE_SYSTEM_LIBHARU=OFF \
    -DVTK_USE_SYSTEM_JSONCPP=ON \
    -DVTK_Group_Qt=OFF \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
    -DCMAKE_BUILD_TYPE=Release

thread_num="$(nproc)"
if [ "${TARGET_ARCH}" = "aarch64" ]; then
    thread_num=$((thread_num / 2))
fi
make -j${thread_num}
make install
popd

ldconfig

info "Ok. Done installing VTK-${VERSION}"

# clean up
rm -rf ${PKG_NAME} VTK-${VERSION}
apt_get_remove \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libeigen3-dev \
    liblzma-dev \
    libxml2-dev \
    liblz4-dev \
    libdouble-conversion-dev \
    libsqlite3-dev \
    libglew-dev \
    libtheora-dev \
    libogg-dev \
    libxt-dev \
    libfreetype6-dev \
    libjsoncpp-dev \
    libhdf5-dev

# install Runtime-deps for VTK
apt_get_update_and_install \
    libglew2.0 \
    libdouble-conversion1 \
    libxml2 \
    libjsoncpp1 \
    libhdf5-100

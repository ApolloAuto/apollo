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
. /tmp/installers/installer_base.sh

if ldconfig -p | grep -q libvtkCommonCore ; then
    info "Found existing VTK installation. Skip re-installing."
    exit 0
fi

# Note(storypku):
#   Although vtk are shipped with Ubuntu distribution, we decide to build VTK
#   from source here to avoid massive amount of system dependency.

VERSION=8.2.0
PKG_NAME="VTK-8.2.0.tar.gz"
CHECKSUM="34c3dc775261be5e45a8049155f7228b6bd668106c72a3c435d95730d17d57bb"
DOWNLOAD_LINK=https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf ${PKG_NAME}

# https://vtk.org/Wiki/VTK/Building/Linux
# Note(storypku): Qt-related features disabled
pushd VTK-${VERSION}
    mkdir build && cd build
    cmake .. \
        -DVTK_Group_Qt=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
        -DCMAKE_BUILD_TYPE=Release

    make -j${THREAD_NUM}
    make install
popd

ldconfig

info "Ok. Done installing VTK-${VERSION}"

# clean up
rm -rf ${PKG_NAME} VTK-${VERSION}

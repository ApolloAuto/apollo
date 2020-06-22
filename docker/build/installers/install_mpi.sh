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

apt-get -y update && \
    apt-get -y install \
    gfortran-7

# Ref: https://www.open-mpi.org/software/ompi/v4.0/
VERSION=4.0.3
PKG_NAME="openmpi-4.0.3.tar.bz2"
DOWNLOAD_LINK="https://download.open-mpi.org/release/open-mpi/v4.0/openmpi-${VERSION}.tar.bz2"
CHECKSUM="1402feced8c3847b3ab8252165b90f7d1fa28c23b6b2ca4632b6e4971267fd03"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xjf "${PKG_NAME}"
pushd openmpi-${VERSION}
    ./configure --prefix="${SYSROOT_DIR}" --enable-mpi-cxx --with-cuda
    make -j$(nproc)
    make install
popd
rm -rf "${PKG_NAME}" "openmpi-${VERSION}"

ldconfig

info "Done installing openmpi-${VERSION}"

# openmpi @cuda
# MPICH
# Ref: https://www.mpich.org
# Inter MPI
# Ref: https://software.intel.com/content/www/us/en/develop/tools/mpi-library.html

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

apt_get_update_and_install mpi-default-dev mpi-default-bin libopenmpi-dev

exit 0

WORKHORSE="$1"
if [ -z "${WORKHORSE}" ]; then
    WORKHORSE="cpu"
fi

apt_get_update_and_install gfortran libhwloc-dev hwloc-nox libevent-dev

# Ref: https://www.open-mpi.org/software/ompi/v4.0/
VERSION="4.0.4"
PKG_NAME="openmpi-${VERSION}.tar.bz2"
CHECKSUM="47e24eb2223fe5d24438658958a313b6b7a55bb281563542e1afc9dec4a31ac4"
DOWNLOAD_LINK="https://download.open-mpi.org/release/open-mpi/v4.0/openmpi-${VERSION}.tar.bz2"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

GPU_OPTIONS=""
if [ "${WORKHORSE}" = "gpu" ]; then
    GPU_OPTIONS="--with-cuda"
else
    GPU_OPTIONS="--with-cuda=no"
fi
TARGET_ARCH="$(uname -m)"

# --target "${TARGET_ARCH}" \
tar xjf "${PKG_NAME}"
pushd openmpi-${VERSION}
    ./configure \
        --prefix="${SYSROOT_DIR}" \
        --build "${TARGET_ARCH}" \
        --disable-silent-rules \
        --enable-builtin-atomics \
        --enable-mpi-cxx \
        --with-pic \
        --enable-mpi1-compatibility \
        --with-hwloc=/usr \
        --with-libevent=external \
        "${GPU_OPTIONS}"

    make -j$(nproc)
    make install
popd
rm -rf "${PKG_NAME}" "openmpi-${VERSION}"

ldconfig

apt-get clean && apt-get -y purge --autoremove gfortran
info "Done installing openmpi-${VERSION}"

# openmpi @cuda
# MPICH
# Ref: https://www.mpich.org
# Inter MPI
# Ref: https://software.intel.com/content/www/us/en/develop/tools/mpi-library.html

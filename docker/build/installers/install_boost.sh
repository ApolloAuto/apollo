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

if ldconfig -p | grep -q "libboost_system.so" ; then
    info "Found existing Boost installation. Reinstallation skipped."
    exit 0
fi

# PreReq for Unicode support for Boost.Regex
#    icu-devtools \
#    libicu-dev
apt_get_update_and_install \
    liblzma-dev \
    libbz2-dev \
    libzstd-dev

# Ref: https://www.boost.org/
VERSION="1_73_0"
PKG_NAME="boost_1_73_0.tar.bz2"
DOWNLOAD_LINK="https://dl.bintray.com/boostorg/release/1.73.0/source/boost_1_73_0.tar.bz2"
CHECKSUM="4eb3b8d442b426dc35346235c8733b5ae35ba431690e38c6a8263dce9fcbb402"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xjf "${PKG_NAME}"

py3_ver="$(py3_version)"

# Ref: https://www.boost.org/doc/libs/1_73_0/doc/html/mpi/getting_started.html
pushd "boost_${VERSION}"
    # A) For mpi built from source
    #  echo "using mpi : ${SYSROOT_DIR}/bin/mpicc ;" > user-config.jam
    # B) For mpi installed via apt
    # echo "using mpi ;" > user-config.jam
    ./bootstrap.sh \
        --with-python-version=${py3_ver} \
        --prefix="${SYSROOT_DIR}" \
        --without-icu

    ./b2 -d+2 -q -j$(nproc) \
        --without-graph_parallel \
        --without-mpi \
        variant=release \
        link=shared \
        threading=multi \
        install
        #--user-config=user-config.jam
popd
ldconfig

# Clean up
rm -rf "boost_${VERSION}" "${PKG_NAME}"
apt_get_remove  \
    liblzma-dev \
    libbz2-dev \
    libzstd-dev


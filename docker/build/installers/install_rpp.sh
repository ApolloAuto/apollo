#!/usr/bin/env bash

###############################################################################
# Copyright 2022 The Apollo Authors. All Rights Reserved.
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

INSTALL_MODE="$1"; shift

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

APOLLO_SYSROOT_INC="/opt/apollo/sysroot/include"

# install half.hpp required dependency
HALF_HPP="half.hpp"
DST_HALF_DIR="/usr/local/include"
DST_HALF="${DST_HALF_DIR}/${HALF_HPP}"
if [ ! -f "${DST_HALF}" ]; then
    VERSION="2.2.0"
    PKG_NAME="half-${VERSION}.zip"
    wget https://sourceforge.net/projects/half/files/half/"${VERSION}"/"${PKG_NAME}"
    unzip "$PKG_NAME" -d "${PKG_NAME%.zip}"
    mkdir -p $DST_HALF_DIR
    mv "${PKG_NAME%.zip}/include/${HALF_HPP}" $DST_HALF_DIR
    ldconfig
    rm "${PKG_NAME}"
    rm -fr "${PKG_NAME%.zip}"
    ok "Successfully installed half ${VERSION}"
else
    info "half is already installed here: ${DST_HALF}"
    info "To reinstall half delete ${DST_HALF} first"
fi

# install rpp
RPP_DIR=rpp/.git
if [ -d $RPP_DIR ]; then
    rm -fr rpp
fi
RPP_SO="libamd_rpp.so"
DST_RPP_DIR="/opt/rocm/rpp/lib"
DST_RPP_SO="${DST_RPP_DIR}/${RPP_SO}"
if [ ! -f "${DST_RPP_SO}" ]; then
    git clone https://github.com/GPUOpen-ProfessionalCompute-Libraries/rpp
    cd rpp
    info ""
    git reset --hard 27284078458fbfa685f11083315394f3a4cd952f
    info ""
    cd ..
    pushd rpp
        mkdir -p build && cd build
        cmake -DBACKEND=HIP -DCMAKE_CXX_FLAGS="-I${DST_HALF_DIR}" -DINCLUDE_LIST="${APOLLO_SYSROOT_INC}" ..
        make -j$(nproc)
        make install
    popd
    cp -rLfs "/opt/rocm/rpp/include/." "/opt/rocm/include/"
    cp -rLfs "/opt/rocm/rpp/lib/." "/opt/rocm/lib/"
    rm -fr rpp
    ok "Successfully installed RPP"
else
    info "RPP is already installed here: ${DST_RPP_DIR}"
    info "To reinstall RPP delete ${DST_RPP_SO} first"
fi

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

INSTALL_MODE="$1"; shift

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

# install half.hpp required dependency
VERSION="2.2.0"
PKG_NAME="half-${VERSION}.zip"
wget https://sourceforge.net/projects/half/files/half/"${VERSION}"/"${PKG_NAME}"
unzip "$PKG_NAME" -d "${PKG_NAME%.zip}"
mv "${PKG_NAME%.zip}"/include/half.hpp /usr/local/include/
ldconfig
rm "${PKG_NAME}"
rm -fr "${PKG_NAME%.zip}"


# install rpp
git clone https://github.com/GPUOpen-ProfessionalCompute-Libraries/rpp
pushd rpp
    mkdir -p build && cd build
    cmake -DBACKEND=HIP -DINCLUDE_LIST="/opt/apollo/sysroot/include" ..
    make -j$(nproc)
    make install
popd
cp -rLf "/opt/rocm/rpp/include/." "/opt/rocm/include/"
cp -rLf "/opt/rocm/rpp/lib/." "/opt/rocm/lib/"

rm -fr rpp
ok "Successfully installed RPP"

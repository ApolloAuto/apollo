#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

geo="${1:-us}"

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

VERSION="6.5.1"
NODE_VERSION="12.18.1"
PKG_NAME="n-${VERSION}.tar.gz"
CHECKSUM="5833f15893b9951a9ed59487e87b6c181d96b83a525846255872c4f92f0d25dd"
DOWNLOAD_LINK="https://github.com/tj/n/archive/v${VERSION}.tar.gz"
download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"

info "Install Node for $geo ..."

if [[ "${geo}" == "cn" ]]; then
    export N_NODE_MIRROR=https://npm.taobao.org/mirrors/node
fi

pushd n-${VERSION}
    make install
    n ${NODE_VERSION}
popd

rm -fr "${PKG_NAME}" "n-${VERSION}"

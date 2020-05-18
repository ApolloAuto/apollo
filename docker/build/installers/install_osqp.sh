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

cd "$(dirname "${BASH_SOURCE[0]}")"
. /tmp/installers/installer_base.sh

ARCH=$(uname -m)

if [ "$ARCH" == "x86_64" ]; then
  PKG_NAME="osqp-contrib-master.zip"
  CHECKSUM="3e431342bbe3bd578f1ef768e8dff1f89d2809d8195aa15f8ed72628ca92f6d8"
  DOWNLOAD_LINK="https://github.com/ApolloAuto/osqp-contrib/archive/master.zip"

  download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

  unzip ${PKG_NAME}
  pushd osqp-contrib-master
    mkdir -p /usr/local/include/osqp
    cp -r osqp/include /usr/local/include/osqp/
    cp osqp/libosqp.so /usr/local/lib/
  popd
  rm -rf libosqp-contrib-master ${PKG_NAME}
elif [ "$ARCH" == "aarch64" ]; then
  BUILD=$1
  shift
  if [ "$BUILD" == "build" ]; then
    wget https://github.com/oxfordcontrol/osqp/archive/v0.4.1.zip
    unzip v0.4.1.zip
    pushd osqp-0.4.1/lin_sys/direct/qdldl/qdldl_sources/
    wget https://apollocache.blob.core.windows.net/apollo-cache/qdldl.zip
    unzip qdldl.zip
    cd ../../../../
    mkdir build && cd build
    cmake ../
    make
    make install
    popd
  else
    wget https://apollocache.blob.core.windows.net/apollo-cache/osqp.zip
    unzip osqp.zip
    pushd osqp
    mkdir -p /usr/local/include/osqp/include
    cp -r include/ /usr/local/include/osqp/
    cp lib/libosqp.so /usr/local/lib/
    popd
  fi
else
    echo "Architecture $ARCH not supported"
fi

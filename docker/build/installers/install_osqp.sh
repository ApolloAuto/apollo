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

ARCH=$(uname -m)

if [ "$ARCH" == "x86_64" ]; then
  wget https://github.com/ApolloAuto/osqp-contrib/archive/master.zip
  unzip master.zip
  pushd osqp-contrib-master
  mkdir -p /usr/local/include/osqp
  cp -r osqp/include /usr/local/include/osqp/
  cp osqp/libosqp.so /usr/local/lib/
  popd
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
    echo "not support $ARCH"
fi

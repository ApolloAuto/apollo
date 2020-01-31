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

git clone --single-branch --branch apollo --depth 1 https://github.com/ApolloAuto/caffe.git
pushd caffe

mkdir build
cd build
cmake \
    -DCUDA_ARCH_NAME=All \
    -DBUILD_docs=OFF \
    -DBUILD_python_layer=OFF \
    -DUSE_OPENCV=OFF \
    -DUSE_LEVELDB=OFF \
    -DUSE_LMDB=OFF \
    ..
make -j8
make install

popd
rm -fr caffe

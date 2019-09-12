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

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

# install Foonathan Memory
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
cmake --build . --target install

#install fast rtps 1.9.0
git clone https://github.com/eProsima/Fast-RTPS.git
pushd Fast-RTPS
git submodule init
git submodule update
mkdir -p build && cd build
cmake -DEPROSIMA_BUILD=ON -DCMAKE_INSTALL_PREFIX=/usr/local/fast-rtps ../
make -j 8
make install
popd

rm -rf foonathan_memory_vendor/ Fast-RTPS/

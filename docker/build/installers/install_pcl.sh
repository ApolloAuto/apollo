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
set -x


cd "$(dirname "${BASH_SOURCE[0]}")"

wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz

rm -rf pcl-pcl-1.9.1/

tar xzvf pcl-1.9.1.tar.gz

cd pcl-pcl-1.9.1/

mkdir -p build
cd build

export LD_LIBRARY_PATH=/usr/local/apollo/boost/lib:$LD_LIBRARY_PATH
export BOOST_ROOT=/usr/local/apollo/boost/

cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8

sudo make install

# Clean up.
rm -fr pcl-1.9.1.tar.gz pcl-pcl-1.9.1/

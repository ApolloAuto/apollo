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

apt-get update -y && apt-get install -y \
	libgtest-dev

# Install gflags.
wget http://www.baiduapollo.club/apollo-docker/v2.2.0.tar.gz
tar xzf v2.2.0.tar.gz
mkdir gflags-2.2.0/build
pushd gflags-2.2.0/build
#CXXFLAGS="-fPIC" cmake ..
#maybe should use the command below
cmake CXXFLAGS="-fPIC" -DBUILD_SHARED_LIBS=ON -DBUILD_STATIC_LIBS=ON ../
make -j8
make install
popd

# Install glog which also depends on gflags.
apt-get install -y autoconf libtool
wget https://github.com/google/glog/archive/master.zip
unzip master.zip
cd glog-master/
./autogen.sh
./configure CPPFLAGS="-fPIC"
make
make install

cd ../

# Clean up.
rm -fr /usr/local/lib/libglog.so*
rm -fr master.zip glog-master/  gflags-2.2.0 v2.2.0.tar.gz

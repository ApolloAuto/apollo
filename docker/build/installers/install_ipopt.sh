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

add-apt-repository universe && apt-get -y update
apt-get install -y libblas-dev liblapack-dev gfortran

wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.11.zip -O Ipopt-3.12.11.zip
unzip Ipopt-3.12.11.zip

pushd Ipopt-3.12.11/ThirdParty/Mumps
bash get.Mumps
popd

pushd Ipopt-3.12.11
./configure --build=x86_64 --disable-shared ADD_CXXFLAGS="-fPIC" ADD_CFLAGS="-fPIC" ADD_FFLAGS="-fPIC"
make -j8 all
make install
mkdir -p /usr/local/ipopt
cp -r include /usr/local/ipopt/ && cp -r lib /usr/local/ipopt/
popd

# Clean up.
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -fr Ipopt-3.12.11.zip Ipopt-3.12.11

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

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

wget https://github.com/gcc-mirror/gcc/archive/gcc-8_2_0-release.tar.gz
tar -vxzf gcc-8_2_0-release.tar.gz
pushd gcc-gcc-8_2_0-release/
./contrib/download_prerequisites
mkdir objdir && cd objdir/
../configure --prefix=/usr --enable-languages=c,c++,fortran,go --disable-multilib
make -j 8
make install
popd

update-alternatives --install /usr/bin/gcc gcc /usr/bin/x86_64-pc-linux-gnu-gcc 10
update-alternatives --install /usr/bin/g++ g++ /usr/bin/x86_64-pc-linux-gnu-g++ 10
update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 20
update-alternatives --set cc /usr/bin/gcc
update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 20
update-alternatives --set c++ /usr/bin/g++

# Clean up.
rm -fr gcc-8_2_0-release.tar.gz gcc-gcc-8_2_0-release/

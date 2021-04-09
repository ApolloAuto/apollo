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

wget https://github.com/startcode/qp-oases/archive/v3.2.1-1.tar.gz
tar xzf v3.2.1-1.tar.gz

pushd qp-oases-3.2.1-1
mkdir bin
make -j8 CPPFLAGS="-Wall -pedantic -Wshadow -Wfloat-equal -O3 -Wconversion \
                   -Wsign-conversion -fPIC -DLINUX -DSOLVER_NONE \
                   -D__NO_COPYRIGHT__"
cp bin/libqpOASES.so /usr/local/lib
cp -r include/* /usr/local/include
popd

# Clean up.
rm -fr v3.2.1-1.tar.gz qp-oases-3.2.1-1

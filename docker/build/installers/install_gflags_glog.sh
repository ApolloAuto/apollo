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

# Install gflags.
wget https://github.com/gflags/gflags/archive/v2.2.0.tar.gz
tar xzf v2.2.0.tar.gz
mkdir gflags-2.2.0/build
pushd gflags-2.2.0/build
CXXFLAGS="-fPIC" cmake ..
make -j8
make install
popd

# Install glog which also depends on gflags.
wget https://github.com/google/glog/archive/v0.3.5.tar.gz
tar xzf v0.3.5.tar.gz
pushd glog-0.3.5
./configure
make -j8 CXXFLAGS='-Wno-sign-compare -Wno-unused-local-typedefs -D_START_GOOGLE_NAMESPACE_="namespace google {" -D_END_GOOGLE_NAMESPACE_="}" -DGOOGLE_NAMESPACE="google" -DHAVE_PTHREAD -DHAVE_SYS_UTSNAME_H -DHAVE_SYS_SYSCALL_H -DHAVE_SYS_TIME_H -DHAVE_STDINT_H -DHAVE_STRING_H -DHAVE_PREAD -DHAVE_FCNTL -DHAVE_SYS_TYPES_H -DHAVE_SYSLOG_H -DHAVE_LIB_GFLAGS -DHAVE_UNISTD_H'
make install
popd

# Clean up.
rm -fr /usr/local/lib/libglog.so*
rm -fr v2.2.0.tar.gz gflags-2.2.0 v0.3.5.tar.gz glog-0.3.5

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
  wget https://github.com/ApolloAuto/osqp-contrib/archive/master.zip
  unzip master.zip
  pushd osqp-contrib-master
    mv osqp/include ${SYSROOT_DIR}/include/osqp
    mv osqp/libosqp.so ${SYSROOT_DIR}/lib
  popd
elif [ "$ARCH" == "aarch64" ]; then
  error "Oops, osqp for aarch64 is broken. You can create an issue here: "
  error "  https://github.com/ApolloAuto/apollo/issues"
  exit 1
fi

ldconfig

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

MACHINE_ARCH=$(uname -m)

mkdir -p /home/libs
cd /home/libs

# Bazel rules.
wget -O bazel-federation-0.0.1.tar.gz \
    https://github.com/bazelbuild/bazel-federation/archive/0.0.1.tar.gz
wget -O rules_proto-97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz \
    https://github.com/bazelbuild/rules_proto/archive/97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz
wget -O rules_boost-9f9fb8b2f0213989247c9d5c0e814a8451d18d7f.tar.gz \
    https://github.com/nelhage/rules_boost/archive/9f9fb8b2f0213989247c9d5c0e814a8451d18d7f.tar.gz

# Libraries.
wget -O abseil-cpp-20190808.tar.gz \
    https://github.com/abseil/abseil-cpp/archive/20190808.tar.gz
wget -O ad-rss-lib-1.1.0.tar.gz \
    https://github.com/intel/ad-rss-lib/archive/v1.1.0.tar.gz
wget -O civetweb-1.11.tar.gz \
    https://github.com/civetweb/civetweb/archive/v1.11.tar.gz
wget -O eigen-3.2.10.tar.gz \
    https://bitbucket.org/eigen/eigen/get/3.2.10.tar.gz
wget -O gflags-2.2.2.tar.gz \
    https://github.com/gflags/gflags/archive/v2.2.2.tar.gz
wget -O glog-0.4.0.tar.gz \
    https://github.com/google/glog/archive/v0.4.0.tar.gz
wget -O googletest-release-1.10.0.tar.gz \
    https://github.com/google/googletest/archive/release-1.10.0.tar.gz
wget -O grpc-1.26.0.tar.gz \
    https://github.com/grpc/grpc/archive/v1.26.0.tar.gz
wget -O proj.4-4.9.3.zip \
    https://github.com/OSGeo/proj.4/archive/4.9.3.zip
wget -O qp-oases-3.2.1-1.zip \
    https://github.com/ApolloAuto/qp-oases/archive/v3.2.1-1.zip
wget -O styleguide-159b4c81bbca97a9ca00f1195a37174388398a67.tar.gz \
    https://github.com/google/styleguide/archive/159b4c81bbca97a9ca00f1195a37174388398a67.tar.gz
wget -O tinyxml2-5.0.1.zip \
    https://github.com/leethomason/tinyxml2/archive/5.0.1.zip
wget -O yaml-cpp-587b24e2eedea1afa21d79419008ca5f7bda3bf4.tar.gz \
    https://github.com/jbeder/yaml-cpp/archive/587b24e2eedea1afa21d79419008ca5f7bda3bf4.tar.gz

# TODO: Align the layout of these two packages.
if [ "$MACHINE_ARCH" == 'x86_64' ]; then
  wget -O osqp-0.4.1.zip https://github.com/ApolloAuto/osqp-contrib/archive/master.zip
elif [ "$MACHINE_ARCH" == 'aarch64' ]; then
  wget -O osqp-0.4.1.zip https://apollocache.blob.core.windows.net/apollo-cache/osqp.zip
else
  echo "Unknown machine architecture $MACHINE_ARCH"
  exit 1
fi

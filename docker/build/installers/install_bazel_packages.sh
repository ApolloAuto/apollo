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

mkdir -p /home/tmp
cd /home/tmp

wget -O opencv-2.4.13.2.zip \
    https://github.com/opencv/opencv/archive/2.4.13.2.zip
wget -O googletest-release-1.8.0.tar.gz \
    https://github.com/google/googletest/archive/release-1.8.0.tar.gz
wget -O gflags-2.2.0.tar.gz \
    https://github.com/gflags/gflags/archive/v2.2.0.tar.gz
wget -O glog-0.3.5.tar.gz \
    https://github.com/google/glog/archive/v0.3.5.tar.gz
wget -O benchmark-1.1.0.tar.gz \
    https://github.com/google/benchmark/archive/v1.1.0.tar.gz
wget -O eigen-3.2.10.tar.gz \
    https://bitbucket.org/eigen/eigen/get/3.2.10.tar.gz
wget -O civetweb-1.10.tar.gz \
    https://github.com/civetweb/civetweb/archive/v1.10.tar.gz
wget -O curlpp-0.8.1.tar.gz \
    https://github.com/jpbarrette/curlpp/archive/v0.8.1.tar.gz
wget -O yaml-cpp-0.5.3.zip \
    https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.5.3.zip
wget -O qp-oases-3.2.1-1.zip \
    https://github.com/startcode/qp-oases/archive/v3.2.1-1.zip
wget -O proj.4-4.9.3.zip \
    https://github.com/OSGeo/proj.4/archive/4.9.3.zip
wget -O tinyxml2-5.0.1.zip \
    https://github.com/leethomason/tinyxml2/archive/5.0.1.zip
wget -O protobuf-3.3.0.tar.gz \
    https://github.com/google/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.tar.gz

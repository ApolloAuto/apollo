#!/usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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

VERSION="7.0.0.11"
PKG_NAME="TensorRT-${VERSION}.Ubuntu-18.04.x86_64-gnu.cuda-10.0.cudnn7.6.tar.gz"

# Note(storypku)
# Download the TensorRT7 tarball from nvidia.com, and start the http
# server right at the archive directory:
# python3 -m http.server 8081

DOWNLOAD_LINK="http://my_host_ip:8081/${PKG_NAME}"

wget "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"

pushd /usr/lib/x86_64-linux-gnu
rm -rf libnvcaffe_parser* libnvinfer* libnvonnxparser.so* libnvparsers*
popd

rm -rf /usr/include/tensorrt

pushd TensorRT-${VERSION}
    mv include /usr/include/tensorrt
    mv lib/* /usr/lib/x86_64-linux-gnu/
popd

rm -rf ${PKG_NAME} TensorRT-${VERSION}

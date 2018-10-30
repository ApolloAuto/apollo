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
    libatlas-base-dev \
    libflann-dev \
    libhdf5-serial-dev \
    libicu-dev \
    liblmdb-dev \
    libopencv-dev \
    libqhull-dev \
    libsnappy-dev \
    libvtk5-dev \
    libvtk5-qt4-dev \
    mpi-default-dev
#libopenblas-dev
#libopenni-dev

#Install openblas via sourcecode
wget http://123.57.58.164/apollo-docker_no/openblas-0.2.18.tar.gz
tar zxvf openblas-0.2.18.tar.gz
mkdir /usr/include/openblas
cp openblas-0.2.18/include/* /usr/include/openblas/
cp -r -d openblas-0.2.18/lib/* /usr/lib/
rm -rf openblas-0.2.18 openblas-0.2.18.tar.gz


wget http://123.57.58.164/apollo-docker/caffe_aarch64.tar.gz
tar xzf caffe_aarch64.tar.gz
mv caffe_aarch64/output-GPU/include/caffe /usr/include/
mv caffe_aarch64/output-GPU/lib/* /usr/lib/aarch64-linux-gnu/

# Clean up.
rm -fr caffe_aarch64.tar.gz caffe_aarch64

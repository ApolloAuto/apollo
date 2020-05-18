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

. /tmp/installers/installer_base.sh

#Install the TensorRT package that fits your particular needs.
#For only running TensorRT C++ applications:
#sudo apt-get install libnvinfer7 libnvonnxparsers7 libnvparsers7 libnvinfer-plugin7
#For also building TensorRT C++ applications:
#sudo apt-get install libnvinfer-dev libnvonnxparsers-dev
# libnvparsers-dev libnvinfer-plugin-dev
#For running TensorRT Python applications:
#sudo apt-get install python-libnvinfer python3-libnvinfer

apt-get -y update && \
    apt-get -y install \
	libnvinfer7 \
	libnvonnxparsers7 \
	libnvparsers7 \
	libnvinfer-plugin7 \
    libnvinfer-dev \
    libnvonnxparsers-dev \
    libnvparsers-dev \
    libnvinfer-plugin-dev

# Make caffe-1.0 compilation pass
CUDNN_HEADER_DIR="/usr/include/$(uname -m)-linux-gnu"
[[ -e "${CUDNN_HEADER_DIR}/cudnn.h" ]] || \
    ln -s "${CUDNN_HEADER_DIR}/cudnn_v7.h" "${CUDNN_HEADER_DIR}/cudnn.h"

# disable nvidia apt source to speed to build process
sed -i 's/^/# /g' /etc/apt/sources.list.d/nvidia-ml.list

info "nVidia machine learning repo disabled after successful TensorRT-7 installation"
info "$(cat /etc/apt/sources.list.d/nvidia-ml.list)"

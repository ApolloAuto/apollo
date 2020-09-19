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
. ./installer_base.sh

ARCH="$(uname -m)"

if [ "${ARCH}" = "aarch64" ]; then
    CUDA_VER="10.2"
    TRT_VER1="7.1.0-1"
    MAJOR="${TRT_VER1%%.*}"
    TRT_VERSION="${TRT_VER1}+cuda${CUDA_VER}"

    TRT_PKGS="\
libnvinfer${MAJOR}_${TRT_VERSION}_arm64.deb \
libnvinfer-bin_${TRT_VERSION}_arm64.deb \
libnvinfer-dev_${TRT_VERSION}_arm64.deb \
libnvinfer-plugin${MAJOR}_${TRT_VERSION}_arm64.deb  \
libnvinfer-plugin-dev_${TRT_VERSION}_arm64.deb  \
libnvonnxparsers${MAJOR}_${TRT_VERSION}_arm64.deb   \
libnvonnxparsers-dev_${TRT_VERSION}_arm64.deb \
libnvparsers${MAJOR}_${TRT_VERSION}_arm64.deb \
libnvparsers-dev_${TRT_VERSION}_arm64.deb \
"

    # tensorrt_7.1.0.16-1+cuda10.2_arm64.deb
    # libnvinfer-doc_${TRT_VERSION}_all.deb
    # libnvinfer-samples_${TRT_VERSION}_all.deb

    for pkg in ${TRT_PKGS}; do
        info "Downloading ${LOCAL_HTTP_ADDR}/${pkg}"
        wget "${LOCAL_HTTP_ADDR}/${pkg}" >/dev/null
    done

    dpkg -i ${TRT_PKGS}

    info "Successfully installed TensorRT ${MAJOR}"

    rm -rf ${TRT_PKGS}
    apt-get clean
    exit 0
fi

#Install the TensorRT package that fits your particular needs.
#For only running TensorRT C++ applications:
#sudo apt-get install libnvinfer7 libnvonnxparsers7 libnvparsers7 libnvinfer-plugin7
#For also building TensorRT C++ applications:
#sudo apt-get install libnvinfer-dev libnvonnxparsers-dev
# libnvparsers-dev libnvinfer-plugin-dev
#For running TensorRT Python applications:
#sudo apt-get install python-libnvinfer python3-libnvinfer

fixed_version=true

if ${fixed_version}; then
    VERSION="7.0.0-1+cuda10.2"
    apt_get_update_and_install \
            libnvinfer7="${VERSION}" \
            libnvonnxparsers7="${VERSION}" \
            libnvparsers7="${VERSION}" \
            libnvinfer-plugin7="${VERSION}" \
            libnvinfer-dev="${VERSION}" \
            libnvonnxparsers-dev="${VERSION}" \
            libnvparsers-dev="${VERSION}" \
            libnvinfer-plugin-dev="${VERSION}"
else
    apt_get_update_and_install \
            libnvinfer7 \
            libnvonnxparsers7 \
            libnvparsers7 \
            libnvinfer-plugin7 \
            libnvinfer-dev \
            libnvonnxparsers-dev \
            libnvparsers-dev \
            libnvinfer-plugin-dev
fi

# FIXME(all):
# Previously soft sym-linked for successful caffe-1.0 compilation.
#   Now that caffe-1.0 is retired, do we still need this?
# Minor changes required:
# 1) cudnn major version hard-code fix: v7,v8,...
# 2) move to cudnn installer section

CUDNN_HEADER_DIR="/usr/include/$(uname -m)-linux-gnu"
[ -e "${CUDNN_HEADER_DIR}/cudnn.h" ] ||
    ln -s "${CUDNN_HEADER_DIR}/cudnn_v7.h" "${CUDNN_HEADER_DIR}/cudnn.h"

# Disable nvidia apt sources.list settings to speed up build process
rm -f /etc/apt/sources.list.d/nvidia-ml.list
rm -f /etc/apt/sources.list.d/cuda.list

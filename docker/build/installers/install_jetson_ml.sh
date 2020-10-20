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

# Note(storypku):
# nvidia-ml-for-jetson installer is expected to run without installer_base.sh

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

# PKG_DOWNLOAD_ADDR="http://172.17.0.1:8388"
PKG_DOWNLOAD_ADDR="https://apollo-system.cdn.bcebos.com/archive/6.0"

apt-get -y update && \
    apt-get -y install --no-install-recommends \
    gnupg2 \
    ca-certificates

##============== Install CUDA ===============##

CUDA_VER_APT="10-2"
CUDA_VERSION="10.2.89"

CUDA_REPO="cuda-repo-l4t-10-2-local-10.2.89_1.0-1_arm64.deb"

wget -O "${CUDA_REPO}" "${PKG_DOWNLOAD_ADDR}/${CUDA_REPO}" >/dev/null \
	&& dpkg -i "${CUDA_REPO}" \
	&& rm -rf "${CUDA_REPO}" \
	&& apt-key add "/var/cuda-repo-${CUDA_VER_APT}-local-${CUDA_VERSION}/7fa2af80.pub"

apt-get -y update &&    \
    apt-get -y install --no-install-recommends \
    cuda-cudart-${CUDA_VER_APT}            \
    cuda-license-${CUDA_VER_APT}           \
    cuda-nvml-dev-${CUDA_VER_APT}          \
    cuda-command-line-tools-${CUDA_VER_APT} \
    cuda-libraries-${CUDA_VER_APT} \
    cuda-libraries-dev-${CUDA_VER_APT} \
    cuda-minimal-build-${CUDA_VER_APT} \
    cuda-cupti-${CUDA_VER_APT} \
    cuda-cupti-dev-${CUDA_VER_APT} \
    libcublas10 \
    libcublas-dev \
    cuda-npp-${CUDA_VER_APT} \
    cuda-npp-dev-${CUDA_VER_APT}

echo "Successfully installed CUDA  ${CUDA_VERSION}"

CUDA_VER="${CUDA_VERSION%.*}"

##================ Install cuDNN =======================##

cuDNN_VERSION="8.0.0.180"
cuDNN_MAJOR="${cuDNN_VERSION%%.*}"
cuDNN_VER_APT="${cuDNN_VERSION}-1+cuda${CUDA_VER}"

cuDNN_PKGS=(
    libcudnn${cuDNN_MAJOR}_${cuDNN_VER_APT}_arm64.deb
    libcudnn${cuDNN_MAJOR}-dev_${cuDNN_VER_APT}_arm64.deb
)

for pkg in "${cuDNN_PKGS[@]}"; do
    wget "${PKG_DOWNLOAD_ADDR}/${pkg}" >/dev/null
done

dpkg -i ${cuDNN_PKGS[*]} && rm -rf ${cuDNN_PKGS[*]}


echo "Delete static cuDNN libraries..."
find /usr/lib/$(uname -m)-linux-gnu -name "libcudnn_*.a" -delete -print

echo "Successfully installed cuDNN${cuDNN_MAJOR}"

##=============== Install TensorRT ================##

TRT_VERSION="7.1.3"
TRT_MAJOR="${TRT_VERSION%%.*}"
TRT_VER_APT="${TRT_VERSION}-1+cuda${CUDA_VER}"

TRT_PKGS=(
    libnvinfer${TRT_MAJOR}_${TRT_VER_APT}_arm64.deb
    libnvinfer-dev_${TRT_VER_APT}_arm64.deb
    libnvinfer-plugin${TRT_MAJOR}_${TRT_VER_APT}_arm64.deb
    libnvinfer-plugin-dev_${TRT_VER_APT}_arm64.deb
    libnvonnxparsers${TRT_MAJOR}_${TRT_VER_APT}_arm64.deb
    libnvonnxparsers-dev_${TRT_VER_APT}_arm64.deb
    libnvparsers${TRT_MAJOR}_${TRT_VER_APT}_arm64.deb
    libnvparsers-dev_${TRT_VER_APT}_arm64.deb
)

for pkg in "${TRT_PKGS[@]}"; do
    wget "${PKG_DOWNLOAD_ADDR}/${pkg}" >/dev/null
done

dpkg -i ${TRT_PKGS[*]} && rm -rf ${TRT_PKGS[*]}

echo "Successfully installed TensorRT ${TRT_VERSION}"

##============== Cleanup ================##

# Kick the ladder and cleanup
apt-get purge --autoremove -y \
    gnupg2 \
    ca-certificates \
    "cuda-repo-l4t-${CUDA_VER_APT}-local-${CUDA_VERSION}"

apt-get clean && \
    rm -rf /var/lib/apt/lists/*

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
# Please start local http cache server at port 8388 before run.
## Package Listing:
# cuda-repo-l4t-10-2-local-10.2.89_1.0-1_arm64.deb
# libcudnn8_8.0.0.145-1+cuda10.2_arm64.deb
# libcudnn8-dev_8.0.0.145-1+cuda10.2_arm64.deb

# Fail on first error.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

LOCAL_HTTP_ADDR="http://172.17.0.1:8388"

apt-get -y update && \
    apt-get -y install --no-install-recommends \
    gnupg2 \
    ca-certificates

##===== Install CUDA 10.2 =====##
VERSION_1="10-2"
VERSION_2="10.2.89"

CUDA_REPO_NAME="cuda-repo-l4t-10-2-local-10.2.89_1.0-1_arm64.deb"

wget -O "${CUDA_REPO_NAME}" "${LOCAL_HTTP_ADDR}/${CUDA_REPO_NAME}" >/dev/null
dpkg -i "${CUDA_REPO_NAME}" && rm -rf "${CUDA_REPO_NAME}"
apt-key add "/var/cuda-repo-${VERSION_1}-local-${VERSION_2}/7fa2af80.pub"
apt-get -y update &&    \
    apt-get -y install --no-install-recommends \
    cuda-cudart-${VERSION_1}            \
    cuda-license-${VERSION_1}           \
    cuda-nvml-dev-${VERSION_1}          \
    cuda-command-line-tools-${VERSION_1} \
    cuda-libraries-${VERSION_1} \
    cuda-libraries-dev-${VERSION_1} \
    cuda-minimal-build-${VERSION_1} \
    cuda-cupti-${VERSION_1} \
    cuda-cupti-dev-${VERSION_1} \
    libcublas10 \
    libcublas-dev \
    cuda-npp-${VERSION_1} \
    cuda-npp-dev-${VERSION_1} \
    && ln -s "cuda-${VERSION_2%.*}" /usr/local/cuda

# Note(storypku): The last two are requred by third_party/npp

CUDA_VER="${VERSION_2%.*}"
echo "Successfully installed CUDA  ${CUDA_VER}"

##===== Install CUDNN 8 =====##

CUDNN_VER1="8.0.0.180-1"
MAJOR="${CUDNN_VER1%%.*}"
CUDNN_VERSION="${CUDNN_VER1}+cuda${CUDA_VER}"

CUDNN_PKGS="\
libcudnn${MAJOR}_${CUDNN_VERSION}_arm64.deb \
libcudnn${MAJOR}-dev_${CUDNN_VERSION}_arm64.deb \
"

for pkg in ${CUDNN_PKGS}; do
    echo "Downloading ${LOCAL_HTTP_ADDR}/${pkg}"
    wget "${LOCAL_HTTP_ADDR}/${pkg}" >/dev/null
done

for pkg in ${CUDNN_PKGS}; do
    dpkg -i "${pkg}"
    rm -rf ${pkg}
done

echo "Stripping off libcudnn8 static libraries..."
find /usr/lib/$(uname -m)-linux-gnu -name "libcudnn_*.a" -delete -print

echo "Successfully installed CUDNN ${MAJOR}"

# Kick the ladder and cleanup
apt-get purge --autoremove -y \
    gnupg2 \
    ca-certificates \
    "cuda-repo-l4t-${VERSION_1}-local-${VERSION_2}"

apt-get clean && \
    rm -rf /var/lib/apt/lists/*
